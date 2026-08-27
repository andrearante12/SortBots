#!/usr/bin/env python3
"""Fuse every robot's recon_cloud into one world-anchored `/fleet/recon_cloud`.

Collaborative *fusion*, not collaborative SLAM: each robot still owns its
RTAB-Map DB and `/{id}/recon_cloud` in `{id}/map`. This node looks up the
same `map -> {id}/map` spawn anchors map_merge uses, transforms points into
`map`, concatenates, and re-budgets so rosbridge/the dashboard stay healthy.

The dashboard 3D panel should subscribe here (not per-robot recon_cloud) so
peer markers and the cloud share one frame.

    python3 nodes/recon_cloud_merge.py --robot-ids robot_0,robot_1
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
import yaml
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parent))
from recon_fuse import fuse_clouds  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "configs" / "recon_merge.yaml"

CLOUD_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

OUT_POINT_STEP = 16
DEFAULTS = {
    "publish_period_s": 2.0,
    "max_points": 200_000,
    "voxel_size": 0.05,
    "keepalive_sec": 5.0,
}


def load_config(path: Path | None) -> dict:
    cfg = dict(DEFAULTS)
    if path and path.exists():
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
        if isinstance(raw, dict):
            cfg.update(raw)
    return cfg


def unpack_cloud(msg: PointCloud2):
    offsets = {f.name: f.offset for f in msg.fields}
    for axis in ("x", "y", "z"):
        if axis not in offsets:
            raise KeyError(f"cloud has no '{axis}' field")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    rows = raw[: msg.height * msg.row_step].reshape(msg.height, msg.row_step)
    rows = rows[:, : msg.width * msg.point_step]
    pts = rows.reshape(-1, msg.point_step)

    def f32(offset):
        return pts[:, offset : offset + 4].copy().view("<f4").ravel()

    xyz = np.stack([f32(offsets[a]) for a in ("x", "y", "z")], axis=1)
    rgb = (
        pts[:, offsets["rgb"] : offsets["rgb"] + 4].copy()
        if "rgb" in offsets
        else None
    )
    keep = np.isfinite(xyz).all(axis=1)
    if not keep.all():
        xyz = xyz[keep]
        if rgb is not None:
            rgb = rgb[keep]
    return xyz, rgb


def pack_cloud(header: Header, xyz: np.ndarray, rgb: np.ndarray) -> PointCloud2:
    n = len(xyz)
    out = np.empty((n, OUT_POINT_STEP), dtype=np.uint8)
    out[:, 0:12] = np.ascontiguousarray(xyz, dtype="<f4").view(np.uint8).reshape(n, 12)
    out[:, 12:16] = rgb
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = n
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = OUT_POINT_STEP
    cloud.row_step = OUT_POINT_STEP * n
    cloud.data = out.tobytes()
    cloud.is_dense = True
    return cloud


class ReconCloudMergeNode(Node):
    def __init__(self, robot_ids: list[str], cfg: dict):
        super().__init__("recon_cloud_merge")
        self.robot_ids = robot_ids
        self.cfg = cfg
        self._clouds: dict[str, tuple[np.ndarray, np.ndarray | None]] = {}
        self._last_out: PointCloud2 | None = None
        self._published_since_tick = False
        self._prev_n = -1
        self._prev_contributors: tuple[str, ...] = ()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, "/fleet/recon_cloud", CLOUD_QOS)

        for rid in robot_ids:
            self.create_subscription(
                PointCloud2,
                f"/{rid}/recon_cloud",
                self._make_on_cloud(rid),
                CLOUD_QOS,
            )

        self.create_timer(float(cfg["publish_period_s"]), self._tick)
        keep = float(cfg.get("keepalive_sec", 0))
        if keep > 0:
            self.create_timer(keep, self._keepalive)
        self.get_logger().info(
            f"recon_cloud_merge: {robot_ids} -> /fleet/recon_cloud "
            f"(budget {cfg['max_points']} pts)"
        )

    def _make_on_cloud(self, rid: str):
        def _cb(msg: PointCloud2):
            if msg.is_bigendian:
                return
            try:
                xyz, rgb = unpack_cloud(msg)
            except (KeyError, ValueError) as exc:
                self.get_logger().warn(f"{rid}: decode failed: {exc}")
                return
            self._clouds[rid] = (xyz, rgb)

        return _cb

    def _lookup_anchor(self, rid: str):
        try:
            tr = self.tf_buffer.lookup_transform(
                "map", f"{rid}/map", rclpy.time.Time()
            )
        except Exception:
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        return (t.x, t.y, t.z, q.x, q.y, q.z, q.w)

    def _tick(self):
        sources = []
        contributors = []
        for rid in self.robot_ids:
            cloud = self._clouds.get(rid)
            if cloud is None:
                continue
            xf = self._lookup_anchor(rid)
            if xf is None:
                continue
            xyz, rgb = cloud
            sources.append((xyz, rgb, xf))
            contributors.append(rid)

        if not sources:
            return

        xyz, rgb, leaf = fuse_clouds(
            sources,
            max_points=int(self.cfg["max_points"]),
            voxel_size=float(self.cfg["voxel_size"]),
        )
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        out = pack_cloud(header, xyz, rgb)
        self.pub.publish(out)
        self._last_out = out
        self._published_since_tick = True

        key = (len(xyz), tuple(contributors))
        if key != (self._prev_n, self._prev_contributors):
            self._prev_n, self._prev_contributors = key
            note = f", leaf {leaf:.3f} m" if leaf else ""
            self.get_logger().info(
                f"fused {len(contributors)} robot(s) {list(contributors)} -> "
                f"{len(xyz)} pts in map{note}"
            )

    def _keepalive(self):
        if self._published_since_tick:
            self._published_since_tick = False
            return
        if self._last_out is not None:
            self.pub.publish(self._last_out)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-ids",
        required=True,
        help="Comma-separated robot ids, e.g. robot_0,robot_1",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args(remove_ros_args(args=argv or sys.argv[1:]))
    robot_ids = [r.strip() for r in args.robot_ids.split(",") if r.strip()]
    if not robot_ids:
        parser.error("--robot-ids must list at least one id")

    cfg = load_config(args.config)
    rclpy.init(args=argv)
    node = ReconCloudMergeNode(robot_ids, cfg)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
