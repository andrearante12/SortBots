#!/usr/bin/env python3
"""Bounded, dashboard-facing republisher of RTAB-Map's assembled cloud.

With `--Grid/3D true` (see launch/sortbots_rtabmap_robot.launch.py's GRID_ARGS)
`cloud_map` stops being a flat 2D pancake and becomes a real 3D surface cloud —
which also means it grows without bound as the robot explores. That is a
problem specifically for the browser:

  * rosbridge base64-encodes the byte payload (+33%), and the vendored roslib
    in webui/vendor has NO fragment reassembly, so anything over rosbridge's
    max_message_size (20 MB, launch/sortbots_webui.launch.py) doesn't degrade
    — it breaks the topic outright.
  * Well before that, a multi-MB JSON.parse + atob on the browser's main
    thread every few seconds janks the whole dashboard while still nominally
    "working".

Raising the cap doesn't fix either one, because the payload is unbounded by
construction. So this node applies a HARD point budget instead, and keeps the
giant message on a local DDS hop instead of across a websocket.

Two things it does:

  1. Voxel-downsamples to --max-points, growing the leaf size until the cloud
     fits. The chosen leaf is logged rather than applied silently, so a
     degraded panel is diagnosable from the console.
  2. Repacks point_step 32 -> 16. rtabmap emits pcl::PointXYZRGB, which pads
     xyz (offsets 0/4/8) and rgb (offset 16) out to 32 bytes — half the wire
     bytes are padding. This is safe because every consumer downstream is
     offset-driven: webui/app.js reads msg.point_step and builds its own
     offset table from msg.fields, and scripts/bag_to_fixture.py documents
     itself as agnostic to the field layout.

Unlike nodes/rtabmap_cloud_pump.py this must NOT wait for any service — a
plain subscription tolerates rtabmap starting late, or restarting under it.
The two are deliberately separate processes: the pump is timer-driven and
blocks on a service at startup, this is callback-driven and blocks on nothing.

Run (system ROS 2 Jazzy sourced, NOT the Isaac venv or conda):

    source /opt/ros/jazzy/setup.bash
    python3 nodes/recon_cloud_relay.py --robot-id robot_0
"""
from __future__ import annotations

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

# rtabmap publishes the assembled clouds latched (MapsManager's `latch` defaults
# true), and the pump only forces a republish every 3s. Matching transient-local
# here means a freshly-loaded dashboard gets the current cloud immediately
# instead of staring at "waiting for cloud..." until the next pump tick. Same
# reasoning as nodes/explorer.py's MAP_QOS.
CLOUD_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

DEFAULT_MAX_POINTS = 200_000
DEFAULT_VOXEL_SIZE = 0.05  # matches Grid/CellSize; a near no-op on first pass
# Republish the last cloud this often when nothing new has arrived.
#
# Publishing only on change is not enough. cloud_map stops changing whenever
# the map does — SLAM stalled, exploration finished, or --localize, where it
# never changes at all — and rosbridge subscribes with VOLATILE durability, so
# it does NOT receive our latched transient-local sample. A dashboard opened
# during any of those windows would sit on "waiting for cloud..." forever,
# while a dashboard that happened to be open earlier looks fine. Transient-local
# only helps subscribers that ask for it, and the browser's roslib can't.
DEFAULT_KEEPALIVE_SEC = 5.0
MIN_LEAF_GROWTH = 1.1      # floor on the per-retry multiplier, to guarantee progress
MAX_LEAF_RETRIES = 8
NO_COLOR_FILL = 0xB0B0B0   # neutral gray when the source cloud has no rgb

OUT_POINT_STEP = 16        # x@0, y@4, z@8, rgb@12 — no padding


class ReconCloudRelay(Node):
    def __init__(self, robot_id: str, max_points: int, voxel_size: float,
                 keepalive_sec: float):
        super().__init__("recon_cloud_relay")
        self.max_points = max_points
        self.voxel_size = voxel_size
        self.last_cloud = None      # last built message, for the keepalive
        self.published_since_tick = False
        self.pub = self.create_publisher(
            PointCloud2, f"/{robot_id}/recon_cloud", CLOUD_QOS
        )
        self.create_subscription(
            PointCloud2, f"/{robot_id}/cloud_map", self._on_cloud, CLOUD_QOS
        )
        if keepalive_sec > 0:
            self.create_timer(keepalive_sec, self._keepalive)
        self.get_logger().info(
            f"relaying /{robot_id}/cloud_map -> /{robot_id}/recon_cloud "
            f"(budget {max_points} pts, leaf {voxel_size} m, "
            f"keepalive {keepalive_sec}s)"
        )

    def _keepalive(self):
        """Re-send the last cloud if nothing new arrived since the last tick."""
        if self.published_since_tick:
            self.published_since_tick = False
            return
        if self.last_cloud is not None:
            self.pub.publish(self.last_cloud)

    # -- decode ------------------------------------------------------------
    @staticmethod
    def _unpack(msg: PointCloud2):
        """Return (xyz float32 (n,3), rgb bytes (n,4) or None), NaNs dropped."""
        offsets = {f.name: f.offset for f in msg.fields}
        for axis in ("x", "y", "z"):
            if axis not in offsets:
                raise KeyError(f"cloud has no '{axis}' field")

        # Slice rows out by row_step so this also handles organized clouds
        # whose rows are padded (cloud_map is unordered, but don't rely on it).
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        rows = raw[: msg.height * msg.row_step].reshape(msg.height, msg.row_step)
        rows = rows[:, : msg.width * msg.point_step]
        pts = rows.reshape(-1, msg.point_step)

        def f32(offset):
            # .copy() first: frombuffer is read-only and .view() on a strided
            # slice of it raises.
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

    # -- downsample --------------------------------------------------------
    def _downsample(self, xyz: np.ndarray):
        """Voxel-dedupe indices under the point budget. Returns (idx, leaf)."""
        n = len(xyz)
        if n <= self.max_points:
            return None, 0.0  # None = keep everything

        lo = xyz.min(axis=0)
        span = xyz.max(axis=0) - lo
        leaf = self.voxel_size
        for _ in range(MAX_LEAF_RETRIES):
            keys = ((xyz - lo) / leaf).astype(np.int64)
            # Pack 3 axis indices into one int64 so np.unique collapses voxels
            # in a single pass. dims are the per-axis cell counts, so the
            # product stays far under 2**63 for any warehouse-sized map.
            dims = np.maximum(keys.max(axis=0) + 1, 1)
            packed = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
            idx = np.unique(packed, return_index=True)[1]
            if len(idx) <= self.max_points:
                return np.sort(idx), leaf
            # These clouds are surfaces — 2D manifolds embedded in 3D — so the
            # voxel count falls off as leaf**-2, not leaf**-3. Estimating the
            # next leaf from that lands within a few percent of the budget in
            # one or two passes; a fixed multiplier instead overshoots and
            # throws away resolution we were entitled to keep.
            leaf *= max(MIN_LEAF_GROWTH, (len(idx) / self.max_points) ** 0.5)

        # Voxelization plateaued (e.g. a genuinely dense flat surface).
        # Fall back to stride decimation so the budget is never exceeded.
        self.get_logger().warn(
            f"voxel dedupe stalled at leaf {leaf:.3f} m ({len(idx)} pts) — "
            f"falling back to stride decimation, span {span.round(1)}"
        )
        return np.sort(idx)[:: int(np.ceil(len(idx) / self.max_points))], leaf

    # -- encode ------------------------------------------------------------
    @staticmethod
    def _build(msg: PointCloud2, xyz: np.ndarray, rgb) -> PointCloud2:
        n = len(xyz)
        out = np.empty((n, OUT_POINT_STEP), dtype=np.uint8)
        out[:, 0:12] = np.ascontiguousarray(xyz, dtype="<f4").view(np.uint8).reshape(n, 12)
        if rgb is None:
            out[:, 12] = NO_COLOR_FILL & 0xFF
            out[:, 13] = (NO_COLOR_FILL >> 8) & 0xFF
            out[:, 14] = (NO_COLOR_FILL >> 16) & 0xFF
            out[:, 15] = 0
        else:
            out[:, 12:16] = rgb

        cloud = PointCloud2()
        cloud.header = msg.header
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

    # -- callback ----------------------------------------------------------
    def _on_cloud(self, msg: PointCloud2):
        if msg.is_bigendian:
            self.get_logger().warn("big-endian cloud, not supported — dropping")
            return
        try:
            xyz, rgb = self._unpack(msg)
        except (KeyError, ValueError) as exc:
            self.get_logger().warn(f"could not decode cloud: {exc}")
            return
        if len(xyz) == 0:
            return

        total = len(xyz)
        idx, leaf = self._downsample(xyz)
        if idx is not None:
            xyz = xyz[idx]
            if rgb is not None:
                rgb = rgb[idx]

        cloud = self._build(msg, xyz, rgb)
        self.pub.publish(cloud)
        self.last_cloud = cloud
        self.published_since_tick = True

        kept = len(xyz)
        note = f", leaf {leaf:.3f} m" if idx is not None else ""
        self.get_logger().info(
            f"{kept} / {total} pts, {len(cloud.data) / 1e6:.2f} MB "
            f"(~{len(cloud.data) * 4 / 3 / 1e6:.2f} MB base64){note}"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument(
        "--max-points",
        type=int,
        default=DEFAULT_MAX_POINTS,
        help="hard cap on republished points (default: %(default)s)",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=DEFAULT_VOXEL_SIZE,
        help="starting voxel leaf in metres (default: %(default)s)",
    )
    parser.add_argument(
        "--keepalive-sec",
        type=float,
        default=DEFAULT_KEEPALIVE_SEC,
        help="republish the last cloud this often when idle, 0 to disable "
             "(default: %(default)s)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = ReconCloudRelay(args.robot_id, args.max_points, args.voxel_size,
                           args.keepalive_sec)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
