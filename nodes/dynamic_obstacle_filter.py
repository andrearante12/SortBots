#!/usr/bin/env python3
"""Split head depth into static (for SLAM) and dynamic (for Nav2).

RTAB-Map permanently paints whatever depth it sees into /{robot}/map — including
other robots. This node invalidates mover pixels in a `depth_static` Image
(NaN) for SLAM, and republishes those points as `/<id>/dynamic_obstacles`
PointCloud2 for an ephemeral Nav2 ObstacleLayer.

Detection v1: frame-to-frame depth change in the robot-height band, then
connected-component blobs of roughly robot footprint size. No peer TF
oracle — vision only (optional radio association is future work).

    python3 nodes/dynamic_obstacle_filter.py --robot-id robot_0
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent

# Defaults tuned for XLeRobot-ish body in the NVIDIA warehouse aisle.
DEFAULTS = {
    "change_m": 0.12,          # |z - z_prev| to count as motion
    "min_height_m": 0.08,      # keep near-floor noise out of mover mask
    "max_height_m": 1.35,      # below shelf tops; covers peer chassis+mast
    "min_range_m": 0.35,
    "max_range_m": 4.5,
    "min_blob_px": 40,         # reject speckles
    "max_blob_px": 25000,      # reject whole-wall flashes
    "morph_close": 2,          # dilate/erode iterations to knit blob
}


def depth_motion_mask(
    depth: np.ndarray,
    prev: np.ndarray | None,
    *,
    change_m: float = DEFAULTS["change_m"],
    min_height_m: float = DEFAULTS["min_height_m"],
    max_height_m: float = DEFAULTS["max_height_m"],
    min_range_m: float = DEFAULTS["min_range_m"],
    max_range_m: float = DEFAULTS["max_range_m"],
) -> np.ndarray:
    """Boolean mask of candidate dynamic pixels (optical frame z = depth)."""
    d = depth.astype(np.float32, copy=False)
    valid = np.isfinite(d) & (d >= min_range_m) & (d <= max_range_m)
    # Without camera tilt model, treat depth as approximate range and gate
    # by range band that corresponds to body-scale returns (sim D435 looking
    # forward). Height gate approximated by excluding very near floor clutter
    # via min_range and very far structure via max_range; optional prev frame
    # is the real dynamic signal.
    if prev is None or prev.shape != d.shape:
        return np.zeros(d.shape, dtype=bool)
    p = prev.astype(np.float32, copy=False)
    prev_valid = np.isfinite(p) & (p >= min_range_m) & (p <= max_range_m)
    changed = valid & prev_valid & (np.abs(d - p) >= change_m)
    # Drop slow map growth (large distant surfaces jittering): prefer mid-field.
    mid = (d >= min_range_m) & (d <= max_range_m)
    # Soft height proxy: close returns that jumped are more body-like.
    bodyish = (d >= min_height_m) | (p >= min_height_m)
    bodyish &= (d <= max_range_m)
    _ = max_height_m  # reserved for RGB-D projection height once available
    return changed & mid & bodyish


def _morph_close(mask: np.ndarray, iterations: int) -> np.ndarray:
    if iterations <= 0:
        return mask
    out = mask.copy()
    h, w = out.shape
    for _ in range(iterations):
        # Dilate
        dil = out.copy()
        dil[1:, :] |= out[:-1, :]
        dil[:-1, :] |= out[1:, :]
        dil[:, 1:] |= out[:, :-1]
        dil[:, :-1] |= out[:, 1:]
        out = dil
    for _ in range(iterations):
        # Erode
        er = out.copy()
        er[1:, :] &= out[:-1, :]
        er[:-1, :] &= out[1:, :]
        er[:, 1:] &= out[:, :-1]
        er[:, :-1] &= out[:, 1:]
        # borders stay
        er[0, :] = out[0, :]
        er[-1, :] = out[-1, :]
        er[:, 0] = out[:, 0]
        er[:, -1] = out[:, -1]
        out = er
        _ = h, w
    return out


def label_blobs(mask: np.ndarray) -> tuple[np.ndarray, list[int]]:
    """4-connected component labels; returns (labels HxW int32, list of sizes).
    Label 0 is background.

    Row-run + union-find, NOT a per-pixel flood fill. The obvious per-pixel
    version is O(h*w) in interpreted Python, which at the D435's 848x480
    (configs/sensors/d435.json) is 407k iterations plus a DFS per blob, every
    frame, against a 30 Hz camera. Diagnosed live 2026-08-09: it throttled
    /<id>/camera/depth_static to ~0.3 Hz, and because that topic is the ONLY
    depth source for RTAB-Map, both costmaps' obstacle_layer and
    collision_monitor, the knock-on was severe — depth_image_proc's
    PointCloudXyzNode never synced a pair (its camera_info queue is 5 deep and
    had long discarded the matching stamp), so depth_static/points went nearly
    silent and the costmaps went blind.

    Here the interpreted loop runs once per RUN of set pixels rather than once
    per pixel, so cost tracks blob perimeter complexity (tens to hundreds of
    runs) instead of resolution. Labels are renumbered in row-major order of
    first appearance, so ids match what the per-pixel scan produced.
    """
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    if not mask.any():
        return labels, [0]

    # A run starts where the column-padded diff is +1 and ends (exclusive)
    # where it is -1. np.nonzero returns row-major order and each row has as
    # many starts as ends, so the two results pair up elementwise.
    padded = np.zeros((h, w + 2), dtype=np.int8)
    padded[:, 1:-1] = mask
    steps = np.diff(padded, axis=1)
    run_y, run_start = np.nonzero(steps == 1)
    _, run_end = np.nonzero(steps == -1)
    n_runs = run_y.size

    parent = list(range(n_runs))

    def find(i: int) -> int:
        root = i
        while parent[root] != root:
            root = parent[root]
        while parent[i] != root:
            parent[i], i = root, parent[i]
        return root

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            # Keep the LOWER index as root so a component's root is its
            # first-appearing run — that's what makes the renumbering below
            # come out in scan order.
            parent[max(ra, rb)] = min(ra, rb)

    # Merge-walk each adjacent row pair: both run sequences are sorted by
    # column, so advancing whichever run ends first visits each one once.
    rows = np.arange(h)
    row_begin = np.searchsorted(run_y, rows, side="left")
    row_end = np.searchsorted(run_y, rows, side="right")
    starts = run_start.tolist()
    ends = run_end.tolist()
    for y in range(1, h):
        i, i_stop = int(row_begin[y - 1]), int(row_end[y - 1])
        j, j_stop = int(row_begin[y]), int(row_end[y])
        while i < i_stop and j < j_stop:
            if starts[i] < ends[j] and starts[j] < ends[i]:
                union(i, j)
            if ends[i] < ends[j]:
                i += 1
            else:
                j += 1

    remap: dict[int, int] = {}
    run_labels = np.zeros(n_runs, dtype=np.int32)
    for i in range(n_runs):
        root = find(i)
        lab = remap.get(root)
        if lab is None:
            lab = len(remap) + 1
            remap[root] = lab
        run_labels[i] = lab

    ys = run_y.tolist()
    label_ids = run_labels.tolist()
    for i in range(n_runs):
        labels[ys[i], starts[i]:ends[i]] = label_ids[i]

    # Sum run lengths per label instead of counting pixels over the full
    # image — same answer, and it stays proportional to the run count.
    lengths = (run_end - run_start).astype(np.int64)
    sizes = np.bincount(run_labels, weights=lengths, minlength=len(remap) + 1)
    return labels, [int(s) for s in sizes]


def select_dynamic_mask(
    depth: np.ndarray,
    prev: np.ndarray | None,
    cfg: dict | None = None,
) -> np.ndarray:
    """Full v1 pipeline → boolean mask of pixels to treat as dynamic."""
    c = {**DEFAULTS, **(cfg or {})}
    raw = depth_motion_mask(
        depth,
        prev,
        change_m=c["change_m"],
        min_height_m=c["min_height_m"],
        max_height_m=c["max_height_m"],
        min_range_m=c["min_range_m"],
        max_range_m=c["max_range_m"],
    )
    closed = _morph_close(raw, int(c["morph_close"]))
    labels, sizes = label_blobs(closed)
    lo, hi = int(c["min_blob_px"]), int(c["max_blob_px"])
    # One lookup-table index rather than a `keep |= labels == lab` pass per
    # label: that loop walked the whole 848x480 image once per blob, so a
    # speckly frame (which is the common case, and the reason min_blob_px
    # exists) cost hundreds of full-image comparisons.
    size_ok = (np.asarray(sizes) >= lo) & (np.asarray(sizes) <= hi)
    size_ok[0] = False  # label 0 is background
    return size_ok[labels]


def apply_static_depth(depth: np.ndarray, dynamic_mask: np.ndarray) -> np.ndarray:
    """Copy of depth with dynamic pixels set to NaN."""
    out = depth.astype(np.float32, copy=True)
    out[dynamic_mask] = np.nan
    return out


def depth_to_points(
    depth: np.ndarray,
    mask: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    *,
    stride: int = 2,
) -> np.ndarray:
    """Nx3 points in optical frame (x right, y down, z forward) for masked pixels."""
    ys, xs = np.where(mask)
    if ys.size == 0:
        return np.zeros((0, 3), dtype=np.float32)
    if stride > 1:
        ys = ys[::stride]
        xs = xs[::stride]
    z = depth[ys, xs].astype(np.float32)
    ok = np.isfinite(z) & (z > 0)
    ys, xs, z = ys[ok], xs[ok], z[ok]
    x = (xs.astype(np.float32) - cx) * z / fx
    y = (ys.astype(np.float32) - cy) * z / fy
    return np.stack([x, y, z], axis=1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_0")
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
    from std_msgs.msg import Header

    robot_id = args.robot_id

    def pack_cloud(header: Header, pts: np.ndarray) -> PointCloud2:
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(pts.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = pts.astype(np.float32).tobytes()
        return msg

    class DynamicObstacleFilter(Node):
        def __init__(self):
            super().__init__(f"{robot_id}_dynamic_obstacle_filter")
            self._prev = None
            self._info = None
            self._cfg = dict(DEFAULTS)
            qos = qos_profile_sensor_data
            self.create_subscription(
                Image, f"/{robot_id}/camera/depth", self._on_depth, qos
            )
            self.create_subscription(
                CameraInfo, f"/{robot_id}/camera/camera_info", self._on_info, qos
            )
            self.static_pub = self.create_publisher(
                Image, f"/{robot_id}/camera/depth_static", qos
            )
            self.dyn_pub = self.create_publisher(
                PointCloud2, f"/{robot_id}/dynamic_obstacles", qos
            )
            self.get_logger().info(
                f"dynamic filter: depth -> depth_static + dynamic_obstacles "
                f"for {robot_id}"
            )

        def _on_info(self, msg: CameraInfo):
            self._info = msg

        def _on_depth(self, msg: Image):
            if msg.encoding not in ("32FC1", "mono16", "16UC1"):
                return
            h, w = msg.height, msg.width
            if msg.encoding == "32FC1":
                depth = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
            else:
                raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
                depth = raw.astype(np.float32) * 0.001  # typical mm→m; sim is 32FC1

            mask = select_dynamic_mask(depth, self._prev, self._cfg)
            self._prev = depth.copy()
            static = apply_static_depth(depth, mask)

            out = Image()
            out.header = msg.header
            out.height = h
            out.width = w
            out.encoding = "32FC1"
            out.is_bigendian = msg.is_bigendian
            out.step = w * 4
            out.data = static.astype(np.float32).tobytes()
            self.static_pub.publish(out)

            if self._info is None or not mask.any():
                # Still publish empty cloud so costmap can clear.
                empty = PointCloud2()
                empty.header = msg.header
                empty.height = 1
                empty.width = 0
                empty.fields = [
                    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                ]
                empty.is_bigendian = False
                empty.point_step = 12
                empty.row_step = 0
                empty.is_dense = True
                empty.data = b""
                self.dyn_pub.publish(empty)
                return

            K = self._info.k
            fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
            pts = depth_to_points(depth, mask, fx, fy, cx, cy)
            self.dyn_pub.publish(pack_cloud(msg.header, pts))

    rclpy.init(args=argv)
    node = DynamicObstacleFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
