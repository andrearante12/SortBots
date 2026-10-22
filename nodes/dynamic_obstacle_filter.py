#!/usr/bin/env python3
"""Split head depth into static (for SLAM) and dynamic (for Nav2).

RTAB-Map permanently paints whatever depth it sees into /{robot}/map — including
other robots. This node invalidates mover pixels in a `depth_static` Image
(NaN) for SLAM, and republishes those points as `/<id>/dynamic_obstacles`
PointCloud2 for an ephemeral Nav2 ObstacleLayer.

Detection v1: frame-to-frame depth change in the robot-height band, then
connected-component blobs of roughly robot footprint size. No peer TF
oracle.

Plus peers by radio: frame differencing misses a robot that stands still or
creeps, so RTAB-Map painted it into the shared map as a wall. Live on
2026-10-22, 76 occupied /map cells sat under robot_0 where robot_1 had seen it.
Robot_0's global costmap went lethal around its own pose, and NavFn's way out of
the ring was away from the goal: the "turns away and goes the long way" detour.
So every pixel whose point lands within `peer_radius_m` (in x/y) of a peer's
self-reported /fleet/status pose is masked too. That is the radio channel the
fleet already uses instead of peer TF, and only this robot's own TF places the
camera in `map`.

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
    # Footprint corner reach is 0.32 m (0.5 x 0.4 m); the rest is margin for
    # the 5 Hz status lag and the two robots' localization disagreeing.
    "peer_radius_m": 0.5,
    "peer_ttl_s": 1.0,         # fleet_radio's status_ttl_s
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


def peer_mask(
    depth: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
    R_map_cam: np.ndarray, t_map_cam: np.ndarray,
    peers_xy: list[tuple[float, float]],
    radius_m: float = DEFAULTS["peer_radius_m"],
) -> np.ndarray:
    """Pixels whose point lies within radius_m (x/y, any height) of a peer.

    `R_map_cam`/`t_map_cam` place the image's optical frame in `map`. Any
    height on purpose: a peer's whole body is masked, and so is the floor right
    around it, which only costs SLAM a little free space it will see again.
    """
    h, w = depth.shape
    mask = np.zeros((h, w), dtype=bool)
    r2 = radius_m * radius_m
    for px, py in peers_xy:
        band = _peer_columns(px, py, radius_m, fx, cx, w, R_map_cam, t_map_cam)
        if band is None:
            continue
        u0, u1 = band
        # Every pixel of the full image is ~23 ms a frame at 848x480; a peer
        # covers a band of columns, so only that band is back-projected.
        z = depth[:, u0:u1].astype(np.float32, copy=False)
        valid = np.isfinite(z) & (z > 0)
        if not valid.any():
            continue
        vs, us = np.nonzero(valid)
        zz = z[vs, us]
        us = us + u0
        x = (us.astype(np.float32) - cx) * zz / fx
        y = (vs.astype(np.float32) - cy) * zz / fy
        # Only x and y in map are needed: two rows of the rotation.
        mx = R_map_cam[0, 0] * x + R_map_cam[0, 1] * y + R_map_cam[0, 2] * zz + t_map_cam[0]
        my = R_map_cam[1, 0] * x + R_map_cam[1, 1] * y + R_map_cam[1, 2] * zz + t_map_cam[1]
        hit = (mx - px) ** 2 + (my - py) ** 2 <= r2
        mask[vs[hit], us[hit]] = True
    return mask


def _peer_columns(px: float, py: float, radius_m: float, fx: float, cx: float,
                  w: int, R_map_cam: np.ndarray, t_map_cam: np.ndarray,
                  z_span: tuple[float, float] = (-0.5, 2.5)) -> tuple[int, int] | None:
    """Image columns [u0, u1) the peer's bounding box can project into, or None.

    The box is the peer's x/y square, from below the floor to above any robot.
    A corner behind the camera means the box straddles it: use every column.
    """
    corners = np.array([[px + dx, py + dy, z] for dx in (-radius_m, radius_m)
                        for dy in (-radius_m, radius_m) for z in z_span], dtype=np.float32)
    cam = (corners - t_map_cam) @ R_map_cam  # R^T (p - t), row-wise
    if (cam[:, 2] <= 0.05).all():
        return None  # wholly behind the camera
    if (cam[:, 2] <= 0.05).any():
        return 0, w
    u = fx * cam[:, 0] / cam[:, 2] + cx
    u0, u1 = int(np.floor(u.min())), int(np.ceil(u.max())) + 1
    if u1 <= 0 or u0 >= w:
        return None
    return max(u0, 0), min(u1, w)


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = (x * x + y * y + z * z + w * w) ** 0.5 or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)


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

    import time

    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
    from std_msgs.msg import Header, String
    from tf2_ros import Buffer, TransformListener

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from fleet_radio import STATUS_TOPIC, parse_status

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
            self._peers: dict[str, tuple[tuple[float, float], float]] = {}
            self._tf = Buffer()
            self._tf_listener = TransformListener(self._tf, self)
            qos = qos_profile_sensor_data
            # fleet_radio publishes BEST_EFFORT (a lossy mesh stand-in); a
            # RELIABLE subscriber is incompatible and silently gets nothing.
            self.create_subscription(String, STATUS_TOPIC, self._on_status, qos)
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

        def _on_status(self, msg: String):
            st = parse_status(msg.data)
            if st and st["robot_id"] != robot_id:
                self._peers[st["robot_id"]] = ((st["x"], st["y"]), time.monotonic())

        def _peer_mask(self, msg: Image, depth: np.ndarray):
            now = time.monotonic()
            peers = [xy for xy, at in self._peers.values()
                     if now - at <= self._cfg["peer_ttl_s"]]
            if not peers or self._info is None:
                return None
            # This robot's OWN camera in map, never a peer's TF. At the image
            # stamp first, else the newest.
            tf = None
            for when in (Time.from_msg(msg.header.stamp), Time()):
                try:
                    tf = self._tf.lookup_transform("map", msg.header.frame_id, when,
                                                   timeout=Duration(seconds=0.0))
                    break
                except Exception:  # noqa: BLE001 - tf2 raises several types
                    continue
            if tf is None:
                return None
            q, t = tf.transform.rotation, tf.transform.translation
            K = self._info.k
            return peer_mask(depth, float(K[0]), float(K[4]), float(K[2]), float(K[5]),
                             _quat_to_matrix(q.x, q.y, q.z, q.w),
                             np.array([t.x, t.y, t.z], dtype=np.float32),
                             peers, self._cfg["peer_radius_m"])

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
            peers = self._peer_mask(msg, depth)
            if peers is not None:
                mask |= peers
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
