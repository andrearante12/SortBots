#!/usr/bin/env python3
"""Split head depth into static (for SLAM) and dynamic (for Nav2).

RTAB-Map permanently paints whatever depth it sees into /{robot}/map — including
other robots. This node invalidates mover pixels in a `depth_static` Image
(NaN) for SLAM, and republishes those points as `/<id>/dynamic_obstacles`
PointCloud2 for an ephemeral Nav2 ObstacleLayer.

Detection v2: each current pixel is back-projected, moved into the PREVIOUS
camera frame by odometry, and compared with what the previous frame saw along
that ray. Only "something is now closer than the ray used to reach" counts,
then connected-component blobs. No peer TF oracle.

Why the warp and the sign: v1 diffed raw frames pixel by pixel, so the moment
the robot moved, every depth edge shifted in the image and shelf edges were
NaN'd out of depth_static as "movers" (found reading the code 2026-11-20, while
planning person masking). The signed backward test cannot fire on a static
scene from parallax: a static point the previous camera could see has
previous-depth = its depth; one it could not see was hidden by something
CLOSER, which makes the difference negative. The cost is that an object moving
straight away from the camera only shows at its edges. The segment masks
(`dynamic_seg`, nodes/dynamic_segment.py) fill whole objects in from those
edges, and the peer mask covers robots.

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
    "change_m": 0.12,          # prev-ray depth minus warped depth to count as motion
    # Heights are metres above `<id>/odom` (the floor), applied only when the
    # camera's odom pose is known. 2.2 m covers a standing adult's head; v1's
    # 1.35 m cut people off at the shoulders and was never applied anyway.
    "min_height_m": 0.08,      # keep near-floor noise out of mover mask
    "max_height_m": 2.2,
    "min_range_m": 0.35,
    "max_range_m": 4.5,
    # Blob sizes are FULL-resolution pixels; motion runs at 1/stride and the
    # thresholds are scaled to match. Stride 2 keeps the warp + labelling
    # inside a 30 Hz frame budget at 848x480.
    "motion_stride": 2,
    "min_blob_px": 40,         # reject speckles
    "max_blob_px": 25000,      # reject whole-wall flashes
    "morph_close": 2,          # dilate/erode iterations to knit blob
    # Exact-stamp TF wait for the warp. On a miss the frame gets no motion
    # mask rather than an unwarped one: an unwarped diff on a moving robot is
    # exactly v1's shelf-edge bug.
    "tf_timeout_s": 0.03,
    # Footprint corner reach is 0.32 m (0.5 x 0.4 m); the rest is margin for
    # the 5 Hz status lag and the two robots' localization disagreeing.
    "peer_radius_m": 0.5,
    "peer_ttl_s": 1.0,         # fleet_radio's status_ttl_s
}


def _pixel_rays(h: int, w: int, fx: float, fy: float, cx: float, cy: float):
    """Per-pixel (x/z, y/z) of the optical ray, as broadcastable row/column."""
    rx = (np.arange(w, dtype=np.float32) - cx) / fx
    ry = (np.arange(h, dtype=np.float32) - cy) / fy
    return rx[None, :], ry[:, None]


def _min3(img: np.ndarray) -> np.ndarray:
    """3x3 minimum, NaN treated as +inf (an invalid neighbour never wins)."""
    a = np.where(np.isfinite(img), img, np.inf).astype(np.float32)
    p = np.pad(a, 1, mode="constant", constant_values=np.inf)
    h, w = a.shape
    out = a.copy()
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            np.minimum(out, p[dy:dy + h, dx:dx + w], out=out)
    return out


def warp_into_prev(
    depth: np.ndarray,
    prev: np.ndarray,
    K: tuple[float, float, float, float],
    R_prev_cur: np.ndarray,
    t_prev_cur: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Where each current pixel's 3D point lands in the PREVIOUS frame.

    Returns (z_prev, seen), both HxW float32, NaN where there is no answer:
      z_prev  that point's depth in the previous optical frame
      seen    what the previous frame actually measured along the ray to it

    `seen` is the 3x3 minimum around the reprojected pixel, not the nearest
    sample. Nearest-pixel sampling at a depth edge picks the far side half the
    time, which reads as "something came closer" along every edge the robot
    drives past. The minimum makes the test answer yes only when the whole
    neighbourhood was farther.
    """
    fx, fy, cx, cy = K
    h, w = depth.shape
    d = depth.astype(np.float32, copy=False)
    rx, ry = _pixel_rays(h, w, fx, fy, cx, cy)
    x, y = rx * d, ry * d
    R = np.asarray(R_prev_cur, dtype=np.float32)
    t = np.asarray(t_prev_cur, dtype=np.float32)
    X = R[0, 0] * x + R[0, 1] * y + R[0, 2] * d + t[0]
    Y = R[1, 0] * x + R[1, 1] * y + R[1, 2] * d + t[1]
    Z = R[2, 0] * x + R[2, 1] * y + R[2, 2] * d + t[2]
    ok = np.isfinite(d) & (d > 0) & np.isfinite(Z) & (Z > 0.05)
    with np.errstate(invalid="ignore", divide="ignore"):
        u2 = np.rint(fx * X / Z + cx)
        v2 = np.rint(fy * Y / Z + cy)
    ok &= (u2 >= 0) & (u2 < w) & (v2 >= 0) & (v2 < h)
    z_prev = np.full((h, w), np.nan, dtype=np.float32)
    seen = np.full((h, w), np.nan, dtype=np.float32)
    z_prev[ok] = Z[ok]
    pmin = _min3(prev)
    s = pmin[v2[ok].astype(np.intp), u2[ok].astype(np.intp)]
    seen[ok] = np.where(np.isfinite(s), s, np.nan)
    return z_prev, seen


def point_heights(
    depth: np.ndarray,
    K: tuple[float, float, float, float],
    R_odom_cam: np.ndarray,
    t_odom_cam: np.ndarray,
) -> np.ndarray:
    """Height above `<id>/odom` of every pixel's point (NaN where invalid)."""
    fx, fy, cx, cy = K
    h, w = depth.shape
    d = depth.astype(np.float32, copy=False)
    rx, ry = _pixel_rays(h, w, fx, fy, cx, cy)
    R = np.asarray(R_odom_cam, dtype=np.float32)
    z = (R[2, 0] * rx + R[2, 1] * ry + R[2, 2]) * d + float(t_odom_cam[2])
    return np.where(np.isfinite(d) & (d > 0), z, np.nan).astype(np.float32)


def depth_motion_mask(
    depth: np.ndarray,
    prev: np.ndarray | None,
    *,
    change_m: float = DEFAULTS["change_m"],
    min_range_m: float = DEFAULTS["min_range_m"],
    max_range_m: float = DEFAULTS["max_range_m"],
    K: tuple[float, float, float, float] | None = None,
    R_prev_cur: np.ndarray | None = None,
    t_prev_cur: np.ndarray | None = None,
    heights: np.ndarray | None = None,
    min_height_m: float = DEFAULTS["min_height_m"],
    max_height_m: float = DEFAULTS["max_height_m"],
) -> np.ndarray:
    """Boolean mask of pixels that are now closer than the previous frame's ray.

    With `K` and the prev<-cur camera motion the previous frame is warped; with
    no `K` the frames are compared pixel for pixel, which is only right for a
    camera that did not move (the tests, and a robot parked on its odometry).
    `heights` (from point_heights) gates to the body band when known.
    """
    d = depth.astype(np.float32, copy=False)
    if prev is None or prev.shape != d.shape:
        return np.zeros(d.shape, dtype=bool)
    if K is None:
        z_prev = d
        seen = _min3(prev)
        seen = np.where(np.isfinite(seen), seen, np.nan)
    else:
        R = np.eye(3, dtype=np.float32) if R_prev_cur is None else R_prev_cur
        t = np.zeros(3, dtype=np.float32) if t_prev_cur is None else t_prev_cur
        z_prev, seen = warp_into_prev(d, prev, K, R, t)
    valid = np.isfinite(d) & (d >= min_range_m) & (d <= max_range_m)
    valid &= np.isfinite(seen) & (seen >= min_range_m)
    with np.errstate(invalid="ignore"):
        moved = valid & (seen - z_prev >= change_m)
        if heights is not None:
            moved &= (heights >= min_height_m) & (heights <= max_height_m)
    return moved


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


def select_dynamic_blobs(
    depth: np.ndarray,
    prev: np.ndarray | None,
    cfg: dict | None = None,
    *,
    K: tuple[float, float, float, float] | None = None,
    R_prev_cur: np.ndarray | None = None,
    t_prev_cur: np.ndarray | None = None,
    R_odom_cam: np.ndarray | None = None,
    t_odom_cam: np.ndarray | None = None,
) -> tuple[np.ndarray, list[dict]]:
    """Full v2 pipeline → (full-res boolean mask, blob summaries).

    Runs at 1/motion_stride. Blob summaries are in full-resolution pixels and
    are what nodes/dynamic_segment.py prompts SAM with (see blob_summaries).
    """
    c = {**DEFAULTS, **(cfg or {})}
    s = max(1, int(c["motion_stride"]))
    h, w = depth.shape
    d = depth[::s, ::s]
    p = prev[::s, ::s] if prev is not None and prev.shape == depth.shape else None
    Ks = None if K is None else (K[0] / s, K[1] / s, K[2] / s, K[3] / s)
    heights = None
    if Ks is not None and R_odom_cam is not None and t_odom_cam is not None:
        heights = point_heights(d, Ks, R_odom_cam, t_odom_cam)
    raw = depth_motion_mask(
        d, p,
        change_m=c["change_m"],
        min_range_m=c["min_range_m"],
        max_range_m=c["max_range_m"],
        K=Ks, R_prev_cur=R_prev_cur, t_prev_cur=t_prev_cur,
        heights=heights,
        min_height_m=c["min_height_m"],
        max_height_m=c["max_height_m"],
    )
    closed = _morph_close(raw, int(c["morph_close"]))
    labels, sizes = label_blobs(closed)
    area = s * s
    lo, hi = c["min_blob_px"] / area, c["max_blob_px"] / area
    # One lookup-table index rather than a `keep |= labels == lab` pass per
    # label: that loop walked the whole 848x480 image once per blob, so a
    # speckly frame (which is the common case, and the reason min_blob_px
    # exists) cost hundreds of full-image comparisons.
    size_ok = (np.asarray(sizes) >= lo) & (np.asarray(sizes) <= hi)
    size_ok[0] = False  # label 0 is background
    small = size_ok[labels]
    blobs = blob_summaries(np.where(small, labels, 0), d, s)
    if s > 1:
        full = np.repeat(np.repeat(small, s, axis=0), s, axis=1)[:h, :w]
    else:
        full = small
    return full, blobs


def blob_summaries(labels: np.ndarray, depth: np.ndarray, scale: int = 1) -> list[dict]:
    """One dict per nonzero label: full-res bbox [u0, v0, u1, v1) , centroid
    [u, v], pixel count and median depth. Pixel coordinates are multiplied by
    `scale` so a blob found at 1/stride still points at the full image."""
    out = []
    ids = np.unique(labels)
    for lab in ids[ids > 0].tolist():
        vs, us = np.nonzero(labels == lab)
        z = depth[vs, us]
        z = z[np.isfinite(z)]
        out.append({
            "bbox": [int(us.min()) * scale, int(vs.min()) * scale,
                     (int(us.max()) + 1) * scale, (int(vs.max()) + 1) * scale],
            "centroid": [float(us.mean()) * scale, float(vs.mean()) * scale],
            "px": int(us.size) * scale * scale,
            "depth_m": float(np.median(z)) if z.size else None,
        })
    return out


def select_dynamic_mask(
    depth: np.ndarray,
    prev: np.ndarray | None,
    cfg: dict | None = None,
    **pose,
) -> np.ndarray:
    """Boolean mask of pixels to treat as dynamic (select_dynamic_blobs' mask)."""
    return select_dynamic_blobs(depth, prev, cfg, **pose)[0]


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

    import json
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
    odom_frame = f"{robot_id}/odom"

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
            self._prev = None       # (depth, stamp, frame_id) of the last frame
            self._info = None
            self._tf_misses = 0
            self._cfg = dict(DEFAULTS)
            self._peers: dict[str, tuple[tuple[float, float], float]] = {}
            self._tf = Buffer()
            # Own spin thread: the warp waits up to tf_timeout_s for the
            # exact-stamp transform, and a listener fed by this node's own
            # executor could never deliver it while _on_depth is blocking.
            self._tf_listener = TransformListener(self._tf, self, spin_thread=True)
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
            # Motion blobs as SAM prompts for nodes/dynamic_segment.py. JSON on
            # String like the rest of the fleet protocol, never an image.
            self.blob_pub = self.create_publisher(
                String, f"/{robot_id}/camera/motion_blobs", qos
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

        def _lookup(self, target, target_stamp, source, source_stamp):
            """(R, t) taking points from `source` at source_stamp into `target`
            at target_stamp, through the odom frame; None on a miss."""
            try:
                tf = self._tf.lookup_transform_full(
                    target, Time.from_msg(target_stamp), source,
                    Time.from_msg(source_stamp), odom_frame,
                    timeout=Duration(seconds=float(self._cfg["tf_timeout_s"])))
            except Exception:  # noqa: BLE001 - tf2 raises several types
                return None
            q, t = tf.transform.rotation, tf.transform.translation
            return (_quat_to_matrix(q.x, q.y, q.z, q.w),
                    np.array([t.x, t.y, t.z], dtype=np.float32))

        def _motion(self, msg: Image, depth: np.ndarray):
            """(mask, blobs) from the warped previous frame, or no motion when
            either pose is unknown (an unwarped diff is v1's shelf-edge bug)."""
            prev, self._prev = self._prev, (depth.copy(), msg.header.stamp, msg.header.frame_id)
            empty = np.zeros(depth.shape, dtype=bool), []
            if prev is None or self._info is None or prev[2] != msg.header.frame_id:
                return empty
            K = self._info.k
            Kt = (float(K[0]), float(K[4]), float(K[2]), float(K[5]))
            frame = msg.header.frame_id
            rel = self._lookup(frame, prev[1], frame, msg.header.stamp)
            odom = rel and self._lookup(odom_frame, msg.header.stamp, frame,
                                        msg.header.stamp)
            if not rel or not odom:
                self._tf_misses += 1
                if self._tf_misses in (1, 30) or self._tf_misses % 300 == 0:
                    self.get_logger().warning(
                        f"no exact-stamp TF {frame} via {odom_frame} "
                        f"({self._tf_misses} frames without a motion mask)")
                return empty
            return select_dynamic_blobs(depth, prev[0], self._cfg, K=Kt,
                                        R_prev_cur=rel[0], t_prev_cur=rel[1],
                                        R_odom_cam=odom[0], t_odom_cam=odom[1])

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

            mask, blobs = self._motion(msg, depth)
            if blobs:
                st = msg.header.stamp
                self.blob_pub.publish(String(data=json.dumps({
                    "stamp": st.sec + st.nanosec * 1e-9, "source": "motion",
                    "frame_id": msg.header.frame_id, "blobs": blobs})))
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
