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
    # Map-conflict cue (nodes/map_conflict.py), only with --map-conflict. It
    # ray-traces every Nth pixel, ~50 ms a pass, so it runs at a fraction of
    # the camera rate; RTAB-Map only takes keyframes at ~2 Hz anyway.
    "conflict_rate_hz": 2.0,
    "conflict_stride": 8,
    "conflict_min_px": 3,      # decimated points per conflict blob
    # Segment masks from nodes/dynamic_segment.py (--dynamic-seg).
    # open:   each depth frame uses the newest mask within seg_max_age_s, or
    #         none (today's behaviour). The mask can lag the depth by the
    #         model's latency, which seg_dilate_px absorbs.
    # strict: a depth frame is only published once the mask for ITS stamp
    #         arrives, so RTAB-Map never sees an unmasked or misaligned frame.
    #         depth_static then runs at the mask rate (5-10 Hz) with the
    #         model's latency, which is the right trade when recording a map
    #         to keep, and the wrong one for Nav2's costmaps.
    "seg_policy": "open",
    # open: the newest mask may be this old. 0.5, not 0.3, since dynamic_segment runs at 3 Hz
    # (0.33 s apart plus ~0.1 s of model latency; 2026-11-20): at 0.3 a third of frames went unmasked.
    "seg_max_age_s": 0.5,
    "seg_match_s": 0.005,      # strict: same frame (rgb and depth share stamps)
    "seg_queue_s": 1.0,        # strict: drop a depth frame whose mask never came
    "seg_dilate_px": 4,        # rgb-vs-depth edge slack and the open-mode lag
    # Flying pixels: a depth edge interpolates between the object and what is
    # behind it, leaving a halo SLAM paints as a faint outline. In a band
    # around each masked object, pixels no farther than the object's far side
    # plus seg_halo_dz_m are masked too.
    "seg_halo_px": 8,
    "seg_halo_dz_m": 0.3,
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


def conflict_blobs(conflict: np.ndarray, depth: np.ndarray, stride: int,
                   min_px: int) -> list[dict]:
    """Per-point conflict flags (row-major over depth[::stride, ::stride]) →
    blob summaries in full-resolution pixels, like select_dynamic_blobs'."""
    d = depth[::stride, ::stride]
    m = conflict.reshape(d.shape)
    labels, sizes = label_blobs(_morph_close(m, 1))
    ok = np.asarray(sizes) >= min_px
    ok[0] = False
    return blob_summaries(np.where(ok[labels], labels, 0), d, stride)


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


def _dilate(mask: np.ndarray, px: int) -> np.ndarray:
    """Square dilation by px, separably (a (2px+1)^2 window)."""
    if px <= 0:
        return mask
    out = mask.copy()
    for axis in (0, 1):
        src = out.copy()
        for k in range(1, px + 1):
            if axis == 0:
                out[k:, :] |= src[:-k, :]
                out[:-k, :] |= src[k:, :]
            else:
                out[:, k:] |= src[:, :-k]
                out[:, :-k] |= src[:, k:]
    return out


def resize_nearest(mask: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    """Nearest-neighbour resize, for a mask published at another resolution."""
    if mask.shape == tuple(shape):
        return mask
    h, w = shape
    ys = (np.arange(h) * mask.shape[0] // h).clip(0, mask.shape[0] - 1)
    xs = (np.arange(w) * mask.shape[1] // w).clip(0, mask.shape[1] - 1)
    return mask[ys[:, None], xs[None, :]]


def grow_seg_mask(seg: np.ndarray, depth: np.ndarray, cfg: dict | None = None) -> np.ndarray:
    """Segment mask → depth mask: resize to the depth image, dilate by
    seg_dilate_px, then add each object's flying-pixel halo (see DEFAULTS)."""
    c = {**DEFAULTS, **(cfg or {})}
    m = resize_nearest(seg.astype(bool, copy=False), depth.shape)
    if not m.any():
        return m
    core = _dilate(m, int(c["seg_dilate_px"]))
    halo_px = int(c["seg_halo_px"])
    if halo_px <= 0:
        return core
    labels, _ = label_blobs(core)
    band = _dilate(core, halo_px) & ~core
    d = depth.astype(np.float32, copy=False)
    # Each band pixel belongs to the object whose label is nearest; a
    # label-propagating dilation is cheaper than a distance transform and
    # exact enough for an 8 px band.
    lab = labels.copy()
    for _ in range(halo_px):
        grown = lab.copy()
        for sl_dst, sl_src in (((slice(1, None), slice(None)), (slice(None, -1), slice(None))),
                               ((slice(None, -1), slice(None)), (slice(1, None), slice(None))),
                               ((slice(None), slice(1, None)), (slice(None), slice(None, -1))),
                               ((slice(None), slice(None, -1)), (slice(None), slice(1, None)))):
            dst = grown[sl_dst]
            src = lab[sl_src]
            fill = (dst == 0) & (src > 0)
            dst[fill] = src[fill]
        lab = grown
    far = np.zeros(int(labels.max()) + 1, dtype=np.float32)
    for k in range(1, far.size):
        z = d[(labels == k) & m]
        z = z[np.isfinite(z) & (z > 0)]
        far[k] = np.percentile(z, 90) if z.size else -np.inf
    bl = lab[band]
    bz = d[band]
    with np.errstate(invalid="ignore"):
        take = (bl > 0) & np.isfinite(bz) & (bz <= far[bl] + float(c["seg_halo_dz_m"]))
    out = core.copy()
    ys, xs = np.nonzero(band)
    out[ys[take], xs[take]] = True
    return out


def _stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def seg_for_stamp(masks, stamp: float, tol: float):
    """Newest (stamp, mask) within tol of `stamp`, or None. `masks` is any
    iterable of (stamp, mask), oldest first."""
    best = None
    for st, m in masks:
        if abs(st - stamp) <= tol and (best is None or st > best[0]):
            best = (st, m)
    return best


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


# /<id>/movers: where the segmented movers are on the floor right now, for the operator
# map's People layer. The SLAM map never holds them (depth_static strips them, and a
# keyframe can't be un-painted); this is the live layer beside it, fresh points kept and
# old ones dropped by the page. A few hundred bytes at movers_rate_hz instead of the
# masked cloud (tens of thousands of points a frame).
MOVERS = {
    "movers_rate_hz": 5.0,
    "movers_cell_m": 0.1,
    "movers_stride": 4,          # full-res pixels between samples of the mask
    "movers_min_z_m": 0.1,       # above base_link (the floor): not the floor under them
    "movers_max_z_m": 2.2,
    "movers_max_range_m": 6.0,
    # A cluster smaller than this is the mask's edge bleeding onto the wall behind
    # (grow_seg_mask's halo), not a mover: a person is 4-20 cells at 0.1 m.
    "movers_min_cells": 3,
}


def mover_cells(pts: np.ndarray, cfg: dict | None = None) -> list[dict]:
    """Movers as floor cells, from masked points (Nx3, base_link, z up from the floor).

    Points in the height band and range are binned into cells; 8-connected cells are one
    mover. Each: {"at": [x, y] (mean of its points), "n": points, "cells": [[x, y], ...]
    (cell centres)}, nearest first, metres rounded to the cm.
    """
    c = {**MOVERS, **(cfg or {})}
    if pts.size == 0:
        return []
    z = pts[:, 2]
    keep = ((z >= c["movers_min_z_m"]) & (z <= c["movers_max_z_m"])
            & (np.hypot(pts[:, 0], pts[:, 1]) <= c["movers_max_range_m"]))
    xy = pts[keep, :2]
    if not len(xy):
        return []
    s = float(c["movers_cell_m"])
    ij = np.floor(xy / s).astype(np.int64)
    cells, inv = np.unique(ij, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    index = {(int(i), int(j)): k for k, (i, j) in enumerate(cells)}
    label = np.full(len(cells), -1)
    out = []
    for k0 in range(len(cells)):
        if label[k0] >= 0:
            continue
        label[k0] = k0
        group, todo = [k0], [k0]
        while todo:
            i, j = cells[todo.pop()]
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    k = index.get((int(i) + di, int(j) + dj))
                    if k is not None and label[k] < 0:
                        label[k] = k0
                        group.append(k)
                        todo.append(k)
        if len(group) < c["movers_min_cells"]:
            continue
        mine = np.isin(inv, group)
        at = xy[mine].mean(axis=0)
        out.append({
            "at": [round(float(at[0]), 2), round(float(at[1]), 2)],
            "n": int(mine.sum()),
            "cells": [[round((float(i) + 0.5) * s, 2), round((float(j) + 0.5) * s, 2)]
                      for i, j in cells[group]],
        })
    out.sort(key=lambda m: m["at"][0] ** 2 + m["at"][1] ** 2)
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument("--map-conflict", action="store_true",
                        help="also publish map-conflict blobs (nodes/map_conflict.py)")
    parser.add_argument("--dynamic-seg", action="store_true",
                        help="also mask /<id>/camera/dynamic_seg (nodes/dynamic_segment.py)")
    parser.add_argument("--seg-policy", choices=("open", "strict"), default=None,
                        help="override DEFAULTS['seg_policy']")
    args = parser.parse_args(argv)

    import json
    import time
    from collections import deque

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
    from map_conflict import FreeSpaceGrid, backproject_odom

    robot_id = args.robot_id
    odom_frame = f"{robot_id}/odom"
    base_frame = f"{robot_id}/base_link"

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
            self._grid = FreeSpaceGrid() if args.map_conflict else None
            self._next_conflict = 0.0
            self._segs: deque = deque(maxlen=16)     # (stamp s, bool mask)
            self._pending: deque = deque()           # strict: (stamp, msg, depth, mask)
            self._seg_stats = {"used": 0, "stale": 0, "dropped": 0}
            self._cfg = dict(DEFAULTS)
            if args.seg_policy:
                self._cfg["seg_policy"] = args.seg_policy
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
            if args.dynamic_seg:
                self.create_subscription(
                    Image, f"/{robot_id}/camera/dynamic_seg", self._on_seg, qos
                )
                self.create_timer(10.0, self._log_seg_stats)
                self.movers_pub = self.create_publisher(String, f"/{robot_id}/movers", 10)
                self._movers_next = 0.0
                self._movers_any = False
            self.static_pub = self.create_publisher(
                Image, f"/{robot_id}/camera/depth_static", qos
            )
            self.dyn_pub = self.create_publisher(
                PointCloud2, f"/{robot_id}/dynamic_obstacles", qos
            )
            # Motion and map-conflict blobs as SAM prompts for
            # nodes/dynamic_segment.py, one message per source per frame. JSON
            # on String like the rest of the fleet protocol, never an image.
            self.blob_pub = self.create_publisher(
                String, f"/{robot_id}/camera/dynamic_blobs", qos
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
            """(mask, blobs, odom pose) from the warped previous frame, or no
            motion when either pose is unknown (an unwarped diff is v1's
            shelf-edge bug)."""
            prev, self._prev = self._prev, (depth.copy(), msg.header.stamp, msg.header.frame_id)
            empty = np.zeros(depth.shape, dtype=bool), [], None
            if prev is None or self._info is None or prev[2] != msg.header.frame_id:
                return empty
            K = self._info.k
            Kt = (float(K[0]), float(K[4]), float(K[2]), float(K[5]))
            frame = msg.header.frame_id
            odom = self._lookup(odom_frame, msg.header.stamp, frame, msg.header.stamp)
            rel = odom and self._lookup(frame, prev[1], frame, msg.header.stamp)
            if not rel or not odom:
                self._tf_misses += 1
                if self._tf_misses in (1, 30) or self._tf_misses % 300 == 0:
                    self.get_logger().warning(
                        f"no exact-stamp TF {frame} via {odom_frame} "
                        f"({self._tf_misses} frames without a motion mask)")
                return empty
            mask, blobs = select_dynamic_blobs(depth, prev[0], self._cfg, K=Kt,
                                               R_prev_cur=rel[0], t_prev_cur=rel[1],
                                               R_odom_cam=odom[0], t_odom_cam=odom[1])
            return mask, blobs, odom

        def _conflict(self, msg: Image, depth: np.ndarray, odom) -> list[dict]:
            now = time.monotonic()
            if self._grid is None or odom is None or now < self._next_conflict:
                return []
            self._next_conflict = now + 1.0 / float(self._cfg["conflict_rate_hz"])
            K = self._info.k
            Kt = (float(K[0]), float(K[4]), float(K[2]), float(K[5]))
            stride = int(self._cfg["conflict_stride"])
            pts = backproject_odom(depth, Kt, odom[0], odom[1], stride)
            st = msg.header.stamp
            flags = self._grid.update(pts, odom[1], st.sec + st.nanosec * 1e-9)
            return conflict_blobs(flags, depth, stride, int(self._cfg["conflict_min_px"]))

        def _publish_blobs(self, msg: Image, source: str, blobs: list[dict]):
            if not blobs:
                return
            st = msg.header.stamp
            self.blob_pub.publish(String(data=json.dumps({
                "stamp": st.sec + st.nanosec * 1e-9, "source": source,
                "frame_id": msg.header.frame_id, "blobs": blobs})))

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

        def _on_seg(self, msg: Image):
            if msg.encoding not in ("mono8", "8UC1"):
                return
            m = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            self._segs.append((_stamp_s(msg.header.stamp), m[:, :msg.width] > 0))
            if self._cfg["seg_policy"] == "strict":
                self._drain_pending()

        def _log_seg_stats(self):
            st = self._seg_stats
            self.get_logger().info(
                f"dynamic_seg ({self._cfg['seg_policy']}): {st['used']} frames masked, "
                f"{st['stale']} without a fresh mask, {st['dropped']} held back")
            for k in st:
                st[k] = 0

        def _drain_pending(self):
            """strict: publish each held depth frame whose own mask has arrived;
            drop the ones whose mask is now too late to come."""
            c = self._cfg
            # Age against the newest DEPTH frame too: with the segmenter down no
            # mask ever arrives, and judging only by mask stamps would hold every
            # 1.6 MB depth frame forever.
            newest = max(self._segs[-1][0] if self._segs else -np.inf,
                         self._pending[-1][0] if self._pending else -np.inf)
            keep = deque()
            for stamp, msg, depth, mask in self._pending:
                hit = seg_for_stamp(self._segs, stamp, c["seg_match_s"])
                if hit is not None:
                    self._seg_stats["used"] += 1
                    self._publish_static(msg, depth, mask | grow_seg_mask(hit[1], depth, c))
                elif newest - stamp > c["seg_queue_s"]:
                    self._seg_stats["dropped"] += 1
                else:
                    keep.append((stamp, msg, depth, mask))
            self._pending = keep

        def _on_depth(self, msg: Image):
            if msg.encoding not in ("32FC1", "mono16", "16UC1"):
                return
            h, w = msg.height, msg.width
            if msg.encoding == "32FC1":
                depth = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
            else:
                raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
                depth = raw.astype(np.float32) * 0.001  # typical mm→m; sim is 32FC1

            mask, blobs, odom = self._motion(msg, depth)
            self._publish_blobs(msg, "motion", blobs)
            self._publish_blobs(msg, "conflict", self._conflict(msg, depth, odom))
            peers = self._peer_mask(msg, depth)
            if peers is not None:
                mask |= peers

            if not args.dynamic_seg:
                self._publish_static(msg, depth, mask)
                self._publish_dynamic(msg, depth, mask)
                return
            stamp = _stamp_s(msg.header.stamp)
            hit = seg_for_stamp(self._segs, stamp, self._cfg["seg_max_age_s"])
            seg = grow_seg_mask(hit[1], depth, self._cfg) if hit else None
            live = mask | seg if hit else mask
            self._publish_movers(msg, depth, seg)
            if self._cfg["seg_policy"] == "strict":
                # depth_static waits for this frame's own mask; Nav2's dynamic
                # cloud does not, so the costmaps keep clearing at camera rate.
                self._pending.append((stamp, msg, depth.copy(), mask))
                self._drain_pending()
            else:
                self._seg_stats["used" if hit else "stale"] += 1
                self._publish_static(msg, depth, live)
            self._publish_dynamic(msg, depth, live)

        def _publish_movers(self, msg: Image, depth: np.ndarray, seg):
            """The segmented movers on the floor (mover_cells), at most movers_rate_hz. In
            base_link at the newest TF, not the frame's stamp: the exact-stamp lookup is
            what keeps missing on the Orin, and the page places them with the base pose
            it holds on arrival, as it does /<id>/obstacle_tracks."""
            now = time.monotonic()
            if now < self._movers_next or self._info is None:
                return
            self._movers_next = now + 1.0 / MOVERS["movers_rate_hz"]
            movers = []
            if seg is not None and seg.any():
                try:
                    tf = self._tf.lookup_transform(base_frame, msg.header.frame_id, Time(),
                                                   timeout=Duration(seconds=0.0))
                except Exception:  # noqa: BLE001 - tf2 raises several types
                    return
                K = self._info.k
                pts = depth_to_points(depth, seg, float(K[0]), float(K[4]), float(K[2]),
                                      float(K[5]), stride=MOVERS["movers_stride"])
                q, t = tf.transform.rotation, tf.transform.translation
                R = _quat_to_matrix(q.x, q.y, q.z, q.w)
                movers = mover_cells(pts @ R.T + np.array([t.x, t.y, t.z], dtype=np.float32))
            # One empty message after the last mover, then quiet until the next one.
            if not movers and not self._movers_any:
                return
            self._movers_any = bool(movers)
            st = msg.header.stamp
            self.movers_pub.publish(String(data=json.dumps({
                "stamp": round(st.sec + st.nanosec * 1e-9, 3), "frame": base_frame,
                "cell_m": MOVERS["movers_cell_m"], "movers": movers})))

        def _publish_static(self, msg: Image, depth: np.ndarray, mask: np.ndarray):
            h, w = depth.shape
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

        def _publish_dynamic(self, msg: Image, depth: np.ndarray, mask: np.ndarray):
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
