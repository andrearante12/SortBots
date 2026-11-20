"""Offline tests for dynamic depth filter (no ROS / no GPU)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))

import dynamic_obstacle_filter as dof  # noqa: E402


def _blank(h=48, w=64, z=2.0):
    return np.full((h, w), z, dtype=np.float32)


def test_no_prev_means_no_dynamic():
    d = _blank()
    mask = dof.select_dynamic_mask(d, None)
    assert not mask.any()
    static = dof.apply_static_depth(d, mask)
    assert np.isfinite(static).all()


def test_moving_blob_masked_and_static_nan():
    prev = _blank()
    cur = prev.copy()
    # Robot-sized blob jumps closer (depth decrease).
    cur[20:30, 25:40] = 1.0
    prev[20:30, 25:40] = 2.5
    mask = dof.select_dynamic_mask(cur, prev, {
        "change_m": 0.1,
        "min_blob_px": 20,
        "max_blob_px": 50000,
        "morph_close": 1,
        "min_range_m": 0.3,
        "max_range_m": 5.0,
    })
    assert mask.sum() >= 20
    static = dof.apply_static_depth(cur, mask)
    assert np.isnan(static[mask]).all()
    assert np.isfinite(static[~mask]).all()


def test_tiny_speckle_rejected():
    prev = _blank()
    cur = prev.copy()
    cur[10, 10] = 1.0
    prev[10, 10] = 3.0
    mask = dof.select_dynamic_mask(cur, prev, {
        "change_m": 0.1,
        "min_blob_px": 40,
        "max_blob_px": 50000,
        "morph_close": 0,
    })
    assert not mask.any()


def test_depth_to_points_optical_frame():
    depth = _blank(10, 10, z=2.0)
    mask = np.zeros_like(depth, dtype=bool)
    mask[5, 5] = True
    pts = dof.depth_to_points(depth, mask, fx=100.0, fy=100.0, cx=5.0, cy=5.0, stride=1)
    assert pts.shape == (1, 3)
    assert pts[0, 2] == pytest.approx(2.0)
    assert pts[0, 0] == pytest.approx(0.0)
    assert pts[0, 1] == pytest.approx(0.0)


# Optical (x right, y down, z fwd) -> map (x fwd, y left, z up), camera at the origin.
R_MAP_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)


def test_peer_mask_covers_a_reported_peer():
    """A wall 2 m ahead with a peer reported dead ahead on it: the pixels within
    peer_radius_m of the peer are masked, the rest of the wall is not."""
    h, w = 48, 64
    depth = _blank(h, w, z=2.0)
    fx = fy = 32.0
    m = dof.peer_mask(depth, fx, fy, w / 2, h / 2, R_MAP_OPT, np.zeros(3, np.float32),
                      [(2.0, 0.0)], radius_m=0.5)
    assert m[h // 2, w // 2]
    # Lateral offset at 2 m: (u - cx) * 2 / 32 m. Columns 8 px either side
    # (0.5 m) are the edge; 16 px (1 m) out must stay unmasked.
    assert not m[h // 2, w // 2 + 16] and not m[h // 2, w // 2 - 16]
    cols = np.nonzero(m[h // 2])[0]
    assert (cols.max() - cols.min()) == pytest.approx(16, abs=1)


def test_peer_mask_ignores_far_or_absent_peers():
    depth = _blank(48, 64, z=2.0)
    args = (32.0, 32.0, 32.0, 24.0, R_MAP_OPT, np.zeros(3, np.float32))
    assert not dof.peer_mask(depth, *args, []).any()
    assert not dof.peer_mask(depth, *args, [(2.0, 5.0)]).any()   # off to the side
    assert not dof.peer_mask(depth, *args, [(-2.0, 0.0)]).any()  # behind


def test_peer_mask_skips_invalid_depth():
    depth = _blank(48, 64, z=2.0)
    depth[:, :] = np.nan
    assert not dof.peer_mask(depth, 32.0, 32.0, 32.0, 24.0, R_MAP_OPT,
                             np.zeros(3, np.float32), [(2.0, 0.0)]).any()


# --- v2: ego-motion warp ----------------------------------------------------
# A tiny numpy raycaster of axis-aligned boxes in a z-up world (= <id>/odom),
# so the camera can actually move and the scene has real depth edges: a shelf
# in front of a far wall is exactly what v1's unwarped diff flagged as a mover.

W, H = 160, 120
K_SMALL = (80.0, 80.0, W / 2, H / 2)
SHELF = ((2.5, 3.0), (0.3, 1.5), (0.0, 1.8))
WALL = ((6.0, 6.5), (-5.0, 5.0), (0.0, 3.0))
FLOOR = ((-1.0, 9.0), (-5.0, 5.0), (-1.0, 0.0))


def _person(y0):
    return ((3.5, 3.8), (y0, y0 + 0.4), (0.0, 1.7))


def _yaw(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def _cam(x, y, yaw):
    """World pose of the optical frame: body at (x, y, 1.0 m), yawed."""
    R = _yaw(yaw) @ R_MAP_OPT.astype(np.float64)
    return R, np.array([x, y, 1.0])


def _render(R, c, boxes, K=K_SMALL, h=H, w=W):
    fx, fy, cx, cy = K
    rx = (np.arange(w) - cx) / fx
    ry = (np.arange(h) - cy) / fy
    dirs = np.stack(np.broadcast_arrays(rx[None, :], ry[:, None], np.ones((1, 1))), -1)
    dw = dirs @ R.T  # optical z = 1, so the ray parameter IS the depth
    best = np.full((h, w), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for box in boxes:
            t0 = np.full((h, w), -np.inf)
            t1 = np.full((h, w), np.inf)
            for k, (lo, hi) in enumerate(box):
                a = (lo - c[k]) / dw[..., k]
                b = (hi - c[k]) / dw[..., k]
                t0 = np.maximum(t0, np.minimum(a, b))
                t1 = np.minimum(t1, np.maximum(a, b))
            hit = (t1 >= t0) & (t0 > 0)
            best = np.where(hit & (t0 < best), t0, best)
    return np.where(np.isfinite(best), best, np.nan).astype(np.float32)


def _rel(Rp, cp, Rc, cc):
    """prev <- cur optical transform."""
    return (Rp.T @ Rc).astype(np.float32), (Rp.T @ (cc - cp)).astype(np.float32)


CFG = {"min_blob_px": 20, "morph_close": 1}


def test_static_scene_moving_camera_is_not_motion():
    Rp, cp = _cam(0.0, 0.0, 0.0)
    Rc, cc = _cam(0.15, 0.03, np.radians(4.0))
    scene = [FLOOR, WALL, SHELF]
    prev, cur = _render(Rp, cp, scene), _render(Rc, cc, scene)
    R, t = _rel(Rp, cp, Rc, cc)
    mask = dof.select_dynamic_mask(cur, prev, CFG, K=K_SMALL, R_prev_cur=R,
                                   t_prev_cur=t, R_odom_cam=Rc, t_odom_cam=cc)
    assert not mask.any()
    # The bug this replaces: the same frames diffed without the warp flag the
    # shelf's edge against the wall.
    assert dof.select_dynamic_mask(cur, prev, CFG).any()


def test_person_moving_while_camera_moves_is_motion():
    Rp, cp = _cam(0.0, 0.0, 0.0)
    Rc, cc = _cam(0.15, 0.03, np.radians(4.0))
    prev = _render(Rp, cp, [FLOOR, WALL, SHELF, _person(-1.2)])
    cur = _render(Rc, cc, [FLOOR, WALL, SHELF, _person(-0.6)])
    person_px = np.isfinite(cur) & ~np.isclose(
        cur, _render(Rc, cc, [FLOOR, WALL, SHELF]), equal_nan=True)
    R, t = _rel(Rp, cp, Rc, cc)
    mask, blobs = dof.select_dynamic_blobs(cur, prev, CFG, K=K_SMALL, R_prev_cur=R,
                                           t_prev_cur=t, R_odom_cam=Rc, t_odom_cam=cc)
    assert mask.any()
    # Everything flagged is person (one stride-2 block of slack at its edge)...
    grown = dof._morph_close(person_px, 0)
    for _ in range(2):
        grown = grown | np.roll(grown, 1, 0) | np.roll(grown, -1, 0) \
            | np.roll(grown, 1, 1) | np.roll(grown, -1, 1)
    assert not (mask & ~grown).any()
    # ...and the part of it that moved onto background is caught.
    assert (mask & person_px).sum() >= 0.3 * person_px.sum()
    assert len(blobs) == 1
    u0, v0, u1, v1 = blobs[0]["bbox"]
    assert u1 > u0 and v1 > v0 and blobs[0]["depth_m"] > 3.0


def test_height_gate_drops_floor_level_motion():
    Rp, cp = _cam(0.0, 0.0, 0.0)
    low = ((2.0, 2.4), (-0.6, -0.2), (0.0, 0.05))  # a 5 cm sheet sliding across the floor
    moved = ((2.0, 2.4), (-0.2, 0.2), (0.0, 0.05))
    prev = _render(Rp, cp, [FLOOR, WALL, low])
    cur = _render(Rp, cp, [FLOOR, WALL, moved])
    eye, zero = np.eye(3, dtype=np.float32), np.zeros(3, np.float32)
    cfg = {**CFG, "change_m": 0.01, "min_height_m": 0.08}
    assert not dof.select_dynamic_mask(cur, prev, cfg, K=K_SMALL, R_prev_cur=eye,
                                       t_prev_cur=zero, R_odom_cam=Rp, t_odom_cam=cp).any()


def test_point_heights_floor_is_zero():
    R, c = _cam(0.0, 0.0, 0.0)
    depth = _render(R, c, [FLOOR, WALL])
    z = dof.point_heights(depth, K_SMALL, R, c)
    assert abs(float(z[-1, W // 2])) < 1e-3  # bottom row sees floor
    assert np.nanmax(z) > 1.0                 # top rows see the wall


def test_missing_tf_frame_has_no_motion():
    """No prev frame (the node passes None on a TF miss) means no mask at all."""
    R, c = _cam(0.0, 0.0, 0.0)
    cur = _render(R, c, [FLOOR, WALL, SHELF])
    assert not dof.select_dynamic_mask(cur, None, CFG, K=K_SMALL).any()


def test_full_res_cost_fits_a_frame():
    import time
    Kf = (430.0, 430.0, 424.0, 240.0)
    Rp, cp = _cam(0.0, 0.0, 0.0)
    Rc, cc = _cam(0.05, 0.0, np.radians(1.0))
    scene = [FLOOR, WALL, SHELF, _person(-0.6)]
    prev = _render(Rp, cp, scene, Kf, 480, 848)
    cur = _render(Rc, cc, scene, Kf, 480, 848)
    R, t = _rel(Rp, cp, Rc, cc)
    t0 = time.perf_counter()
    for _ in range(5):
        dof.select_dynamic_blobs(cur, prev, None, K=Kf, R_prev_cur=R, t_prev_cur=t,
                                 R_odom_cam=Rc, t_odom_cam=cc)
    per = (time.perf_counter() - t0) / 5
    print(f"select_dynamic_blobs 848x480: {per * 1e3:.1f} ms")
    assert per < 0.2  # loose: CI boxes vary; the real budget is 33 ms
