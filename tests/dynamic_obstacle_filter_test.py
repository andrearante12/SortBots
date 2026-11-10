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
