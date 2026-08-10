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
