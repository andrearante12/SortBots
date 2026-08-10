"""Offline tests for recon cloud fusion (no ROS)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))
import recon_fuse as rf  # noqa: E402


def test_identity_transform():
    xyz = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    out = rf.transform_xyz(xyz, 0, 0, 0)
    assert out == pytest.approx(xyz)


def test_translation_only():
    xyz = np.array([[1.0, 0.0, 0.5]], dtype=np.float32)
    out = rf.transform_xyz(xyz, -7.15, 11.62, 0.05)
    assert out[0] == pytest.approx([-6.15, 11.62, 0.55])


def test_fuse_two_robots_disjoint():
    a = np.array([[0.0, 0.0, 0.1], [0.1, 0.0, 0.1]], dtype=np.float32)
    b = np.array([[0.0, 0.0, 0.2]], dtype=np.float32)
    # robot_0 at origin, robot_1 translated +5 m in x
    xyz, rgb, leaf = rf.fuse_clouds(
        [
            (a, None, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)),
            (b, None, (5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)),
        ],
        max_points=1000,
        voxel_size=0.05,
    )
    assert len(xyz) == 3
    assert rgb.shape == (3, 4)
    xs = sorted(xyz[:, 0].tolist())
    assert xs[0] == pytest.approx(0.0, abs=1e-5)
    assert xs[-1] == pytest.approx(5.0, abs=1e-5)


def test_budget_caps_output():
    rng = np.random.default_rng(0)
    xyz = rng.random((5000, 3), dtype=np.float32) * 10.0
    out, _, leaf = rf.fuse_clouds(
        [(xyz, None, (0, 0, 0, 0, 0, 0, 1))],
        max_points=200,
        voxel_size=0.05,
    )
    assert len(out) <= 200
    assert leaf > 0
