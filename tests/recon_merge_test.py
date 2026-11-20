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


# --- free-space-aware fusion --------------------------------------------------

V = 0.05


def _block(x0, x1, y0, y1, z0, z1, v=V):
    """Voxel centres filling a box."""
    g = np.mgrid[x0:x1:v, y0:y1:v, z0:z1:v].reshape(3, -1).T + v / 2
    return g.astype(np.float32)


IDENT = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def test_other_robots_free_space_erases_a_ghost():
    wall = _block(3.0, 3.1, -1.0, 1.0, 0.0, 1.0)
    ghost = _block(1.5, 1.8, 0.0, 0.3, 0.0, 1.5)        # robot_0 saw a person here
    free_b = _block(0.5, 2.9, -0.5, 0.5, 0.0, 1.6)      # robot_1 later saw it empty
    keep = rf.free_space_erase([np.vstack([wall, ghost]), wall], [None, free_b], voxel=V)
    assert keep[0][: len(wall)].all()                    # robot_0's wall survives
    assert not keep[0][len(wall):].any()                 # its ghost is gone
    assert keep[1].all()


def test_own_free_space_never_erases_own_points():
    pts = _block(1.0, 1.2, 0.0, 0.2, 0.0, 0.2)
    keep = rf.free_space_erase([pts], [_block(0.5, 2.0, -0.5, 0.5, -0.2, 0.6)], voxel=V)
    assert keep[0].all()


def test_misaligned_wall_is_not_erased():
    # robot_1's wall is one voxel behind robot_0's; its free space runs up to
    # its own wall, i.e. through robot_0's wall voxels.
    wall_a = _block(3.0, 3.05, -1.0, 1.0, 0.0, 1.0)
    wall_b = _block(3.05, 3.1, -1.0, 1.0, 0.0, 1.0)
    free_b = _block(0.5, 3.05, -1.0, 1.0, 0.0, 1.0)
    keep = rf.free_space_erase([wall_a, wall_b], [None, free_b], voxel=V)
    assert keep[0].all()


def test_free_space_at_a_boundary_is_not_enough():
    # One stray free voxel on top of an occupied one, nothing free around it.
    pt = np.array([[1.025, 1.025, 1.025]], np.float32)
    keep = rf.free_space_erase([pt, np.zeros((0, 3), np.float32)], [None, pt.copy()], voxel=V)
    assert keep[0].all()


def test_fuse_clouds_clearing_applies_anchors_before_erasing():
    # robot_1's own frame is shifted +5 m in x: its free space at x 0.5..2.9
    # in its frame is x 5.5..7.9 in map, so it must NOT erase robot_0's ghost
    # at map x 1.5..1.8; at -5 m it lands on top of it and must.
    ghost = _block(1.5, 1.8, 0.0, 0.3, 0.0, 1.5)
    free_local = _block(6.5, 7.9, -0.5, 0.5, 0.0, 1.6)
    empty_occ = np.zeros((0, 3), np.float32)
    far = (5.0, 0, 0, 0, 0, 0, 1)
    near = (-5.0, 0, 0, 0, 0, 0, 1)
    xyz, _, _, erased = rf.fuse_clouds_clearing([(ghost, None, None, IDENT),
                                                 (empty_occ, None, free_local, far)], free_voxel=V)
    assert erased == 0 and len(xyz) == len(ghost)
    xyz, rgb, _, erased = rf.fuse_clouds_clearing([(ghost, None, None, IDENT),
                                                   (empty_occ, None, free_local, near)], free_voxel=V)
    assert erased == len(ghost) and len(xyz) == 0 and rgb.shape == (0, 4)


def test_fuse_clouds_clearing_keeps_colour_alignment():
    keep_pts = _block(0.0, 0.1, 0.0, 0.1, 0.0, 0.1)
    ghost = _block(1.5, 1.6, 0.0, 0.1, 0.0, 0.1)
    rgb = np.vstack([np.tile([[1, 2, 3, 0]], (len(keep_pts), 1)),
                     np.tile([[9, 9, 9, 0]], (len(ghost), 1))]).astype(np.uint8)
    free_b = _block(1.2, 1.9, -0.3, 0.4, -0.3, 0.4)
    xyz, out_rgb, _, erased = rf.fuse_clouds_clearing(
        [(np.vstack([keep_pts, ghost]), rgb, None, IDENT),
         (np.zeros((0, 3), np.float32), None, free_b, IDENT)], free_voxel=V)
    assert erased == len(ghost)
    assert (out_rgb[:, :3] == [1, 2, 3]).all()


def test_coarse_free_grid_still_erases_a_person_sized_ghost():
    ghost = _block(1.5, 1.9, 0.0, 0.4, 0.0, 1.7)
    free_b = _block(0.5, 2.9, -0.6, 0.6, 0.0, 1.8, v=0.1)   # relayed at 10 cm
    xyz, _, _, erased = rf.fuse_clouds_clearing([(ghost, None, None, IDENT),
                                                 (np.zeros((0, 3), np.float32), None, free_b, IDENT)])
    assert erased == len(ghost) and len(xyz) == 0
