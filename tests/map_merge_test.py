#!/usr/bin/env python3
"""Unit tests for nodes/map_fuse.py — the ROS-free grid-fusion core behind
nodes/map_merge.py.

Pure python + numpy: no ROS, no Isaac, no GPU. Runs anywhere in under a
second — the point, per CLAUDE.md's testing section ("no ROS"), since
nodes/map_merge.py itself imports rclpy at module scope and can't be
unit-tested this way (see nodes/map_fuse.py's docstring for why the split
exists).

    python3 -m pytest tests/map_merge_test.py
    python3 tests/map_merge_test.py            # same, without pytest
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "nodes"))

from map_fuse import GridInfo, Transform2D, fuse_grids  # noqa: E402

IDENTITY = Transform2D(0.0, 0.0, 0.0)


def test_single_source_identity_transform_passes_through_unchanged():
    data = np.full((4, 4), -1, dtype=np.int8)
    data[1, 1] = 0
    data[2, 2] = 100
    info = GridInfo(resolution=1.0, origin_x=0.0, origin_y=0.0, width=4, height=4)

    fused, out_info = fuse_grids([(data, info, IDENTITY)], resolution=1.0)

    assert (out_info.width, out_info.height) == (4, 4)
    assert (out_info.origin_x, out_info.origin_y) == (0.0, 0.0)
    assert np.array_equal(fused, data)


def test_two_sources_offset_in_x_union_to_a_wider_grid():
    # Two robots' own grids, side by side in the merged frame — the basic
    # "two spawn points" case this whole node exists for.
    a = np.full((4, 4), -1, dtype=np.int8)
    a[1, 1] = 0
    a[2, 2] = 100
    a_info = GridInfo(resolution=1.0, origin_x=0.0, origin_y=0.0, width=4, height=4)

    b = np.full((4, 4), -1, dtype=np.int8)
    b[0, 0] = 0
    b[3, 3] = 100
    b_info = GridInfo(resolution=1.0, origin_x=0.0, origin_y=0.0, width=4, height=4)
    # b's local frame sits 4m to the +x of the merged frame — e.g. robot_1
    # spawned 4m east of robot_1/map's own origin.
    b_transform = Transform2D(4.0, 0.0, 0.0)

    fused, out_info = fuse_grids(
        [(a, a_info, IDENTITY), (b, b_info, b_transform)], resolution=1.0
    )

    assert (out_info.width, out_info.height) == (8, 4)
    assert fused[1, 1] == 0
    assert fused[2, 2] == 100
    assert fused[0, 4] == 0    # b's (0,0) lands at world x in [4,5) -> out col 4
    assert fused[3, 7] == 100  # b's (3,3) lands at world x in [7,8) -> out col 7


def test_occupied_beats_free_beats_unknown_on_overlap():
    a = np.array([[100, -1], [0, -1]], dtype=np.int8)
    b = np.array([[-1, 0], [50, -1]], dtype=np.int8)
    info = GridInfo(1.0, 0.0, 0.0, 2, 2)

    fused, _ = fuse_grids([(a, info, IDENTITY), (b, info, IDENTITY)], resolution=1.0)

    assert fused[0, 0] == 100  # occupied(a) vs unknown(b) -> occupied
    assert fused[0, 1] == 0    # unknown(a) vs free(b) -> free
    assert fused[1, 0] == 50   # free(a)=0 vs occupied(b)=50 (>= default thresh) -> occupied
    assert fused[1, 1] == -1   # both unknown


def test_prev_info_grows_the_output_but_never_shrinks_it():
    data = np.full((4, 4), -1, dtype=np.int8)
    info = GridInfo(resolution=1.0, origin_x=0.0, origin_y=0.0, width=4, height=4)
    prev = GridInfo(resolution=1.0, origin_x=-5.0, origin_y=-5.0, width=20, height=20)

    fused, out_info = fuse_grids(
        [(data, info, IDENTITY)], resolution=1.0, prev_info=prev
    )

    assert (out_info.origin_x, out_info.origin_y) == (-5.0, -5.0)
    assert out_info.width >= 20 and out_info.height >= 20


def test_ninety_degree_yaw_rotates_the_source_into_the_output_frame():
    # A 1x3 strip along the source's local +x, with the far cell occupied.
    # Under a +90deg yaw, local +x maps to output +y — verify the occupied
    # cell lands on the output's y axis (col ~0), not still along x.
    data = np.zeros((1, 3), dtype=np.int8)
    data[0, 2] = 100
    info = GridInfo(resolution=1.0, origin_x=0.0, origin_y=0.0, width=3, height=1)
    transform = Transform2D(0.0, 0.0, yaw_rad=np.pi / 2)

    fused, out_info = fuse_grids([(data, info, transform)], resolution=1.0)

    occupied_rows, occupied_cols = np.where(fused == 100)
    assert len(occupied_rows) == 1
    # The occupied cell ends up near the LOW-x edge of the output (col 0),
    # confirming the rotation actually moved it off the original x axis.
    assert occupied_cols[0] <= 1
    assert out_info.height >= 3  # the strip's 3m local extent is now along y


def test_fuse_grids_rejects_an_empty_source_list():
    with pytest.raises(ValueError):
        fuse_grids([], resolution=1.0)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
