"""nodes/grid_dist.py: bid distances along the map, not through racks."""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))

from grid_dist import GridDistance  # noqa: E402

RES = 0.05


def grid(size_m=10.0, wall_x=5.0, gap_from_y=None):
    """size x size map, free, with a wall at wall_x from y=0 to gap_from_y."""
    n = int(size_m / RES)
    data = [0] * (n * n)
    if wall_x is not None:
        wx = int(wall_x / RES)
        top = n if gap_from_y is None else int(gap_from_y / RES)
        for y in range(top):
            data[y * n + wx] = 100
    return GridDistance(n, n, RES, (0.0, 0.0), data)


def test_open_floor_is_about_the_straight_line():
    g = grid(wall_x=None)
    d = g.path_length((1.0, 1.0), (8.0, 1.0))
    assert d is not None and 7.0 <= d <= 7.6  # coarse cells, +/- one cell


def test_rack_between_robot_and_shelf_lengthens_the_drive():
    g = grid(wall_x=5.0, gap_from_y=8.0)  # wall with a gap above y=8
    straight = math.dist((2.0, 2.0), (8.0, 2.0))
    d = g.path_length((2.0, 2.0), (8.0, 2.0))
    assert d is not None and d > straight + 6.0  # up to the gap and back down


def test_no_path_and_off_map_give_none():
    g = grid(wall_x=5.0)  # full-height wall
    assert g.path_length((2.0, 2.0), (8.0, 2.0)) is None
    assert g.path_length((2.0, 2.0), (12.0, 2.0)) is None


def test_same_cell_and_unknown_space():
    g = grid(wall_x=None)
    assert g.path_length((1.0, 1.0), (1.05, 1.0)) == math.dist((1.0, 1.0), (1.05, 1.0))
    n = int(10.0 / RES)
    unknown = GridDistance(n, n, RES, (0.0, 0.0), [-1] * (n * n))
    assert unknown.path_length((1.0, 1.0), (8.0, 1.0)) is None
