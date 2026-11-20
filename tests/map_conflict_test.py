"""Offline tests for the map-conflict cue (no ROS / no GPU)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))

import map_conflict as mc  # noqa: E402
from tests._raycast import FLOOR, K_SMALL, SHELF, WALL, _cam, _person, _render  # noqa: E402

STRIDE = 2


def _frame(grid, R, c, boxes, now):
    depth = _render(R, c, boxes)
    pts = mc.backproject_odom(depth, K_SMALL, R, c, STRIDE)
    return pts, grid.update(pts, c, now)


def _in_box(pts, box):
    (x0, x1), (y0, y1), (z0, z1) = box
    e = 0.02
    return ((pts[:, 0] >= x0 - e) & (pts[:, 0] <= x1 + e) & (pts[:, 1] >= y0 - e)
            & (pts[:, 1] <= y1 + e) & (pts[:, 2] >= z0 - e) & (pts[:, 2] <= z1 + e))


def test_static_scene_from_a_moving_camera_never_conflicts():
    g = mc.FreeSpaceGrid()
    for i in range(12):
        R, c = _cam(0.1 * i, 0.02 * i, np.radians(2.0 * i))
        _, conf = _frame(g, R, c, [FLOOR, WALL, SHELF], float(i))
        assert not conf.any(), f"frame {i}"


def test_person_stepping_into_seen_free_space_conflicts():
    g = mc.FreeSpaceGrid()
    R, c = _cam(0.0, 0.0, 0.0)
    for i in range(6):
        _frame(g, R, c, [FLOOR, WALL, SHELF], float(i))
    person = _person(-0.6)
    pts, conf = _frame(g, R, c, [FLOOR, WALL, SHELF, person], 6.0)
    on_person = _in_box(pts, person) & (pts[:, 2] > 0.1)
    assert conf[on_person].mean() > 0.8
    assert not conf[~_in_box(pts, person)].any()


def test_too_few_looks_is_not_a_conflict():
    g = mc.FreeSpaceGrid({"min_free": 5})
    R, c = _cam(0.0, 0.0, 0.0)
    for i in range(3):
        _frame(g, R, c, [FLOOR, WALL], float(i))
    _, conf = _frame(g, R, c, [FLOOR, WALL, _person(-0.6)], 3.0)
    assert not conf.any()


def test_stale_free_evidence_does_not_conflict():
    g = mc.FreeSpaceGrid({"recent_s": 10.0})
    R, c = _cam(0.0, 0.0, 0.0)
    for i in range(6):
        _frame(g, R, c, [FLOOR, WALL], float(i))
    _, conf = _frame(g, R, c, [FLOOR, WALL, _person(-0.6)], 100.0)
    assert not conf.any()


def test_a_stopped_object_fades_into_map():
    cfg = {"min_free": 5, "free_cap": 8, "hit_decay": 1.0}
    g = mc.FreeSpaceGrid(cfg)
    R, c = _cam(0.0, 0.0, 0.0)
    for i in range(8):
        _frame(g, R, c, [FLOOR, WALL], float(i))
    person = _person(-0.6)
    flagged = []
    for i in range(8, 20):
        pts, conf = _frame(g, R, c, [FLOOR, WALL, person], float(i))
        flagged.append(conf[_in_box(pts, person)].mean())
    assert flagged[0] > 0.8
    # (cap 8 - min 5) / decay 1 = 3 frames as a conflict, then ordinary map.
    assert flagged[-1] == 0.0
    assert sum(f > 0.5 for f in flagged) <= 4


def test_recentre_keeps_evidence_in_place():
    g = mc.FreeSpaceGrid({"size_m": 4.0, "cell_m": 0.1})
    g.recentre(0.0, 0.0)
    ix, iy, _ = g._cells(np.array([0.55]), np.array([0.25]))
    g.free[ix[0], iy[0]] = 7
    g.recentre(1.5, -1.2)   # far enough to shift
    ix2, iy2, inside = g._cells(np.array([0.55]), np.array([0.25]))
    assert inside[0]
    assert g.free[ix2[0], iy2[0]] == 7
    assert g.free.sum() == 7


def test_conflict_blobs_point_at_the_person_in_full_res_pixels():
    import dynamic_obstacle_filter as dof
    g = mc.FreeSpaceGrid()
    R, c = _cam(0.0, 0.0, 0.0)
    for i in range(6):
        _frame(g, R, c, [FLOOR, WALL, SHELF], float(i))
    person = _person(-0.6)
    depth = _render(R, c, [FLOOR, WALL, SHELF, person])
    flags = g.update(mc.backproject_odom(depth, K_SMALL, R, c, STRIDE), c, 6.0)
    blobs = dof.conflict_blobs(flags, depth, STRIDE, 3)
    assert len(blobs) == 1
    u0, v0, u1, v1 = blobs[0]["bbox"]
    # Person spans y -0.6..-0.2 at 3.5 m: columns right of centre.
    assert u0 > K_SMALL[2] and u1 <= depth.shape[1]
    assert 3.4 < blobs[0]["depth_m"] < 3.9
