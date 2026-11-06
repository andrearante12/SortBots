"""nodes/box_geometry.py: boxes on a table from a depth image. Offline, numpy + cv2.

Frames come from a tiny ray caster over axis-aligned boxes in the map frame, seen by the real
camera's intrinsics from pick_table's start pose, so true sizes and positions are known.

    /usr/bin/python3 -m pytest tests/box_geometry_test.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "nodes"))

pytest.importorskip("cv2")
import box_geometry as bg  # noqa: E402

W, H = 848, 480
K = (605.0, 605.0, 424.0, 240.0)
TABLE = ((0.60, -0.45, 0.0), (0.96, 0.45, 0.95))          # (min corner, max corner)
FLOOR = ((-5.0, -5.0, -0.1), (5.0, 5.0, 0.0))
RACK = ((1.08, -3.0, 0.0), (1.6, 3.0, 2.2))               # taller than the table, right behind it


def camera(tilt=0.5, at=(-0.09, 0.02, 1.24)):
    """map <- optical, looking along +x and tilted down by `tilt`."""
    fwd = np.array([math.cos(tilt), 0.0, -math.sin(tilt)])
    right = np.array([0.0, -1.0, 0.0])
    down = np.cross(fwd, right)
    t = np.eye(4)
    t[:3, 0], t[:3, 1], t[:3, 2], t[:3, 3] = right, down, fwd, at
    return t


def render(boxes, cam):
    """Depth along the optical axis, nearest hit per pixel."""
    fx, fy, cx, cy = K
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    ray_c = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us, dtype=float)], axis=-1)
    ray = ray_c @ cam[:3, :3].T
    o = cam[:3, 3]
    depth = np.full((H, W), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for lo, hi in boxes:
            t1, t2 = (np.array(lo) - o) / ray, (np.array(hi) - o) / ray
            near, far = np.minimum(t1, t2).max(axis=-1), np.maximum(t1, t2).min(axis=-1)
            hit = (near <= far) & (near > 0)
            depth = np.where(hit & (near < depth), near, depth)
    return depth.astype(np.float32)


def block(cx, cy, sx=0.14, sy=0.14, h=0.10, z=0.95):
    return ((cx - sx / 2, cy - sy / 2, z), (cx + sx / 2, cy + sy / 2, z + h))


SCENE = [FLOOR, TABLE, RACK]


def test_a_block_on_the_table_is_found_with_its_position_and_size():
    (b,) = bg.find_boxes(render(SCENE + [block(0.67, -0.13)], camera()), K, camera())
    assert (b["x"], b["y"]) == pytest.approx((0.67, -0.13), abs=0.015)
    assert b["z"] == pytest.approx(1.0, abs=0.01)
    assert b["size"] == pytest.approx([0.14, 0.14, 0.10], abs=0.02)
    assert b["support_z"] == pytest.approx(0.95, abs=0.005)


def test_an_empty_table_has_no_boxes_and_the_rack_behind_it_is_not_one():
    assert bg.find_boxes(render(SCENE, camera()), K, camera()) == []


def test_two_blocks_are_two_boxes():
    found = bg.find_boxes(render(SCENE + [block(0.72, 0.2), block(0.72, -0.2)], camera()), K, camera())
    assert sorted(round(b["y"], 1) for b in found) == [-0.2, 0.2]


def test_a_longer_box_keeps_its_long_side():
    (b,) = bg.find_boxes(render(SCENE + [block(0.72, 0.0, sx=0.14, sy=0.30)], camera()), K, camera())
    assert b["size"][:2] == pytest.approx([0.30, 0.14], abs=0.02)


def test_it_reports_where_the_box_is_in_the_picture():
    (b,) = bg.find_boxes(render(SCENE + [block(0.67, -0.13)], camera()), K, camera())
    u, v, w, h = b["px"]
    assert 0 <= u and u + w < W and 0 <= v and v + h < H and w > 30 and h > 30


def test_it_works_with_the_head_level_and_from_further_back():
    cam = camera(tilt=0.3, at=(-0.5, 0.02, 1.24))
    (b,) = bg.find_boxes(render(SCENE + [block(0.72, 0.1)], cam), K, cam)
    assert (b["x"], b["y"]) == pytest.approx((0.72, 0.1), abs=0.02)


def test_depth_noise_and_holes_do_not_lose_it():
    rng = np.random.default_rng(3)
    depth = render(SCENE + [block(0.67, -0.13)], camera())
    depth = depth + rng.normal(0.0, 0.003, depth.shape).astype(np.float32)
    depth[rng.random(depth.shape) < 0.1] = np.nan
    (b,) = bg.find_boxes(depth, K, camera())
    assert (b["x"], b["y"]) == pytest.approx((0.67, -0.13), abs=0.02)


def test_nothing_flat_in_view_is_an_empty_answer():
    assert bg.find_boxes(np.full((H, W), np.nan, np.float32), K, camera()) == []
    assert bg.find_boxes(render([FLOOR], camera(tilt=1.0)), K, camera(tilt=1.0)) == []
