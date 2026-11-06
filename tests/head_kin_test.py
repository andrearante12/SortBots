"""simtest/head.py: the camera's pose as the head moves. Offline, numpy.

    /usr/bin/python3 -m pytest tests/head_kin_test.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from simtest import head, world  # noqa: E402

if not head.URDF_PATH.exists():
    pytest.skip("third_party/XLeRobot is not checked out", allow_module_level=True)

# What Isaac logged live for robot_0 (`base->optical`, 2026-11-05).
ISAAC_ZERO_T = (-0.09199909, 0.01999750, 1.19065028)


@pytest.fixture(scope="module")
def kin():
    return head.HeadKin()


def _zero(kin):
    m = kin.matrix(0.0, 0.0)
    return tuple(m[:3, 3]), head.matrix_to_quat(m[:3, :3])


def _forward(q):
    return head.quat_to_matrix(q) @ np.array([0.0, 0.0, 1.0])     # optical z


def test_the_urdf_chain_lands_where_isaac_put_the_camera(kin):
    assert kin.matrix(0.0, 0.0)[:3, 3] == pytest.approx(ISAAC_ZERO_T, abs=2e-3)


def test_at_zero_the_camera_looks_level_and_forward(kin):
    assert _forward(_zero(kin)[1]) == pytest.approx((1.0, 0.0, 0.0), abs=1e-3)


def test_positive_tilt_looks_down_by_that_angle(kin):
    _, q = kin.extrinsic(0.0, 0.5, _zero(kin))
    assert _forward(q) == pytest.approx((math.cos(0.5), 0.0, -math.sin(0.5)), abs=1e-3)


def test_positive_pan_looks_left(kin):
    _, q = kin.extrinsic(0.4, 0.0, _zero(kin))
    assert _forward(q) == pytest.approx((math.cos(0.4), math.sin(0.4), 0.0), abs=1e-3)


def test_tilting_swings_the_lens_about_the_tilt_axis_not_in_place(kin):
    t0, _ = _zero(kin)
    t1, _ = kin.extrinsic(0.0, 0.5, _zero(kin))
    assert 0.01 < math.dist(t0, t1) < 0.06          # the lens sits a few cm off the axis


def test_at_zero_the_measured_pose_comes_straight_back(kin):
    zero = ((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0))
    t, q = kin.extrinsic(0.0, 0.0, zero)
    assert t == pytest.approx(zero[0]) and q == pytest.approx(zero[1])


@pytest.mark.parametrize("q", [(0, 0, 0, 1), (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0),
                               (0.5, -0.5, 0.5, 0.5), (-0.5, 0.5, -0.5, 0.5)])
def test_quaternions_round_trip(q):
    back = head.matrix_to_quat(head.quat_to_matrix(q))
    assert head.quat_to_matrix(back) == pytest.approx(head.quat_to_matrix(q))


def test_pick_tables_head_is_inside_the_joint_limits():
    w = world.load("pick_table")
    assert w["head"]["tilt"] > 0 and w["view"]["eye"][2] > w["view"]["target"][2]


@pytest.mark.parametrize("raw,err", [
    ({"head": {"tilt": 2.0}}, "head.tilt"),
    ({"head": {"roll": 0.1}}, "head:"),
    ({"view": {"eye": [0, 0, 1]}}, "view:"),
    ({"view": {"eye": [0, 0], "target": [0, 0, 0]}}, "view.eye"),
])
def test_world_head_and_view_validation(raw, err):
    with pytest.raises(world.WorldError, match=err):
        world.validate({"name": "t", **raw})
