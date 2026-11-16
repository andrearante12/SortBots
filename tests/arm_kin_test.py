"""nodes/arm_kin.py: FK/IK over the XLeRobot arm URDF. Offline, numpy only.

Needs the third_party/XLeRobot submodule (it parses the URDF); skips without it.

    /usr/bin/python3 -m pytest tests/arm_kin_test.py
"""
from __future__ import annotations

import math
import random
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "nodes"))
sys.path.insert(0, str(REPO_ROOT))

import arm_kin  # noqa: E402
from simtest import magnet  # noqa: E402

pytestmark = pytest.mark.skipif(not arm_kin.URDF_PATH.exists(),
                                reason="third_party/XLeRobot submodule not checked out")


@pytest.fixture(scope="module")
def kin():
    return arm_kin.ArmKin()


def test_fk_at_zero(kin):
    t = kin.fk([0, 0, 0, 0, 0])
    assert t[:3, 3] == pytest.approx((0.248, -0.133, 0.856), abs=1e-3)
    pos, a = kin.tcp([0, 0, 0, 0, 0])
    assert pos == pytest.approx((0.2955, -0.133, 0.856), abs=1e-3)   # 47.1 mm beyond Fixed_Jaw
    # the URDF's rpy values are 1.5708, not pi/2, so a few micro-radians of slop
    assert a == pytest.approx((1, 0, 0), abs=1e-4)


def test_rotation_swings_the_arm_about_the_shoulder(kin):
    pos, a = kin.tcp([math.pi / 2, 0, 0, 0, 0])
    assert pos == pytest.approx((-0.090, -0.518, 0.856), abs=2e-3)
    assert a == pytest.approx((0, -1, 0), abs=1e-4)


def test_wrist_roll_changes_neither_tcp_nor_approach(kin):
    base = kin.tcp([0.3, 0.8, 1.2, -0.4, 0.0])
    for roll in (-2.0, 0.7, 3.0):
        pos, a = kin.tcp([0.3, 0.8, 1.2, -0.4, roll])
        assert pos == pytest.approx(base[0], abs=1e-9)
        assert a == pytest.approx(base[1], abs=1e-9)


def test_shoulder_matches_the_magnet_module_constant(kin):
    assert kin.shoulder() == pytest.approx(magnet.SHOULDER_XYZ_BASE, abs=1e-3)


def test_joint_limits_come_from_the_urdf(kin):
    # the same numbers spawn_warehouse.ARM_JOINT_LIMITS clamps commands to
    assert kin.lower.tolist() == pytest.approx([-2.1, -0.1, -0.2, -1.8, -3.14159])
    assert kin.upper.tolist() == pytest.approx([2.1, 3.45, 3.14159, 1.8, 3.14159])


def test_envelope_constants_match_the_joint_space(kin):
    """magnet.TCP_Z_RANGE_BASE and REACH_MAX_M are hand-copied from a sweep of the
    joint space; recompute them so a URDF bump can't silently invalidate the worlds."""
    rng = np.random.default_rng(0)
    zs, rs = [], []
    sh = kin.shoulder()
    for _ in range(4000):
        q = rng.uniform(kin.lower, kin.upper)
        pos, _ = kin.tcp(q)
        zs.append(pos[2])
        rs.append(math.hypot(pos[0] - sh[0], pos[1] - sh[1]))
    assert min(zs) == pytest.approx(magnet.TCP_Z_RANGE_BASE[0], abs=0.03)
    assert max(zs) == pytest.approx(magnet.TCP_Z_RANGE_BASE[1], abs=0.03)
    assert max(rs) < 0.42      # nothing reaches further than the arm's links sum to
    # and solve()'s cheap rejection band really does contain every reachable pose
    assert arm_kin.TCP_Z_BAND_BASE[0] <= min(zs) and max(zs) <= arm_kin.TCP_Z_BAND_BASE[1]
    for q in rng.uniform(kin.lower, kin.upper, size=(300, 5)):
        pos, _ = kin.tcp(q)
        assert np.linalg.norm(pos - sh) <= arm_kin.MAX_SHOULDER_DIST_M


def test_ik_round_trip_on_reachable_poses(kin):
    rng = random.Random(7)
    n = 0
    while n < 40:
        q = [rng.uniform(-1.4, 1.4), rng.uniform(0.2, 2.4), rng.uniform(0.3, 2.4),
             rng.uniform(-1.2, 1.2), 0.0]
        pos, a = kin.tcp(q)
        if abs(a[2]) > math.sin(math.radians(55)):
            continue        # near vertical the azimuth means nothing: the arm may fold either way
        if math.hypot(*(pos[:2] - kin.shoulder()[:2])) < 0.12:
            continue        # on the shoulder's own axis the same: no radial direction
        if (pos[:2] - kin.shoulder()[:2]) @ a[:2] < 0:
            continue        # pointing back at the shoulder: a folded branch picking never uses
        n += 1
        sol = kin.solve(pos, a)
        assert sol is not None, f"no solution for q={q}"
        assert sol["pos_err_m"] < arm_kin.POS_TOL_M
        assert sol["angle_deg"] < 8.0
        got, _ = kin.tcp(sol["q"])
        assert got == pytest.approx(pos, abs=arm_kin.POS_TOL_M)
        assert sol["q"][4] == 0.0 and sol["q"][5] == 0.0       # Wrist_Roll and Jaw stay put
        assert np.all(np.array(sol["q"][:5]) >= kin.lower - 1e-9)
        assert np.all(np.array(sol["q"][:5]) <= kin.upper + 1e-9)


@pytest.mark.parametrize("world_z", [0.36, 0.06, 0.45])
def test_targets_below_the_envelope_have_no_solution(kin, world_z):
    # a plate 0.18 m ahead of the base at floor/low-deck height (see pick_low/pick_floor)
    assert kin.solve((0.18, -0.133, world_z - magnet.BASE_Z_M)) is None


def test_targets_out_of_reach_have_no_solution(kin):
    assert kin.solve((0.9, -0.133, 0.8)) is None


def test_bench_height_plate_solves_with_a_horizontal_approach(kin):
    # the pick_bench plate in base_link once docked: 0.18 ahead of base, shoulder-level
    plate = (0.18, -0.133, 0.81)
    sol = kin.solve(plate)
    assert sol is not None and sol["angle_deg"] < 5.0


def _docked_plate(world_z):
    """A plate facing the robot, docked at nominal reach, at a height above the floor."""
    return ((magnet.SHOULDER_XYZ_BASE[0] + magnet.REACH_NOM_M, magnet.SHOULDER_XYZ_BASE[1],
             world_z - magnet.BASE_Z_M), (-1.0, 0.0, 0.0))


# plate centres of pick_heights' decks (0.70, 0.78, 0.90 + half a 0.12 m box) and pick_bench's
@pytest.mark.parametrize("world_z", [0.76, 0.84, 0.86, 0.96])
def test_plan_reach_for_every_pick_world_plate_height(kin, world_z):
    centre, normal = _docked_plate(world_z)
    plan = arm_kin.plan_reach(kin, centre, normal)
    assert plan is not None, f"plate at {world_z} m is out of reach at nominal dock"
    # the contact pose captures, and the pre-approach is a straight push back along the approach
    assert plan["angle_deg"] <= magnet.CAPTURE_DEG - arm_kin.ANGLE_MARGIN_DEG
    assert plan["final"]["pos_err_m"] < arm_kin.POS_TOL_M
    pre_tcp, a = kin.tcp(plan["pre"]["q"])
    final_tcp, _ = kin.tcp(plan["final"]["q"])
    back = np.array(final_tcp) - np.array(pre_tcp)
    assert np.linalg.norm(back) == pytest.approx(magnet.STANDOFF_M, abs=2 * arm_kin.POS_TOL_M)
    assert back / np.linalg.norm(back) == pytest.approx(a, abs=0.12)


def test_a_low_plate_is_reached_by_tilting_the_magnet_down(kin):
    # 0.72 m plate centre (a 0.66 m deck): below what a level approach reaches at
    # nominal dock, so it only works with the magnet pointing down onto the plate
    plan = arm_kin.plan_reach(kin, *_docked_plate(0.72))
    assert plan is not None and plan["elevation_deg"] < 0
    assert plan["angle_deg"] <= magnet.CAPTURE_DEG - arm_kin.ANGLE_MARGIN_DEG


@pytest.mark.parametrize("world_z", [0.36, 0.06])
def test_plan_reach_refuses_plates_below_the_envelope(kin, world_z):
    # pick_low and pick_floor
    assert arm_kin.plan_reach(kin, *_docked_plate(world_z)) is None


def test_plan_reach_refuses_a_plate_beyond_the_arm(kin):
    centre = (0.9, -0.133, 0.81)
    assert arm_kin.plan_reach(kin, centre, (-1.0, 0.0, 0.0)) is None


def test_seed_is_tried_first(kin):
    sol = kin.solve((0.2, -0.133, 0.8))
    again = kin.solve((0.2, -0.133, 0.8), q_seed=sol["q"])
    assert again["q"] == pytest.approx(sol["q"], abs=1e-3)


def test_missing_urdf_is_a_clear_error(tmp_path):
    with pytest.raises(arm_kin.ArmKinError, match="submodule"):
        arm_kin.ArmKin(tmp_path / "nope.urdf")
