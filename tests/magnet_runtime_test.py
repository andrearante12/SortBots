"""scripts/_magnet_runtime.py: the magnet inside Isaac, driven by a fake robot and a stub
`pxr`. Offline: no Isaac, no ROS.

This is the part of the pick stack that can only be exercised for real in a live sim, so
what it can be held to here is everything around the live part: the joint-name lookup
(Isaac's DOF order is not the URDF's), the FK that finds the magnet, what is written to
the package prims and when, what ground truth reports, and that the FK-vs-USD
cross-check says so when the two disagree.

    /usr/bin/python3 -m pytest tests/magnet_runtime_test.py
"""
from __future__ import annotations

import json
import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))
sys.path.insert(0, str(REPO_ROOT / "nodes"))
sys.path.insert(0, str(REPO_ROOT))

import arm_kin  # noqa: E402
from simtest import magnet, world  # noqa: E402

pytestmark = pytest.mark.skipif(not arm_kin.URDF_PATH.exists(),
                                reason="third_party/XLeRobot submodule not checked out")

ARM = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]
# Isaac interleaves the two arms and the head by joint type; this is deliberately NOT the
# URDF's order, and the arm joints are not contiguous (spawn_warehouse.py's comment).
DOF_NAMES = ["root_x_axis_joint", "root_y_axis_joint", "root_z_rotation_joint", "Rotation",
             "Rotation_2", "head_pan_joint", "Pitch", "Pitch_2", "head_tilt_joint", "Elbow",
             "Elbow_2", "Wrist_Pitch", "Wrist_Pitch_2", "Wrist_Roll", "Wrist_Roll_2", "Jaw",
             "Jaw_2"]
SPAWN = (-7.15, 11.62, 0.05)
SIZE = (0.24, 0.18, 0.12)


class FakeRobot:
    def __init__(self):
        self.dof_names = DOF_NAMES
        self.q = np.zeros(len(DOF_NAMES))

    def set(self, base_xy_yaw, arm):
        self.q[0], self.q[1], self.q[2] = base_xy_yaw[0] - SPAWN[0], base_xy_yaw[1] - SPAWN[1], base_xy_yaw[2]
        for name, v in zip(ARM, arm):
            self.q[DOF_NAMES.index(name)] = v

    def get_joint_positions(self):
        return self.q


class Vec(tuple):
    def __new__(cls, *a):
        return super().__new__(cls, [float(v) for v in a])

    def __truediv__(self, k):
        return Vec(*(v / k for v in self))

    def GetLength(self):
        return math.sqrt(sum(v * v for v in self))


class Op:
    def __init__(self):
        self.values = []

    def Set(self, v):
        self.values.append(v)


class FakeXform:
    """A USD link transform: p' = R p + t, with the methods the runtime calls."""

    def __init__(self, r, t):
        self.r, self.t = np.asarray(r, float), np.asarray(t, float)

    def Transform(self, v):
        return Vec(*(self.r @ np.array(v) + self.t))

    def TransformDir(self, v):
        return Vec(*(self.r @ np.array(v)))


class FakePrim:
    def __init__(self, xf):
        self.xf = xf

    def IsValid(self):
        return self.xf is not None


class FakeStage:
    def __init__(self, xf=None):
        self.xf = xf

    def GetPrimAtPath(self, path):
        return FakePrim(self.xf)


@pytest.fixture(autouse=True)
def fake_pxr(monkeypatch):
    holder = {"xf": None}
    gf = types.SimpleNamespace(Vec3d=Vec, Vec3f=Vec)
    usd = types.SimpleNamespace(TimeCode=types.SimpleNamespace(Default=lambda: 0))
    usdgeom = types.SimpleNamespace(Xformable=lambda prim: types.SimpleNamespace(
        ComputeLocalToWorldTransform=lambda t: prim.xf))
    monkeypatch.setitem(sys.modules, "pxr", types.SimpleNamespace(Gf=gf, Usd=usd, UsdGeom=usdgeom))
    return holder


@pytest.fixture(scope="module")
def kin():
    return arm_kin.ArmKin()


def make_world(pkg_pose, *, z=0.8, face="+x", floating=False):
    """A world with one deck and one package. `floating` skips the support check: the
    tests that need a package already in the magnet's grip put it mid-air, which no real
    world may (simtest.world.support_for)."""
    from unittest import mock

    raw = {"name": "t",
           "props": [{"id": "deck", "shape": "box", "size": [1.0, 1.0, 0.8], "pose": [-7.9, 10.0, 0.0]}],
           "packages": [{"id": "pkg_0", "pose": list(pkg_pose), "z": z, "magnet_face": face}]}
    if floating:
        with mock.patch.object(world, "support_for", lambda *a, **k: None):
            return world.validate(raw)
    return world.validate(raw)


def build(w, *, stage=None, emit=None):
    import _magnet_runtime as mr

    ops = {"pkg_0": (Op(), Op())}
    pick = {"data": "", "seq": 0}
    log = [] if emit is None else emit
    rt = mr.MagnetRuntime(stage or FakeStage(), w, ops, pick, "robot_0", SPAWN,
                          emit=log.append, arm_link="Fixed_Jaw", arm_joints=ARM)
    return rt, ops, pick, log


def usd_xform_for(kin, base, arm):
    """What the arm link's USD world transform would be if USD agreed with the physics."""
    t = kin.fk(arm[:5])
    c, s = math.cos(base[2]), math.sin(base[2])
    rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return FakeXform(rz @ t[:3, :3], rz @ t[:3, 3] + np.array([base[0], base[1], magnet.BASE_Z_M]))


# -- where the magnet is -------------------------------------------------------------------

def test_tcp_at_the_booted_pose_is_ahead_of_and_right_of_the_base(kin):
    rt, *_ = build(make_world((-7.9, 9.9, 1.5708)))
    robot = FakeRobot()
    tcp, a = rt.tcp_pose(robot)                    # all joints 0: base at the spawn, arm straight out
    assert tcp == pytest.approx((SPAWN[0] + 0.2955, SPAWN[1] - 0.133, 0.05 + 0.856), abs=2e-3)
    assert a == pytest.approx((1, 0, 0), abs=1e-4)


def test_tcp_follows_the_base_pose_and_the_named_arm_joints(kin):
    rt, *_ = build(make_world((-7.9, 9.9, 1.5708)))
    robot = FakeRobot()
    arm = [0.3, 0.9, 1.2, -0.4, 0.5, 0.0]
    robot.set((-5.0, 8.0, 1.2), arm)
    tcp, a = rt.tcp_pose(robot)
    pos_b, a_b = kin.tcp(arm[:5])
    want = magnet.to_map((-5.0, 8.0, 1.2), tuple(pos_b))
    assert tcp == pytest.approx(want) and a == pytest.approx(magnet.dir_to_map((-5.0, 8.0, 1.2), tuple(a_b)))
    # the SECOND arm's joints, and the head's, are in the same vector and must not matter
    robot.q[DOF_NAMES.index("Pitch_2")] = 2.0
    robot.q[DOF_NAMES.index("head_pan_joint")] = 1.0
    assert rt.tcp_pose(robot)[0] == pytest.approx(want)


# -- capture, carry, release -------------------------------------------------------------------

def captured_setup(kin):
    """A runtime whose magnet is exactly on a package's plate, and the robot that puts it there."""
    arm = [0.0, 0.5, 1.0, 0.0, 0.0, 0.0]
    base = (-6.0, 9.0, 0.4)
    robot = FakeRobot()
    robot.set(base, arm)
    rt0, *_ = build(make_world((-7.9, 9.9, 1.5708)))
    tcp, a = rt0.tcp_pose(robot)
    held = magnet.held_pose(tcp, a, SIZE, "+x", 0.0)          # the box exactly as the magnet would hold it
    w = make_world((held["x"], held["y"], held["yaw"]), z=held["z"], floating=True)
    return w, robot, base, arm


def test_energized_magnet_on_the_plate_captures_and_the_prim_follows(kin):
    w, robot, base, arm = captured_setup(kin)
    rt, ops, pick, log = build(w)
    rt.step({"robot_0": (robot,)})
    assert rt.sim.held is None and ops["pkg_0"][0].values == []           # off: nothing happens
    pick["data"] = "attach"
    rt.step({"robot_0": (robot,)})
    assert rt.sim.held == "pkg_0"
    assert any("captured pkg_0" in l for l in log)
    t_op, r_op = ops["pkg_0"]
    assert len(t_op.values) == 1 and len(r_op.values) == 1
    x, y, z = t_op.values[-1]
    p = rt.sim.pose("pkg_0")
    assert (x, y, z) == pytest.approx((p["x"], p["y"], p["z"]))
    assert r_op.values[-1][2] == pytest.approx(math.degrees(p["yaw"]))
    # the arm moves: the package rides it, a write every step
    robot.set((-6.0, 9.0, 0.4), [0.3, 0.6, 1.0, 0.0, 0.0, 0.0])
    rt.step({"robot_0": (robot,)})
    assert len(t_op.values) == 2 and t_op.values[-1] != t_op.values[0]


def test_detach_releases_it_onto_the_floor_or_deck_and_logs_it(kin):
    w, robot, base, arm = captured_setup(kin)
    rt, ops, pick, log = build(w)
    pick["data"] = "attach"
    rt.step({"robot_0": (robot,)})
    pick["data"] = "detach"
    rt.step({"robot_0": (robot,)})
    assert rt.sim.held is None
    assert any("released pkg_0" in l for l in log)
    # carried over open floor (the deck is at x -8.4..-7.4): it falls to z = 0
    assert ops["pkg_0"][0].values[-1][2] == pytest.approx(0.0)
    n = len(ops["pkg_0"][0].values)
    rt.step({"robot_0": (robot,)})
    assert len(ops["pkg_0"][0].values) == n                # and then stays put, no more writes


def test_a_magnet_that_is_off_never_moves_anything(kin):
    rt, ops, pick, _ = build(make_world((-7.9, 9.9, 1.5708)))
    robot = FakeRobot()
    for _ in range(5):
        robot.set((-7.9, 11.3, -1.5708), [0.0, 0.5, 1.0, 0.0, 0.0, 0.0])
        rt.step({"robot_0": (robot,)})
    assert ops["pkg_0"][0].values == [] and rt.sim.held is None


# -- ground truth --------------------------------------------------------------------------------

def test_truth_reports_packages_and_the_magnet_as_json(kin):
    w, robot, *_ = captured_setup(kin)
    rt, ops, pick, _ = build(w)
    rt.step({"robot_0": (robot,)})
    packages, mag = rt.truth()
    assert packages["pkg_0"]["held_by"] is None
    assert set(packages["pkg_0"]) == {"x", "y", "z", "yaw", "held_by"}
    assert mag["robot_0"]["energized"] is False and mag["robot_0"]["held"] is None
    assert mag["robot_0"]["nearest"]["id"] == "pkg_0" and mag["robot_0"]["nearest"]["dist_m"] < 0.01
    pick["data"] = "attach"
    rt.step({"robot_0": (robot,)})
    packages, mag = rt.truth()
    assert packages["pkg_0"]["held_by"] == "robot_0"
    assert mag["robot_0"]["energized"] is True and mag["robot_0"]["held"] == "pkg_0"
    assert len(mag["robot_0"]["tcp"]) == 3 and len(mag["robot_0"]["approach"]) == 3
    json.dumps({"packages": packages, "magnet": mag})


def test_truth_before_the_first_step_has_no_tcp_yet(kin):
    rt, *_ = build(make_world((-7.9, 9.9, 1.5708)))
    packages, mag = rt.truth()
    assert mag["robot_0"]["tcp"] is None and packages["pkg_0"]["held_by"] is None


# -- the FK-vs-USD cross-check ---------------------------------------------------------------------

def test_cross_check_says_so_when_fk_and_usd_agree(kin):
    base, arm = (-6.0, 9.0, 0.4), [0.2, 0.7, 1.1, -0.2, 0.0, 0.0]
    robot = FakeRobot()
    robot.set(base, arm)
    rt, _, _, log = build(make_world((-7.9, 9.9, 1.5708)), stage=FakeStage(usd_xform_for(kin, base, arm)))
    rt.step({"robot_0": (robot,)})
    (line,) = [l for l in log if "magnet tcp:" in l]
    assert "0.0 cm apart" in line and "DISAGREE" not in line


def test_cross_check_shouts_when_usd_is_somewhere_else(kin):
    base, arm = (-6.0, 9.0, 0.4), [0.2, 0.7, 1.1, -0.2, 0.0, 0.0]
    robot = FakeRobot()
    robot.set(base, arm)
    wrong = usd_xform_for(kin, base, arm)
    wrong.t = wrong.t + np.array([0.0, 0.0, 0.05])             # 5 cm high, e.g. a stale USD stage
    rt, _, _, log = build(make_world((-7.9, 9.9, 1.5708)), stage=FakeStage(wrong))
    rt.step({"robot_0": (robot,)})
    (line,) = [l for l in log if "magnet tcp:" in l]
    assert "DISAGREE" in line and "5.0 cm apart" in line


def test_cross_check_repeats_but_stops_nagging(kin):
    base, arm = (-6.0, 9.0, 0.4), [0.2, 0.7, 1.1, -0.2, 0.0, 0.0]
    robot = FakeRobot()
    robot.set(base, arm)
    wrong = usd_xform_for(kin, base, arm)
    wrong.t = wrong.t + np.array([0.3, 0.0, 0.0])
    rt, _, _, log = build(make_world((-7.9, 9.9, 1.5708)), stage=FakeStage(wrong))
    for _ in range(rt.CROSSCHECK_EVERY * (rt.CROSSCHECK_WARNINGS + 4) + 1):
        rt.step({"robot_0": (robot,)})
    assert len([l for l in log if "DISAGREE" in l]) == rt.CROSSCHECK_WARNINGS


def test_cross_check_is_skipped_once_when_the_prim_is_missing(kin):
    robot = FakeRobot()
    rt, _, _, log = build(make_world((-7.9, 9.9, 1.5708)), stage=FakeStage(None))
    for _ in range(rt.CROSSCHECK_EVERY + 2):
        rt.step({"robot_0": (robot,)})
    assert len([l for l in log if "skipped" in l]) == 1          # said once, at step 0
    assert rt.sim.held is None                                   # and the magnet still works without it


def test_a_robot_without_the_arm_joints_is_a_clear_error(kin):
    rt, *_ = build(make_world((-7.9, 9.9, 1.5708)))
    robot = FakeRobot()
    robot.dof_names = [n for n in DOF_NAMES if n != "Elbow"]
    robot.q = np.zeros(len(robot.dof_names))
    with pytest.raises(ValueError):
        rt.step({"robot_0": (robot,)})
