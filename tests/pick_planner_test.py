"""nodes/pick_planner.py in a closed loop: the planner, the real IK (nodes/arm_kin.py),
the real magnet logic (simtest/magnet.py) and a fake camera, base and arm, ticked at the
10 Hz the ground truth arrives at. Offline: no ROS, no Isaac.

The fake base integrates cmd_vel and KEEPS the last command until a new one arrives,
exactly as the sim does, so a planner that forgets to send a final zero drives off. The
fake arm follows its commands instantly (the real one lags a little; the planner's
settle dwell covers that). The fake camera sees a plate only from the side it faces,
inside the head camera's field of view and range, with a centimetre of noise.

    /usr/bin/python3 -m pytest tests/pick_planner_test.py
"""
from __future__ import annotations

import math
import random
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "nodes"))
sys.path.insert(0, str(REPO_ROOT))

import arm_kin  # noqa: E402
import pick_planner as pp  # noqa: E402
from simtest import geometry, magnet, world  # noqa: E402

pytestmark = pytest.mark.skipif(not arm_kin.URDF_PATH.exists(),
                                reason="third_party/XLeRobot submodule not checked out")

DT = 0.1                      # ground truth is 10 Hz sim time
SPAWN = (-7.15, 11.62, 0.0)
CAM_Z = 1.25                  # head camera height above the floor, roughly
IMG = (848, 480)


@pytest.fixture(scope="module")
def kin():
    return arm_kin.ArmKin()


class FakeWorld:
    def __init__(self, kin, world_name, base=SPAWN, *, sees=True, noise=True, tcp_error=(0, 0, 0),
                 seed=1):
        self.kin = kin
        self.w = world.load(world_name)
        self.base = list(base)
        self.vel = (0.0, 0.0, 0.0)
        self.cmd = ""
        self.q = [0.0] * 6
        self.now = 100.0
        self.sees, self.noise, self.tcp_error = sees, noise, tcp_error
        self.rng = random.Random(seed)
        self.min_gap = 9.0
        pk = {p["id"]: {"pose": {"x": p["pose"][0], "y": p["pose"][1], "z": p["z"],
                                 "yaw": p["pose"][2]}, "size": p["size"], "face": p["magnet_face"]}
              for p in self.w["packages"]}
        self.msim = magnet.MagnetSim(pk, lambda x, y: magnet.support_top_z(self.w, x, y))
        self.spec = {p["id"]: p for p in self.w["packages"]}
        self.log = []

    # -- what the planner is shown ------------------------------------------------------
    def tcp(self):
        pos, a = self.kin.tcp(self.q[:5])
        pos = [pos[i] + self.tcp_error[i] for i in range(3)]
        return magnet.to_map(tuple(self.base), pos), magnet.dir_to_map(tuple(self.base), a)

    def plates(self):
        out = []
        if not self.sees:
            return out
        for pid, spec in self.spec.items():
            if self.msim.held == pid:
                continue
            pose = self.msim.pose(pid)
            c, n = magnet.magnet_face_pose(pose, spec["size"], spec["magnet_face"])
            dx, dy = c[0] - self.base[0], c[1] - self.base[1]
            dist = math.hypot(dx, dy)
            facing = -(n[0] * dx + n[1] * dy) / max(dist, 1e-6)          # plate looks at us
            off = abs(math.atan2(math.sin(math.atan2(dy, dx) - self.base[2]),
                                 math.cos(math.atan2(dy, dx) - self.base[2])))
            elev = math.degrees(math.atan2(c[2] - CAM_Z, max(dist, 1e-6)))
            if not (0.8 <= dist <= 3.5 and off <= math.radians(40) and facing >= 0.3
                    and abs(elev) <= 26):
                continue
            j = (lambda s: self.rng.gauss(0.0, s)) if self.noise else (lambda s: 0.0)
            nn = (n[0] + j(0.03), n[1] + j(0.03), n[2] + j(0.03))
            nl = math.sqrt(sum(v * v for v in nn))
            out.append({"x": c[0] + j(0.01), "y": c[1] + j(0.01), "z": c[2] + j(0.01),
                        "nx": nn[0] / nl, "ny": nn[1] / nl, "nz": nn[2] / nl, "conf": 0.9,
                        "age_s": 0.0, "px": [300, 200, 60, 40]})
        return out

    def obs(self):
        return pp.Obs(now=self.now, base=tuple(self.base), plates=self.plates(),
                      held=self.msim.held is not None)

    # -- what the planner does -------------------------------------------------------------
    def apply(self, d):
        if d.vel is not None:
            self.vel = d.vel
        if d.pick_cmd is not None:
            self.cmd = d.pick_cmd
        if d.q is not None:
            self.q = list(d.q)
        c, s = math.cos(self.base[2]), math.sin(self.base[2])
        vx, vy, wz = self.vel
        self.base[0] += (c * vx - s * vy) * DT
        self.base[1] += (s * vx + c * vy) * DT
        self.base[2] += wz * DT
        tcp, a = self.tcp()
        self.msim.step(tcp, a, self.cmd == "attach")
        self.now += DT
        hx, hy = magnet.CHASSIS_HALF_M
        chassis = geometry.rect(self.base[0], self.base[1], self.base[2], 2 * hx, 2 * hy)
        for prop in self.w["props"]:
            self.min_gap = min(self.min_gap, geometry.distance(chassis, world.prop_footprint(prop)))

    def run(self, planner, max_s=400.0):
        t_end = self.now + max_s
        while self.now < t_end:
            d = planner.tick(self.obs())
            self.log.append((round(self.now, 1), d.phase))
            self.apply(d)
            if d.result:
                return d
        return None

    def package_pose(self, pid="pkg_0"):
        return self.msim.pose(pid)


def cargo(w, pid):
    p = next(p for p in w["packages"] if p["id"] == pid)
    return {"package": pid, "size": list(p["size"]), "face": p["magnet_face"],
            "at": {"x": p["pose"][0], "y": p["pose"][1]}}


def pick_req(w, pid):
    return {"cmd": "attach", "task_id": "t", **cargo(w, pid)}


def place_req(w, pid, shift):
    p = next(p for p in w["packages"] if p["id"] == pid)
    x, y, yaw = p["pose"]
    return {"cmd": "detach", "task_id": "t", **cargo(w, pid),
            "place": {"x": x + shift[0], "y": y + shift[1], "z": p["z"], "yaw": yaw}}


def do(fw, planner, req):
    planner.request(req, fw.now)
    return fw.run(planner)


# -- a whole pick and place ----------------------------------------------------------------

def test_pick_and_place_on_the_bench(kin):
    fw = FakeWorld(kin, "pick_bench")
    planner = pp.PickPlanner(kin)
    d = do(fw, planner, pick_req(fw.w, "pkg_0"))
    assert d is not None and d.result == "picked", d
    assert fw.msim.held == "pkg_0"
    # the base was told to stop: hands off now must not leave it rolling
    assert fw.vel == (0.0, 0.0, 0.0)
    d = do(fw, planner, place_req(fw.w, "pkg_0", (-0.12, 0.0)))
    assert d is not None and d.result == "placed", d
    assert fw.msim.held is None
    pose = fw.package_pose()
    assert pose["x"] == pytest.approx(-8.02, abs=0.04) and pose["y"] == pytest.approx(9.9, abs=0.04)
    assert pose["z"] == pytest.approx(0.8, abs=1e-6)                 # on the bench, not through it
    assert fw.cmd == "detach" and fw.vel == (0.0, 0.0, 0.0)
    assert fw.min_gap > 0.0, "the chassis drove into the bench"


def test_the_arm_is_tucked_before_the_base_moves(kin):
    fw = FakeWorld(kin, "pick_bench")
    planner = pp.PickPlanner(kin)
    planner.request(pick_req(fw.w, "pkg_0"), fw.now)
    moved_with_arm_out = False
    for _ in range(300):
        d = planner.tick(fw.obs())
        fw.apply(d)
        if d.vel and any(abs(v) > 1e-6 for v in d.vel[:2]):
            tip, _ = kin.tcp(fw.q[:5])
            moved_with_arm_out |= tip[0] > 0.1                       # the tcp ahead of the base
        if planner.phase == "dock":
            break
    assert not moved_with_arm_out
    assert fw.q == pytest.approx(pp.DEFAULTS["tuck_q"], abs=1e-6)


def test_the_planner_turns_to_find_a_plate_behind_it(kin):
    # starts facing away from the bench; the work order's location is all it has
    fw = FakeWorld(kin, "pick_bench", base=(-7.15, 11.62, 2.0))
    d = do(fw, pp.PickPlanner(kin), pick_req(fw.w, "pkg_0"))
    assert d is not None and d.result == "picked"
    assert fw.msim.held == "pkg_0"


def test_a_plate_facing_away_is_found_by_walking_round_the_box(kin):
    # south of the bench looking at the BACK of the box: the plate faces north
    fw = FakeWorld(kin, "pick_bench", base=(-7.9, 8.3, math.pi / 2))
    d = do(fw, pp.PickPlanner(kin), pick_req(fw.w, "pkg_0"))
    assert d is not None and d.result == "picked", (d, fw.log[-3:])
    assert any(p == "acquire" for _, p in fw.log)
    assert fw.msim.held == "pkg_0"


@pytest.mark.parametrize("name,order", [
    ("pick_orientations", ["pkg_0", "pkg_1", "pkg_2", "pkg_3"]),
    ("pick_heights", ["pkg_0", "pkg_1", "pkg_2"]),
])
def test_every_package_in_a_pick_world_in_order(kin, name, order):
    fw = FakeWorld(kin, name)
    planner = pp.PickPlanner(kin)
    for pid in order:
        shift = {"pkg_0": (-0.12, 0.0), "pkg_1": (0.0, 0.12), "pkg_2": (0.0, -0.12),
                 "pkg_3": (0.12, 0.0)}[pid] if name == "pick_orientations" else (-0.12, 0.0)
        assert do(fw, planner, pick_req(fw.w, pid)).result == "picked", (pid, fw.log[-4:])
        d = do(fw, planner, place_req(fw.w, pid, shift))
        assert d.result == "placed", (pid, d.note)
        pose, spec = fw.package_pose(pid), fw.spec[pid]
        want = (spec["pose"][0] + shift[0], spec["pose"][1] + shift[1])
        assert math.hypot(pose["x"] - want[0], pose["y"] - want[1]) < 0.06, pid
        assert pose["z"] == pytest.approx(spec["z"], abs=1e-6), pid
    assert fw.min_gap > 0.0, "the chassis drove into a deck"


def test_it_works_with_noiseless_and_with_biased_vision_alike(kin):
    for noise in (False, True):
        fw = FakeWorld(kin, "pick_bench", noise=noise, seed=5)
        assert do(fw, pp.PickPlanner(kin), pick_req(fw.w, "pkg_0")).result == "picked"


# -- refusing cleanly -----------------------------------------------------------------------

@pytest.mark.parametrize("name", ["pick_low", "pick_floor"])
def test_a_plate_below_the_arm_is_refused_without_wasting_a_move(kin, name):
    fw = FakeWorld(kin, name, base=(-7.9, 11.5, -math.pi / 2))   # looking straight at it
    # the camera cannot see a plate that low from near: hand it over so the planner has a target
    fw.sees = False
    spec = fw.spec["pkg_0"]
    c, n = magnet.magnet_face_pose(fw.msim.pose("pkg_0"), spec["size"], spec["magnet_face"])
    plate = {"x": c[0], "y": c[1], "z": c[2], "nx": n[0], "ny": n[1], "nz": n[2], "conf": 1.0,
             "age_s": 0.0}
    planner = pp.PickPlanner(kin)
    planner.request(pick_req(fw.w, "pkg_0"), fw.now)
    while planner.phase in ("tuck", "idle") and fw.now < 120:
        fw.apply(planner.tick(fw.obs()))
    t0, base0 = fw.now, list(fw.base)
    result = None
    for _ in range(200):
        o = fw.obs()
        o.plates = [plate]
        d = planner.tick(o)
        fw.apply(d)
        if d.result:
            result = d
            break
    assert result is not None and result.result == "failed" and "out of reach" in result.note
    assert fw.now - t0 < 5.0                              # decided at once
    assert math.dist(fw.base[:2], base0[:2]) < 0.05       # without driving anywhere
    assert fw.cmd == "detach" and fw.msim.held is None
    for _ in range(60):                                   # the tuck it started still plays out
        fw.apply(planner.tick(fw.obs()))
    assert fw.q == pytest.approx(pp.DEFAULTS["tuck_q"], abs=1e-3)


def test_no_plate_in_sight_fails_after_the_search(kin):
    fw = FakeWorld(kin, "pick_bench", sees=False)
    planner = pp.PickPlanner(kin, {"acquire": {"timeout_s": 40.0}})
    d = do(fw, planner, pick_req(fw.w, "pkg_0"))
    assert d.result == "failed" and "plate not found" in d.note
    assert fw.vel == (0.0, 0.0, 0.0) and fw.cmd == "detach"


def test_a_magnet_that_misses_is_reported_not_assumed(kin):
    fw = FakeWorld(kin, "pick_bench", tcp_error=(0.0, 0.0, 0.08))   # the arm lands 8 cm high
    d = do(fw, pp.PickPlanner(kin), pick_req(fw.w, "pkg_0"))
    assert d.result == "failed" and "magnet missed" in d.note
    assert fw.msim.held is None and fw.cmd == "detach"


def test_asked_to_place_with_nothing_held_fails(kin):
    fw = FakeWorld(kin, "pick_bench")
    d = do(fw, pp.PickPlanner(kin), place_req(fw.w, "pkg_0", (-0.12, 0.0)))
    assert d.result == "failed" and "nothing held" in d.note


def test_the_budget_is_a_hard_stop(kin):
    fw = FakeWorld(kin, "pick_bench")
    d = do(fw, pp.PickPlanner(kin, {"budget_s": 6.0}), pick_req(fw.w, "pkg_0"))
    assert d.result == "failed" and "budget" in d.note and fw.cmd == "detach"


def test_cancel_stops_the_base_turns_the_magnet_off_and_tucks(kin):
    fw = FakeWorld(kin, "pick_bench")
    planner = pp.PickPlanner(kin)
    planner.request(pick_req(fw.w, "pkg_0"), fw.now)
    for _ in range(80):
        fw.apply(planner.tick(fw.obs()))
    d = planner.cancel(fw.now)
    fw.apply(d)
    assert d.vel == (0.0, 0.0, 0.0) and d.pick_cmd == "detach" and not planner.busy
    for _ in range(80):
        fw.apply(planner.tick(fw.obs()))                  # the tuck ramp still plays out
    assert fw.q == pytest.approx(pp.DEFAULTS["tuck_q"], abs=1e-3)
    assert fw.vel == (0.0, 0.0, 0.0)


# -- the pieces -------------------------------------------------------------------------------

def test_drive_commands_are_body_frame_and_stop_on_arrival(kin):
    p = pp.PickPlanner(kin)
    # goal 1 m along world +x, robot facing +y: that is the robot's RIGHT, a -y body velocity
    vel, ok = p._drive((0.0, 0.0, math.pi / 2), (1.0, 0.0), math.pi / 2, v_max=0.3,
                       pos_tol=0.02, yaw_tol=0.05)
    assert not ok and vel[0] == pytest.approx(0.0, abs=1e-9) and vel[1] == pytest.approx(-0.3)
    vel, ok = p._drive((0.0, 0.0, 0.0), (0.01, 0.0), 0.0, v_max=0.3, pos_tol=0.02, yaw_tol=0.05)
    assert ok
    vel, _ = p._drive((0.0, 0.0, 0.0), (0.0, 0.0), 1.0, v_max=0.3, pos_tol=0.02, yaw_tol=0.05)
    assert vel[2] == pytest.approx(p.cfg["dock"]["wz_max"])          # turns the short way, capped


def test_arm_ramps_are_smooth_and_land_exactly(kin):
    p = pp.PickPlanner(kin)
    p._arm_to([0.0, 1.0, 1.0, 0.0, 0.0, 0.0], 10.0)                   # 1 rad at 1 rad/s: 1.0 s
    assert p.arm_end == pytest.approx(11.0)
    qs = [p._ramp_q(10.0 + t * 0.1)[1] for t in range(0, 11)]
    assert qs[0] == pytest.approx(0.0)
    assert all(b >= a for a, b in zip(qs, qs[1:]))                    # monotone, no overshoot
    assert qs[-1] == pytest.approx(1.0) and p.ramp is None
    assert p._arm_done(11.0) and not p._arm_ready(11.2) and p._arm_ready(11.5)
    assert p._ramp_q(99.0) is None                                    # nothing more to send


def test_config_overrides_merge_by_key():
    c = pp.merged({"stage_m": 0.5, "dock": {"v_max": 0.1}})
    assert c["stage_m"] == 0.5 and c["dock"]["v_max"] == 0.1
    assert c["dock"]["kp"] == pp.DEFAULTS["dock"]["kp"] and pp.DEFAULTS["dock"]["v_max"] == 0.30


def test_a_new_request_replaces_the_old_one(kin):
    fw = FakeWorld(kin, "pick_bench")
    planner = pp.PickPlanner(kin)
    planner.request(pick_req(fw.w, "pkg_0"), fw.now)
    planner.request(place_req(fw.w, "pkg_0", (-0.12, 0.0)), fw.now)
    assert planner.mode == "detach" and planner.phase == "tuck"
