"""scripts/pick_core.py: the one-button pick extracted from scripts/handeye.py cmd_pick.

The golden tests boot handeye's own cmd_pick on fake hardware (bus, magnet, base, camera
worker, keys, clock) and run its PICK button beside PickCore.auto_pick on the same fakes: every
goal, torque limit, magnet and base command must come out the same, in the same order. Its
closures (classify_face, do_plan, lift_path, ...) are lifted out of the running cmd_pick frame
and compared one by one. Those need handeye importable (numpy, no lerobot needed); the marker
detector test needs OpenCV. Either missing skips just those tests.
"""
import json
import math
import shutil
import sys
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import head_cam  # noqa: E402
import pick_core as pc  # noqa: E402

KIN, HC, POSES = pc.load_robot()
CAL_HEAD = REPO / "configs" / "calibration" / "robots" / "sortbots_head"
CAL_ARM = json.loads((REPO / "configs/calibration/robots/so_follower/sortbots_arm.json").read_text())
CAL_HEADJ = json.loads((CAL_HEAD / "sortbots_head.json").read_text())
# a magnet-down touch from the calibration (head 24/73, on the table in front of the arm): a
# face point the arm has reached flat for certain
TARGET = KIN.tool_point(np.radians(POSES.samples[26]["arm_deg"]), HC.magnet_m)


def _skip(why):
    raise unittest.SkipTest(why)


def _handeye():
    try:
        import handeye
        import arm_xpad  # noqa: F401  (cmd_pick imports it)
    except Exception as e:  # noqa: BLE001
        _skip(f"handeye not importable here: {e}")
    return handeye


# ---- fakes -----------------------------------------------------------------------------------
class FakeClock:
    """Time passes only in sleep(): both runs see the same instants, so their timed
    interpolation writes the same goals."""

    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        self.t += max(0.0, dt)

    def time(self):
        return 1.8e9 + self.t

    @staticmethod
    def strftime(fmt, *a):
        return "2026-11-16 00:00:00"


class FakeBus:
    """Servos that are wherever their last goal was. Logs every write."""

    def __init__(self, q_deg=None, head=(0.0, 0.0)):
        self.pos = dict(zip(pc.ARM, q_deg if q_deg is not None else POSES.samples[0]["arm_deg"]))
        self.pos.update(head_pan=head[0], head_tilt=head[1])
        self.log, self.torque = [], set()
        self.calibration = {j: types.SimpleNamespace(**CAL_ARM[j]) for j in pc.ARM}
        self.on_goal = None

    def sync_read(self, reg, motors=None, num_retry=0):
        return {m: self.pos[m] for m in (motors or list(self.pos))}

    def sync_write(self, reg, goal, **kw):
        goal = {m: float(v) for m, v in goal.items()}
        self.log.append(("goal", goal))
        self.pos.update(goal)
        if self.on_goal:
            self.on_goal(goal)

    def write(self, reg, motor, val, normalize=True):
        self.log.append(("torque", motor, int(val)))

    def enable_torque(self, motors=None):
        self.torque |= set(motors or self.pos)
        self.log.append(("hold", tuple(motors)))

    def disable_torque(self, motors=None):
        self.torque -= set(motors or self.pos)

    def disconnect(self, disable_torque=True):
        pass


class FakeArmIO:
    """PickCore's ArmIO on a FakeBus, written the way handeye drives the bus."""

    def __init__(self, bus):
        self.bus = bus

    def read(self):
        return self.bus.sync_read("Present_Position")

    def write(self, goal):
        self.bus.sync_write("Goal_Position", goal)

    def set_torque_limit(self, joint, raw):
        self.bus.write("Torque_Limit", joint, raw, normalize=False)

    def hold(self, joints):  # handeye.set_hold(bus, joints, True)
        self.bus.sync_write("Goal_Position", self.bus.sync_read("Present_Position", list(joints)))
        self.bus.enable_torque(list(joints))


class FakeMagnet:
    """PickCore's Magnet; `confirms=False` is a board whose coil never comes on."""

    def __init__(self, log, confirms=True):
        self.log, self.confirms, self.coil = log, confirms, False

    def on(self, seconds=None):
        self.log.append(("magnet", "on"))
        self.coil = self.confirms

    def rearm(self):
        self.log.append(("magnet", "on"))

    def off(self):
        self.log.append(("magnet", "off"))
        self.coil = False

    def status(self):
        return "ON" if self.coil else "OFF"


class FakeBoard:
    """handeye's magnet.Magnet: replies like the ESP32 firmware."""

    def __init__(self, log):
        self.log, self.coil = log, False
        self.s = types.SimpleNamespace(in_waiting=0)

    def on(self, seconds=None):
        self.log.append(("magnet", "on"))
        self.coil = True
        return "ok on"

    def off(self):
        self.log.append(("magnet", "off"))
        self.coil = False
        return "ok off"

    def status(self):
        return "ok state " + ("on" if self.coil else "off")

    def close(self):
        pass


class FakeBase:
    def __init__(self, log):
        self.log = log

    def move(self, dist_m, *a, **kw):
        self.log.append(("base", round(float(dist_m), 6)))
        return True

    def stop(self):
        pass

    def close(self):
        pass


def marker_entry(bus, p_base, normal=(0.0, 0.0, 1.0), k=0, marker_m=0.02):
    """What the camera reports for a package whose magnet target is p_base (base_link), at the
    head angle the bus is at now: centre, orientation, target in the camera frame. Frame k gets
    a little deterministic noise so the medians have work to do."""
    T = HC.matrix(bus.pos["head_pan"], bus.pos["head_tilt"])
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    x = np.cross([0.0, 0.0, 1.0], n) if abs(n[2]) < 0.9 else np.array([0.0, -1.0, 0.0])
    x /= np.linalg.norm(x)
    Rb = np.stack([x, np.cross(n, x), n], 1)
    rng = np.random.default_rng(k)
    c_base = np.asarray(p_base) - Rb @ head_cam.target_in_marker(marker_m) + rng.normal(0, 0.0005, 3)
    Ti = np.linalg.inv(T)
    c = (Ti @ np.append(c_base, 1.0))[:3]
    R = Ti[:3, :3] @ Rb
    tgt = c + R @ head_cam.target_in_marker(marker_m)
    return {"centre": c.tolist(), "R": R.tolist(), "normal": R[:, 2].tolist(), "target_pt": tgt.tolist(),
            "px": [640.0, 360.0], "size_px": 50.0}


def fake_entries(bus, p_base, normal=(0.0, 0.0, 1.0), visible=lambda: True):
    count = [0]

    def entries(marker_id, n=10, timeout=4.0, give_up_s=1.5):
        if not visible():
            return []
        out = []
        for _ in range(n):
            count[0] += 1
            out.append(marker_entry(bus, p_base, normal, count[0]))
        return out
    return entries


def make_core(q_deg=None, head=pc.PICK_HEAD, magnet_confirms=True, place=True, rest=True, **cfg):
    bus = FakeBus(q_deg, head)
    log = bus.log
    mag = FakeMagnet(log, magnet_confirms)
    poses = pc.PickPoses(**{**POSES.__dict__, "place_q": POSES.place_q if place else None,
                            "rest_q": POSES.rest_q if rest else None})
    reports = []
    core = pc.PickCore(FakeArmIO(bus), None, mag, FakeBase(log), KIN, HC, poses, pc.PickConfig(**cfg),
                       report=lambda phase, note, **kw: reports.append((phase, note)), clock=FakeClock())
    core.head_goal = core.head_now()
    core.entries = fake_entries(bus, TARGET)
    return core, bus, mag, reports


# ---- handeye's cmd_pick on the fakes ---------------------------------------------------------
class HandeyeRun:
    """Boot handeye.cmd_pick with every piece of hardware faked; `keys` is what its main loop
    reads, one list per tick (then "x" until it quits). Its locals (the closures and `st`) are
    kept from the first main-loop tick."""

    def __init__(self, keys=(), q_deg=None, head=(0.0, 0.0)):
        he = _handeye()
        self.he, self.bus = he, FakeBus(q_deg, head)
        self.log, self.locals, self.snap = self.bus.log, None, None
        self.keys = list(keys)
        self.tmp = Path(tempfile.mkdtemp())
        shutil.copytree(CAL_HEAD, self.tmp / "sortbots_head")
        clock = FakeClock()
        head_cal = {m: types.SimpleNamespace(**CAL_HEADJ[m]) for m in pc.HEAD}
        log = self.log
        run = self

        class Worker:
            def __init__(self, cam, no_view=False):
                pass

            def entries(self, *a, **kw):
                return run.entries(*a, **kw)

            def show(self, *a):
                pass

            def stop(self):
                pass

        def read_keys(timeout):
            caller = sys._getframe(1).f_code.co_name
            if caller != "cmd_pick":
                return []
            if self.locals is None:
                self.locals = dict(sys._getframe(1).f_locals)
            if self.keys:
                return list(self.keys.pop(0))
            if self.snap is None:
                self.snap = len(log)
            return ["x"]

        self.patches = {
            "time": clock, "calib_dir": lambda: self.tmp / "sortbots_head", "SO101Kin": lambda: KIN,
            "Camera": lambda marker_m=0.02: types.SimpleNamespace(want=None, close=lambda: None),
            "open_bus": lambda *a: (self.bus, head_cal), "CamWorker": Worker, "read_keys": read_keys,
            "termios": types.SimpleNamespace(tcgetattr=lambda fd: None, tcsetattr=lambda *a: None, TCSADRAIN=0),
            "tty": types.SimpleNamespace(setcbreak=lambda fd: None),
            "signal": types.SimpleNamespace(signal=lambda *a: None, SIGHUP=1),
        }
        self.mods = {"magnet": types.SimpleNamespace(Magnet=lambda: FakeBoard(log)),
                     "base": types.SimpleNamespace(Base=lambda: FakeBase(log))}
        self.entries = lambda *a, **kw: []

    def run(self, **args):
        he = self.he
        saved = {k: getattr(he, k) for k in self.patches}
        saved_mods = {k: sys.modules.get(k) for k in self.mods}
        a = types.SimpleNamespace(marker_id=10, marker_m=0.02, head=list(pc.PICK_HEAD), port="", arm_id="",
                                  head_id="sortbots_head", no_view=True, magnet_on_mm=10.0, press_mm=15.0,
                                  press_torque=pc.PRESS_TORQUE, base_back_cm=15.0, lift_tilt_deg=pc.LIFT_TILT_DEG,
                                  wrist_torque=pc.WRIST_COMPLY_TORQUE, pitch_deg=pc.PICK_PITCH_DEG,
                                  side_pitch_deg=None, side_press_mm=pc.SIDE_PRESS_MM, radial_mm=1.0)
        a.__dict__.update(args)
        try:
            for k, v in self.patches.items():
                setattr(he, k, v)
            sys.modules.update(self.mods)
            he.cmd_pick(a)
        finally:
            for k, v in saved.items():
                setattr(he, k, v)
            for k, v in saved_mods.items():
                if v is None:
                    sys.modules.pop(k, None)
                else:
                    sys.modules[k] = v
            shutil.rmtree(self.tmp, ignore_errors=True)
        return self


def same_log(a, b):
    assert len(a) == len(b), f"{len(a)} vs {len(b)} commands"
    for i, (x, y) in enumerate(zip(a, b)):
        assert x[0] == y[0], f"#{i}: {x[0]} vs {y[0]}"
        if x[0] == "goal":
            assert x[1].keys() == y[1].keys(), f"#{i}: {x[1].keys()} vs {y[1].keys()}"
            for m in x[1]:
                assert abs(x[1][m] - y[1][m]) < 1e-6, f"#{i} {m}: {x[1][m]} vs {y[1][m]}"
        else:
            assert x == y, f"#{i}: {x} vs {y}"


# ---- the pure pieces -------------------------------------------------------------------------
def test_consolidate_is_handeyes():
    he = _handeye()
    rng = np.random.default_rng(3)
    for _ in range(5):
        ents = [marker_entry(FakeBus(head=(rng.uniform(0, 40), rng.uniform(40, 70))), TARGET,
                             (rng.normal(0, 0.1), rng.normal(0, 0.1), 1.0), k) for k in range(15)]
        a, b = he.consolidate(ents, 0.02), pc.consolidate(ents, 0.02)
        for x, y in zip(a, b):
            assert np.allclose(x, y)
    assert pc.consolidate([{"R": None}], 0.02) is None


def test_consolidate_recovers_the_target():
    bus = FakeBus(head=pc.PICK_HEAD)
    t, n, sc, sn = pc.consolidate([marker_entry(bus, TARGET, k=k) for k in range(20)], 0.02)
    T = HC.matrix(*pc.PICK_HEAD)
    assert np.linalg.norm((T @ np.append(t, 1))[:3] - TARGET) < 0.0015
    assert sc < 1.0 and sn < 0.5


def test_pieces_match_handeyes_closures():
    run = HandeyeRun(keys=[[]], q_deg=np.degrees(POSES.rest_q)).run()
    L, st = run.locals, run.locals["st"]
    core, bus, _, _ = make_core(q_deg=np.degrees(POSES.rest_q))
    rng = np.random.default_rng(7)
    # aim_corrected / classify_face: top, side facing the arm, side turned away, slanted
    for _ in range(30):
        p = np.array([rng.uniform(0.1, 0.35), rng.uniform(-0.2, 0.2), rng.uniform(-0.05, 0.15)])
        r = rng.uniform(-0.01, 0.01)
        assert np.allclose(L["aim_corrected"](p, r), pc.aim_corrected(p, r))
        rad = head_cam.arm_radial(p)
        for n in ([0, 0, 1], [rng.normal(0, .2), rng.normal(0, .2), 1], [-rad[0], -rad[1], 0.05],
                  [-rad[0] + rng.normal(0, .3), -rad[1] + rng.normal(0, .3), 0], [1, 0, 1], [-rad[0], -rad[1], 1.2]):
            n = np.asarray(n, float) / np.linalg.norm(n)
            want, (got, _) = L["classify_face"](p, n), pc.classify_face(p, n)
            assert (want is None) == (got is None), (p, n)
            if want is not None:
                assert want[0] == got[0] and np.allclose(want[1], got[1])
    # do_plan on the recorded touches: same pitches, heights, poses and approach
    tried = oks = 0
    for s in POSES.samples:
        q = np.radians(s["arm_deg"])
        if KIN.flange(q)[2, 2] < 0.9:
            continue
        tgt = KIN.tool_point(q, HC.magnet_m)
        for b in (run.bus, bus):
            b.pos.update(zip(pc.ARM, np.degrees(POSES.rest_q)))
        for d in (st, core.st):
            d.update(face="top", axis=np.array([0.0, 0.0, 1.0]), locked=tgt.copy(), state="lock", plan=None,
                     pitch_top=None, pitch_app=None)
        ok_h, ok_c = L["do_plan"](), core.do_plan()
        assert ok_h == ok_c
        for k in ("pitch_top", "pitch_app", "high_m", "hover_m", "state"):
            assert st[k] == core.st[k], k
        if ok_h:
            oks += 1
            assert np.allclose(st["plan"]["high"], core.st["plan"]["high"])
            assert len(st["plan"]["steps"]) == len(core.st["plan"]["steps"])
            for (z1, q1), (z2, q2) in zip(st["plan"]["steps"], core.st["plan"]["steps"]):
                assert z1 == z2 and np.allclose(q1, q2)
        tried += 1
        if tried >= 4:
            break
    assert tried >= 3 and oks >= 1
    # straight / lift paths from a seated pose on the target, top and side
    q_seat = np.radians(POSES.samples[26]["arm_deg"])
    for b in (run.bus, bus):
        b.pos.update(zip(pc.ARM, np.degrees(q_seat)))
    for face, axis in (("top", np.array([0.0, 0.0, 1.0])),):
        for d in (st, core.st):
            d.update(face=face, axis=axis, locked=TARGET.copy(), pitch_top=pc.PICK_PITCH_DEG)
        for args in ((0.0, 0.12, 0.01), (0.0, 0.015, 0.005, -4.7), (0.05, 0.01, 0.01)):
            a_, b_ = L["straight"](*args), core.straight(*args)
            assert (a_ is None) == (b_ is None)
            for (z1, q1), (z2, q2) in zip(a_ or [], b_ or []):
                assert z1 == z2 and np.allclose(q1, q2)
        a_, b_ = L["lift_path"](TARGET, 0.08, -4.7), core.lift_path(TARGET, 0.08, -4.7)
        assert (a_ is None) == (b_ is None)
        for (z1, q1), (z2, q2) in zip(a_ or [], b_ or []):
            assert z1 == z2 and np.allclose(q1, q2)


# ---- the batched IK is ik_down, bit for bit ---------------------------------------------------
def test_flange_many_is_flange_bit_for_bit():
    Q = np.random.default_rng(5).uniform(-3.2, 3.2, (500, 5))
    assert np.array_equal(KIN.flange_many(Q), np.array([KIN.flange(q) for q in Q]))
    assert head_cam._flange_many_exact(KIN)  # else ik_down_batch falls back (slower, same answer)


def test_ik_down_batch_is_ik_down_bit_for_bit():
    seeds = [np.radians(s["arm_deg"]) for s in POSES.samples] + [POSES.rest_q]
    rng = np.random.default_rng(11)
    cases = [(TARGET, {}), (TARGET + [0, 0, 0.05], {"pitch_deg": -4.7}), (np.array([0.8, 0.0, 0.0]), {}),
             (np.array([0.229, 0.125, 0.191]), {"pitch_deg": -22.7, "tilt_tol_deg": 2.0}),
             (np.array([0.229, 0.125, 0.211]), {"pitch_deg": -28.7}),
             (np.array([0.25, 0.0, 0.05]), {"pitch_deg": 0.0, "tilt_tol_deg": 3.0, "axis": [-1.0, 0.0, 0.1]})]
    cases += [(np.array([rng.uniform(0.1, 0.35), rng.uniform(-0.2, 0.2), rng.uniform(-0.05, 0.2)]),
               {"pitch_deg": float(rng.uniform(-25, 5))}) for _ in range(6)]
    solved = 0
    for tgt, kw in cases:
        a = head_cam.ik_down(KIN, tgt, seeds, POSES.lo, POSES.hi, HC.magnet_m, 0.04, **kw)
        b = head_cam.ik_down_batch(KIN, tgt, seeds, POSES.lo, POSES.hi, HC.magnet_m, 0.04, **kw)
        assert (a is None) == (b is None), (tgt, kw)
        if a is not None:
            assert np.array_equal(a[0], b[0]) and a[1] == b[1] and a[2] == b[2], (tgt, kw)
            solved += 1
    assert 3 <= solved < len(cases)  # both outcomes covered


def test_the_ik_memo_answers_only_exact_repeats():
    core, bus, mag, _ = make_core()
    seeds = [POSES.rest_q]
    r1 = core._ik(KIN, TARGET, seeds, POSES.lo, POSES.hi, HC.magnet_m, 0.0, pitch_deg=-4.7)
    r2 = core._ik(KIN, TARGET, seeds, POSES.lo, POSES.hi, HC.magnet_m, 0.0, pitch_deg=-4.7, tilt_tol_deg=2.0,
                  axis=None)
    assert len(core._ik_memo) == 1 and (r1 is None) == (r2 is None)
    if r1 is not None:
        assert r1[0] is not r2[0] and np.array_equal(r1[0], r2[0])  # copies: nobody edits the memo
    core._ik(KIN, TARGET, seeds, POSES.lo, POSES.hi, HC.magnet_m, 0.0, pitch_deg=-4.7 + 1e-12)
    assert len(core._ik_memo) == 2


# ---- the whole PICK, command for command -----------------------------------------------------
def _pick_pair(**core_kw):
    """handeye: boot (head ramp, torque reset, go to rest), then PICK. PickCore: the same."""
    q0 = POSES.samples[0]["arm_deg"]
    run = HandeyeRun(keys=[[], ["o"]], q_deg=q0)
    run.entries = fake_entries(run.bus, TARGET)
    run.run()
    he_log = run.log[:run.snap]
    core, bus, mag, _ = make_core(q_deg=q0, head=(0.0, 0.0), **core_kw)
    bus.log.append(("magnet", "off"))  # cmd_pick switches the magnet off as it opens it
    core.start()
    core.go_rest()
    result = core.auto_pick(10)
    return he_log, bus.log, result, run, core


def test_auto_pick_writes_what_handeye_writes():
    he_log, core_log, result, run, core = _pick_pair()
    assert result == "placed", result
    same_log(he_log, core_log)
    kinds = [x[0] if x[0] != "magnet" else x[1] for x in core_log]
    assert "on" in kinds and "off" in kinds and ("base", -0.15) in core_log and ("base", 0.15) in core_log
    assert run.locals["st"]["state"] == core.st["state"] == "lock"


# ---- safety ----------------------------------------------------------------------------------
def _tool_z(bus):
    return KIN.tool_point(np.radians([bus.pos[j] for j in pc.ARM]), HC.magnet_m)[2]


def test_abort_in_the_press_holds_and_leaves_the_magnet_on():
    core, bus, mag, _ = make_core()
    core.go_rest()

    def trip(goal):
        if core.st["flag"] == "PRESSING" and mag.coil and set(goal) == set(pc.ARM):
            core.abort.set()
    bus.on_goal = trip
    assert core.auto_pick(10) == "aborted"
    assert mag.coil, "abort switched the magnet off"
    assert ("magnet", "off") not in bus.log[[i for i, x in enumerate(bus.log) if x == ("magnet", "on")][0]:]
    i_hold = max(i for i, x in enumerate(bus.log) if x[0] == "hold" and x[1] == tuple(pc.ARM))
    after = bus.log[i_hold + 1:]
    # after the hold: only freeze (goal = present) and the torque back to full, no more motion
    goals = [x for x in after if x[0] == "goal"]
    assert len(goals) == 1 and all(abs(goals[0][1][j] - bus.pos[j]) < 1e-9 for j in pc.ARM)
    assert [x for x in after if x[0] == "torque"] == [("torque", j, pc.FULL_TORQUE) for j in pc.PRESS_JOINTS]
    assert core.st["flag"] == "ABORTED"


def test_abort_mid_move_holds_where_it_is():
    core, bus, mag, _ = make_core()
    n = [0]

    def trip(goal):
        n[0] += 1
        if n[0] == 10:
            core.abort.set()
    bus.on_goal = trip
    assert not core.move_to(POSES.rest_q, "MOVING")
    assert bus.log[-1] == ("hold", tuple(pc.ARM)) and core.st["flag"] == "ABORTED"
    assert not mag.coil and ("magnet", "off") not in bus.log


def test_magnet_must_confirm_on_before_the_arm_goes_lower():
    core, bus, mag, reports = make_core(magnet_confirms=False)
    core.go_rest()
    lowest = [1.0]
    locked = []

    def watch(goal):
        if set(goal) == set(pc.ARM) and core.st["locked"] is not None:
            locked.append(core.st["locked"][2])
            lowest[0] = min(lowest[0], _tool_z(bus))
    bus.on_goal = watch
    assert core.auto_pick(10) == "failed"
    assert [x for x in bus.log if x[0] == "magnet"].count(("magnet", "on")) == 2  # one retry, then stop
    assert core.st["flag"] == "ABORTED" and core.st["state"] == "at_hover"
    # never below the magnet-on height (10 mm over the surface), less the model's few-mm slack
    assert lowest[0] > locked[-1] + pc.CONTACT_ABOVE_M - 0.003, (lowest[0], locked[-1])
    assert any("did NOT confirm ON" in note for _, note in reports)


def test_refusals():
    # rest with the magnet on: no motion at all
    core, bus, mag, _ = make_core()
    core.hold(True)
    assert core.magnet_set(True) is True
    n = len(bus.log)
    assert core.go_rest() is False and len(bus.log) == n
    # the magnet on with the arm limp
    core, bus, mag, _ = make_core()
    assert core.magnet_set(True) is False and not mag.coil
    # no magnet board
    core = pc.PickCore(FakeArmIO(FakeBus()), None, None, None, KIN, HC, POSES, clock=FakeClock())
    assert core.auto_pick(10) == "failed"
    # a second PICK while holding
    core, bus, mag, _ = make_core()
    core.st["state"] = "holding"
    n = len(bus.log)
    assert core.auto_pick(10) == "failed" and len(bus.log) == n
    # a target out of reach: the plan refuses, nothing moves past rest
    core, bus, mag, _ = make_core()
    core.go_rest()
    core.entries = fake_entries(bus, np.array([0.8, 0.0, 0.0]))
    assert core.auto_pick(10) == "failed" and core.st["plan"] is None and not mag.coil
    # a slanted face is refused at the lock
    assert pc.classify_face(TARGET, np.array([0.0, 0.7, 0.7]))[0] is None
    # unseen marker: head search, step aside both ways, then refuse with the arm held, magnet off
    core, bus, mag, reports = make_core()
    core.go_rest()
    core.entries = fake_entries(bus, TARGET, visible=lambda: False)
    assert core.auto_pick(10) == "failed" and not mag.coil
    assert any("not seen from rest or from either side" in note for _, note in reports)


def test_without_a_place_pose_it_stops_holding():
    core, bus, mag, _ = make_core(place=False)
    core.go_rest()
    assert core.auto_pick(10) == "picked"
    assert mag.coil and core.st["state"] == "holding"
    assert core.go_rest() is False  # still refuses: magnet on


# ---- the marker detector on a rendered image -------------------------------------------------
def _render(K, R, c, marker_id=10, marker_m=0.02, wh=(1280, 720)):
    """A DICT_4X4_50 marker on a plane (marker frame R, centre c, camera frame) into a white
    image, with the plane's exact depth everywhere."""
    try:
        import cv2
    except ImportError:
        _skip("no OpenCV")
    a = cv2.aruco
    table = a.getPredefinedDictionary(a.DICT_4X4_50)
    side = 400
    img = (a.generateImageMarker(table, marker_id, side) if hasattr(a, "generateImageMarker")
           else a.drawMarker(table, marker_id, side))
    h = marker_m / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    cam = (R @ obj.T).T + c
    px = (K @ cam.T).T
    px = (px[:, :2] / px[:, 2:]).astype(np.float32)
    src = np.float32([[0, 0], [side, 0], [side, side], [0, side]])
    H = cv2.getPerspectiveTransform(src, px)
    w, hh = wh
    gray = cv2.warpPerspective(img, H, (w, hh), flags=cv2.INTER_AREA, borderValue=255)
    color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    n = R[:, 2]
    vv, uu = np.mgrid[0:hh, 0:w]
    rays = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu, float)], -1)
    depth = (n @ c) / (rays @ n)
    return color, depth


def test_marker_detector_recovers_the_target():
    K = np.array([[910.0, 0, 640.0], [0, 910.0, 360.0], [0, 0, 1]])
    for tilt_deg, c in ((0.0, [0.03, -0.02, 0.35]), (35.0, [-0.05, 0.04, 0.42]), (-25.0, [0.0, 0.06, 0.30])):
        t = math.radians(tilt_deg)
        # facing the camera (marker z toward the lens), turned about the camera's x axis
        Rx = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])
        R = Rx @ np.array([[1.0, 0, 0], [0, -1, 0], [0, 0, -1]])
        c = np.array(c)
        color, depth = _render(K, R, c)
        det = pc.MarkerDetector(0.02)
        det.want = {10}
        ms, _, ids = det.markers(color, depth, K)
        assert 10 in ms, f"tilt {tilt_deg}: not found"
        m = ms[10]
        want = c + R @ head_cam.target_in_marker(0.02)
        err = np.linalg.norm(np.array(m["target_pt"]) - want) * 1000
        assert err < 3.0, f"tilt {tilt_deg}: target {err:.1f} mm off"
        assert np.degrees(np.arccos(np.clip(np.dot(m["normal"], R[:, 2]), -1, 1))) < 2.0


def test_entries_reads_new_frames_from_the_camera():
    K = np.array([[910.0, 0, 640.0], [0, 910.0, 360.0], [0, 0, 1]])
    R = np.array([[1.0, 0, 0], [0, -1, 0], [0, 0, -1]])
    color, depth = _render(K, R, np.array([0.0, 0.0, 0.35]))
    n = [0]

    class Cam:
        def latest(self):
            n[0] += 1
            return {"color": color, "depth": depth, "K": K, "stamp": n[0], "head_deg": (9.0, 57.0)}
    core, bus, mag, _ = make_core()
    del core.entries  # the real one, on the camera
    core.cam = Cam()
    ents = core.entries(10, n=3, timeout=2.0)
    assert len(ents) == 3 and ents[0]["head_deg"] == (9.0, 57.0)
    assert core.entries(11, n=3, timeout=2.0, give_up_s=0.2) == []


if __name__ == "__main__":  # no pytest: run every test_* and report
    failed = 0
    for name, fn in sorted((k, v) for k, v in globals().items() if k.startswith("test_") and callable(v)):
        try:
            fn()
            print(f"PASS {name}")
        except unittest.SkipTest as e:
            print(f"SKIP {name}: {e}")
        except Exception:  # noqa: BLE001
            import traceback
            failed += 1
            print(f"FAIL {name}")
            traceback.print_exc()
    sys.exit(1 if failed else 0)
