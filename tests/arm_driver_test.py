"""nodes/arm_driver.py's Head on a fake bus: the radians <-> degrees mapping, limits, slew,
the calibration check and torque-on without a jerk. No ROS, no hardware."""
import math
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))
import arm_driver as ad  # noqa: E402
import feetech as ft  # noqa: E402

CFG = ad.load_config(REPO / "configs" / "arm_board.yaml")
# The head tests reason in head.py's degrees; the live sign (pan is mirrored on this robot)
# has its own test below, so pin both to +1 here.
CFG["head"]["pan_sign"] = CFG["head"]["tilt_sign"] = 1.0
DT = 1.0 / ad.CONTROL_HZ


class FakeBus:
    """Registers per id, like the servos; records writes and goal frames."""

    def __init__(self, cal, present=None, tamper=None):
        self.regs, self.writes, self.goals, self.open_ = {}, [], [], False
        self.timeout_s = 0.05
        for m, c in cal.items():
            r = {ft.MODEL_NUMBER: 777, ft.MIN_POSITION_LIMIT: c["range_min"], ft.MAX_POSITION_LIMIT: c["range_max"],
                 ft.HOMING_OFFSET: ad.homing_register(c["homing_offset"]), ft.OPERATING_MODE: 0,
                 ft.RETURN_DELAY: 0, ft.MAX_ACCELERATION: 254, ft.ACCELERATION: 254,
                 ft.P_COEFFICIENT: 16, ft.I_COEFFICIENT: 0, ft.D_COEFFICIENT: 32,
                 ft.TORQUE_ENABLE: 0, ft.LOCK: 1,
                 ft.PRESENT_POSITION: (present or {}).get(m, (c["range_min"] + c["range_max"]) // 2)}
            self.regs[c["id"]] = r
        if tamper:
            tamper(self.regs)

    def open(self):
        self.open_ = True

    def close(self):
        self.open_ = False

    def read(self, sid, reg, sign_bit=None, retries=2):
        if sid not in self.regs:
            raise ft.BusError(f"servo {sid}: no reply")
        return self.regs[sid][reg]

    def write(self, sid, reg, val, retries=2):
        self.regs[sid][reg] = val
        self.writes.append((sid, reg, val))

    def sync_write(self, reg, values, sign_bit=None):
        self.goals.append(dict(values))
        for sid, v in values.items():
            self.regs[sid][reg] = v


def make(**kw):
    bus = FakeBus(CFG["head"]["cal"], **kw)
    return ad.Head(CFG, opener=lambda port, **kw: bus, holders_of=lambda port: []), bus


def test_opens_on_first_command_and_torque_on_at_current_pose():
    head, bus = make(present={"head_pan": 2100})
    assert head.bus is None
    head.command({"head_pan_joint": 0.0}, 0.0)
    assert head.state == "holding"
    first_goal = bus.goals[0]
    assert first_goal[7] == 2100            # goal = where the head is, before torque on
    t_on = [i for i, w in enumerate(bus.writes) if w[1] == ft.TORQUE_ENABLE and w[2] == 1]
    assert t_on and all(w[1] != ft.OPERATING_MODE for w in bus.writes)  # already position: no EEPROM write


def test_slew_and_clamp():
    head, bus = make()
    head.command({"head_pan_joint": math.radians(30)}, 0.0)
    head.tick(0.0, DT)
    pan = ft.raw_to_deg(bus.goals[-1][7], 567, 3412)
    assert 0 < pan <= head.max_dps * DT + 0.1      # one tick of slew, not the whole move
    for i in range(200):
        head.tick(i * DT, DT)
    assert abs(ft.raw_to_deg(bus.goals[-1][7], 567, 3412) - 30) < 0.1
    head.command({"head_pan_joint": math.radians(400)}, 5.0)   # far past the stop
    assert head.target["head_pan"] == pytest.approx(head.lim["head_pan"][1])


def test_tilt_sign_maps_the_sim_convention():
    cfg = {**CFG, "head": {**CFG["head"], "tilt_sign": -1.0}}
    bus = FakeBus(cfg["head"]["cal"])
    head = ad.Head(cfg, opener=lambda port, **kw: bus, holders_of=lambda port: [])
    head.command({"head_tilt_joint": math.radians(10)}, 0.0)
    assert head.target["head_tilt"] == pytest.approx(-10)
    head.pos, head.pos_at = {"head_pan": 0.0, "head_tilt": -10.0}, 0.0
    assert head.status(now=0.0)["tilt"] == pytest.approx(math.radians(10), abs=1e-4)


def test_mismatched_calibration_is_refused():
    head, bus = make(tamper=lambda regs: regs[8].__setitem__(ft.HOMING_OFFSET, 0))
    head.command({"head_tilt_joint": 0.1}, 0.0)
    assert head.bus is None and head.state == "error" and "re-homed" in head.message
    assert not any(w[1] == ft.TORQUE_ENABLE and w[2] == 1 for w in bus.writes)


def test_busy_board_never_opened():
    bus = FakeBus(CFG["head"]["cal"])
    head = ad.Head(CFG, opener=lambda port, **kw: bus, holders_of=lambda port: [(99, "python handeye.py pick")])
    head.command({"head_pan_joint": 0.1}, 0.0)
    assert head.state == "busy" and "99" in head.message and not bus.open_


def test_release_turns_torque_off():
    head, bus = make()
    head.command({"head_pan_joint": 0.0}, 0.0)
    head.release()
    assert bus.regs[7][ft.TORQUE_ENABLE] == 0 and bus.regs[8][ft.TORQUE_ENABLE] == 0 and not bus.open_


def test_garbage_ignored():
    head, bus = make()
    head.command({"head_pan_joint": float("nan"), "elbow": 1.0, "head_tilt_joint": "x"}, 0.0)
    assert head.target is None or all(math.isfinite(v) for v in head.target.values())


def test_peek_reads_position_without_torque_or_holding():
    head, bus = make(present={"head_pan": 2100})
    head.poll(0.0)
    assert head.bus is None and not bus.open_ and not bus.writes
    assert head.status(now=0.0)["pan"] == pytest.approx(math.radians(ft.raw_to_deg(2100, 567, 3412)), abs=1e-4)
    head.poll(0.5)                 # rate-limited: no second peek inside PEEK_S
    bus.regs[7][ft.PRESENT_POSITION] = 1989
    head.poll(0.6)
    assert head.status(now=0.6)["pan"] != pytest.approx(0, abs=1e-3)
    head.poll(1.1)
    assert head.status(now=1.1)["pan"] == pytest.approx(0, abs=1e-2)


def test_failed_peek_keeps_the_last_pose_until_stale():
    head, bus = make(present={"head_pan": 2100})
    head.poll(0.0)
    good = head.status(now=0.0)["pan"]
    del bus.regs[7]                # the board stops answering
    head.poll(1.1)
    assert head.status(now=1.1)["pan"] == good          # a failed peek doesn't blank it
    assert "pan" not in head.status(now=ad.POSE_STALE_S + 0.1)


def test_peek_skips_a_held_board():
    bus = FakeBus(CFG["head"]["cal"])
    head = ad.Head(CFG, opener=lambda port, **kw: bus, holders_of=lambda port: [(7, "handeye")])
    head.poll(0.0)
    assert head.pos is None and not bus.open_


class FlakyBus(FakeBus):
    """Drops every read while `down` is set, like the Jetson's arm-board link did."""
    down = False

    def read(self, sid, reg, sign_bit=None, retries=2):
        if self.down:
            raise ft.BusError(f"servo {sid}: no reply")
        return super().read(sid, reg, sign_bit, retries)


def test_open_rides_through_a_dropout():
    bus = FlakyBus(CFG["head"]["cal"])
    bus.down = True
    head = ad.Head(CFG, opener=lambda port, **kw: bus, holders_of=lambda port: [])
    head.command({"head_pan_joint": 0.0}, 0.0)
    assert head.state == "connecting" and head.bus is None
    bus.down = False
    head.tick(0.0 + ad.OPEN_RETRY_FAST_S + 0.01, DT)        # the tick retries, no new command needed
    assert head.state == "holding" and head.bus is bus


def test_open_gives_up_after_a_long_silence():
    bus = FlakyBus(CFG["head"]["cal"])
    bus.down = True
    head = ad.Head(CFG, opener=lambda port, **kw: bus, holders_of=lambda port: [])
    head.command({"head_pan_joint": 0.0}, 0.0)
    t = 0.0
    while t < ad.OPEN_GIVE_UP_S + 1 and head.state == "connecting":
        t += ad.OPEN_RETRY_FAST_S + 0.01
        head.tick(t, DT)
    assert head.state == "error"


def test_held_board_survives_short_read_gaps_and_resends_goals():
    bus = FlakyBus(CFG["head"]["cal"])
    head = ad.Head(CFG, opener=lambda port, **kw: bus, holders_of=lambda port: [])
    head.command({"head_pan_joint": 0.0}, 0.0)
    n = len(bus.goals)
    bus.down = True
    for i in range(1, 11):                     # 2 s of lost reads at STATE_HZ
        head.poll(i * 0.2)
        head.tick(i * 0.2, DT)
    assert head.bus is bus and head.state == "holding"
    assert len(bus.goals) > n                  # goals kept going out while reads were lost
    head.poll(ad.LINK_LOST_S + 1.0)
    assert head.bus is None and head.state == "error"


def test_calibration_mismatch_is_not_retried():
    head, bus = make(tamper=lambda regs: regs[8].__setitem__(ft.HOMING_OFFSET, 0))
    head.command({"head_tilt_joint": 0.1}, 0.0)
    assert head.state == "error"


# -- the arm ----------------------------------------------------------------------------------

def arm_bus(present_deg=None, tamper=None):
    """A FakeBus with the head AND the arm's five servos, the arm resting at present_deg."""
    cal = {**CFG["head"]["cal"], **CFG["arm"]["cal"]}
    present = {}
    for j, d in (present_deg or {}).items():
        c = cal[j]
        present[j] = ft.deg_to_raw(d, c["range_min"], c["range_max"])
    bus = FakeBus(cal, present=present, tamper=tamper)
    for c in CFG["arm"]["cal"].values():
        bus.regs[c["id"]][ad.TORQUE_LIMIT] = 300
    return bus


# A reachable, unfolded pose (degrees) so IK has room to move the hand every way.
READY = {"shoulder_pan": 0.0, "shoulder_lift": -40.0, "elbow_flex": 40.0, "wrist_flex": 40.0, "wrist_roll": 0.0}


def make_arm(**kw):
    bus = arm_bus(**kw)
    return ad.Arm(CFG), bus


def run_arm(arm, bus, t0, seconds, jog=None):
    t = t0
    for _ in range(int(round(seconds / DT))):
        if jog is not None:
            arm.command(jog, t)
        arm.tick(bus, t, DT)
        t += DT
    return t


def test_arm_engage_never_moves_and_restores_torque_limit():
    arm, bus = make_arm(present_deg=READY)
    arm.engage(bus)
    first = bus.goals[0]
    for j, sid in arm.ids.items():
        assert first[sid] == bus.regs[sid][ft.PRESENT_POSITION]   # goal = present before torque on
        assert bus.regs[sid][ft.TORQUE_ENABLE] == 1
        assert bus.regs[sid][ad.TORQUE_LIMIT] == 1000
    assert arm.engaged and arm.state == "holding"


def test_jog_moves_the_hand_the_way_asked():
    arm, bus = make_arm(present_deg=READY)
    arm.engage(bus)
    x0, y0, z0, _ = arm.kin.pose(arm.q)
    run_arm(arm, bus, 0.0, 1.0, jog={"up": 1})
    z1 = arm.kin.pose(arm.q)[2]
    assert z1 - z0 > 0.02                                    # ~5 cm/s for 1 s, joint-capped
    r0 = math.hypot(*arm.kin.pose(arm.q)[:2])
    run_arm(arm, bus, 1.0, 1.0, jog={"out": 1})
    assert math.hypot(*arm.kin.pose(arm.q)[:2]) > r0 + 0.02
    th0 = math.atan2(*arm.kin.pose(arm.q)[1::-1])
    run_arm(arm, bus, 2.0, 1.0, jog={"left": 1})
    assert math.atan2(*arm.kin.pose(arm.q)[1::-1]) > th0      # left = counter-clockwise


def test_arm_deadman_stops_the_hand():
    arm, bus = make_arm(present_deg=READY)
    arm.engage(bus)
    t = run_arm(arm, bus, 0.0, 0.5, jog={"up": 1})
    q_stop = arm.q.copy()
    run_arm(arm, bus, t + arm.watchdog_s + 0.01, 1.0)        # no more commands
    assert (arm.q == q_stop).all()


def test_joint_speed_is_capped():
    arm, bus = make_arm(present_deg=READY)
    arm.engage(bus)
    q0 = arm.q.copy()
    arm.command({"out": 1, "up": 1, "left": 1, "tilt": 1}, 0.0)
    arm.tick(bus, 0.0, DT)
    assert max(abs(a - b) for a, b in zip(arm.q, q0)) <= arm.joint_rps * ad.LEVELS[arm.level] * DT + 1e-9


def test_arm_calibration_mismatch_refused():
    arm, bus = make_arm(present_deg=READY, tamper=lambda regs: regs[3].__setitem__(ft.HOMING_OFFSET, 0))
    with pytest.raises(ad.CalibrationMismatch):
        arm.engage(bus)
    assert not any(w[1] == ft.TORQUE_ENABLE and w[2] == 1 and w[0] in (1, 2, 3, 4, 5) for w in bus.writes)


def test_arm_state_reports_pose():
    arm, bus = make_arm(present_deg=READY)
    arm.engage(bus)
    arm.poll(bus, 1.0)
    st = arm.status(now=1.0)
    assert st["engaged"] and set(st["pose"]) == {"x", "y", "z", "pitch"}


# -- the magnet ---------------------------------------------------------------------------------

class FakeESP32:
    """hardware/magnet_esp32/main.py's line protocol on a pty."""

    def __init__(self):
        import os as _os, pty, threading, tty
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.path, self.cmds, self.on = _os.ttyname(self.slave), [], False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        import os as _os
        buf = b""
        while True:
            try:
                buf += _os.read(self.master, 256)
            except OSError:
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                cmd = line.decode().strip()
                self.cmds.append(cmd)
                if cmd == "ping":
                    reply = "ok pong"
                elif cmd.startswith("on"):
                    self.on, reply = True, "ok on"
                elif cmd == "off":
                    self.on, reply = False, "ok off"
                else:
                    reply = "ok " + ("on" if self.on else "off")
                _os.write(self.master, (reply + "\n").encode())


def test_magnet_on_rearm_and_off_on_close(monkeypatch):
    import time as _t
    esp = FakeESP32()
    monkeypatch.setattr(ad.fcntl, "ioctl", lambda *a: None)   # a pty has no DTR/RTS lines
    mag = ad.Magnet({"magnet": {"port_glob": esp.path, "on_s": 30, "rearm_s": 0.3}})
    mag.set(True)
    deadline = _t.time() + 5
    while mag.status()["board"] != "ON" and _t.time() < deadline:
        _t.sleep(0.05)
    assert mag.status()["board"] == "ON" and esp.on
    _t.sleep(0.8)
    assert sum(c.startswith("on") for c in esp.cmds) >= 2          # re-armed
    assert all(c in ("ping", "on 30", "off") for c in esp.cmds)
    mag.close()
    assert not esp.on and esp.cmds[-1] == "off"                    # never left on


def test_wrist_flex_and_level():
    arm, bus = make_arm(present_deg=READY)
    arm.engage(bus)
    p0 = arm.kin.pose(arm.q)[3]
    run_arm(arm, bus, 0.0, 0.5, jog={"wf": 1})
    assert arm.kin.pose(arm.q)[3] > p0                     # "wrist up" raises the tool pitch
    arm2, bus2 = make_arm(present_deg=READY)
    arm2.engage(bus2)
    arm2.command({"up": 1}, 0.0, level=0)
    z0 = arm2.kin.pose(arm2.q)[2]
    run_arm(arm2, bus2, 0.0, 1.0, jog={"up": 1})
    slow = arm2.kin.pose(arm2.q)[2] - z0
    arm3, bus3 = make_arm(present_deg=READY)
    arm3.engage(bus3)
    arm3.command({"up": 1}, 0.0, level=3)
    z0 = arm3.kin.pose(arm3.q)[2]
    run_arm(arm3, bus3, 0.0, 1.0, jog={"up": 1})
    assert arm3.kin.pose(arm3.q)[2] - z0 > 2 * slow        # L4 is 4x L1, joint-capped


def test_nav_pose_aims_and_holds_the_head():
    head, bus = make(present={"head_pan": 2600})
    head.nav = {"head_pan_joint": 0.0, "head_tilt_joint": math.radians(5)}
    head.hold_nav(0.0)
    assert head.state == "holding"
    assert head.target == {"head_pan": 0.0, "head_tilt": pytest.approx(5.0)}
    head.command({"head_pan_joint": 0.5}, 1.0)        # moved by hand meanwhile: nav puts it back
    head.hold_nav(1.5)
    assert head.target["head_pan"] == 0.0


def test_nav_head_can_look_around_and_comes_back():
    # 2026-11-20: a /head_cmd in nav turns the head (mapping pauses on off_nav), and stays
    # turned; one at the nav pose hands it back to hold_nav
    head, bus = make(present={"head_pan": 2600})
    head.nav = {"head_pan_joint": 0.0, "head_tilt_joint": math.radians(5)}
    head.hold_nav(0.0)
    assert head.off_nav()                               # aimed, but the servo isn't there yet
    head.pos = dict(head.nav_deg())                     # arrived
    assert not head.looking and not head.off_nav()
    head.look({"head_pan_joint": 0.6}, 1.0)
    assert head.looking and head.off_nav()
    turned = dict(head.target)
    head.hold_nav(1.5)                                  # nav no longer drags it back
    assert head.target == turned and abs(turned["head_pan"]) > 10
    head.look({"head_pan_joint": 0.0, "head_tilt_joint": math.radians(5)}, 2.0)
    assert not head.looking
    head.pos = {"head_pan": 25.0, "head_tilt": 5.0}     # sent back, but not there yet
    assert head.off_nav()
    head.pos = {"head_pan": 1.0, "head_tilt": 5.5}      # within NAV_TOL_DEG: mapping may resume
    assert not head.off_nav()
    st = head.status(0.0)
    assert st["nav"] == {"pan": 0.0, "tilt": pytest.approx(math.radians(5), abs=1e-4)} and st["looking"] is False


def test_nav_pose_opens_a_board_that_shows_up_later():
    plugged = {"in": False}

    class Unplugged(FakeBus):
        def open(self):
            if not plugged["in"]:
                raise OSError(2, "No such file or directory")   # ft.Bus opens the port here
            super().open()
    bus = Unplugged(CFG["head"]["cal"])
    head = ad.Head(CFG, opener=lambda port, **kw: bus, holders_of=lambda port: [])
    head.hold_nav(0.0)
    assert head.bus is None
    plugged["in"] = True
    head.hold_nav(0.5)                                  # inside the retry gap: not yet
    assert head.bus is None
    head.hold_nav(ad.RETRY_OPEN_S + 0.1)
    assert head.state == "holding" and head.bus is bus and bus.open_


class PowerBus(FakeBus):
    """The arm board with its supply switched: the USB chip answers opens, servos only reads
    while powered."""
    powered = True

    def read(self, sid, reg, sign_bit=None, retries=2):
        if not PowerBus.powered:
            raise ft.BusError(f"servo {sid}: no reply")
        return super().read(sid, reg, sign_bit, retries)


def test_peek_names_the_board_and_a_powered_board_is_held_again():
    bus = PowerBus({**CFG["head"]["cal"], **CFG["arm"]["cal"]})
    head = ad.Head(CFG, opener=lambda port, **kw: bus, holders_of=lambda port: [])
    PowerBus.powered = True
    try:
        head.command({"head_pan_joint": 0.0}, 0.0)
        assert head.state == "holding" and head.status(now=0.0)["board"] == "ok"
        head._lost(1.0, "arm board silent")                 # the supply went off
        PowerBus.powered = False
        head.poll(1.0)
        assert head.board == "no_power" and head.status(now=1.0)["board"] == "no_power"
        head.tick(1.0 + ad.RETRY_OPEN_S, DT)                # nothing answers: not reopened
        assert head.bus is None
        PowerBus.powered = True
        head.poll(1.0 + ad.PEEK_S)
        assert head.board == "ok"
        head.tick(1.0 + ad.PEEK_S + ad.RETRY_OPEN_S, DT)    # back: held again, no restart
        assert head.state == "holding" and head.bus is bus
    finally:
        PowerBus.powered = True


def test_peek_names_silent_arm_servos_and_a_busy_board():
    bus = FakeBus(CFG["head"]["cal"])                       # the arm's ids don't answer
    head = ad.Head(CFG, opener=lambda port, **kw: bus, holders_of=lambda port: [])
    head.poll(0.0)
    assert head.board == "partial" and sorted(head.missing) == sorted(head.arm_ids)
    busy = ad.Head(CFG, opener=lambda port, **kw: bus, holders_of=lambda port: [(9, "handeye")])
    busy.poll(0.0)
    assert busy.board == "busy"
