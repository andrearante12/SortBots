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
DT = 1.0 / ad.CONTROL_HZ


class FakeBus:
    """Registers per id, like the servos; records writes and goal frames."""

    def __init__(self, cal, present=None, tamper=None):
        self.regs, self.writes, self.goals, self.open_ = {}, [], [], False
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
