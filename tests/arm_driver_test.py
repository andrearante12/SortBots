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
    return ad.Head(CFG, opener=lambda port: bus, holders_of=lambda port: []), bus


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
    head = ad.Head(cfg, opener=lambda port: bus, holders_of=lambda port: [])
    head.command({"head_tilt_joint": math.radians(10)}, 0.0)
    assert head.target["head_tilt"] == pytest.approx(-10)
    head.pos = {"head_pan": 0.0, "head_tilt": -10.0}
    assert head.status()["tilt"] == pytest.approx(math.radians(10), abs=1e-4)


def test_mismatched_calibration_is_refused():
    head, bus = make(tamper=lambda regs: regs[8].__setitem__(ft.HOMING_OFFSET, 0))
    head.command({"head_tilt_joint": 0.1}, 0.0)
    assert head.bus is None and head.state == "error" and "re-homed" in head.message
    assert not any(w[1] == ft.TORQUE_ENABLE and w[2] == 1 for w in bus.writes)


def test_busy_board_never_opened():
    bus = FakeBus(CFG["head"]["cal"])
    head = ad.Head(CFG, opener=lambda port: bus, holders_of=lambda port: [(99, "python handeye.py pick")])
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
    assert head.status()["pan"] == pytest.approx(math.radians(ft.raw_to_deg(2100, 567, 3412)), abs=1e-4)
    head.poll(0.5)                 # rate-limited: no second peek inside PEEK_S
    bus.regs[7][ft.PRESENT_POSITION] = 1989
    head.poll(0.6)
    assert head.status()["pan"] != pytest.approx(0, abs=1e-3)
    head.poll(1.1)
    assert head.status()["pan"] == pytest.approx(0, abs=1e-2)


def test_peek_skips_a_held_board():
    bus = FakeBus(CFG["head"]["cal"])
    head = ad.Head(CFG, opener=lambda port: bus, holders_of=lambda port: [(7, "handeye")])
    head.poll(0.0)
    assert head.pos is None and not bus.open_
