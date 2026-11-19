"""nodes/base_driver.py's Drive: cmd_vel -> wheel frames on a fake clock and a fake bus.
No ROS, no hardware."""
import math

import pytest
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))
sys.path.insert(0, str(REPO / "scripts"))
import base_driver as bd  # noqa: E402
import feetech  # noqa: E402
from drive_bridge import body_to_wheel_raw, load_config  # noqa: E402

CFG = load_config()
DT = 1.0 / bd.CONTROL_HZ


class FakeBus:
    def __init__(self):
        self.frames, self.closed, self.fail = [], False, False

    def sync_velocity(self, raw_by_id):
        if self.fail:
            raise OSError("device disconnected")
        self.frames.append(dict(raw_by_id))

    def close(self):
        self.closed = True


def make(cfg=CFG, held=()):
    st = {"opened": 0, "released": 0, "bus": None}

    def opener(port, ids):
        st["opened"] += 1
        st["bus"] = FakeBus()
        return st["bus"]

    def releaser(bus, ids):
        st["released"] += 1
        bus.frames.append(dict.fromkeys(ids, 0))
        bus.close()

    d = bd.Drive(cfg, opener=opener, releaser=releaser, holders_of=lambda port: list(held))
    return d, st


def run(d, t0, seconds, cmd=None):
    t = t0
    for _ in range(int(round(seconds / DT))):
        if cmd is not None:
            d.command(*cmd, t)
        d.tick(t, DT)
        t += DT
    return t


def test_board_is_opened_only_when_driving_and_released_when_idle():
    d, st = make()
    t = run(d, 0.0, 1.0)                     # nothing asked: never opened
    assert st["opened"] == 0 and d.state == "idle"
    t = run(d, t, 1.0, cmd=(0.5, 0, 0))      # pad held
    assert st["opened"] == 1 and d.state == "driving"
    t = run(d, t, d.ros["idle_release_s"] + 1.0)  # let go: ramps down, then releases
    assert st["released"] == 1 and d.bus is None and d.state == "idle"


def test_commands_clamped_to_ros_speed_level():
    d, st = make()
    run(d, 0.0, 2.0, cmd=(1.0, 0.0, 3.0))    # past every level, so both are clamped
    assert math.isclose(d.vel[0], d.ros["max_xy"]) and math.isclose(d.vel[2], d.ros["max_w"])
    assert d.ros["max_xy"] == CFG["speed_levels"][CFG["ros"]["speed_level"]]["xy_mps"]
    assert st["bus"].frames[-1] == dict(zip(d.ids, body_to_wheel_raw(*d.vel, CFG)))


def test_deadman_stops_without_commands():
    d, st = make()
    t = run(d, 0.0, 1.0, cmd=(0.1, 0, 0))
    assert any(st["bus"].frames[-1].values())
    t = run(d, t, d.ros["watchdog_s"] + 0.5)  # link dropped: no more cmd_vel
    assert d.vel == (0.0, 0.0, 0.0)
    assert not any(st["bus"].frames[-1].values())


def test_ramp_limits_acceleration():
    d, st = make()
    run(d, 0.0, 2 * DT, cmd=(0.1, 0, 0))
    assert 0 < d.vel[0] <= CFG["accel_mps2"] * 2 * DT + 1e-9


def test_busy_board_is_never_opened_and_holder_named():
    d, st = make(held=[(4242, "python3 scripts/drive_bridge.py serve")])
    run(d, 0.0, 1.0, cmd=(0.1, 0, 0))
    assert st["opened"] == 0 and d.state == "busy" and "4242" in d.message
    run(d, 1.0, d.ros["watchdog_s"] + 0.2)    # nobody asking any more: back to idle
    assert d.state == "idle"


def test_lost_board_stops_and_retries():
    d, st = make()
    t = run(d, 0.0, 0.5, cmd=(0.1, 0, 0))
    st["bus"].fail = True
    t = run(d, t, 0.1, cmd=(0.1, 0, 0))
    assert d.state == "error" and d.bus is None and d.vel == (0.0, 0.0, 0.0)
    t = run(d, t, bd.RETRY_OPEN_S + 0.1, cmd=(0.1, 0, 0))
    assert st["opened"] == 2 and d.state == "driving"


def test_garbage_command_is_zero():
    d, st = make()
    run(d, 0.0, 0.5, cmd=(float("nan"), "x", None))
    assert st["opened"] == 0


def test_release_on_shutdown_zeroes_and_frees():
    d, st = make()
    run(d, 0.0, 0.5, cmd=(0.1, 0, 0))
    bus = st["bus"]
    d.release()
    assert bus.closed and not any(bus.frames[-1].values())


def test_stop_once_without_board(tmp_path, capsys):
    cfg = {**CFG, "port": str(tmp_path / "nope")}
    assert bd.stop_once(cfg) == 0
    assert "nothing to stop" in capsys.readouterr().out


def test_probe_reports_the_board_and_a_returning_board_clears_the_error():
    seq = iter([("no_usb", [7, 8, 9]), ("no_power", [7, 8, 9]), ("ok", [])])
    d = bd.Drive(CFG, opener=lambda port, ids: FakeBus(), releaser=lambda bus, ids: None,
                 holders_of=lambda port: [], prober=lambda port, ids: next(seq))
    d.state, d.message = "error", "wheel board gone"
    d.probe(0.0)
    assert d.status()["board"] == "no_usb" and d.state == "error"
    d.probe(1.0)                                         # not due yet: no second probe
    assert d.board == "no_usb"
    d.probe(bd.PROBE_S)
    assert d.board == "no_power" and d.missing == [7, 8, 9]
    d.probe(2 * bd.PROBE_S)
    assert d.board == "ok" and d.state == "idle" and d.message == ""
    d.command(0.2, 0, 0, 2 * bd.PROBE_S)
    d.tick(2 * bd.PROBE_S, DT)                           # the next press drives at once
    assert d.state == "driving"


def test_probe_leaves_a_held_board_alone():
    calls = []
    d = bd.Drive(CFG, opener=lambda port, ids: FakeBus(), releaser=lambda bus, ids: None,
                 holders_of=lambda port: [(42, "handeye")], prober=lambda *a: calls.append(a))
    d.probe(0.0)
    assert d.board == "busy" and not calls


def test_estop_brakes_ignores_commands_and_holds_until_released():
    d, st = make()
    t = run(d, 0.0, 1.0, cmd=(0.3, 0, 0))                # driving
    assert d.state == "driving" and any(st["bus"].frames[-1].values())
    d.set_estop(True, t)
    t = run(d, t, 0.2, cmd=(0.3, 0, 0))                  # the pad keeps pushing: ignored
    assert d.state == "estop" and not any(st["bus"].frames[-1].values())
    assert d.vel == (0.0, 0.0, 0.0)
    t = run(d, t, d.ros["idle_release_s"] + 1.0)         # latched: held braked, never released
    assert st["released"] == 0 and d.bus is not None and d.status()["estop"] is True
    d.set_estop(False, t)
    t = run(d, t, 0.5, cmd=(0.2, 0, 0))                  # released: drives again
    assert d.state == "driving" and any(st["bus"].frames[-1].values())


def test_estop_brakes_harder_than_the_drive_ramp():
    d, st = make()
    t = run(d, 0.0, 1.0, cmd=(0.3, 0, 0))
    d.set_estop(True, t)
    d.tick(t, DT)
    one_tick = 0.3 - d.vel[0]
    assert one_tick == pytest.approx(CFG.get("estop_decel_mps2", 3.0) * DT, rel=1e-3)
    assert one_tick > CFG["accel_mps2"] * DT


def test_estop_without_the_board_never_opens_it():
    d, st = make()
    d.set_estop(True, 0.0)
    run(d, 0.0, 1.0, cmd=(0.3, 0, 0))
    assert st["opened"] == 0 and d.state == "estop"
