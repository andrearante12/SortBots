"""nodes/pick_ros.py: the adapters that put scripts/pick_core.py on arm_driver's board, the ROS
camera, the magnet and the base. Fakes only: no ROS, no hardware."""
import sys
import threading
import types
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))
sys.path.insert(0, str(REPO / "tests"))
import arm_driver as ad  # noqa: E402
import feetech as ft  # noqa: E402
import pick_ros as pr  # noqa: E402
from arm_driver_test import CFG, arm_bus, READY  # noqa: E402


def test_arm_io_reads_writes_and_holds_without_a_jerk():
    bus = arm_bus(present_deg=READY)
    io = pr.BusArmIO(bus, CFG["head"]["cal"], CFG["arm"]["cal"])
    pos = io.read()
    assert set(pos) == set(pr.ARM + pr.HEAD)
    assert abs(pos["shoulder_lift"] - READY["shoulder_lift"]) < 0.2
    io.hold(pr.ARM)
    hold_goal = bus.goals[-1]
    for j in pr.ARM:
        sid = io.ids[j]
        assert hold_goal[sid] == bus.regs[sid][ft.PRESENT_POSITION]
        assert bus.regs[sid][ft.TORQUE_ENABLE] == 1
    io.write({"elbow_flex": 10.0})
    c = CFG["arm"]["cal"]["elbow_flex"]
    assert bus.goals[-1] == {io.ids["elbow_flex"]: ft.deg_to_raw(10.0, c["range_min"], c["range_max"])}
    io.set_torque_limit("wrist_flex", 100)
    assert bus.regs[io.ids["wrist_flex"]][pr.TORQUE_LIMIT] == 100


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t

    def sleep(self, s):
        self.t += s


def test_base_moves_for_dist_over_speed_and_stops():
    sent, clock = [], FakeClock()
    base = pr.CmdVelBase(sent.append, lambda: {"state": "driving"}, threading.Event(), clock=clock)
    assert base.move(-0.15, 0.05)
    assert abs(clock.t - 3.0) < 0.1
    assert all(x == -0.05 for x in sent[:-3]) and sent[-3:] == [0.0] * 3


def test_base_gives_up_without_wheels():
    sent, clock = [], FakeClock()
    base = pr.CmdVelBase(sent.append, lambda: {"state": "error"}, threading.Event(), clock=clock)
    assert not base.move(0.15, 0.05)
    assert clock.t < 1.5 and sent[-1] == 0.0      # stopped soon after START_S, wheels zeroed


def test_base_stops_on_abort():
    sent, clock, abort = [], FakeClock(), threading.Event()
    def state():
        if clock.t > 1.0:
            abort.set()
        return {"state": "driving"}
    base = pr.CmdVelBase(sent.append, state, abort, clock=clock)
    assert not base.move(0.15, 0.05)
    assert clock.t < 1.5 and sent[-1] == 0.0


def _img(h, w, enc, data, sec=1, ns=0, step=None):
    return types.SimpleNamespace(height=h, width=w, encoding=enc, data=bytes(data), step=step or len(data) // h,
                                 header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec, nanosec=ns)))


def test_camera_bundle_converts_and_pairs_by_stamp():
    io = types.SimpleNamespace(last={"head_pan": 9.0, "head_tilt": 57.0})
    cam = pr.RosCamera(io)
    assert cam.latest() is None
    rgb = np.zeros((2, 3, 3), np.uint8)
    rgb[..., 0] = 200                                   # red in RGB
    cam.on_rgb(_img(2, 3, "rgb8", rgb.tobytes()))
    cam.on_depth(_img(2, 3, "16UC1", np.full((2, 3), 1234, np.uint16).tobytes()))
    cam.on_info(types.SimpleNamespace(k=[600, 0, 1.5, 0, 600, 1, 0, 0, 1]))
    b = cam.latest()
    assert b["color"][0, 0, 2] == 200 and b["color"][0, 0, 0] == 0   # BGR now
    assert abs(b["depth"][0, 0] - 1.234) < 1e-6 and b["K"][0, 0] == 600
    assert b["head_deg"] == (9.0, 57.0)
    cam.on_depth(_img(2, 3, "16UC1", np.zeros((2, 3), np.uint16).tobytes(), sec=2))  # 1 s apart
    assert cam.latest() is None


def test_magnet_io_asks_the_board():
    asked = []
    m = types.SimpleNamespace(ask=lambda c: asked.append(c) or ("ON" if c != "off" else "OFF"))
    io = pr.MagnetIO(m)
    io.on(30); io.rearm(); io.off()
    assert io.status() == "ON" and asked == ["on", "on", "off", "status"]
