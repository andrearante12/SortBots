"""The real robot's one-button pick (scripts/pick_core.py, handeye.py's PICK) on the pieces
nodes/arm_driver.py already owns: the arm board, the magnet, and the ROS camera and base.

arm_driver imports this; it holds the board, so the pick runs in its process, on its own
thread, through these adapters (pick_core's duck-typed interfaces):

    BusArmIO    ArmIO on the held feetech.Bus (lerobot DEGREES, the repo calibration)
    RosCamera   Camera: /<id>/camera/{rgb,depth,camera_info}, subscribed only while a pick
                runs (a 848x480 RGB + depth pair at 30 fps is real CPU on the Orin)
    MagnetIO    Magnet: arm_driver's Magnet thread, with a synchronous status() ask
    CmdVelBase  Base: drives by publishing /<id>/cmd_vel (base_driver owns the wheels), and
                gives up if /<id>/base_status doesn't say "driving" soon: handeye's
                drive_base returning False, which stops the place before the arm swings

No ROS imports at module level: the tests drive these with fakes.
"""
from __future__ import annotations

import math
import threading
import time

import feetech as ft

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
HEAD = ["head_pan", "head_tilt"]
TORQUE_LIMIT = (48, 2)


class BusArmIO:
    """pick_core's ArmIO on one held board: read/write in lerobot's DEGREES convention."""

    def __init__(self, bus, head_cal: dict, arm_cal: dict):
        self.bus = bus
        self.cal = {**{m: head_cal[m] for m in HEAD}, **{j: arm_cal[j] for j in ARM}}
        self.ids = {j: int(c["id"]) for j, c in self.cal.items()}
        self.last = {}
        self.goal = {}   # the last goal written per joint: arm_driver's pick_trace (no second bus reader)

    def _deg(self, j, raw):
        c = self.cal[j]
        return ft.raw_to_deg(raw, c["range_min"], c["range_max"])

    def _raw(self, j, deg):
        c = self.cal[j]
        return ft.deg_to_raw(deg, c["range_min"], c["range_max"])

    def read(self):
        out = {j: self._deg(j, self.bus.read(sid, ft.PRESENT_POSITION)) for j, sid in self.ids.items()}
        self.last = out
        return dict(out)

    def write(self, deg: dict):
        self.bus.sync_write(ft.GOAL_POSITION, {self.ids[j]: self._raw(j, d) for j, d in deg.items()})
        self.goal = {**self.goal, **deg}

    def set_torque_limit(self, joint, raw):
        self.bus.write(self.ids[joint], TORQUE_LIMIT, int(max(0, min(1000, raw))))

    def hold(self, joints):
        """handeye's set_hold: goal = present, then torque on (never a jerk)."""
        now = {j: self._deg(j, self.bus.read(self.ids[j], ft.PRESENT_POSITION)) for j in joints}
        self.write(now)
        for j in joints:
            self.bus.write(self.ids[j], ft.TORQUE_ENABLE, 1)
            self.bus.write(self.ids[j], ft.LOCK, 1)


def image_to_bgr(msg):
    import numpy as np
    a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    a = a.reshape(msg.height, msg.width, 3)
    return a[..., ::-1].copy() if msg.encoding.lower() == "rgb8" else a.copy()


def depth_to_m(msg):
    import numpy as np
    if msg.encoding in ("16UC1", "mono16"):
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) * 0.001
    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
    raise ValueError(f"depth encoding {msg.encoding}")


class RosCamera:
    """pick_core's Camera from the RealSense topics (depth aligned to colour, one camera_info).
    Frames are paired by nearest stamp; head_deg is the head the pick last read, which is where
    the head sat while the frame was taken (pick_core reads the head after a move settles)."""

    MAX_SKEW_S = 0.025

    def __init__(self, io: BusArmIO):
        self.io = io
        self.lock = threading.Lock()
        self.rgb = self.depth = self.K = None
        self.seq = 0

    @staticmethod
    def _t(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_rgb(self, msg):
        with self.lock:
            self.rgb = msg
            self.seq += 1

    def on_depth(self, msg):
        with self.lock:
            self.depth = msg

    def on_info(self, msg):
        import numpy as np
        with self.lock:
            self.K = np.array(msg.k, dtype=float).reshape(3, 3)

    def latest(self):
        with self.lock:
            rgb, depth, K, seq = self.rgb, self.depth, self.K, self.seq
        if rgb is None or depth is None or K is None or abs(self._t(rgb) - self._t(depth)) > self.MAX_SKEW_S:
            return None
        head = self.io.last
        return {"color": image_to_bgr(rgb), "depth": depth_to_m(depth), "K": K, "stamp": seq,
                "head_deg": (head.get("head_pan", 0.0), head.get("head_tilt", 0.0))}


class MagnetIO:
    """pick_core's Magnet on arm_driver's Magnet thread. on/off wait for the board's reply;
    status() asks the board, so a confirmation is the ESP32's word, as handeye trusts."""

    def __init__(self, magnet, wait_s=2.0):
        self.m, self.wait_s = magnet, wait_s

    def on(self, seconds=None):
        self.m.ask("on")

    def off(self):
        self.m.ask("off")

    def rearm(self):
        self.m.ask("on")

    def status(self):
        return self.m.ask("status")


class CmdVelBase:
    """pick_core's Base: a straight move by publishing cmd_vel, the way the drive pad does.
    publish(x) sends a Twist with linear.x = x; base_state() is base_driver's last status."""

    HZ = 20
    START_S = 0.8   # base_driver opens the board on the first command; this long to say "driving"

    def __init__(self, publish, base_state, abort, clock=time):
        self.publish, self.base_state, self.abort, self.clock = publish, base_state, abort, clock

    def move(self, dist_m, speed_mps):
        speed = abs(float(speed_mps))
        if speed <= 0 or dist_m == 0:
            return True
        x = math.copysign(speed, dist_m)
        t0 = self.clock.monotonic()
        end = t0 + abs(dist_m) / speed
        ok = True
        try:
            while self.clock.monotonic() < end:
                if self.abort.is_set():
                    ok = False
                    break
                self.publish(x)
                st = self.base_state() or {}
                if self.clock.monotonic() - t0 > self.START_S and st.get("state") != "driving":
                    ok = False   # no wheels (board unplugged, busy, error): stop before the arm swings
                    break
                self.clock.sleep(1.0 / self.HZ)
        finally:
            for _ in range(3):
                self.publish(0.0)
        return ok
