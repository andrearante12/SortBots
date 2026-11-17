#!/usr/bin/env python3
"""The real robot's arm board over ROS: the pan/tilt head, the SO-101 arm and the magnet.

    python3 nodes/arm_driver.py --robot-id robot_0     the node (platform:=real bringup)

One process owns the board (configs/arm_board.yaml), because the head and the arm share it
and a servo board takes one owner: anything that moves either goes through here.

    /<id>/head_cmd    sensor_msgs/JointState  head_pan_joint / head_tilt_joint, radians, in
                                              the sim's convention (tilt > 0 looks down), the
                                              same message the operator view's sliders send
    /<id>/head_state  std_msgs/String (JSON)  {"pan", "tilt"} radians where the head IS,
                                              "held", and "state"/"message" when it can't move
    /<id>/arm_cmd     std_msgs/String (JSON)  {"jog": {"out","left","up","tilt","roll"}} each
                                              -1..1, re-sent at ~10 Hz while a pad button is
                                              held (the deadman); {"magnet": "on"|"off"};
                                              {"stop": true}
    /<id>/arm_state   std_msgs/String (JSON)  "engaged", "state"/"message", "pose" (the tip in
                                              base_link: x, y, z m, pitch rad), "magnet"

The arm jogs the HAND, not joints: scripts/arm_cartesian.py's teleop (radial / around the
base / up-down / tool tilt / wrist roll, damped-least-squares IK in so101_kin, joint speed
capped so a goal never outruns the servo), driven by the operator view's arm pad.

The servo work is head.py's, re-homed on nodes/feetech.py because lerobot isn't in the robot
container: the same calibration file, position mode, gains and limits, with every frame
checked against lerobot's (tests/feetech_test.py).

Safety:
- The board is opened on the first head command and a holder is never opened underneath
  (the Robot view's handeye pick, head.py keys): /<id>/head_state names it.
- The servo registers must match the calibration file (homing offset, position limits), as
  lerobot's is_calibrated checks: a swapped or re-homed servo is refused, not driven.
- Goals are clamped inside the calibrated range and slewed at `head.max_dps`.
- Torque comes on only after the goal is set to where the head already is, so it never jerks.
  The arm likewise, and its jog stops ARM_WATCHDOG after the last command (deadman).
- On stop the magnet is switched off and the arm HOLDS where it is, torque on: limp, it
  would drop onto whatever is under it (decided 2026-11-16; handeye.py goes limp instead).
"""
from __future__ import annotations

import argparse
import fcntl
import glob
import json
import math
import os
import queue
import select
import signal
import struct
import sys
import termios
import threading
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
# so101_kin reads the arm's kinematic offsets from lerobot's calibration dir; in the robot
# container that is the repo copy (~/.cache isn't mounted there, scripts/calibration.sh).
os.environ.setdefault("HF_LEROBOT_CALIBRATION", str(REPO / "configs" / "calibration"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(REPO / "scripts"))
import feetech as ft  # noqa: E402
from drive_bridge import busy_message, holders  # noqa: E402

CONTROL_HZ = 50
STATE_HZ = 5
RETRY_OPEN_S = 2.0
PEEK_S = 1.0  # while not holding the board: read where the head is this often, torque untouched
POSE_STALE_S = 5.0  # a pose older than this isn't reported (a failed peek keeps the last one)
# The arm board's link dropped out for ~1-2 s at a time (2026-11-16: ~25% of reads lost in
# bursts, through lerobot too). The cause was the arm's half-seated servo cable loading the
# shared line; reseated, 0 of 1488 were lost. Still ridden through, in case it comes back:
# opening retries fast for a while, and a held board is dropped only after this much silence.
LINK_LOST_S, OPEN_RETRY_FAST_S, OPEN_GIVE_UP_S = 5.0, 0.3, 6.0
GOAL_RESEND_S = 0.2
HELD_READ_TIMEOUT_S = 0.015  # a reply takes ~0.4 ms when it comes
# Every other peek, and the first head command, failed live (2026-11-16): the first read right
# after opening the port went unanswered. A short settle after open, and a longer reply wait
# than the 20 ms default, on every open of this board.
PEEK_SETTLE_S, PEEK_TIMEOUT_S = 0.03, 0.05
HEAD_JOINTS = {"head_pan_joint": "head_pan", "head_tilt_joint": "head_tilt"}
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
JOG_AXES = ("out", "left", "up", "tilt", "roll")
TORQUE_LIMIT = (48, 2)  # RAM, 0..1000; handeye.py cuts it for the press and restores 1000


def load_config(path: Path) -> dict:
    with open(path) as f:
        cfg = yaml.safe_load(f)
    head = cfg["head"]
    with open(REPO / head["calibration"]) as f:
        head["cal"] = json.load(f)
    for m in ("head_pan", "head_tilt"):
        if m not in head["cal"]:
            raise ValueError(f"{head['calibration']}: no {m}")
    arm = cfg.get("arm")
    if arm:
        with open(REPO / arm["calibration"]) as f:
            arm["cal"] = json.load(f)
        missing = [j for j in ARM_JOINTS if j not in arm["cal"]]
        if missing:
            raise ValueError(f"{arm['calibration']}: no {missing}")
    return cfg


def head_limits(cal: dict, margin: float) -> dict:
    """Degrees around the calibrated centre, `margin` inside each stop (head.py limits())."""
    out = {}
    for m, c in cal.items():
        lo = ft.raw_to_deg(c["range_min"], c["range_min"], c["range_max"])
        hi = ft.raw_to_deg(c["range_max"], c["range_min"], c["range_max"])
        out[m] = (lo + margin, hi - margin)
    return out


def homing_register(offset: int) -> int:
    """Homing_Offset as the servo stores it: sign-magnitude on bit 11."""
    return abs(offset) | (1 << 11 if offset < 0 else 0)


class CalibrationMismatch(ft.BusError):
    """The servos answer, but not as the calibration file says: never retried into."""


class Head:
    """Commands in, goal frames out, on whatever bus `opener` returns. `now` is passed in so
    the tests can run it on a fake clock."""

    def __init__(self, cfg: dict, opener=None, holders_of=holders):
        h = cfg["head"]
        self.port, self.cal = cfg["port"], h["cal"]
        self.ids = {m: int(self.cal[m]["id"]) for m in ("head_pan", "head_tilt")}
        self.sign = {"head_pan": float(h.get("pan_sign", 1.0)), "head_tilt": float(h.get("tilt_sign", 1.0))}
        self.max_dps = float(h.get("max_dps", 90.0))
        self.pid = h.get("pid") or {"p": 16, "i": 0, "d": 32}
        self.lim = head_limits(self.cal, float(h.get("margin_deg", 2.0)))
        self.opener = opener or (lambda port, timeout_s=0.02: ft.Bus(port, timeout_s=timeout_s))
        self.holders_of = holders_of
        self.bus = None
        self.target = None      # degrees, where the operator wants it
        self.goal = None        # degrees, slewed toward target, what was last sent
        self.pos = None         # degrees, last read
        self.pos_at = -1e9
        self.next_open = 0.0
        self.next_peek = 0.0
        self.open_fail_since = None
        self.read_fail_since = None
        self.goal_sent_at = -1e9
        self.state, self.message = "idle", ""

    # -- conversions: the sim's radians <-> head.py's degrees -----------------------------------
    def to_deg(self, m, rad):
        lo, hi = self.lim[m]
        return max(lo, min(hi, self.sign[m] * math.degrees(rad)))

    def to_rad(self, m, deg):
        return math.radians(deg) * self.sign[m]

    def command(self, joints: dict, now: float):
        """joints: {"head_pan_joint": rad, ...}; unknown names and non-numbers are ignored."""
        tgt = dict(self.target or {})
        for name, rad in joints.items():
            m = HEAD_JOINTS.get(name)
            try:
                rad = float(rad)
            except (TypeError, ValueError):
                continue
            if m and math.isfinite(rad):
                tgt[m] = self.to_deg(m, rad)
        if tgt:
            self.target = tgt
        if self.bus is None and now >= self.next_open:
            self._open(now)

    def _open(self, now):
        self.next_open = now + RETRY_OPEN_S
        held = self.holders_of(self.port)
        if held:
            self.state, self.message = "busy", busy_message(f"The arm board {self.port}", held)
            return
        bus = self.opener(self.port, timeout_s=PEEK_TIMEOUT_S)
        try:
            bus.open()
            time.sleep(PEEK_SETTLE_S)  # the same first-read-after-open loss the peek hit
            self._configure(bus)
        except (OSError, ft.BusError) as e:
            try:
                bus.close()
            except OSError:
                pass
            if isinstance(e, ft.BusError) and not isinstance(e, CalibrationMismatch):
                # A link dropout: try again soon, and only call it an error once it has lasted.
                self.open_fail_since = self.open_fail_since if self.open_fail_since is not None else now
                if now - self.open_fail_since < OPEN_GIVE_UP_S:
                    self.next_open = now + OPEN_RETRY_FAST_S
                    self.state, self.message = "connecting", f"arm board not answering yet: {e}"
                    return
            self.open_fail_since = None
            self.state, self.message = "error", f"arm board {self.port}: {e}"
            return
        self.bus, self.open_fail_since, self.read_fail_since = bus, None, None
        self.state, self.message = "holding", ""

    def _configure(self, bus):
        for m, sid in self.ids.items():
            c = self.cal[m]
            if bus.read(sid, ft.MODEL_NUMBER) != ft.STS3215_MODEL:
                raise ft.BusError(f"{m} (id {sid}) is not an STS3215")
            # lerobot's is_calibrated: the registers must be the ones the file was made with.
            got = (bus.read(sid, ft.MIN_POSITION_LIMIT), bus.read(sid, ft.MAX_POSITION_LIMIT),
                   bus.read(sid, ft.HOMING_OFFSET))
            want = (c["range_min"], c["range_max"], homing_register(c["homing_offset"]))
            if got != want:
                raise CalibrationMismatch(f"{m} (id {sid}) registers {got} don't match the calibration {want}: "
                                  f"servo swapped or re-homed? Recalibrate (head.py calibrate)")
        for m, sid in self.ids.items():
            # head.py keys: configure_motors, position mode, the follower's gains, all with
            # torque off (the EEPROM lock opens with torque off). lerobot rewrites them on
            # every connect; here only a value that differs is written, since EEPROM wears.
            bus.write(sid, ft.TORQUE_ENABLE, 0)
            bus.write(sid, ft.LOCK, 0)
            for reg, val in ((ft.RETURN_DELAY, 0), (ft.MAX_ACCELERATION, 254), (ft.ACCELERATION, 254),
                             (ft.OPERATING_MODE, ft.POSITION_MODE), (ft.P_COEFFICIENT, int(self.pid["p"])),
                             (ft.I_COEFFICIENT, int(self.pid["i"])), (ft.D_COEFFICIENT, int(self.pid["d"]))):
                if bus.read(sid, reg) != val:
                    bus.write(sid, reg, val)
        # Start from where the head is, so torque-on never jerks it.
        pos = self._read(bus)
        bus.sync_write(ft.GOAL_POSITION, {self.ids[m]: self._raw(m, d) for m, d in pos.items()})
        for m, sid in self.ids.items():
            bus.write(sid, ft.TORQUE_ENABLE, 1)
            bus.write(sid, ft.LOCK, 1)
        self.pos, self.goal = pos, dict(pos)
        self.pos_at = time.monotonic()
        # An axis the first command didn't name stays where it is.
        self.target = {**pos, **(self.target or {})}

    def _raw(self, m, deg):
        c = self.cal[m]
        return ft.deg_to_raw(deg, c["range_min"], c["range_max"])

    def _read(self, bus, retries=2):
        out = {}
        for m, sid in self.ids.items():
            c = self.cal[m]
            out[m] = ft.raw_to_deg(bus.read(sid, ft.PRESENT_POSITION, retries=retries), c["range_min"], c["range_max"])
        return out

    def tick(self, now, dt):
        if self.bus is None:
            if self.state == "connecting" and now >= self.next_open:
                self._open(now)
            return
        if self.target is None:
            return
        step = self.max_dps * dt
        moved = False
        for m in self.ids:
            g, t = self.goal[m], self.target[m]
            ng = t if abs(t - g) <= step else g + math.copysign(step, t - g)
            if ng != g:
                self.goal[m], moved = ng, True
        # Re-sent while holding even when unchanged: a goal frame is never acknowledged, and one
        # lost in a link dropout would otherwise leave the head short of where it was sent.
        if moved or now - self.goal_sent_at >= GOAL_RESEND_S:
            self.goal_sent_at = now
            try:
                self.bus.sync_write(ft.GOAL_POSITION, {self.ids[m]: self._raw(m, d) for m, d in self.goal.items()})
            except (OSError, ft.BusError) as e:
                self._lost(now, f"arm board write failed: {e}")

    def poll(self, now):
        """Read where the head is, for /head_state (STATE_HZ, not every tick)."""
        if self.bus is None:
            self._peek(now)
            return
        try:
            # One quick attempt per servo: this runs on the node's only thread, and retried
            # reads through a link dropout held up the goal frames ~300 ms at a time, which
            # made the head lag the pad (2026-11-16). The next poll is 1/STATE_HZ away anyway.
            saved, self.bus.timeout_s = self.bus.timeout_s, HELD_READ_TIMEOUT_S
            try:
                self.pos, self.pos_at = self._read(self.bus, retries=0), now
            finally:
                self.bus.timeout_s = saved
            self.read_fail_since = None
        except OSError as e:
            self._lost(now, f"arm board gone: {e}")
        except ft.BusError as e:
            # A dropout: keep holding (the goal frames are re-sent) unless it has gone on.
            self.read_fail_since = self.read_fail_since if self.read_fail_since is not None else now
            if now - self.read_fail_since >= LINK_LOST_S:
                self._lost(now, f"arm board silent for {LINK_LOST_S:.0f} s: {e}")

    def _peek(self, now):
        """Not holding the board: open it just long enough to read the head's position, so the
        operator view can show the sliders where the head is before anyone moves it. Reads only
        (torque untouched), and never while another process holds the board."""
        if now < self.next_peek:
            return
        self.next_peek = now + PEEK_S
        if self.holders_of(self.port):
            return
        bus = self.opener(self.port, timeout_s=PEEK_TIMEOUT_S)
        try:
            bus.open()
            time.sleep(PEEK_SETTLE_S)
            self.pos, self.pos_at = self._read(bus), now
        except (OSError, ft.BusError):
            pass  # keep the last pose; status() drops it once it is POSE_STALE_S old
        finally:
            try:
                bus.close()
            except OSError:
                pass

    def _lost(self, now, why):
        bus, self.bus = self.bus, None
        try:
            bus.close()
        except OSError:
            pass
        self.state, self.message = "error", why
        self.next_open = now + RETRY_OPEN_S

    def release(self, limp=True):
        """Close the board. limp: torque off first (head.py's own exit; the tilt may settle
        under its weight). The node passes limp=False: hold where it is (see the docstring)."""
        if self.bus is None:
            return
        bus, self.bus = self.bus, None
        try:
            for sid in (self.ids.values() if limp else ()):
                try:
                    bus.write(sid, ft.TORQUE_ENABLE, 0)
                except ft.BusError:
                    pass
        finally:
            bus.close()
        self.state = "idle"

    def status(self, now=None):
        now = time.monotonic() if now is None else now
        s = {"held": self.bus is not None, "state": self.state, "message": self.message}
        if self.pos and now - self.pos_at <= POSE_STALE_S:
            s["pan"] = round(self.to_rad("head_pan", self.pos["head_pan"]), 4)
            s["tilt"] = round(self.to_rad("head_tilt", self.pos["head_tilt"]), 4)
        return s


class Arm:
    """Cartesian jog of the SO-101 on the board Head holds (scripts/arm_cartesian.py's loop):
    jog values in, joint goal frames out. `now` is passed in so tests run it on a fake clock."""

    def __init__(self, cfg: dict, kin=None):
        a = cfg["arm"]
        self.cal = a["cal"]
        self.ids = {j: int(self.cal[j]["id"]) for j in ARM_JOINTS}
        self.hand_mps = float(a.get("hand_mps", 0.05))
        self.tilt_rps = math.radians(float(a.get("tilt_dps", 22.5)))
        self.roll_rps = math.radians(float(a.get("roll_dps", 45.0)))
        self.joint_rps = math.radians(float(a.get("joint_dps", 75.0)))
        self.watchdog_s = float(a.get("watchdog_s", 0.5))
        self.pid = a.get("pid") or {"p": 16, "i": 0, "d": 32}
        margin = float(a.get("margin_deg", 2.0))
        # Calibrated range, `margin` inside each stop, in radians (arm_cartesian's lo/hi).
        lim = head_limits(self.cal, margin)
        self.lo = [math.radians(lim[j][0]) for j in ARM_JOINTS]
        self.hi = [math.radians(lim[j][1]) for j in ARM_JOINTS]
        if kin is None:
            from so101_kin import SO101Kin
            kin = SO101Kin()
        self.kin = kin
        self.engaged = False
        self.q = None            # radians, the goal last sent (robot angles, arm_cartesian's q)
        self.target = None       # (x, y, z, pitch): the hand target the jog integrates
        self.pose = None         # where the tip IS (last read), for arm_state
        self.pose_at = -1e9
        self.jog, self.jog_at = dict.fromkeys(JOG_AXES, 0.0), -1e9
        self.goal_sent_at = -1e9
        self.state, self.message = "idle", ""

    def command(self, jog: dict, now: float):
        clean = {}
        for k in JOG_AXES:
            try:
                v = float(jog.get(k, 0.0))
            except (TypeError, ValueError):
                v = 0.0
            clean[k] = max(-1.0, min(1.0, v)) if math.isfinite(v) else 0.0
        self.jog, self.jog_at = clean, now

    def stop(self):
        self.jog_at = -1e9

    def _clip(self, q):
        import numpy as np
        return np.clip(q, self.lo, self.hi)

    def _read_q(self, bus, retries=2):
        import numpy as np
        deg = []
        for j in ARM_JOINTS:
            c = self.cal[j]
            deg.append(ft.raw_to_deg(bus.read(self.ids[j], ft.PRESENT_POSITION, retries=retries),
                                     c["range_min"], c["range_max"]))
        return np.radians(deg)

    def _send(self, bus, q):
        bus.sync_write(ft.GOAL_POSITION, {
            self.ids[j]: ft.deg_to_raw(math.degrees(v), self.cal[j]["range_min"], self.cal[j]["range_max"])
            for j, v in zip(ARM_JOINTS, q)})

    def engage(self, bus):
        """Check the arm against its calibration, configure it like handeye.py's open_bus, set
        the goal to where it is, then torque on: engaging never moves it."""
        for j, sid in self.ids.items():
            c = self.cal[j]
            if bus.read(sid, ft.MODEL_NUMBER) != ft.STS3215_MODEL:
                raise ft.BusError(f"{j} (id {sid}) is not an STS3215")
            got = (bus.read(sid, ft.MIN_POSITION_LIMIT), bus.read(sid, ft.MAX_POSITION_LIMIT),
                   bus.read(sid, ft.HOMING_OFFSET))
            want = (c["range_min"], c["range_max"], homing_register(c["homing_offset"]))
            if got != want:
                raise CalibrationMismatch(f"{j} (id {sid}) registers {got} don't match the arm calibration "
                                          f"{want}: servo swapped or re-homed? Recalibrate")
        for j, sid in self.ids.items():
            bus.write(sid, ft.TORQUE_ENABLE, 0)
            bus.write(sid, ft.LOCK, 0)
            for reg, val in ((ft.RETURN_DELAY, 0), (ft.MAX_ACCELERATION, 254), (ft.ACCELERATION, 254),
                             (ft.OPERATING_MODE, ft.POSITION_MODE), (ft.P_COEFFICIENT, int(self.pid["p"])),
                             (ft.I_COEFFICIENT, int(self.pid["i"])), (ft.D_COEFFICIENT, int(self.pid["d"]))):
                if bus.read(sid, reg) != val:
                    bus.write(sid, reg, val)
            # handeye.py cuts the torque limit for the press; a crash mid-press must not leave
            # the arm weak, so every engage restores it (handeye.py does the same at open).
            bus.write(sid, TORQUE_LIMIT, 1000)
        # The folded rest pose sits a hair outside the margin (shoulder_lift -105.3 against a
        # -105.0 limit): clip what is read, as arm_cartesian does, or IK refuses every move.
        q_read = self._read_q(bus)
        self._send(bus, q_read)
        for sid in self.ids.values():
            bus.write(sid, ft.TORQUE_ENABLE, 1)
            bus.write(sid, ft.LOCK, 1)
        self.q = self._clip(q_read)
        self.target = self.kin.pose(self.q)
        self.pose, self.pose_at = self.kin.pose(q_read), time.monotonic()
        self.engaged = True
        self.state, self.message = "holding", ""

    def tick(self, bus, now, dt):
        """One control step: integrate the jog into a hand target, IK it, send joint goals."""
        import numpy as np
        if not self.engaged:
            return
        moving = now - self.jog_at <= self.watchdog_s and any(self.jog.values())
        if moving:
            j = self.jog
            x, y, z, pitch = self.target
            r, th = math.hypot(x, y), math.atan2(y, x)
            r = max(0.03, r + j["out"] * self.hand_mps * dt)
            th += j["left"] * self.hand_mps * dt / max(r, 0.08)   # left = counter-clockwise from above
            z += j["up"] * self.hand_mps * dt
            pitch += j["tilt"] * self.tilt_rps * dt
            cand = np.array([r * math.cos(th), r * math.sin(th), z, pitch])
            q_try = self.q.copy()
            q_try[4] = min(self.hi[4], max(self.lo[4], q_try[4] + j["roll"] * self.roll_rps * dt))
            q_new, ok = self.kin.ik(cand, q_try, self.lo, self.hi)
            if not ok:
                # can't hold the tool tilt here (wrist_flex on a stop): put the HAND first and
                # let the tilt drift, instead of freezing the whole arm (arm_cartesian.py)
                q_new, ok = self.kin.ik(cand, q_try, self.lo, self.hi, pitch_w=0.001)
                if ok:
                    cand[3] = self.kin.pose(q_new)[3]
            if ok:
                cap = self.joint_rps * dt
                if np.any(np.abs(q_new - self.q) > cap + 1e-9):
                    q_new = self.q + np.clip(q_new - self.q, -cap, cap)
                    cand = self.kin.pose(q_new)
                self.target, self.q = cand, q_new
                self.message = ""
            else:
                self.q = q_try                      # the hand can't go there; roll still can
                self.message = "out of reach"
        if moving or now - self.goal_sent_at >= GOAL_RESEND_S:
            self.goal_sent_at = now
            self._send(bus, self.q)

    def poll(self, bus, now):
        if not self.engaged:
            return
        saved, bus.timeout_s = bus.timeout_s, HELD_READ_TIMEOUT_S
        try:
            self.pose, self.pose_at = self.kin.pose(self._read_q(bus, retries=0)), now
        except ft.BusError:
            pass
        finally:
            bus.timeout_s = saved

    def status(self, now=None):
        now = time.monotonic() if now is None else now
        s = {"engaged": self.engaged, "state": self.state, "message": self.message}
        if self.pose is not None and now - self.pose_at <= POSE_STALE_S:
            x, y, z, p = (float(v) for v in self.pose)
            s["pose"] = {"x": round(x, 4), "y": round(y, 4), "z": round(z, 4), "pitch": round(p, 4)}
        return s


class Magnet:
    """The electromagnet's ESP32 (hardware/magnet_esp32/main.py) on its CP2102, stdlib only:
    scripts/magnet.py's protocol (115200, "on [s]" / "off" / "status" / "ping", replies "ok ..."
    or "err ..."). Every exchange runs on its own thread: a reply can take up to a second, and
    the arm's 50 Hz loop must never wait on it.

    "on" is re-armed every `rearm_s` with `on <on_s>`: the firmware switches off by itself after
    its cap, so a dead process can't leave a package glued on (handeye.py's rearm_if_due)."""

    def __init__(self, cfg: dict):
        m = cfg.get("magnet") or {}
        self.port_glob = m.get("port_glob", "/dev/serial/by-id/usb-Silicon_Labs_CP2102*")
        self.on_s, self.rearm_s = float(m.get("on_s", 30)), float(m.get("rearm_s", 10))
        self.want = None            # "on" / "off" / None (nothing asked yet)
        self.board = None           # "ON" / "OFF" / None: what the ESP32 last said
        self.state, self.message = "idle", ""
        self.q = queue.Queue()
        self.fd = None
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def set(self, on: bool):
        self.want = "on" if on else "off"
        self.q.put(self.want)

    def close(self):
        """Switch off, then stop the thread. Called on shutdown: the magnet never stays on."""
        if self.fd is not None or self.want == "on":
            self.q.put("off")
        self.q.put(None)
        self.thread.join(timeout=3)

    # -- serial, on the thread -----------------------------------------------------------------
    def _open(self):
        port = next(iter(sorted(glob.glob(self.port_glob))), None)
        if not port:
            raise OSError(f"no magnet board ({self.port_glob})")
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            # DTR/RTS reset the ESP32 (wired to EN/GPIO0): drop them. Opening already pulsed
            # them, so it is booting (magnet off, ~2 s): wait for a real pong, as magnet.py does.
            fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS))
        except Exception:
            os.close(fd)
            raise
        self.fd, self.buf = fd, b""
        deadline, last = time.monotonic() + 6.0, 0.0
        while time.monotonic() < deadline:
            if time.monotonic() - last > 0.6:
                os.write(fd, b"ping\n")
                last = time.monotonic()
            line = self._line(0.2)
            if line == "ok pong":
                return
        self._close_fd()
        raise OSError("the magnet's ESP32 never answered ping")

    def _close_fd(self):
        if self.fd is not None:
            fd, self.fd = self.fd, None
            os.close(fd)

    def _line(self, timeout):
        deadline = time.monotonic() + timeout
        while b"\n" not in self.buf:
            left = deadline - time.monotonic()
            if left <= 0 or not select.select([self.fd], [], [], left)[0]:
                return None
            self.buf += os.read(self.fd, 256)
        line, self.buf = self.buf.split(b"\n", 1)
        return line.decode(errors="replace").strip()

    def _ask(self, cmd):
        os.write(self.fd, (cmd + "\n").encode())
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            line = self._line(deadline - time.monotonic())
            if line is None:
                break
            if line.startswith(("ok", "err")) and line not in ("ok ready", "ok pong") \
                    and not line.startswith(("ok timeout", "ok heartbeat")):
                return line
        raise OSError(f"no reply to '{cmd}'")

    def _do(self, cmd):
        if self.fd is None:
            self._open()
        reply = self._ask(f"on {self.on_s:g}" if cmd == "on" else cmd)
        if reply.startswith("err"):
            raise OSError(reply)
        self.board = "ON" if cmd == "on" else "OFF" if cmd == "off" else self.board
        if cmd == "status":
            self.board = "ON" if "on" in reply.lower().split() else "OFF"

    def _run(self):
        next_rearm = None
        while not self._stop.is_set():
            timeout = None if next_rearm is None else max(0.0, next_rearm - time.monotonic())
            try:
                cmd = self.q.get(timeout=timeout)
            except queue.Empty:
                cmd = "on"  # re-arm
            if cmd is None:
                break
            try:
                self._do(cmd)
                self.state, self.message = "ok", ""
            except OSError as e:
                self._close_fd()
                self.board = None
                self.state, self.message = "error", f"magnet: {e}"
            next_rearm = time.monotonic() + self.rearm_s if (cmd == "on" and self.want == "on") else None
        self._close_fd()

    def status(self):
        return {"want": self.want, "board": self.board, "state": self.state, "message": self.message}


def run_node(robot_id: str, cfg: dict):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node(f"arm_driver_{robot_id}")
    log = node.get_logger()
    head = Head(cfg)
    arm = Arm(cfg) if cfg.get("arm") else None
    magnet = Magnet(cfg) if cfg.get("magnet") else None
    steady = Clock(clock_type=ClockType.STEADY_TIME)

    def on_head(msg):
        head.command(dict(zip(msg.name, msg.position)), time.monotonic())

    def on_arm(msg):
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(cmd, dict):
            return
        now = time.monotonic()
        if arm is not None and isinstance(cmd.get("jog"), dict):
            arm.command(cmd["jog"], now)
            if head.bus is None and now >= head.next_open:
                head._open(now)          # the arm shares the head's board: open it for both
        if cmd.get("stop") and arm is not None:
            arm.stop()
        if magnet is not None and cmd.get("magnet") in ("on", "off"):
            if cmd["magnet"] == "on" and not (arm and arm.engaged):
                # handeye.py's refusal: a magnet on a free arm would drop whatever it grabs.
                arm_msg["note"] = "magnet on needs the arm held: move it with the pad first"
            else:
                magnet.set(cmd["magnet"] == "on")

    arm_msg = {"note": ""}
    node.create_subscription(JointState, f"/{robot_id}/head_cmd", on_head, 10)
    node.create_subscription(String, f"/{robot_id}/arm_cmd", on_arm, 10)
    head_pub = node.create_publisher(String, f"/{robot_id}/head_state", 10)
    arm_pub = node.create_publisher(String, f"/{robot_id}/arm_state", 10)
    last = {"t": time.monotonic(), "head": None, "arm": None}

    def logged(key, what, state, message):
        if state != last[key]:
            text = f"{what} {state}{': ' + message if message else ''}"
            # Two call sites, not one with a chosen severity: rclpy raises "Logger severity
            # cannot be changed between calls" for that, which killed this node (2026-11-16).
            if state in ("busy", "error"):
                log.warning(text)
            else:
                log.info(text)
            last[key] = state

    def on_tick():
        now = time.monotonic()
        dt = min(0.1, now - last["t"])
        last["t"] = now
        head.tick(now, dt)
        if arm is not None:
            if head.bus is None:
                arm.engaged = False
                if now - arm.jog_at <= arm.watchdog_s:
                    arm.state, arm.message = head.state, head.message
            elif not arm.engaged and now - arm.jog_at <= arm.watchdog_s:
                try:
                    arm.engage(head.bus)
                except (OSError, ft.BusError) as e:
                    arm.state, arm.message = "error", f"arm: {e}"
                    arm.jog_at = -1e9   # don't retry every tick; the next press tries again
            if arm.engaged:
                try:
                    arm.tick(head.bus, now, dt)
                except (OSError, ft.BusError) as e:
                    arm.state, arm.message = "error", f"arm write failed: {e}"
            logged("arm", "arm", arm.state, arm.message)
        logged("head", "head", head.state, head.message)

    def on_state():
        now = time.monotonic()
        head.poll(now)
        head_pub.publish(String(data=json.dumps(head.status())))
        if arm is not None:
            if head.bus is not None:
                arm.poll(head.bus, now)
            st = arm.status(now)
            st["magnet"] = magnet.status() if magnet else None
            if arm_msg["note"]:
                st["note"], arm_msg["note"] = arm_msg["note"], ""
            arm_pub.publish(String(data=json.dumps(st)))

    # Steady clock: these loops must not stretch with use_sim_time (CLAUDE.md).
    node.create_timer(1.0 / CONTROL_HZ, on_tick, clock=steady)
    node.create_timer(1.0 / STATE_HZ, on_state, clock=steady)
    log.info(f"arm_driver: head {head.ids}" + (f", arm {list(arm.ids.values())}" if arm else "")
             + (", magnet" if magnet else "") + f" on {head.port}")

    def shutdown():
        # Magnet off first, then close the board with everything HOLDING (torque stays on).
        if magnet is not None:
            magnet.close()
        head.release(limp=False)

    def _sig(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sig)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--robot-id", default="robot_0")
    p.add_argument("--config", type=Path, default=REPO / "configs" / "arm_board.yaml")
    args = p.parse_args(argv)
    run_node(args.robot_id, load_config(args.config))
    return 0


if __name__ == "__main__":
    sys.exit(main())
