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
    /<id>/arm_cmd     std_msgs/String (JSON)  {"jog": {"out","left","up","tilt","wf","roll"},
                                              "level": 0..3}: each axis -1..1 (proportional, the
                                              touchpad's deflection), re-sent every ~80 ms while a
                                              finger is down (the deadman); {"magnet": "on"|"off"};
                                              {"stop": true}
    /<id>/arm_state   std_msgs/String (JSON)  "engaged", "state"/"message", "pose" (the tip in
                                              base_link: x, y, z m, pitch rad), "magnet"
    /<id>/pick_request std_msgs/String (JSON) {"cmd": "auto"} the one-button pick and place
                                              (scripts/pick_core.py, handeye.py's PICK);
                                              {"cmd": "pick_hold"} the pick without the place:
                                              it holds the package and hands the arm back to
                                              the pad; {"cmd": "save_place"} saves where the
                                              arm is now as the place pose;
                                              {"cmd": "unload"} the package in the body out
                                              onto the table pose (pick_core.unload), and
                                              {"cmd": "save_table"} saves that pose;
                                              {"cmd": "rest"} back to the taught rest pose,
                                              releasing the magnet first if it's on;
                                              {"cmd": "cancel"} holds the arm where it is
    /<id>/pick_status  std_msgs/String (JSON, latched) phase/busy/note/result, the keys the
                                              sim's magnet_pick.py publishes
    /<id>/pick_trace   std_msgs/String (JSON)  during a pick only, at the state rate: "phase",
                                              "goal_deg" / "read_deg" (the arm's last goal and
                                              last read, no extra bus traffic) and "pose" (the
                                              goal's tip, as arm_state's). arm_state goes quiet
                                              while the pick owns the board, so without this a
                                              bag of a failed pick had no arm at all (2026-11-20)
    /<id>/estop        std_msgs/Bool           true latches the e-stop (nodes/base_driver.py
                                              brakes the wheels): a running pick is aborted
                                              (the arm holds, the magnet is NOT dropped),
                                              and arm jogs and picks are refused until false
    /<id>/robot_mode_cmd std_msgs/String (JSON) {"mode": "nav" | "manip"}
    /<id>/robot_mode   std_msgs/String (JSON)  {"mode", "mapping": running|paused|unavailable}

Nav and manipulation are robot states. Manipulation pauses RTAB-Map's mapping (a full core on
the Orin, 2026-11-16, and the robot isn't exploring while it picks); nav resumes it on the same
map. Odometry runs in both: the place backs the base off, and the robot must stay localized.
A pick or rest switches to manipulation by itself; nav is refused while one runs. Nav also
holds the head at head.nav_pose and ignores /head_cmd: the camera's TF is a static mount, so
a head turned while mapping would put the map's walls in the wrong place.

During a pick the pads are paused (their goal re-sends would fight it) and any arm pad touch
aborts it, as any key did on handeye's page. An abort holds the arm and never drops the magnet.

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
import dataclasses
import fcntl
import glob
import json
import logging
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
MAGNET_PROBE_S = 5.0
PEEK_S = 1.0  # while not holding the board: read where the head is this often, torque untouched
POSE_STALE_S = 5.0  # a pose older than this isn't reported (a failed peek keeps the last one)
# The arm board's link dropped out for ~1-2 s at a time (2026-11-16: ~25% of reads lost in
# bursts, through lerobot too). The cause was the arm's half-seated servo cable loading the
# shared line; reseated, 0 of 1488 were lost. Still ridden through, in case it comes back:
# opening retries fast for a while, and a held board is dropped only after this much silence.
LINK_LOST_S, OPEN_RETRY_FAST_S, OPEN_GIVE_UP_S = 5.0, 0.3, 6.0
GOAL_RESEND_S = 0.2
REST_RELEASE_S = 0.6  # Rest with the magnet on: release, wait this long for the package to drop
HELD_READ_TIMEOUT_S = 0.015  # a reply takes ~0.4 ms when it comes
# Every other peek, and the first head command, failed live (2026-11-16): the first read right
# after opening the port went unanswered. A short settle after open, and a longer reply wait
# than the 20 ms default, on every open of this board.
PEEK_SETTLE_S, PEEK_TIMEOUT_S = 0.03, 0.05
HEAD_JOINTS = {"head_pan_joint": "head_pan", "head_tilt_joint": "head_tilt"}
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
JOG_AXES = ("out", "left", "up", "tilt", "wf", "roll")
# handeye.py's speed buttons L1..L4 (arm_xpad.LEVELS' first four), multiplying the base speeds.
LEVELS = (0.25, 0.5, 0.75, 1.0)
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
        # Where nav mode holds the camera, in /head_cmd's radians: the static base_link ->
        # camera_link mount (configs/sensors/d435_real.yaml) is what mapping assumes.
        nav = h.get("nav_pose") or {}
        self.nav = {"head_pan_joint": math.radians(float(nav.get("pan_deg", 0.0))),
                    "head_tilt_joint": math.radians(float(nav.get("tilt_deg", 0.0)))}
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
        # The board as the operator view shows it (feetech.probe's words), from the peek while
        # not holding it: unplugged or unpowered showed only once a press failed (2026-11-19).
        # The arm's ids are pinged too: one board, and a dead arm cable is worth saying.
        arm_cal = (cfg.get("arm") or {}).get("cal") or {}
        self.arm_ids = [int(c["id"]) for c in arm_cal.values() if isinstance(c, dict) and "id" in c]
        self.board, self.missing = "unknown", []

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

    def hold_nav(self, now):
        """Nav mode: aim at the nav pose and keep it there. Called at STATE_HZ, so a board
        plugged in (or freed by another process) later is opened and aimed by itself."""
        want = {HEAD_JOINTS[n]: self.to_deg(HEAD_JOINTS[n], rad) for n, rad in self.nav.items()}
        if self.bus is None or self.target != want:
            self.command(self.nav, now)

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
            elif self.state == "error" and self.target is not None and self.board == "ok" \
                    and now >= self.next_open:
                self._open(now)       # the board came back (the peek saw it): hold again
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
            self.board, self.missing = "ok", []
        except OSError as e:
            self.board = "no_usb"
            self._lost(now, f"arm board gone: {e}")
        except ft.BusError as e:
            # A dropout: keep holding (the goal frames are re-sent) unless it has gone on.
            self.read_fail_since = self.read_fail_since if self.read_fail_since is not None else now
            if now - self.read_fail_since >= LINK_LOST_S:
                self.board = "no_power"
                self._lost(now, f"arm board silent for {LINK_LOST_S:.0f} s: {e}")

    def _peek(self, now):
        """Not holding the board: open it just long enough to read the head's position, so the
        operator view can show the sliders where the head is before anyone moves it. Reads only
        (torque untouched), and never while another process holds the board."""
        if now < self.next_peek:
            return
        self.next_peek = now + PEEK_S
        if self.holders_of(self.port):
            self.board = "busy"
            return
        bus = self.opener(self.port, timeout_s=PEEK_TIMEOUT_S)
        try:
            bus.open()
            time.sleep(PEEK_SETTLE_S)
            missing = []
            try:
                self.pos, self.pos_at = self._read(bus), now
            except ft.BusError:
                missing += list(self.ids.values())  # keep the last pose; status() ages it out
            for sid in self.arm_ids:
                try:
                    bus.read(sid, ft.MODEL_NUMBER, retries=0)
                except ft.BusError:
                    missing.append(sid)
            n = len(self.ids) + len(self.arm_ids)
            self.board = "no_power" if len(missing) == n else "partial" if missing else "ok"
            self.missing = missing
        except OSError:
            self.board, self.missing = "no_usb", []
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
        s = {"held": self.bus is not None, "state": self.state, "message": self.message,
             "board": "ok" if self.bus is not None and self.read_fail_since is None else self.board,
             "missing": self.missing}
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
        # handeye.py's jog_cfg, scaled by the speed level (default L2, x0.5: positioning, not racing)
        self.hand_mps = float(a.get("hand_mps", 0.10))
        self.tilt_rps = math.radians(float(a.get("tilt_dps", 45.0)))
        self.roll_rps = math.radians(float(a.get("roll_dps", 90.0)))
        self.joint_rps = math.radians(float(a.get("joint_dps", 150.0)))
        self.level = max(0, min(len(LEVELS) - 1, int(a.get("level", 1))))
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

    def command(self, jog: dict, now: float, level=None):
        if level is not None:
            try:
                self.level = max(0, min(len(LEVELS) - 1, int(level)))
            except (TypeError, ValueError):
                pass
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
            # arm_xpad.cartesian_step, which handeye.py's touchpad drives (that module imports
            # evdev at the top, which the robot container doesn't have, hence the copy).
            j, k = self.jog, LEVELS[self.level]
            if j["wf"]:
                # direct wrist_flex. Which way "up" is depends on the motor sign: probe it at the
                # zero pose, NOT at q (tool pitch peaks with the magnet straight down, so a probe
                # at q flips sign there and the wrist dithered in place, live 2026-11-10).
                ref = np.zeros(5)
                probe = ref.copy()
                probe[3] += 1e-3
                up = 1.0 if self.kin.pose(probe)[3] > self.kin.pose(ref)[3] else -1.0
                self.q = self.q.copy()
                self.q[3] = min(self.hi[3], max(self.lo[3], self.q[3] + up * j["wf"] * self.tilt_rps * k * dt))
                self.target = self.kin.pose(self.q)   # the hand moves with the joint
            x, y, z, pitch = self.target
            r, th = math.hypot(x, y), math.atan2(y, x)
            r = max(0.03, r + j["out"] * self.hand_mps * k * dt)
            th += j["left"] * self.hand_mps * k * dt / max(r, 0.08)   # left = counter-clockwise from above
            z += j["up"] * self.hand_mps * k * dt
            pitch += j["tilt"] * self.tilt_rps * k * dt
            cand = np.array([r * math.cos(th), r * math.sin(th), z, pitch])
            q_try = self.q.copy()
            q_try[4] = min(self.hi[4], max(self.lo[4], q_try[4] + j["roll"] * self.roll_rps * k * dt))
            q_new, ok = self.kin.ik(cand, q_try, self.lo, self.hi)
            if not ok:
                # can't hold the tool tilt here (wrist_flex on a stop): put the HAND first and
                # let the tilt drift, instead of freezing the whole arm (arm_cartesian.py)
                q_new, ok = self.kin.ik(cand, q_try, self.lo, self.hi, pitch_w=0.001)
                if ok:
                    cand[3] = self.kin.pose(q_new)[3]
            if ok:
                cap = self.joint_rps * k * dt
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
        s["level"] = self.level
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

    def ask(self, cmd, timeout=3.0):
        """Run cmd ("on" / "off" / "status") on the magnet thread and wait for it: the pick
        needs the board's own answer. -> "ON" / "OFF", or None when the board can't say."""
        if cmd in ("on", "off"):
            self.want = cmd
        done = threading.Event()
        self.q.put((cmd, done))
        if not done.wait(timeout) or self.state == "error":
            return None
        return self.board

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
            # Idle, it asks the board's status every MAGNET_PROBE_S: an unplugged or re-plugged
            # magnet shows on the page by itself, not only at the next press (2026-11-19).
            timeout = MAGNET_PROBE_S if next_rearm is None else max(0.0, next_rearm - time.monotonic())
            try:
                cmd = self.q.get(timeout=timeout)
            except queue.Empty:
                cmd = "on" if next_rearm is not None else "status"  # re-arm, or the idle check
            if cmd is None:
                break
            done = None
            if isinstance(cmd, tuple):
                cmd, done = cmd
            try:
                self._do(cmd)
                self.state, self.message = "ok", ""
            except OSError as e:
                self._close_fd()
                self.board = None
                self.state, self.message = "error", f"magnet: {e}"
            if done is not None:
                done.set()
            if cmd in ("on", "off"):
                next_rearm = time.monotonic() + self.rearm_s if (cmd == "on" and self.want == "on") else None
        self._close_fd()

    def status(self):
        return {"want": self.want, "board": self.board, "state": self.state, "message": self.message}


def _json(text):
    try:
        v = json.loads(text)
    except ValueError:
        return None
    return v if isinstance(v, dict) else None


def run_node(robot_id: str, cfg: dict):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node(f"arm_driver_{robot_id}")
    log = node.get_logger()

    class _ToRosLog(logging.Handler):
        """pick_core's logging (the "seated ... magnet pitch" line and the like) into this node's
        log: unconfigured, Python dropped every INFO line, so a failed pick left only "pick:
        starting auto" behind (2026-11-20). Two call sites: rclpy won't change a site's severity."""
        def emit(self, record):
            if record.levelno >= logging.WARNING:
                log.warning(f"pick_core: {record.getMessage()}")
            else:
                log.info(f"pick_core: {record.getMessage()}")
    pick_log = logging.getLogger("pick_core")
    pick_log.addHandler(_ToRosLog())
    pick_log.setLevel(logging.INFO)
    pick_log.propagate = False
    head = Head(cfg)
    arm = Arm(cfg) if cfg.get("arm") else None
    magnet = Magnet(cfg) if cfg.get("magnet") else None
    steady = Clock(clock_type=ClockType.STEADY_TIME)

    def on_head(msg):
        if mode["mode"] == "nav":
            return                     # nav holds the camera at the mount mapping assumes
        head.command(dict(zip(msg.name, msg.position)), time.monotonic())

    def on_arm(msg):
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(cmd, dict):
            return
        now = time.monotonic()
        if estop["on"]:
            if arm is not None:
                arm.stop()
            if magnet is not None and cmd.get("magnet") == "off":
                magnet.set(False)        # letting go is never blocked
            elif cmd.get("jog") or cmd.get("magnet") == "on":
                arm_msg["note"] = "e-stop is latched: release it to move the arm"
            return
        if pick["thread"] is not None:
            # handeye's page: any key aborts a pick. A pad touch or the stop button does here.
            if (isinstance(cmd.get("jog"), dict) and any(cmd["jog"].values())) or cmd.get("stop"):
                pick["abort"].set()
            return
        if arm is not None and isinstance(cmd.get("jog"), dict):
            arm.command(cmd["jog"], now, cmd.get("level"))
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
    estop = {"on": False}

    def on_estop(msg):
        on = bool(msg.data)
        if on != estop["on"]:
            log.warning(f"e-stop {'LATCHED: pick aborted, arm holding' if on else 'released'}")
        estop["on"] = on
        if on:
            pick["abort"].set()          # pick_core holds where it is; the magnet stays as it is
            if arm is not None:
                arm.stop()

    from std_msgs.msg import Bool
    node.create_subscription(Bool, f"/{robot_id}/estop", on_estop, 10)

    # -- the one-button pick (nodes/pick_ros.py around scripts/pick_core.py) ------------------
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import CameraInfo, Image
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    pick_pub = node.create_publisher(String, f"/{robot_id}/pick_status", latched)
    vel_pub = node.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)
    base_state = {"st": None}
    node.create_subscription(String, f"/{robot_id}/base_status",
                             lambda m: base_state.__setitem__("st", _json(m.data)), 10)
    pick = {"thread": None, "abort": threading.Event(), "subs": [], "done": False, "robot": None, "last": None}

    def pick_status(phase, note="", busy=False, **extra):
        mag = magnet.board if magnet else None
        msg = {"phase": phase, "busy": busy, "mode": "manipulation" if busy else "idle", "marker": 10,
               "note": note, "cmd": "auto", "target": None, "magnet": extra.pop("magnet", mag), **extra}
        pick["last"] = String(data=json.dumps(msg, default=str))
        pick_pub.publish(pick["last"])

    pick_status("IDLE", "ready" if (arm and magnet) else "no arm or magnet configured")

    def publish_vel(x):
        t = Twist()
        t.linear.x = float(x)
        vel_pub.publish(t)

    def start_pick(action="auto"):
        """action "auto": the one-button pick and place. "rest": handeye's Go to rest (the old
        page's reset button), the same thread and pausing, so a pad touch interrupts it too."""
        import pick_core
        import pick_ros
        if pick["thread"] is not None:
            return pick_status("BUSY", "a pick is already running", busy=True)
        if arm is None or magnet is None:
            return pick_status("FAILED", "the pick needs the arm and the magnet (configs/arm_board.yaml)")
        now = time.monotonic()
        if head.bus is None:
            head._open(now)
        if head.bus is None:
            return pick_status("FAILED", f"arm board not available: {head.message}")
        if pick["robot"] is None:
            try:
                pick["robot"] = pick_core.load_robot()
            except (OSError, RuntimeError, ValueError) as e:
                return pick_status("FAILED", f"calibration: {e}")
        kin, hc, poses = pick["robot"]
        io = pick_ros.BusArmIO(head.bus, head.cal, arm.cal)
        pick["io"] = io
        cam = pick_ros.RosCamera(io)
        pick["subs"] = [
            node.create_subscription(Image, f"/{robot_id}/camera/rgb", cam.on_rgb, qos_profile_sensor_data),
            node.create_subscription(Image, f"/{robot_id}/camera/depth", cam.on_depth, qos_profile_sensor_data),
            node.create_subscription(CameraInfo, f"/{robot_id}/camera/camera_info", cam.on_info, qos_profile_sensor_data),
        ]
        abort = pick["abort"]
        base = pick_ros.CmdVelBase(publish_vel, lambda: base_state["st"], abort)

        said = {"text": ""}

        def report(phase, text, **extra):
            if text:
                said["text"] = text
                log.info(f"pick {phase}: {text}")
            busy = phase in pick_core.PickCore.MOVING_FLAGS or phase in ("HELD", "LOCKED", "PLANNED", "HOVERING", "HOLDING")
            pick_status(phase, text, busy=True if pick["thread"] else busy, **{k: v for k, v in extra.items() if k in ("state", "magnet")})

        # configs/arm_board.yaml `pick:` overrides handeye's defaults (PickConfig's fields only).
        fields = {f.name for f in dataclasses.fields(pick_core.PickConfig)}
        over = {k: v for k, v in (cfg.get("pick") or {}).items() if k in fields}
        core = pick_core.PickCore(io, cam, pick_ros.MagnetIO(magnet), base, kin, hc, poses,
                                  cfg=pick_core.PickConfig(**over), abort=abort, report=report)

        def run():
            result, note = "failed", ""
            try:
                if action == "rest":
                    # Rest releases first, then folds: whatever the magnet holds drops where the
                    # hand is (chosen 2026-11-16 over handeye's refusal). A fresh core hasn't asked
                    # the board yet, hence the poll; and the fold waits for the board to CONFIRM
                    # off, so a magnet that didn't let go is never carried into the folded pose.
                    core.mag.poll()
                    if core.mag.on:
                        if core.mag.set(False) is not False:
                            core.say("magnet did not confirm OFF: not folding with it maybe still holding", 1e9)
                            raise_result = "failed"
                        else:
                            time.sleep(REST_RELEASE_S)   # let the package fall clear first
                            raise_result = None
                    else:
                        raise_result = None
                    result = raise_result or ("rest" if core.go_rest() else ("aborted" if abort.is_set() else "failed"))
                elif action == "unload":
                    core.start()
                    result = core.unload()
                else:
                    core.start()
                    result = core.auto_pick(place=action != "pick_hold")
                    if action == "pick_hold" and result == "picked":
                        core.say("holding it: drive the arm to the place spot with the pad, then Save place", 1e9)
            except Exception as e:   # the thread must always end in a status, never silently
                note = f"{type(e).__name__}: {e}"
                log.error(f"pick crashed: {note}")
            # The pick's own last words say WHY ("magnet is ON: release before going to rest");
            # a bare "failed" in their place told the operator nothing (2026-11-16).
            pick_status(result.upper(), note or said["text"] or result, busy=False, result=result)
            pick["done"] = True

        pick["done"] = False
        abort.clear()
        pick["thread"] = threading.Thread(target=run, daemon=True)
        log.info(f"pick: starting {action}")
        pick_status("STARTING", {"rest": "going to rest", "unload": "unloading onto the table"}.get(action, "pick and place"),
                    busy=True)
        pick["thread"].start()

    # -- nav / manipulation mode --------------------------------------------------------------
    from std_srvs.srv import Empty
    pause_cli = node.create_client(Empty, f"/{robot_id}/rtabmap/pause")
    resume_cli = node.create_client(Empty, f"/{robot_id}/rtabmap/resume")
    mode_pub = node.create_publisher(String, f"/{robot_id}/robot_mode", latched)
    mode = {"mode": "nav", "mapping": "unknown", "applied": None, "note": ""}

    def publish_mode():
        mode_pub.publish(String(data=json.dumps({k: mode[k] for k in ("mode", "mapping", "note")})))

    def apply_mode():
        """Make RTAB-Map match the mode. Re-run every second, so a mapping node that restarted,
        or wasn't up yet when the mode was set, ends up paused or running as it should."""
        want = "paused" if mode["mode"] == "manip" else "running"
        if mode["applied"] == want:
            return
        cli = pause_cli if want == "paused" else resume_cli
        if not cli.service_is_ready():
            if mode["mapping"] != "unavailable":
                mode["mapping"] = "unavailable"
                publish_mode()
            return
        mode["applied"] = want   # one call in flight at a time; a failure clears it to retry

        def done(fut):
            if fut.exception() is None:
                mode["mapping"] = want
                log.info(f"mapping {want} ({mode['mode']} mode)")
            else:
                mode["applied"] = None
            publish_mode()
        cli.call_async(Empty.Request()).add_done_callback(done)

    def set_mode(m):
        if m not in ("nav", "manip"):
            return
        if m == "nav" and pick["thread"] is not None:
            mode["note"] = "a pick is running: nav mode once it's done (or touch the arm pad to stop it)"
            return publish_mode()
        mode["note"] = ""
        if m != mode["mode"]:
            mode["mode"], mode["applied"] = m, None
            log.info(f"{m} mode")
        apply_mode()
        publish_mode()

    node.create_subscription(String, f"/{robot_id}/robot_mode_cmd",
                             lambda msg: set_mode((_json(msg.data) or {}).get("mode")), 10)
    publish_mode()

    def on_pick_request(msg):
        cmd = _json(msg.data) or {}
        if cmd.get("cmd") in ("auto", "rest", "pick_hold", "unload") and estop["on"]:
            return pick_status("FAILED", "e-stop is latched: release it first")
        if cmd.get("cmd") in ("auto", "rest", "pick_hold", "unload"):
            set_mode("manip")
            start_pick(cmd["cmd"])
        elif cmd.get("cmd") == "save_place":
            save_pose("pick_place_pose.json", "place")
        elif cmd.get("cmd") == "save_table":
            save_pose("pick_table_pose.json", "table")
        elif cmd.get("cmd") in ("cancel", "stop", "abort"):
            pick["abort"].set()

    def save_pose(name, what):
        """Where the arm is now becomes the place pose (handeye's pick_place_pose.json) or
        Unload's table pose (pick_table_pose.json), in the repo's calibration copy that
        load_robot reads (commit it to keep it). The next pick loads it fresh."""
        import pick_core
        import pick_ros
        if pick["thread"] is not None:
            return pick_status("BUSY", f"a pick is running: save the {what} pose once it's done", busy=True)
        if arm is None:
            return pick_status("FAILED", "no arm configured")
        now = time.monotonic()
        if head.bus is None:
            head._open(now)
        if head.bus is None:
            return pick_status("FAILED", f"arm board not available: {head.message}")
        try:
            deg = pick_ros.BusArmIO(head.bus, head.cal, arm.cal).read()
        except (OSError, ft.BusError) as e:
            return pick_status("FAILED", f"could not read the arm: {e}")
        path = pick_core.CALIBRATION_DIR / "robots" / "sortbots_head" / name
        rec = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "arm_deg": [round(deg[j], 4) for j in pick_ros.ARM]}
        try:
            path.write_text(json.dumps(rec, indent=1))
        except OSError as e:
            return pick_status("FAILED", f"could not save the {what} pose: {e}")
        pick["robot"] = None          # the next pick reloads the poses
        log.info(f"{what} pose saved: {rec['arm_deg']}")
        pick_status("SAVED", f"{what} pose saved: release the magnet, then Rest", busy=False)

    node.create_subscription(String, f"/{robot_id}/pick_request", on_pick_request, 10)

    def pick_finished():
        """Back from a pick: drop the camera, and let the pads continue from where the arm and
        head ARE (the pick moved them), so the next pad touch never snaps them back."""
        for sub in pick["subs"]:
            node.destroy_subscription(sub)
        pick["subs"], pick["thread"] = [], None
        if head.bus is not None:
            try:
                pos = head._read(head.bus)
                head.pos, head.goal, head.target = pos, dict(pos), dict(pos)
                if arm is not None:
                    q = arm._read_q(head.bus)
                    arm.q, arm.target, arm.engaged = arm._clip(q), arm.kin.pose(arm._clip(q)), True
                    arm.state = "holding"
            except (OSError, ft.BusError):
                pass
    node.create_subscription(JointState, f"/{robot_id}/head_cmd", on_head, 10)
    node.create_subscription(String, f"/{robot_id}/arm_cmd", on_arm, 10)
    head_pub = node.create_publisher(String, f"/{robot_id}/head_state", 10)
    arm_pub = node.create_publisher(String, f"/{robot_id}/arm_state", 10)
    trace_pub = node.create_publisher(String, f"/{robot_id}/pick_trace", 10)
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
        if pick["thread"] is not None:
            if pick["done"]:
                pick_finished()
            return                     # the pick owns the arm and head while it runs
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

    def publish_trace():
        io = pick.get("io")
        if io is None or arm is None:
            return
        g = {j: round(float(io.goal[j]), 2) for j in ARM_JOINTS if j in io.goal}
        msg = {"phase": (_json(pick["last"].data) or {}).get("phase") if pick["last"] is not None else None,
               "goal_deg": g, "read_deg": {j: round(float(io.last[j]), 2) for j in ARM_JOINTS if j in io.last}}
        if len(g) == len(ARM_JOINTS):
            import numpy as np
            x, y, z, p = (float(v) for v in arm.kin.pose(np.radians([g[j] for j in ARM_JOINTS])))
            msg["pose"] = {"x": round(x, 4), "y": round(y, 4), "z": round(z, 4), "pitch": round(p, 4)}
        trace_pub.publish(String(data=json.dumps(msg)))

    def on_state():
        now = time.monotonic()
        # Latched, and re-sent at 1 Hz too: rosbridge doesn't hand a latched message to a page
        # that subscribes later, so without this the operator view never saw the pick (2026-11-16).
        if now - last.get("pick_pub", 0.0) >= 1.0:
            if pick["last"] is not None:
                pick_pub.publish(pick["last"])
            apply_mode()
            publish_mode()       # latched too, but rosbridge doesn't replay it to a late page
            last["pick_pub"] = now
        if pick["thread"] is not None:
            publish_trace()
            return                     # the pick reads the board itself
        if mode["mode"] == "nav":
            head.hold_nav(now)
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
        # A running pick holds where it is first; then magnet off, then close the board with
        # everything HOLDING (torque stays on).
        if pick["thread"] is not None:
            pick["abort"].set()
            pick["thread"].join(timeout=3)
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
