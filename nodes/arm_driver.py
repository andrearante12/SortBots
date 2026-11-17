#!/usr/bin/env python3
"""The real robot's arm board over ROS: the pan/tilt head today, the SO-101 arm next.

    python3 nodes/arm_driver.py --robot-id robot_0     the node (platform:=real bringup)

One process owns the board (configs/arm_board.yaml), because the head and the arm share it
and a servo board takes one owner: anything that moves either goes through here.

    /<id>/head_cmd    sensor_msgs/JointState  head_pan_joint / head_tilt_joint, radians, in
                                              the sim's convention (tilt > 0 looks down), the
                                              same message the operator view's sliders send
    /<id>/head_state  std_msgs/String (JSON)  {"pan", "tilt"} radians where the head IS,
                                              "held", and "state"/"message" when it can't move

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
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(REPO / "scripts"))
import feetech as ft  # noqa: E402
from drive_bridge import busy_message, holders  # noqa: E402

CONTROL_HZ = 50
STATE_HZ = 5
RETRY_OPEN_S = 2.0
PEEK_S = 1.0  # while not holding the board: read where the head is this often, torque untouched
POSE_STALE_S = 5.0  # a pose older than this isn't reported (a failed peek keeps the last one)
# Every other peek failed live (2026-11-16): the first read right after opening the port went
# unanswered. A short settle after open, and a longer reply wait than the 20 ms default.
PEEK_SETTLE_S, PEEK_TIMEOUT_S = 0.03, 0.05
HEAD_JOINTS = {"head_pan_joint": "head_pan", "head_tilt_joint": "head_tilt"}


def load_config(path: Path) -> dict:
    with open(path) as f:
        cfg = yaml.safe_load(f)
    head = cfg["head"]
    with open(REPO / head["calibration"]) as f:
        head["cal"] = json.load(f)
    for m in ("head_pan", "head_tilt"):
        if m not in head["cal"]:
            raise ValueError(f"{head['calibration']}: no {m}")
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
        bus = self.opener(self.port)
        try:
            bus.open()
            self._configure(bus)
        except (OSError, ft.BusError) as e:
            try:
                bus.close()
            except OSError:
                pass
            self.state, self.message = "error", f"arm board {self.port}: {e}"
            return
        self.bus = bus
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
                raise ft.BusError(f"{m} (id {sid}) registers {got} don't match the calibration {want}: "
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

    def _read(self, bus):
        out = {}
        for m, sid in self.ids.items():
            c = self.cal[m]
            out[m] = ft.raw_to_deg(bus.read(sid, ft.PRESENT_POSITION), c["range_min"], c["range_max"])
        return out

    def tick(self, now, dt):
        if self.bus is None or self.target is None:
            return
        step = self.max_dps * dt
        moved = False
        for m in self.ids:
            g, t = self.goal[m], self.target[m]
            ng = t if abs(t - g) <= step else g + math.copysign(step, t - g)
            if ng != g:
                self.goal[m], moved = ng, True
        if moved:
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
            self.pos, self.pos_at = self._read(self.bus), now
        except (OSError, ft.BusError) as e:
            self._lost(now, f"arm board read failed: {e}")

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

    def release(self):
        """Torque off and close (head.py's own exit). The tilt may settle under its weight."""
        if self.bus is None:
            return
        bus, self.bus = self.bus, None
        try:
            for sid in self.ids.values():
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


def run_node(robot_id: str, cfg: dict):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node(f"arm_driver_{robot_id}")
    log = node.get_logger()
    head = Head(cfg)
    steady = Clock(clock_type=ClockType.STEADY_TIME)

    def on_head(msg):
        head.command(dict(zip(msg.name, msg.position)), time.monotonic())

    node.create_subscription(JointState, f"/{robot_id}/head_cmd", on_head, 10)
    pub = node.create_publisher(String, f"/{robot_id}/head_state", 10)
    last = {"t": time.monotonic(), "state": None}

    def on_tick():
        now = time.monotonic()
        head.tick(now, min(0.1, now - last["t"]))
        last["t"] = now
        if head.state != last["state"]:
            text = f"head {head.state}{': ' + head.message if head.message else ''}"
            # Two call sites, not one with a chosen severity: rclpy raises "Logger severity
            # cannot be changed between calls" for that, which killed this node (2026-11-16).
            if head.state in ("busy", "error"):
                log.warning(text)
            else:
                log.info(text)
            last["state"] = head.state

    def on_state():
        head.poll(time.monotonic())
        pub.publish(String(data=json.dumps(head.status())))

    # Steady clock: these loops must not stretch with use_sim_time (CLAUDE.md).
    node.create_timer(1.0 / CONTROL_HZ, on_tick, clock=steady)
    node.create_timer(1.0 / STATE_HZ, on_state, clock=steady)
    log.info(f"arm_driver: /{robot_id}/head_cmd -> head {head.ids} on {head.port}")

    def _sig(*_):
        head.release()
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sig)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        head.release()
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
