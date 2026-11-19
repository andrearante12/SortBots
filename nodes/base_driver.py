#!/usr/bin/env python3
"""The real robot's omni base on /<id>/cmd_vel: the same Twist the sim's base takes, so the
operator view's drive pad (and later Nav2 and the explorer) drive either platform unchanged.

    python3 nodes/base_driver.py --robot-id robot_0     the node (platform:=real bringup)
    python3 nodes/base_driver.py --stop                 wheels to 0 + torque off, then exit
                                                        (no ROS; run_robot.sh stop runs it)

Kinematics, clamping and the ramp are drive_bridge.py's (configs/drive_base.yaml, the Robot
view's own numbers); the servo bus is nodes/feetech.py, because lerobot isn't in the robot
container. Publishes /<id>/base_status (JSON) so the operator view can say why the base
doesn't move.

Safety, in the order it bites:
- Deadman. The wheels ramp to 0 when no cmd_vel has arrived for `ros.watchdog_s`: the pad
  republishes at 10 Hz while held, so a closed tab or a dropped link stops the robot.
- Speed. Every command is clamped to `ros.speed_level` of drive_base.yaml's speed_levels
  (default 0, the slowest), whatever the publisher asked for: the pad sends the sim's 0.5 m/s.
- The board is held ONLY while driving. It is opened on the first non-zero command and
  released (wheels 0, torque off) after `ros.idle_release_s` stopped, so the Robot view's
  drive bridge and `handeye pick` can take it the rest of the time. A board some other
  process holds is never opened underneath it; base_status names the holder.
- An STS3215 keeps its last speed if this process is SIGKILLed mid-drive. SIGINT/SIGTERM
  release the wheels, and run_robot.sh stop runs `--stop` after its pkill -9 sweep. The
  base's power switch is the e-stop for everything software can't reach.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(REPO / "scripts"))
import feetech  # noqa: E402
from drive_bridge import (WHEELS, body_to_wheel_raw, busy_message, clamp_cmd,  # noqa: E402
                          holders, load_config, ramp)

CONTROL_HZ = 50
RESEND_S = 0.2       # Goal_Velocity re-sent this often unchanged: a sync write is never acked
RETRY_OPEN_S = 2.0   # a busy or missing board is retried this often while commands come in
STATUS_HZ = 2.0
PROBE_S = 3.0        # not driving: look for the board (plugged in? powered?) this often

ROS_DEFAULTS = {"speed_level": 0, "watchdog_s": 0.5, "idle_release_s": 3.0}


def ros_settings(cfg: dict) -> dict:
    s = {**ROS_DEFAULTS, **(cfg.get("ros") or {})}
    levels = cfg["speed_levels"]
    lvl = max(0, min(len(levels) - 1, int(s["speed_level"])))
    s["max_xy"] = float(levels[lvl]["xy_mps"])
    s["max_w"] = math.radians(float(levels[lvl]["theta_dps"]))
    s["speed_level"] = lvl
    return s


class Drive:
    """Everything but ROS: commands in, wheel frames out. `now` is passed in so tests can
    run it on a fake clock."""

    def __init__(self, cfg: dict, opener=feetech.open_wheels, releaser=feetech.release_wheels,
                 holders_of=holders, prober=feetech.probe):
        self.cfg, self.ros = cfg, ros_settings(cfg)
        self.port = cfg["port"]
        self.ids = [int(cfg["wheels"][w]) for w in WHEELS]
        self.opener, self.releaser, self.holders_of = opener, releaser, holders_of
        self.bus = None
        self.target, self.target_at = (0.0, 0.0, 0.0), -1e9
        self.vel = (0.0, 0.0, 0.0)
        self.raw, self.sent_at = [0, 0, 0], -1e9
        self.stopped_since = None
        self.next_open = 0.0
        self.state, self.message = "idle", ""
        # What the operator view shows for the board, kept current while nothing drives (probe):
        # unplugged or unpowered used to show only once a drive press failed (2026-11-19).
        self.prober, self.board, self.missing, self.next_probe = prober, "unknown", [], 0.0

    def command(self, x, y, w, now):
        self.target = clamp_cmd(x, y, w, self.ros["max_xy"], self.ros["max_w"])
        self.target_at = now

    def _wanted(self, now):
        if now - self.target_at > self.ros["watchdog_s"]:
            return (0.0, 0.0, 0.0)
        return self.target

    def tick(self, now, dt):
        tgt = self._wanted(now)
        moving_wanted = any(abs(v) > 1e-6 for v in tgt)
        if self.bus is None:
            self.vel = (0.0, 0.0, 0.0)
            if moving_wanted and now >= self.next_open:
                self._open(now)
            elif not moving_wanted:
                # busy/error describe the last attempt to drive; with no one asking, idle.
                self.state, self.message = "idle", ""
            return
        a = float(self.cfg.get("accel_mps2", 1.0)) * dt
        al = math.radians(float(self.cfg.get("alpha_dps2", 180))) * dt
        self.vel = (ramp(self.vel[0], tgt[0], a), ramp(self.vel[1], tgt[1], a), ramp(self.vel[2], tgt[2], al))
        raw = body_to_wheel_raw(*self.vel, self.cfg)
        if raw != self.raw or now - self.sent_at >= RESEND_S:
            try:
                self.bus.sync_velocity(dict(zip(self.ids, raw)))
            except (OSError, feetech.BusError) as e:
                self._lost(now, f"wheel board write failed: {e}")
                return
            self.raw, self.sent_at = raw, now
        if any(raw) or moving_wanted:
            self.stopped_since = None
            self.state, self.message = "driving", ""
        else:
            self.stopped_since = self.stopped_since if self.stopped_since is not None else now
            if now - self.stopped_since >= self.ros["idle_release_s"]:
                self.release()
                self.state, self.message = "idle", ""

    def probe(self, now):
        """Not holding the board: is it there and powered? A board that comes back clears the
        last error by itself, so the page stops saying it's gone without a restart."""
        if self.bus is not None:
            self.board, self.missing = "ok", []
            return
        if now < self.next_probe:
            return
        self.next_probe = now + PROBE_S
        if self.holders_of(self.port):
            self.board = "busy"
            return
        was = self.board
        self.board, self.missing = self.prober(self.port, self.ids)
        if self.board == "ok" and was != "ok" and self.state == "error":
            self.state, self.message = "idle", ""
            self.next_open = now      # back: the next press opens it at once

    def _open(self, now):
        self.next_open = now + RETRY_OPEN_S
        held = self.holders_of(self.port)
        if held:
            self.state, self.message = "busy", busy_message(f"The wheel board {self.port}", held)
            return
        try:
            self.bus = self.opener(self.port, self.ids)
        except (OSError, feetech.BusError) as e:
            self.state, self.message = "error", f"wheel board {self.port}: {e}"
            return
        self.raw, self.sent_at, self.stopped_since = [0, 0, 0], -1e9, None
        self.state, self.message = "driving", ""

    def _lost(self, now, why):
        bus, self.bus = self.bus, None
        try:
            bus.close()
        except OSError:
            pass
        self.vel = (0.0, 0.0, 0.0)
        self.state, self.message = "error", why
        self.next_open = now + RETRY_OPEN_S

    def release(self):
        if self.bus is None:
            return
        bus, self.bus = self.bus, None
        try:
            self.releaser(bus, self.ids)
        except OSError:
            pass
        self.raw, self.vel = [0, 0, 0], (0.0, 0.0, 0.0)

    def status(self):
        return {"state": self.state, "message": self.message, "port": self.port,
                "board": self.board, "missing": self.missing,
                "max_mps": self.ros["max_xy"], "max_dps": round(math.degrees(self.ros["max_w"])),
                "speed_level": self.ros["speed_level"], "wheels_raw": self.raw}


def stop_once(cfg: dict) -> int:
    """`--stop`: zero the wheels if the board is free. A holder is left alone (it is the
    one driving them, and it has its own watchdog)."""
    port = cfg["port"]
    ids = [int(cfg["wheels"][w]) for w in WHEELS]
    if not Path(port).exists():
        print(f"[base_driver] no wheel board at {port}: nothing to stop")
        return 0
    held = holders(port)
    if held:
        print(f"[base_driver] {busy_message('The wheel board', held)} Left alone.")
        return 0
    bus = feetech.Bus(port)
    try:
        bus.open()
        feetech.release_wheels(bus, ids)
    except (OSError, feetech.BusError) as e:
        print(f"[base_driver] could not stop the wheels on {port}: {e}")
        return 1
    print(f"[base_driver] wheels {ids} on {port}: 0, torque off")
    return 0


def run_node(robot_id: str, cfg: dict):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.clock import Clock, ClockType
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node(f"base_driver_{robot_id}")
    drive = Drive(cfg)
    log = node.get_logger()
    steady = Clock(clock_type=ClockType.STEADY_TIME)

    def on_cmd(msg):
        drive.command(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())

    node.create_subscription(Twist, f"/{robot_id}/cmd_vel", on_cmd, 10)
    status_pub = node.create_publisher(String, f"/{robot_id}/base_status", 10)
    last = {"t": time.monotonic(), "state": None}

    def on_tick():
        now = time.monotonic()
        drive.tick(now, min(0.1, now - last["t"]))
        last["t"] = now
        if drive.state != last["state"]:
            text = f"base {drive.state}{': ' + drive.message if drive.message else ''}"
            # Two call sites, not one with a chosen severity: rclpy raises "Logger severity
            # cannot be changed between calls" for that, which killed this node (2026-11-16).
            if drive.state in ("busy", "error"):
                log.warning(text)
            else:
                log.info(text)
            last["state"] = drive.state

    def on_status():
        was = drive.board
        drive.probe(time.monotonic())
        if drive.board != was:
            log.info(f"wheel board: {drive.board}{' (no reply from ' + str(drive.missing) + ')' if drive.missing else ''}")
        status_pub.publish(String(data=json.dumps(drive.status())))

    # Steady clock, not the node clock: this loop is the deadman, it must not stretch with
    # use_sim_time (CLAUDE.md, "A ROS timer follows use_sim_time").
    node.create_timer(1.0 / CONTROL_HZ, on_tick, clock=steady)
    node.create_timer(1.0 / STATUS_HZ, on_status, clock=steady)
    log.info(f"base_driver: /{robot_id}/cmd_vel -> wheels {drive.ids} on {drive.port}, "
             f"capped at {drive.ros['max_xy']} m/s, {math.degrees(drive.ros['max_w']):.0f} deg/s")

    def _sig(*_):
        drive.release()
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sig)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        drive.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--robot-id", default="robot_0")
    p.add_argument("--config", type=Path, default=REPO / "configs" / "drive_base.yaml")
    p.add_argument("--stop", action="store_true", help="zero the wheels and exit (no ROS)")
    args = p.parse_args(argv)
    cfg = load_config(args.config)
    if args.stop:
        return stop_once(cfg)
    run_node(args.robot_id, cfg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
