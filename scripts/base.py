#!/usr/bin/env python3
"""The XLeRobot base: three STS3215 omni wheels (ids 7/8/9) on their OWN servo board, driven in
velocity mode with LeKiwi's kinematics (lerobot/robots/lekiwi). Open loop: a move is a speed for
a time, good to a cm or so on the floor, which is all the pick-and-place's back-off needs.

    scripts/arm.sh base move 0.05        drive 5 cm forward (negative: back)
    scripts/arm.sh base move 0 --turn 10 turn 10 deg on the spot (+ = counter-clockwise)
    scripts/arm.sh base stop

Importable: `Base` below (handeye pick backs off with it while carrying a package).
"""
import argparse
import glob
import math
import os
import time

import numpy as np

WHEELS = {"base_left_wheel": 7, "base_back_wheel": 8, "base_right_wheel": 9}
# LeKiwi's geometry (lerobot/robots/lekiwi/lekiwi.py): wheels at 240/0/120 deg minus 90
WHEEL_RADIUS_M, BASE_RADIUS_M = 0.05, 0.125
ANGLES = np.radians(np.array([240.0, 0.0, 120.0]) - 90.0)
STEPS_PER_DEG = 4096.0 / 360.0
# By USB serial, not ttyACM order: the second board renumbers the first (2026-11-15).
BASE_BOARD_SERIAL = os.environ.get("BASE_BOARD_SERIAL", "5B79016064")
SPEED_MPS, TURN_DPS = 0.05, 20.0


def base_port():
    if os.environ.get("BASE_PORT"):
        return os.environ["BASE_PORT"]
    hits = sorted(glob.glob(f"/dev/serial/by-id/*{BASE_BOARD_SERIAL}*"))
    return hits[0] if hits else None


def wheel_raw(x, y, theta_dps):
    """Body velocity (m/s forward, m/s left, deg/s CCW) -> Goal_Velocity raw per wheel."""
    m = np.array([[math.cos(a), math.sin(a), BASE_RADIUS_M] for a in ANGLES])
    lin = m @ np.array([x, y, math.radians(theta_dps)])
    degps = np.degrees(lin / WHEEL_RADIUS_M)
    return {n: int(max(-0x7FFF, min(0x7FFF, round(d * STEPS_PER_DEG)))) for n, d in zip(WHEELS, degps)}


class Base:
    def __init__(self, port=None):
        from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus
        port = port or base_port()
        if not port:
            raise RuntimeError(f"no base board (USB serial {BASE_BOARD_SERIAL}) plugged in")
        self.bus = SerialMotorsBus(port=port, motors={n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
                                                      for n, i in WHEELS.items()})
        self.bus.connect(handshake=False)
        self.bus.disable_torque()
        self.bus.set_operating_mode("velocity", list(WHEELS))
        self.bus.enable_torque()
        self.stop()

    def stop(self):
        self.bus.sync_write("Goal_Velocity", dict.fromkeys(WHEELS, 0), normalize=False, num_retry=5)

    def move(self, dist_m=0.0, turn_deg=0.0, should_stop=lambda: False, tick=None):
        """Drive dist_m straight (+ forward) and/or turn turn_deg (+ CCW) at SPEED_MPS / TURN_DPS,
        then stop. `should_stop()` polled every 20 ms aborts (wheels stop); `tick()` too, for the
        caller's page. Returns True if it ran the whole way."""
        t_lin = abs(dist_m) / SPEED_MPS
        t_rot = abs(turn_deg) / TURN_DPS
        dur = max(t_lin, t_rot)
        if dur <= 0:
            return True
        x = dist_m / dur
        th = turn_deg / dur
        try:
            self.bus.sync_write("Goal_Velocity", wheel_raw(x, 0.0, th), normalize=False, num_retry=2)
            t0 = time.monotonic()
            while time.monotonic() - t0 < dur:
                if should_stop():
                    return False
                if tick:
                    tick()
                time.sleep(0.02)
            return True
        finally:
            self.stop()

    def close(self):
        try:
            self.stop()
        finally:
            self.bus.disconnect(disable_torque=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    sub = ap.add_subparsers(dest="cmd", required=True)
    mv = sub.add_parser("move")
    mv.add_argument("dist_m", type=float, help="metres, + forward")
    mv.add_argument("--turn", type=float, default=0.0, help="degrees, + counter-clockwise")
    sub.add_parser("stop")
    a = ap.parse_args(argv)
    b = Base(a.port)
    try:
        if a.cmd == "move":
            b.move(a.dist_m, a.turn)
    finally:
        b.close()


if __name__ == "__main__":
    main()
