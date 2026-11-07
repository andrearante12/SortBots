#!/usr/bin/env python3
"""Joint-space keyboard teleop for the 5-DOF SO-101 follower.

Runs in the `lerobot` conda env, NOT in a ROS shell. Reads raw keys from the terminal
(works over SSH, no evdev, no gamepad). Needs the `sortbots-5dof` lerobot branch and a
finished `lerobot-calibrate` with --robot.id matching --id here.

    q / a   shoulder_pan   + / -
    w / s   shoulder_lift  + / -
    e / d   elbow_flex     + / -
    r / f   wrist_flex     + / -
    t / g   wrist_roll     + / -   (LOCKED by default: --lock with no names unlocks it)
    [ / ]   step size down / up (degrees per keypress, 1 2 5 10)
    space   re-sync targets to where the arm actually is
    x / Esc / Ctrl-C   quit, torque off

A terminal reports key presses, not releases, so each press (and each key-repeat) nudges
the target by one step; nothing moves unless a key is arriving, and the arm holds its pose
otherwise. Targets are clamped to the calibrated range and may lead the present position by
at most --max-step degrees (lerobot's max_relative_target), same safety as arm_gamepad.py.
Start position is the arm's present pose, so connecting never jerks. Flip a joint's
direction with --invert NAME.
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

from arm_gamepad import JOINTS, MAX_RES

from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

KEYS = {  # key -> (joint, direction)
    "q": ("shoulder_pan", 1), "a": ("shoulder_pan", -1),
    "w": ("shoulder_lift", 1), "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", 1), "d": ("elbow_flex", -1),
    "r": ("wrist_flex", 1), "f": ("wrist_flex", -1),
    "t": ("wrist_roll", 1), "g": ("wrist_roll", -1),
}
STEPS = [1.0, 2.0, 5.0, 10.0]


def read_keys(timeout):
    """All pending single-char keys within `timeout` seconds ('\\x1b' = bare Esc)."""
    keys = []
    if select.select([sys.stdin], [], [], timeout)[0]:
        # os.read, not sys.stdin.read: the text layer would block until 64 chars arrive
        keys = list(os.read(sys.stdin.fileno(), 64).decode(errors="ignore"))
    return keys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="sortbots_arm")
    ap.add_argument("--step", type=float, default=2.0, choices=STEPS, help="degrees per keypress")
    ap.add_argument("--max-step", type=float, default=8.0, help="max degrees goal may lead present pos")
    ap.add_argument("--invert", nargs="*", default=[], choices=JOINTS)
    ap.add_argument("--lock", nargs="*", default=["wrist_roll"], choices=JOINTS,
                    help="joints whose keys are ignored; they hold their pose (default: wrist_roll; "
                         "--lock with no names unlocks everything)")
    a = ap.parse_args()

    if not sys.stdin.isatty():
        raise SystemExit("arm_keys: needs an interactive terminal")

    robot = SOFollower(SOFollowerRobotConfig(
        port=a.port, id=a.id, use_degrees=True, max_relative_target=a.max_step))
    robot.connect(calibrate=False)
    if not robot.is_calibrated:
        robot.disconnect()
        raise SystemExit(f"arm not calibrated for id '{a.id}' - run lerobot-calibrate first")

    # clamp limits in degrees, same mapping lerobot's DEGREES norm uses
    lim = {}
    for j in JOINTS:
        c = robot.bus.calibration[j]
        mid = (c.range_min + c.range_max) / 2
        lim[j] = ((c.range_min - mid) * 360 / MAX_RES + 2, (c.range_max - mid) * 360 / MAX_RES - 2)

    def present():
        obs = robot.get_observation()
        return {j: obs[f"{j}.pos"] for j in JOINTS}

    target = present()
    sign = {j: -1.0 if j in a.invert else 1.0 for j in JOINTS}
    step = a.step
    run = True

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print("ready: q/a w/s e/d r/f t/g move joints, [ ] step, space resync, x quit\r")
    try:
        tty.setcbreak(fd)  # cbreak keeps Ctrl-C working, unlike full raw mode
        while run:
            moved = False
            for k in read_keys(0.05):
                if k in KEYS:
                    j, d = KEYS[k]
                    if j in a.lock:
                        continue
                    lo, hi = lim[j]
                    target[j] = min(hi, max(lo, target[j] + sign[j] * d * step))
                    moved = True
                elif k == "[":
                    step = STEPS[max(0, STEPS.index(step) - 1)]
                elif k == "]":
                    step = STEPS[min(len(STEPS) - 1, STEPS.index(step) + 1)]
                elif k == " ":
                    target = present()
                elif k in ("x", "\x1b"):
                    run = False
            if moved:
                robot.send_action({f"{j}.pos": v for j, v in target.items()})
            print("step %4.1f  " % step + "  ".join(f"{j[:5]}{v:7.1f}" for j, v in target.items()),
                  end="\r", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        robot.disconnect()  # disable_torque_on_disconnect=True by default
        print("\ntorque off, disconnected")


if __name__ == "__main__":
    main()
