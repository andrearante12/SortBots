#!/usr/bin/env python3
"""One joint at a time: hold a button to pick the joint, push the stick to move just that one.

For bring-up and debugging (is each motor wired to the right joint, which way does it turn, where
are its stops) — scripts/arm_cartesian.py is the one for driving the arm. Same env/rules as
scripts/arm_gamepad.py: `lerobot` conda env, no ROS shell, needs the 5-DOF lerobot patch and
a finished calibration.

    hold Y   shoulder_pan        hold X   shoulder_lift     hold A   elbow_flex
    hold B   wrist_flex          hold R   wrist_roll
    left OR right stick, up/down    move the held joint (further = faster)
    d-pad up / down                 speed level up / down (1-4, starts at 2)
    + (start)                       quit, torque off

Holding the button IS the deadman: release it and the joint holds. If two buttons are held,
only the one pressed last is live. Positions are calibrated degrees (0 = middle of the range),
clamped to the calibrated range. Which way "up" turns a joint is whatever the motor does; the
status line shows the angle so you can see it.
"""
import argparse
import logging
import math
import time

from evdev import ecodes as e

from arm_gamepad import JOINTS, MAX_RES, deadzone, find_pad
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

# physical button -> joint. hid-nintendo reports B as BTN_SOUTH and A as BTN_EAST (checked live
# 2026-10-25), so these are the labels printed on the pad, not the evdev names' usual layout.
BUTTON_JOINT = {
    e.BTN_WEST: "shoulder_pan",    # Y
    e.BTN_NORTH: "shoulder_lift",  # X
    e.BTN_EAST: "elbow_flex",      # A
    e.BTN_SOUTH: "wrist_flex",     # B
    e.BTN_TR: "wrist_roll",        # R
}
LEVELS = [0.25, 0.5, 0.75, 1.0]  # x --dps; capped for the 3 A supply, see arm_cartesian.py
START_LEVEL = 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="sortbots_arm")
    ap.add_argument("--dps", type=float, default=60.0, help="degrees/s at full stick, level x1.0")
    ap.add_argument("--max-step", type=float, default=20.0)
    a = ap.parse_args()

    logging.getLogger("lerobot.robots.utils").setLevel(logging.ERROR)
    pad = find_pad()
    pad.grab()
    robot = None
    try:
        robot = SOFollower(SOFollowerRobotConfig(
            port=a.port, id=a.id, use_degrees=True, max_relative_target=a.max_step))
        robot.connect(calibrate=False)
        if not robot.is_calibrated:
            raise SystemExit(f"arm not calibrated for id '{a.id}' - run lerobot-calibrate first")

        lim = {}
        for j in JOINTS:
            c = robot.bus.calibration[j]
            mid = (c.range_min + c.range_max) / 2
            lim[j] = ((c.range_min - mid) * 360 / MAX_RES + 2, (c.range_max - mid) * 360 / MAX_RES - 2)

        obs = robot.get_observation()
        # start inside the limits: the folded rest pose can sit a fraction of a degree outside
        target = {j: min(lim[j][1], max(lim[j][0], obs[f"{j}.pos"])) for j in JOINTS}
        absinfo = {c: pad.absinfo(c) for c in (e.ABS_Y, e.ABS_RY)}
        stick = {e.ABS_Y: 0.0, e.ABS_RY: 0.0}
        held = []  # buttons currently down, in press order; last one is live
        level, run = START_LEVEL, True
        print("hold Y/X/A/B/R to pick pan/lift/elbow/wrist_flex/roll; stick moves it; + quits")

        last = time.monotonic()
        while run:
            for ev in iter(pad.read_one, None):
                if ev.type == e.EV_ABS and ev.code in stick:
                    stick[ev.code] = deadzone(ev.value / absinfo[ev.code].max)
                elif ev.type == e.EV_ABS and ev.code == e.ABS_HAT0Y and ev.value:
                    level = min(len(LEVELS) - 1, max(0, level - ev.value))  # d-pad up is -1
                    print(f"\nspeed level {level + 1}/{len(LEVELS)} (x{LEVELS[level]})")
                elif ev.type == e.EV_KEY:
                    if ev.code in BUTTON_JOINT:
                        if ev.value and ev.code not in held:
                            held.append(ev.code)
                        elif not ev.value and ev.code in held:
                            held.remove(ev.code)
                    elif ev.code == e.BTN_START and ev.value:
                        run = False

            now = time.monotonic()
            dt = now - last
            last = now
            joint = BUTTON_JOINT[held[-1]] if held else None
            if joint:
                # both sticks work so either hand can do it; stick up is negative Y
                push = -(stick[e.ABS_Y] if abs(stick[e.ABS_Y]) >= abs(stick[e.ABS_RY]) else stick[e.ABS_RY])
                lo, hi = lim[joint]
                target[joint] = min(hi, max(lo, target[joint] + push * a.dps * LEVELS[level] * dt))
                print(f"{joint:14} {target[joint]:7.1f} deg  [{lo:6.1f}..{hi:6.1f}]   ", end="\r", flush=True)
            # every tick, so non-selected joints keep holding their target
            robot.send_action({f"{j}.pos": v for j, v in target.items()})
            time.sleep(max(0.0, 0.02 - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        pad.ungrab()
        if robot is not None and robot.is_connected:
            robot.disconnect()  # torque off
        print("\ntorque off, disconnected")


if __name__ == "__main__":
    main()
