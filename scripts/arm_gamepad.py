#!/usr/bin/env python3
"""Joint-space gamepad teleop for the 5-DOF SO-101 follower (Switch Pro Controller).

Runs in the `lerobot` conda env (evdev + lerobot), NOT in a ROS shell. Needs the
`sortbots-5dof` lerobot branch (no gripper) and a finished `lerobot-calibrate`
with --robot.id matching --id here.

    hold ZR  = deadman: sticks move joints; release and the arm holds its pose
    left  stick X / Y   shoulder_pan / shoulder_lift
    right stick Y / X   elbow_flex / wrist_roll
    hold L              right stick Y drives wrist_flex instead of elbow_flex
    d-pad up / down     wrist_flex (backup)
    A                   speed x0.25 while held (fine moves)
    + (start)           quit, torque off

Targets integrate stick * max_dps and are clamped to the calibrated range, so a
held stick can never ask for past the end stops. Start position is the arm's
present pose, so connecting never jerks. --dry-run prints targets, sends nothing:
use it first to check each joint's DIRECTION (flip with --invert NAME).
"""
import argparse
import time

from evdev import InputDevice, ecodes as e, list_devices

from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
MAX_RES = 4095


def find_pad():
    for path in list_devices():
        d = InputDevice(path)
        if "Pro Controller" in d.name and "IMU" not in d.name:
            return d
    raise SystemExit("no Pro Controller found - is it paired and connected? (bluetoothctl)")


def deadzone(v, flat=0.12):
    return 0.0 if abs(v) < flat else (abs(v) - flat) / (1 - flat) * (1 if v > 0 else -1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="sortbots_arm")
    ap.add_argument("--max-dps", type=float, default=30.0, help="degrees/s at full stick")
    ap.add_argument("--max-step", type=float, default=8.0, help="max degrees goal may lead present pos")
    ap.add_argument("--invert", nargs="*", default=[], choices=JOINTS)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    pad = find_pad()
    pad.grab()
    absinfo = {c: pad.absinfo(c) for c in (e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_RY)}

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

    obs = robot.get_observation()
    target = {j: obs[f"{j}.pos"] for j in JOINTS}
    sign = {j: -1.0 if j in a.invert else 1.0 for j in JOINTS}
    held, dpad_y, slow, mod, run = False, 0, False, False, True
    stick = {e.ABS_X: 0.0, e.ABS_Y: 0.0, e.ABS_RX: 0.0, e.ABS_RY: 0.0}
    print("ready: hold ZR to move, + to quit", "(DRY RUN)" if a.dry_run else "")

    dt, last = 0.02, time.monotonic()
    try:
        while run:
            for ev in iter(pad.read_one, None):  # non-blocking drain
                if ev.type == e.EV_ABS:
                    if ev.code in stick:
                        i = absinfo[ev.code]
                        stick[ev.code] = deadzone(ev.value / i.max)
                    elif ev.code == e.ABS_HAT0Y:
                        dpad_y = ev.value
                elif ev.type == e.EV_KEY:
                    if ev.code == e.BTN_TR2:
                        held = ev.value > 0
                    elif ev.code == e.BTN_EAST:
                        slow = ev.value > 0
                    elif ev.code == e.BTN_TL:
                        mod = ev.value > 0
                    elif ev.code == e.BTN_START and ev.value:
                        run = False

            now = time.monotonic()
            h = now - last
            last = now
            if held:
                k = a.max_dps * h * (0.25 if slow else 1.0)
                # screen-up on a stick is negative Y, so negate for "up = positive"
                ry = -stick[e.ABS_RY]
                vel = {"shoulder_pan": stick[e.ABS_X], "shoulder_lift": -stick[e.ABS_Y],
                       "elbow_flex": 0.0 if mod else ry, "wrist_roll": stick[e.ABS_RX],
                       "wrist_flex": ry if mod else -float(dpad_y)}
                for j in JOINTS:
                    lo, hi = lim[j]
                    target[j] = min(hi, max(lo, target[j] + sign[j] * vel[j] * k))
            else:
                # not held: follow where the arm actually is, so re-grab never snaps back
                obs = robot.get_observation()
                target = {j: obs[f"{j}.pos"] for j in JOINTS}

            if a.dry_run:
                if held:
                    print("  ".join(f"{j[:5]}{v:7.1f}" for j, v in target.items()), end="\r", flush=True)
            elif held:
                robot.send_action({f"{j}.pos": v for j, v in target.items()})
            time.sleep(max(0.0, dt - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        pad.ungrab()
        robot.disconnect()  # disable_torque_on_disconnect=True by default
        print("\ntorque off, disconnected")


if __name__ == "__main__":
    main()
