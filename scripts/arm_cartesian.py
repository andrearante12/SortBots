#!/usr/bin/env python3
"""Cartesian gamepad teleop for the 5-DOF SO-101: sticks move the HAND, IK picks joints.

Same env/rules as scripts/arm_gamepad.py (lerobot conda env, no ROS shell, needs
`sortbots-5dof` lerobot branch + a finished calibration). Kinematics: scripts/so101_kin.py.

    hold ZR  = deadman; release and the arm holds where it is
    left  stick Y      hand away from / toward the base (radial)
    left  stick X      hand swings left / right around the base
    right stick Y      hand up / down
    right stick X      wrist roll (joint-space, doesn't move the hand)
    L / ZL             tool tilts up / down (hand pitch)
    d-pad up / down    speed level up / down (4 levels, starts at 2)
    X / B              wrist_flex joint up / down (moves the hand; the stick targets resync)
    A                  speed x0.25 while held, on top of the level
    + (start)          quit, torque off

Modes that don't move the arm:
    --fk       torque OFF; move the arm by hand and watch the hand position print.
               USE THIS FIRST to check the model: raising the elbow must raise z,
               swinging the base must swing y, and so on.
    --dry-run  torque on (holds pose), prints the IK targets, sends nothing.

If the hand moves the wrong way for a stick, a motor's sign is flipped relative to the
URDF: pass --flip JOINT (repeatable) and re-check with --fk. A target IK can't reach
(out of range / past a limit) is simply not taken, so the hand stops at the boundary.
"""
import argparse
import logging
import math
import time

import numpy as np
from evdev import ecodes as e

from arm_gamepad import JOINTS, MAX_RES, deadzone, find_pad
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
from so101_kin import SO101Kin

# d-pad speed levels, as multiples of --speed / --joint-dps / --pitch-dps / --roll-dps. The top
# is deliberately low: 400 deg/s joint goals (live, 2026-10-25) were too fast to steer, and
# the bus dropped out under fast moves on the 3 A / 12 V supply. 1.0 was re-enabled by choice;
# add 1.25+ only once the supply is 5 A or more.
LEVELS = [0.25, 0.5, 0.75, 1.0]
START_LEVEL = 1  # index into LEVELS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="sortbots_arm")
    ap.add_argument("--speed", type=float, default=0.10, help="hand speed, m/s at full stick")
    ap.add_argument("--pitch-dps", type=float, default=45.0)
    ap.add_argument("--roll-dps", type=float, default=90.0)
    ap.add_argument("--joint-dps", type=float, default=150.0, help="cap on any joint's goal speed")
    ap.add_argument("--max-step", type=float, default=20.0, help="max degrees goal may lead present pos")
    ap.add_argument("--flip", nargs="*", default=[], choices=JOINTS, help="negate a joint vs the URDF")
    ap.add_argument("--fk", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    # The follower's max_relative_target (--max-step) is a backstop that logs a WARNING every
    # time it trims a goal. The joint-speed cap below keeps goals inside it, so anything still
    # reaching it is rare and not worth a screenful of dict dumps.
    logging.getLogger("lerobot.robots.utils").setLevel(logging.ERROR)

    kin = SO101Kin()
    sign = np.array([-1.0 if j in a.flip else 1.0 for j in JOINTS])
    to_q = lambda deg: sign * np.radians([deg[j] for j in JOINTS])          # robot deg -> urdf rad
    to_deg = lambda q: dict(zip(JOINTS, np.degrees(q * sign)))               # urdf rad -> robot deg

    robot = SOFollower(SOFollowerRobotConfig(
        port=a.port, id=a.id, use_degrees=True, max_relative_target=a.max_step))
    robot.connect(calibrate=False)
    if not robot.is_calibrated:
        robot.disconnect()
        raise SystemExit(f"arm not calibrated for id '{a.id}' - run lerobot-calibrate first")

    # joint limits in URDF radians, from the calibrated range (same mapping as arm_gamepad.py)
    lo_d, hi_d = {}, {}
    for j in JOINTS:
        c = robot.bus.calibration[j]
        mid = (c.range_min + c.range_max) / 2
        lo_d[j] = (c.range_min - mid) * 360 / MAX_RES + 2
        hi_d[j] = (c.range_max - mid) * 360 / MAX_RES - 2
    a_lo = np.radians([lo_d[j] for j in JOINTS]) * sign
    a_hi = np.radians([hi_d[j] for j in JOINTS]) * sign
    lo, hi = np.minimum(a_lo, a_hi), np.maximum(a_lo, a_hi)

    def read_deg():
        o = robot.get_observation()
        return {j: o[f"{j}.pos"] for j in JOINTS}

    # The folded rest pose sits within a fraction of a degree of the end stops (shoulder_lift
    # reads -105.3 against a -105.0 limit). IK rejects any pose outside [lo, hi], so a start
    # pose that is a hair outside would refuse every move. Clip what we read into range.
    read_q = lambda: np.clip(to_q(read_deg()), lo, hi)

    pad = None
    try:
        if a.fk:
            robot.bus.disable_torque()
            print("torque OFF - move the arm by hand. x,y,z in cm, pitch in deg. Ctrl+C quits.")
            while True:
                q = to_q(read_deg())
                p = kin.pose(q)
                print(f"x{p[0]*100:7.1f} y{p[1]*100:7.1f} z{p[2]*100:7.1f} pitch{math.degrees(p[3]):7.1f}   "
                      + " ".join(f"{j[:5]}{d:7.1f}" for j, d in to_deg(q).items()), end="\r", flush=True)
                time.sleep(0.05)

        pad = find_pad()
        pad.grab()
        absinfo = {c: pad.absinfo(c) for c in (e.ABS_X, e.ABS_Y, e.ABS_RX, e.ABS_RY)}
        stick = {c: 0.0 for c in absinfo}
        held = slow = tilt_up = tilt_dn = wf_up = wf_dn = False
        level = START_LEVEL
        run = True

        q = read_q()
        target = kin.pose(q)  # start exactly where the arm is: connecting never jerks
        print("ready: hold ZR to move, + to quit", "(DRY RUN)" if a.dry_run else "")
        last = time.monotonic()
        while run:
            for ev in iter(pad.read_one, None):
                if ev.type == e.EV_ABS and ev.code in stick:
                    stick[ev.code] = deadzone(ev.value / absinfo[ev.code].max)
                elif ev.type == e.EV_ABS and ev.code == e.ABS_HAT0Y and ev.value:
                    level = min(len(LEVELS) - 1, max(0, level - ev.value))  # d-pad up is -1
                    print(f"\nspeed level {level + 1}/{len(LEVELS)} (x{LEVELS[level]})")
                elif ev.type == e.EV_KEY:
                    if ev.code == e.BTN_TR2:
                        held = ev.value > 0
                    elif ev.code == e.BTN_EAST:
                        slow = ev.value > 0
                    elif ev.code == e.BTN_NORTH:  # physical X (hid-nintendo: B is BTN_SOUTH, A is BTN_EAST)
                        wf_up = ev.value > 0
                    elif ev.code == e.BTN_SOUTH:  # physical B
                        wf_dn = ev.value > 0
                    elif ev.code == e.BTN_TL:
                        tilt_up = ev.value > 0
                    elif ev.code == e.BTN_TL2:
                        tilt_dn = ev.value > 0
                    elif ev.code == e.BTN_START and ev.value:
                        run = False

            now = time.monotonic()
            dt = now - last
            last = now
            if held:
                k = (0.25 if slow else 1.0) * LEVELS[level]
                if wf_up or wf_dn:
                    # direct wrist_flex: X raises the tool tilt, B lowers it. Which way that is
                    # in joint space depends on the motor sign, so take it from the model.
                    probe = q.copy()
                    probe[3] += 1e-3
                    up = 1.0 if kin.pose(probe)[3] > kin.pose(q)[3] else -1.0
                    q = q.copy()
                    q[3] = np.clip(q[3] + up * (wf_up - wf_dn) * math.radians(a.pitch_dps) * k * dt, lo[3], hi[3])
                    target = kin.pose(q)  # the hand moves with the joint; sticks continue from here
                x, y, z, pitch = target
                r, th = math.hypot(x, y), math.atan2(y, x)
                r = max(0.03, r + -stick[e.ABS_Y] * a.speed * k * dt)
                th += -stick[e.ABS_X] * a.speed * k * dt / max(r, 0.08)  # stick right = clockwise from above
                z += -stick[e.ABS_RY] * a.speed * k * dt
                pitch += (tilt_up - tilt_dn) * math.radians(a.pitch_dps) * k * dt
                cand = np.array([r * math.cos(th), r * math.sin(th), z, pitch])
                # roll is joint-space: integrate it on the arm's current roll
                q_try = q.copy()
                q_try[4] = np.clip(q[4] + sign[4] * stick[e.ABS_RX] * math.radians(a.roll_dps) * k * dt, lo[4], hi[4])
                q_new, ok = kin.ik(cand, q_try, lo, hi)
                if not ok:
                    # can't hold the tool tilt here (usually wrist_flex on its stop): put the
                    # HAND first and let the tilt drift, instead of freezing the whole arm
                    q_new, ok = kin.ik(cand, q_try, lo, hi, pitch_w=0.001)
                    if ok:
                        cand[3] = kin.pose(q_new)[3]
                if ok:
                    # IK near the folded pose asks for big joint swings per tick (wrist_flex +25
                    # deg in one step). Cap joint speed so the goal never outruns the servo; if
                    # that trims the step, the hand target follows the joints we actually allow.
                    cap = math.radians(a.joint_dps) * LEVELS[level] * dt
                    step = np.clip(q_new - q, -cap, cap)
                    if np.any(np.abs(q_new - q) > cap + 1e-9):
                        q_new = q + step
                        cand = kin.pose(q_new)
                    target, q = cand, q_new
                else:
                    q = q_try  # hand move unreachable; roll alone is still fine
                if a.dry_run:
                    print(f"x{target[0]*100:6.1f} y{target[1]*100:6.1f} z{target[2]*100:6.1f} "
                          f"p{math.degrees(target[3]):6.1f} ok={int(ok)}", end="\r", flush=True)
                else:
                    deg = to_deg(q)
                    robot.send_action({f"{j}.pos": deg[j] for j in JOINTS})
            else:
                # resync to where the arm really is so re-grabbing never snaps back
                q = read_q()
                target = kin.pose(q)
            time.sleep(max(0.0, 0.02 - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        if pad is not None:
            pad.ungrab()
        robot.disconnect()  # torque off
        print("\ntorque off, disconnected")


if __name__ == "__main__":
    main()
