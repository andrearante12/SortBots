#!/usr/bin/env python3
"""XInput gamepad teleop + electromagnet for the 5-DOF SO-101 (Flydigi Vader 5 Pro, read over
raw USB by scripts/xinput_pad.py; the Jetson kernel has no xpad).

Runs in the `lerobot` env (lerobot + pyusb + pyserial), NOT in a ROS shell. Needs a finished
`lerobot-calibrate` for --id and, for the magnet, the ESP32 (docs/magnet.md).

Default is CARTESIAN: sticks move the HAND and IK picks the joints (same model and IK as
scripts/arm_cartesian.py, whose live-tuned behavior this copies; kinematics in so101_kin.py):

    hold RT (right trigger)  deadman: nothing moves without it; release and the arm holds
    left  stick Y            hand away from / toward the base
    left  stick X            hand swings left / right around the base
                             (together: the hand moves on a horizontal plane)
    right stick Y            hand up / down
    Y / A                    wrist_flex JOINT up / down, directly (the hand moves with the joint;
                             the stick targets resync to it)
    LB / LT                  tool tilts up / down in hand space (IK keeps the hand put)
    d-pad up / down          speed level up / down (levels 1-4, x0.25..x1.0, starts at 4: the
                             most the 3 A supply handled; --max-level 6 unlocks x1.5, x2.0)
    hold L3 (left stick click)  speed x0.25 on top of the level
    right stick X            wrist_roll (LOCKED by default: --lock)
    B                        magnet toggle: ON needs the deadman held; OFF always works
    hold RB                  HEAD: right stick X / Y = pan / tilt, one axis at a time (whichever
                             the stick leans to more); no RT needed; the right stick stops
                             driving the arm while RB is held
    X                        head back to center (0 / 0)
    START                    quit: torque off, magnet off

`--joint` switches to joint space: left stick X/Y = shoulder_pan/lift, right stick Y = elbow_flex
(hold LB: wrist_flex), d-pad up/down and Y/A = wrist_flex, L3 slow.

One status line is rewritten in place (no scrolling): mode, deadman, hand x/y/z (cm) + pitch,
speed level, magnet state and time on, servo bus voltage and hottest servo, and short notices.

Releasing the deadman does NOT drop the magnet (a held package stays held); quitting, an
unplugged pad or Ctrl-C does. The firmware caps one `on` at 30 s, so this re-arms it every 10 s while on.

The head (scripts/head.py, ids 7/8 on the arm's bus) is on whenever its calibration file
exists; --no-head runs the arm alone. --head-invert pan|tilt flips a stick direction.

    --no-arm      pad + magnet (+ a virtual arm in cartesian mode: prints the hand pose)
    --no-magnet   arm only          --dry-run   cartesian: print IK targets, send nothing
    --flip J...   a joint's motor sign differs from the URDF (cartesian; check with
                  arm_cartesian.py --fk)       --invert J...  same idea in --joint mode
"""
import argparse
import logging
import math
import shutil
import time

import numpy as np

from arm_gamepad import JOINTS, MAX_RES
from head import head_axes, head_step
from so101_kin import SO101Kin
from status_line import servo_fault, status_line
from xinput_pad import XPad, deadzone

# d-pad speed levels, multiples of --speed / --pitch-dps / --roll-dps / --joint-dps. Levels 1-4 are
# arm_cartesian.py's: the top of those (x1.0) is the most the 3 A / 12 V supply handled without the
# servo bus dropping out (live, 2026-10-25), and 400 deg/s joint goals were too fast to steer.
# Levels 5-6 exist for a bigger supply and stay locked behind --max-level (default 4).
LEVELS = [0.25, 0.5, 0.75, 1.0, 1.5, 2.0]
SUPPLY_3A_MAX_LEVEL = 4

MAGNET_REARM_S = 10.0
MAGNET_ON_S = 30


def cartesian_step(kin, q, target, inp, a, lo, hi, sign, dt, level):
    """One control tick of the hand-space mode (the loop body of arm_cartesian.py).
    inp: dict lx ly rx ry tilt(-1/0/1) wf(-1/0/1). Returns (q, target, ok)."""
    k = (0.25 if inp["slow"] else 1.0) * LEVELS[level]
    if inp["wf"]:
        # direct wrist_flex: which way "up" is in joint space depends on the motor sign
        probe = q.copy()
        probe[3] += 1e-3
        up = 1.0 if kin.pose(probe)[3] > kin.pose(q)[3] else -1.0
        q = q.copy()
        q[3] = np.clip(q[3] + up * inp["wf"] * math.radians(a.pitch_dps) * k * dt, lo[3], hi[3])
        target = kin.pose(q)  # the hand moves with the joint; sticks continue from here
    x, y, z, pitch = target
    r, th = math.hypot(x, y), math.atan2(y, x)
    r = max(0.03, r + inp["ly"] * a.speed * k * dt)                  # stick up = away from base
    th += -inp["lx"] * a.speed * k * dt / max(r, 0.08)               # stick right = clockwise from above
    z += inp["ry"] * a.speed * k * dt                                # stick up = hand up
    pitch += inp["tilt"] * math.radians(a.pitch_dps) * k * dt
    cand = np.array([r * math.cos(th), r * math.sin(th), z, pitch])
    q_try = q.copy()
    if "wrist_roll" not in a.lock:  # roll is joint-space: integrate it on the arm's current roll
        q_try[4] = np.clip(q[4] + sign[4] * inp["rx"] * math.radians(a.roll_dps) * k * dt, lo[4], hi[4])
    q_new, ok = kin.ik(cand, q_try, lo, hi)
    if not ok:
        # can't hold the tool tilt here (usually wrist_flex on its stop): put the HAND first and
        # let the tilt drift, instead of freezing the whole arm
        q_new, ok = kin.ik(cand, q_try, lo, hi, pitch_w=0.001)
        if ok:
            cand[3] = kin.pose(q_new)[3]
    if ok:
        # IK near the folded pose asks for big joint swings per tick; cap joint speed so the goal
        # never outruns the servo, and let the hand target follow the joints we actually allow
        cap = math.radians(a.joint_dps) * LEVELS[level] * dt
        if np.any(np.abs(q_new - q) > cap + 1e-9):
            q_new = q + np.clip(q_new - q, -cap, cap)
            cand = kin.pose(q_new)
        return q_new, cand, True
    return q_try, target, False  # hand move unreachable; roll alone is still fine


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="sortbots_arm")
    ap.add_argument("--joint", action="store_true", help="joint-space instead of hand-space")
    ap.add_argument("--max-level", type=int, default=SUPPLY_3A_MAX_LEVEL, choices=range(1, len(LEVELS) + 1),
                    help=f"highest d-pad speed level (1-{len(LEVELS)}). 4 (x1.0) is the most the current "
                         "3 A supply handles; raise it only with a 5 A or larger supply")
    ap.add_argument("--level", type=int, help="starting level (default: --max-level, the fastest allowed)")
    ap.add_argument("--speed", type=float, default=0.10, help="hand speed, m/s at full stick")
    ap.add_argument("--pitch-dps", type=float, default=45.0)
    ap.add_argument("--roll-dps", type=float, default=90.0)
    ap.add_argument("--joint-dps", type=float, default=150.0, help="cap on any joint's goal speed")
    ap.add_argument("--max-dps", type=float, default=30.0, help="--joint: degrees/s at full stick")
    ap.add_argument("--max-step", type=float, help="max degrees a goal may lead present pos "
                                                   "(default 20 cartesian, 8 joint)")
    ap.add_argument("--flip", nargs="*", default=[], choices=JOINTS)
    ap.add_argument("--invert", nargs="*", default=[], choices=JOINTS)
    ap.add_argument("--lock", nargs="*", default=["wrist_roll"], choices=JOINTS,
                    help="joints that ignore the sticks (default wrist_roll; bare --lock unlocks all)")
    ap.add_argument("--no-arm", action="store_true")
    ap.add_argument("--no-magnet", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-head", action="store_true", help="arm only, even if the head is calibrated")
    ap.add_argument("--head-id", default="sortbots_head")
    ap.add_argument("--head-dps", type=float, default=90.0, help="head degrees/s at full stick")
    ap.add_argument("--head-invert", nargs="*", default=[], choices=["pan", "tilt"])
    a = ap.parse_args()
    if a.max_step is None:
        a.max_step = 8.0 if a.joint else 20.0
    # the follower's max_relative_target logs a WARNING per trimmed goal; the joint-speed cap
    # keeps goals inside it, so anything still reaching it is not worth a screenful of dicts
    logging.getLogger("lerobot.robots.utils").setLevel(logging.ERROR)

    pad = XPad()
    magnet = None
    if not a.no_magnet:
        from magnet import Magnet
        magnet = Magnet()
        print("magnet:", magnet.off())  # known state: released

    kin = SO101Kin()
    flip = a.invert if a.joint else a.flip
    sign = np.array([-1.0 if j in flip else 1.0 for j in JOINTS])
    to_q = lambda deg: sign * np.radians([deg[j] for j in JOINTS])          # robot deg -> urdf rad
    to_deg = lambda q: dict(zip(JOINTS, np.degrees(q * sign)))               # urdf rad -> robot deg

    robot, head_lim = None, None
    if not a.no_arm:
        from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
        robot = SOFollower(SOFollowerRobotConfig(
            port=a.port, id=a.id, use_degrees=True, max_relative_target=a.max_step))
        if not a.no_head:
            import head as headmod
            head_cal = headmod.load_calibration(a.head_id)
            if head_cal is None:
                print(f"no head calibration ({headmod.calib_path(a.head_id)}): arm only")
            else:
                # One port, one process: the head rides on the follower's own bus. send_action
                # only writes the arm's keys, and configure()/disconnect() cover all seven, so
                # the head gets the arm's gains and goes limp with it on quit.
                from lerobot.motors import SerialMotorsBus
                robot.bus = SerialMotorsBus(port=a.port, motors={**robot.bus.motors, **headmod.head_motors()},
                                            calibration={**robot.calibration, **head_cal})
                head_lim = headmod.limits(head_cal)
        try:
            robot.connect(calibrate=False)
        except RuntimeError as err:  # the handshake wants every motor, head included
            raise SystemExit(f"{err}\n(head unplugged? run with --no-head)")
        if not robot.is_calibrated:
            robot.disconnect()
            raise SystemExit(f"arm or head not calibrated (ids '{a.id}', '{a.head_id}') - "
                             "lerobot-calibrate / arm.sh head calibrate")
        lo_d, hi_d = {}, {}
        for j in JOINTS:  # calibrated range in degrees, same mapping lerobot's DEGREES norm uses
            c = robot.bus.calibration[j]
            mid = (c.range_min + c.range_max) / 2
            lo_d[j] = (c.range_min - mid) * 360 / MAX_RES + 2
            hi_d[j] = (c.range_max - mid) * 360 / MAX_RES - 2
        a_lo = np.radians([lo_d[j] for j in JOINTS]) * sign
        a_hi = np.radians([hi_d[j] for j in JOINTS]) * sign
        lo, hi = np.minimum(a_lo, a_hi), np.maximum(a_lo, a_hi)
    else:  # virtual arm: URDF limits
        lo = np.array([kin.urdf_limits[j][0] for j in JOINTS])
        hi = np.array([kin.urdf_limits[j][1] for j in JOINTS])

    def read_deg():
        o = robot.get_observation()
        return {j: o[f"{j}.pos"] for j in JOINTS}

    # The folded rest pose sits within a fraction of a degree of the end stops, and IK rejects
    # any pose outside [lo, hi]: clip what we read into range or every move is refused.
    q = np.clip(to_q(read_deg()), lo, hi) if robot else np.clip(np.zeros(5), lo, hi)
    target = kin.pose(q)  # start exactly where the arm is: connecting never jerks
    head_sign = {m: -1.0 if m[5:] in a.head_invert else 1.0 for m in ("head_pan", "head_tilt")}
    if head_lim:  # same no-jerk rule for the head: start from where it is
        o = robot.get_observation()
        head = {m: min(head_lim[m][1], max(head_lim[m][0], o[f"{m}.pos"])) for m in head_lim}
    x_prev = False
    jtarget = to_deg(q)   # --joint mode targets, degrees
    level, dpad_prev = min(a.level or a.max_level, a.max_level) - 1, (False, False)
    mag_on, mag_armed_at, b_prev = False, 0.0, False
    mode = "joint" if a.joint else "cartesian"
    print(f"ready ({mode}): hold RT to move, B magnet, START quit",
          "| RB+right stick head, X center" if head_lim else "",
          "(NO ARM)" if a.no_arm else "", "(DRY RUN)" if a.dry_run else "")
    ui = {"note": "", "until": 0.0, "bus": "", "bus_at": -9.0, "draw_at": 0.0, "mag_t0": 0.0}
    ok = True

    def note(msg, secs=2.5):
        ui["note"], ui["until"] = msg, time.monotonic() + secs

    def read_bus():
        # STS3215 registers: Present_Voltage in 0.1 V, Present_Temperature in deg C. A failed
        # read must not interrupt teleop, so it shows "bus ?" and moves on.
        try:
            v = robot.bus.sync_read("Present_Voltage", normalize=False)
            t = robot.bus.sync_read("Present_Temperature", normalize=False)
            vmin = min(v.values()) / 10.0
            txt = f"bus {vmin:4.1f}V {max(t.values()):.0f}C" + (" LOW" if vmin < 10.5 else "")
            # A servo in overload protection drops to Protective_Torque (20%) and looks "stuck"
            # under the magnet's weight; its Status bits say so. Held 10 s on screen so a fault
            # that clears between redraws is still readable.
            for m, bits in robot.bus.sync_read("Status", normalize=False).items():
                if bits:
                    note(f"FAULT {m} {servo_fault(bits)}", 10.0)
            return txt
        except Exception:
            return "bus ?"

    last = time.monotonic()
    try:
        while True:
            st = pad.poll(20)
            btn = st["buttons"]
            now = time.monotonic()
            dt, last = now - last, now
            # NOT a staleness check: the pad sends a report only when an input CHANGES, so a held
            # steady stick or an idle pad is silent (a 0.5 s "no report = lost" rule dropped the
            # magnet half a second after every toggle, 2026-11-06). Unplugging raises a USB error
            # in pad.poll(), which unwinds through the finally below and releases the magnet.
            held = st["rt"] > 0.5
            if btn["START"]:
                break

            # magnet toggle on the B press edge; ON only with the deadman, OFF anytime
            b = btn["B"]
            if b and not b_prev and magnet:
                if mag_on:
                    mag_on = False
                    note("magnet " + magnet.off())
                elif held:
                    mag_on, mag_armed_at, ui["mag_t0"] = True, now, now
                    note("magnet " + magnet.on(MAGNET_ON_S))
                else:
                    note("hold RT to turn the magnet on")
            b_prev = b
            if magnet and mag_on and now - mag_armed_at > MAGNET_REARM_S:
                magnet.on(MAGNET_ON_S)
                mag_armed_at = now

            # Head: RB is its own deadman. Stick left = +pan (as `head.py keys` a), up = +tilt.
            head_held = bool(head_lim) and btn["RB"] and not a.dry_run
            if head_held:
                pan_in, tilt_in = head_axes(-deadzone(st["rx"]), deadzone(st["ry"]))
                new = head_step(head, pan_in * head_sign["head_pan"], tilt_in * head_sign["head_tilt"],
                                a.head_dps, dt, head_lim)
                if new != head:
                    head = new
                    robot.bus.sync_write("Goal_Position", head)
            if head_lim and not a.dry_run and btn["X"] and not x_prev:
                head = {m: min(head_lim[m][1], max(head_lim[m][0], 0.0)) for m in head}
                robot.bus.sync_write("Goal_Position", head)
                note("head centered")
            x_prev = btn["X"]
            if head_held:  # the right stick belongs to the head while RB is held
                st = {**st, "rx": 0.0, "ry": 0.0}

            if not a.joint:
                du, dd = btn["DUP"], btn["DDOWN"]
                if (du and not dpad_prev[0]) or (dd and not dpad_prev[1]):
                    level = min(a.max_level - 1, max(0, level + (1 if du else -1)))
                    note(f"speed level {level + 1}/{a.max_level} (x{LEVELS[level]})")
                dpad_prev = (du, dd)

            if held and a.joint:
                k = a.max_dps * dt * (0.25 if btn["L3"] else 1.0)
                ry = deadzone(st["ry"])
                dpad = float(btn["DUP"]) - float(btn["DDOWN"])
                vel = {"shoulder_pan": deadzone(st["lx"]), "shoulder_lift": deadzone(st["ly"]),
                       "elbow_flex": 0.0 if btn["LB"] else ry, "wrist_roll": deadzone(st["rx"]),
                       "wrist_flex": (ry if btn["LB"] else dpad) + float(btn["Y"]) - float(btn["A"])}
                for i, j in enumerate(JOINTS):
                    if j not in a.lock:
                        lo_j, hi_j = np.degrees(lo[i] * sign[i]), np.degrees(hi[i] * sign[i])
                        lo_j, hi_j = min(lo_j, hi_j), max(lo_j, hi_j)
                        jtarget[j] = min(hi_j, max(lo_j, jtarget[j] + vel[j] * k))
                if robot:
                    robot.send_action({f"{j}.pos": v for j, v in jtarget.items()})
            elif held:
                inp = {"lx": deadzone(st["lx"]), "ly": deadzone(st["ly"]),
                       "rx": deadzone(st["rx"]), "ry": deadzone(st["ry"]),
                       "tilt": float(btn["LB"]) - float(st["lt"] > 0.5),
                       "wf": float(btn["Y"]) - float(btn["A"]), "slow": btn["L3"]}
                q, target, ok = cartesian_step(kin, q, target, inp, a, lo, hi, sign, dt, level)
                if robot and not a.dry_run:
                    deg = to_deg(q)
                    robot.send_action({f"{j}.pos": deg[j] for j in JOINTS})
            elif robot:
                # resync to where the arm really is so re-grabbing never snaps back
                q = np.clip(to_q(read_deg()), lo, hi)
                target, jtarget = kin.pose(q), to_deg(q)
            if now - ui["draw_at"] > 0.1:  # redraw at 10 Hz: enough to read, no flicker
                ui["draw_at"] = now
                if robot and now - ui["bus_at"] > 1.0:
                    ui["bus"], ui["bus_at"] = read_bus(), now
                if a.joint:
                    pose_txt = " ".join(f"{j[:5]}{v:6.1f}" for j, v in jtarget.items())
                else:
                    pose_txt = (f"x{target[0]*100:6.1f} y{target[1]*100:6.1f} z{target[2]*100:6.1f}cm "
                                f"p{math.degrees(target[3]):4.0f}")
                if head_lim:
                    pose_txt += f" head{head['head_pan']:+4.0f}/{head['head_tilt']:+4.0f}"
                label = ("JOINT" if a.joint else "CART") + (" DRY" if a.dry_run else "") + (" SIM" if not robot else "")
                line = status_line(shutil.get_terminal_size().columns - 1, label, held, pose_txt,
                                   f"L{level + 1}/{a.max_level}" + (" slow" if btn["L3"] else ""),
                                   mag_on, now - ui["mag_t0"], ok, ui["bus"],
                                   ui["note"] if now < ui["until"] else "")
                print("\r" + line + "\x1b[K", end="", flush=True)
            time.sleep(max(0.0, 0.02 - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        if magnet:
            try:
                print("\nmagnet:", magnet.off())
            finally:
                magnet.close()
        if robot:
            robot.disconnect()  # torque off
        pad.close()
        print("\ndone: magnet off, torque off")


if __name__ == "__main__":
    main()
