#!/usr/bin/env python3
"""Pan/tilt camera head: two STS3215 servos daisy-chained on the ARM's bus board.

Runs in the `lerobot` conda env, NOT in a ROS shell (use `scripts/arm.sh head ...`, which
checks the port and the env). The head shares the arm's serial bus, so it gets ids the arm
never uses: 7 = head_pan, 8 = head_tilt (XLeRobot's head ids; 6 stays free for a gripper).
Its calibration lives in its own file, so `lerobot-calibrate` on the arm never touches it.

    head.py scan              who answers on the bus (ids, model, volts, temp, raw position)
    head.py setup pan|tilt    give ONE new servo its id; it must be ALONE on the board
    head.py calibrate         center the head, record its range by hand, save the calibration
    head.py read              torque off, live pan/tilt in degrees (check signs and center)
    head.py keys              torque on, a/d pan, w/s tilt, c center, [ ] step, x quit

Setup has to be one servo alone because every new STS3215 ships as id 1 at 1 Mbaud, the
same id as the arm's shoulder_pan: with the arm still chained, the id write would go to both.
The arm process and this one cannot hold /dev/ttyACM0 at the same time.
"""
import argparse
import json
import os
import select
import sys
import termios
import time
import tty

# lerobot is imported where it is used, so the pure helpers (head_limits, head_step) load under
# the system python3 that runs the offline tests.

HEAD = {"head_pan": 7, "head_tilt": 8}
ARM = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5}
MAX_RES = 4095
STEPS = [1.0, 2.0, 5.0, 10.0]
KEYS = {"a": ("head_pan", 1), "d": ("head_pan", -1), "w": ("head_tilt", 1), "s": ("head_tilt", -1)}


def calib_path(robot_id):
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION
    return HF_LEROBOT_CALIBRATION / "robots" / "sortbots_head" / f"{robot_id}.json"


def load_calibration(robot_id):
    from lerobot.motors import MotorCalibration
    p = calib_path(robot_id)
    if not p.is_file():
        return None
    return {m: MotorCalibration(**c) for m, c in json.loads(p.read_text()).items()}


def head_motors():
    from lerobot.motors import Motor, MotorNormMode
    return {m: Motor(i, "sts3215", MotorNormMode.DEGREES) for m, i in HEAD.items()}


def make_bus(port, calibration=None):
    from lerobot.motors import SerialMotorsBus
    return SerialMotorsBus(port=port, calibration=calibration, motors=head_motors())


def cmd_scan(a):
    from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus
    names = {**{i: m for m, i in ARM.items()}, **{i: m for m, i in HEAD.items()}}
    # A bus that knows every expected id, so registers can be read by name on whatever answers.
    bus = SerialMotorsBus(port=a.port, motors={m: Motor(i, "sts3215", MotorNormMode.DEGREES)
                                               for i, m in names.items()})
    bus.connect(handshake=False)
    try:
        found = bus._scan(bus._definition("head_pan"))
        if not found:
            print("nothing answers at 1 Mbaud: 12 V supply on? servo cable seated? (a brand-new "
                  "servo at another baud rate is found by `setup`)")
        for i in sorted(found):
            name = names.get(i)
            if name is None:
                print(f"id {i:3d}  model {found[i]}  ??? not an arm or head id")
                continue
            r = lambda reg: bus.read(reg, name, normalize=False, num_retry=2)
            print(f"id {i:3d}  {name:<14} model {found[i]}  {r('Present_Voltage') / 10:5.1f} V  "
                  f"{r('Present_Temperature'):3d} C  pos {r('Present_Position'):5d}")
        missing = [f"{m}({i})" for i, m in sorted(names.items()) if i not in found]
        if missing:
            print("missing:", " ".join(missing))
        if 1 in found and len(found) == 1:
            print("only id 1 answers: either the arm is unplugged except shoulder_pan, or this is a "
                  "NEW servo still at its factory id -> `head.py setup pan|tilt`")
    finally:
        bus.disconnect(disable_torque=False)


def cmd_setup(a):
    motor = f"head_{a.which}"
    bus = make_bus(a.port)
    print(f"Plug ONLY the {motor} servo into the board (unplug the arm's cable and the other head\n"
          f"servo), 12 V on. It will get id {HEAD[motor]} at 1 Mbaud.")
    input("press ENTER when it is the only servo on the bus...")
    # lerobot's setup_motor takes the FIRST servo a scan finds and renumbers it; it does not
    # check it is alone. A forgotten arm cable would renumber shoulder_pan, so count first,
    # across every baud rate a servo might be at.
    bus.connect(handshake=False)
    definition = bus._definition(motor)
    seen = []
    try:
        for rate in definition.baudrates:
            bus.set_baudrate(rate)
            seen += [(rate, i) for i in bus._scan(definition)]
    finally:
        bus.set_baudrate(bus.default_baudrate)
    if len(seen) != 1:
        bus.disconnect(disable_torque=False)
        raise SystemExit(f"need exactly one servo on the bus, found {len(seen)}: "
                         + (", ".join(f"id {i} @ {r}" for r, i in seen) or "none (12 V on?)"))
    rate, found_id = seen[0]
    print(f"found id {found_id} @ {rate} baud")
    bus.setup_motor(motor, initial_baudrate=rate, initial_id=found_id)
    # setup_motor already leaves the servo's torque off; disable_torque here would also
    # address the other head servo, which is not plugged in, and time out.
    bus.disconnect(disable_torque=False)
    print(f"{motor}: id {HEAD[motor]} set. Label it, then do the other one or chain them back.")


def cmd_calibrate(a):
    bus = make_bus(a.port)
    bus.connect()
    try:
        print("Center the head by hand: pan looking straight ahead (along the arm's forward\n"
              "direction), tilt with the camera level. That pose becomes 0 deg on both.")
        cal = bus.record_calibration("position", full_turn_motors=[], inverted_motors=a.invert)
        bus.write_calibration(cal)
    finally:
        bus.disconnect(disable_torque=True)
    p = calib_path(a.id)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps({m: vars(c) for m, c in cal.items()}, indent=4))
    print("saved", p)
    for m, c in cal.items():
        print(f"  {m:<10} range {c.range_min}..{c.range_max}  "
              f"({(c.range_min - 2047) * 360 / MAX_RES:+.0f} .. {(c.range_max - 2047) * 360 / MAX_RES:+.0f} deg)")


def connect_calibrated(a):
    cal = load_calibration(a.id)
    if cal is None:
        raise SystemExit(f"no head calibration at {calib_path(a.id)} - run `head.py calibrate` first")
    bus = make_bus(a.port, cal)
    bus.connect()
    if not bus.is_calibrated:
        bus.disconnect(disable_torque=False)
        raise SystemExit("servo registers don't match the head calibration file (servo swapped or "
                         "re-homed?) - run `head.py calibrate` again")
    return bus, cal


def limits(cal):
    """Calibrated range in degrees around the mid, 2 deg inside the stops (same as arm_keys).
    `cal` values only need range_min / range_max."""
    out = {}
    for m, c in cal.items():
        mid = (c.range_min + c.range_max) / 2
        out[m] = ((c.range_min - mid) * 360 / MAX_RES + 2, (c.range_max - mid) * 360 / MAX_RES - 2)
    return out


def head_step(target, pan_in, tilt_in, dps, dt, lim):
    """One teleop tick: integrate stick inputs (-1..1, already deadzoned) at `dps` and clamp to
    `lim` (limits()). Returns a new target dict; pure, so the offline test covers it."""
    out = dict(target)
    for m, v in (("head_pan", pan_in), ("head_tilt", tilt_in)):
        lo, hi = lim[m]
        out[m] = min(hi, max(lo, out[m] + v * dps * dt))
    return out


def cmd_read(a):
    bus, _ = connect_calibrated(a)
    bus.disable_torque()
    print("torque off: move the head by hand, Ctrl-C to stop")
    try:
        while True:
            pos = bus.sync_read("Present_Position", num_retry=2)
            print("  ".join(f"{m} {v:7.1f}" for m, v in pos.items()), end="\r", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        bus.disconnect(disable_torque=True)
        print()


def read_keys(timeout):
    keys = []
    if select.select([sys.stdin], [], [], timeout)[0]:
        keys = list(os.read(sys.stdin.fileno(), 64).decode(errors="ignore"))
    return keys


def cmd_keys(a):
    if not sys.stdin.isatty():
        raise SystemExit("head.py keys: needs an interactive terminal")
    bus, cal = connect_calibrated(a)
    lim = limits(cal)
    with bus.torque_disabled():
        bus.configure_motors()
        bus.set_operating_mode("position")
        for m in bus.motors:  # the SO-101 follower's gains, which the arm already runs on
            bus.write("P_Coefficient", m, 16)
            bus.write("I_Coefficient", m, 0)
            bus.write("D_Coefficient", m, 32)
    # Start from where the head is, so torque-on never jerks it.
    target = bus.sync_read("Present_Position", num_retry=2)
    bus.sync_write("Goal_Position", target)
    bus.enable_torque()
    step = a.step
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print("a/d pan, w/s tilt, c center, [ ] step, x quit\r")
    try:
        tty.setcbreak(fd)
        run = True
        while run:
            moved = False
            for k in read_keys(0.05):
                if k in KEYS:
                    m, d = KEYS[k]
                    lo, hi = lim[m]
                    target[m] = min(hi, max(lo, target[m] + d * step))
                    moved = True
                elif k == "c":
                    target = {m: 0.0 for m in target}
                    moved = True
                elif k == "[":
                    step = STEPS[max(0, STEPS.index(step) - 1)]
                elif k == "]":
                    step = STEPS[min(len(STEPS) - 1, STEPS.index(step) + 1)]
                elif k in ("x", "\x1b"):
                    run = False
            if moved:
                bus.sync_write("Goal_Position", target)
            print(f"step {step:4.1f}  " + "  ".join(f"{m} {v:7.1f}" for m, v in target.items()),
                  end="\r", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        bus.disconnect(disable_torque=True)
        print("\ntorque off, disconnected")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=os.environ.get("ARM_PORT", "/dev/ttyACM0"))
    ap.add_argument("--id", default="sortbots_head")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    s = sub.add_parser("setup")
    s.add_argument("which", choices=["pan", "tilt"])
    c = sub.add_parser("calibrate")
    c.add_argument("--invert", nargs="*", default=[], choices=list(HEAD),
                   help="servos that turn the wrong way (flips the sign in software)")
    sub.add_parser("read")
    k = sub.add_parser("keys")
    k.add_argument("--step", type=float, default=2.0, choices=STEPS, help="degrees per keypress")
    a = ap.parse_args()
    {"scan": cmd_scan, "setup": cmd_setup, "calibrate": cmd_calibrate,
     "read": cmd_read, "keys": cmd_keys}[a.cmd](a)


if __name__ == "__main__":
    main()
