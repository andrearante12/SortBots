#!/usr/bin/env python3
"""Widen the arm's joint limits to its real end stops, without breaking what was calibrated on top.

The limits are whatever range each joint was swept through during `lerobot-calibrate`. lerobot
writes them into the servos (Min/Max_Position_Limit, which refuse goals outside), and every tool
keeps 2 deg inside them. A short sweep leaves real reach unused: on 2026-11-15 a box picked fine by
hand was out of the planner's reach with wrist_flex at 108 of its 108.7, and a taught drop behind
the base sat with elbow and wrist both at their ends.

The catch: lerobot's degrees are 0 at the MIDDLE of the calibrated range, so a wider range moves
every joint's zero. Everything stored in those degrees would silently shift: the hand-eye solve's
joint offsets, the touch samples, the rest / place / flat poses. `apply` shifts them all by the
exact change, so nothing needs recalibrating.

    scripts/arm.sh limits measure      arm limp: push each joint to both of its stops, Enter
    scripts/arm.sh limits apply        show the plan, then (on "yes") write it, backups first
    scripts/arm.sh limits apply --dry-run

Runs in the `lerobot` env and owns the servo port: stop any other arm tool first.
"""
import argparse
import json
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
# wrist_roll already spans its full turn; the four that fold the arm are the ones to widen
WIDEN = ARM[:4]
MAX_RES = 4095            # STS3215: lerobot's degrees = (raw - mid) * 360 / 4095
MARGIN_DEG = 3.0          # stay this far inside each measured stop: never drive into the hard stop
CAL_DIR = Path(os.environ.get("HF_LEROBOT_CALIBRATION",
                              Path.home() / ".cache/huggingface/lerobot/calibration")).expanduser() / "robots"
ARM_CAL = CAL_DIR / "so_follower" / "sortbots_arm.json"
STOPS = CAL_DIR / "so_follower" / "sortbots_arm_stops.json"
OFFSETS = CAL_DIR / "so_follower" / "sortbots_arm_kin_offsets.json"
# files holding arm angles in lerobot degrees: {"arm_deg": [...]} (one) or a list of them
POSE_FILES = [CAL_DIR / "sortbots_head" / n for n in
              ("handeye_samples.json", "pick_flat_poses.json", "pick_rest_pose.json", "pick_place_pose.json")]


def to_deg(raw, mid):
    return (raw - mid) * 360.0 / MAX_RES


def open_bus(port, arm_id):
    from lerobot.motors import SerialMotorsBus
    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
    arm = SOFollower(SOFollowerRobotConfig(port=port, id=arm_id, use_degrees=True))
    if not arm.calibration:
        raise SystemExit(f"no arm calibration ({ARM_CAL}): run lerobot-calibrate first")
    # a bus of our own, as handeye.open_bus: arm.bus closed itself once `arm` was collected on
    # return, and the first read failed "not connected" (2026-11-15)
    bus = SerialMotorsBus(port=port, motors=dict(arm.bus.motors), calibration=dict(arm.calibration))
    bus.connect()
    if not bus.is_calibrated:
        bus.disconnect(disable_torque=False)
        raise SystemExit("servo registers don't match the calibration file: recalibrate first")
    bus.disable_torque()
    return bus


def cmd_measure(a):
    cal = json.loads(ARM_CAL.read_text())
    bus = open_bus(a.port, a.arm_id)
    seen = {j: [None, None] for j in ARM}
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print("Arm is LIMP: support it. Push every joint to BOTH of its mechanical stops (gently, by hand).\n"
          "Enter = done and save, q = quit without saving.\n")
    try:
        tty.setcbreak(fd)
        while True:
            raw = bus.sync_read("Present_Position", ARM, normalize=False, num_retry=2)
            lines = []
            for j in ARM:
                r = int(raw[j])
                lo_, hi_ = seen[j]
                seen[j] = [r if lo_ is None else min(lo_, r), r if hi_ is None else max(hi_, r)]
                c = cal[j]
                mid = (c["range_min"] + c["range_max"]) / 2
                gain_lo = to_deg(c["range_min"], mid) - to_deg(seen[j][0], mid)
                gain_hi = to_deg(seen[j][1], mid) - to_deg(c["range_max"], mid)
                lines.append(f"{j:13s} now {to_deg(r, mid):+7.1f}  seen {to_deg(seen[j][0], mid):+7.1f} .. "
                             f"{to_deg(seen[j][1], mid):+7.1f}   past the limits: {max(0, gain_lo):4.1f} / "
                             f"{max(0, gain_hi):4.1f} deg")
            print("\x1b[J" + "\n".join(lines) + f"\x1b[{len(lines)}A\r", end="", flush=True)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                k = os.read(fd, 8).decode(errors="ignore")
                if "q" in k:
                    print("\n" * len(ARM) + "quit: nothing saved")
                    return
                if "\n" in k or "\r" in k:
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        bus.disconnect(disable_torque=True)
    STOPS.write_text(json.dumps({"time": time.strftime("%Y-%m-%d %H:%M:%S"),
                                 "raw": {j: seen[j] for j in ARM}}, indent=1))
    print("\n" * len(ARM) + f"saved the stops to {STOPS}\nnext: scripts/arm.sh limits apply")


def plan():
    """{joint: (old_min, old_max, new_min, new_max, zero shift deg)} for the joints that widen."""
    cal = json.loads(ARM_CAL.read_text())
    stops = json.loads(STOPS.read_text())["raw"]
    out = {}
    for j in WIDEN:
        c, (s_lo, s_hi) = cal[j], stops[j]
        m = int(round(MARGIN_DEG * MAX_RES / 360))
        # only ever widen, and never past 0..4095 (the raw range wraps there)
        new_lo = max(0, min(c["range_min"], s_lo + m))
        new_hi = min(MAX_RES, max(c["range_max"], s_hi - m))
        if (new_lo, new_hi) == (c["range_min"], c["range_max"]):
            continue
        shift = ((new_lo + new_hi) / 2 - (c["range_min"] + c["range_max"]) / 2) * 360.0 / MAX_RES
        out[j] = (c["range_min"], c["range_max"], new_lo, new_hi, shift)
    return out


def shift_poses(obj, shift):
    """arm_deg -= shift (per joint) in a pose dict or a list of them; returns how many changed."""
    items = obj if isinstance(obj, list) else [obj]
    n = 0
    for it in items:
        if isinstance(it, dict) and "arm_deg" in it:
            it["arm_deg"] = [round(v - shift.get(j, 0.0), 4) for v, j in zip(it["arm_deg"], ARM)]
            n += 1
    return n


def cmd_apply(a):
    if not STOPS.is_file():
        raise SystemExit("no measured stops yet: scripts/arm.sh limits measure")
    p = plan()
    if not p:
        print("nothing to widen: the measured stops are inside the current limits (plus the margin)")
        return
    print(f"widen ({MARGIN_DEG:g} deg inside the measured stops):")
    for j, (lo0, hi0, lo1, hi1, sh) in p.items():
        m0, m1 = (lo0 + hi0) / 2, (lo1 + hi1) / 2
        print(f"  {j:13s} {to_deg(lo0, m0):+7.1f} .. {to_deg(hi0, m0):+7.1f}  ->  "
              f"{to_deg(lo1, m1):+7.1f} .. {to_deg(hi1, m1):+7.1f} deg   (zero moves {sh:+.2f} deg)")
    shift = {j: v[4] for j, v in p.items()}
    print("then every stored angle of those joints shifts by minus that: "
          + ", ".join(f.name for f in [OFFSETS] + POSE_FILES if f.is_file()))
    if a.dry_run:
        return
    if input("type yes to write: ").strip() != "yes":
        print("nothing written")
        return
    stamp = time.strftime("%Y%m%d-%H%M%S")
    for f in [ARM_CAL, OFFSETS] + POSE_FILES:
        if f.is_file():
            f.with_name(f"{f.stem}.before-limits-{stamp}{f.suffix}").write_text(f.read_text())
    # the servos first: if the bus fails, the files still match what the servos hold
    from lerobot.motors import MotorCalibration
    cal = json.loads(ARM_CAL.read_text())
    for j, (_, _, lo1, hi1, _) in p.items():
        cal[j]["range_min"], cal[j]["range_max"] = int(lo1), int(hi1)
    bus = open_bus(a.port, a.arm_id)
    try:
        bus.disable_torque()  # opens the Feetech EEPROM lock
        bus.write_calibration({j: MotorCalibration(**{k: cal[j][k] for k in
                                                      ("id", "drive_mode", "homing_offset", "range_min", "range_max")})
                               for j in ARM}, cache=False)
        back = bus.read_calibration()
        bad = [j for j in p if (back[j].range_min, back[j].range_max) != (cal[j]["range_min"], cal[j]["range_max"])]
        if bad:
            raise SystemExit(f"the servos didn't take the new limits: {bad}. Nothing else written; "
                             "re-run, or restore with lerobot-calibrate")
    finally:
        bus.disconnect(disable_torque=True)
    ARM_CAL.write_text(json.dumps(cal, indent=4))
    if OFFSETS.is_file():
        off = json.loads(OFFSETS.read_text())
        for j, sh in shift.items():  # q + off must stay the same angle: q drops by sh, off rises by sh
            off["offsets_deg"][j] = off["offsets_deg"].get(j, 0.0) + sh
        off["limits_widened"] = stamp
        OFFSETS.write_text(json.dumps(off, indent=2))
    for f in POSE_FILES:
        if f.is_file():
            obj = json.loads(f.read_text())
            n = shift_poses(obj, shift)
            f.write_text(json.dumps(obj, indent=1))
            print(f"  {f.name}: {n} pose{'s' if n != 1 else ''} shifted")
    print(f"done. Backups: *.before-limits-{stamp}.json beside each file")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=os.environ.get("ARM_PORT", "/dev/ttyACM0"))
    ap.add_argument("--arm-id", default="sortbots_arm")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("measure")
    ap_apply = sub.add_parser("apply")
    ap_apply.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)
    {"measure": cmd_measure, "apply": cmd_apply}[a.cmd](a)


if __name__ == "__main__":
    main()
