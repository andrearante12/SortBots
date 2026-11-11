#!/usr/bin/env python3
"""Calibrate the D435 on the pan/tilt head to the SO-101's base frame, then read package
markers in arm coordinates. Run through `scripts/arm.sh head handeye ...` (lerobot env, no ROS).

    handeye capture   collect samples: a marker taped flat on the magnet face, the arm put
                      in poses by hand, the head in several angles
    handeye solve     fit the head-camera model (scripts/head_cam.py) to the samples, print
                      the error of every sample, save the calibration
    handeye check     live: every visible package's magnet target in base_link (cm), how
                      far the magnet face is from it, and whether the face points up

capture keys (terminal, the preview window only shows):
    f        arm free (torque off: move it by hand) / hold (torque on where it is)
    space    take a sample (arm must be HELD, marker in view; 10 frames averaged)
    a/d w/s  head pan / tilt by the step;  [ ]  step 2/5/10 deg
    u        drop the last sample          x    save and quit

A good set: 6+ arm poses at one head pose that sees the workspace (tilted down: at 0/0 our
head looks over the arm), then 4+ poses at each of 4 more head angles (pan +/-20, tilt +/-15
from there), the magnet spread over the camera's view and over 25-60 cm from it,
the tool tilted different ways. The marker must face the camera.

The 3-D point is the D435 depth at the marker's centre (deprojected), with the marker's
own PnP pose as a fallback where depth has a hole; both are stored.
"""
import argparse
import json
import math
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np

import head as headmod
import head_cam
from so101_kin import SO101Kin

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
STEPS = [2.0, 5.0, 10.0]


def calib_dir():
    return headmod.calib_path("x").parent


class Camera:
    """D435 colour + depth aligned to colour, ArUco DICT_4X4_50."""

    def __init__(self, width=1280, height=720, fps=30, marker_m=0.02):
        import cv2
        import pyrealsense2 as rs
        self.cv2, self.rs, self.marker_m = cv2, rs, marker_m
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        try:
            prof = self.pipe.start(cfg)
        except RuntimeError as e:
            raise SystemExit(f"D435: {e} (plugged into a USB 3 port? `lsusb -t` should say 5000M)")
        self.depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)
        self.intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = np.array([[self.intr.fx, 0, self.intr.ppx], [0, self.intr.fy, self.intr.ppy], [0, 0, 1]])
        self.dist = np.array(self.intr.coeffs[:5], float)
        params = cv2.aruco.DetectorParameters()
        # 2 cm markers are ~30-60 px across here: subpixel corners matter for PnP
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)
        for _ in range(15):  # auto-exposure settles
            self.pipe.wait_for_frames()

    def grab(self):
        fs = self.align.process(self.pipe.wait_for_frames())
        color = np.asanyarray(fs.get_color_frame().get_data())
        depth = np.asanyarray(fs.get_depth_frame().get_data()).astype(float) * self.depth_scale
        return color, depth

    def markers(self, color, depth):
        """{id: {"px", "depth_pt", "pnp_pt", "size_px", "normal", "target_pt"}}; colour optical
        frame, metres. target_pt is the package's magnet target (head_cam.TARGET_FROM_TOP_EDGE_M)."""
        cv2 = self.cv2
        corners, ids, _ = self.det.detectMarkers(color)
        out = {}
        if ids is None:
            return out, corners, ids
        h = self.marker_m / 2
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
        for c, i in zip(corners, ids.ravel()):
            q = c.reshape(4, 2)
            u, v = q.mean(0)
            # depth: median over the marker's inner half (the black/white print is fine for the
            # D435's IR stereo; the edge pixels mix in whatever is behind)
            r = max(2, int(np.linalg.norm(q[0] - q[2]) / 6))
            win = depth[int(v) - r:int(v) + r + 1, int(u) - r:int(u) + r + 1]
            win = win[(win > 0.1) & (win < 3.0)]
            dpt = None
            if win.size > 0.3 * (2 * r + 1) ** 2:
                dpt = list(self.rs.rs2_deproject_pixel_to_point(self.intr, [float(u), float(v)], float(np.median(win))))
            ok, rvec, tvec = cv2.solvePnP(obj, q.astype(np.float32), self.K, self.dist,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            size_px = float(np.linalg.norm(q[0] - q[2]) / 1.414)
            m = {"px": [float(u), float(v)], "depth_pt": dpt,
                 "pnp_pt": [float(t) for t in tvec.ravel()] if ok else None, "size_px": size_px,
                 "normal": None, "target_pt": None}
            centre = dpt if dpt is not None else m["pnp_pt"]
            if ok and centre is not None:
                R = cv2.Rodrigues(rvec)[0]
                n = self.face_normal(depth, u, v, size_px)
                if n is not None:
                    R = head_cam.align_z(R, n)
                m["normal"] = [float(x) for x in R[:, 2]]
                m["target_pt"] = [float(x) for x in
                                  np.asarray(centre) + R @ head_cam.target_in_marker(self.marker_m)]
            out[int(i)] = m
        return out, corners, ids

    def face_normal(self, depth, u, v, size_px):
        """Unit normal of the surface round a marker, from depth: a box 3 marker-widths wide
        (the target is ~2.6 widths from the centre, on the same face). None if too few points."""
        r = int(1.5 * size_px)
        h, w = depth.shape
        v0, v1, u0, u1 = max(0, int(v) - r), min(h, int(v) + r), max(0, int(u) - r), min(w, int(u) + r)
        z = depth[v0:v1:2, u0:u1:2]
        vv, uu = np.mgrid[v0:v1:2, u0:u1:2]
        ok = (z > 0.1) & (z < 3.0)
        if ok.sum() < 50:
            return None
        z, uu, vv = z[ok], uu[ok], vv[ok]
        # colour stream is distortion-free on the D435 (coeffs all 0), so plain pinhole
        pts = np.stack([(uu - self.intr.ppx) / self.intr.fx * z, (vv - self.intr.ppy) / self.intr.fy * z, z], 1)
        return head_cam.plane_fit(pts)[1]

    def point(self, m):
        return m["depth_pt"] if m["depth_pt"] is not None else m["pnp_pt"]

    def close(self):
        self.pipe.stop()


def open_bus(port, arm_id, head_id):
    """Arm + head on one bus, torque OFF, nothing moved. Hold/free per group with set_hold."""
    from lerobot.motors import SerialMotorsBus
    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
    arm = SOFollower(SOFollowerRobotConfig(port=port, id=arm_id, use_degrees=True))
    head_cal = headmod.load_calibration(head_id)
    if not arm.calibration or head_cal is None:
        raise SystemExit("arm or head calibration file missing (lerobot-calibrate / arm.sh head calibrate)")
    bus = SerialMotorsBus(port=port, motors={**arm.bus.motors, **headmod.head_motors()},
                          calibration={**arm.calibration, **head_cal})
    bus.connect()
    if not bus.is_calibrated:
        bus.disconnect(disable_torque=False)
        raise SystemExit("servo registers don't match the calibration files: recalibrate first")
    with bus.torque_disabled():
        bus.configure_motors()
        bus.set_operating_mode("position")
        for m in bus.motors:  # the SO-101 follower's gains
            bus.write("P_Coefficient", m, 16)
            bus.write("I_Coefficient", m, 0)
            bus.write("D_Coefficient", m, 32)
    bus.disable_torque()  # torque_disabled() turns it back on as it exits
    return bus, head_cal


def set_hold(bus, motors, hold):
    """Torque on where the joints ARE (goal written first, so nothing jumps) or off."""
    if hold:
        bus.sync_write("Goal_Position", bus.sync_read("Present_Position", motors, num_retry=2))
        bus.enable_torque(motors)
    else:
        bus.disable_torque(motors)


def read_keys(timeout):
    if select.select([sys.stdin], [], [], timeout)[0]:
        return list(os.read(sys.stdin.fileno(), 64).decode(errors="ignore"))
    return []


def draw(cam, color, corners, ids, text):
    cv2 = cam.cv2
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(color, corners, ids)
    cv2.putText(color, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.imshow("handeye", color)
    cv2.waitKey(1)


def cmd_capture(a):
    path = calib_dir() / "handeye_samples.json"
    samples = [] if a.fresh or not path.is_file() else json.loads(path.read_text())
    cam = Camera(marker_m=a.marker_m)
    bus, head_cal = open_bus(a.port, a.arm_id, a.head_id)
    lim = headmod.limits(head_cal)
    head = {m: v for m, v in bus.sync_read("Present_Position", list(headmod.HEAD), num_retry=2).items()}
    set_hold(bus, list(headmod.HEAD), True)
    arm_held, step = False, 5.0
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(f"{len(samples)} samples loaded from {path}. f free/hold arm, space sample, a/d w/s head, u undo, x quit\r")
    try:
        tty.setcbreak(fd)
        run = True
        while run:
            color, depth = cam.grab()
            ms, corners, ids = cam.markers(color, depth)
            m = ms.get(a.marker_id)
            msg = ""
            for k in read_keys(0.0):
                if k == "f":
                    arm_held = not arm_held
                    set_hold(bus, ARM, arm_held)
                elif k in "adws":
                    mot, d = {"a": ("head_pan", 1), "d": ("head_pan", -1),
                              "w": ("head_tilt", 1), "s": ("head_tilt", -1)}[k]
                    lo, hi = lim[mot]
                    head[mot] = min(hi, max(lo, head[mot] + d * step))
                    bus.sync_write("Goal_Position", head)
                elif k == "[":
                    step = STEPS[max(0, STEPS.index(step) - 1)]
                elif k == "]":
                    step = STEPS[min(len(STEPS) - 1, STEPS.index(step) + 1)]
                elif k == "u" and samples:
                    samples.pop()
                    msg = "dropped last sample"
                elif k == " ":
                    msg = take_sample(a, cam, bus, samples, arm_held)
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text(json.dumps(samples, indent=1))
                elif k in ("x", "\x1b"):
                    run = False
            groups = len(head_cam.head_groups(samples)) if samples else 0
            pt = cam.point(m) if m else None
            status = (f"{len(samples)} samples / {groups} head poses | arm {'HELD' if arm_held else 'free'} | "
                      f"head {head['head_pan']:+5.1f}/{head['head_tilt']:+5.1f} step {step:g} | marker "
                      + (f"{a.marker_id} {m['size_px']:.0f}px z{pt[2] * 100:.1f}cm" if pt else "--"))
            print("\r" + status + (" | " + msg if msg else "") + "\x1b[K", end="", flush=True)
            if not a.no_view:
                draw(cam, color, corners, ids, status)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(samples, indent=1))
        bus.disconnect(disable_torque=True)
        cam.close()
        print(f"\n{len(samples)} samples in {path}; torque off")


def take_sample(a, cam, bus, samples, arm_held):
    if not arm_held:
        return "hold the arm first (f): a free arm drifts while the frames are taken"
    pts, pnps = [], []
    for _ in range(10):
        color, depth = cam.grab()
        m = cam.markers(color, depth)[0].get(a.marker_id)
        if m:
            pts.append(cam.point(m))
            if m["pnp_pt"]:
                pnps.append(m["pnp_pt"])
    if len(pts) < 6:
        return f"marker {a.marker_id} seen in {len(pts)}/10 frames: not taken"
    pts = np.array(pts)
    spread = float(np.linalg.norm(pts.std(0)) * 1000)
    if spread > 4.0:
        return f"point jitters {spread:.1f} mm: not taken (depth edge? tilt the marker to the camera)"
    pos = bus.sync_read("Present_Position", num_retry=2)
    samples.append({"arm_deg": [pos[j] for j in ARM], "head_deg": [pos["head_pan"], pos["head_tilt"]],
                    "p_cam": [float(v) for v in np.median(pts, 0)],
                    "pnp_cam": [float(v) for v in np.median(pnps, 0)] if pnps else None,
                    "t": time.time()})
    return f"sample {len(samples)} taken (jitter {spread:.1f} mm)"


def cmd_solve(a):
    path = calib_dir() / "handeye_samples.json"
    if not path.is_file():
        raise SystemExit(f"no samples yet ({path}): run `scripts/arm.sh head handeye capture "
                         "--marker-id N` first and take samples with space")
    samples = json.loads(path.read_text())
    keep = [i for i in range(len(samples)) if i not in set(a.drop)]
    samples = [samples[i] for i in keep]  # numbering below stays the capture's
    kin = SO101Kin()
    pan_axis = None
    if a.pan_axis:  # measured from the shoulder_pan axis at the mounting surface
        pan_axis = np.array(a.pan_axis) + [head_cam.PAN_AXIS_X_M, 0.0, 0.0]
    hc, err = head_cam.fit(samples, kin, pan_axis=pan_axis, tilt_fwd=a.tilt_fwd,
                           magnet_m=a.magnet_m, fit_magnet=not a.fixed_magnet)
    med = float(np.median(err))
    print(f"{len(samples)} samples, {len(head_cam.head_groups(samples))} head poses")
    for i, s, e in zip(keep, samples, err):
        flag = "  <-- outlier? (drop it with --drop)" if e > max(3 * med, 8.0) else ""
        print(f"  {i:3d} head {s['head_deg'][0]:+6.1f}/{s['head_deg'][1]:+6.1f}  err {e:5.1f} mm{flag}")
    rms = float(np.sqrt(np.mean(err ** 2)))
    print(f"RMS {rms:.1f} mm, max {err.max():.1f} mm; marker surface {hc.magnet_m * 1000:.1f} mm "
          "out from the flange")
    # Sanity check against the rig (measured 2026-11-11): the lens is 457.2 mm above the mounting
    # surface and its mast 127 mm horizontally from the arm base's centre. The lens moves a few
    # cm as the head turns, so this is a ballpark, not a pass/fail.
    pan0, tilt0 = max(head_cam.head_groups(samples).values(), key=len)[0]["head_deg"]
    lens = hc.matrix(pan0, tilt0)[:3, 3]
    horiz = math.hypot(lens[0] - head_cam.PAN_AXIS_X_M, lens[1])
    print(f"lens at head {pan0:+.0f}/{tilt0:+.0f}: {lens[2] * 1000:.0f} mm up (rig: ~457), "
          f"{horiz * 1000:.0f} mm from the arm base centre (rig: ~127)")
    out = calib_dir() / f"{a.head_id}_camera.json"
    hc.save(out, {"rms_mm": rms, "n": len(samples), "marker_m": a.marker_m, "saved": time.ctime()})
    print("saved", out)
    if rms > 10:
        print("RMS over 1 cm: check the marker size (--marker-m), that the marker is flat on the "
              "magnet face, and that the arm calibration is recent")


def cmd_check(a):
    hc = head_cam.HeadCam.load(calib_dir() / f"{a.head_id}_camera.json")
    kin = SO101Kin()
    cam = Camera(marker_m=a.marker_m)
    bus, _ = open_bus(a.port, a.arm_id, a.head_id)
    print("torque off everywhere: move the arm/head by hand. Ctrl-C quits")
    try:
        while True:
            color, depth = cam.grab()
            ms, corners, ids = cam.markers(color, depth)
            pos = bus.sync_read("Present_Position", num_retry=2)
            tcp = kin.tool_point(np.radians([pos[j] for j in ARM]), hc.magnet_m)
            parts = [f"magnet {np.round(tcp * 100, 1)}"]
            for i, m in sorted(ms.items()):
                if m["target_pt"] is None:
                    continue
                T = hc.matrix(pos["head_pan"], pos["head_tilt"])
                pb = (T @ np.append(m["target_pt"], 1.0))[:3]
                nz = (T[:3, :3] @ m["normal"])[2]  # ~1 for a top face, ~0 for a side face
                parts.append(f"id{i} target {np.round(pb * 100, 1)} up{nz:+.2f} "
                             f"d{np.linalg.norm(pb - tcp) * 100:.1f}cm")
            line = " | ".join(parts)
            print("\r" + line + "\x1b[K", end="", flush=True)
            if not a.no_view:
                draw(cam, color, corners, ids, line[:90])
    except KeyboardInterrupt:
        pass
    finally:
        bus.disconnect(disable_torque=True)
        cam.close()
        print()


def main(argv=None):
    ap = argparse.ArgumentParser(prog="handeye", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=os.environ.get("ARM_PORT", "/dev/ttyACM0"))
    ap.add_argument("--arm-id", default="sortbots_arm")
    ap.add_argument("--head-id", default="sortbots_head")
    ap.add_argument("--marker-m", type=float, default=0.020,
                    help="black square's outer edge, metres (measure the print)")
    ap.add_argument("--no-view", action="store_true", help="no preview window (headless / ssh)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("--marker-id", type=int, default=0, help="id of the marker taped on the magnet")
    c.add_argument("--fresh", action="store_true", help="discard the saved samples")
    s = sub.add_parser("solve")
    s.add_argument("--pan-axis", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="ruler guess, metres, from the shoulder_pan axis at the mounting surface "
                        "(x forward, y left, z up); only seeds the fit")
    s.add_argument("--tilt-fwd", type=float, default=0.02, help="ruler guess: tilt axis ahead of pan axis, m")
    s.add_argument("--magnet-m", type=float, default=head_cam.MAGNET_FACE_M,
                   help="flange to marker surface along the tool axis, m (seed, or fixed with --fixed-magnet)")
    s.add_argument("--fixed-magnet", action="store_true", help="don't fit the magnet length")
    s.add_argument("--drop", type=int, nargs="*", default=[], help="sample numbers to leave out")
    sub.add_parser("check")
    a = ap.parse_args(argv)
    {"capture": cmd_capture, "solve": cmd_solve, "check": cmd_check}[a.cmd](a)


if __name__ == "__main__":
    main()
