#!/usr/bin/env python3
"""Calibrate the D435 on the pan/tilt head to the SO-101's base frame, then read package
markers in arm coordinates. Run through `scripts/arm.sh head handeye ...` (lerobot env, no ROS).

    handeye capture   collect samples: a marker taped flat on the magnet face, the arm put
                      in poses by hand, the head in several angles. --touch: no marker on the
                      magnet; look at a package's marker, then touch the magnet on its circle
    handeye solve     fit the head-camera model (scripts/head_cam.py) to the samples, print
                      the error of every sample, save the calibration
    handeye check     live: every visible package's magnet target in base_link (cm), how
                      far the magnet face is from it, and whether the face points up

capture with the XInput pad (`--pad`, the same hand-space driving as `arm.sh pad`; the arm
starts HELD and the pad owns it, so no hands on the arm):
    hold RT  + sticks   move the hand (left: plane, right Y: up/down)
    Y / A               wrist_flex     LB / LT  tool tilt     d-pad up/down  speed   L3 slow
    hold RB + right stick   head pan / tilt
    B        take a sample            BACK  drop the last one      START  save and quit
The keyboard keys below keep working alongside it.

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
import contextlib
import json
import math
import os
import select
import sys
import tempfile
import threading
import termios
import time
import tty
from pathlib import Path

import numpy as np

import head as headmod
import capture_guide
import head_cam
import so101_kin
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
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        prof = self._start(cfg)
        self.depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)
        self.intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = np.array([[self.intr.fx, 0, self.intr.ppx], [0, self.intr.fy, self.intr.ppy], [0, 0, 1]])
        self.dist = np.array(self.intr.coeffs[:5], float)
        params = cv2.aruco.DetectorParameters()
        # 2 cm markers are ~30-60 px across here: subpixel corners matter for PnP
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)

    def _start(self, cfg):
        """Start streaming and see frames arrive. A D435 left mid-stream by a crashed run starts
        fine and then sends nothing ("Frame didn't arrive within 5000", 2026-11-11); a hardware
        reset clears it, so do that once before giving up."""
        rs = self.rs
        for attempt in (1, 2):
            self.pipe = rs.pipeline()
            try:
                prof = self.pipe.start(cfg)
            except RuntimeError as e:
                raise SystemExit(f"D435: {e} (plugged into a USB 3 port? `lsusb -t` should say 5000M)")
            try:
                for _ in range(15):  # and auto-exposure settles
                    self.pipe.wait_for_frames()
                return prof
            except RuntimeError:
                self.pipe.stop()
                if attempt == 2:
                    raise SystemExit("D435 sends no frames even after a reset: unplug and replug it")
                print("D435 sends no frames: resetting it (~6 s)")
                prof.get_device().hardware_reset()
                time.sleep(6)

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
                # Fit the face's plane halfway to the target, not round the marker: the package's
                # marker sits near a corner of its face, so a box round it takes in the table
                # and the side face too.
                mid = np.asarray(centre) + R @ (head_cam.target_in_marker(self.marker_m) / 2)
                n = self.face_normal(depth, self.intr.fx * mid[0] / mid[2] + self.intr.ppx,
                                     self.intr.fy * mid[1] / mid[2] + self.intr.ppy, size_px)
                if n is not None:
                    R = head_cam.align_z(R, n)
                m["normal"] = [float(x) for x in R[:, 2]]
                m["target_pt"] = [float(x) for x in
                                  np.asarray(centre) + R @ head_cam.target_in_marker(self.marker_m)]
            out[int(i)] = m
        return out, corners, ids

    def face_normal(self, depth, u, v, size_px):
        """Unit normal of the surface round pixel (u, v), from depth: a box 2.4 marker-widths
        wide. None if too few points."""
        r = int(1.2 * size_px)
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


PREVIEW = {"window": True, "file_at": 0.0}
# Its own directory, so the page can be served (`python3 -m http.server --directory`) without
# exposing the rest of /tmp; a browser pane that won't live-reload file:// needs that.
PREVIEW_DIR = Path(tempfile.gettempdir()) / "handeye_preview"
PREVIEW_JPG = PREVIEW_DIR / "handeye_view.jpg"
PREVIEW_HTML = PREVIEW_JPG.with_suffix(".html")
# The page is opened from file://, where fetch() of a sibling file is blocked; a <script src>
# is not, so the guide state goes out as a JS file the page re-includes every 300 ms.
PREVIEW_JS = PREVIEW_JPG.with_name("handeye_status.js")
PREVIEW_PAGE = """<!doctype html><meta charset=utf-8><title>handeye capture</title>
<style>
body{margin:0;background:#111;color:#e8e8e8;font:15px/1.4 system-ui,sans-serif;display:flex;gap:14px}
#img{flex:1 1 auto;min-width:0}#img img{width:100%;display:block}
#p{flex:0 0 360px;padding:12px 14px 12px 0}
h1{font-size:20px;margin:2px 0 10px}.step{color:#9ab;font-size:13px;margin-bottom:2px}
.do{background:#1d2d1d;border-left:4px solid #5c5;padding:10px 12px;border-radius:4px;font-size:16px}
.do p{margin:0 0 6px}.ready{border-color:#5f5;background:#1f3d1f}
ul{list-style:none;padding:0;margin:10px 0}li{padding:2px 0}.ok{color:#6d6}.no{color:#e77}
table{border-collapse:collapse;width:100%;margin-top:10px;font-size:14px}
td{padding:4px 6px;border-bottom:1px solid #2a2a2a}tr.cur td{background:#202838}
.go{font-size:18px;font-weight:600;color:#6f6;margin:8px 0}.msg{color:#fd6;margin-top:8px}
.st{color:#999;font-size:12px;margin-top:10px;word-break:break-word}
@media (max-width:900px){body{flex-direction:column}#p{flex:none;padding:12px}}
</style>
<div id=img><img id=v alt="camera"></div><div id=p>waiting for capture...</div>
<script>
function esc(t){return String(t).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
function render(S){
  var g=S.guide,h='<div class=step>step '+g.stage+' of '+g.stages+'</div><h1>'+esc(g.title)+'</h1>';
  h+='<div class="do'+(g.ready?' ready':'')+'">'+g.lines.map(l=>'<p>'+esc(l)+'</p>').join('')+'</div>';
  if(g.checks.length)h+='<ul>'+g.checks.map(c=>'<li class='+(c[1]?'ok':'no')+'>'+(c[1]?'&#10003; ':'&#10007; ')+esc(c[0])+'</li>').join('')+'</ul>';
  if(g.ready)h+='<div class=go>Ready: press B</div>';
  h+='<table>'+g.progress.map((r,i)=>'<tr'+(i+1==g.stage?' class=cur':'')+'><td>'+esc(r.name)+
     (r.target?' <span style=color:#888>('+r.target[0].toFixed(0)+', '+r.target[1].toFixed(0)+')</span>':'')+
     '</td><td>'+r.have+' / '+r.need+'</td><td>'+(r.have>=r.need?'&#10003;':'')+'</td></tr>').join('')+'</table>';
  if(S.msg)h+='<div class=msg>'+esc(S.msg)+'</div>';
  h+='<div class=st>'+esc(S.status)+'</div>';
  document.getElementById('p').innerHTML=h}
function tick(){
  v.src='handeye_view.jpg?'+Date.now();
  var s=document.createElement('script');s.src='handeye_status.js?'+Date.now();
  s.onload=s.onerror=function(){s.remove();if(window.S)render(window.S)};document.body.appendChild(s)}
setInterval(tick,300);tick();
</script>"""


def draw(cam, color, corners, ids, text, guide=None, msg=""):
    """Preview in a window, or, where OpenCV can't open one, a JPEG + self-reloading page.
    lerobot pulls in opencv-python-HEADLESS (imshow raises "not implemented", 2026-11-11), and
    installing the GUI build beside it fights over the same cv2 module, so don't."""
    cv2 = cam.cv2
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(color, corners, ids)
    if guide and guide.get("reach"):
        dots = color.copy()
        for u, v in guide["reach"]:
            cv2.circle(dots, (int(u), int(v)), 9, (80, 200, 80), -1)
        cv2.addWeighted(dots, 0.35, color, 0.65, 0, dst=color)
    if guide and guide["box"]:
        x0, y0, x1, y1 = guide["box"]
        cv2.rectangle(color, (x0 + 3, y0 + 3), (x1 - 3, y1 - 3), (0, 255, 0) if guide["ready"] else (0, 200, 0),
                      6 if guide["ready"] else 3)
    cv2.putText(color, text if not guide else guide["title"], (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (0, 255, 255), 2)
    if PREVIEW["window"]:
        try:
            cv2.imshow("handeye", color)
            cv2.waitKey(1)
            return
        except cv2.error:
            PREVIEW["window"] = False
            PREVIEW_DIR.mkdir(exist_ok=True)
            PREVIEW_HTML.write_text(PREVIEW_PAGE)
            print(f"\r\x1b[Kno GUI in this OpenCV: live preview in a browser:  xdg-open {PREVIEW_HTML}\n")
    now = time.monotonic()
    if now - PREVIEW["file_at"] > 0.25:
        PREVIEW["file_at"] = now
        tmp = PREVIEW_JPG.with_name(".handeye_view.jpg")  # rename is atomic: no torn frames
        cv2.imwrite(str(tmp), color, [cv2.IMWRITE_JPEG_QUALITY, 70])
        os.replace(tmp, PREVIEW_JPG)
        if guide is not None:
            tmp = PREVIEW_JS.with_name(".handeye_status.js")
            tmp.write_text("window.S=" + json.dumps({"guide": guide, "status": text, "msg": msg}) + ";")
            os.replace(tmp, PREVIEW_JS)


def cmd_capture(a):
    path = calib_dir() / "handeye_samples.json"
    samples = [] if a.fresh or not path.is_file() else json.loads(path.read_text())
    if a.marker_id is None:
        a.marker_id = 10 if a.touch else 0
    cam = Camera(marker_m=a.marker_m)
    bus, head_cal = open_bus(a.port, a.arm_id, a.head_id)
    lim = headmod.limits(head_cal)
    head = {m: v for m, v in bus.sync_read("Present_Position", list(headmod.HEAD), num_retry=2).items()}
    set_hold(bus, list(headmod.HEAD), True)
    arm_held, step = False, 5.0
    pad = None
    if a.pad:
        set_hold(bus, ARM, True)  # the pad drives a held arm; f still frees it
        arm_held = True
        try:
            pad = PadArm(bus, lim, head)
        except BaseException:  # no pad / no permission: leave the arm limp, camera stopped
            bus.disconnect(disable_torque=True)
            cam.close()
            raise
    lock = pad.lock if pad else contextlib.nullcontext()
    note = {"msg": "", "until": 0.0}
    pending = None  # --touch: the camera's look at the package, waiting for the magnet's touch
    # --touch: the reach map, rebuilt in a thread (a second or so of IK) whenever the number of
    # home touches changes, so the camera loop and the pad never wait on it
    reach = {"key": None, "px": None, "busy": False}
    kin_reach, (lo_reach, hi_reach) = SO101Kin(), arm_limits(bus)

    def build_reach(snapshot, key):
        try:
            reach["px"] = reach_map(snapshot, kin_reach, lo_reach, hi_reach, cam.intr,
                                    (cam.intr.width, cam.intr.height))
        finally:
            reach["key"], reach["busy"] = key, False

    def say(text, secs=3.0):  # a result stays on the status line long enough to read
        note["msg"], note["until"] = text, time.monotonic() + secs
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
            keys = read_keys(0.0)
            if pad:
                if pad.error is not None:
                    raise SystemExit(f"pad thread stopped: {pad.error}")
                keys += [{"sample": " ", "undo": "u", "quit": "x"}[e] for e in sorted(pad.take_events())]
                head = pad.head
            for k in keys:
                if k == "f":
                    arm_held = not arm_held
                    with lock:
                        set_hold(bus, ARM, arm_held)
                        if pad:
                            pad.resync()
                elif k in "adws":
                    mot, d = {"a": ("head_pan", 1), "d": ("head_pan", -1),
                              "w": ("head_tilt", 1), "s": ("head_tilt", -1)}[k]
                    lo, hi = lim[mot]
                    with lock:
                        head = dict(pad.head if pad else head)
                        head[mot] = min(hi, max(lo, head[mot] + d * step))
                        bus.sync_write("Goal_Position", head)
                        if pad:
                            pad.head = head
                elif k == "[":
                    step = STEPS[max(0, STEPS.index(step) - 1)]
                elif k == "]":
                    step = STEPS[min(len(STEPS) - 1, STEPS.index(step) + 1)]
                elif k == "u" and pending is not None:
                    pending = None
                    say("look cancelled: take it again")
                elif k == "u" and samples:
                    samples.pop()
                    say("dropped last sample")
                elif k == " " and a.touch and pending is None:
                    msg_, pending = take_look(a, cam, head)
                    say(msg_, 6.0)
                elif k == " " and a.touch:
                    msg_, done = take_touch(bus, samples, arm_held, lock, pending)
                    say(msg_, 6.0)
                    if done:
                        pending = None
                        path.parent.mkdir(parents=True, exist_ok=True)
                        path.write_text(json.dumps(samples, indent=1))
                elif k == " ":
                    say(take_sample(a, cam, bus, samples, arm_held, lock))
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text(json.dumps(samples, indent=1))
                elif k in ("x", "\x1b"):
                    run = False
            groups = len(head_cam.head_groups(samples)) if samples else 0
            pt = (m["target_pt"] if a.touch else cam.point(m)) if m else None
            status = (f"{len(samples)} samples / {groups} head poses | arm {'HELD' if arm_held else 'free'} | "
                      f"head {head['head_pan']:+5.1f}/{head['head_tilt']:+5.1f} step {step:g} | marker "
                      + (f"{a.marker_id} {m['size_px']:.0f}px z{pt[2] * 100:.1f}cm" if pt else "--"))
            if pad:
                status += f" | pad L{pad.level + 1}" + ("" if pad.ok else " LIMIT")
            if a.touch:
                status += " | TOUCH the circle, then B" if pending else " | LOOK: arm clear, then B"
            msg = note["msg"] if time.monotonic() < note["until"] else ""
            print("\r" + status + (" | " + msg if msg else "") + "\x1b[K", end="", flush=True)
            if a.touch and not reach["busy"] and reach["key"] != len(samples):
                reach["busy"] = True
                threading.Thread(target=build_reach, args=([dict(x) for x in samples], len(samples)),
                                 daemon=True).start()
            if not a.no_view:
                mk = {"px": m["px"], "size_px": m["size_px"], "z": pt[2] if pt else None} if m else None
                g = capture_guide.guide(samples, [head["head_pan"], head["head_tilt"]], mk,
                                        (color.shape[1], color.shape[0]), lim,
                                        mode="touch" if a.touch else "magnet", pending=pending is not None,
                                        marker_id=a.marker_id, reach_px=reach["px"])
                draw(cam, color, corners, ids, status, g, msg)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(samples, indent=1))
        if pad:
            pad.close()  # stop its thread before the bus goes away under it
        bus.disconnect(disable_torque=True)
        cam.close()
        print(f"\n{len(samples)} samples in {path}; torque off")


class PadArm:
    """The XInput pad driving the arm (arm_xpad.cartesian_step: the IK and speed caps teleop
    already runs on) and the head, inside capture: capture owns the servo port, so the pad
    can't be a second process. It runs in its OWN 50 Hz thread: ticking it from the camera
    loop (frame wait + depth align + ArUco, 1280x720) fed the arm a goal every 50-100 ms and
    driving lagged badly (2026-11-11). `lock` serialises every bus access between the two."""

    PERIOD_S = 0.02

    def __init__(self, bus, head_lim, head, level=2):
        import types
        import arm_xpad
        from xinput_pad import XPad, deadzone
        self.step_fn, self.deadzone = arm_xpad.cartesian_step, deadzone
        self.pad = XPad()
        self.bus, self.head_lim, self.kin = bus, head_lim, SO101Kin()
        self.head = dict(head)
        res = 4095
        lo, hi = [], []
        for j in ARM:  # calibrated range in URDF radians, 2 deg inside the stops (as arm_xpad)
            c = bus.calibration[j]
            mid = (c.range_min + c.range_max) / 2
            lo.append(np.radians((c.range_min - mid) * 360 / res + 2))
            hi.append(np.radians((c.range_max - mid) * 360 / res - 2))
        self.lo, self.hi = np.array(lo), np.array(hi)
        self.sign = np.ones(5)
        self.a = types.SimpleNamespace(speed=0.10, pitch_dps=45.0, roll_dps=90.0, joint_dps=150.0,
                                       lock=["wrist_roll"])
        self.level = level - 1   # x0.5 by default: positioning, not racing
        self.prev, self.events, self.ok, self.error = {}, set(), True, None
        self.lock = threading.Lock()
        self.resync()
        self._run = True
        self._last = time.monotonic()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def resync(self):
        """Targets back to where the arm IS. Caller holds `lock` (or the thread isn't running)."""
        pos = self.bus.sync_read("Present_Position", ARM, num_retry=2)
        self.q = np.clip(np.radians([pos[j] for j in ARM]), self.lo, self.hi)
        self.target = self.kin.pose(self.q)

    def _loop(self):
        try:
            while self._run:
                t0 = time.monotonic()
                st = self.pad.poll(5)  # outside the lock: it can wait a few ms for a report
                with self.lock:
                    self._tick(st)
                time.sleep(max(0.0, self.PERIOD_S - (time.monotonic() - t0)))
        except BaseException as e:  # pad unplugged (SystemExit from poll), bus error: surface it
            self.error = e

    def _tick(self, st):
        now = time.monotonic()
        dt, self._last = min(0.1, now - self._last), now
        btn = st["buttons"]
        edge = {b for b in ("B", "BACK", "START", "DUP", "DDOWN") if btn[b] and not self.prev.get(b)}
        self.prev = dict(btn)
        self.events |= {e for b, e in (("B", "sample"), ("BACK", "undo"), ("START", "quit")) if b in edge}
        if "DUP" in edge:
            self.level = min(3, self.level + 1)
        if "DDOWN" in edge:
            self.level = max(0, self.level - 1)
        dz = self.deadzone
        if btn["RB"] and self.head_lim:
            pan_in, tilt_in = headmod.head_axes(-dz(st["rx"]), dz(st["ry"]))
            new = headmod.head_step(self.head, pan_in, tilt_in, 60.0, dt, self.head_lim)
            if new != self.head:
                self.head = new
                self.bus.sync_write("Goal_Position", self.head)
            st = {**st, "rx": 0.0, "ry": 0.0}
        if st["rt"] > 0.5:
            inp = {"lx": dz(st["lx"]), "ly": dz(st["ly"]), "rx": dz(st["rx"]), "ry": dz(st["ry"]),
                   "tilt": float(btn["LB"]) - float(st["lt"] > 0.5),
                   "wf": float(btn["Y"]) - float(btn["A"]), "slow": btn["L3"]}
            self.q, self.target, self.ok = self.step_fn(self.kin, self.q, self.target, inp, self.a,
                                                        self.lo, self.hi, self.sign, dt, self.level)
            self.bus.sync_write("Goal_Position", dict(zip(ARM, np.degrees(self.q))))
        else:
            self.resync()  # re-grabbing never snaps back to a stale target

    def take_events(self):
        with self.lock:
            ev, self.events = self.events, set()
        return ev

    def close(self):
        self._run = False
        self._thread.join(timeout=1.0)
        self.pad.close()


def arm_limits(bus):
    """Calibrated joint range in URDF radians, 2 deg inside the stops (as arm_xpad)."""
    lo, hi = [], []
    for j in ARM:
        c = bus.calibration[j]
        mid = (c.range_min + c.range_max) / 2
        lo.append(np.radians((c.range_min - mid) * 360 / 4095 + 2))
        hi.append(np.radians((c.range_max - mid) * 360 / 4095 - 2))
    return np.array(lo), np.array(hi)


_REACH_RZ = {}


def down_reach_rz(kin, lo, hi, n=25000, within_deg=10.0):
    """(r, z) of the magnet face over the arm's straight-down poses (tool within `within_deg`
    of vertical), r measured out from the shoulder_pan axis. The reach is the same at every pan
    angle, so one sweep at pan 0 covers the table. Sampled, not solved: IK on "pitch = -90 deg"
    sits on the elevation fold (the same one that froze wrist_flex) and stalls there."""
    key = (tuple(np.round(lo, 3)), tuple(np.round(hi, 3)))
    if key not in _REACH_RZ:
        rng = np.random.default_rng(0)
        cos_lim, pts = math.cos(math.radians(within_deg)), []
        for _ in range(n):
            q = rng.uniform(lo, hi)
            q[0] = q[4] = 0.0
            F = kin.flange(q)
            if -F[2, 2] < -cos_lim:  # tool axis is the flange's -z
                p = F[:3, 3] - head_cam.MAGNET_FACE_M * F[:3, 2]
                if p[0] > head_cam.PAN_AXIS_X_M:  # reaching forward, not back over the shoulder
                    pts.append((math.hypot(p[0] - head_cam.PAN_AXIS_X_M, p[1]), p[2]))
        _REACH_RZ[key] = np.array(pts)
    return _REACH_RZ[key]


def reach_map(samples, kin, lo, hi, intr, wh, step_m=0.03, tol_m=0.012):
    """Pixels (at the home head angle) of package-top spots the magnet can touch pointing down.
    Built from a ROUGH rigid fit of the home touches (>= 3, not in a line), good to a cm or two:
    enough to keep the guide's boxes inside the reach, not a calibration. None until then."""
    home = samples[0]["head_deg"] if samples else None
    g = [s for s in samples if s.get("kind") == "touch" and home
         and max(abs(s["head_deg"][0] - home[0]), abs(s["head_deg"][1] - home[1])) <= capture_guide.GROUP_DEG]
    if len(g) < 3 or not np.any(kin.off):
        # without measured joint offsets the arm model can be 10 cm out (2026-11-11), and a map
        # built on it boxes spots the arm can't reach: text guidance until `solve` has run once
        return None
    tcp = np.array([kin.tool_point(np.radians(s["arm_deg"]), head_cam.MAGNET_FACE_M) for s in g])
    if np.linalg.svd(tcp - tcp.mean(0), compute_uv=False)[1] < 0.02:
        return None  # in a line: the rotation about it is unknown
    cam_from_base = np.linalg.inv(head_cam.kabsch([s["p_cam"] for s in g], tcp))
    z = float(np.median(tcp[:, 2]))  # the package top the touches were on
    rz = down_reach_rz(kin, lo, hi)
    rz = rz[np.abs(rz[:, 1] - z) < tol_m]
    if not len(rz):
        return None
    r_lo, r_hi = rz[:, 0].min(), rz[:, 0].max()
    out = []
    for x in np.arange(-0.15, 0.55, step_m):
        for y in np.arange(-0.50, 0.50, step_m):
            r = math.hypot(x - head_cam.PAN_AXIS_X_M, y)
            # pan turns +y to -y (FK: +pan swings the tip clockwise from above)
            pan = -math.atan2(y, x - head_cam.PAN_AXIS_X_M)
            if not (r_lo <= r <= r_hi and lo[0] <= pan <= hi[0]):
                continue
            if np.min(np.abs(rz[:, 0] - r)) > tol_m:
                continue
            pc = cam_from_base @ np.array([x, y, z, 1.0])
            if pc[2] < 0.1:
                continue
            u, v = intr.fx * pc[0] / pc[2] + intr.ppx, intr.fy * pc[1] / pc[2] + intr.ppy
            if 40 <= u < wh[0] - 40 and 40 <= v < wh[1] - 40:
                out.append([float(u), float(v)])
    return out or None


def take_look(a, cam, head):
    """--touch, first B: where the camera puts the package's magnet target, arm out of the way.
    -> (message, pending or None)."""
    pts, pxs = [], []
    for _ in range(10):
        color, depth = cam.grab()
        m = cam.markers(color, depth)[0].get(a.marker_id)
        if m and m["target_pt"] is not None:
            pts.append(m["target_pt"])
            pxs.append(m["px"] + [m["size_px"]])
    if len(pts) < 6:
        return f"marker {a.marker_id} seen in {len(pts)}/10 frames: move the arm out of the way", None
    pts = np.array(pts)
    spread = float(np.linalg.norm(pts.std(0)) * 1000)
    if spread > 4.0:
        return f"target jitters {spread:.1f} mm: not taken", None
    px = np.median(pxs, 0)
    return ("LOOKED. Now drive the magnet onto the circle centre, touch lightly, press B "
            "(BACK cancels). Don't move the package or the head."), {
        "p_cam": [float(v) for v in np.median(pts, 0)], "head_deg": [head["head_pan"], head["head_tilt"]],
        "px": [float(px[0]), float(px[1])], "size_px": float(px[2])}


def take_touch(bus, samples, arm_held, lock, pending):
    """--touch, second B: the magnet is on the circle; the arm's joints finish the sample.
    -> (message, done)."""
    if not arm_held:
        return "hold the arm first (f)", False
    with lock:
        pos = bus.sync_read("Present_Position", num_retry=2)
    moved = max(abs(pos["head_pan"] - pending["head_deg"][0]), abs(pos["head_tilt"] - pending["head_deg"][1]))
    if moved > 1.0:
        return f"the head moved {moved:.1f} deg since the look: BACK, then look again", False
    samples.append({"arm_deg": [pos[j] for j in ARM], "head_deg": pending["head_deg"],
                    "p_cam": pending["p_cam"], "px": pending["px"], "size_px": pending["size_px"],
                    "kind": "touch", "t": time.time()})
    return f"sample {len(samples)} taken. Lift the magnet off, move the package to the next box", True


def take_sample(a, cam, bus, samples, arm_held, lock=contextlib.nullcontext()):
    if not arm_held:
        return "hold the arm first (f): a free arm drifts while the frames are taken"
    pts, pnps, pxs = [], [], []
    for _ in range(10):
        color, depth = cam.grab()
        m = cam.markers(color, depth)[0].get(a.marker_id)
        if m:
            pts.append(cam.point(m))
            pxs.append(m["px"] + [m["size_px"]])
            if m["pnp_pt"]:
                pnps.append(m["pnp_pt"])
    if len(pts) < 6:
        return f"marker {a.marker_id} seen in {len(pts)}/10 frames: not taken"
    pts = np.array(pts)
    spread = float(np.linalg.norm(pts.std(0)) * 1000)
    if spread > 4.0:
        return f"point jitters {spread:.1f} mm: not taken (depth edge? tilt the marker to the camera)"
    with lock:
        pos = bus.sync_read("Present_Position", num_retry=2)
    samples.append({"arm_deg": [pos[j] for j in ARM], "head_deg": [pos["head_pan"], pos["head_tilt"]],
                    "p_cam": [float(v) for v in np.median(pts, 0)],
                    "pnp_cam": [float(v) for v in np.median(pnps, 0)] if pnps else None,
                    "px": [float(v) for v in np.median(pxs, 0)[:2]],
                    "size_px": float(np.median(pxs, 0)[2]),
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
    # Touch samples all have the tool pointing down, so the magnet's length and the camera's
    # height trade off exactly: fit it only if some samples had the tool at other angles. Picks
    # approach the same way with the same length, so the error cancels there.
    all_touch = all(s.get("kind") == "touch" for s in samples)
    if all_touch and not a.fixed_magnet:
        print(f"touch samples only: magnet length held at {a.magnet_m * 1000:.1f} mm (--magnet-m)")
    fit_off = not a.no_offsets and len(samples) >= 15
    if not fit_off and not a.no_offsets:
        print(f"{len(samples)} samples: too few to fit arm joint offsets as well (15+); skipping them")
    res = head_cam.fit(samples, kin, pan_axis=pan_axis, tilt_fwd=a.tilt_fwd, magnet_m=a.magnet_m,
                       fit_magnet=not (a.fixed_magnet or all_touch), fit_offsets=fit_off)
    hc, err = res[0], res[1]
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
    if fit_off:
        # the fit's offsets sit on top of whatever `kin` already applied (a previous solve)
        delta = res[2]
        total = {j: float(math.degrees(kin.off[i]) + delta.get(j, 0.0)) for i, j in enumerate(ARM)}
        print("arm joint offsets (deg, added to the robot's angles):",
              {j: round(float(v), 1) for j, v in total.items() if j in delta})
        if so101_kin.OFFSETS_FILE.is_file():
            so101_kin.OFFSETS_FILE.with_suffix(f".backup-{time.strftime('%Y%m%d-%H%M%S')}.json").write_text(
                so101_kin.OFFSETS_FILE.read_text())
        so101_kin.OFFSETS_FILE.write_text(json.dumps(
            {"offsets_deg": total, "from": "handeye solve", "n": len(samples), "saved": time.ctime()}, indent=2))
        print("saved", so101_kin.OFFSETS_FILE, "(teleop, IK and check use it from now on)")
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
    c.add_argument("--marker-id", type=int, default=None,
                   help="the marker to track: taped on the magnet (default 0), or with --touch the "
                        "package's top-face marker (default 10)")
    c.add_argument("--touch", action="store_true",
                   help="no marker on the magnet: B once to look at a package's marker (arm clear), "
                        "then touch the magnet onto its circle and B again")
    c.add_argument("--fresh", action="store_true", help="discard the saved samples")
    c.add_argument("--pad", action="store_true", help="drive the arm and head with the XInput pad")
    s = sub.add_parser("solve")
    s.add_argument("--pan-axis", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="ruler guess, metres, from the shoulder_pan axis at the mounting surface "
                        "(x forward, y left, z up); only seeds the fit")
    s.add_argument("--tilt-fwd", type=float, default=0.02, help="ruler guess: tilt axis ahead of pan axis, m")
    s.add_argument("--magnet-m", type=float, default=head_cam.MAGNET_FACE_M,
                   help="flange to marker surface along the tool axis, m (seed, or fixed with --fixed-magnet)")
    s.add_argument("--fixed-magnet", action="store_true", help="don't fit the magnet length")
    s.add_argument("--drop", type=int, nargs="*", default=[], help="sample numbers to leave out")
    s.add_argument("--no-offsets", action="store_true",
                   help="don't fit the arm's joint offsets (lift/elbow/wrist zero vs the URDF's)")
    sub.add_parser("check")
    a = ap.parse_args(argv)
    {"capture": cmd_capture, "solve": cmd_solve, "check": cmd_check}[a.cmd](a)


if __name__ == "__main__":
    main()
