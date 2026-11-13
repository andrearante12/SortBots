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
import functools
import http.server
import json
import math
import os
import queue
import select
import sys
import tempfile
import threading
import termios
import time
import tty
import urllib.parse
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
        self.cfg = cfg
        self._last_frame, self._stale_since = None, None
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

    STALL_S = 1.5  # colour frame number unchanged this long: the stream has stalled

    def grab(self):
        """(colour, depth) of a NEW colour frame. A D435 colour stream can stall on USB while
        depth keeps coming (2026-11-11): the pipeline then hands back the same colour frame
        forever, the preview froze, and a LOOK read a stale package position. Same frame number
        for STALL_S -> reset the camera and carry on. The colour image is copied: the frame's
        own buffer is what got drawn over again and again."""
        while True:
            fs = self.align.process(self.pipe.wait_for_frames())
            cf = fs.get_color_frame()
            n, now = cf.get_frame_number(), time.monotonic()
            if n != self._last_frame:
                self._last_frame, self._stale_since = n, None
                break
            if self._stale_since is None:
                self._stale_since = now
            elif now - self._stale_since > self.STALL_S:
                self._recover()
        color = np.array(cf.get_data(), copy=True)
        depth = np.asanyarray(fs.get_depth_frame().get_data()).astype(float) * self.depth_scale
        return color, depth

    def _recover(self):
        print(f"\r\x1b[KD435 colour stream stalled: resetting the camera (~6 s)\n", flush=True)
        try:
            dev = self.pipe.get_active_profile().get_device()
            self.pipe.stop()
            dev.hardware_reset()
        except RuntimeError:
            pass
        time.sleep(6)
        self._start(self.cfg)
        self._last_frame, self._stale_since = None, None

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
        if v1 - v0 < 4 or u1 - u0 < 4:
            # the box lies (mostly) off the image: a marker near the edge puts the halfway point
            # past it, and an empty slice crashed capture mid-set (2026-11-12, 21 samples in)
            return None
        z = depth[v0:v1:2, u0:u1:2]
        vv, uu = np.mgrid[v0:v1:2, u0:u1:2]
        ok = (z > 0.1) & (z < 3.0)
        if ok.sum() < 50:
            return None
        z, uu, vv = z[ok], uu[ok], vv[ok]
        # colour stream is distortion-free on the D435 (coeffs all 0), so plain pinhole
        pts = np.stack([(uu - self.intr.ppx) / self.intr.fx * z, (vv - self.intr.ppy) / self.intr.fy * z, z], 1)
        return head_cam.plane_fit(pts)[1]

    def marker(self, color, depth, marker_id):
        """One marker's entry, or None; a frame that fails to process counts as not seen."""
        try:
            return self.markers(color, depth)[0].get(marker_id)
        except Exception:
            return None

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


# Keys the page's buttons send (/cmd?k=...), merged into read_keys so a button is the same as
# its keyboard key. Single characters only, from this set: nothing else is accepted.
WEB_KEYS = queue.Queue()
WEB_ALLOWED = set(" fpguxadws!1234mkor")
# The page's touchpad: the latest hand velocity (-1..1 per axis) and when it came. The page
# resends it every 80 ms while a finger is down; JOG_FRESH_S without one and the arm stops, so a
# lifted finger, a closed tab or a dropped connection can't leave the arm driving.
JOG = {"v": {}, "t": 0.0}
JOG_AXES = ("x", "y", "z", "t", "w", "hp", "ht")   # hand x/y/z, tool tilt, wrist; head pan/tilt
ARM_JOG_AXES, HEAD_JOG_AXES = JOG_AXES[:5], JOG_AXES[5:]
HEAD_DPS = 15.0   # head speed while a camera button or a/d/w/s is held
JOG_FRESH_S = 0.25


def read_keys(timeout):
    keys = []
    if select.select([sys.stdin], [], [], timeout)[0]:
        keys = list(os.read(sys.stdin.fileno(), 64).decode(errors="ignore"))
    while not WEB_KEYS.empty():
        keys.append(WEB_KEYS.get_nowait())
    return keys


class _PreviewHandler(http.server.SimpleHTTPRequestHandler):
    """The preview directory, plus GET /cmd?k=KEY for the page's buttons."""

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        if u.path == "/jog":
            q = urllib.parse.parse_qs(u.query)
            try:
                v = {ax: max(-1.0, min(1.0, float(q.get(ax, ["0"])[0]))) for ax in JOG_AXES}
            except ValueError:
                self.send_error(400)
                return
            JOG["v"], JOG["t"] = v, time.monotonic()
            self.send_response(204)
            self.end_headers()
            return
        if u.path == "/cmd":
            k = urllib.parse.parse_qs(u.query, keep_blank_values=True).get("k", [""])[0]
            if len(k) == 1 and k in WEB_ALLOWED:
                WEB_KEYS.put(k)
                self.send_response(204)
                self.end_headers()
            else:
                self.send_error(400)
            return
        super().do_GET()

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, *args):
        pass


def start_preview_server(port=8765):
    """Serve the preview page (and its buttons) on 127.0.0.1 only. None if the port is taken,
    e.g. by an old `python3 -m http.server 8765`: then the page shows but buttons can't reach us."""
    try:
        srv = http.server.ThreadingHTTPServer(
            ("127.0.0.1", port), functools.partial(_PreviewHandler, directory=str(PREVIEW_DIR)))
    except OSError:
        return None
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


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
/* full-window camera, every control a small translucent overlay on it (2026-11-13) */
html,body{margin:0;height:100%;background:#000;color:#eee;font:13px/1.35 system-ui,sans-serif;overflow:hidden}
#img{position:fixed;inset:0}#img img{width:100%;height:100%;object-fit:contain;display:block}
#side{position:fixed;inset:0;pointer-events:none}#side>div{pointer-events:auto}
#top{position:absolute;top:58px;left:8px;max-width:44vw}
#top .note{background:rgba(74,59,0,.75);border-left:3px solid #f5c400;color:#ffe680;padding:5px 10px;
  border-radius:4px;font-size:13px}
#btn{position:absolute;top:58px;right:8px;display:flex;flex-direction:column;align-items:flex-end;max-width:40vw}
#btn .row{display:flex;flex-wrap:wrap;justify-content:flex-end;align-items:center}
#btn .lbl{color:rgba(255,255,255,.55);font-size:10px;margin:0 6px 4px 0}
button{font:600 12px system-ui,sans-serif;margin:0 0 5px 5px;padding:5px 9px;border:1px solid rgba(255,255,255,.22);
  border-radius:5px;background:rgba(45,74,115,.5);color:#fff;cursor:pointer;backdrop-filter:blur(3px);touch-action:none}
button:active{filter:brightness(1.5)}button.go{background:rgba(46,125,50,.55)}
button.stop{background:rgba(198,40,40,.75)}#btn button.stop{font-size:16px;padding:9px 18px}
button.head,button.hold{background:rgba(60,60,60,.5)}
#jog{position:absolute;left:8px;bottom:8px;background:rgba(0,0,0,.25);border-radius:8px;padding:6px}
#jog .lbl{color:rgba(255,255,255,.6);font-size:10px;margin:2px 0 4px}#jog .row{margin:2px 0}
#jog button{margin:0 4px 4px 0;padding:4px 8px;font-size:11px}
#pad{position:relative;width:150px;height:150px;border-radius:10px;background:rgba(29,38,51,.35);touch-action:none;
  border:1px solid rgba(120,150,190,.4)}#zpad{position:relative;width:40px;height:150px;border-radius:10px;
  background:rgba(29,38,51,.35);touch-action:none;border:1px solid rgba(120,150,190,.4)}
#knob,#zknob{position:absolute;width:24px;height:24px;border-radius:50%;background:rgba(74,144,226,.8);left:50%;top:50%;
  transform:translate(-50%,-50%);pointer-events:none}.ax{position:absolute;color:rgba(170,190,210,.7);font-size:10px;pointer-events:none}
#p{position:absolute;right:8px;bottom:8px;max-width:300px;max-height:40vh;overflow:auto;background:rgba(0,0,0,.45);
  padding:6px 9px;border-radius:6px;font-size:12px}
#p h1{font-size:14px;margin:0 0 4px}.step{color:#9ab;font-size:11px}
.do{background:rgba(29,45,29,.6);border-left:3px solid #5c5;padding:5px 8px;border-radius:3px}.do p{margin:0 0 3px}
.ready{border-color:#5f5}ul{list-style:none;padding:0;margin:5px 0}li{padding:1px 0}.ok{color:#6d6}.no{color:#e77}
table{border-collapse:collapse;width:100%;margin-top:6px;font-size:11px}td{padding:2px 4px;border-bottom:1px solid rgba(255,255,255,.1)}
tr.cur td{background:rgba(32,40,56,.7)}.go{font-size:13px;font-weight:600;color:#6f6;margin:4px 0}.msg{color:#fd6;margin-top:4px}
.st{color:#aaa;font-size:10px;margin-top:5px;word-break:break-word}
</style>
<div id=img><img id=v alt="camera"></div><div id=side><div id=top></div><div id=btn></div>
<div id=jog style="display:none"><div class=lbl>hand: drag / arrows / PgUp PgDn (hold)</div>
<div style="display:flex;gap:10px;align-items:flex-start">
<div id=pad><div class=ax style="top:4px;left:50%;transform:translateX(-50%)">away</div>
<div class=ax style="bottom:4px;left:50%;transform:translateX(-50%)">toward</div>
<div class=ax style="left:4px;top:50%;transform:translateY(-50%)">left</div>
<div class=ax style="right:4px;top:50%;transform:translateY(-50%)">right</div><div id=knob></div></div>
<div id=zpad><div class=ax style="top:4px;left:50%;transform:translateX(-50%)">up</div>
<div class=ax style="bottom:4px;left:50%;transform:translateX(-50%)">down</div><div id=zknob></div></div></div>
<div class=row style="margin-top:8px"><button class=hold data-j="t" data-s="1">tilt up</button><button class=hold data-j="t" data-s="-1">tilt down</button>
<button class=hold data-j="w" data-s="1">wrist up</button><button class=hold data-j="w" data-s="-1">wrist down</button></div>
<div class=lbl>camera (hold; a/d/w/s)</div><div class=row><button class=hold data-j="hp" data-s="1">pan +</button>
<button class=hold data-j="hp" data-s="-1">pan -</button><button class=hold data-j="ht" data-s="1">tilt +</button>
<button class=hold data-j="ht" data-s="-1">tilt -</button></div>
<div class=row><span class=lbl>speed </span><button data-k="1">L1</button><button data-k="2">L2</button><button data-k="3">L3</button><button data-k="4">L4</button></div></div>
<div id=p>waiting...</div></div>
<script>
function esc(t){return String(t).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
var FLAG_COL={FREE:'#777',HELD:'#3a7bd5',PLANNED:'#c9a227',MOVING:'#e8742a',DESCENDING:'#e8742a',
  PRESSING:'#e8742a',LIFTING:'#e8742a',HOVERING:'#2e9e4f',HOLDING:'#8e44ad',ABORTED:'#d33',IDLE:'#777'};
var lastBtn='';
function send(k){fetch('/cmd?k='+encodeURIComponent(k)).catch(function(){})}
function buttons(g){
  var b=JSON.stringify(g.buttons||[]);if(b===lastBtn)return;lastBtn=b;
  document.getElementById('btn').innerHTML=(g.buttons||[]).map(function(row){
    return '<div class=row>'+(row.label?'<div class=lbl>'+esc(row.label)+'</div>':'')+row.items.map(function(it){
      return '<button class="'+(it[2]||'')+'" data-k="'+it[1]+'">'+esc(it[0])+'</button>'}).join('')+'</div>'}).join('')}
document.addEventListener('click',function(e){var t=e.target.closest('button[data-k]');if(t)send(t.dataset.k)});
var J={x:0,y:0,z:0,t:0,w:0,hp:0,ht:0},jt=null;
function jsend(){fetch('/jog?x='+J.x.toFixed(2)+'&y='+J.y.toFixed(2)+'&z='+J.z.toFixed(2)+'&t='+J.t+'&w='+J.w+'&hp='+J.hp+'&ht='+J.ht).catch(function(){})}
function jlive(){return J.x||J.y||J.z||J.t||J.w||J.hp||J.ht}
function jupdate(){if(jlive()){if(!jt){jsend();jt=setInterval(jsend,80)}}else{if(jt){clearInterval(jt);jt=null}jsend()}}
function dragger(el,knob,f){
  function at(e){var r=el.getBoundingClientRect(),dx=(e.clientX-r.left)/r.width*2-1,dy=(e.clientY-r.top)/r.height*2-1;
    dx=Math.max(-1,Math.min(1,dx));dy=Math.max(-1,Math.min(1,dy));f(dx,dy);
    knob.style.left=((dx+1)/2*100)+'%';knob.style.top=((dy+1)/2*100)+'%';jupdate()}
  el.addEventListener('pointerdown',function(e){el.setPointerCapture(e.pointerId);at(e)});
  el.addEventListener('pointermove',function(e){if(el.hasPointerCapture(e.pointerId))at(e)});
  function end(){f(0,0);knob.style.left='50%';knob.style.top='50%';jupdate()}
  el.addEventListener('pointerup',end);el.addEventListener('pointercancel',end);el.addEventListener('lostpointercapture',end)}
window.addEventListener('load',function(){
  dragger(document.getElementById('pad'),document.getElementById('knob'),function(dx,dy){J.x=Math.abs(dx)<.08?0:dx;J.y=Math.abs(dy)<.08?0:-dy});
  dragger(document.getElementById('zpad'),document.getElementById('zknob'),function(dx,dy){J.z=Math.abs(dy)<.08?0:-dy});
  document.querySelectorAll('#jog button.hold').forEach(function(b){
    b.addEventListener('pointerdown',function(e){b.setPointerCapture(e.pointerId);J[b.dataset.j]=+b.dataset.s;jupdate()});
    ['pointerup','pointercancel','lostpointercapture'].forEach(function(n){b.addEventListener(n,function(){J[b.dataset.j]=0;jupdate()})})})});
window.addEventListener('blur',function(){J={x:0,y:0,z:0,t:0,w:0,hp:0,ht:0};jupdate()});
// keyboard: the terminal's keys, arrows/PageUp/PageDown drive like the touchpad (hold to move)
var KEYJ={ArrowUp:['y',1],ArrowDown:['y',-1],ArrowLeft:['x',-1],ArrowRight:['x',1],PageUp:['z',1],PageDown:['z',-1],
  a:['hp',1],d:['hp',-1],w:['ht',1],s:['ht',-1]};
var KEYCMD={' ':' ',f:'f',p:'p',g:'g',u:'u',k:'k',o:'o',r:'r',m:'m','1':'1','2':'2','3':'3','4':'4',x:'x',Escape:'!'};
function moving(){var g=window.S&&window.S.guide;return g&&['MOVING','DESCENDING','PRESSING','LIFTING'].indexOf(g.flag)>=0}
document.addEventListener('keydown',function(e){
  if(e.ctrlKey||e.metaKey||e.altKey)return;
  if(moving()){send('!');e.preventDefault();return}          // any key aborts a planned move
  var kj=KEYJ[e.key]||KEYJ[(e.key||'').toLowerCase()];
  if(kj){J[kj[0]]=kj[1];jupdate();e.preventDefault();return}
  if(e.repeat)return;
  var k=KEYCMD[e.key]||KEYCMD[(e.key||'').toLowerCase()];
  if(k){send(k);e.preventDefault()}});
document.addEventListener('keyup',function(e){var kj=KEYJ[e.key]||KEYJ[(e.key||'').toLowerCase()];if(kj){J[kj[0]]=0;jupdate()}});
function render(S){
  var g=S.guide,h=(g.flag?'<div style="display:inline-block;padding:2px 7px;border-radius:4px;font-weight:700;'+
    'font-size:11px;margin:0 4px 4px 0;color:#fff;background:'+(FLAG_COL[g.flag]||'#777')+'">ARM: '+
    esc(g.flag)+'</div>':'')+(g.magnet?'<div style="display:inline-block;padding:2px 7px;border-radius:4px;'+
    'font-weight:700;font-size:11px;margin:0 0 4px;color:#fff;background:'+(g.magnet==='ON'?'#c62828':'#555')+
    '">MAGNET: '+esc(g.magnet)+'</div>':'')+(g.stages?'<div class=step>step '+g.stage+' of '+g.stages+'</div>':'')+'<h1>'+esc(g.title)+'</h1>';
  h+='<div class="do'+(g.ready?' ready':'')+'">'+g.lines.map(l=>'<p>'+esc(l)+'</p>').join('')+'</div>';
  if(g.checks.length)h+='<ul>'+g.checks.map(c=>'<li class='+(c[1]?'ok':'no')+'>'+(c[1]?'&#10003; ':'&#10007; ')+esc(c[0])+'</li>').join('')+'</ul>';
  if(g.ready&&!g.buttons)h+='<div class=go>Ready: press B</div>';
  buttons(g);document.getElementById('jog').style.display=g.jog?'block':'none';
  // the latest message (a refusal, a warning) goes on top, above the buttons: below the drive
  // panel it was out of sight and a refused GO looked like "nothing happens" (2026-11-13)
  var t=document.getElementById('top'),mm=g.msg||S.msg,m=g.buttons&&mm?'<div class=note>'+esc(mm)+'</div>':'';
  if(t.innerHTML!==m)t.innerHTML=m;
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
    if guide and guide.get("flag"):
        # the ARM state across the top of the picture itself (BGR), same colours as the page badge
        col = {"FREE": (119, 119, 119), "HELD": (213, 123, 58), "PLANNED": (39, 162, 201),
               "MOVING": (42, 116, 232), "DESCENDING": (42, 116, 232), "PRESSING": (42, 116, 232),
               "LIFTING": (42, 116, 232), "HOVERING": (79, 158, 46), "HOLDING": (173, 68, 142),
               "ABORTED": (51, 51, 221)}.get(guide["flag"], (119, 119, 119))
        cv2.rectangle(color, (0, 0), (color.shape[1], 52), col, -1)
        cv2.putText(color, f"ARM: {guide['flag']}", (14, 38), cv2.FONT_HERSHEY_SIMPLEX, 1.15, (255, 255, 255), 3)
        cv2.putText(color, guide.get("title", "")[:44], (380, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        if guide.get("magnet"):
            on = guide["magnet"] == "ON"
            x0 = color.shape[1] - 300
            cv2.rectangle(color, (x0, 6), (color.shape[1] - 8, 46), (40, 40, 198) if on else (85, 85, 85), -1)
            cv2.putText(color, f"MAGNET: {guide['magnet']}", (x0 + 14, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.95,
                        (255, 255, 255), 2)
    if guide and guide["box"]:
        x0, y0, x1, y1 = guide["box"]
        cv2.rectangle(color, (x0 + 3, y0 + 3), (x1 - 3, y1 - 3), (0, 255, 0) if guide["ready"] else (0, 200, 0),
                      6 if guide["ready"] else 3)
    if not (guide and guide.get("flag")):
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
            if start_preview_server():
                print("\r\x1b[Klive page with buttons:  http://127.0.0.1:8765/handeye_view.html\n")
            else:
                print(f"\r\x1b[Kport 8765 is taken (an old http.server?): the page works but its buttons "
                      f"won't. Stop that server, or open {PREVIEW_HTML}\n")
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
            try:
                ms, corners, ids = cam.markers(color, depth)
            except Exception as e:  # one bad frame must not end a calibration session
                ms, corners, ids = {}, [], None
                say(f"frame skipped: {type(e).__name__}: {e}", 3.0)
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
                    msg_, pending = take_look(a, cam, bus, lock)
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


HEAD_MOVED_DEG = 1.5  # head shift between look and touch that voids the look (servo jitter ~0.3)


def take_look(a, cam, bus, lock):
    """--touch, first B: where the camera puts the package's magnet target, arm out of the way.
    -> (message, pending or None)."""
    pts, pxs = [], []
    for _ in range(10):
        color, depth = cam.grab()
        m = cam.marker(color, depth, a.marker_id)
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
    # The head's MEASURED angle, not its goal: a held STS3215 sits ~1 deg off its goal, and the
    # touch compares against a measurement (the goal here failed every touch by 1.1 deg,
    # 2026-11-11). It is also the angle the camera really was at.
    with lock:
        hp = bus.sync_read("Present_Position", list(headmod.HEAD), num_retry=2)
    return ("LOOKED. Now drive the magnet onto the circle centre, touch lightly, press B "
            "(BACK cancels). Don't move the package or the head."), {
        "p_cam": [float(v) for v in np.median(pts, 0)], "head_deg": [hp["head_pan"], hp["head_tilt"]],
        "px": [float(px[0]), float(px[1])], "size_px": float(px[2])}


def take_touch(bus, samples, arm_held, lock, pending):
    """--touch, second B: the magnet is on the circle; the arm's joints finish the sample.
    -> (message, done)."""
    if not arm_held:
        return "hold the arm first (f)", False
    with lock:
        pos = bus.sync_read("Present_Position", num_retry=2)
    moved = max(abs(pos["head_pan"] - pending["head_deg"][0]), abs(pos["head_tilt"] - pending["head_deg"][1]))
    if moved > HEAD_MOVED_DEG:
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
        m = cam.marker(color, depth, a.marker_id)
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


# A fit worse than this doesn't get to rewrite the arm model: the offsets reach teleop and IK.
# A 28.6 mm fit (two bad touches) did, 2026-11-11.
OFFSETS_MAX_RMS_MM = 8.0


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
                       fit_magnet=not (a.fixed_magnet or all_touch), fit_offsets=fit_off,
                       touch_z=a.touch_z)
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
    rms_ok = float(np.sqrt(np.mean(err ** 2))) <= OFFSETS_MAX_RMS_MM
    if fit_off and not (rms_ok or a.save_offsets):
        print(f"arm joint offsets NOT saved: RMS over {OFFSETS_MAX_RMS_MM:.0f} mm (they'd change teleop and "
              "IK on a doubtful fit). Fitted:", {j: round(float(v), 1) for j, v in res[2].items()},
              "| --save-offsets to save anyway")
    if fit_off and (rms_ok or a.save_offsets):
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
    if fit_off and not (rms_ok or a.save_offsets):
        # The camera model was fitted WITH those offsets; saved without them it describes an arm
        # that isn't the one in use (a 50 mm fit's camera got saved alone this way, 2026-11-12).
        print("camera model NOT saved either: it only fits together with those offsets")
        return
    hc.save(out, {"rms_mm": rms, "n": len(samples), "marker_m": a.marker_m, "saved": time.ctime()})
    print("saved", out)
    if rms > 10:
        print("RMS over 1 cm: check the marker size (--marker-m), that the marker is flat on the "
              "magnet face, and that the arm calibration is recent")


def cmd_check(a):
    """Verify a saved calibration on a real package, two steps as in capture --touch:
    space LOCKS the package target (arm clear, 10 frames, stored in base_link), then the magnet
    is moved onto the circle and `d` is the live distance to the locked target. Touching hides
    the marker, so a live-only readout went blank exactly when it mattered. --head PAN TILT
    holds the head at a calibrated angle (held-out angles fitted ~2 cm worse, 2026-11-12); the
    arm stays free (torque off) for placing by hand."""
    if a.marker_id is None:
        a.marker_id = 10
    hc = head_cam.HeadCam.load(calib_dir() / f"{a.head_id}_camera.json")
    kin = SO101Kin()
    cam = Camera(marker_m=a.marker_m)
    bus, head_cal = open_bus(a.port, a.arm_id, a.head_id)
    if a.head:
        lim = headmod.limits(head_cal)
        goal = {"head_pan": min(lim["head_pan"][1], max(lim["head_pan"][0], a.head[0])),
                "head_tilt": min(lim["head_tilt"][1], max(lim["head_tilt"][0], a.head[1]))}
        set_hold(bus, list(headmod.HEAD), True)
        start = bus.sync_read("Present_Position", list(headmod.HEAD), num_retry=2)
        for k in range(1, 41):  # ~2 s ramp: no snap of the mast-mounted camera
            bus.sync_write("Goal_Position", {m: start[m] + (goal[m] - start[m]) * k / 40 for m in goal})
            time.sleep(0.05)
        time.sleep(0.5)
    locked, results = None, []
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(f"arm torque off. space = lock marker {a.marker_id}'s target (arm clear), r = record the "
          "current d, c = clear the lock, x = quit\r")
    try:
        tty.setcbreak(fd)
        run = True
        while run:
            color, depth = cam.grab()
            try:
                ms, corners, ids = cam.markers(color, depth)
            except Exception:
                ms, corners, ids = {}, [], None
            pos = bus.sync_read("Present_Position", num_retry=2)
            tcp = kin.tool_point(np.radians([pos[j] for j in ARM]), hc.magnet_m)
            T = hc.matrix(pos["head_pan"], pos["head_tilt"])
            msg = ""
            for k in read_keys(0.0):
                if k == " ":
                    pts = []
                    for _ in range(10):
                        c2, d2 = cam.grab()
                        m = cam.marker(c2, d2, a.marker_id)
                        if m and m["target_pt"] is not None:
                            pts.append(m["target_pt"])
                    if len(pts) >= 6:
                        hp = bus.sync_read("Present_Position", list(headmod.HEAD), num_retry=2)
                        Tl = hc.matrix(hp["head_pan"], hp["head_tilt"])
                        locked = (Tl @ np.append(np.median(pts, 0), 1.0))[:3]
                        msg = f"LOCKED target at {np.round(locked * 100, 1)} cm: now put the magnet on the circle"
                    else:
                        msg = f"marker {a.marker_id} seen in {len(pts)}/10 frames: move the arm clear"
                elif k == "r" and locked is not None:
                    results.append(float(np.linalg.norm(tcp - locked) * 1000))
                    msg = f"recorded d = {results[-1]:.1f} mm ({len(results)} so far)"
                elif k == "c":
                    locked, msg = None, "lock cleared"
                elif k in ("x", "\x1b"):
                    run = False
            head_txt = f"head {pos['head_pan']:+.1f}/{pos['head_tilt']:+.1f}"
            if locked is not None:
                dv = (tcp - locked) * 100
                line = (f"d {np.linalg.norm(dv):5.1f} cm (dx {dv[0]:+.1f} dy {dv[1]:+.1f} dz {dv[2]:+.1f}) "
                        f"| magnet {np.round(tcp * 100, 1)} | {head_txt}")
            else:
                seen = ms.get(a.marker_id)
                line = (f"marker {a.marker_id} " + (f"{seen['size_px']:.0f}px" if seen else "--")
                        + f" | arm clear, then space to lock | {head_txt}")
            print("\r" + line + (" | " + msg if msg else "") + "\x1b[K", end="", flush=True)
            if not a.no_view:
                lines = [line] + ([msg] if msg else []) + (
                    [f"recorded: {', '.join(f'{v:.0f}' for v in results)} mm"] if results else [])
                draw(cam, color, corners, ids, line[:90],
                     {"title": "Check: d to the locked target" if locked is not None else "Check: lock a target",
                      "lines": lines, "checks": [], "box": None, "reach": None, "progress": [],
                      "stage": 0, "stages": 0, "done": False, "ready": locked is not None})
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        bus.disconnect(disable_torque=True)
        cam.close()
        if results:
            print(f"\nrecorded d: {', '.join(f'{v:.1f}' for v in results)} mm; "
                  f"median {np.median(results):.1f} mm, worst {max(results):.1f} mm")
        print()


HOME_HEAD = (90.4, 60.7)   # the head angle most of the 2026-11-12 calibration was taken at
# pick's default view, set from the working setup (2026-11-13). It is OUTSIDE the range the
# 2026-11-12 calibration covers (pan ~70..110): a hover there was 1.3 cm off. Add touch samples at
# this angle and re-solve before relying on it; the page warns while it's outside.
PICK_HEAD = (32.0, 70.0)
MAGNET_REARM_S, MAGNET_ON_S = 10.0, 30   # as arm_xpad: re-arm well inside the firmware's 30 s cap
HIGH_M, HOVER_M = 0.12, 0.05
# Stage 2: the calibration's height is good to ~1 cm, so the last centimetre is a gentle press with
# lift/elbow/wrist torque cut to PRESS_TORQUE: the goal goes PRESS_BELOW_M under the computed
# surface and the servos give where the package stops them, instead of driving into it.
CONTACT_ABOVE_M, PRESS_BELOW_M = 0.01, 0.01
PRESS_TORQUE, FULL_TORQUE = 350, 1000   # Torque_Limit, 0..1000 (= % x 10)
PRESS_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
GRIP_WAIT_S = 0.5
MOVING_FLAGS = ("MOVING", "DESCENDING", "PRESSING", "LIFTING")
CLEAR_M = 0.08   # GO's first move is joint-interpolated: it starts only this far above the package   # above the target: clearance for the joint move, and stage 1's stop
PICK_DPS = 20.0            # joint speed cap for every pick motion


def hold_head_at(bus, head_cal, pan, tilt, ramp_s=2.0):
    """Torque the head on where it is, then ramp it to (pan, tilt): no snap of the mast camera."""
    lim = headmod.limits(head_cal)
    goal = {"head_pan": min(lim["head_pan"][1], max(lim["head_pan"][0], pan)),
            "head_tilt": min(lim["head_tilt"][1], max(lim["head_tilt"][0], tilt))}
    set_hold(bus, list(headmod.HEAD), True)
    start = bus.sync_read("Present_Position", list(headmod.HEAD), num_retry=2)
    n = max(1, int(ramp_s / 0.05))
    for k in range(1, n + 1):
        bus.sync_write("Goal_Position", {m: start[m] + (goal[m] - start[m]) * k / n for m in goal})
        time.sleep(0.05)
    time.sleep(0.5)


class MagnetLink:
    """The magnet as the BOARD reports it, not as last commanded. The page once showed MAG ON
    while the coil was off (2026-11-13): replies weren't checked, and the board's own lines (its
    30 s cap expiring: "ok timeout off"; a reset, e.g. the coil's 0.36 A sagging a shared 5 V:
    "ok ready") sat unread and were taken as the reply to the next command. So: drain those
    first, check every reply, and poll `status` (the caller does, about once a second).
    `say(text, secs)` reports to the page. A board that stops answering is reconnected once;
    opening the port resets it, so afterwards the magnet is OFF for certain."""

    def __init__(self, magnet, say, reconnect=None):
        self.magnet, self.say = magnet, say
        self.on, self.fault = False, False
        self.t0 = self.armed_at = self.polled = 0.0
        self._reconnect = reconnect or self._open

    @staticmethod
    def _open():
        from magnet import Magnet
        m = Magnet()
        m.off()
        return m

    def _call(self, fn):
        try:
            return fn()
        except BaseException as e:  # the driver signals "no answer" as SystemExit
            if isinstance(e, KeyboardInterrupt):
                raise
            self._lost(e)
            return None

    def _lost(self, e):
        self.on = False
        try:
            self.magnet.close()
        except BaseException:
            pass
        self.magnet = None
        try:
            self.magnet = self._reconnect()
            self.fault = False
            self.say(f"magnet board stopped answering ({e}); reconnected, which resets it: magnet is OFF", 1e9)
        except BaseException as e2:
            if isinstance(e2, KeyboardInterrupt):
                raise
            self.magnet, self.fault = None, True
            self.say(f"magnet board not answering ({e2}): treat the magnet as OFF", 1e9)

    def events(self):
        if not self.magnet:
            return

        def drain():
            while self.magnet.s.in_waiting:
                ln = self.magnet.s.readline().decode(errors="replace").strip()
                if ln == "ok ready":
                    if self.on:
                        self.say("magnet board RESET (a 5 V dip when the coil switched on?): magnet is OFF", 1e9)
                    self.on = False
                elif ln in ("ok timeout off", "ok heartbeat off"):
                    if self.on:
                        self.say(f"magnet board: '{ln}': magnet is OFF", 1e9)
                    self.on = False
        self._call(drain)

    def poll(self):
        """The coil's real state (True on), or None if the board can't say."""
        self.events()
        if not self.magnet:
            return None
        reply = self._call(self.magnet.status)
        if reply is None:
            return None
        self.polled = time.monotonic()
        if not reply.startswith("ok state"):
            self.say(f"magnet board: unexpected status reply '{reply}'", 1e9)
            return None
        actual = reply.split()[-1] == "on"
        if self.on and not actual:
            self.say("magnet board reports OFF (it reset or timed out)", 1e9)
        self.on = actual
        return actual

    def set(self, on, settle_s=0.15):
        """Command on/off and return what the board then reports (None if it can't say)."""
        if not self.magnet:
            return None
        self.events()
        reply = self._call(lambda: self.magnet.on(MAGNET_ON_S) if on else self.magnet.off())
        if reply is None:
            return None
        if reply != ("ok on" if on else "ok off"):
            self.say(f"magnet: board replied '{reply}'", 1e9)
        if on:
            self.armed_at = self.t0 = time.monotonic()
        time.sleep(settle_s)  # a reset from the coil's inrush shows up within this
        actual = self.poll()
        if actual is not None and actual != on:
            self.say(f"magnet did NOT turn {'on' if on else 'off'}: the board reports {'on' if actual else 'off'}", 1e9)
        elif actual is not None:
            self.say(f"magnet {'ON' if on else 'OFF'} (confirmed by the board)")
        return actual

    def rearm_if_due(self):
        """Re-send `on` well inside the firmware's 30 s cap while the magnet is on."""
        if self.magnet and self.on and time.monotonic() - self.armed_at > MAGNET_REARM_S:
            self.events()
            if self.magnet and self.on:
                reply = self._call(lambda: self.magnet.on(MAGNET_ON_S))
                self.armed_at = time.monotonic()
                if reply is not None and reply != "ok on":
                    self.say(f"magnet re-arm: board replied '{reply}'", 1e9)


class CamWorker:
    """The D435 on its own thread: grab, find markers, draw the page. Control loops then run at
    their own rate instead of waiting on frames (pad driving at camera rate lagged badly,
    2026-11-11). The control side sets `status`/`guide` for the page and reads markers back."""

    def __init__(self, cam, no_view=False):
        self.cam, self.no_view = cam, no_view
        self.status, self.guide = "", None
        self.ms, self.seq, self.err = {}, 0, None
        self.lock = threading.Lock()
        self._run = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        while self._run:
            try:
                color, depth = self.cam.grab()
                try:
                    ms, corners, ids = self.cam.markers(color, depth)
                except Exception:
                    ms, corners, ids = {}, [], None
                with self.lock:
                    self.ms, self.seq = ms, self.seq + 1
                    status, guide = self.status, self.guide
                if not self.no_view:
                    draw(self.cam, color, corners, ids, status[:90], guide)
            except Exception as e:  # keep the arm side running; it shows on the page
                self.err = e
                time.sleep(0.5)

    def show(self, status, guide):
        with self.lock:
            self.status, self.guide = status, guide

    def targets(self, marker_id, n=10, timeout=4.0):
        """target_pt of `marker_id` from the next n processed frames (fewer on timeout)."""
        out, last, t0 = [], self.seq, time.monotonic()
        while len(out) < n and time.monotonic() - t0 < timeout:
            with self.lock:
                seq, ms = self.seq, self.ms
            if seq != last:
                last = seq
                m = ms.get(marker_id)
                if m and m.get("target_pt") is not None:
                    out.append(m["target_pt"])
            time.sleep(0.01)
        return out

    def stop(self):
        self._run = False
        self._t.join(timeout=2.0)
        self.cam.close()


def cmd_pick(a):
    """Pick stage 1 plus hand driving, from the page's buttons and touchpad (or the keyboard):
    lock the package target, plan magnet-down poses with the calibrated arm, move HIGH_M above
    the target, descend straight down to HOVER_M and stop. No contact, no magnet. Between plans
    the touchpad drives the hand like the pad (arm_xpad.cartesian_step, same speed caps).

    keys: f arm free/held  space lock  p plan  g go  u back up to HIGH_M  a/d/w/s head +-2 deg
          m magnet on/off (on needs the arm held)   k descend and pick (from the hover)
          o one-button pick: hold, lock, plan, raise clear, go, descend and pick
          r record a flat pose (magnet set flat on the package by hand) to pick_flat_poses.json
          1-4 drive speed  any key during a planned move aborts it and holds  x quit (twice if held)"""
    import types
    import arm_xpad
    if a.marker_id is None:
        a.marker_id = 10
    hc = head_cam.HeadCam.load(calib_dir() / f"{a.head_id}_camera.json")
    kin = SO101Kin()
    if not np.any(kin.off):
        raise SystemExit("no arm joint offsets saved: run `handeye solve` first")
    path = calib_dir() / "handeye_samples.json"
    samples = json.loads(path.read_text()) if path.is_file() else []
    seeds = [np.radians(s["arm_deg"]) for s in samples]
    # the head range the calibration was fitted over: outside it the camera pose is extrapolated
    hd = np.array([s["head_deg"] for s in samples]) if samples else np.array([[a.head[0], a.head[1]]])
    cal_lo, cal_hi = hd.min(0) - 3.0, hd.max(0) + 3.0
    # The magnet's ESP32 (docs/magnet.md) first, before any thread or torque: everything else
    # works without it. Its driver raises SystemExit on no answer, and a fresh plug-in can miss the
    # first ping window, so: BaseException, and one retry (2026-11-13, it then crashed the run).
    magnet = None
    for attempt in (1, 2):
        try:
            from magnet import Magnet
            magnet = Magnet()
            magnet.off()  # known state: released
            break
        except BaseException as e:
            if isinstance(e, KeyboardInterrupt):
                raise
            print(f"magnet attempt {attempt}: {e}")
            magnet = None
    if magnet is None:
        print("no magnet: running without the magnet button")
    cam = Camera(marker_m=a.marker_m)
    bus, head_cal = open_bus(a.port, a.arm_id, a.head_id)
    lo, hi = arm_limits(bus)
    hold_head_at(bus, head_cal, *a.head)
    head_lim = headmod.limits(head_cal)
    head_goal = bus.sync_read("Present_Position", list(headmod.HEAD), num_retry=2)
    for j in PRESS_JOINTS:  # a crashed press can leave the RAM torque limit low: start at full
        bus.write("Torque_Limit", j, FULL_TORQUE, normalize=False)
    worker = CamWorker(cam, a.no_view)
    jog_cfg = types.SimpleNamespace(speed=0.10, pitch_dps=45.0, roll_dps=90.0, joint_dps=150.0, lock=["wrist_roll"])
    st = {
          "arm_held": False, "quit_armed": False, "locked": None, "plan": None, "state": "lock",
          "flag": "FREE", "level": 1, "jog_q": None, "jog_target": None, "jog_ok": True,
          "msg": f"arm free: move it clear of the package, then Lock target (marker {a.marker_id})",
          "until": time.monotonic() + 1e9}

    def say(text, secs=8.0):
        st["msg"], st["until"] = text, time.monotonic() + secs

    pan0, tilt0 = head_goal["head_pan"], head_goal["head_tilt"]
    st["head_inside"] = bool(cal_lo[0] <= pan0 <= cal_hi[0] and cal_lo[1] <= tilt0 <= cal_hi[1])
    if not st["head_inside"]:
        say(f"head {pan0:+.0f}/{tilt0:+.0f} is OUTSIDE the calibrated range (pan {cal_lo[0] + 3:.0f}.."
            f"{cal_hi[0] - 3:.0f}, tilt {cal_lo[1] + 3:.0f}..{cal_hi[1] - 3:.0f}): targets locked here may be "
            "cm off. Arm free: move it clear, then Lock target", 1e9)

    def arm_q():
        pos = bus.sync_read("Present_Position", ARM, num_retry=2)
        return np.radians([pos[j] for j in ARM])

    def buttons():
        if st["flag"] in MOVING_FLAGS:
            return [{"label": "arm is moving", "items": [["STOP  [any key]", "!", "stop"]]}]
        main, state = [], st["state"]
        if state == "lock":
            if mag.magnet:
                main.append(["PICK (one button)  [o]", "o", "go"])
            main.append(["Lock target  [space]", " ", "go"])
            if st["locked"] is not None:
                main.append(["Plan  [p]", "p", "go"])
        elif state == "planned":
            main += [["GO: hover 5 cm above  [g]", "g", "go"], ["Re-lock target  [space]", " ", ""]]
        elif state == "at_hover":
            if mag.magnet:
                main.append(["Descend and pick  [k]", "k", "go"])
            main.append(["Back up to 12 cm  [u]", "u", ""])
        elif state == "holding":
            main.append(["Release: turn magnet OFF  [m]", "m", "stop"])
        main.append(["Free arm (limp)  [f]" if st["arm_held"] else "Hold arm  [f]", "f", ""])
        rows = [{"label": "pick", "items": main}]
        if mag.magnet:
            # the label says what the button DOES; the state is the MAGNET pill, from the board.
            # "Magnet OFF" on the button of an energized magnet read as a status (2026-11-13).
            rows.append({"label": "magnet", "items": [["Turn magnet OFF  [m]", "m", "stop"] if mag.on
                                                      else ["Turn magnet ON  [m]", "m", "go"]]})
        elif mag.fault:
            rows.append({"label": "magnet: board not answering, treat as OFF", "items": []})
        n_flat = len(flat_poses)
        rows.append({"label": f"teach: {n_flat} flat pose{'' if n_flat == 1 else 's'} saved",
                     "items": [["Record flat pose  [r]", "r", ""]]})
        return rows + [
                {"label": "", "items": [["Quit: again to confirm  [x]" if st["quit_armed"] else "Quit  [x]", "x", ""]]}]

    mag = MagnetLink(magnet, say)
    # Taught "magnet flat on the package" poses (2026-11-13): the planned magnet-down pose left
    # the face tilted on the package, so it seated on one edge. The user sets the magnet flat by
    # hand (limp arm, or the drive panel) and records it; the model's tilt at each such pose is
    # the pitch error the planner has to take out.
    flat_path = calib_dir() / "pick_flat_poses.json"
    flat_poses = json.loads(flat_path.read_text()) if flat_path.is_file() else []

    def record_flat():
        q = arm_q()
        F = kin.flange(q)
        tool = kin.tool_point(q, hc.magnet_m)
        radial = tool[:2] / max(1e-6, np.linalg.norm(tool[:2]))
        z = F[:3, 2]  # flange +z: straight up when the magnet points straight down
        pitch = math.degrees(math.atan2(float(z[:2] @ radial), float(z[2])))
        side = math.degrees(math.atan2(float(z[0] * -radial[1] + z[1] * radial[0]), float(z[2])))
        rec = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "arm_deg": np.degrees(q).round(2).tolist(),
               "head_deg": [round(head_goal["head_pan"], 2), round(head_goal["head_tilt"], 2)],
               "tool_m": tool.round(4).tolist(), "reach_m": round(float(np.linalg.norm(tool[:2])), 4),
               "model_pitch_deg": round(pitch, 2), "model_side_deg": round(side, 2),
               "locked_m": None if st["locked"] is None else np.asarray(st["locked"]).round(4).tolist(),
               "arm_held": st["arm_held"]}
        flat_poses.append(rec)
        flat_path.write_text(json.dumps(flat_poses, indent=1))
        say(f"flat pose {len(flat_poses)} saved: reach {rec['reach_m'] * 100:.1f} cm, the model reads the "
            f"magnet {pitch:+.1f} deg off vertical (pitch, + = magnet face turned toward the base), {side:+.1f} deg sideways", 20.0)
        print(f"\nflat pose: {rec}")

    def magnet_set(on):
        if on and not st["arm_held"]:
            say("hold the arm before turning the magnet on")
            return False
        return mag.set(on)

    def publish(q):
        tcp = kin.tool_point(q, hc.magnet_m)
        txt = (f"{st['flag']} | magnet {np.round(tcp * 100, 1)} cm | head {head_goal['head_pan']:+.0f}/"
               f"{head_goal['head_tilt']:+.0f} | drive L{st['level'] + 1}" + ("" if st["jog_ok"] else " LIMIT")
               + (f" | MAG ON {time.monotonic() - mag.t0:.0f}s (board)" if mag.on
                  else (" | mag off (board)" if mag.magnet else " | no magnet")))
        if st["locked"] is not None:
            dv = (tcp - st["locked"]) * 100
            txt += f" | to target dx {dv[0]:+.1f} dy {dv[1]:+.1f} dz {dv[2]:+.1f}"
        msg = st["msg"] if time.monotonic() < st["until"] else ""
        title = {"lock": "Pick: lock the target", "planned": "Pick: plan ready",
                 "at_hover": "Pick: hovering 5 cm above the target",
                 "holding": "Pick: holding the package (drive it, then release)"}[st["state"]]
        worker.show(txt, {"title": title, "lines": [txt], "msg": msg, "checks": [], "box": None,
                          "reach": None, "progress": [], "stage": 0, "stages": 0, "done": False,
                          "ready": st["state"] == "planned", "flag": st["flag"], "buttons": buttons(),
                          "magnet": None if not mag.magnet else ("ON" if mag.on else "OFF"),
                          "jog": st["flag"] not in MOVING_FLAGS})
        return txt, msg

    def hold(on):
        set_hold(bus, ARM, on)
        st["arm_held"] = on
        if st["flag"] in ("FREE", "HELD", "ABORTED"):
            st["flag"] = "HELD" if on else "FREE"
        st["jog_q"] = None

    def move_to(q_goal, label, dps=PICK_DPS):
        """Joint-interpolated move at `dps`, 50 Hz, the page live from the camera thread. Any
        key or the STOP button aborts and holds. Returns True if it arrived."""
        start = dict(zip(ARM, np.degrees(arm_q())))
        goal = dict(zip(ARM, np.degrees(q_goal)))
        dur = max(0.5, max(abs(goal[j] - start[j]) for j in ARM) / dps)
        st["flag"] = label
        t0 = time.monotonic()
        while True:
            s_ = min(1.0, (time.monotonic() - t0) / dur)
            s_ = s_ * s_ * (3 - 2 * s_)
            bus.sync_write("Goal_Position", {j: start[j] + (goal[j] - start[j]) * s_ for j in ARM})
            if read_keys(0.0):
                set_hold(bus, ARM, True)
                st["flag"] = "ABORTED"
                say("ABORTED: holding where it is", 1e9)
                return False
            publish(arm_q())
            if s_ >= 1.0:
                time.sleep(0.4)
                return True
            time.sleep(0.02)

    def torque_limit(val):
        for j in PRESS_JOINTS:
            bus.write("Torque_Limit", j, val, normalize=False)

    def freeze():
        """Goal = where the arm IS, before torque comes back: after a press the goal sits under
        the package, and full torque at that goal would drive into it."""
        bus.sync_write("Goal_Position", bus.sync_read("Present_Position", ARM, num_retry=2))

    def straight(z_from, z_to, step):
        """(z, q) magnet-down poses from z_from to z_to above the locked target (excl. the start)."""
        qs, seed = [], [arm_q()]
        n = max(1, int(round(abs(z_to - z_from) / step)))
        for i in range(1, n + 1):
            z = z_from + (z_to - z_from) * i / n
            r = head_cam.ik_down(kin, st["locked"] + [0, 0, z], seed, lo, hi, hc.magnet_m, float(seed[0][4]))
            if r is None:
                return None
            qs.append((z, r[0]))
            seed = [r[0]]
        return qs

    def descend_and_pick():
        """Stage 2, from the hover: down to CONTACT_ABOVE_M, then a gentle press at reduced torque
        to --press-mm under the computed surface. The magnet goes on --magnet-on-mm above the
        surface, on the way down, so it is already pulling as it seats (2026-11-13: switching on
        only after the press left the package short of the face). The board must confirm ON
        before the arm goes any lower, and again after the press, or it doesn't lift. Then
        freeze, full torque, lift straight up to HIGH_M. Any abort holds; an abort never switches
        the magnet off, so a package that's caught stays caught."""
        h_on = a.magnet_on_mm / 1000.0
        down = straight(HOVER_M, CONTACT_ABOVE_M, 0.01)
        press = straight(CONTACT_ABOVE_M, -a.press_mm / 1000.0, 0.004)
        if down is None or press is None:
            say("the descent isn't solvable straight down from here", 1e9)
            return
        on = False

        def switch_on():
            if magnet_set(True) is not True:
                say("the magnet did NOT confirm ON: stopped here. Check the board, then Back up", 1e9)
                st["flag"] = "ABORTED"
                return False
            return True

        for z, q in down:  # on once ARRIVED at or below h_on, not before setting off for it
            if not move_to(q, "DESCENDING"):
                return
            if not on and z <= h_on + 1e-9:
                if not switch_on():
                    return
                on = True
        torque_limit(PRESS_TORQUE)
        try:
            for z, q in press:
                if not move_to(q, "PRESSING", dps=PICK_DPS * 0.3):
                    return
                if not on and z <= h_on + 1e-9:
                    if not switch_on():
                        return
                    on = True
            st["flag"] = "PRESSING"
            if not on and not switch_on():
                return
            time.sleep(GRIP_WAIT_S)
            if mag.poll() is not True:
                say("the magnet is not ON after the press (board): not lifting. Check the board, then Back up", 1e9)
                st["flag"] = "ABORTED"
                return
        finally:
            freeze()
            torque_limit(FULL_TORQUE)
        z_now = float(kin.tool_point(arm_q(), hc.magnet_m)[2] - st["locked"][2])
        up = straight(z_now, HIGH_M, 0.01)
        if up is None:
            say("magnet ON and holding, but the lift isn't solvable straight up: drive it up by hand", 1e9)
            st["state"], st["flag"] = "holding", "HOLDING"
            return
        for _, q in up:
            if not move_to(q, "LIFTING"):
                say("ABORTED during the lift: holding, magnet still ON", 1e9)
                st["state"] = "holding"
                return
        st["state"], st["flag"] = "holding", "HOLDING"
        say("HOLDING: magnet on, 12 cm up. Drive to the drop spot, then Release", 1e9)

    def aim_corrected(p, radial_m):
        """Shift a target along the horizontal line from the shoulder_pan axis through it:
        negative = toward the base. Magnet-down picks landed ~8 mm too far out (the magnet slides
        as it seats, 2026-11-13, at head 32/69); one radial number fixes it from any direction."""
        d = np.array([p[0] - head_cam.PAN_AXIS_X_M, p[1]])
        n = np.linalg.norm(d)
        if n < 1e-6 or radial_m == 0.0:
            return p
        return p + np.array([*(d / n * radial_m), 0.0])

    def do_lock():
        """Lock marker a.marker_id's target from 10 frames. True if locked."""
        pts = worker.targets(a.marker_id)
        if len(pts) >= 6 and np.linalg.norm(np.std(pts, 0)) < 0.004:
            hp = bus.sync_read("Present_Position", list(headmod.HEAD), num_retry=2)
            raw = (hc.matrix(hp["head_pan"], hp["head_tilt"]) @ np.append(np.median(pts, 0), 1.0))[:3]
            st["locked"] = aim_corrected(raw, a.radial_mm / 1000.0)
            st["state"], st["plan"] = "lock", None
            if st["flag"] in ("PLANNED", "ABORTED", "HOVERING"):
                st["flag"] = "HELD" if st["arm_held"] else "FREE"
            say(f"LOCKED target {np.round(st['locked'] * 100, 1)} cm (aim {a.radial_mm:+.0f} mm radial). Plan next")
            return True
        say(f"marker {a.marker_id} not steady in view ({len(pts)}/10): move the arm clear of it", 20.0)
        return False

    def do_plan():
        """Magnet-down poses 12 cm above, at the 5 cm hover and at contact, and the descent
        between. True if all solvable inside the joint limits."""
        q_now = arm_q()
        locked, roll = st["locked"], float(q_now[4])
        sols = {n: head_cam.ik_down(kin, locked + [0, 0, dz], seeds + [q_now], lo, hi, hc.magnet_m, roll)
                for n, dz in (("high", HIGH_M), ("hover", HOVER_M), ("contact", 0.0))}
        bad = [n for n, v_ in sols.items() if v_ is None]
        if bad:
            say(f"NOT reachable straight down: {', '.join(bad)}. Move the package closer to the arm", 20.0)
            return False
        steps, seed = [], [sols["high"][0]]
        for dz in np.arange(HIGH_M - 0.01, HOVER_M - 1e-6, -0.01):
            r = head_cam.ik_down(kin, locked + [0, 0, dz], seed, lo, hi, hc.magnet_m, roll)
            if r is None:
                break
            steps.append(r[0])
            seed = [r[0]]
        if len(steps) < round((HIGH_M - HOVER_M) / 0.01):
            say("descent path not solvable straight down: move the package", 20.0)
            return False
        st["plan"], st["state"], st["flag"] = {"high": sols["high"][0], "steps": steps}, "planned", "PLANNED"
        span = max(abs(np.degrees(sols["high"][0] - q_now)))
        say(f"PLAN ok: {span:.0f} deg joint move to 12 cm above, then down to 5 cm. GO when ready", 1e9)
        return True

    def do_go():
        """Joint move to 12 cm above the target, straight down to the 5 cm hover. True if there."""
        if not st["arm_held"]:
            hold(True)
        ok = move_to(st["plan"]["high"], "MOVING")
        for q in st["plan"]["steps"] if ok else []:
            ok = move_to(q, "DESCENDING")
            if not ok:
                break
        if ok:
            st["state"], st["flag"] = "at_hover", "HOVERING"
            err = (kin.tool_point(arm_q(), hc.magnet_m) - (st["locked"] + [0, 0, HOVER_M])) * 1000
            say(f"HOVERING 5 cm above the target (arm model: {np.linalg.norm(err):.0f} mm off its goal). "
                "Is the magnet centred over the circle?", 1e9)
        else:
            st["state"] = "lock"
        return ok

    def raise_clear():
        """Raise the hand straight up (its tilt kept) until the magnet is CLEAR_M + 1 cm above
        the locked target, 1 cm per step. True if it got there (or already was)."""
        need = st["locked"][2] + CLEAR_M + 0.01
        q = arm_q()
        for _ in range(40):
            z = kin.tool_point(q, hc.magnet_m)[2]
            if z >= need:
                return True
            goal = kin.pose(q) + np.array([0.0, 0.0, min(0.01, need - z + 0.002), 0.0])
            q_new, ok = kin.ik(goal, q, lo, hi)
            if not ok:  # near straight down the tilt solve folds: put the height first
                q_new, ok = kin.ik(goal, q, lo, hi, pitch_w=0.001)
            if not ok:
                say("could not raise the arm straight up from here: drive it up by hand, then PICK again", 20.0)
                return False
            if not move_to(q_new, "MOVING"):
                return False
            q = arm_q()
        return kin.tool_point(q, hc.magnet_m)[2] >= need

    def auto_pick():
        """One button: hold, lock, plan, raise clear if low, GO to the hover, descend and pick.
        Each step is the one its own button runs; the first refusal or abort stops the chain
        where it is, with the reason on the page."""
        if not mag.magnet:
            say("no magnet board: plug in the ESP32 and restart", 20.0)
            return
        if not st["arm_held"]:
            hold(True)
        if not do_lock() or not do_plan():
            return
        if not raise_clear():
            return
        if not do_go():
            return
        descend_and_pick()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print("keys:" + cmd_pick.__doc__.split("keys:")[1].replace("\n", "\r\n") + "\r")
    last_print = 0.0
    try:
        tty.setcbreak(fd)
        run = True
        while run:
            t_tick = time.monotonic()
            q_now = arm_q()
            tcp = kin.tool_point(q_now, hc.magnet_m)
            # ---- touchpad drive: only while fresh, never during a planned move
            v = JOG["v"] if time.monotonic() - JOG["t"] < JOG_FRESH_S else {}
            # head, held: HEAD_DPS while fresh, stops with the deadman like the hand
            if any(v.get(ax, 0.0) for ax in HEAD_JOG_AXES):
                dt_h = min(0.1, time.monotonic() - st.get("head_t", 0.0))
                st["head_t"] = time.monotonic()
                for mot, ax in (("head_pan", "hp"), ("head_tilt", "ht")):
                    lo_h, hi_h = head_lim[mot]
                    head_goal[mot] = min(hi_h, max(lo_h, head_goal[mot] + v.get(ax, 0.0) * HEAD_DPS * dt_h))
                bus.sync_write("Goal_Position", head_goal)
                pan, tilt = head_goal["head_pan"], head_goal["head_tilt"]
                inside = bool(cal_lo[0] <= pan <= cal_hi[0] and cal_lo[1] <= tilt <= cal_hi[1])
                if inside != st.get("head_inside"):
                    st["head_inside"] = inside
                    say(f"head {pan:+.0f}/{tilt:+.0f} " + ("inside the calibrated range" if inside else
                        f"is OUTSIDE the calibrated range (pan {cal_lo[0] + 3:.0f}..{cal_hi[0] - 3:.0f}, tilt "
                        f"{cal_lo[1] + 3:.0f}..{cal_hi[1] - 3:.0f}): targets locked here may be cm off"),
                        8.0 if inside else 1e9)
            else:
                st["head_t"] = time.monotonic()
            if any(v.get(ax, 0.0) for ax in ARM_JOG_AXES):
                if not st["arm_held"]:
                    hold(True)
                    say("arm held for driving")
                if st["jog_q"] is None:  # start from where the arm is: no snap
                    st["jog_q"] = np.clip(q_now, lo, hi)
                    st["jog_target"] = kin.pose(st["jog_q"])
                    st["jog_t"] = time.monotonic()
                dt = min(0.1, time.monotonic() - st["jog_t"])
                st["jog_t"] = time.monotonic()
                inp = {"lx": v.get("x", 0.0), "ly": v.get("y", 0.0), "rx": 0.0, "ry": v.get("z", 0.0),
                       "tilt": float(np.sign(v.get("t", 0.0))), "wf": float(np.sign(v.get("w", 0.0))),
                       "slow": False}
                st["jog_q"], st["jog_target"], st["jog_ok"] = arm_xpad.cartesian_step(
                    kin, st["jog_q"], st["jog_target"], inp, jog_cfg, lo, hi, np.ones(5), dt, st["level"])
                bus.sync_write("Goal_Position", dict(zip(ARM, np.degrees(st["jog_q"]))))
                if st["state"] not in ("lock", "holding"):  # driving away invalidates a plan
                    st["state"], st["plan"] = "lock", None
            else:
                st["jog_q"], st["jog_ok"] = None, True
            if mag.magnet and time.monotonic() - mag.polled > 1.0:
                mag.poll()
            mag.rearm_if_due()
            # ---- keys and buttons
            for k in read_keys(0.0):
                if k != "x":
                    st["quit_armed"] = False
                if k in ("a", "d", "w", "s"):
                    mot, d = {"a": ("head_pan", 1), "d": ("head_pan", -1),
                              "w": ("head_tilt", 1), "s": ("head_tilt", -1)}[k]
                    lo_h, hi_h = head_lim[mot]
                    head_goal[mot] = min(hi_h, max(lo_h, head_goal[mot] + 2.0 * d))
                    bus.sync_write("Goal_Position", head_goal)
                    pan, tilt = head_goal["head_pan"], head_goal["head_tilt"]
                    if not (cal_lo[0] <= pan <= cal_hi[0] and cal_lo[1] <= tilt <= cal_hi[1]):
                        say(f"head {pan:+.0f}/{tilt:+.0f} is OUTSIDE the calibrated range (pan "
                            f"{cal_lo[0] + 3:.0f}..{cal_hi[0] - 3:.0f}, tilt {cal_lo[1] + 3:.0f}..{cal_hi[1] - 3:.0f}): "
                            "targets locked here may be cm off", 1e9)
                    else:
                        say(f"head {pan:+.0f}/{tilt:+.0f} (inside the calibrated range)")
                elif k == "m":
                    on = magnet_set(not mag.on)
                    if st["state"] == "holding" and on is False:
                        st["state"], st["flag"] = "lock", "HELD"
                        say("released. Lock the next target")
                elif k == "k" and st["state"] == "at_hover":
                    if not mag.magnet:
                        say("no magnet board: plug in the ESP32 and restart")
                    else:
                        descend_and_pick()
                elif k in "1234":
                    st["level"] = int(k) - 1
                    say(f"drive speed L{k}")
                elif k == "f" and st["state"] != "at_hover":
                    if st["arm_held"] and mag.on:
                        say("magnet is ON: turn it off before freeing the arm")
                    else:
                        hold(not st["arm_held"])
                elif k == " ":
                    do_lock()
                elif k == "p" and st["locked"] is not None and st["state"] == "lock":
                    do_plan()
                elif k == "g" and st["state"] == "planned":
                    q_g = arm_q()
                    if kin.tool_point(q_g, hc.magnet_m)[2] < st["locked"][2] + CLEAR_M:
                        # the first move is joint-interpolated: from low down it can sweep the
                        # magnet through the package, so it starts only from well above it
                        say(f"GO refused: the magnet is {(kin.tool_point(q_g, hc.magnet_m)[2] - st['locked'][2]) * 100:.1f} "
                            "cm above the target and the first move needs 8 cm. Drive it up (PageUp / height strip), "
                            "then GO", 20.0)
                    else:
                        do_go()
                elif k == "o" and st["state"] == "lock":
                    auto_pick()
                elif k == "r":
                    record_flat()
                elif k == "u" and st["state"] == "at_hover" and st["plan"]:
                    if move_to(st["plan"]["high"], "MOVING"):
                        st["state"], st["flag"] = "planned", "PLANNED"
                        say("at 12 cm above. GO repeats the descent", 1e9)
                elif k in ("x", "\x1b"):
                    if st["arm_held"] and not st["quit_armed"]:
                        st["quit_armed"] = True
                        say("SUPPORT THE ARM: torque goes off. Quit again to confirm", 1e9)
                    else:
                        run = False
            txt, msg = publish(q_now)
            if time.monotonic() - last_print > 0.2:
                last_print = time.monotonic()
                print("\r" + txt + (" | " + msg if msg else "") + "\x1b[K", end="", flush=True)
            time.sleep(max(0.0, 0.02 - (time.monotonic() - t_tick)))
    except KeyboardInterrupt:
        pass
    finally:
        # each step on its own: a magnet that won't answer skipped the rest once, and the process
        # aborted with the arm still under torque (2026-11-13)
        for step in (lambda: termios.tcsetattr(fd, termios.TCSADRAIN, old),
                     lambda: mag.magnet and mag.magnet.off(), lambda: mag.magnet and mag.magnet.close(),
                     worker.stop, lambda: torque_limit(FULL_TORQUE), lambda: bus.disconnect(disable_torque=True)):
            try:
                step()
            except BaseException as e:
                print(f"\ncleanup: {e}")
        print("\nmagnet off, torque off")


def cmd_view(a):
    """Camera only, no servos: the live image with every marker's id, size and distance, in the
    same preview page as capture. For aiming the head or checking the D435 when the arm's board
    is off or unplugged."""
    cam = Camera(marker_m=a.marker_m)
    print("live view; Ctrl-C quits")
    try:
        while True:
            color, depth = cam.grab()
            try:
                ms, corners, ids = cam.markers(color, depth)
            except Exception:
                ms, corners, ids = {}, [], None
            parts = []
            for i, m in sorted(ms.items()):
                pt = cam.point(m)
                parts.append(f"id{i} {m['size_px']:.0f}px" + (f" {pt[2] * 100:.0f}cm" if pt else ""))
            line = " | ".join(parts) or "no markers"
            print("\r" + line + "\x1b[K", end="", flush=True)
            if not a.no_view:
                draw(cam, color, corners, ids, line[:90],
                     {"title": "Camera view", "lines": [line], "checks": [], "box": None, "reach": None,
                      "progress": [], "stage": 0, "stages": 0, "done": False, "ready": False})
    except KeyboardInterrupt:
        pass
    finally:
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
    s.add_argument("--touch-z", type=float,
                   help="height (m, base_link: the arm's mounting surface) of the surface the touches "
                        "were on, e.g. the package top; pins the height that touches on one surface leave loose")
    s.add_argument("--save-offsets", action="store_true",
                   help=f"save the arm joint offsets even if RMS is over {OFFSETS_MAX_RMS_MM:.0f} mm")
    s.add_argument("--no-offsets", action="store_true",
                   help="don't fit the arm's joint offsets (lift/elbow/wrist zero vs the URDF's)")
    k = sub.add_parser("check")
    k.add_argument("--marker-id", type=int, default=None, help="the package's marker (default 10)")
    k.add_argument("--head", type=float, nargs=2, metavar=("PAN", "TILT"),
                   help="hold the head here (degrees): use an angle the calibration was taken at")
    sub.add_parser("view", help="live camera + markers only; no servos needed")
    pk = sub.add_parser("pick", help="stage 1: lock a package, plan, hover 5 cm above it (no contact)")
    pk.add_argument("--marker-id", type=int, default=None, help="the package's marker (default 10)")
    pk.add_argument("--magnet-on-mm", type=float, default=CONTACT_ABOVE_M * 1000,
                    help="switch the magnet on this far above the computed surface on the way down, mm "
                         "(default 10: at the start of the press)")
    pk.add_argument("--press-mm", type=float, default=PRESS_BELOW_M * 1000,
                    help="how far under the computed surface the gentle press aims, mm (default 10)")
    pk.add_argument("--radial-mm", type=float, default=-8.0,
                    help="aim correction along the line from the arm base through the target, mm; "
                         "negative = toward the base (default -8: the magnet slides out ~8 mm as it seats)")
    pk.add_argument("--head", type=float, nargs=2, metavar=("PAN", "TILT"), default=list(PICK_HEAD),
                    help=f"hold the head here (default {PICK_HEAD[0]:.0f}/{PICK_HEAD[1]:.0f}; "
                         f"HOME, the best-calibrated angle, is {HOME_HEAD[0]}/{HOME_HEAD[1]})")
    a = ap.parse_args(argv)
    {"capture": cmd_capture, "solve": cmd_solve, "check": cmd_check, "view": cmd_view,
     "pick": cmd_pick}[a.cmd](a)


if __name__ == "__main__":
    main()
