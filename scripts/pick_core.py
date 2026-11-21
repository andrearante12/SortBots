"""The real robot's one-button pick and place, without the hardware: what `handeye pick`'s PICK
button (scripts/handeye.py cmd_pick -> auto_pick) does, over injected interfaces so a ROS node
can drive it. numpy + head_cam + so101_kin; OpenCV only for the marker detector. No ROS, no
lerobot, no pyrealsense2.

    find_marker -> lock_with_step_aside (do_lock) -> do_plan -> raise_clear / do_go
                -> descend_and_pick -> do_place

The maths, constants, thresholds, step order and refusals are handeye's, copied rather than
re-derived: each piece names the handeye.py function it came from, and
tests/pick_core_test.py runs handeye's own closures beside these on the same fakes and checks
they write the same goals. handeye.py stays the team's working tool; a later change can make it
call this instead of carrying its own copy. Change one, change both, until then.

Interfaces (duck-typed; the node fills them from its topics and drivers):

    ArmIO     read() -> {joint: deg} for ARM + HEAD, lerobot DEGREES convention
              write({joint: deg})            sync goal write, any subset of ARM + HEAD
              set_torque_limit(joint, raw)   Torque_Limit, 0..1000 raw
              hold(joints)                   goal = present, then torque on (handeye set_hold)
    Camera    latest() -> None or {"color": BGR uint8 HxWx3, "depth": metres HxW aligned to
              colour, "K": 3x3, "stamp": anything that changes per frame, "head_deg": (pan, tilt)
              at capture, optional "dist": 5 coeffs}
    Magnet    on(seconds) / off() / rearm() (re-send on inside the firmware's 30 s cap) /
              status() -> "ON" / "OFF" / None (can't say). Confirmation is by status(), as
              handeye's MagnetLink trusts only what the board reports.
    Base      move(dist_m, speed_mps) -> bool, + forward; False if it stopped short. It must
              watch the same abort Event and stop the wheels on it.
    abort     threading.Event: checked where handeye's moves read "any key". Abort holds the arm
              where it is and NEVER switches the magnet off: a caught package stays caught.
    report    report(phase, note, **extra): phase is handeye's flag (MOVING, DESCENDING, ...).
"""
from __future__ import annotations

import dataclasses
import json
import logging
import math
import threading
import time
from pathlib import Path

import numpy as np

import head_cam
import so101_kin
from so101_kin import SO101Kin

log = logging.getLogger("pick_core")

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
HEAD = ["head_pan", "head_tilt"]
REPO = Path(__file__).resolve().parent.parent
CALIBRATION_DIR = REPO / "configs" / "calibration"   # the committed copy of the robot's files

# ---- constants: handeye.py 1154-1207, unchanged (the rationale for each lives there) ----------
HOME_HEAD = (90.4, 60.7)
PICK_HEAD = (9.0, 57.0)
MAGNET_REARM_S, MAGNET_ON_S = 10.0, 30
HIGH_M, HOVER_M = 0.12, 0.05
CONTACT_ABOVE_M, PRESS_BELOW_M = 0.01, 0.015
PRESS_TORQUE, FULL_TORQUE = 350, 1000
PICK_PITCH_DEG = -4.7
FACE_TOP_DEG, FACE_SIDE_DEG, SIDE_MAX_YAW_DEG = 35.0, 35.0, 12.0
TOP_HIGH_CANDS_M, TOP_HOVER_CANDS_M = (0.12, 0.09, 0.06), (0.05, 0.03, 0.02)
PITCH_FALLBACK_DEG = (-3, 3, -6, 6, -9, 9, -12, -15, -18, -21)
LIFT_PRE_M, LIFT_TILT_DEG = 0.015, 15.0
PULL_IN_CM, PULL_IN_OFF_FLAT_DEG, PULL_MPS = 7.0, 12.0, 0.02
SIDE_HIGH_M, SIDE_HOVER_M, SIDE_LIFT_M, SIDE_PRESS_MM = 0.06, 0.03, 0.08, 8.0
WRIST_COMPLY_TORQUE = 100
PRESS_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
GRIP_WAIT_S = 0.5
SEARCH_PAN_STEPS = (0, 20, -20, 40, -40, 60, -60, 80, -80)
SEARCH_TILT_STEPS = (0, -15, 15)
SEARCH_DPS, SEARCH_SETTLE_S = 60.0, 0.3
CLEAR_M = 0.08
PICK_DPS = 30.0
DESCEND_MPS, PRESS_MPS, LIFT_MPS = 0.06, 0.012, 0.06
LOCK_FRAMES, LOCK_MIN_FRAMES, LOCK_MAX_SPREAD_MM = 20, 12, 8.0
LOCK_ASIDE_DEG = 40.0
BASE_SPEED_MPS = 0.05   # scripts/base.py SPEED_MPS: the speed handeye's place backs off at
MAX_RES = 4095          # scripts/head.py MAX_RES (STS3215 ticks per turn)


@dataclasses.dataclass
class PickConfig:
    """handeye pick's command-line options (handeye.py main(), the `pick` parser), same defaults."""
    marker_m: float = 0.020
    magnet_on_mm: float = CONTACT_ABOVE_M * 1000
    press_mm: float = PRESS_BELOW_M * 1000
    press_torque: int = PRESS_TORQUE
    base_back_cm: float = 15.0
    lift_tilt_deg: float = LIFT_TILT_DEG
    pull_in_cm: float = PULL_IN_CM
    pull_in_off_flat_deg: float = PULL_IN_OFF_FLAT_DEG
    wrist_torque: int = WRIST_COMPLY_TORQUE
    pitch_deg: float = PICK_PITCH_DEG
    side_pitch_deg: float | None = None   # None: the mean of the taught side flat poses, else 0
    side_press_mm: float = SIDE_PRESS_MM
    radial_mm: float = 1.0
    head: tuple = PICK_HEAD


@dataclasses.dataclass
class PickPoses:
    """What cmd_pick loads from the calibration directory: the touch samples (IK seeds), the
    taught flat poses (side pitch), the rest and place poses, and the joint limits."""
    samples: list
    flat_poses: list
    rest_q: np.ndarray | None
    place_q: np.ndarray | None
    lo: np.ndarray
    hi: np.ndarray
    head_lim: dict
    table_q: np.ndarray | None = None   # Unload's table pose (pick_table_pose.json); not handeye's


def arm_limits(cal):
    """handeye.arm_limits, from the calibration JSON instead of a bus: the calibrated range in
    radians, 2 deg inside the stops."""
    lo, hi = [], []
    for j in ARM:
        c = cal[j]
        mid = (c["range_min"] + c["range_max"]) / 2
        lo.append(np.radians((c["range_min"] - mid) * 360 / MAX_RES + 2))
        hi.append(np.radians((c["range_max"] - mid) * 360 / MAX_RES - 2))
    return np.array(lo), np.array(hi)


def head_limits(cal):
    """head.limits, from the calibration JSON: degrees round the mid, 2 deg inside the stops."""
    out = {}
    for m in HEAD:
        c = cal[m]
        mid = (c["range_min"] + c["range_max"]) / 2
        out[m] = ((c["range_min"] - mid) * 360 / MAX_RES + 2, (c["range_max"] - mid) * 360 / MAX_RES - 2)
    return out


def load_robot(calib_dir=CALIBRATION_DIR, arm_id="sortbots_arm", head_id="sortbots_head"):
    """(kin, headcam, poses) from a lerobot-style calibration tree (default: the repo's copy,
    configs/calibration/robots/{so_follower,sortbots_head}). The joint offsets are passed to
    SO101Kin explicitly, so HF_LEROBOT_CALIBRATION doesn't have to point here."""
    root = Path(calib_dir) / "robots"
    arm_dir, head_dir = root / "so_follower", root / "sortbots_head"
    kin = SO101Kin(offsets=so101_kin.load_offsets(arm_dir / f"{arm_id}_kin_offsets.json"))
    if not np.any(kin.off):  # cmd_pick refuses the same way
        raise RuntimeError("no arm joint offsets saved: run `handeye solve` first")
    hc = head_cam.HeadCam.load(head_dir / f"{head_id}_camera.json")
    lo, hi = arm_limits(json.loads((arm_dir / f"{arm_id}.json").read_text()))
    head_lim = head_limits(json.loads((head_dir / f"{head_id}.json").read_text()))

    def jload(name, default):
        p = head_dir / name
        return json.loads(p.read_text()) if p.is_file() else default
    rest, place = jload("pick_rest_pose.json", None), jload("pick_place_pose.json", None)
    table = jload("pick_table_pose.json", None)
    poses = PickPoses(samples=jload("handeye_samples.json", []), flat_poses=jload("pick_flat_poses.json", []),
                      rest_q=None if rest is None else np.radians(rest["arm_deg"]),
                      place_q=None if place is None else np.radians(place["arm_deg"]),
                      lo=lo, hi=hi, head_lim=head_lim,
                      table_q=None if table is None else np.radians(table["arm_deg"]))
    return kin, hc, poses


# ---- the marker: handeye.Camera.markers / face_normal on plain arrays --------------------------
class MarkerDetector:
    """ArUco DICT_4X4_50 markers in a colour frame, measured with the aligned depth: handeye's
    Camera.markers and Camera.face_normal, given arrays instead of a pyrealsense2 pipeline."""

    def __init__(self, marker_m=0.020):
        import cv2
        self.cv2, self.marker_m = cv2, marker_m
        self.want = None  # marker ids that get the depth plane fit + target; None = all
        a = cv2.aruco
        table = a.getPredefinedDictionary(a.DICT_4X4_50)
        # both OpenCV aruco APIs, as nodes/package_vision.py: ArucoDetector from 4.7 on
        new_api = hasattr(a, "ArucoDetector")
        params = a.DetectorParameters() if new_api else a.DetectorParameters_create()
        # handeye's setting, kept: 2 cm markers are ~30-60 px across, subpixel corners matter for
        # PnP. (package_vision found it a no-op in OpenCV >= 4.7 and refines by cornerSubPix
        # itself; that would change the numbers handeye was tuned with, so not here.)
        params.cornerRefinementMethod = a.CORNER_REFINE_SUBPIX
        if new_api:
            det = a.ArucoDetector(table, params)
            self._detect = det.detectMarkers
        else:
            self._detect = lambda img: a.detectMarkers(img, table, parameters=params)

    def markers(self, color, depth, K, dist=None):
        """handeye Camera.markers: {id: {"px", "depth_pt", "pnp_pt", "size_px", "normal",
        "target_pt", ("centre", "R")}}, colour optical frame, metres; and the raw corners, ids."""
        cv2 = self.cv2
        K = np.asarray(K, float)
        dist = np.zeros(5) if dist is None else np.asarray(dist, float)
        fx, fy, ppx, ppy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        corners, ids, _ = self._detect(color)
        out = {}
        if ids is None:
            return out, corners, ids
        h = self.marker_m / 2
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
        for c, i in zip(corners, ids.ravel()):
            q = c.reshape(4, 2)
            u, v = q.mean(0)
            # depth: median over the marker's inner half (handeye: the edge pixels mix in
            # whatever is behind)
            r = max(2, int(np.linalg.norm(q[0] - q[2]) / 6))
            win = depth[int(v) - r:int(v) + r + 1, int(u) - r:int(u) + r + 1]
            win = win[(win > 0.1) & (win < 3.0)]
            dpt = None
            if win.size > 0.3 * (2 * r + 1) ** 2:
                # rs2_deproject_pixel_to_point with the D435 colour stream's all-zero distortion
                # (handeye.face_normal relies on the same): plain pinhole
                z = float(np.median(win))
                dpt = [(float(u) - ppx) / fx * z, (float(v) - ppy) / fy * z, z]
            ok, rvec, tvec = cv2.solvePnP(obj, q.astype(np.float32), K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            size_px = float(np.linalg.norm(q[0] - q[2]) / 1.414)
            m = {"px": [float(u), float(v)], "depth_pt": dpt,
                 "pnp_pt": [float(t) for t in tvec.ravel()] if ok else None, "size_px": size_px,
                 "normal": None, "target_pt": None}
            centre = dpt if dpt is not None else m["pnp_pt"]
            if ok and centre is not None and (self.want is None or int(i) in self.want):
                R = cv2.Rodrigues(rvec)[0]
                # the face's plane halfway to the target, not round the marker (handeye: the
                # marker sits near a corner of its face; a box round it takes in the table)
                mid = np.asarray(centre) + R @ (head_cam.target_in_marker(self.marker_m) / 2)
                n = self.face_normal(depth, fx * mid[0] / mid[2] + ppx, fy * mid[1] / mid[2] + ppy, size_px, K)
                if n is not None:
                    R = head_cam.align_z(R, n)
                m["normal"] = [float(x) for x in R[:, 2]]
                m["centre"], m["R"] = [float(x) for x in centre], R.tolist()
                m["target_pt"] = [float(x) for x in
                                  np.asarray(centre) + R @ head_cam.target_in_marker(self.marker_m)]
            out[int(i)] = m
        return out, corners, ids

    @staticmethod
    def face_normal(depth, u, v, size_px, K):
        """handeye Camera.face_normal: unit normal of the surface round pixel (u, v), from depth,
        a box 2.4 marker-widths wide; None if too few points or the box is off the image."""
        fx, fy, ppx, ppy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        r = int(1.2 * size_px)
        h, w = depth.shape
        v0, v1, u0, u1 = max(0, int(v) - r), min(h, int(v) + r), max(0, int(u) - r), min(w, int(u) + r)
        if v1 - v0 < 4 or u1 - u0 < 4:
            return None
        z = depth[v0:v1:2, u0:u1:2]
        vv, uu = np.mgrid[v0:v1:2, u0:u1:2]
        ok = (z > 0.1) & (z < 3.0)
        if ok.sum() < 50:
            return None
        z, uu, vv = z[ok], uu[ok], vv[ok]
        pts = np.stack([(uu - ppx) / fx * z, (vv - ppy) / fy * z, z], 1)
        return head_cam.plane_fit(pts)[1]


def consolidate(ents, marker_m):
    """handeye.consolidate: one target from several frames of one marker, the median centre and
    median face orientation first, THEN the target out along the face. -> (target, normal,
    centre spread mm, normal spread deg), camera frame; None if no frame had an orientation."""
    ents = [m for m in ents if m.get("R") is not None]
    if not ents:
        return None
    C = np.array([m["centre"] for m in ents])
    Rs = np.array([m["R"] for m in ents])
    c = np.median(C, 0)
    n = np.median(Rs[:, :, 2], 0)
    n /= np.linalg.norm(n)
    x = np.median(Rs[:, :, 0], 0)
    x -= n * (x @ n)
    x /= np.linalg.norm(x)
    R = np.stack([x, np.cross(n, x), n], 1)
    spread_c = float(np.median(np.linalg.norm(C - c, axis=1)) * 1000)
    spread_n = float(np.degrees(np.median(np.arccos(np.clip(Rs[:, :, 2] @ n, -1, 1)))))
    return c + R @ head_cam.target_in_marker(marker_m), n, spread_c, spread_n


def aim_corrected(p, radial_m):
    """handeye cmd_pick.aim_corrected: shift a target along the horizontal line from the
    shoulder_pan axis through it, negative = toward the base."""
    d = np.array([p[0] - head_cam.PAN_AXIS_X_M, p[1]])
    n = np.linalg.norm(d)
    if n < 1e-6 or radial_m == 0.0:
        return p
    return p + np.array([*(d / n * radial_m), 0.0])


def classify_face(p, n):
    """handeye cmd_pick.classify_face: (("top", up) or ("side", the normal turned into the arm's
    plane), None) or (None, why) for a slanted face or a side face not facing the arm."""
    elev = math.degrees(math.asin(max(-1.0, min(1.0, float(n[2])))))
    if elev >= 90.0 - FACE_TOP_DEG:
        return ("top", np.array([0.0, 0.0, 1.0])), None
    if abs(elev) > FACE_SIDE_DEG:
        return None, f"the marked face is slanted ({elev:.0f} deg up): only a top or a side face can be picked"
    rad = head_cam.arm_radial(p)
    nh = n[:2] / max(1e-6, np.linalg.norm(n[:2]))
    yaw = math.degrees(math.acos(max(-1.0, min(1.0, float(nh @ -rad)))))
    if yaw > SIDE_MAX_YAW_DEG:
        return None, (f"the marked side faces {yaw:.0f} deg away from the arm (max {SIDE_MAX_YAW_DEG:.0f}): turn "
                      "the box so that side faces the arm's base")
    ax = np.array([-rad[0] * math.cos(math.radians(elev)), -rad[1] * math.cos(math.radians(elev)),
                   math.sin(math.radians(elev))])
    if yaw > 4.0:
        log.info("side face %.0f deg off the arm's plane: approaching square to the arm instead", yaw)
    return ("side", ax), None


class _MagnetLink:
    """handeye.MagnetLink over the Magnet interface: the magnet as the board reports it, not as
    last commanded. Reconnecting and draining the board's own lines are the node's job now
    (status() answers None while it can't say)."""

    def __init__(self, magnet, say, clock):
        self.magnet, self.say, self.clock = magnet, say, clock
        self.on = False
        self.t0 = self.armed_at = self.polled = 0.0

    def _call(self, fn):
        try:
            return fn()
        except Exception as e:  # a driver that raises is a board that can't say
            self.say(f"magnet: {e}", 1e9)
            return None

    def poll(self):
        if not self.magnet:
            return None
        s = self._call(self.magnet.status)
        if s is None:
            return None
        self.polled = self.clock.monotonic()
        actual = s == "ON"
        if self.on and not actual:
            self.say("magnet board reports OFF (it reset or timed out)", 1e9)
        self.on = actual
        return actual

    def set(self, on, settle_s=0.15):
        if not self.magnet:
            return None
        self._call(lambda: self.magnet.on(MAGNET_ON_S) if on else self.magnet.off())
        if not on:
            self.on = False  # else the poll below calls a commanded release "reports OFF (it reset)"
        else:
            self.armed_at = self.t0 = self.clock.monotonic()
        self.clock.sleep(settle_s)  # a reset from the coil's inrush shows up within this
        actual = self.poll()
        if actual is not None and actual != on:
            self.say(f"magnet did NOT turn {'on' if on else 'off'}: the board reports {'on' if actual else 'off'}", 1e9)
        elif actual is not None:
            self.say(f"magnet {'ON' if on else 'OFF'} (confirmed by the board)")
        return actual

    def rearm_if_due(self):
        if self.magnet and self.on and self.clock.monotonic() - self.armed_at > MAGNET_REARM_S:
            self._call(self.magnet.rearm)
            self.armed_at = self.clock.monotonic()


class PickCore:
    """handeye cmd_pick's pick-and-place steps as methods; `st` keeps cmd_pick's state keys."""

    MOVING_FLAGS = ("MOVING", "DESCENDING", "PRESSING", "LIFTING", "SEARCHING")

    def __init__(self, io, cam, magnet, base, kin, headcam, poses, cfg=None, *, abort=None, report=None,
                 clock=time, detector=None):
        self.io, self.cam, self.base, self.kin, self.hc = io, cam, base, kin, headcam
        self.cfg = cfg or PickConfig()
        self.poses = poses
        self.abort = abort or threading.Event()
        self.report = report or (lambda phase, note, **extra: None)
        self.clock = clock
        self.lo, self.hi, self.head_lim = poses.lo, poses.hi, poses.head_lim
        self._detector = detector
        self.marker_id = 10
        # cmd_pick 1456-1460: the touches are IK seeds; side faces get their own fold of the arm
        self.seeds = [np.radians(s["arm_deg"]) for s in poses.samples]
        self.side_seeds = [np.radians([0, l_, e_, w_, 0]) for l_ in (-40, 0, 30, 60) for e_ in (-60, 0, 40, 80)
                           for w_ in (-60, -20, 20)]
        if self.cfg.side_pitch_deg is None:  # cmd_pick 1576-1579
            side_taught = [r["model_pitch_deg"] for r in poses.flat_poses
                           if r.get("face") == "side" and abs(r["model_pitch_deg"]) < 20.0]
            self.cfg = dataclasses.replace(self.cfg, side_pitch_deg=float(np.mean(side_taught)) if side_taught else 0.0)
        self.mag = _MagnetLink(magnet, self.say, clock)
        self.st = {"arm_held": False, "locked": None, "plan": None, "state": "lock", "flag": "FREE",
                   "face": "top", "axis": np.array([0.0, 0.0, 1.0]), "high_m": HIGH_M, "hover_m": HOVER_M,
                   "pitch_top": None, "pitch_app": None, "rest_q": poses.rest_q, "place_q": poses.place_q,
                   "lift_tilt_rad": 0.0, "table_q": poses.table_q}
        self.head_goal = None
        self.unloading = False   # unload(): no pull-in, the table pose, no base move
        self._reported = None
        self._ik_memo = {}

    # ---- reporting (cmd_pick say / publish) -------------------------------------------------
    def say(self, text, secs=8.0):
        self.report(self.st["flag"], text, state=self.st["state"], secs=secs, magnet=self._mag_str())
        self._reported = (self.st["flag"], self.st["state"])

    def publish(self):
        """cmd_pick.publish, minus the page: a report when the flag or state changed."""
        key = (self.st["flag"], self.st["state"])
        if key != self._reported:
            self._reported = key
            self.report(key[0], "", state=key[1], magnet=self._mag_str())

    def _mag_str(self):
        return None if not self.mag.magnet else ("ON" if self.mag.on else "OFF")

    @property
    def detector(self):
        if self._detector is None:
            self._detector = MarkerDetector(self.cfg.marker_m)
        return self._detector

    # ---- the arm -----------------------------------------------------------------------------
    def arm_q(self):
        pos = self.io.read()
        return np.radians([pos[j] for j in ARM])

    def head_now(self):
        pos = self.io.read()
        return {m: pos[m] for m in HEAD}

    def hold(self, on=True):
        """cmd_pick.hold(True). Freeing the arm is the node's business, not the pick's."""
        if on:
            self.io.hold(ARM)
        self.st["arm_held"] = on
        if self.st["flag"] in ("FREE", "HELD", "ABORTED"):
            self.st["flag"] = "HELD" if on else "FREE"

    def _aborted(self):
        """cmd_pick move loops on any key: hold where it is (goal = present, torque on)."""
        self.io.hold(ARM)
        self.st["flag"] = "ABORTED"
        self.say("ABORTED: holding where it is", 1e9)
        return False

    def move_to(self, q_goal, label, dps=PICK_DPS):
        """cmd_pick.move_to: joint-interpolated, smoothstep, 50 Hz; abort holds. True if arrived."""
        start = dict(zip(ARM, np.degrees(self.arm_q())))
        goal = dict(zip(ARM, np.degrees(q_goal)))
        dur = max(0.5, max(abs(goal[j] - start[j]) for j in ARM) / dps)
        self.st["flag"] = label
        t0 = self.clock.monotonic()
        while True:
            s_ = min(1.0, (self.clock.monotonic() - t0) / dur)
            s_ = s_ * s_ * (3 - 2 * s_)
            self.io.write({j: start[j] + (goal[j] - start[j]) * s_ for j in ARM})
            if self.abort.is_set():
                return self._aborted()
            self.mag.rearm_if_due()
            self.publish()
            if s_ >= 1.0:
                self.clock.sleep(0.4)
                return True
            self.clock.sleep(0.02)

    def move_path(self, zq, label, mps):
        """cmd_pick.move_path: one continuous move through straight()'s (key, q) poses, timed by
        the magnet's travel at `mps`, eased at the two ends; abort holds. True if arrived."""
        if not zq:
            return True
        q0 = self.arm_q()
        qr = [q0] + [q for _, q in zq]
        pts = np.array([self.kin.tool_point(q, self.hc.magnet_m) for q in qr])
        qs = np.degrees(np.array(qr))
        cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        dur = max(0.5, cum[-1] / mps)
        self.st["flag"] = label
        t0 = self.clock.monotonic()
        while True:
            s_ = min(1.0, (self.clock.monotonic() - t0) / dur)
            d = s_ * s_ * (3 - 2 * s_) * cum[-1]
            i = min(len(cum) - 2, int(np.searchsorted(cum, d, side="right")) - 1)
            f = 0.0 if cum[i + 1] <= cum[i] else (d - cum[i]) / (cum[i + 1] - cum[i])
            self.io.write(dict(zip(ARM, qs[i] + (qs[i + 1] - qs[i]) * f)))
            if self.abort.is_set():
                return self._aborted()
            self.mag.rearm_if_due()
            self.publish()
            if s_ >= 1.0:
                self.clock.sleep(0.2)
                return True
            self.clock.sleep(0.02)

    def torque_limit(self, val):
        """cmd_pick.torque_limit"""
        for j in PRESS_JOINTS:
            self.io.set_torque_limit(j, val)

    def freeze(self):
        """cmd_pick.freeze: goal = where the arm IS before torque comes back (after a press the
        goal sits under the package)."""
        pos = self.io.read()
        self.io.write({j: pos[j] for j in ARM})

    def magnet_set(self, on):
        """cmd_pick.magnet_set: on needs the arm held."""
        if on and not self.st["arm_held"]:
            self.say("hold the arm before turning the magnet on")
            return False
        return self.mag.set(on)

    # ---- the planner (cmd_pick closures) ----------------------------------------------------
    def _ik(self, kin, target, seeds, lo, hi, magnet_m, roll, **kw):
        """head_cam.ik_down, through head_cam.ik_down_batch (the same answer, all seeds solved at
        once) and a memo of exact repeats: do_plan asks for the same high/hover/contact poses its
        reach search already solved (2026-11-16: 3 of 28 solves). The key is every input's exact
        bytes, so a hit is the very call it stands for; joint arrays go out as copies."""
        arr = lambda v: None if v is None else np.asarray(v, float).tobytes()  # noqa: E731
        kw = {"pos_tol_m": 0.002, "tilt_tol_deg": 2.0, "pitch_deg": 0.0, "axis": None, **kw}  # ik_down's defaults
        key = (id(kin), arr(target), tuple(arr(s) for s in seeds), arr(lo), arr(hi), float(magnet_m), float(roll),
               float(kw["pos_tol_m"]), float(kw["tilt_tol_deg"]), float(kw["pitch_deg"]), arr(kw["axis"]))
        if key not in self._ik_memo:
            if len(self._ik_memo) > 512:
                self._ik_memo.clear()
            self._ik_memo[key] = head_cam.ik_down_batch(kin, target, seeds, lo, hi, magnet_m, roll, **kw)
        r = self._ik_memo[key]
        return None if r is None else (r[0].copy(), r[1], r[2])

    def down_ik(self, target, seeds_, roll):
        return self._ik(self.kin, target, seeds_, self.lo, self.hi, self.hc.magnet_m, roll,
                        pitch_deg=self.cfg.pitch_deg)

    def model_pitch(self, q):
        """cmd_pick.model_pitch"""
        F, tool = self.kin.flange(q), self.kin.tool_point(q, self.hc.magnet_m)
        return head_cam.face_pitch(F[:3, 2], head_cam.arm_radial(tool), self.st["axis"])

    def face_ik(self, target, seeds_, roll, pitch=None, tol=None):
        """cmd_pick.face_ik"""
        st, cfg = self.st, self.cfg
        if st["face"] == "top" and pitch is None and tol is None:
            if st.get("pitch_top") is not None and st["pitch_top"] != cfg.pitch_deg:
                return self._ik(self.kin, target, seeds_, self.lo, self.hi, self.hc.magnet_m, roll,
                                pitch_deg=st["pitch_top"])
            return self.down_ik(target, seeds_, roll)
        side = st["face"] == "side"
        return self._ik(self.kin, target, list(seeds_) + (self.side_seeds if side else []), self.lo,
                        self.hi, self.hc.magnet_m, roll, tilt_tol_deg=tol or 2.0,
                        pitch_deg=(cfg.side_pitch_deg if side else cfg.pitch_deg) if pitch is None else pitch,
                        axis=st["axis"] if side else None)

    def along(self, d):
        """cmd_pick.along"""
        return self.st["locked"] + self.st["axis"] * d

    def straight(self, z_from, z_to, step, pitch=None):
        """cmd_pick.straight"""
        qs, seed = [], [self.arm_q()]
        n = max(1, int(round(abs(z_to - z_from) / step)))
        for i in range(1, n + 1):
            z = z_from + (z_to - z_from) * i / n
            r = self.face_ik(self.along(z), seed, float(seed[0][4]), pitch=pitch,
                             tol=3.0 if pitch is not None else None)
            if r is None:
                return None
            qs.append((z, r[0]))
            seed = [r[0]]
        return qs

    def straight_blend(self, z_from, z_to, step):
        """cmd_pick.straight_blend"""
        st = self.st
        p_app, p_top, h0 = st["pitch_app"], st["pitch_top"], max(st["hover_m"], 1e-6)
        sgn = 1.0 if p_app >= p_top else -1.0
        qs, seed = [], [self.arm_q()]
        n = max(1, int(round(abs(z_to - z_from) / step)))
        for i in range(1, n + 1):
            z = z_from + (z_to - z_from) * i / n
            want = p_top + (p_app - p_top) * min(1.0, max(0.0, z / h0))
            r = None
            for extra in range(0, 11):
                r = self.face_ik(self.along(z), seed, float(seed[0][4]), pitch=want + sgn * extra, tol=2.0)
                if r is not None:
                    break
            if r is None:
                return None
            qs.append((z, r[0]))
            seed = [r[0]]
        return qs

    def lift_path(self, p0, height, pitch):
        """cmd_pick.lift_path"""
        qs, seed = [], [self.arm_q()]
        n = max(1, int(round(height / 0.01)))
        for i in range(1, n + 1):
            h = height * i / n
            r = self.face_ik(p0 + np.array([0.0, 0.0, h]), seed, float(seed[0][4]), pitch=pitch, tol=3.0)
            if r is None:
                return None
            qs.append((h, r[0]))
            seed = [r[0]]
        return qs

    def pull_in_path(self, p0, dist_m, pitch):
        """cmd_pick.pull_in_path"""
        r_hat = np.append(head_cam.arm_radial(p0), 0.0)
        qs, seed = [], [self.arm_q()]
        n = max(1, int(round(dist_m / 0.01)))
        for i in range(1, n + 1):
            d = dist_m * i / n
            # the pitch may drift a few degrees as the arm comes in (the hand-driven pull went
            # -24 -> -26): held exactly, only 4 of 7 cm solved
            for p_ in (pitch, pitch - 3.0, pitch + 3.0, pitch - 6.0):
                r = self.face_ik(p0 - r_hat * d, seed, float(seed[0][4]), pitch=p_, tol=3.0)
                if r is not None:
                    pitch = p_
                    break
            if r is None:
                break
            qs.append((d, r[0]))
            seed = [r[0]]
        return qs if qs and qs[-1][0] >= 0.02 - 1e-9 else None

    # ---- the steps --------------------------------------------------------------------------
    def entries(self, marker_id, n=10, timeout=4.0, give_up_s=1.5):
        """handeye CamWorker.entries over Camera.latest(): `marker_id`'s entries with a target
        from the next n NEW frames; none at all after give_up_s ends it early."""
        self.detector.want = {marker_id}
        fr = self.cam.latest()
        out, last, t0 = [], (None if fr is None else fr["stamp"]), self.clock.monotonic()
        while len(out) < n and self.clock.monotonic() - t0 < timeout:
            if not out and self.clock.monotonic() - t0 > give_up_s:
                break
            fr = self.cam.latest()
            if fr is not None and fr["stamp"] != last:
                last = fr["stamp"]
                try:
                    ms = self.detector.markers(fr["color"], fr["depth"], fr["K"], fr.get("dist"))[0]
                except Exception:  # a frame that fails to process counts as not seen (handeye)
                    ms = {}
                m = ms.get(marker_id)
                if m and m.get("target_pt") is not None:
                    out.append(dict(m, head_deg=fr.get("head_deg")))
            self.clock.sleep(0.01)
        return out

    def do_lock(self):
        """handeye cmd_pick.do_lock: marker's target from LOCK_FRAMES frames. True if locked."""
        st, cfg = self.st, self.cfg
        ents = self.entries(self.marker_id, n=LOCK_FRAMES, timeout=6.0)
        pts = np.array([m["target_pt"] for m in ents]).reshape(-1, 3)
        cons = consolidate(ents, cfg.marker_m) if len(ents) >= LOCK_MIN_FRAMES else None
        med = np.median(pts, 0) if len(pts) else None
        dist = np.linalg.norm(pts - med, axis=1) if len(pts) else np.zeros(0)
        spread = float(np.median(dist) * 1000) if len(pts) else 0.0
        good = pts[dist <= max(0.004, 2.5 * spread / 1000)] if len(pts) else pts
        if cons is not None:  # judged by the marker's own steadiness, not the lever-arm wobble
            spread = cons[2]
        if len(pts) >= LOCK_MIN_FRAMES and spread <= LOCK_MAX_SPREAD_MM:
            # the head's angle read now, after the frames, as handeye (it holds still while looking)
            hp = self.head_now()
            T = self.hc.matrix(hp["head_pan"], hp["head_tilt"])
            if cons is not None:
                raw = (T @ np.append(cons[0], 1.0))[:3]
                n = T[:3, :3] @ cons[1]
                log.info("lock: centre spread %.1f mm, face normal spread %.1f deg", cons[2], cons[3])
            else:
                raw = (T @ np.append(np.median(good, 0), 1.0))[:3]
                nrm = [m["normal"] for m in ents if m.get("normal") is not None]
                n = T[:3, :3] @ np.median(nrm, 0) if nrm else np.array([0.0, 0.0, 1.0])
            n /= np.linalg.norm(n)
            face, why = classify_face(raw, n)
            if face is None:
                self.say(why, 20.0)
                st["lock_why"] = "face"
                return False
            st["face"], st["axis"] = face
            top = st["face"] == "top"
            st["high_m"], st["hover_m"] = (HIGH_M, HOVER_M) if top else (SIDE_HIGH_M, SIDE_HOVER_M)
            st["locked"] = aim_corrected(raw, cfg.radial_mm / 1000.0) if top else raw
            st["state"], st["plan"] = "lock", None
            if st["flag"] in ("PLANNED", "ABORTED", "HOVERING", "LOOKING"):
                st["flag"] = "HELD" if st["arm_held"] else "FREE"
            self.say(f"LOCKED {st['face']} face, target {np.round(st['locked'] * 100, 1)} cm ({len(good)}/{len(pts)} "
                     f"frames, spread {spread:.1f} mm" + (f"; aim {cfg.radial_mm:+.0f} mm radial" if top else "")
                     + "). Plan next")
            return True
        log.info("lock refused: %d frames, median spread %.1f mm", len(pts), spread)
        st["lock_why"] = "unseen" if len(pts) < LOCK_MIN_FRAMES else f"jitter {spread:.0f} mm"
        if len(pts) < LOCK_MIN_FRAMES:
            self.say(f"marker {self.marker_id} seen in only {len(pts)}/{LOCK_FRAMES} frames: move the arm clear "
                     "of it", 20.0)
        else:
            self.say(f"marker {self.marker_id} seen {len(pts)}/{LOCK_FRAMES} but its target jumps around (median "
                     f"{spread:.1f} mm): glare or part of it hidden? Try Lock again, or move the camera a little",
                     20.0)
        return False

    def do_plan(self):
        """handeye cmd_pick.do_plan: magnet-flat poses high, at the hover and at contact, and the
        approach between. True if all solvable inside the joint limits."""
        st, cfg = self.st, self.cfg
        q_now = self.arm_q()
        locked, roll = st["locked"], float(q_now[4])
        st["pitch_app"] = None
        seeds = self.seeds
        if st["face"] == "top":
            def at(pitch, d_):
                return self.face_ik(self.along(d_), seeds + [q_now], roll, pitch=pitch, tol=2.0) is not None
            st["pitch_top"] = next((cfg.pitch_deg + d for d in (0,) + PITCH_FALLBACK_DEG
                                    if at(cfg.pitch_deg + d, 0.0)), cfg.pitch_deg)
            if st["pitch_top"] != cfg.pitch_deg:
                log.info("taught pitch %+.1f out of reach here: contact at %+.1f", cfg.pitch_deg, st["pitch_top"])
            for d in (0,) + PITCH_FALLBACK_DEG:
                pa = st["pitch_top"] + d
                hv = next((h for h in TOP_HOVER_CANDS_M if at(pa, h)), None)
                if hv is not None:
                    st["pitch_app"], st["hover_m"] = pa, hv
                    break
            if st["pitch_app"] is None:
                self.say("top face NOT reachable from above with the magnet near flat: move the package", 20.0)
                return False
            if st["pitch_app"] != st["pitch_top"]:
                log.info("approach at %+.1f, turning to %+.1f on the way down", st["pitch_app"], st["pitch_top"])
            st["high_m"] = next((h for h in TOP_HIGH_CANDS_M if h > st["hover_m"] and at(st["pitch_app"], h)),
                                st["hover_m"])
        hi_m, hv_m = st["high_m"], st["hover_m"]
        pa = st["pitch_app"]
        sols = {n: self.face_ik(self.along(dz), seeds + [q_now], roll, pitch=pa if n != "contact" else None,
                                tol=2.0 if pa is not None and n != "contact" else None)
                for n, dz in (("high", hi_m), ("hover", hv_m), ("contact", 0.0))}
        bad = [n for n, v_ in sols.items() if v_ is None]
        if bad:
            self.say(f"{st['face']} face NOT reachable with the magnet flat: {', '.join(bad)}. "
                     + ("Move the package closer to the arm" if st["face"] == "top" else
                        "A side face needs to be ~20-30 cm from the base and facing it"), 20.0)
            return False
        steps, seed = [], [sols["high"][0]]
        for dz in np.arange(hi_m - 0.01, hv_m - 1e-6, -0.01):
            r = self.face_ik(self.along(dz), seed, roll, pitch=pa, tol=2.0 if pa is not None else None)
            if r is None:
                break
            steps.append((float(dz), r[0]))
            seed = [r[0]]
        if len(steps) < round((hi_m - hv_m) / 0.01):
            self.say("approach path not solvable with the magnet flat: move the package", 20.0)
            return False
        st["plan"], st["state"], st["flag"] = {"high": sols["high"][0], "steps": steps}, "planned", "PLANNED"
        span = max(abs(np.degrees(sols["high"][0] - q_now)))
        tilt = (f", magnet at {st['pitch_top']:+.0f} deg" if st["face"] == "top" and st["pitch_top"] != cfg.pitch_deg
                else "")
        self.say(f"PLAN ok ({st['face']} face{tilt}): {span:.0f} deg joint move to {hi_m * 100:.0f} cm out, then in "
                 f"to {hv_m * 100:.0f} cm. GO when ready", 1e9)
        return True

    def do_go(self):
        """handeye cmd_pick.do_go: joint move to the high pose, straight in to the hover."""
        st = self.st
        if not st["arm_held"]:
            self.hold(True)
        ok = self.move_to(st["plan"]["high"], "MOVING")
        if ok:
            ok = self.move_path(st["plan"]["steps"], "DESCENDING", DESCEND_MPS)
        if ok:
            st["state"], st["flag"] = "at_hover", "HOVERING"
            err = (self.kin.tool_point(self.arm_q(), self.hc.magnet_m) - self.along(st["hover_m"])) * 1000
            self.say(f"HOVERING {st['hover_m'] * 100:.0f} cm off the {st['face']} face (arm model: "
                     f"{np.linalg.norm(err):.0f} mm off its goal)", 1e9)
        else:
            st["state"] = "lock"
        return ok

    def raise_clear(self):
        """handeye cmd_pick.raise_clear"""
        return self.raise_to(self.st["locked"][2] + CLEAR_M + 0.01)

    def raise_to(self, need, quiet=False):
        """handeye cmd_pick.raise_to: the hand straight up (its tilt kept) to height `need`."""
        kin, mm, lo, hi = self.kin, self.hc.magnet_m, self.lo, self.hi
        q = self.arm_q()
        tool = kin.tool_point(q, mm)
        if tool[2] >= need:
            return True
        F = kin.flange(q)
        if F[2, 2] > math.cos(math.radians(25)):
            # magnet roughly down: SO101Kin.ik folds there, so the flange-axis solver (handeye)
            pitch = head_cam.face_pitch(F[:3, 2], head_cam.arm_radial(tool))
            path, seed = [], [q]
            n = max(1, int(math.ceil((need - tool[2]) / 0.01)))
            for i in range(1, n + 1):
                z = tool[2] + (need - tool[2]) * i / n
                r = self._ik(kin, [tool[0], tool[1], z], seed, lo, hi, mm, float(q[4]),
                             tilt_tol_deg=3.0, pitch_deg=pitch)
                if r is None:
                    break
                path.append((z, r[0]))
                seed = [r[0]]
            if path:
                if not self.move_path(path, "MOVING", LIFT_MPS):
                    return False
                if path[-1][0] >= need - 1e-6:
                    return True
                q = self.arm_q()
                if kin.flange(q)[2, 2] > math.cos(math.radians(25)):
                    if not quiet:
                        self.say("could only raise the arm part way straight up: drive it up by hand, then PICK "
                                 "again", 20.0)
                    return False
        for _ in range(40):
            z = kin.tool_point(q, mm)[2]
            if z >= need:
                return True
            goal = kin.pose(q) + np.array([0.0, 0.0, min(0.01, need - z + 0.002), 0.0])
            q_new, ok = kin.ik(goal, q, lo, hi)
            if not ok:
                q_new, ok = kin.ik(goal, q, lo, hi, pitch_w=0.001)
            if not ok:
                if not quiet:
                    self.say("could not raise the arm straight up from here: drive it up by hand, then PICK again",
                             20.0)
                return False
            if not self.move_to(q_new, "MOVING"):
                return False
            q = self.arm_q()
        return kin.tool_point(q, mm)[2] >= need

    def go_rest(self):
        """handeye cmd_pick.go_rest: refused with the magnet on; lifts straight up first if well
        below the rest height, then a joint move to rest."""
        st, kin, mm = self.st, self.kin, self.hc.magnet_m
        rest_q = st["rest_q"]
        if rest_q is None:
            self.say("no rest pose saved: put the arm there, then Save as rest pose", 20.0)
            return False
        if self.mag.on:
            self.say("magnet is ON: release before going to rest", 20.0)
            return False
        if not st["arm_held"]:
            self.hold(True)
        q = self.arm_q()
        z_rest = float(kin.tool_point(rest_q, mm)[2])
        if kin.tool_point(q, mm)[2] < z_rest - 0.02 and np.max(np.abs(q - rest_q)) > math.radians(5):
            if not self.raise_to(z_rest, quiet=True) and st["flag"] == "ABORTED":
                return False
        if not self.move_to(rest_q, "MOVING"):
            return False
        st["state"], st["plan"], st["flag"] = "lock", None, "HELD"
        self.say("at rest. PICK when the package is in place", 20.0)
        return True

    def descend_and_pick(self):
        """handeye cmd_pick.descend_and_pick: down to CONTACT_ABOVE_M, a gentle press at reduced
        torque; the board must confirm ON before the arm goes lower and again after the press, or
        it doesn't lift. Then freeze, full torque, break away and lift. An abort never switches
        the magnet off."""
        st, cfg, kin, mm = self.st, self.cfg, self.kin, self.hc.magnet_m
        h_on = cfg.magnet_on_mm / 1000.0
        press_mm = cfg.side_press_mm if st["face"] == "side" else cfg.press_mm
        if st["face"] == "top" and st.get("pitch_app") is not None and st["pitch_app"] != st["pitch_top"]:
            down = self.straight_blend(st["hover_m"], CONTACT_ABOVE_M, 0.005)
            press = self.straight_blend(CONTACT_ABOVE_M, -press_mm / 1000.0, 0.004)
        else:
            down = self.straight(st["hover_m"], CONTACT_ABOVE_M, 0.01)
            press = self.straight(CONTACT_ABOVE_M, -press_mm / 1000.0, 0.004)
        if down is None or press is None:
            self.say(f"the approach to the {st['face']} face isn't solvable from here", 1e9)
            return
        on = False
        down_a = [(z, q) for z, q in down if z > h_on + 1e-9]
        down_b = [(z, q) for z, q in down if z <= h_on + 1e-9]
        press_a = [(z, q) for z, q in press if z > h_on + 1e-9]
        press_b = [(z, q) for z, q in press if z <= h_on + 1e-9]

        def switch_on():
            # handeye: the ESP32 resets now and then as the coil switches on: one more ON then
            for attempt in (1, 2):
                if self.magnet_set(True) is True:
                    return True
                if attempt == 1:
                    self.say("the magnet did not confirm ON (board reset?): trying once more", 1e9)
                    self.clock.sleep(0.3)
            self.say("the magnet did NOT confirm ON: stopped here. Check the board, then Back up", 1e9)
            st["flag"] = "ABORTED"
            return False

        if not self.move_path(down_a, "DESCENDING", DESCEND_MPS):
            return
        if down_b:
            if not switch_on():
                return
            on = True
            if not self.move_path(down_b, "DESCENDING", DESCEND_MPS):
                return
        self.torque_limit(cfg.press_torque)
        # compliant wrist through the press (handeye: the edge's contact force turns it flat)
        self.io.set_torque_limit("wrist_flex", cfg.wrist_torque)
        try:
            if not self.move_path(press_a, "PRESSING", PRESS_MPS):
                return
            if press_b and not on:
                if not switch_on():
                    return
                on = True
            if not self.move_path(press_b, "PRESSING", PRESS_MPS):
                return
            st["flag"] = "PRESSING"
            if not on and not switch_on():
                return
            self.clock.sleep(GRIP_WAIT_S)
            if self.mag.poll() is not True:
                self.say("magnet went OFF during the press: switching it on again", 1e9)
                if self.magnet_set(True) is True:
                    self.clock.sleep(GRIP_WAIT_S)
            if self.mag.poll() is not True:
                self.say("the magnet is not ON after the press (board): not lifting. Check the board, then Back up",
                         1e9)
                st["flag"] = "ABORTED"
                return
        finally:
            self.freeze()
            self.torque_limit(FULL_TORQUE)
        q_seat = self.arm_q()
        p_seat = kin.tool_point(q_seat, mm)
        z_now = float((p_seat - st["locked"]) @ st["axis"])
        settled = self.model_pitch(q_seat)
        planned = cfg.side_pitch_deg if st["face"] == "side" else (st["pitch_top"] or cfg.pitch_deg)
        log.info("seated (%s face): magnet pitch %+.1f deg in the model (planned %+.1f), reach %.1f cm, height "
                 "%.1f cm", st["face"], settled, planned, np.linalg.norm(p_seat[:2]) * 100, p_seat[2] * 100)
        pull = None
        # never while unloading: pulling toward the robot from the body drags the box into the mast
        if (st["face"] == "top" and cfg.pull_in_cm and not self.unloading and
                abs((st["pitch_top"] or cfg.pitch_deg) - cfg.pitch_deg) >= cfg.pull_in_off_flat_deg):
            pull = self.pull_in_path(p_seat, cfg.pull_in_cm / 100.0, settled)
        if pull:
            log.info("landed %+.0f deg off flat: pulling the package in %.0f cm before the lift",
                     (st["pitch_top"] or cfg.pitch_deg) - cfg.pitch_deg, pull[-1][0] * 100)
            if not self.move_path(pull, "LIFTING", PULL_MPS):
                self.say("ABORTED during the pull-in: holding, magnet still ON", 1e9)
                st["state"] = "holding"
                return
            q_seat = self.arm_q()
            p_seat = kin.tool_point(q_seat, mm)
            settled = self.model_pitch(q_seat)
        st["lift_tilt_rad"] = 0.0
        if st["face"] == "top" and cfg.lift_tilt_deg and not pull:
            pre = self.straight(z_now, z_now + LIFT_PRE_M, 0.005, pitch=settled)
            if pre and not self.move_path(pre, "LIFTING", PRESS_MPS * 2):
                self.say("ABORTED during the lift: holding, magnet still ON", 1e9)
                st["state"] = "holding"
                return
            # "wrist up" in joint terms, probed at the zero pose as the pad and page buttons do
            ref = np.zeros(5)
            probe = ref.copy()
            probe[3] += 1e-3
            up_sign = 1.0 if kin.pose(probe)[3] > kin.pose(ref)[3] else -1.0
            q_t = self.arm_q()
            w0 = q_t[3]
            q_t[3] = float(np.clip(q_t[3] + up_sign * math.radians(cfg.lift_tilt_deg), self.lo[3], self.hi[3]))
            if not self.move_to(q_t, "LIFTING", dps=20.0):
                self.say("ABORTED during the lift: holding, magnet still ON", 1e9)
                st["state"] = "holding"
                return
            # what the joint limit let it turn, so do_place undoes exactly this much
            st["lift_tilt_rad"] = float(q_t[3] - w0)
            q_seat = self.arm_q()
            z_now = float((kin.tool_point(q_seat, mm) - st["locked"]) @ st["axis"])
            settled = self.model_pitch(q_seat)
        if st["face"] == "top" and pull:
            up = None
            for top_h in (HIGH_M,) + TOP_HIGH_CANDS_M[1:] + (0.04, 0.03):
                for p_ in (settled, settled - 6.0, settled - 12.0, settled - 18.0):
                    up = self.lift_path(p_seat, top_h - z_now, p_)
                    if up:
                        break
                if up:
                    break
        elif st["face"] == "top":
            up = None
            for top_h in (HIGH_M,) + TOP_HIGH_CANDS_M[1:] + (0.04, 0.03):
                for p_ in (settled, None, (st["pitch_top"] or cfg.pitch_deg) - 6.0):
                    up = self.straight(z_now, top_h, 0.01, pitch=p_)
                    if up:
                        break
                if up:
                    break
        else:
            up = (self.lift_path(p_seat, SIDE_LIFT_M, settled)
                  or self.lift_path(p_seat, SIDE_LIFT_M, cfg.side_pitch_deg))
        if up is None:
            self.say("magnet ON and holding, but the lift isn't solvable straight up: drive it up by hand", 1e9)
            st["state"], st["flag"] = "holding", "HOLDING"
            return
        if not self.move_path(up, "LIFTING", LIFT_MPS):
            self.say("ABORTED during the lift: holding, magnet still ON", 1e9)
            st["state"] = "holding"
            return
        st["state"], st["flag"] = "holding", "HOLDING"
        self.say("HOLDING: magnet on, lifted", 1e9)

    def drive_base(self, dist_m):
        """handeye cmd_pick.drive_base: True if the base ran the whole way."""
        self.st["flag"] = "MOVING"
        self.publish()
        ok = bool(self.base.move(dist_m, BASE_SPEED_MPS))
        if not ok:
            self.st["flag"] = "ABORTED"
            self.say("ABORTED: base stopped", 1e9)
        return ok

    def do_place(self, q_pl=None, base_back=True):
        """handeye cmd_pick.do_place: back the base off, via rest to a few cm above the place pose
        (its tilt kept), straight down, magnet off, rest, base back. True if released there.
        q_pl / base_back: unload() puts it on the table pose, the base where it is."""
        st, cfg, kin, mm = self.st, self.cfg, self.kin, self.hc.magnet_m
        q_pl = st["place_q"] if q_pl is None else q_pl
        tool = kin.tool_point(q_pl, mm)
        axis = kin.flange(q_pl)[:3, 2]
        lo_f, hi_f = self.lo - math.radians(2.5), self.hi + math.radians(2.5)
        path = None
        for h in (0.06, 0.04, 0.02):
            pts, seed, ok = [], [q_pl], True
            for hh in np.arange(0.0, h + 1e-9, 0.01)[::-1]:
                r = self._ik(kin, tool + [0, 0, hh], seed, lo_f, hi_f, mm, float(q_pl[4]),
                             tilt_tol_deg=3.0, axis=axis)
                if r is None:
                    ok = False
                    break
                pts.append((hh, r[0]))
                seed = [r[0]]
            if ok:
                path = pts
                break
        backed = False
        if base_back and self.base is not None and cfg.base_back_cm:
            if not self.drive_base(-cfg.base_back_cm / 100.0):
                return False
            backed = True
        # The lift's wrist tilt, turned back the same amount once clear of the table and before
        # the swing to rest (2026-11-20: asked for on the real robot).
        if st["lift_tilt_rad"]:
            q_u = self.arm_q()
            q_u[3] = float(np.clip(q_u[3] - st["lift_tilt_rad"], self.lo[3], self.hi[3]))
            if not self.move_to(q_u, "MOVING", dps=20.0):
                return False
            st["lift_tilt_rad"] = 0.0
        if st["rest_q"] is not None and not self.move_to(st["rest_q"], "MOVING"):
            return False
        if not path:
            # the angle can't be kept: the highest pose straight above with it free, then a short
            # joint move down; none at all: refuse, never a blind joint swing onto the spot (handeye)
            def res(x, hh):
                qq = np.array([*x, q_pl[4]])
                pen = np.concatenate([np.maximum(0, lo_f[:4] - x), np.maximum(0, x - hi_f[:4])]) * 5
                return np.concatenate([kin.tool_point(qq, mm) - (tool + [0, 0, hh]), 0.01 * (x - q_pl[:4]), pen])
            for h in (0.06, 0.04, 0.03, 0.02, 0.015):
                x, _ = head_cam._lm(lambda x_, hh=h: res(x_, hh), q_pl[:4].copy(), iters=200)
                q_pre = np.array([*x, q_pl[4]])
                if (np.all(q_pre >= lo_f) and np.all(q_pre <= hi_f) and
                        np.linalg.norm(kin.tool_point(q_pre, mm) - tool - [0, 0, h]) < 0.003):
                    path = [(h, q_pre)]
                    break
        if not path:
            self.say("can't approach the place pose from above (it is at the arm's joint limits): drive the "
                     "package there and Release by hand, or teach a place away from the limits", 1e9)
            return False
        if path[0][0] < 0.04:
            log.info("place: only %.1f cm of room above the place pose (joint limits)", path[0][0] * 100)
        if not self.move_to(path[0][1], "MOVING") or not self.move_path(path[1:] + [(0.0, q_pl)], "DESCENDING",
                                                                        DESCEND_MPS):
            return False
        if self.magnet_set(False) is not False:
            self.say("placed, but the magnet did not confirm OFF: check it", 1e9)
            st["flag"] = "ABORTED"
            return False
        self.clock.sleep(0.3)
        st["state"], st["flag"] = "lock", "HELD"
        self.say("placed: back to rest", 8.0)
        self.go_rest()
        if backed:
            self.drive_base(cfg.base_back_cm / 100.0)
            st["flag"] = "HELD"
        return True

    def lock_with_step_aside(self):
        """handeye cmd_pick.lock_with_step_aside: lock; if not seen, swing shoulder_pan
        LOCK_ASIDE_DEG one way, then the other, and look again from each. handeye drops keys that
        came in while looking (a queued second PICK press); an abort Event isn't a key queue, so
        here an abort pressed while looking stays set and stops the next move."""
        st = self.st
        st["flag"] = "LOOKING"
        self.publish()
        if self.do_lock():
            return True
        q0 = self.arm_q()
        for d in (LOCK_ASIDE_DEG, -LOCK_ASIDE_DEG):
            q = q0.copy()
            q[0] = float(np.clip(q0[0] + math.radians(d), self.lo[0], self.hi[0]))
            if abs(q[0] - q0[0]) < math.radians(10):
                continue  # against the joint limit: no room to step this way
            self.say(f"marker hidden from here: stepping aside {d:+.0f} deg to look again", 1e9)
            if not self.move_to(q, "MOVING"):
                return False
            st["flag"] = "LOOKING"
            self.publish()
            if self.do_lock():
                return True
        st["flag"] = "HELD"
        if st.get("lock_why", "unseen") == "unseen":
            self.say(f"marker {self.marker_id} not seen from rest or from either side: is it in view, and not "
                     "covered by the arm?", 20.0)
        elif st["lock_why"] != "face":
            self.say(f"marker {self.marker_id} seen, but its position wouldn't hold still ({st['lock_why']}): "
                     "glare, or too far/steep for the camera. Move the camera closer to it, or try again", 20.0)
        return False

    def move_head(self, pan, tilt):
        """handeye cmd_pick.move_head: ramp the head at SEARCH_DPS, clipped; abort stops it (the
        arm isn't touched). True if it got there."""
        lim, hg = self.head_lim, self.head_goal
        goal = {"head_pan": min(lim["head_pan"][1], max(lim["head_pan"][0], pan)),
                "head_tilt": min(lim["head_tilt"][1], max(lim["head_tilt"][0], tilt))}
        start = dict(hg)
        dur = max(0.2, max(abs(goal[m] - start[m]) for m in goal) / SEARCH_DPS)
        t0 = self.clock.monotonic()
        while True:
            s_ = min(1.0, (self.clock.monotonic() - t0) / dur)
            for m in goal:
                hg[m] = start[m] + (goal[m] - start[m]) * s_
            self.io.write(dict(hg))
            if self.abort.is_set():
                self.st["flag"] = "ABORTED"
                self.say("ABORTED: search stopped", 1e9)
                return False
            self.mag.rearm_if_due()
            self.publish()
            if s_ >= 1.0:
                self.clock.sleep(SEARCH_SETTLE_S)
                return True
            self.clock.sleep(0.02)

    def aim_head_at(self, p_base):
        """handeye cmd_pick.aim_head_at: head angles putting p_base on the optical axis, by the
        calibrated model, coarse then fine grid inside the head limits."""
        hc, hg = self.hc, self.head_goal

        def miss(pan, tilt):
            T = hc.matrix(pan, tilt)
            d = p_base - T[:3, 3]
            return 1.0 - float(T[:3, 2] @ d) / float(np.linalg.norm(d))
        (plo, phi), (tlo, thi) = self.head_lim["head_pan"], self.head_lim["head_tilt"]
        best = (miss(hg["head_pan"], hg["head_tilt"]), hg["head_pan"], hg["head_tilt"])
        for step, span in ((2.0, 60.0), (0.25, 3.0)):
            c_p, c_t = best[1], best[2]
            for pan in np.arange(max(plo, c_p - span), min(phi, c_p + span) + 1e-9, step):
                for tilt in np.arange(max(tlo, c_t - span), min(thi, c_t + span) + 1e-9, step):
                    e = miss(pan, tilt)
                    if e < best[0]:
                        best = (e, float(pan), float(tilt))
        return best[1], best[2]

    def find_marker(self):
        """handeye cmd_pick.find_marker: look; if unseen, sweep the head; then centre the marker."""
        st, lim, hg = self.st, self.head_lim, self.head_goal
        p0, t0_ = hg["head_pan"], hg["head_tilt"]
        seen = self.entries(self.marker_id, n=3, timeout=1.5, give_up_s=1.0)
        if not seen:
            st["flag"] = "SEARCHING"
            self.say(f"marker {self.marker_id} not in view: searching with the head", 1e9)
            tried = set()
            for dt in SEARCH_TILT_STEPS:
                for dp in SEARCH_PAN_STEPS:
                    pan = min(lim["head_pan"][1], max(lim["head_pan"][0], p0 + dp))
                    tilt = min(lim["head_tilt"][1], max(lim["head_tilt"][0], t0_ + dt))
                    key = (round(pan), round(tilt))
                    if key in tried or (dp == 0 and dt == 0):
                        continue
                    tried.add(key)
                    if not self.move_head(pan, tilt):
                        return False
                    seen = self.entries(self.marker_id, n=3, timeout=1.2, give_up_s=0.8)
                    if seen:
                        break
                if seen:
                    break
            if not seen:
                self.say(f"marker {self.marker_id} not found by the head search: trying with the arm stepped aside",
                         1e9)
                ok = self.move_head(p0, t0_)
                st["flag"] = "HELD" if ok else "ABORTED"
                return ok
        hp = self.head_now()
        p_base = (self.hc.matrix(hp["head_pan"], hp["head_tilt"])
                  @ np.append(np.median([m["target_pt"] for m in seen], 0), 1.0))[:3]
        pan, tilt = self.aim_head_at(p_base)
        if max(abs(pan - hg["head_pan"]), abs(tilt - hg["head_tilt"])) > 2.0:
            st["flag"] = "SEARCHING"
            if not self.move_head(pan, tilt):
                return False
        st["flag"] = "HELD"
        self.say(f"marker {self.marker_id} in view, head {hg['head_pan']:+.0f}/{hg['head_tilt']:+.0f}", 8.0)
        return True

    # ---- the public entry points ------------------------------------------------------------
    def start(self, head=None, ramp_s=2.0):
        """cmd_pick's start-up (1494-1498): the head held and ramped to `head` (default cfg.head;
        handeye hold_head_at, no snap of the mast camera), then the press joints back to full
        torque (a crashed press can leave the RAM limit low)."""
        pan, tilt = self.cfg.head if head is None else head
        lim = self.head_lim
        goal = {"head_pan": min(lim["head_pan"][1], max(lim["head_pan"][0], pan)),
                "head_tilt": min(lim["head_tilt"][1], max(lim["head_tilt"][0], tilt))}
        self.io.hold(HEAD)
        start = self.head_now()
        n = max(1, int(ramp_s / 0.05))
        for k in range(1, n + 1):
            self.io.write({m: start[m] + (goal[m] - start[m]) * k / n for m in goal})
            self.clock.sleep(0.05)
        self.clock.sleep(0.5)
        self.head_goal = self.head_now()
        for j in PRESS_JOINTS:
            self.io.set_torque_limit(j, FULL_TORQUE)

    def auto_pick(self, marker_id=10, place=True):
        """handeye cmd_pick.auto_pick: hold, rest, find the marker, lock (stepping aside), plan,
        raise clear if not from rest, GO, descend and pick, place if a place pose is set. The
        first refusal or abort stops the chain where it is, the reason in a report.
        -> "placed" | "picked" (holding it, no place pose or the place refused) | "failed" | "aborted".
        `abort` is cleared on entry: a stop pressed before this pick doesn't cancel it.
        place=False stops once it holds the package: the operator drives it to a new place spot
        by hand and saves that as the place pose (the operator view's Pick & hold)."""
        st = self.st
        self.marker_id = marker_id
        self.place_after = place
        self.abort.clear()
        if not self.mag.magnet:
            self.say("no magnet board: plug in the ESP32 and restart", 20.0)
            return "failed"
        # handeye only takes PICK in its "lock" state; there, driving the arm by hand also put it
        # back to "lock". Nothing here tracks that driving, so: refused only while holding.
        if st["state"] == "holding":
            self.say("holding a package (magnet ON): place or release it before the next PICK", 20.0)
            return "failed"
        st["state"], st["plan"] = "lock", None
        # the head may have been moved by something else since the last pick: start from where it is
        self.head_goal = self.head_now()
        result = self._chain()
        if self.abort.is_set():
            return "aborted"
        return result

    def unload(self, marker_id=10):
        """The operator view's Unload: the package the place pose left in the body, out onto the
        table. The head aimed at the place pose's magnet spot, then the pick's own lock, plan and
        press (no pull-in), then the taught table pose with the base where it is. No table pose:
        it holds the package for the pad, and Save table keeps where it's driven to. Not in
        handeye. -> "placed" | "picked" | "failed" | "aborted"."""
        st = self.st
        if st["place_q"] is None:
            self.say("no place pose saved: Unload looks where the place pose leaves the package", 20.0)
            return "failed"
        if not self.mag.magnet:
            self.say("no magnet board: plug in the ESP32 and restart", 20.0)
            return "failed"
        if st["state"] == "holding":
            self.say("holding a package (magnet ON): place or release it before Unload", 20.0)
            return "failed"
        self.marker_id = marker_id
        self.place_after = True
        self.abort.clear()
        st["state"], st["plan"] = "lock", None
        self.head_goal = self.head_now()
        self.unloading = True
        try:
            result = self._chain(unload=True)
        finally:
            self.unloading = False
        if self.abort.is_set():
            return "aborted"
        if result == "picked" and st["table_q"] is None:
            self.say("holding it: drive the arm over the table with the pad, then Save table", 1e9)
        return result

    def _chain(self, unload=False):
        st = self.st
        if not st["arm_held"]:
            self.hold(True)
        from_rest = st["rest_q"] is not None
        if from_rest and np.max(np.abs(self.arm_q() - st["rest_q"])) > math.radians(3):
            if not self.go_rest():
                return "failed"
        # Unload: the package sits where the place pose's magnet left it; look there first
        if unload:
            st["flag"] = "SEARCHING"
            if not self.move_head(*self.aim_head_at(self.kin.tool_point(st["place_q"], self.hc.magnet_m))):
                return "failed"
        if not self.find_marker() or not self.lock_with_step_aside() or not self.do_plan():
            return "failed"
        # from rest the joint move starts above and behind: no raise first (handeye, 2026-11-15)
        if st["face"] == "top" and not from_rest and not self.raise_clear():
            return "failed"
        if not self.do_go():
            return "failed"
        self.descend_and_pick()
        q_pl = st["table_q"] if unload else st["place_q"]
        if st["state"] == "holding" and q_pl is not None and getattr(self, "place_after", True):
            if self.do_place(q_pl, base_back=not unload):
                return "placed"
        return "picked" if st["state"] == "holding" else "failed"

    def place(self):
        """handeye's Place button (n): only while holding, with a place pose."""
        if self.st["state"] != "holding" or self.st["place_q"] is None:
            self.say("nothing held, or no place pose saved", 20.0)
            return False
        self.abort.clear()
        return self.do_place()
