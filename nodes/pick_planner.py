#!/usr/bin/env python3
"""The pick-and-place decisions, with no ROS: nodes/magnet_pick.py is only the wiring.

One request at a time, as a state machine ticked with what the robot knows:

    attach:  tuck -> acquire -> dock -> pre -> approach -> energize -> lift -> carry -> backout
    detach:  tuck -> dock -> pre -> approach (lower) -> release -> retreat -> stow -> backout

and each tick returns what to do about it (arm joints, magnet on/off, base velocity,
and finally `picked` / `placed` / `failed`). Everything that can be wrong about a pick
lives here, which is why it is a plain class a test can drive in a closed loop against
simtest.magnet.MagnetSim and nodes/arm_kin.py (tests/pick_planner_test.py).

THE GEOMETRY, in one paragraph. The arm moves in a single vertical plane through its
shoulder, so the magnet can only arrive along the line from the shoulder to the plate.
To pick a plate that faces any way, the BASE has to come round to the plate's side:
simtest.magnet.dock_pose says where. The pick therefore is: find the plate (a camera
detection, or ground truth in oracle mode), compute the dock pose, drive there (staging
in front of the plate first, so the last stretch is a straight push in), then solve the
arm for a pre-approach pose and a contact pose (arm_kin.plan_reach), energize the magnet
at contact, and withdraw. A place is the same thing against a VIRTUAL plate: where the
held box's plate would be at its resting pose.

WHAT IT KNOWS. `base` (x, y, yaw) and `plates` come in the same frame, the map, from the
same TF chain at the same instant, so a drifting SLAM map moves both together and the
dock is still right relative to the plate. `held` is "the magnet is holding something":
the simulator's ground truth today, the microswitch on the real arm. There is no
/joint_states, so the arm's own state is its last command, as nodes/scripted_pick.py
does it; every arm move is a ramp from there.

DEADLINES are sim seconds from `obs.now` (the arm and base move in sim time), never the
wall clock.

THE SAFE POSE. The sim boots with the arm straight out in front at deck height, which
would put its tip inside the box at the dock. So every request starts by tucking: the arm
folded upright behind the shoulder (`tuck_q`, chosen by sampling the joint space for the
pose whose every link stays behind the shoulder). Driving and carrying happen tucked.
"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from typing import Callable

from simtest import magnet

# Every key is documented in configs/magnet_pick.yaml, which may override any of them.
DEFAULTS = {
    "tuck_q": [0.0, 1.95, -0.05, 0.0, 0.0, 0.0],
    "arm_speed_rad_s": 1.0,
    "arm_min_move_s": 0.8,
    "arm_max_move_s": 4.0,
    "settle_s": 0.4,
    "arm_tol_rad": 0.02,
    "arm_timeout_s": 2.0,
    "reach_m": magnet.REACH_NOM_M,
    "stage_m": 0.35,
    "push_m": 0.008,
    "capture_timeout_s": 2.5,
    "release_timeout_s": 2.5,
    "pose_wait_s": 15.0,
    "budget_s": 300.0,
    "dock": {"pos_tol_m": 0.02, "yaw_tol_rad": 0.05, "v_max": 0.30, "v_final": 0.12,
             "wz_max": 0.6, "kp": 1.2, "kyaw": 1.5, "timeout_s": 45.0,
             "skip_within_m": 0.55, "skip_lateral_m": 0.06},
    "confirm": {"samples": 5, "timeout_s": 3.0, "max_redocks": 2, "redock_m": 0.015},
    "acquire": {"assoc_m": 0.5, "box_assoc_m": 0.25, "min_conf": 0.2, "max_age_s": 1.0, "stable_ticks": 2,
                "view_m": 2.6, "scan_rate": 0.5, "orbit_m": 1.5, "orbit_steps": 8,
                "orbit_settle_s": 1.5, "timeout_s": 120.0},
    "update": {"track_m": 0.25, "min_dist_m": 0.9, "max_age_s": 0.35,
               "image": [848, 480], "border_px": 6},
}


def merged(cfg: dict | None) -> dict:
    out = copy.deepcopy(DEFAULTS)
    for k, v in (cfg or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k].update(v)
        else:
            out[k] = v
    return out


@dataclass
class Obs:
    now: float                              # sim seconds
    base: tuple | None                      # (x, y, yaw), the frame the plates are in
    plates: list = field(default_factory=list)   # {x,y,z,nx,ny,nz,conf,age_s?,px?}
    held: bool = False                      # the magnet is holding a package
    q: list | None = None                   # the arm's measured joints, None if nothing reports them
    boxes: list = field(default_factory=list)    # from depth geometry: {x, y, z, size, ...}


@dataclass
class Decision:
    q: list | None = None                   # arm joint command (6), None = nothing new
    pick_cmd: str | None = None             # "attach" | "detach" to publish this tick
    vel: tuple | None = None                # base (vx, vy, wz) body frame; None = hands off
    result: str | None = None               # "picked" | "placed" | "failed"
    phase: str = "idle"
    note: str = ""                          # why, for failures and the status stream


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _smooth(f: float) -> float:
    return f * f * (3.0 - 2.0 * f)


def _hnorm(n) -> tuple[float, float] | None:
    h = math.hypot(n[0], n[1])
    return None if h < 1e-3 else (n[0] / h, n[1] / h)


class PickPlanner:
    def __init__(self, kin, cfg: dict | None = None, *,
                 plan_reach: Callable | None = None):
        import arm_kin

        self.kin = kin
        self.cfg = merged(cfg)
        self._plan_reach = plan_reach or arm_kin.plan_reach
        self.q_cmd = [0.0] * 6              # the sim boots with every arm joint at 0
        self.ramp: dict | None = None
        self.arm_end = 0.0                  # sim time the latest ramp ends
        self.pending = "failed"             # the result to report once the base has backed out
        self.t_nopose: float | None = None  # when the robot's pose first went missing
        self.req: dict | None = None
        self.mode: str | None = None
        self.phase = "idle"
        self.t_req = self.t_phase = 0.0
        self.target: dict | None = None     # {c, n}: the plate being aimed at (frame of `base`)
        self.plan: dict | None = None
        self.dock: tuple | None = None
        self.sub = "stage"
        self.acq: dict = {}
        self.fixes: list = []               # fresh plate fixes gathered at the dock
        self.redocks = 0
        self.arm_err: float | None = None   # worst joint error last seen, radians
        self.dock_shift: float | None = None  # how far the fix at the dock moved the target
        self._stop_sent = True
        self._seen = 0

    # -- the request ------------------------------------------------------------------
    @property
    def busy(self) -> bool:
        return self.req is not None

    def request(self, req: dict, now: float) -> None:
        """Start an attach (pick) or detach (place). A request while busy replaces the
        old one (the controller aborted it: task_manager only asks one at a time)."""
        self.req = dict(req)
        self.mode = req["cmd"]
        self.t_req = now
        self.target = self.plan = self.dock = None
        self.acq = {}
        self.fixes, self.redocks, self.dock_shift = [], 0, None
        self._seen = 0
        self._stop_sent = False
        self._enter("tuck", now)
        self._arm_to(self.cfg["tuck_q"], now)

    def cancel(self, now: float) -> Decision:
        """Abort whatever is running: base stopped, magnet off, arm tucked."""
        d = Decision(phase="cancelled", vel=(0.0, 0.0, 0.0), pick_cmd="detach")
        if self.req is not None:
            self._arm_to(self.cfg["tuck_q"], now)
        self.req = None
        self.phase = "idle"
        return d

    def _enter(self, phase: str, now: float) -> None:
        self.phase, self.t_phase = phase, now

    # -- the arm: ramps from the last command -------------------------------------------
    def _arm_to(self, q, now: float, dur: float | None = None) -> None:
        q0 = list(self.q_cmd)
        q1 = [float(v) for v in q]
        if dur is None:
            far = max(abs(a - b) for a, b in zip(q0, q1))
            dur = min(self.cfg["arm_max_move_s"],
                      max(self.cfg["arm_min_move_s"], far / self.cfg["arm_speed_rad_s"]))
        self.ramp = {"q0": q0, "q1": q1, "t0": now, "dur": dur}
        self.arm_end = now + dur

    def _arm_done(self, now: float) -> bool:
        """The commanded ramp has finished."""
        return now >= self.arm_end

    def _arm_ready(self, obs: Obs) -> bool:
        """... and the joints have got there. With measured joints that is what is checked,
        up to arm_timeout_s past the ramp's end; without them (nothing reports them on
        this platform) a settle_s dwell stands in for it. Wrist roll and the jaw are left
        out of the comparison only in that neither carries the TCP's position."""
        if obs.q is None:
            return obs.now >= self.arm_end + self.cfg["settle_s"]
        self.arm_err = max(abs(a - b) for a, b in zip(obs.q[:4], self.q_cmd[:4]))
        if obs.now < self.arm_end:
            return False
        return self.arm_err <= self.cfg["arm_tol_rad"] or obs.now >= self.arm_end + self.cfg["arm_timeout_s"]

    def _ramp_q(self, now: float) -> list | None:
        r = self.ramp
        if r is None:
            return None
        f = min(1.0, max(0.0, (now - r["t0"]) / r["dur"])) if r["dur"] > 0 else 1.0
        s = _smooth(f)
        self.q_cmd = [a + (b - a) * s for a, b in zip(r["q0"], r["q1"])]
        if f >= 1.0:
            self.ramp = None
        return list(self.q_cmd)

    # -- the base ------------------------------------------------------------------------
    def _drive(self, base, goal_xy, goal_yaw, *, v_max, pos_tol, yaw_tol) -> tuple[tuple, bool]:
        """A proportional world-frame move to a pose, as a body-frame velocity. The base
        is holonomic (cmd_vel carries linear.y), so heading and position are corrected
        together. Returns (vel, arrived)."""
        d = self.cfg["dock"]
        ex, ey = goal_xy[0] - base[0], goal_xy[1] - base[1]
        dist = math.hypot(ex, ey)
        eyaw = _wrap(goal_yaw - base[2])
        c, s = math.cos(base[2]), math.sin(base[2])
        speed = min(v_max, d["kp"] * dist)
        if dist > 1e-6:
            wx, wy = ex / dist * speed, ey / dist * speed
        else:
            wx = wy = 0.0
        vx, vy = c * wx + s * wy, -s * wx + c * wy
        wz = max(-d["wz_max"], min(d["wz_max"], d["kyaw"] * eyaw))
        return (vx, vy, wz), (dist <= pos_tol and abs(eyaw) <= yaw_tol)

    # -- the tick ------------------------------------------------------------------------
    def tick(self, obs: Obs) -> Decision:
        d = Decision(phase=self.phase)
        if self.req is not None:
            if obs.now - self.t_req > self.cfg["budget_s"]:
                self._fail(d, obs, "budget exceeded")
            else:
                getattr(self, "_ph_" + self.phase)(obs, d)
            d.phase = self.phase
        d.q = self._ramp_q(obs.now)
        return d

    def _no_pose(self, obs: Obs, d: Decision) -> bool:
        """True while the robot's own pose is unknown. TF can lag the first seconds of a
        run (SLAM has not published map -> odom yet), so wait up to pose_wait_s with the
        base held still, and only then give up."""
        if obs.base is not None:
            self.t_nopose = None
            return False
        if self.t_nopose is None:
            self.t_nopose = obs.now
        d.vel = (0.0, 0.0, 0.0)
        if obs.now - self.t_nopose > self.cfg["pose_wait_s"]:
            self._fail(d, obs, "no robot pose")
        return True

    def _fail(self, d: Decision, obs: Obs, why: str) -> None:
        d.result, d.note, d.pick_cmd = "failed", why, "detach"
        d.vel = (0.0, 0.0, 0.0)
        self._arm_to(self.cfg["tuck_q"], obs.now)
        self.req = None
        self.phase = "idle"

    def _finish(self, d: Decision, obs: Obs, result: str) -> None:
        d.result = result
        d.vel = (0.0, 0.0, 0.0)                 # the last word to the base: stop, and stay stopped
        self.req = None
        self.phase = "idle"

    # -- phases ------------------------------------------------------------------------------
    def _ph_tuck(self, obs: Obs, d: Decision) -> None:
        d.vel = (0.0, 0.0, 0.0)
        if not self._arm_done(obs.now):
            return
        if self.mode == "attach":
            self.acq = {"t": obs.now, "sub": "look", "angle": 0.0, "last_yaw": None,
                        "orbit": 0, "t_wp": obs.now}
            self._enter("acquire", obs.now)
        elif self.mode == "detach":
            if not obs.held:
                return self._fail(d, obs, "asked to place with nothing held")
            self._begin_place(obs, d)
        else:
            self._fail(d, obs, f"unknown request {self.mode!r}")

    # .. acquire (attach only) ..............................................................
    def _hint(self, obs: Obs) -> tuple[float, float] | None:
        """Where to look: the work order's location if it gives one, else the box depth
        geometry found nearest the robot, else nowhere yet."""
        at = self.req.get("at")
        if at:
            return (at["x"], at["y"])
        if not obs.boxes or obs.base is None:
            return None
        b = min(obs.boxes, key=lambda b: math.hypot(b["x"] - obs.base[0], b["y"] - obs.base[1]))
        return (b["x"], b["y"])

    def _candidate(self, obs: Obs):
        a, hint = self.cfg["acquire"], self._hint(obs)
        if hint is None:
            return None
        # a plate is ON the box geometry found, so that match can be much tighter
        reach = a["assoc_m"] if self.req.get("at") else a["box_assoc_m"]
        best = None
        for p in obs.plates:
            if p.get("conf", 1.0) < a["min_conf"] or p.get("age_s", 0.0) > a["max_age_s"]:
                continue
            dist = math.hypot(p["x"] - hint[0], p["y"] - hint[1])
            if dist <= reach and (best is None or dist < best[0]):
                best = (dist, p)
        return best[1] if best else None

    def _ph_acquire(self, obs: Obs, d: Decision) -> None:
        a = self.cfg["acquire"]
        if self._no_pose(obs, d):
            return
        d.vel = (0.0, 0.0, 0.0)
        cand = self._candidate(obs)
        self._seen = self._seen + 1 if cand else 0
        if cand and self._seen >= a["stable_ticks"]:
            return self._lock(obs, d, cand)
        if obs.now - self.acq["t"] > a["timeout_s"]:
            return self._fail(d, obs, "plate not found")
        bx, by, byaw = obs.base
        hint = self._hint(obs) or (bx, by)      # nothing to go on: look round from here
        bearing = math.atan2(hint[1] - by, hint[0] - bx)
        dist = math.hypot(hint[0] - bx, hint[1] - by)
        sub = self.acq["sub"]
        if sub == "look":
            # face where the work order says the package is: most of the time that is
            # all the search there is
            if dist < 0.4:
                self.acq["sub"] = "scan"
                return
            vel, ok = self._drive(obs.base, (bx, by), bearing, v_max=0.0, pos_tol=1.0, yaw_tol=0.12)
            d.vel = vel
            if ok:
                self.acq["sub"], self.acq["t_sub"] = "approach", obs.now
        elif sub == "approach":
            if dist <= a["view_m"]:
                self.acq["sub"], self.acq["t_sub"] = "dwell", obs.now
                return
            gx, gy = (hint[0] - math.cos(bearing) * a["view_m"],
                      hint[1] - math.sin(bearing) * a["view_m"])
            d.vel, _ = self._drive(obs.base, (gx, gy), bearing, v_max=self.cfg["dock"]["v_max"],
                                   pos_tol=0.1, yaw_tol=0.2)
        elif sub == "dwell":
            if obs.now - self.acq["t_sub"] > 1.5:          # let the detector get a few frames
                self.acq["sub"] = "scan"
                self.acq["angle"], self.acq["last_yaw"] = 0.0, byaw
        elif sub == "scan":
            # turn on the spot: the plate may be anywhere round us, in view or not
            if self.acq["last_yaw"] is None:
                self.acq["last_yaw"] = byaw
            self.acq["angle"] += abs(_wrap(byaw - self.acq["last_yaw"]))
            self.acq["last_yaw"] = byaw
            if self.acq["angle"] >= 2 * math.pi:
                self.acq.update(sub="orbit", orbit=0, orbit_from=math.atan2(by - hint[1], bx - hint[0]))
                return
            d.vel = (0.0, 0.0, a["scan_rate"])
        elif sub == "orbit":
            # Still nothing: a plate facing away from where we stand is hidden behind its
            # own box, so walk a circle round the hint, facing it, to look from every side.
            k = self.acq["orbit"]
            if k >= a["orbit_steps"]:
                return self._fail(d, obs, "plate not found from any side")
            if self.acq.get("wp_k") != k:
                ang = self.acq["orbit_from"] + 2 * math.pi / a["orbit_steps"] * (k + 1)
                self.acq.update(wp=(hint[0] + a["orbit_m"] * math.cos(ang),
                                    hint[1] + a["orbit_m"] * math.sin(ang)),
                                wp_k=k, arrived=None)
            wp = self.acq["wp"]
            face = math.atan2(hint[1] - wp[1], hint[0] - wp[0])
            vel, ok = self._drive(obs.base, wp, face, v_max=self.cfg["dock"]["v_max"],
                                  pos_tol=0.12, yaw_tol=0.15)
            d.vel = vel
            if ok and self.acq["arrived"] is None:
                self.acq["arrived"] = obs.now
            if self.acq["arrived"] is not None and obs.now - self.acq["arrived"] > a["orbit_settle_s"]:
                self.acq["orbit"] += 1

    def _lock(self, obs: Obs, d: Decision, plate: dict) -> None:
        self.target = {"c": (plate["x"], plate["y"], plate["z"]),
                       "n": (plate["nx"], plate["ny"], plate["nz"])}
        self._start_dock(obs, d)

    # .. place target (detach only) ...........................................................
    def _begin_place(self, obs: Obs, d: Decision) -> None:
        r = self.req
        centre, normal = magnet.place_face(r["place"], r["size"], r["face"])
        self.target = {"c": centre, "n": normal}
        self._start_dock(obs, d)

    # .. dock .................................................................................
    def _dock_pose(self) -> tuple:
        yaw0 = self.dock[2] if self.dock else 0.0
        return magnet.dock_pose(self.target["c"], self.target["n"], reach=self.cfg["reach_m"],
                                fallback_yaw=yaw0)

    def _plan_at(self, base: tuple) -> dict | None:
        """The arm solution for the target plate from a given base pose, or None."""
        c, n = self.target["c"], self.target["n"]
        push = self.cfg["push_m"] if self.mode == "attach" else 0.0
        aim = tuple(c[i] - n[i] * push for i in range(3))
        return self._plan_reach(self.kin, magnet.to_base(base, aim), magnet.dir_to_base(base, n),
                                magnet.STANDOFF_M, self.q_cmd)

    def _start_dock(self, obs: Obs, d: Decision) -> None:
        self.dock = self._dock_pose()
        # Is the plate reachable AT ALL from where the arm will be? Refusing here, before
        # driving anywhere, is what makes an out-of-reach package a seconds-long failure.
        if self._plan_at(self.dock) is None:
            return self._fail(d, obs, "out of reach: no arm solution at the dock pose")
        self.sub = "stage"
        self._enter("dock", obs.now)
        if obs.base is not None:
            dx, dy = obs.base[0] - self.dock[0], obs.base[1] - self.dock[1]
            n = _hnorm(self.target["n"])
            if n and math.hypot(dx, dy) < self.cfg["dock"]["skip_within_m"]:
                lateral = abs(dx * -n[1] + dy * n[0])      # offset across the dock axis
                if lateral < self.cfg["dock"]["skip_lateral_m"]:
                    self.sub = "final"

    def _refresh(self, obs: Obs) -> None:
        """Re-aim at a fresh detection of the same plate. Only while the plate is well in
        view: close up it is clipped by the image edge and its centroid is biased."""
        u = self.cfg["update"]
        if self.mode != "attach" or obs.base is None:
            return
        w, h = u["image"]
        for p in obs.plates:
            if p.get("age_s", 0.0) > u["max_age_s"]:
                continue
            px = p.get("px")
            if px and (px[0] < u["border_px"] or px[1] < u["border_px"]
                       or px[0] + px[2] > w - u["border_px"] or px[1] + px[3] > h - u["border_px"]):
                continue
            c = self.target["c"]
            if math.dist((p["x"], p["y"], p["z"]), c) > u["track_m"]:
                continue
            if math.hypot(p["x"] - obs.base[0], p["y"] - obs.base[1]) < u["min_dist_m"]:
                continue
            self.target = {"c": (0.5 * c[0] + 0.5 * p["x"], 0.5 * c[1] + 0.5 * p["y"],
                                 0.5 * c[2] + 0.5 * p["z"]),
                           "n": (p["nx"], p["ny"], p["nz"])}
            self.dock = self._dock_pose()
            return

    def _ph_dock(self, obs: Obs, d: Decision) -> None:
        dk = self.cfg["dock"]
        if self._no_pose(obs, d):
            return
        if obs.now - self.t_phase > dk["timeout_s"]:
            return self._fail(d, obs, "dock timeout")
        self._refresh(obs)
        n = _hnorm(self.target["n"])
        stage = self.dock
        if n:
            stage = (self.dock[0] + n[0] * self.cfg["stage_m"],
                     self.dock[1] + n[1] * self.cfg["stage_m"], self.dock[2])
        if self.sub == "stage":
            vel, ok = self._drive(obs.base, stage[:2], stage[2], v_max=dk["v_max"],
                                  pos_tol=0.06, yaw_tol=0.1)
            d.vel = vel
            if ok:
                self.sub = "final"
            return
        vel, ok = self._drive(obs.base, self.dock[:2], self.dock[2], v_max=dk["v_final"],
                              pos_tol=dk["pos_tol_m"], yaw_tol=dk["yaw_tol_rad"])
        d.vel = vel
        if not ok:
            return
        d.vel = (0.0, 0.0, 0.0)
        if self.mode == "attach":
            self.fixes = []
            self._enter("confirm", obs.now)
            return
        self._reach(obs, d)

    def _reach(self, obs: Obs, d: Decision) -> None:
        self.plan = self._plan_at(obs.base)         # again, from where we actually stand
        if self.plan is None:
            return self._fail(d, obs, "out of reach: no arm solution after docking")
        self._arm_to(self.plan["pre"]["q"], obs.now)
        self._enter("pre", obs.now)

    # .. confirm (attach only) ...................................................................
    def _clipped(self, p: dict) -> bool:
        """The plate touches the image's edge, so its centroid is biased."""
        u, px = self.cfg["update"], p.get("px")
        if not px:
            return False
        w, h = u["image"]
        return (px[0] < u["border_px"] or px[1] < u["border_px"]
                or px[0] + px[2] > w - u["border_px"] or px[1] + px[3] > h - u["border_px"])

    def _ph_confirm(self, obs: Obs, d: Decision) -> None:
        """Stopped at the dock, measure the plate again before committing the arm.

        The lock was taken from further away and the base has moved on its own localization
        since, so this is what catches drift. It costs nothing when the fix agrees, and
        changes nothing when the plate cannot be seen from here: then the arm goes on the
        last fix, as it always did, and the status says so."""
        d.vel = (0.0, 0.0, 0.0)
        if self._no_pose(obs, d):
            return
        c, u = self.cfg["confirm"], self.cfg["update"]
        for p in obs.plates:
            fix = (p["x"], p["y"], p["z"], p["nx"], p["ny"], p["nz"])
            if (p.get("age_s", 0.0) <= u["max_age_s"] and not self._clipped(p) and fix not in self.fixes
                    and math.dist(fix[:3], self.target["c"]) <= u["track_m"]):
                self.fixes.append(fix)
        if len(self.fixes) >= c["samples"]:
            mean = [sum(f[i] for f in self.fixes) / len(self.fixes) for i in range(6)]
            norm = math.sqrt(sum(v * v for v in mean[3:])) or 1.0
            self.dock_shift = math.dist(mean[:3], self.target["c"])
            self.target = {"c": tuple(mean[:3]), "n": tuple(v / norm for v in mean[3:])}
            self.fixes = []
            dock = self._dock_pose()
            if (math.hypot(dock[0] - obs.base[0], dock[1] - obs.base[1]) > c["redock_m"]
                    and self.redocks < c["max_redocks"]):
                self.redocks += 1
                self.dock, self.sub = dock, "final"
                self._enter("dock", obs.now)
                return
            return self._reach(obs, d)
        if obs.now - self.t_phase > c["timeout_s"]:
            d.note = "no fresh plate fix at the dock: going on the last one"
            self._reach(obs, d)

    # .. reach ................................................................................
    def _ph_pre(self, obs: Obs, d: Decision) -> None:
        d.vel = (0.0, 0.0, 0.0)
        if self._arm_ready(obs):
            self._arm_to(self.plan["final"]["q"], obs.now)
            self._enter("approach", obs.now)

    def _ph_approach(self, obs: Obs, d: Decision) -> None:
        d.vel = (0.0, 0.0, 0.0)
        if not self._arm_ready(obs):
            return
        if self.mode == "attach":
            d.pick_cmd = "attach"
            self._enter("energize", obs.now)
        else:
            d.pick_cmd = "detach"
            self._enter("release", obs.now)

    def _ph_energize(self, obs: Obs, d: Decision) -> None:
        d.vel = (0.0, 0.0, 0.0)
        if obs.held:
            self._arm_to(self.plan["pre"]["q"], obs.now)
            self._enter("lift", obs.now)
        elif obs.now - self.t_phase > self.cfg["capture_timeout_s"]:
            self._fail(d, obs, "magnet missed: nothing within capture range")

    def _ph_lift(self, obs: Obs, d: Decision) -> None:
        d.vel = (0.0, 0.0, 0.0)
        if self._arm_done(obs.now):
            self._arm_to(self.cfg["tuck_q"], obs.now)
            self._enter("carry", obs.now)

    def _ph_carry(self, obs: Obs, d: Decision) -> None:
        d.vel = (0.0, 0.0, 0.0)
        if self._arm_done(obs.now):
            self._backout("picked", obs)

    def _backout(self, result: str, obs: Obs) -> None:
        """Back away from the deck before reporting done. The base is next sent somewhere
        else in a straight line, and from the dock that line can clip the deck it is
        standing against; from the staging point in front of it, it cannot (that is the
        geometry the pick worlds' transits are checked against)."""
        self.pending = result
        self._enter("backout", obs.now)

    def _ph_backout(self, obs: Obs, d: Decision) -> None:
        n = _hnorm(self.target["n"]) if self.target else None
        if obs.base is None or n is None or self.dock is None:
            return self._finish(d, obs, self.pending)
        stage = (self.dock[0] + n[0] * self.cfg["stage_m"], self.dock[1] + n[1] * self.cfg["stage_m"])
        d.vel, ok = self._drive(obs.base, stage, self.dock[2], v_max=self.cfg["dock"]["v_final"] * 2,
                                pos_tol=0.06, yaw_tol=0.2)
        if ok:
            self._finish(d, obs, self.pending)
        elif obs.now - self.t_phase > self.cfg["dock"]["timeout_s"]:
            # the job is done either way; say the base is not where it was meant to be
            d.note = "backout timed out"
            self._finish(d, obs, self.pending)

    def _ph_release(self, obs: Obs, d: Decision) -> None:
        d.vel = (0.0, 0.0, 0.0)
        if not obs.held:
            self._arm_to(self.plan["pre"]["q"], obs.now)
            self._enter("retreat", obs.now)
        elif obs.now - self.t_phase > self.cfg["release_timeout_s"]:
            self._fail(d, obs, "magnet would not let go")

    def _ph_retreat(self, obs: Obs, d: Decision) -> None:
        d.vel = (0.0, 0.0, 0.0)
        if self._arm_done(obs.now):
            self._arm_to(self.cfg["tuck_q"], obs.now)
            self._enter("stow", obs.now)

    def _ph_stow(self, obs: Obs, d: Decision) -> None:
        d.vel = (0.0, 0.0, 0.0)
        if self._arm_done(obs.now):
            self._backout("placed", obs)
