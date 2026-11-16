"""reaction — how fast, and how far, a robot slows and stops for a mover, from ground truth.

Pass/fail (clearance) says whether anything touched. This says how the robot
behaved on the way there: for every ENCOUNTER between a robot and an actor it
measures the time from the actor becoming a threat to the robot slowing and
stopping, the distance driven while braking, the hardest deceleration, the
closest the two came in time (time-to-collision) and how long the robot sat
yielding. All of it from /sim/ground_truth, so the numbers are about the
robot's true motion, not what its planner intended.

An encounter OPENS at the first sample where the actor is a threat:
  * within threat_range_m, in the robot's front half-plane, and
  * on a collision course under constant velocities — the closest point of
    approach lies 0 < t_cpa <= threat_ttc_s ahead and would bring the two
    footprints within conflict_margin_m of each other.
It CLOSES once the actor has not been a threat for release_s (or the run
ends). Onset speed v0 is the robot's speed when it opened; an encounter the
robot entered already slower than min_speed_mps is counted but excluded from
the timing statistics (there was nothing to react from).

Robot speed is the finite difference of 10 Hz ground-truth poses, smoothed
with a centred smooth_s window — offline scoring can look both ways in
time, and the 1 cm quantisation of a 10 Hz pose would otherwise read as
±0.1 m/s of jitter.
"""
from __future__ import annotations

import math
import statistics

from simtest import geometry
from simtest import world as wl

from . import Metric, MetricError, register

MAX_DT = 0.5   # s — a gap in ground truth longer than this isn't integrated across
DECEL_WINDOW_S = 0.3


def smooth_series(ts: list[float], vals: list, half_window: float) -> list:
    """Centred moving mean over |t_j - t_i| <= half_window, skipping None."""
    out, n, j0 = [], len(ts), 0
    for i, t in enumerate(ts):
        while j0 < n and ts[j0] < t - half_window:
            j0 += 1
        acc, cnt, j = 0.0, 0, j0
        while j < n and ts[j] <= t + half_window:
            if vals[j] is not None:
                acc += vals[j]
                cnt += 1
            j += 1
        out.append(acc / cnt if cnt else None)
    return out


def cpa(rx, ry, vx, vy):
    """Time to and distance at the closest point of approach of a relative
    position (rx, ry) moving at relative velocity (vx, vy). t is inf when the
    two aren't closing."""
    v2 = vx * vx + vy * vy
    if v2 < 1e-9:
        return math.inf, math.hypot(rx, ry)
    t = -(rx * vx + ry * vy) / v2
    if t <= 0:
        return t, math.hypot(rx, ry)
    return t, math.hypot(rx + vx * t, ry + vy * t)


@register
class Reaction(Metric):
    name = "reaction"
    version = 1
    streams = {"gt"}
    summary = "Time to slow/stop, braking distance, min time-to-collision and yield time per encounter."
    params = {
        "robots": (list, None, "robot ids to score (default: all in the run)"),
        "actors": (list, None, "actor ids that count as threats (default: every actor)"),
        "threat_range_m": (float, 4.0, "an actor farther than this is never a threat"),
        "threat_ttc_s": (float, 6.0, "closest approach farther ahead than this isn't a threat yet"),
        "conflict_margin_m": (float, 0.3, "footprints would pass closer than this: collision course"),
        "slow_frac": (float, 0.5, "slowed = speed at or below this fraction of the onset speed"),
        "stop_mps": (float, 0.05, "stopped = speed at or below this"),
        "min_speed_mps": (float, 0.15, "onset slower than this: encounter counted, timing excluded"),
        "release_s": (float, 1.0, "no-threat time that closes an encounter"),
        "smooth_s": (float, 0.5, "centred window the robot's speed is averaged over"),
    }
    values = {
        "encounters": "robot-actor encounters (threat episodes) seen",
        "timed": "encounters the robot entered faster than min_speed_mps",
        "reacted": "timed encounters in which the robot slowed to slow_frac of its onset speed",
        "unreacted": "timed encounters in which it never did",
        "mean_time_to_slow_s": "mean sim seconds from onset to slowed, over reacted encounters",
        "max_time_to_slow_s": "slowest such reaction",
        "max_time_to_stop_s": "slowest onset-to-stopped, over encounters that stopped",
        "stopped": "timed encounters in which the robot came to a stop",
        "max_brake_m": "longest distance driven from onset to the encounter's slowest point",
        "max_decel_mps2": "hardest deceleration over any 0.3 s during an encounter",
        "min_ttc_s": "smallest time-to-collision while on a collision course, any encounter",
        "yield_s": "total sim seconds stopped during encounters",
        "min_gap_m": "closest footprint-to-footprint gap during encounters",
        "samples": "ground-truth samples scored",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.actors = {a["id"]: a for a in ctx.actors(self.p["actors"])}
        self.rows: list[dict] = []
        self.samples = 0
        self.robot_r = max((math.hypot(x, y) for x, y in ctx.footprint), default=0.32)

    def check_context(self):
        if not self.actors:
            raise MetricError("reaction: the world has no actors to react to")
        if self.p["actors"]:
            missing = set(self.p["actors"]) - {a["id"] for a in self.ctx.actors()}
            if missing:
                raise MetricError(f"reaction: no actor(s) {sorted(missing)} in the world")

    def on_event(self, ev):
        self.samples += 1
        self.rows.append({"t": ev["t"], "robots": ev["robots"], "actors": ev["actors"]})

    # -- scoring (all in result(): speed smoothing looks both ways in time) ----

    def _robot_series(self, rid):
        ts, xs, ys, yaws = [], [], [], []
        for r in self.rows:
            p = r["robots"].get(rid)
            if p is None:
                continue
            ts.append(r["t"]); xs.append(p["x"]); ys.append(p["y"]); yaws.append(p["yaw"])
        vx, vy = [None], [None]
        for i in range(1, len(ts)):
            dt = ts[i] - ts[i - 1]
            if 0 < dt <= MAX_DT:
                vx.append((xs[i] - xs[i - 1]) / dt)
                vy.append((ys[i] - ys[i - 1]) / dt)
            else:
                vx.append(None); vy.append(None)
        hw = self.p["smooth_s"] / 2.0
        vx, vy = smooth_series(ts, vx, hw), smooth_series(ts, vy, hw)
        speed = [None if a is None else math.hypot(a, b) for a, b in zip(vx, vy)]
        return ts, xs, ys, yaws, vx, vy, speed

    def _encounters(self, rid, aid):
        actor = self.actors[aid]
        ts, xs, ys, yaws, vxs, vys, speeds = self._robot_series(rid)
        if not ts:
            return []
        idx_by_t = {r["t"]: r for r in self.rows}
        conflict_r = self.robot_r + wl.half_extent(actor) + self.p["conflict_margin_m"]
        p = self.p
        encs, cur, quiet_since = [], None, None
        for i, t in enumerate(ts):
            ap = idx_by_t[t]["actors"].get(aid)
            sp = speeds[i]
            if ap is None or sp is None:
                continue
            vx, vy = vxs[i], vys[i]
            rx, ry = ap["x"] - xs[i], ap["y"] - ys[i]
            dist = math.hypot(rx, ry)
            front = rx * math.cos(yaws[i]) + ry * math.sin(yaws[i]) > 0
            t_cpa, d_cpa = cpa(rx, ry, ap["vx"] - vx, ap["vy"] - vy)
            on_course = 0 < t_cpa <= p["threat_ttc_s"] and d_cpa < conflict_r
            threat = dist <= p["threat_range_m"] and front and on_course
            if cur is None:
                if not threat:
                    continue
                cur = {"robot": rid, "actor": aid, "onset_s": t, "v0": sp, "min_speed": sp,
                       "min_speed_at": t, "slow_at": None, "stop_at": None, "path_m": 0.0,
                       "brake_m": None, "min_ttc_s": math.inf, "yield_s": 0.0,
                       "min_gap_m": math.inf, "max_decel": 0.0, "_last": (t, xs[i], ys[i]),
                       "_speeds": [(t, sp)], "end_s": t}
                quiet_since = None
            else:
                lt, lx, ly = cur["_last"]
                dt = min(MAX_DT, max(0.0, t - lt))
                cur["path_m"] += math.hypot(xs[i] - lx, ys[i] - ly)
                cur["_last"] = (t, xs[i], ys[i])
                cur["_speeds"].append((t, sp))
                if sp <= p["stop_mps"]:
                    cur["yield_s"] += dt
                if sp < cur["min_speed"]:
                    cur["min_speed"], cur["min_speed_at"] = sp, t
                    cur["brake_m"] = cur["path_m"]
            if cur["v0"] > p["min_speed_mps"]:
                if cur["slow_at"] is None and sp <= p["slow_frac"] * cur["v0"]:
                    cur["slow_at"] = t
                if cur["stop_at"] is None and sp <= p["stop_mps"]:
                    cur["stop_at"] = t
                    cur["brake_m"] = cur["path_m"]
            if on_course:
                cur["min_ttc_s"] = min(cur["min_ttc_s"], t_cpa)
            fp = geometry.footprint_poly(self.ctx.footprint, xs[i], ys[i], yaws[i])
            cur["min_gap_m"] = min(cur["min_gap_m"],
                                   geometry.distance(fp, wl.footprint(actor, ap["x"], ap["y"],
                                                                      ap["yaw"])))
            cur["end_s"] = t
            if threat:
                quiet_since = None
            else:
                quiet_since = t if quiet_since is None else quiet_since
                if t - quiet_since >= p["release_s"]:
                    encs.append(self._close(cur))
                    cur, quiet_since = None, None
        if cur is not None:
            encs.append(self._close(cur))
        return encs

    def _close(self, e):
        sp = e.pop("_speeds")
        e.pop("_last", None)
        # Hardest deceleration over any DECEL_WINDOW_S: the smoothed speed
        # drop across the window, not between adjacent 10 Hz samples.
        j, best = 0, 0.0
        for i, (t, v) in enumerate(sp):
            while j < len(sp) and sp[j][0] < t + DECEL_WINDOW_S:
                j += 1
            if j < len(sp):
                dt = sp[j][0] - t
                if dt > 0:
                    best = max(best, (v - sp[j][1]) / dt)
        e["max_decel"] = best
        e["timed"] = e["v0"] > self.p["min_speed_mps"]
        e["time_to_slow_s"] = None if e["slow_at"] is None else e["slow_at"] - e["onset_s"]
        e["time_to_stop_s"] = None if e["stop_at"] is None else e["stop_at"] - e["onset_s"]
        if e["min_ttc_s"] == math.inf:
            e["min_ttc_s"] = None
        if e["min_gap_m"] == math.inf:
            e["min_gap_m"] = None
        return e

    def _all(self):
        if getattr(self, "_cache", None) is None:
            robots = self.p["robots"] or self.ctx.robots
            self._cache = [e for rid in robots for aid in self.actors
                           for e in self._encounters(rid, aid)]
        return self._cache

    def result(self):
        encs = self._all()
        timed = [e for e in encs if e["timed"]]
        slow = [e["time_to_slow_s"] for e in timed if e["time_to_slow_s"] is not None]
        stop = [e["time_to_stop_s"] for e in timed if e["time_to_stop_s"] is not None]
        brake = [e["brake_m"] for e in timed if e["brake_m"] is not None]
        ttc = [e["min_ttc_s"] for e in encs if e["min_ttc_s"] is not None]
        gaps = [e["min_gap_m"] for e in encs if e["min_gap_m"] is not None]
        r3 = lambda v: None if v is None else round(v, 3)  # noqa: E731
        return {
            "encounters": len(encs),
            "timed": len(timed),
            "reacted": len(slow),
            "unreacted": len(timed) - len(slow),
            "mean_time_to_slow_s": r3(statistics.fmean(slow)) if slow else None,
            "max_time_to_slow_s": r3(max(slow)) if slow else None,
            "max_time_to_stop_s": r3(max(stop)) if stop else None,
            "stopped": len(stop),
            "max_brake_m": r3(max(brake)) if brake else None,
            "max_decel_mps2": r3(max((e["max_decel"] for e in encs), default=None)) if encs else None,
            "min_ttc_s": r3(min(ttc)) if ttc else None,
            "yield_s": round(sum(e["yield_s"] for e in encs), 2),
            "min_gap_m": r3(min(gaps)) if gaps else None,
            "samples": self.samples,
        }

    def detail(self):
        keep = ("onset_s", "end_s", "v0", "timed", "time_to_slow_s", "time_to_stop_s",
                "brake_m", "max_decel", "min_ttc_s", "yield_s", "min_gap_m")
        return {f"{e['robot']}~{e['actor']}@{e['onset_s']:.1f}":
                {k: (round(e[k], 3) if isinstance(e[k], float) else e[k]) for k in keep}
                for e in self._all()}
