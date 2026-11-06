"""goals — did the robots get where the test sent them, and how fast.

Outcome comes from Nav2 (the action result the probe received); position
error comes from ground truth, because "Nav2 says succeeded" only means the
robot believes it arrived. A large final_error_m with a success is SLAM drift,
not navigation, and is worth seeing as such.
"""
from __future__ import annotations

import math
import statistics

from . import Metric, MetricError, register

TERMINAL = {"succeeded", "aborted", "rejected", "canceled", "timeout"}


MIN_LEG_M = 1.0   # a detour ratio over a shorter leg is mostly goal tolerance


@register
class Goals(Metric):
    name = "goals"
    version = 2   # v2 (2026-11-04): per-goal detour ratio and mean speed
    streams = {"goal", "gt"}
    summary = "Nav2 goal outcomes, time to goal, detour and true final position error."
    params = {
        "robots": (list, None, "robot ids to score (default: every robot with a goal)"),
    }
    values = {
        "sent": "goals the probe sent",
        "succeeded": "goals Nav2 reported succeeded",
        "failed": "goals that ended aborted / rejected / canceled / timeout",
        "unfinished": "goals still active (or never sent) when the run stopped",
        "success_rate": "succeeded / total goals in the test (unsent ones count as failures)",
        "mean_time_s": "mean sim seconds from send to success, over succeeded goals",
        "max_time_s": "slowest success, sim seconds",
        "max_final_error_m": "worst true distance from the goal at success (ground truth)",
        "path_m": "total distance the scored robots actually drove (ground truth)",
        "max_detour_ratio": "worst (distance driven / straight line) over succeeded goals "
                            "at least 1 m long — 1.0 is a straight run",
        "mean_speed_mps": "distance driven while goals were active / that time",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        robots = self.p["robots"]
        self.planned = [g for g in ctx.goals if robots is None or g["robot"] in robots]
        self.goals: dict[str, dict] = {}
        self.last_pose: dict[str, dict] = {}
        self.path_m = 0.0
        self.active: dict[str, str] = {}      # rid -> goal_id currently being driven

    def check_context(self):
        if not self.planned:
            raise MetricError("goals: the test sends no goals"
                              + (f" to {self.p['robots']}" if self.p["robots"] else ""))

    def _scored(self, rid):
        return self.p["robots"] is None or rid in self.p["robots"]

    def on_event(self, ev):
        if ev["s"] == "gt":
            for rid, p in ev["robots"].items():
                if not self._scored(rid):
                    continue
                prev = self.last_pose.get(rid)
                if prev is not None:
                    step = math.hypot(p["x"] - prev["x"], p["y"] - prev["y"])
                    self.path_m += step
                    gid = self.active.get(rid)
                    if gid is not None:
                        self.goals[gid]["leg_m"] += step
                self.last_pose[rid] = p
            return
        if not self._scored(ev["rid"]):
            return
        g = self.goals.setdefault(ev["goal_id"], {
            "rid": ev["rid"], "pose": ev.get("pose"), "sent_at": None,
            "state": None, "ended_at": None, "error_m": None,
            "start": None, "leg_m": 0.0})
        g["state"] = ev["state"]
        if ev["state"] == "sent":
            g["sent_at"] = ev["t"]
            g["start"] = self.last_pose.get(ev["rid"])
            self.active[ev["rid"]] = ev["goal_id"]
        elif ev["state"] in TERMINAL and g["ended_at"] is None:
            g["ended_at"] = ev["t"]
            if self.active.get(ev["rid"]) == ev["goal_id"]:
                del self.active[ev["rid"]]
            p = self.last_pose.get(ev["rid"])
            if p is not None and g["pose"]:
                g["error_m"] = math.hypot(p["x"] - g["pose"][0], p["y"] - g["pose"][1])

    @staticmethod
    def _detour(g) -> float | None:
        if g["state"] != "succeeded" or g["start"] is None or not g["pose"]:
            return None
        straight = math.hypot(g["pose"][0] - g["start"]["x"], g["pose"][1] - g["start"]["y"])
        return g["leg_m"] / straight if straight >= MIN_LEG_M else None

    def result(self):
        done = [g for g in self.goals.values() if g["state"] == "succeeded"]
        failed = [g for g in self.goals.values() if g["state"] in TERMINAL - {"succeeded"}]
        times = [g["ended_at"] - g["sent_at"] for g in done if g["sent_at"] is not None]
        errs = [g["error_m"] for g in done if g["error_m"] is not None]
        total = max(len(self.planned), len(self.goals))
        detours = [d for d in (self._detour(g) for g in done) if d is not None]
        active_s = sum(g["ended_at"] - g["sent_at"] for g in self.goals.values()
                       if g["sent_at"] is not None and g["ended_at"] is not None)
        active_m = sum(g["leg_m"] for g in self.goals.values() if g["ended_at"] is not None)
        return {
            "max_detour_ratio": round(max(detours), 3) if detours else None,
            "mean_speed_mps": round(active_m / active_s, 3) if active_s > 0 else None,
            "sent": sum(1 for g in self.goals.values() if g["sent_at"] is not None),
            "succeeded": len(done),
            "failed": len(failed),
            "unfinished": total - len(done) - len(failed),
            "success_rate": round(len(done) / total, 3) if total else None,
            "mean_time_s": round(statistics.fmean(times), 2) if times else None,
            "max_time_s": round(max(times), 2) if times else None,
            "max_final_error_m": round(max(errs), 3) if errs else None,
            "path_m": round(self.path_m, 2),
        }

    def detail(self):
        return {gid: {"robot": g["rid"], "state": g["state"], "sent_at": g["sent_at"],
                      "ended_at": g["ended_at"], "leg_m": round(g["leg_m"], 2),
                      "detour_ratio": None if self._detour(g) is None else round(self._detour(g), 3),
                      "error_m": None if g["error_m"] is None else round(g["error_m"], 3)}
                for gid, g in self.goals.items()}
