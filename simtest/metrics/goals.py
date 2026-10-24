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


@register
class Goals(Metric):
    name = "goals"
    version = 1
    streams = {"goal", "gt"}
    summary = "Nav2 goal outcomes, time to goal and true final position error."
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
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        robots = self.p["robots"]
        self.planned = [g for g in ctx.goals if robots is None or g["robot"] in robots]
        self.goals: dict[str, dict] = {}
        self.last_pose: dict[str, dict] = {}
        self.path_m = 0.0

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
                    self.path_m += math.hypot(p["x"] - prev["x"], p["y"] - prev["y"])
                self.last_pose[rid] = p
            return
        if not self._scored(ev["rid"]):
            return
        g = self.goals.setdefault(ev["goal_id"], {
            "rid": ev["rid"], "pose": ev.get("pose"), "sent_at": None,
            "state": None, "ended_at": None, "error_m": None})
        g["state"] = ev["state"]
        if ev["state"] == "sent":
            g["sent_at"] = ev["t"]
        elif ev["state"] in TERMINAL and g["ended_at"] is None:
            g["ended_at"] = ev["t"]
            p = self.last_pose.get(ev["rid"])
            if p is not None and g["pose"]:
                g["error_m"] = math.hypot(p["x"] - g["pose"][0], p["y"] - g["pose"][1])

    def result(self):
        done = [g for g in self.goals.values() if g["state"] == "succeeded"]
        failed = [g for g in self.goals.values() if g["state"] in TERMINAL - {"succeeded"}]
        times = [g["ended_at"] - g["sent_at"] for g in done if g["sent_at"] is not None]
        errs = [g["error_m"] for g in done if g["error_m"] is not None]
        total = max(len(self.planned), len(self.goals))
        return {
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
                      "ended_at": g["ended_at"],
                      "error_m": None if g["error_m"] is None else round(g["error_m"], 3)}
                for gid, g in self.goals.items()}
