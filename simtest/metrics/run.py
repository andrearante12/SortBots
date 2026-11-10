"""run_stats — how long the run took, in sim and wall time, and why it stopped."""
from __future__ import annotations

from . import Metric, register


@register
class RunStats(Metric):
    name = "run_stats"
    version = 1
    streams = {"start", "end", "gt"}
    summary = "Sim/wall duration, real-time factor and stop reason."
    params = {}
    values = {
        "sim_s": "sim seconds from t0 to the stop",
        "wall_s": "wall seconds from t0 to the stop",
        "rtf": "real-time factor over the scored window (sim_s / wall_s)",
        "stop_reason": "why the run stopped: sim_s | goals_done | tasks_done | "
                       "explorer_done | wall_timeout | interrupted | ...",
        "gt_hz": "ground-truth messages per sim second (expect ~10; low means "
                 "the sim side is starved and every gt metric is coarser)",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.end = None
        self.gt = 0
        self.last_t = 0.0

    def on_event(self, ev):
        if ev["s"] == "gt":
            self.gt += 1
            self.last_t = ev["t"]
        elif ev["s"] == "end":
            self.end = ev

    def result(self):
        e = self.end or {}
        sim_s = e.get("sim_s", self.last_t)
        wall = e.get("wall_s")
        return {
            "sim_s": round(sim_s, 2) if sim_s is not None else None,
            "wall_s": round(wall, 2) if wall is not None else None,
            "rtf": round(sim_s / wall, 3) if wall and sim_s else None,
            "stop_reason": e.get("reason"),
            "gt_hz": round(self.gt / sim_s, 2) if sim_s else None,
        }
