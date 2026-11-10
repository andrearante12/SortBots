"""coverage — how much floor the fused /map knows about (exploration progress)."""
from __future__ import annotations

from . import Metric, register


@register
class Coverage(Metric):
    name = "coverage"
    version = 1
    streams = {"map", "end"}
    summary = "Free / known floor area in the fused /map, and mapping rate."
    params = {}
    values = {
        "free_m2": "free area in the final map, square metres",
        "known_m2": "free + occupied area in the final map",
        "free_m2_per_min": "free_m2 per sim minute of the run",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.free = None
        self.known = None
        self.sim_s = None

    def on_event(self, ev):
        if ev["s"] == "end":
            self.sim_s = ev.get("sim_s")
            return
        g = ev["grid"]
        a = g.resolution ** 2
        self.free = float((g.data == 0).sum()) * a
        self.known = float((g.data >= 0).sum()) * a
        self.sim_s = ev["t"]

    def result(self):
        return {
            "free_m2": None if self.free is None else round(self.free, 1),
            "known_m2": None if self.known is None else round(self.known, 1),
            "free_m2_per_min": round(self.free / (self.sim_s / 60.0), 2)
                               if self.free is not None and self.sim_s else None,
        }
