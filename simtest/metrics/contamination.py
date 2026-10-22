"""map_contamination — did moving actors get painted into the SLAM map?

dynamic_obstacle_filter.py exists so that movers reach Nav2 (ephemerally) but
NOT RTAB-Map's permanent grid. This scores exactly that: the floor every actor
walks over (its path legs, grown by its footprint radius + margin) is free
space by construction, so any OCCUPIED cell there in /map is an actor (or
another mover) that leaked into SLAM. Cells inside a static prop's footprint
are excluded — props are meant to be mapped.

Scored on the LAST map of the run (what a saved map would keep), with the
worst map seen reported as well, since a map that was dirty mid-run and
cleaned up later still misled Nav2's global planner while it was dirty.

Only `observed` cells count toward the fraction: a swept cell the robots never
saw is neither clean nor dirty. A low `observed_frac` means the test didn't
actually look at the actor's floor and the result says little.
"""
from __future__ import annotations

from simtest import geometry
from simtest import world as wl

from . import Metric, MetricError, register

OCCUPIED_AT = 50  # OccupancyGrid: 100 = occupied, 0 = free, -1 = unknown


def _swept_mask(grid, actors, margin_m, props, prop_margin_m):
    """Boolean (h, w) mask of cells over any actor's walked floor."""
    import numpy as np

    xs, ys = grid.cell_centres()
    mask = np.zeros(grid.data.shape, dtype=bool)
    for a in actors:
        r = wl.half_extent(a) + margin_m
        pts = list(a["path"]["waypoints"])
        legs = list(zip(pts[:-1], pts[1:]))
        if a["path"]["mode"] == "loop":
            legs.append((pts[-1], pts[0]))
        for (ax, ay), (bx, by) in legs:
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = np.clip(((xs - ax) * dx + (ys - ay) * dy) / L2, 0.0, 1.0)
            mask |= np.hypot(xs - (ax + t * dx), ys - (ay + t * dy)) <= r
    for p in props:
        x0, y0, x1, y1 = geometry.bounds(wl.prop_footprint(p))
        m = prop_margin_m
        mask &= ~((xs >= x0 - m) & (xs <= x1 + m) & (ys >= y0 - m) & (ys <= y1 + m))
    return mask


@register
class MapContamination(Metric):
    name = "map_contamination"
    version = 1
    streams = {"map"}
    summary = "Occupied /map cells on floor only actors walk on (movers leaking into SLAM)."
    params = {
        "actors": (list, None, "actor ids whose floor to check (default: all)"),
        "margin_m": (float, 0.10, "grow each actor's swept band by this much"),
        "prop_margin_m": (float, 0.30, "exclude cells this close to a static prop"),
    }
    values = {
        "occupied_cells": "occupied cells on the actors' floor in the final map",
        "occupied_m2": "the same, in square metres",
        "occupied_frac": "occupied / observed cells on that floor, final map (0 = clean)",
        "peak_occupied_frac": "worst occupied_frac over every map sample in the run",
        "observed_frac": "observed (free or occupied) / all cells on that floor, final "
                         "map — low means the robots barely saw it",
        "maps": "map samples scored",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.actors = ctx.actors(self.p["actors"])
        self._mask_key = None
        self._mask = None
        self.last = None
        self.peak = None
        self.maps = 0

    def check_context(self):
        if not self.actors:
            raise MetricError("map_contamination: the world has no actors")

    def _score(self, grid):
        if grid.key() != self._mask_key:
            self._mask = _swept_mask(grid, self.actors, self.p["margin_m"],
                                     self.ctx.props(), self.p["prop_margin_m"])
            self._mask_key = grid.key()
        cells = grid.data[self._mask]
        total = int(cells.size)
        occ = int((cells >= OCCUPIED_AT).sum())
        observed = int((cells >= 0).sum())
        return {"total": total, "occupied": occ, "observed": observed,
                "res": grid.resolution}

    def on_event(self, ev):
        s = self._score(ev["grid"])
        self.maps += 1
        self.last = s
        if s["observed"]:
            frac = s["occupied"] / s["observed"]
            self.peak = frac if self.peak is None else max(self.peak, frac)

    def result(self):
        s = self.last
        if s is None or not s["total"]:
            return {"occupied_cells": None, "occupied_m2": None, "occupied_frac": None,
                    "peak_occupied_frac": None, "observed_frac": None, "maps": self.maps}
        return {
            "occupied_cells": s["occupied"],
            "occupied_m2": round(s["occupied"] * s["res"] ** 2, 3),
            "occupied_frac": round(s["occupied"] / s["observed"], 4) if s["observed"] else None,
            "peak_occupied_frac": None if self.peak is None else round(self.peak, 4),
            "observed_frac": round(s["observed"] / s["total"], 3),
            "maps": self.maps,
        }
