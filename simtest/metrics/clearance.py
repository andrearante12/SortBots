"""clearance — how close robots came to actors (and optionally props), from ground truth.

Measured footprint-to-footprint: the robot is Nav2's own footprint polygon at
its TRUE pose, an actor is its cylinder/box outline at its true pose. So
`min_m` is the physical gap, not a centre distance and not what the robot's
SLAM believed. 0.0 means they touched.

Contact is counted whoever moved: actors follow fixed paths and never yield
(simtest/world.py), so a robot that stopped dead can still be walked into.
That is deliberate — the question a test asks is "did anything hit anything",
and a world whose paths force contact on a robot that yielded is a badly laid
out world, which this metric will make obvious.
"""
from __future__ import annotations

import math

from simtest import geometry
from simtest import world as wl

from . import Metric, MetricError, register

MAX_DT = 0.5  # s — a gap in ground truth longer than this isn't integrated across


@register
class Clearance(Metric):
    name = "clearance"
    version = 2   # v2 (2026-11-04): `peers` — robot-to-robot pairs
    streams = {"gt"}
    summary = "Minimum robot-to-actor (or prop, or peer robot) gap, contacts and near-miss time."
    params = {
        "robots": (list, None, "robot ids to check (default: all in the run)"),
        "actors": (list, None, "actor ids to check (default: every actor in the world)"),
        "props": (bool, False, "also measure against the world's static props"),
        "peers": (bool, False, "also measure every checked robot against the other robots "
                               "in the run (both at their Nav2 footprint)"),
        "near_m": (float, 0.5, "gap below which time counts as a near miss"),
        "contact_m": (float, 0.02, "gap at or below which it counts as contact"),
    }
    values = {
        "min_m": "smallest gap seen between any checked robot and object, metres",
        "contacts": "number of distinct contact episodes (gap <= contact_m)",
        "contact_s": "total sim seconds in contact",
        "near_s": "total sim seconds closer than near_m (includes contact)",
        "samples": "ground-truth samples scored",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.targets = [("actor", a) for a in ctx.actors(self.p["actors"])]
        if self.p["props"]:
            self.targets += [("prop", p) for p in ctx.props()]
        self.prop_shapes = {p["id"]: wl.prop_footprint(p) for p in ctx.props()}
        self.pairs: dict[tuple, dict] = {}
        self.prev_t = None
        self.samples = 0

    def check_context(self):
        peers = self.p["peers"] and len(self.ctx.robots) >= 2
        if self.p["peers"] and len(self.ctx.robots) < 2:
            raise MetricError("clearance: peers: true needs a run with at least two robots")
        if not self.targets and not peers:
            raise MetricError("clearance: the world has no actors to measure against "
                              "(set props: true or peers: true, or use a world with actors)")
        if self.p["actors"]:
            missing = set(self.p["actors"]) - {a["id"] for a in self.ctx.actors()}
            if missing:
                raise MetricError(f"clearance: no actor(s) {sorted(missing)} in the world")

    def on_event(self, ev):
        dt = 0.0 if self.prev_t is None else min(MAX_DT, max(0.0, ev["t"] - self.prev_t))
        self.prev_t = ev["t"]
        robots = self.p["robots"] or self.ctx.robots
        self.samples += 1
        for rid in robots:
            rp = ev["robots"].get(rid)
            if rp is None:
                continue
            fp = geometry.footprint_poly(self.ctx.footprint, rp["x"], rp["y"], rp["yaw"])
            targets = list(self.targets)
            if self.p["peers"]:
                # Every other robot in the run, each pair once (sorted ids),
                # at its own true pose and the same Nav2 footprint.
                for other in self.ctx.robots:
                    op = ev["robots"].get(other)
                    if other <= rid or op is None:
                        continue
                    targets.append(("peer", {"id": other, "pose": op}))
            for kind, item in targets:
                if kind == "actor":
                    ap = ev["actors"].get(item["id"])
                    if ap is None:
                        continue
                    shape = wl.footprint(item, ap["x"], ap["y"], ap["yaw"])
                elif kind == "peer":
                    op = item["pose"]
                    shape = geometry.footprint_poly(self.ctx.footprint, op["x"], op["y"], op["yaw"])
                else:
                    shape = self.prop_shapes[item["id"]]
                gap = geometry.distance(fp, shape)
                st = self.pairs.setdefault((rid, item["id"]), {
                    "min_m": math.inf, "min_at_s": None, "contacts": 0,
                    "contact_s": 0.0, "near_s": 0.0, "in_contact": False})
                if gap < st["min_m"]:
                    st["min_m"], st["min_at_s"] = gap, ev["t"]
                touching = gap <= self.p["contact_m"]
                if touching and not st["in_contact"]:
                    st["contacts"] += 1
                if touching:
                    st["contact_s"] += dt
                if gap < self.p["near_m"]:
                    st["near_s"] += dt
                st["in_contact"] = touching

    def result(self):
        if not self.pairs:
            return {"min_m": None, "contacts": None, "contact_s": None, "near_s": None,
                    "samples": self.samples}
        ps = self.pairs.values()
        return {
            "min_m": round(min(p["min_m"] for p in ps), 3),
            "contacts": sum(p["contacts"] for p in ps),
            "contact_s": round(sum(p["contact_s"] for p in ps), 2),
            "near_s": round(sum(p["near_s"] for p in ps), 2),
            "samples": self.samples,
        }

    def detail(self):
        return {f"{r}~{o}": {"min_m": round(p["min_m"], 3), "min_at_s": p["min_at_s"],
                             "contacts": p["contacts"], "near_s": round(p["near_s"], 2)}
                for (r, o), p in sorted(self.pairs.items())}
