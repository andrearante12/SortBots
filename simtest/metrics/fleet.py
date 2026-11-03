"""Fleet-scale metrics: deliveries, contacts, comms, map divergence, nav.

Written for fleetsim (fleetsim/README.md), which emits these streams on top
of the ones Isaac runs share (gt, dispatch, task). They are ordinary simtest
metrics: deterministic functions of the event stream, versioned, and
re-scorable from a recorded run (`python3 -m fleetsim rescore DIR`).
"""
from __future__ import annotations

import statistics

from . import Metric, register

FINAL = {"DONE", "FAILED", "CANCELLED"}
ACTIVE = {"NAV_TO_PICKUP", "PICKING", "NAV_TO_DROPOFF", "PLACING"}


def _pct(xs, q):
    if not xs:
        return None
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(q / 100 * (len(s) - 1))))]


@register
class FleetTasks(Metric):
    name = "fleet_tasks"
    version = 1
    streams = {"announce", "task", "dispatch", "end"}
    summary = "Auctioned deliveries: completion, throughput, latency, duplicate work."
    params = {}
    values = {
        "announced": "tasks the gateway announced",
        "done": "announced tasks some robot finished (DONE)",
        "done_frac": "done / announced",
        "failed": "FAILED reports (a robot gave up a leg; the task may be redone)",
        "cancelled": "CANCELLED reports (lease lost, out-bid, or withdrawn)",
        "redispatched": "dispatches beyond the first per task (it moved robot)",
        "duplicates": "episodes of two or more robots working the same task at once "
                      "(the later one drives to a shelf the item has left)",
        "mean_s": "mean sim seconds from announce to DONE",
        "p95_s": "95th percentile announce-to-DONE, sim seconds",
        "per_hour": "deliveries per sim hour over the whole run",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.announced: dict[str, float] = {}
        self.done_at: dict[str, float] = {}
        self.active: dict[str, set] = {}
        self.dispatches: dict[str, int] = {}
        self.failed = self.cancelled = self.duplicates = 0
        self.end_t = None

    def on_event(self, ev):
        s = ev["s"]
        if s == "announce":
            self.announced[ev["task_id"]] = ev["t"]
        elif s == "dispatch":
            self.dispatches[ev["task_id"]] = self.dispatches.get(ev["task_id"], 0) + 1
        elif s == "end":
            self.end_t = ev["t"]
        elif s == "task":
            tid, st, rid = ev["task_id"], ev["state"], ev["rid"]
            act = self.active.setdefault(tid, set())
            if st in ACTIVE:
                if rid not in act:
                    act.add(rid)
                    if len(act) == 2:
                        self.duplicates += 1
            elif st in FINAL:
                act.discard(rid)
                if st == "DONE":
                    self.done_at.setdefault(tid, ev["t"])
                elif st == "FAILED":
                    self.failed += 1
                else:
                    self.cancelled += 1

    def result(self):
        lat = [self.done_at[t] - a for t, a in self.announced.items() if t in self.done_at]
        n = len(self.announced)
        return {
            "announced": n,
            "done": len(lat),
            "done_frac": round(len(lat) / n, 3) if n else None,
            "failed": self.failed,
            "cancelled": self.cancelled,
            "redispatched": sum(max(0, c - 1) for c in self.dispatches.values()),
            "duplicates": self.duplicates,
            "mean_s": round(statistics.fmean(lat), 1) if lat else None,
            "p95_s": round(_pct(lat, 95), 1) if lat else None,
            "per_hour": round(len(lat) / self.end_t * 3600, 1) if self.end_t else None,
        }


@register
class Contacts(Metric):
    name = "contacts"
    version = 1
    streams = {"contact"}
    summary = "Physical contacts: a robot driving into a wall, a peer or a mover (or walked into)."
    params = {}
    values = {
        "contacts": "contact episodes of any kind",
        "robot": "robot-robot contacts",
        "static": "robot-wall/rack contacts (the robot's map was wrong and it didn't see it)",
        "actor": "robot-mover contacts, whoever moved",
        "driven": "contacts where the ROBOT moved into something: its local layer didn't "
                  "stop it in time (the rest are movers walking into a robot)",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.n = {"robot": 0, "static": 0, "actor": 0}
        self.driven = 0

    def on_event(self, ev):
        self.n[ev["kind"]] = self.n.get(ev["kind"], 0) + 1
        self.driven += ev.get("by", "robot") == "robot"

    def result(self):
        return {"contacts": sum(self.n.values()), **self.n, "driven": self.driven}


@register
class Comms(Metric):
    name = "comms"
    version = 1
    streams = {"net"}
    summary = "Delivery ratio and latency per traffic kind, channel load, reachability."
    params = {}
    values = {
        "status_delivery": "share of /fleet/status copies delivered",
        "tasks_delivery": "share of /fleet/tasks copies delivered (reliable)",
        "map_delivery": "share of map patch copies delivered",
        "obstacle_delivery": "share of obstacle notice copies delivered",
        "tasks_p95_ms": "worst 1-s window's p95 /fleet/tasks latency, ms",
        "status_p95_ms": "worst 1-s window's p95 /fleet/status latency, ms",
        "load_max": "highest per-node channel load factor seen (1.0 = saturated)",
        "load_mean": "mean over samples of the fleet's mean load factor",
        "reach_min": "lowest share of node pairs with a route",
        "bytes_per_robot_s": "mean bytes offered per robot per second, all kinds",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.tot: dict[str, list] = {}
        self.p95: dict[str, float] = {}
        self.loads, self.load_max, self.reach_min = [], 0.0, None
        self.bytes, self.t0, self.t1 = 0, None, None

    def on_event(self, ev):
        self.t0 = ev["t"] if self.t0 is None else self.t0
        self.t1 = ev["t"]
        for kind, st in ev["kinds"].items():
            t = self.tot.setdefault(kind, [0, 0])
            t[0] += st["sent"]
            t[1] += st["delivered"]
            self.bytes += st["bytes"]
            if st.get("p95_ms") is not None:
                self.p95[kind] = max(self.p95.get(kind, 0.0), st["p95_ms"])
        self.loads.append(ev["load_mean"])
        self.load_max = max(self.load_max, ev["load_max"])
        r = ev.get("reach_frac")
        if r is not None:
            self.reach_min = r if self.reach_min is None else min(self.reach_min, r)

    def _ratio(self, kind):
        t = self.tot.get(kind)
        return round(t[1] / t[0], 4) if t and t[0] else None

    def result(self):
        span = (self.t1 - self.t0) if self.t0 is not None and self.t1 > self.t0 else None
        robots = max(1, len(self.ctx.robots))
        return {
            "status_delivery": self._ratio("status"),
            "tasks_delivery": self._ratio("tasks"),
            "map_delivery": self._ratio("map"),
            "obstacle_delivery": self._ratio("obstacle"),
            "tasks_p95_ms": self.p95.get("tasks"),
            "status_p95_ms": self.p95.get("status"),
            "load_max": round(self.load_max, 3),
            "load_mean": round(statistics.fmean(self.loads), 3) if self.loads else None,
            "reach_min": self.reach_min,
            "bytes_per_robot_s": round(self.bytes / span / robots, 1) if span else None,
        }


@register
class MapDivergence(Metric):
    name = "map_divergence"
    version = 1
    streams = {"belief"}
    summary = "How far each robot's own map is from the truth, over time."
    params = {}
    values = {
        "final_known_free": "fleet mean share of the true free floor each robot knows, at the end",
        "mean_known_free": "the same, averaged over every sample",
        "final_wrong_free": "fleet total of obstacle-surface cells believed free, at the end "
                            "(the dangerous kind: a blockage nobody told it about)",
        "max_wrong_free": "worst single robot's believed-free obstacle-surface cells, any sample",
        "final_wrong_occ": "fleet total of cells believed occupied that are free, at the end",
        "final_stale": "fleet total of cells where a robot differs from the NEWEST observation "
                       "any robot holds, at the end (what map sharing should drive to ~0)",
        "mean_stale": "fleet mean stale cells per robot, averaged over every sample",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.means: list[float] = []
        self.stale: list[float] = []
        self.last = None
        self.max_wrong_free = 0

    def on_event(self, ev):
        rows = ev["robots"].values()
        if not rows:
            return
        self.means.append(statistics.fmean(r["known_free"] for r in rows))
        self.stale.append(statistics.fmean(r.get("stale", 0) for r in rows))
        self.max_wrong_free = max(self.max_wrong_free, max(r["wrong_free"] for r in rows))
        self.last = ev

    def result(self):
        if self.last is None:
            return {k: None for k in self.values}
        rows = self.last["robots"].values()
        return {
            "final_known_free": round(statistics.fmean(r["known_free"] for r in rows), 4),
            "mean_known_free": round(statistics.fmean(self.means), 4),
            "final_wrong_free": sum(r["wrong_free"] for r in rows),
            "max_wrong_free": self.max_wrong_free,
            "final_wrong_occ": sum(r["wrong_occ"] for r in rows),
            "final_stale": sum(r.get("stale", 0) for r in rows),
            "mean_stale": round(statistics.fmean(self.stale), 1),
        }


@register
class Nav(Metric):
    name = "nav"
    version = 1
    streams = {"nav"}
    summary = "Replans by cause, blocking and give-ups of the local layer."
    params = {}
    values = {
        "replans": "replans caused by something (sensing, a peer's patch, a blockage, a retry); "
                   "the periodic backstop replan is counted apart",
        "replans_periodic": "backstop replans every nav.replan_period_s while driving",
        "replans_sensed": "replans because the robot's own sensing changed its path",
        "replans_shared": "replans because a peer's map patch changed its path",
        "replans_blocked": "replans around something the local layer stopped for",
        "blocked": "times the local layer stopped the robot",
        "blocked_s": "total sim seconds stopped and then released",
        "gave_up": "legs abandoned (blocked too long, or no path)",
        "recoveries": "random recovery pauses (nav.hiccup_rate_hz)",
        "empty_shelf": "picks that failed because another robot already had the item",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.c = {k: 0 for k in self.values}
        self.c["blocked_s"] = 0.0

    def on_event(self, ev):
        e = ev["event"]
        if e == "replan":
            cause = ev.get("cause")
            if cause != "periodic":
                self.c["replans"] += 1
            key = f"replans_{cause}"
            if key in self.c:
                self.c[key] += 1
        elif e == "blocked":
            self.c["blocked"] += 1
        elif e == "unblocked":
            self.c["blocked_s"] += ev.get("after_s", 0.0)
        elif e == "gave_up":
            self.c["gave_up"] += 1
        elif e == "recovery":
            self.c["recoveries"] += 1
        elif e == "empty_shelf":
            self.c["empty_shelf"] += 1

    def result(self):
        return {**self.c, "blocked_s": round(self.c["blocked_s"], 1)}
