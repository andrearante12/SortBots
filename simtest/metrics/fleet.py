"""Fleet-scale metrics: deliveries, contacts, comms, map divergence, nav,
utilisation and per-task lower bounds.

Written for fleetsim (docs/fleetsim.md), which emits these streams on top
of the ones Isaac runs share (gt, dispatch, task). They are ordinary simtest
metrics: deterministic functions of the event stream, versioned, and
re-scorable from a recorded run (`python3 -m fleetsim rescore DIR`).
"""
from __future__ import annotations

import statistics

from . import Metric, register

FINAL = {"DONE", "FAILED", "CANCELLED"}
ACTIVE = {"NAV_TO_PICKUP", "AWAIT_PICK", "PICKING", "NAV_TO_DROPOFF", "PLACING"}


def _pct(xs, q):
    if not xs:
        return None
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(q / 100 * (len(s) - 1))))]


@register
class FleetTasks(Metric):
    name = "fleet_tasks"
    # 2: AWAIT_PICK (confirmed picks, 2026-11-17) counts as working the task.
    # 3: wait_assign_s, wait_pickup_s, backlog_end; per_hour's window starts
    #    at the first request and ends at the last delivery once everything
    #    is delivered (2026-11-26). Dividing by the whole run made the same
    #    delivery_smoke fleet 112/h stopped at tasks_done and 48/h left to
    #    run its 900 s: the idle tail counted as time it failed to deliver.
    version = 3
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
        "mean_s": "mean sim seconds from announce (the operator's request) to DONE",
        "p95_s": "95th percentile announce-to-DONE, sim seconds",
        "per_hour": "deliveries per sim hour: done x 3600 / (end - first announce), where end "
                    "is the last DONE if every announced task was delivered, else the end of "
                    "the run. Neither the lead-in before the first request nor an idle tail "
                    "after the last delivery counts, and work left undone keeps costing until "
                    "the run stops",
        "wait_assign_s": "mean sim seconds from announce to the task's first dispatch, over "
                         "delivered tasks",
        "wait_pickup_s": "mean sim seconds from the first dispatch to the item leaving the "
                         "shelf (the first NAV_TO_DROPOFF), over delivered tasks; with "
                         "wait_assign_s and the carry leg it adds up to mean_s",
        "backlog_end": "announced tasks not delivered when the run ended (unassigned, queued "
                       "or in progress)",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.announced: dict[str, float] = {}
        self.done_at: dict[str, float] = {}
        self.active: dict[str, set] = {}
        self.dispatches: dict[str, int] = {}
        self.first_dispatch: dict[str, float] = {}
        self.picked_at: dict[str, float] = {}
        self.failed = self.cancelled = self.duplicates = 0
        self.end_t = None

    def on_event(self, ev):
        s = ev["s"]
        if s == "announce":
            self.announced[ev["task_id"]] = ev["t"]
        elif s == "dispatch":
            self.dispatches[ev["task_id"]] = self.dispatches.get(ev["task_id"], 0) + 1
            self.first_dispatch.setdefault(ev["task_id"], ev["t"])
        elif s == "end":
            self.end_t = ev["t"]
        elif s == "task":
            tid, st, rid = ev["task_id"], ev["state"], ev["rid"]
            act = self.active.setdefault(tid, set())
            if st == "NAV_TO_DROPOFF":
                # One physical item per task (sim.take_item): the first carry
                # leg is the pick, whichever robot made it.
                self.picked_at.setdefault(tid, ev["t"])
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
        done = [t for t in self.announced if t in self.done_at]
        lat = [self.done_at[t] - self.announced[t] for t in done]
        n = len(self.announced)
        assign = [self.first_dispatch[t] - self.announced[t] for t in done
                  if t in self.first_dispatch]
        pickup = [self.picked_at[t] - self.first_dispatch[t] for t in done
                  if t in self.first_dispatch and t in self.picked_at]
        per_hour = None
        if n and self.end_t is not None:
            t0 = min(self.announced.values())
            t1 = max(self.done_at[t] for t in done) if len(done) == n else self.end_t
            per_hour = round(len(done) / (t1 - t0) * 3600, 1) if t1 > t0 else None
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
            "per_hour": per_hour,
            "wait_assign_s": round(statistics.fmean(assign), 1) if assign else None,
            "wait_pickup_s": round(statistics.fmean(pickup), 1) if pickup else None,
            "backlog_end": n - len(done),
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
    version = 2
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
        "over_cap_s": "seconds (1-Hz samples) in which some node's offered load was past the "
                      "95 % cap; with network.capacity: unlimited, how long the real radio "
                      "would have been saturated",
        "reach_min": "lowest share of node pairs with a route",
        "bytes_per_robot_s": "mean bytes offered per robot per second, all kinds",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.tot: dict[str, list] = {}
        self.p95: dict[str, float] = {}
        self.loads, self.load_max, self.reach_min = [], 0.0, None
        self.over_cap = 0
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
        self.over_cap += ev.get("over_cap", 0.0) > 0  # absent before comms v2
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
            "over_cap_s": self.over_cap,
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
    # 2: robot-caused stops apart, and path-intent yields (2026-11-17).
    # 3: holds short of a single-lane passage (fleetsim/traffic.py, 2026-11-26).
    version = 3
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
        "blocked_robot": "stops for a peer robot (what path intents try to prevent)",
        "blocked_robot_s": "total sim seconds stopped for a peer robot",
        "yields": "holds at a safe spot for a peer's path intent (fleetsim/intent.py)",
        "yield_s": "total sim seconds held for a peer's intent",
        "yield_timeouts": "holds given up after intent.yield_max_s",
        "pass_holds": "holds short of a single-lane passage another robot claimed first "
                      "(traffic.passages), a new claim's settle_s wait included",
        "pass_hold_s": "total sim seconds held short of a passage",
        "pass_hold_timeouts": "passage holds given up after traffic.passages.hold_max_s",
    }
    ROUNDED = ("blocked_s", "blocked_robot_s", "yield_s", "pass_hold_s")

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.c = {k: 0 for k in self.values}
        for k in self.ROUNDED:
            self.c[k] = 0.0
        self._by: dict = {}  # rid -> what its open stop is for

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
            self._by[ev.get("rid")] = ev.get("by")
            if ev.get("by") == "robot":
                self.c["blocked_robot"] += 1
        elif e == "unblocked":
            self.c["blocked_s"] += ev.get("after_s", 0.0)
            if self._by.pop(ev.get("rid"), None) == "robot":
                self.c["blocked_robot_s"] += ev.get("after_s", 0.0)
        elif e == "yield":
            self.c["yields"] += 1
        elif e == "yield_end":
            self.c["yield_s"] += ev.get("after_s", 0.0)
        elif e == "yield_timeout":
            self.c["yield_timeouts"] += 1
        elif e == "pass_hold":
            self.c["pass_holds"] += 1
        elif e == "pass_hold_end":
            self.c["pass_hold_s"] += ev.get("after_s", 0.0)
        elif e == "pass_hold_timeout":
            self.c["pass_hold_timeouts"] += 1
        elif e == "gave_up":
            self.c["gave_up"] += 1
        elif e == "recovery":
            self.c["recoveries"] += 1
        elif e == "empty_shelf":
            self.c["empty_shelf"] += 1

    def result(self):
        return {**self.c, **{k: round(self.c[k], 1) for k in self.ROUNDED}}


# The task FSM's states (fleetsim/robot.py, task_manager's) as shares of a
# robot's time. A FINAL state ends the task and the robot is idle until its
# next one: nothing records IDLE itself, since a robot with no task sends a
# status with no task in it.
PHASE = {"NAV_TO_PICKUP": "to_pickup", "AWAIT_PICK": "at_pickup", "PICKING": "at_pickup",
         "NAV_TO_DROPOFF": "carrying", "PLACING": "placing"}
BUSY = ("to_pickup", "at_pickup", "carrying", "placing")
DRIVING = {"to_pickup", "carrying"}
CAUSES = ("robot", "actor", "notice", "bump", "yield", "passage", "await_pick", "no_path")


class _Bot:
    """One robot's state as the event stream tells it."""

    __slots__ = ("last", "phase", "task", "state", "home", "off", "block", "yielding",
                 "passage", "no_path")

    def __init__(self):
        self.last = 0.0
        self.phase = "idle"
        self.task = self.state = None  # the last task status: (task_id, FSM state)
        self.home = False      # no task, driving home (a `path` with purpose home)
        self.off = False       # killed by a disturbance
        self.block = None      # what the open local-layer stop is for
        self.yielding = False  # holding for a peer's path intent
        self.passage = False   # holding short of a single-lane passage
        self.no_path = False   # plan_failed, no plan since

    def copy(self) -> "_Bot":
        b = _Bot()
        for k in self.__slots__:
            setattr(b, k, getattr(self, k))
        return b


@register
class Utilisation(Metric):
    name = "utilisation"
    # 2: holds short of a single-lane passage, lost_passage_* (2026-11-26).
    version = 2
    streams = {"task", "nav", "path", "disturb", "end"}
    summary = "Where the fleet's robot-time goes: task phases, and time lost by cause."
    params = {}
    values = {
        "robot_s": "robot-seconds in the run: robots x its sim length",
        "idle_frac": "share of robot-time with no task, parked or waiting (going home apart)",
        "to_pickup_frac": "share of robot-time driving to a pickup (NAV_TO_PICKUP)",
        "at_pickup_frac": "share of robot-time at the shelf: awaiting the pick ack "
                          "(AWAIT_PICK) and picking (PICKING)",
        "carrying_frac": "share of robot-time driving an item to its dropoff (NAV_TO_DROPOFF)",
        "placing_frac": "share of robot-time placing (PLACING)",
        "home_frac": "share of robot-time with no task, driving back to its home spot "
                     "(nav.return_home_s); None when the run recorded no `path` events "
                     "(record.paths: false), and that time is then in idle_frac",
        "off_frac": "share of robot-time killed by a disturbance",
        "busy_frac": "share of robot-time working a task (to_pickup + at_pickup + carrying "
                     "+ placing)",
        "lost_s": "robot-seconds lost while working a task, every cause below (and any stop "
                  "cause not listed; no-path time only when it is known)",
        "lost_frac": "lost_s as a share of robot-time",
        "lost_robot_s": "robot-seconds driving a task but stopped by the local layer for a peer "
                        "robot",
        "lost_robot_frac": "lost_robot_s as a share of robot-time",
        "lost_actor_s": "robot-seconds stopped for a mover (a scripted actor)",
        "lost_actor_frac": "lost_actor_s as a share of robot-time",
        "lost_notice_s": "robot-seconds stopped for a peer's obstacle notice",
        "lost_notice_frac": "lost_notice_s as a share of robot-time",
        "lost_bump_s": "robot-seconds stopped for something it physically bumped",
        "lost_bump_frac": "lost_bump_s as a share of robot-time",
        "lost_yield_s": "robot-seconds holding for a peer's path intent (fleetsim/intent.py)",
        "lost_yield_frac": "lost_yield_s as a share of robot-time",
        "lost_passage_s": "robot-seconds held short of a single-lane passage another robot "
                          "claimed first (traffic.passages)",
        "lost_passage_frac": "lost_passage_s as a share of robot-time",
        "lost_await_pick_s": "robot-seconds at the shelf waiting for the majority's pick "
                             "acks (AWAIT_PICK)",
        "lost_await_pick_frac": "lost_await_pick_s as a share of robot-time",
        "lost_no_path_s": "robot-seconds driving a task with no path on its own map "
                          "(plan_failed until the next plan, gave_up or the task's next "
                          "state); None without `path` events",
        "lost_no_path_frac": "lost_no_path_s as a share of robot-time",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.bots: dict[str, _Bot] = {rid: _Bot() for rid in ctx.robots}
        self.phase_s = dict.fromkeys(("idle", "home", "off", *BUSY), 0.0)
        self.lost_s: dict[str, float] = dict.fromkeys(CAUSES, 0.0)
        self.paths = False
        self.end_t = None
        self.t_max = 0.0

    @staticmethod
    def _advance(b: _Bot, t: float, phase_s: dict, lost_s: dict) -> None:
        """Book the time since b.last to what the robot was doing. One cause
        at a time: yielding over a stop (a held robot isn't checked for
        blockers, so a stop open across a hold is the hold's; a passage hold
        likewise), and a stop over
        no path (a replan around a blocker that finds no way out is the
        blocker's doing). Lost time is counted only while working a task: a
        robot stopped on its way home costs no delivery."""
        dt = t - b.last
        if dt <= 0:
            return
        b.last = t
        if b.off:
            phase_s["off"] += dt
            return
        phase_s["home" if b.phase == "idle" and b.home else b.phase] += dt
        if b.state == "AWAIT_PICK":
            cause = "await_pick"
        elif b.phase not in DRIVING:
            return
        elif b.yielding:
            cause = "yield"
        elif b.passage:
            cause = "passage"
        elif b.block is not None:
            cause = b.block
        elif b.no_path:
            cause = "no_path"
        else:
            return
        lost_s[cause] = lost_s.get(cause, 0.0) + dt

    def on_event(self, ev):
        s, t = ev["s"], ev["t"]
        self.t_max = max(self.t_max, t)
        if s == "end":
            self.end_t = t
            return
        if s == "disturb":
            rid = ev.get("kill") or ev.get("revive")
            if not isinstance(rid, str):
                return
            b = self.bots.setdefault(rid, _Bot())
            self._advance(b, t, self.phase_s, self.lost_s)
            if "kill" in ev and ev["on"]:
                b.off = True
            elif "kill" in ev or ev["on"]:  # the kill's end, or a revive
                b.off = False
                if ev.get("fresh"):  # a reboot drops its task without a status (sim._revive)
                    b.phase, b.task, b.state = "idle", None, None
                    b.block, b.no_path, b.home = None, False, False
            return
        rid = ev.get("rid")
        if rid is None:
            return
        b = self.bots.setdefault(rid, _Bot())
        self._advance(b, t, self.phase_s, self.lost_s)
        if s == "task":
            st = ev["state"]
            if (ev["task_id"], st) == (b.task, b.state):
                return  # the same status again: a dispatch or cancel of a queued task
            # Every change of state set a new goal or cleared it
            # (robot._clear_goal), which ends a stop or a planning failure
            # without an `unblocked`.
            b.block, b.no_path, b.home = None, False, False
            b.phase = PHASE.get(st, "idle")
            b.task, b.state = (None, None) if st in FINAL else (ev["task_id"], st)
        elif s == "nav":
            e = ev["event"]
            if e == "blocked":
                b.block = ev.get("by") or "other"
            elif e == "unblocked":
                b.block = None
            elif e == "gave_up":
                b.block, b.no_path, b.home = None, False, False
            elif e == "plan_failed":
                b.no_path = True
            elif e == "yield":
                b.yielding = True
            elif e == "yield_end":
                b.yielding = False
            elif e == "pass_hold":
                b.passage = True
            elif e == "pass_hold_end":
                b.passage = False
        elif s == "path":
            # The one place a recovered plan shows: robot._replan records the
            # new path, and a failed one records none (or an empty one).
            self.paths = True
            if ev.get("pts"):
                b.no_path = False
                if ev.get("purpose") == "home":
                    b.home = True
            elif ev.get("purpose") == "home":
                b.home = False  # arrived, re-tasked, or lost its path home

    def result(self):
        end = self.end_t if self.end_t is not None else self.t_max
        phase_s, lost_s = dict(self.phase_s), dict(self.lost_s)
        for b in self.bots.values():
            self._advance(b.copy(), end, phase_s, lost_s)
        robot_s = end * len(self.bots)
        if robot_s <= 0:
            return {k: None for k in self.values}
        frac = lambda v: round(v / robot_s, 4)  # noqa: E731
        idle = phase_s["idle"] + (0.0 if self.paths else phase_s["home"])
        out = {
            "robot_s": round(robot_s, 1),
            "idle_frac": frac(idle),
            **{f"{p}_frac": frac(phase_s[p]) for p in BUSY},
            "home_frac": frac(phase_s["home"]) if self.paths else None,
            "off_frac": frac(phase_s["off"]),
            "busy_frac": frac(sum(phase_s[p] for p in BUSY)),
        }
        # Without `path` events a failed plan never shows its recovery, so
        # no-path time is unknown, and left out of the total too.
        total = sum(v for c, v in lost_s.items() if c != "no_path" or self.paths)
        out["lost_s"], out["lost_frac"] = round(total, 1), frac(total)
        for c in CAUSES:
            known = c != "no_path" or self.paths
            out[f"lost_{c}_s"] = round(lost_s[c], 1) if known else None
            out[f"lost_{c}_frac"] = frac(lost_s[c]) if known else None
        return out


@register
class Bounds(Metric):
    name = "bounds"
    version = 1
    streams = {"announce", "task_bound", "task"}
    summary = "Per-task lower bounds, and how loaded the scenario is: could any method keep up?"
    params = {}
    values = {
        "lb_mean_s": "mean over announced tasks of the lower bound on its delivery time: the "
                     "shortest drive pickup -> dropoff on the true layout at "
                     "robots.max_speed_mps, plus both dwells (the sim's task_bound event)",
        "stretch": "mean over delivered tasks of delivery time (announce -> DONE) / its lower "
                   "bound; 1.0 only if a robot were already at the shelf and nothing slowed it",
        "ub_per_hour": "fleet capacity if every robot did nothing but back-to-back "
                       "lower-bound tasks: robots x 3600 / lb_mean_s. It ignores the empty "
                       "drive from a dropoff to the next pickup, so it is optimistic",
        "arrival_per_hour": "requests per sim hour over the announce window: (announced - 1) "
                            "x 3600 / (last announce - first announce)",
        "offered_load": "arrival_per_hour / ub_per_hour: > 1 means no method can keep up; "
                        "since ub_per_hour is optimistic, a fleet can be overloaded below 1",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.announced: dict[str, float] = {}
        self.lb: dict[str, float] = {}
        self.done_at: dict[str, float] = {}

    def on_event(self, ev):
        s = ev["s"]
        if s == "announce":
            self.announced[ev["task_id"]] = ev["t"]
        elif s == "task_bound":
            if ev.get("lb_s") is not None:  # None: no path on the true map
                self.lb[ev["task_id"]] = float(ev["lb_s"])
        elif s == "task" and ev["state"] == "DONE":
            self.done_at.setdefault(ev["task_id"], ev["t"])

    def result(self):
        lbs = [self.lb[t] for t in self.announced if t in self.lb]
        lb_mean = statistics.fmean(lbs) if lbs else None
        stretch = [(self.done_at[t] - a) / self.lb[t] for t, a in self.announced.items()
                   if t in self.done_at and self.lb.get(t, 0) > 0]
        n = len(self.ctx.robots)
        ub = n * 3600 / lb_mean if lb_mean and n else None
        ts = sorted(self.announced.values())
        arr = (len(ts) - 1) * 3600 / (ts[-1] - ts[0]) if len(ts) >= 2 and ts[-1] > ts[0] else None
        return {
            "lb_mean_s": round(lb_mean, 1) if lb_mean is not None else None,
            "stretch": round(statistics.fmean(stretch), 2) if stretch else None,
            "ub_per_hour": round(ub, 1) if ub is not None else None,
            "arrival_per_hour": round(arr, 1) if arr is not None else None,
            "offered_load": round(arr / ub, 3) if arr is not None and ub else None,
        }
