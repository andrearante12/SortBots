"""dispatch.method: central_greedy | central_hungarian — one dispatcher at the
gateway assigns, robots execute.

The dispatcher runs at network node 0 (sim.GW) and talks to robots over the
same network model as everything else:

    gateway -> robot   Dispatch {task_id, pickup, dropoff[, carrying]},
                       Cancel {task_id}, Query {tasks}       reliable, kind `tasks`
    robot -> gateway   Report {seq, cur, state, queue, finals[, outcomes]}
                                                             reliable, kind `tasks`
    robot -> gateway   /fleet/status (pose, mode)            as for every method

so it pays airtime for its traffic, its messages are late or lost under
load and partitions, and it stops assigning when the gateway is cut off:
part of what gets measured. With `info.dispatch: truth` the /fleet/tasks
messages arrive instantly and always (sim.Sim.handle_actions) and the
dispatcher reads each robot's true pose and liveness instead of Status.

Robots keep running the same task FSM. There is no AWAIT_PICK: a central
planner is its own single source of truth, so it needs no confirmed-pick
handshake. What stands in for it is the physical rule "whoever holds the
item keeps the task": a robot refuses a Cancel for a task whose item is on
board and reports instead, and the dispatcher then takes the task back
from whoever else it gave it to.

Bookkeeping that keeps the two sides consistent without acks on every
message: every gateway -> robot message carries a per-robot sequence number
and every Report echoes the highest one the robot has seen. Messages on one
pair arrive in order, so a Report with seq >= s reflects message s, or s was
dropped (a reliable message is dropped only once retries outlast the
transport lease). Anything still unexplained is settled by a Query, whose
answer is authoritative for the tasks it names: held, finished (and how),
or unknown (the Dispatch never arrived, or the robot rebooted).

A robot is lost when the dispatcher hasn't heard it for fleet_tasks
status_timeout_s (6 s). Its tasks that were not yet picked are re-planned
(and cancelled on it, in case only its Status was lost); a task whose item
is on it stays with it, since nobody else can deliver it. A lost robot
keeps working what it has; when it is heard again it is queried and the
two views are merged (it keeps a task nobody else took, it is cancelled off
one somebody else now has, unless it already holds the item).

Greedy (FCFS to the nearest idle robot): on every event (task announced,
robot finished/failed a task, robot lost/back/first heard) and every
resolve_period_s, give the oldest open task to the idle robot that reaches
its pickup soonest; repeat while both remain. Never reassigns.

Rolling Hungarian: on the same triggers, solve (robots) x (tasks not yet
picked up: open or assigned and not yet at the shelf) minimising total drive
time to pickup. A robot still carrying an item (or picking one) counts as
free at that dropoff after its remaining drive and dwell, so it may get one
queued task; every robot gets at most one task per solve. A task stays with
its current robot unless another assignment saves at least reassign_gain_s
(a bonus on current pairs, so a change must pay for itself; without it the
assignment flips back and forth as robots move). A reassignment is a
Cancel to the old robot and a Dispatch to the new one. A task whose robot
is at the shelf (PICKING) is never moved: the Cancel could arrive after
the pick.

Drive times come from the true static map at robots.max_speed_mps
(costs.py); a blockage is known only through the failed legs it causes,
unless `info.map: truth`.

Robot types (`robots.types`): a task that requires capabilities goes only to
robots whose type has them all (both methods), and every robot is priced at
its own max speed, pick and place times. In a fleet whose robots differ in
those, a pair also costs the rest of the task after the pickup is reached
(pick, carry at that robot's speed, place), so the choice is who would
DELIVER soonest, as the auction's bids then are. For the Hungarian solve
that term is taken relative to the task's best eligible robot: a constant
per task would change which tasks win the robots when there are more tasks
than robots, and only the difference between robots should count. With
identical robots the term is zero and the costs are exactly the old ones.
"""
from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import Method
from . import hungarian
from .costs import DriveTimes

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "nodes"))
import fleet_tasks as ft  # noqa: E402

GW = 0
ON_BOARD = ("NAV_TO_DROPOFF", "PLACING")
FINAL = ("DONE", "FAILED", "CANCELLED")
BIG = 1e6  # a forbidden pair for the solver (drive times are < 1e4 s)
COMMITTED_COST = -1.0  # what Status carries for a task with the item on board (protocol.py)


def _tid(s) -> int | None:
    return ft.FleetTasks._fleet_tid(s)


def _name(tid: int) -> str:
    return f"{ft.TASK_PREFIX}{tid}"


# ---------------------------------------------------------------------------
# robot side
# ---------------------------------------------------------------------------

class CentralClient:
    """A robot's task side under a central dispatcher: Dispatch/Cancel from
    the gateway become the FSM's dispatch/cancel; the FSM's status changes
    become Reports, coalesced to one per step."""

    def __init__(self, robot, method: "Central"):
        sim = robot.sim
        self.robot = robot
        self.sim = sim
        self.num = robot.node
        bat = sim.ft_cfg["battery"].get("sim_start_pct")
        self.battery_pct = None if bat is None else float(bat)
        drain = robot.profile["battery_drain_pct_per_s"]  # robot type; None = fleet-wide
        self.drain_pct_s = float(sim.params.battery_drain_pct_per_s if drain is None else drain)
        self.seen_seq = 0
        self.cur: int | None = None
        self.state = "IDLE"
        self.queue: list[int] = []
        self.at: dict[int, str] = {}         # task -> its last active state (where a FAILED was)
        self.outcomes: dict[int, list] = {}  # task -> [final, at], for Query answers
        self._finals: list[list] = []        # since the last Report
        self._asked: list[int] | None = None
        self._dirty = False
        self._actions: list[tuple] = []

    # ---- RobotAgent -------------------------------------------------------

    def set_pose(self, xy, now) -> None:
        pass

    def tick(self, now) -> None:
        pass

    def on_peer_status(self, rid, now, *, battery_pct=None, mode=None) -> None:
        pass

    def heard_network(self, now, peer) -> None:
        pass

    def owned(self) -> list:
        out = []
        if self.cur is not None:
            out.append((self.cur, COMMITTED_COST if self.state in ON_BOARD else 0.0))
        out += [(t, 0.0) for t in self.queue]
        return out

    def drive(self, dt, moving) -> None:
        if self.battery_pct is not None and moving:
            self.battery_pct = max(0.0, self.battery_pct - dt * self.drain_pct_s)

    def drain(self) -> list:
        out, self._actions = self._actions, []
        return out

    def held(self) -> set:
        return ({self.cur} if self.cur is not None else set()) | set(self.queue)

    def on_task_status(self, raw: str, now: float) -> None:
        st = json.loads(raw)
        tid, state = _tid(st.get("task_id")), st.get("state")
        queue = [q for q in (_tid(x.get("task_id")) for x in st.get("queue") or []) if q is not None]
        rej = st.get("rejected") if isinstance(st.get("rejected"), dict) else None
        if rej is not None and _tid(rej.get("task_id")) is not None:
            self._final(_tid(rej.get("task_id")), "FAILED", "rejected")
        if tid is not None and state in FINAL:
            self._final(tid, state, self.at.pop(tid, "queued"))
            tid, state = None, "IDLE"
        elif tid is not None:
            self.at[tid] = state
        if (tid, state, queue) != (self.cur, self.state, self.queue):
            self.cur, self.state, self.queue = tid, state, queue
            self._mark()

    def _final(self, tid: int, state: str, at: str) -> None:
        self.outcomes[tid] = [state, at]
        self._finals.append([tid, state, at])
        self._mark()

    def on_message(self, sender: int, m: dict, now: float) -> None:
        if sender != GW:
            return
        self.seen_seq = max(self.seen_seq, int(m.get("seq", 0)))
        kind = m["type"]
        tid = m.get("task_id")
        if kind == "Dispatch" and tid not in self.held():
            self._actions.append(("dispatch", {"task_id": _name(tid), "pickup": m["pickup"],
                                               "dropoff": m["dropoff"],
                                               **({"carrying": True} if m.get("carrying") else {})}))
        elif kind == "Cancel" and tid in self.held():
            # The item is on board: nobody else can deliver it. Refuse and
            # report, so the dispatcher takes it back from whoever else it
            # went to (a cancel crossing the pick on a slow link).
            if not (tid == self.cur and self.state in ON_BOARD):
                self._actions.append(("cancel", _name(tid)))
        elif kind == "Query":
            self._asked = list(m.get("tasks") or [])
        self._mark()  # every message is answered: the Report echoes its seq

    # ---- Reports ------------------------------------------------------------

    def _mark(self) -> None:
        # One Report per step, sent once the step is over: a dispatch to an
        # idle robot changes its status twice in one step (queued, then
        # NAV_TO_PICKUP), and two frames cost twice the airtime of one.
        if not self._dirty:
            self._dirty = True
            self.sim.at(self.sim.now, self._flush)

    def _flush(self) -> None:
        self._dirty = False
        if not self.robot.alive or self.robot.core is not self:
            return
        m = {"type": "Report", "to": GW, "seq": self.seen_seq, "cur": self.cur,
             "state": self.state, "queue": list(self.queue), "finals": self._finals}
        self._finals = []
        if self._asked is not None:
            held = self.held()
            m["outcomes"] = {str(t): self.outcomes[t] for t in self._asked
                             if t in self.outcomes and t not in held}
            m["unknown"] = [t for t in self._asked if t not in held and t not in self.outcomes]
            self._asked = None
        self._actions.append(("publish", ft.encode(self.num, m)))
        self.sim.handle_actions(self.robot, self.drain())


# ---------------------------------------------------------------------------
# gateway side
# ---------------------------------------------------------------------------

@dataclass
class CTask:
    tid: int
    pickup: str
    dropoff: str
    at: float
    state: str = "open"        # open | assigned | picked | elsewhere | done
    robot: int | None = None   # robot index k
    sub: str | None = None     # what the robot last said: queued, NAV_TO_PICKUP, PICKING, ...
    seq: int = 0               # the Dispatch that gave it to `robot`
    sent_at: float = -math.inf
    retry_at: float | None = None  # a failed dropoff with the item on board: re-dispatch then
    requires: tuple = ()           # capabilities a robot needs to be given it (robot types)


@dataclass
class RView:
    k: int
    heard: bool = False
    lost: bool = False
    last_heard: float = -math.inf
    xy: tuple | None = None
    mode: str | None = None
    battery_pct: float | None = None
    seq: int = 0                                  # last message sequence number sent
    out: dict = field(default_factory=dict)       # task -> (kind, seq, sent_at), latest about it
    query: tuple | None = None                    # (seq, sent_at, back) of the open Query
    mismatch_since: float | None = None
    next_query: float = -math.inf


class CentralDispatcher:
    """The gateway node under a central method (Sim.gw). Subclasses decide
    in `solve`."""

    def __init__(self, sim):
        self.sim = sim
        c = sim.cfg["dispatch"]
        self.gain = float(c["reassign_gain_s"])
        self.period = float(c["resolve_period_s"])
        self.truth = sim.cfg["info"]["dispatch"] == "truth"
        self.timeout = float(sim.params.status_timeout_s)
        self.backoff = float(sim.ft_cfg["failed_backoff_s"])
        self.lease = float(sim.cfg["network"]["transport_lease_s"])
        # Robot types: a central planner knows its fleet's specs. Speeds go
        # to the drive-time lookups only when some robot differs from
        # robots.max_speed_mps, and the rest-of-task term only when robots
        # differ at all, so an untyped run computes exactly what it did.
        self.prof = sim.profiles
        self.caps = [frozenset(p["capabilities"]) for p in self.prof]
        base_v = float(sim.cfg["robots"]["max_speed_mps"])
        self.own_speed = any(p["max_speed_mps"] != base_v for p in self.prof)
        self.mixed = len({(p["max_speed_mps"], p["pick_s"], p["place_s"]) for p in self.prof}) > 1
        self.costs = DriveTimes(sim, live=sim.cfg["info"]["map"] == "truth")
        self.tasks: dict[int, CTask] = {}
        self.active: dict[int, CTask] = {}  # not done: what every per-robot lookup scans
        self.robots = [RView(k) for k in range(sim.n)]
        self.failed: dict[tuple, float] = {}  # (k, task) -> not to k again before this
        self._next_id = 0
        self._dirty = False
        self._next_solve = 0.0
        self._actions: list[tuple] = []
        self.now = 0.0
        self.stats = {"solves": 0, "solve_wall_s": 0.0, "dispatches": 0, "cancels": 0,
                      "reassigned": 0, "queries": 0, "lost": 0}

    # ---- the gateway API --------------------------------------------------

    def request(self, pickup: str, dropoff: str, now: float, requires=()) -> int | None:
        self.now = now
        wps = self.sim.waypoints
        if pickup not in wps or dropoff not in wps:
            self._log(f"rejected task: unknown waypoint in {pickup!r} -> {dropoff!r}")
            return None
        self._next_id += 1
        tid = self._next_id
        self.tasks[tid] = self.active[tid] = CTask(tid, pickup, dropoff, now,
                                                   requires=tuple(requires or ()))
        self._dirty = True
        self._log(f"announced task {tid}: {pickup} -> {dropoff}")
        return tid

    def drain(self) -> list:
        out, self._actions = self._actions, []
        return out

    def on_status(self, k: int, now: float, battery_pct, mode, xy) -> None:
        """robot_k's /fleet/status reached the gateway."""
        self.now = now
        rv = self.robots[k]
        rv.battery_pct, rv.mode = battery_pct, mode
        if self.truth:
            return  # pose and liveness come from the robot itself (tick)
        rv.xy = xy
        self._heard(rv, now)
        # Backstop for a lost Report: the robot says it isn't working, we
        # think it has work. Asked only if that lasts (a Status sent just
        # before a Dispatch arrived says the same).
        mine = [t for t in self._mine(k) if t.retry_at is None]
        if mine and mode != "task":
            if rv.mismatch_since is None:
                rv.mismatch_since = now
            elif now - rv.mismatch_since >= 2.0:
                self._query(rv, now)
        else:
            rv.mismatch_since = None

    def on_message(self, sender: int, m: dict, now: float) -> None:
        self.now = now
        if m.get("type") != "Report" or not 1 <= sender <= len(self.robots):
            return
        rv = self.robots[sender - 1]
        self._heard(rv, now)
        self._reconcile(rv, m, now)

    def tick(self, now: float) -> None:
        self.now = now
        if self.truth:
            for rv, r in zip(self.robots, self.sim.robots):
                rv.xy = (r.x, r.y)
                if r.alive:
                    self._heard(rv, now)
                elif rv.heard and not rv.lost:
                    self._lose(rv, now)
        else:
            for rv in self.robots:
                if rv.heard and not rv.lost and now - rv.last_heard > self.timeout:
                    self._lose(rv, now)
        for t in self.active.values():
            if t.retry_at is not None and now >= t.retry_at and t.state == "picked":
                rv = self.robots[t.robot]
                if not rv.lost:
                    t.retry_at = None
                    self._send_dispatch(rv, t, now, carrying=True)
        if any(rv.query is not None and rv.query[2] and now - rv.query[1] <= self.lease + 1.0
               for rv in self.robots):
            return  # a robot just back hasn't said what it holds yet
        if self._dirty or now >= self._next_solve:
            self._dirty = False
            self._next_solve = now + self.period
            t0 = time.perf_counter()
            self.solve(now)
            self.stats["solves"] += 1
            self.stats["solve_wall_s"] += time.perf_counter() - t0

    def task_done(self, tid: int) -> bool:
        t = self.tasks.get(tid)
        return t is not None and t.state == "done"

    def board(self) -> dict:
        now = self.now
        names = {"open": "open", "assigned": "claimed", "picked": "claimed",
                 "elsewhere": "claimed", "done": "done"}
        tasks = [{"task_id": t.tid, "pickup": t.pickup, "dropoff": t.dropoff,
                  "state": names[t.state], "owner": None if t.robot is None else f"robot_{t.robot}",
                  "committed": t.state == "picked", "age_s": round(now - t.at, 1)}
                 for t in self.tasks.values()]
        robots = []
        for rv in self.robots:
            if not rv.heard:
                continue
            mine = self._mine(rv.k)
            robots.append({"id": f"robot_{rv.k}", "alive": not rv.lost,
                           "last_heard_s": round(now - rv.last_heard, 1),
                           "battery_pct": rv.battery_pct, "mode": rv.mode,
                           "owned": [t.tid for t in mine],
                           "carrying": [t.tid for t in mine if t.state == "picked"]})
        return {"t": round(now, 2), "robots": robots, "tasks": tasks, "alarms": []}

    # ---- the fleet view ----------------------------------------------------

    def _mine(self, k: int) -> list:
        return [t for t in self.active.values() if t.robot == k and t.state in ("assigned", "picked")]

    @staticmethod
    def _locked(t: CTask) -> bool:
        """Not to be moved: the item is on the robot, or about to be."""
        return t.state == "picked" or (t.state == "assigned" and t.sub == "PICKING")

    def _heard(self, rv: RView, now: float) -> None:
        rv.last_heard = now
        if not rv.heard:
            rv.heard = True
            self._dirty = True
        if rv.lost:
            rv.lost = False
            self._dirty = True
            self._log(f"robot_{rv.k} back")
            # It kept working while we couldn't hear it: ask what it holds
            # now, rather than wait for its next status change. Planning
            # waits for the answer (tick), or a partition heal hands its
            # tasks to someone else a moment before it says it still has them.
            self._query(rv, now, force=True, back=True)

    def _lose(self, rv: RView, now: float) -> None:
        rv.lost = True
        self.stats["lost"] += 1
        self._dirty = True
        moved = []
        for t in self._mine(rv.k):
            if not self._locked(t):
                self._reopen(t)
                # In case only its Status was lost: the cancel reaches it and
                # it stops, instead of racing whoever gets the task next.
                self._send_cancel(rv, t.tid, now)
                moved.append(t.tid)
        self._log(f"robot_{rv.k} lost; re-planning {moved}")

    def _reopen(self, t: CTask) -> None:
        t.state, t.robot, t.sub, t.seq, t.retry_at = "open", None, None, 0, None
        self._dirty = True

    def _adopt(self, t: CTask, rv: RView, sub: str, now: float) -> None:
        """robot rv holds t and keeps it."""
        if t.robot is not None and t.robot != rv.k and t.state == "assigned":
            self._send_cancel(self.robots[t.robot], t.tid, now)
        t.robot, t.sub, t.seq, t.retry_at = rv.k, sub, 0, None
        t.state = "picked" if sub in ON_BOARD else "assigned"
        rv.out.pop(t.tid, None)
        self._dirty = True

    def _reconcile(self, rv: RView, m: dict, now: float) -> None:
        k, seq = rv.k, int(m.get("seq", 0))
        answer = rv.query is not None and seq >= rv.query[0] and "unknown" in m
        if answer:
            rv.query = None
            self._dirty = True
        finals = [tuple(f) for f in m.get("finals") or []]
        finals += [(int(t), *o) for t, o in (m.get("outcomes") or {}).items()]
        done_here = set()
        for tid, final, at in finals:
            done_here.add(tid)
            self._final(rv, tid, final, at, now)
        held = []
        if m.get("cur") is not None:
            held.append((int(m["cur"]), m.get("state")))
        held += [(int(q), "queued") for q in m.get("queue") or []]
        held_ids = {tid for tid, _ in held}
        for tid, sub in held:
            t = self.tasks.get(tid)
            msg = rv.out.get(tid)
            if t is None:
                self._send_cancel(rv, tid, now)
                continue
            if t.robot == k and t.state in ("assigned", "picked"):
                t.sub = sub
                if sub in ON_BOARD:
                    t.state = "picked"
                if msg is not None and msg[0] == "dispatch" and seq >= msg[1]:
                    del rv.out[tid]
                continue
            if sub in ON_BOARD:  # whoever holds the item keeps the task
                self._adopt(t, rv, sub, now)
            elif msg is not None and msg[0] == "cancel" and msg[1] > seq \
                    and now - msg[2] <= self.lease + 1.0:
                continue  # our Cancel is on its way
            elif t.state == "open" and not any(not self._locked(x) for x in self._mine(k)):
                self._adopt(t, rv, sub, now)  # nobody else took it meanwhile: keep it there
            else:
                self._send_cancel(rv, tid, now)
        for tid, msg in list(rv.out.items()):
            if msg[0] == "cancel" and tid not in held_ids and seq >= msg[1]:
                del rv.out[tid]  # the cancel took effect
        need_query = False
        for t in self._mine(k):
            if t.tid in held_ids or t.tid in done_here or t.retry_at is not None:
                continue
            if t.seq > seq:  # our Dispatch hasn't reached it yet
                if now - t.sent_at > self.lease + 1.0:
                    need_query = True  # ...and never will: reliable traffic gives up at the lease
                continue
            if answer and t.tid in (m.get("unknown") or []):
                if t.state == "picked":
                    t.state, t.robot = "elsewhere", None  # it forgot (a reboot); the item is lost
                else:
                    self._reopen(t)  # the Dispatch was lost
            else:
                need_query = True
        if need_query:
            self._query(rv, now)

    def _final(self, rv: RView, tid: int, final: str, at: str, now: float) -> None:
        t = self.tasks.get(tid)
        if t is None:
            return
        msg = rv.out.pop(tid, None)
        ours = msg is not None and msg[0] == "cancel"
        if final == "DONE":
            if t.state != "done":
                if t.robot is not None and t.robot != rv.k and t.state == "assigned":
                    self._send_cancel(self.robots[t.robot], tid, now)
                t.state, t.robot, t.retry_at = "done", rv.k, None
                self.active.pop(tid, None)
                self._dirty = True
        elif t.robot != rv.k or t.state not in ("assigned", "picked"):
            return  # not ours on that robot any more: nothing to undo
        elif final == "CANCELLED":
            if not ours:
                self._reopen(t)
        elif at in ON_BOARD:
            # The dropoff leg failed with the item on board: nobody else can
            # deliver it. Try again after the backoff (fleet_tasks' rule).
            t.state, t.sub, t.retry_at = "picked", None, now + self.backoff
            self._dirty = True
        elif at == "PICKING":
            # The shelf was empty: a robot we lost track of took the item.
            # It reports the task when it is heard again.
            t.state, t.robot, t.sub = "elsewhere", None, None
            self._dirty = True
        else:
            # Couldn't reach the pickup (or the waypoint is unknown there):
            # not to this robot again for a while, so it can reach another.
            self.failed[(rv.k, tid)] = now + self.backoff
            self._reopen(t)

    # ---- sending ------------------------------------------------------------

    def _send(self, rv: RView, m: dict) -> int:
        rv.seq += 1
        self._actions.append(("publish", ft.encode(GW, {**m, "to": rv.k + 1, "seq": rv.seq})))
        return rv.seq

    def _send_dispatch(self, rv: RView, t: CTask, now: float, carrying: bool = False) -> None:
        seq = self._send(rv, {"type": "Dispatch", "task_id": t.tid, "pickup": t.pickup,
                              "dropoff": t.dropoff, **({"carrying": True} if carrying else {})})
        t.robot, t.seq, t.sent_at, t.sub = rv.k, seq, now, "queued"
        t.state = "picked" if carrying else "assigned"
        rv.out[t.tid] = ("dispatch", seq, now)
        self.stats["dispatches"] += 1

    def _send_cancel(self, rv: RView, tid: int, now: float) -> None:
        seq = self._send(rv, {"type": "Cancel", "task_id": tid})
        rv.out[tid] = ("cancel", seq, now)
        self.stats["cancels"] += 1

    def _query(self, rv: RView, now: float, force: bool = False, back: bool = False) -> None:
        if rv.query is not None and now - rv.query[1] <= self.lease + 1.0 and not force:
            return  # one open at a time, asked again once it must have been lost
        if not force and now < rv.next_query:
            return
        tasks = sorted(t.tid for t in self._mine(rv.k))
        seq = self._send(rv, {"type": "Query", "tasks": tasks})
        rv.query = (seq, now, back)
        rv.next_query = now + 5.0
        rv.mismatch_since = None
        self.stats["queries"] += 1

    def _log(self, text: str) -> None:
        self._actions.append(("log", text))

    # ---- planning -----------------------------------------------------------

    def _available(self) -> list:
        return [rv for rv in self.robots if rv.heard and not rv.lost and rv.xy is not None]

    def _allowed(self, k: int, t: CTask, now: float) -> bool:
        """robot_k may be given t: its type has what t requires, and it did
        not just fail t."""
        if t.requires and not self.caps[k].issuperset(t.requires):
            return False
        until = self.failed.get((k, t.tid))
        return until is None or now >= until

    def _speed(self, k: int) -> float | None:
        return self.prof[k]["max_speed_mps"] if self.own_speed else None

    def _speeds(self, rvs) -> list | None:
        return [self.prof[rv.k]["max_speed_mps"] for rv in rvs] if self.own_speed else None

    def _rest(self, rvs, t: CTask) -> np.ndarray:
        """Per robot: pick, the carry pickup -> dropoff at its own speed, and
        place. Only called for a fleet whose robots differ (self.mixed)."""
        legs: dict = {}
        out = []
        for rv in rvs:
            p = self.prof[rv.k]
            v = p["max_speed_mps"]
            if v not in legs:
                legs[v] = self.costs.between(t.pickup, t.dropoff, v)
            out.append(p["pick_s"] + legs[v] + p["place_s"])
        return np.array(out, dtype=float)

    def _assign(self, plan: dict, now: float) -> None:
        """plan: task -> robot index or None. Cancels go out before the
        Dispatches, so a task never has two robots told to fetch it."""
        changes = []
        for tid, k in plan.items():
            t = self.tasks[tid]
            old = t.robot if t.state == "assigned" else None
            if old == k:
                continue
            changes.append((t, old, k))
        for t, old, _k in changes:
            if old is not None:
                self._send_cancel(self.robots[old], t.tid, now)
                self.stats["reassigned"] += 1
        for t, old, k in changes:
            if k is None:
                self._reopen(t)
                self._log(f"task {t.tid} taken off robot_{old}")
            else:
                self._send_dispatch(self.robots[k], t, now)
                self._log(f"task {t.tid} -> robot_{k}" + (f" (was robot_{old})" if old is not None
                                                          else ""))

    def solve(self, now: float) -> None:  # pragma: no cover - interface
        raise NotImplementedError


class Greedy(CentralDispatcher):
    def solve(self, now: float) -> None:
        free = [rv for rv in self._available() if not self._mine(rv.k)]
        tasks = sorted((t for t in self.active.values() if t.state == "open"), key=lambda t: t.tid)
        if not free or not tasks:
            return
        cells = [self.costs.snap(rv.xy) for rv in free]
        plan = {}
        for t in tasks:
            if not free:
                break
            s = self.costs.seconds(t.pickup, cells, self._speeds(free))
            if self.mixed:
                s = s + self._rest(free, t)  # who delivers soonest, not who arrives
            for i, rv in enumerate(free):
                if not self._allowed(rv.k, t, now):
                    s[i] = np.inf
            i = int(np.argmin(s))
            if not np.isfinite(s[i]):
                continue  # nobody free can reach it: leave it for later
            plan[t.tid] = free[i].k
            del free[i], cells[i]
        self._assign(plan, now)


class RollingHungarian(CentralDispatcher):
    def _row(self, rv: RView):
        """(cell the robot is free at, seconds until then), at its own
        speed and service times."""
        here = self.costs.snap(rv.xy)
        p, v = self.prof[rv.k], self._speed(rv.k)
        for t in self._mine(rv.k):
            if not self._locked(t):
                continue
            drop = self.costs.snap(self.sim.waypoint_xy(t.dropoff))
            if t.sub == "PICKING":
                off = p["pick_s"] + self.costs.between(t.pickup, t.dropoff, v) + p["place_s"]
            elif t.sub == "PLACING":
                off = p["place_s"]
            else:
                off = float(self.costs.seconds(t.dropoff, [here],
                                               None if v is None else [v])[0]) + p["place_s"]
                if t.retry_at is not None:
                    off += max(0.0, t.retry_at - self.now)
            return drop, off
        return here, 0.0

    def solve(self, now: float) -> None:
        rows = self._available()
        cols = [t for t in self.active.values()
                if t.state == "open" or (t.state == "assigned" and not self._locked(t)
                                         and t.robot is not None)]
        if not rows or not cols:
            return
        cols.sort(key=lambda t: t.tid)
        rinfo = [self._row(rv) for rv in rows]
        cells = [c for c, _ in rinfo]
        off = np.array([o for _, o in rinfo])
        sp = self._speeds(rows)
        cost = np.empty((len(rows), len(cols)))
        for j, t in enumerate(cols):
            cost[:, j] = off + self.costs.seconds(t.pickup, cells, sp)
        ok = np.isfinite(cost)
        for i, rv in enumerate(rows):
            for j, t in enumerate(cols):
                if not self._allowed(rv.k, t, now):
                    ok[i, j] = False
        if self.mixed:
            # The rest of the task, relative to its best eligible robot (the
            # module docstring): only how much longer this robot would take.
            for j, t in enumerate(cols):
                rest = self._rest(rows, t)
                ok[:, j] &= np.isfinite(rest)
                if ok[:, j].any():
                    cost[:, j] += np.where(ok[:, j], rest - rest[ok[:, j]].min(), 0.0)
        c = np.where(ok, cost, BIG)
        row_of = {rv.k: i for i, rv in enumerate(rows)}
        for j, t in enumerate(cols):
            i = row_of.get(t.robot) if t.state == "assigned" else None
            if i is not None and ok[i, j]:
                c[i, j] -= self.gain  # keeping costs nothing; a change must save the gain
        plan = {t.tid: None for t in cols}
        for i, j in enumerate(hungarian.solve(c)):
            if j >= 0 and ok[i, j]:
                plan[cols[j].tid] = rows[i].k
        self._assign(plan, now)


class Central(Method):
    def __init__(self, sim, method: str):
        self.sim = sim
        self.name = method
        self.gateway = (Greedy if method == "central_greedy" else RollingHungarian)(sim)

    def robot_agent(self, robot):
        return CentralClient(robot, self)

    def gateway_status(self, k, now, battery_pct, mode, xy) -> None:
        self.gateway.on_status(k, now, battery_pct, mode, xy)

    def task_done(self, task_id: str) -> bool:
        return self.gateway.task_done(int(task_id[len(ft.TASK_PREFIX):]))

    def summary(self) -> dict:
        s = dict(self.gateway.stats)
        c = self.gateway.costs
        s.update({"method": self.name, "solve_wall_s": round(s["solve_wall_s"], 3),
                  "fields": c.builds, "field_wall_s": round(c.build_wall_s, 3)})
        return s
