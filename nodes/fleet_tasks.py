#!/usr/bin/env python3
"""Fleet task auction on the robots: commsim's Allocator over ROS 2 topics.

There is no central dispatcher. Every robot runs the same allocator
(commsim/core/protocol.py — the code the DES and the loopback harness
validated, imported, not copied), so all robots reach the same winner on
their own: lowest travel-time-plus-battery cost, ties to the lower ID. The
gateway only injects tasks and keeps a board of what it has heard.

    gateway:  /fleet/task_request {"pickup": "shelf_a", "dropoff": "zone_b"}
                -> TaskAnnounce on /fleet/tasks
    robots:   Bid -> (1 s window) -> Claim by the winner
    winner:   /<robot>/dispatch_task -> task_manager.py works it
              /<robot>/task_status NAV_TO_DROPOFF -> commit (item on board:
                                     the task can no longer move, protocol.py)
                                   DONE   -> Complete (fleet-wide)
                                   FAILED -> Release  (re-auctioned), or if
                                     committed: re-dispatched here after the
                                     backoff with "carrying" (dropoff only)
    loser of ownership (lease lost while isolated, out-bid on a partition
    heal, cancelled by the gateway) -> /<robot>/cancel_task

After a restart this node reads task_manager's latched task_status: a task
with the item on board is adopted (claimed back as committed), anything
else task_manager still holds is cancelled so the fleet re-auctions it.

Bid cost is the path length on the fused /map when one has arrived
(nodes/grid_dist.py), else the straight line.

Every task-protocol message travels on ONE reliable topic, `/fleet/tasks`.
Leases are renewed by LeaseBeacons on that same topic, not by
fleet_radio's `/fleet/status`: a single publisher's messages arrive in
order, so a peer can never see a Release and THEN an older status that
still lists the task, which would put the releaser back as owner until its
lease ran out. `/fleet/status` still counts as liveness (who is alive,
whether we are isolated), and fleet_radio copies our owned tasks and battery
into it for anyone watching.

Robot IDs: robot_N is N+1 on the wire, the gateway is 0 — the allocator's
tie-break is the integer, so robot_0 still wins exact ties, matching
fleet_radio's priority order.

The protocol clock is the steady clock, never sim time: leases and the bid
window bound how long a *message* may take, not how far a robot drives.

    python3 nodes/fleet_tasks.py --robot-id robot_0
    python3 nodes/fleet_tasks.py --gateway

Request a task:

    ros2 topic pub --once /fleet/task_request std_msgs/String \\
        '{data: "{\\"pickup\\": \\"shelf_a\\", \\"dropoff\\": \\"zone_a\\"}"}'
    ros2 topic echo /fleet/task_board

The board (latched, 1 Hz) is the operator GUI's whole view of the fleet —
requirement 2026-10-20: battery, heartbeat and which robot holds which task
must reach the GUI no matter what. It is the gateway's OWN protocol state,
not a second opinion: `alive` is the same silence rule the robots apply to
each other (status_timeout_s), so the GUI and the fleet never disagree.

    {"t": <steady s>,
     "robots": [{"id": "robot_0", "alive": true, "last_heard_s": 0.3,
                 "battery_pct": 82.0, "mode": "nav",
                 "owned": [<task_id>...], "carrying": [<task_id>...]}],
     "tasks":  [{"task_id", "pickup", "dropoff", "state", "owner",
                 "committed", "age_s"}],
     "alarms": [{"id", "kind", "robot", "task_id", "t", "active", "acked",
                 "text"}]}

Alarms latch like an annunciator: raised by the protocol (robot_silent,
owner_missing, lease_expired) or by the gateway (task_stalled: nobody has
claimed it for stalled_after_s), `active` while the condition holds, shown
until acknowledged on /fleet/alarm_ack {"id": <id>}, dropped once both.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from commsim.core.protocol import (  # noqa: E402
    BIDDING, CLAIMED, COMMITTED_COST, OPEN, Allocator, Params, TaskView, deliver,
)
from grid_dist import GridDistance  # noqa: E402
from waypoints import load_waypoints  # noqa: E402

DEFAULT_CFG = REPO_ROOT / "configs" / "fleet_tasks.yaml"

TASKS_TOPIC = "/fleet/tasks"
REQUEST_TOPIC = "/fleet/task_request"
CANCEL_TOPIC = "/fleet/task_cancel"
BOARD_TOPIC = "/fleet/task_board"
ALARM_ACK_TOPIC = "/fleet/alarm_ack"

GATEWAY = 0
# task_manager's task_id for an auctioned task; the prefix tells its
# task_status apart from tasks a person dispatched by hand.
TASK_PREFIX = "fleet-"

DEFAULTS = {
    "speed_mps": 0.5,           # Nav2 max_velocity x (configs/nav2_params.yaml)
    "failed_backoff_s": 60.0,
    "pose_stale_s": 5.0,        # no TF this long -> we don't know where we are: no bids
    "reannounce_s": 5.0,
    "gateway_status_hz": 1.0,
    "stalled_after_s": 30.0,    # gateway: an unclaimed task this old raises an alarm
    "battery": {"sim_start_pct": None},
    "protocol": {},
}


def load_config(path: Path | None = None) -> dict:
    cfg = json.loads(json.dumps(DEFAULTS))
    p = path or DEFAULT_CFG
    if p.exists():
        with open(p) as f:
            raw = yaml.safe_load(f) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"{p}: expected a mapping")
        for k, v in raw.items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
    return cfg


def robot_num(robot_id: str) -> int:
    """robot_0 -> 1, robot_1 -> 2, …; 0 is the gateway."""
    if robot_id.startswith("robot_"):
        return int(robot_id.split("_", 1)[1]) + 1
    raise ValueError(f"not a robot id: {robot_id!r}")


def robot_name(num: int | None) -> str | None:
    if num is None:
        return None
    return "gateway" if num == GATEWAY else f"robot_{num - 1}"


def encode(sender: int, m: dict) -> str:
    return json.dumps({**m, "from": sender})


def decode(raw: str) -> tuple[int, dict] | None:
    try:
        d = json.loads(raw)
        sender = int(d.pop("from"))
        if not isinstance(d.get("type"), str):
            return None
        return sender, d
    except (json.JSONDecodeError, KeyError, TypeError, ValueError, AttributeError):
        return None


class FleetTasks:
    """The protocol side of the node, without ROS: feed it messages and the
    clock, read actions back from `drain()`. Actions are tuples:
        ("publish", raw)         -> /fleet/tasks
        ("dispatch", payload)    -> /<robot>/dispatch_task (JSON dict;
                                    "carrying": true = item already on board)
        ("cancel", task_id_str)  -> /<robot>/cancel_task
        ("alloc", payload)       -> /<robot>/task_alloc, on change
        ("log", text)
    """

    def __init__(self, robot_id: str | None, params: Params, waypoints, *,
                 gateway: bool = False, can_bid: bool = True, cfg: dict | None = None,
                 now: float = 0.0):
        cfg = cfg or load_config()
        self.cfg = cfg
        self.gateway = gateway
        self.num = GATEWAY if gateway else robot_num(robot_id)
        # A dict of name -> Waypoint, or a zero-argument callable returning
        # one. The node passes a loader so a pin the operator places on the map
        # resolves without a restart, same as task_manager re-reading it on
        # every dispatch.
        self._waypoints = waypoints
        self.speed = float(cfg["speed_mps"])
        self.pose: tuple[float, float] | None = None
        self._pose_at: float | None = None
        self.grid: GridDistance | None = None
        self.battery_pct: float | None = None
        # Committed tasks whose dropoff leg failed: re-dispatch (carrying) at
        # this time. Releasing them would send another robot to an empty shelf.
        self._retry: dict[int, float] = {}
        self._synced = False  # task_manager's latched status seen (restart adoption)
        # Fleet tasks WE told task_manager to drop. Its CANCELLED for one of
        # these is the echo of our own decision; a CANCELLED for anything else
        # we own came from a person (operator Stop all / Clear tasks / x).
        self._cancelling: set[int] = set()
        self.names: dict[int, tuple[str, str]] = {}
        # (pickup, dropoff) -> steady time until which we do not bid on it
        # again. Without it a robot that cannot do a task (no arm, a pick it
        # keeps fumbling) is still the closest bidder, wins it straight back
        # after every Release, and the task never reaches anyone else.
        self.failed: dict[tuple, float] = {}
        self.now = now
        self._next_id = int(time.time() * 10)
        self._next_reannounce = now
        self._next_heartbeat = now
        self._last_alloc: str | None = None
        self._actions: list[tuple] = []
        # Gateway only (the board): what each robot last told /fleet/status
        # beyond liveness, when each task was announced, and latched alarms.
        self.peer_info: dict[int, dict] = {}
        self._announced: dict[int, float] = {}
        self.alarms: list[dict] = []
        self._alarm_seq = 0
        bids = not gateway and can_bid
        self.alloc = Allocator(self.num, params, self._cost if bids else None,
                               is_gateway=gateway, battery_fn=lambda: self.battery_pct)

    def waypoints(self) -> dict:
        return self._waypoints() if callable(self._waypoints) else self._waypoints

    # ---- inputs -----------------------------------------------------------

    def set_pose(self, xy: tuple[float, float], now: float) -> None:
        self.pose, self._pose_at = xy, now

    def _path_m(self, a, b) -> float:
        """Drive distance a -> b: along the fused map's free cells when we
        have one, else straight line. Not the Nav2 path: asking
        planner_server here would preempt the path request of whatever Nav2
        is executing (ComputePathToPose takes one goal at a time)."""
        if self.grid is not None:
            d = self.grid.path_length(a, b)
            if d is not None:
                return d
        return math.dist(a, b)

    def _cost(self, pickup, dropoff=None, start=None):
        if self.pose is None:
            return None  # no localization yet: we cannot say how far it is
        until = self.failed.get((tuple(pickup), tuple(dropoff or ())))
        if until is not None and self.now < until:
            return None
        # `start` is the last dropoff of what we already own (the allocator
        # bids with the backlog): task_manager works its queue in order.
        origin = start if start is not None else self.pose
        d = self._path_m(origin, pickup)
        leg = self._path_m(pickup, dropoff) if dropoff is not None else 0.0
        return d / self.speed, math.dist(origin, pickup), (d + leg) / self.speed

    def on_wire(self, raw: str, now: float) -> None:
        self.now = now
        got = decode(raw)
        if got is None:
            self._log(f"dropped malformed message: {raw[:120]!r}")
            return
        sender, m = got
        if sender == self.num:
            return  # DDS delivers our own publications back to us
        if m["type"] == "TaskAnnounce" and "pickup_station" in m:
            self.names[int(m["task_id"])] = (m["pickup_station"], m["dropoff_station"])
        try:
            deliver(self.alloc, sender, m, now)
        except (KeyError, TypeError, ValueError) as exc:
            self._log(f"dropped {m.get('type')} from {robot_name(sender)}: {exc}")
        self._flush()

    def on_peer_status(self, robot_id: str, now: float, *, battery_pct: float | None = None,
                       mode: str | None = None) -> None:
        """A /fleet/status heartbeat: liveness (see module docstring), plus,
        at the gateway, the battery and mode the board shows the operator."""
        self.now = now
        try:
            num = robot_num(robot_id)
        except ValueError:
            return
        if num != self.num:
            self.alloc.on_status(num, [], now)
            if self.gateway:
                self.peer_info[num] = {"battery_pct": battery_pct, "mode": mode}
            self._flush()

    @staticmethod
    def _fleet_tid(tid_s) -> int | None:
        if not isinstance(tid_s, str) or not tid_s.startswith(TASK_PREFIX):
            return None
        try:
            return int(tid_s[len(TASK_PREFIX):])
        except ValueError:
            return None

    def on_task_status(self, raw: str, now: float) -> None:
        """task_manager's /<robot>/task_status for an auctioned task."""
        self.now = now
        try:
            st = json.loads(raw)
            tid_s, state = st.get("task_id"), st.get("state")
        except (json.JSONDecodeError, AttributeError):
            return
        if not self._synced:
            self._synced = True
            self._adopt(st, now)
        rej = st.get("rejected") if isinstance(st.get("rejected"), dict) else None
        if rej is not None and self._fleet_tid(rej.get("task_id")) is not None:
            # task_manager refused it (a waypoint renamed or deleted since the
            # announce): same outcome as a failed first leg.
            tid_s, state = rej.get("task_id"), "FAILED"
        tid = self._fleet_tid(tid_s)
        if tid is None:
            return
        if state == "CANCELLED" and tid in self._cancelling:
            self._cancelling.discard(tid)
            return  # the echo of our own cancel
        t = self.alloc.tasks.get(tid)
        if t is None or t.state != CLAIMED or t.owner != self.num:
            return  # we no longer own it (cancelled, out-bid): not ours to report
        if state == "CANCELLED":
            # A person stopped it on this robot. Left alone, we would keep
            # renewing the lease on a task nothing is working, and the board
            # would show it claimed forever. Withdraw it fleet-wide instead:
            # the operator meant "don't do this", not "someone else do it".
            self._retry.pop(tid, None)
            self.alloc.cancel(tid, now)
            self._log(f"task {tid} cancelled on this robot by hand; withdrawn fleet-wide")
            self._flush()
            # The cancel we just broadcast comes back to us as stop_task, which
            # would tell task_manager to drop a task it already dropped.
            self._cancelling.discard(tid)
            self._actions = [a for a in self._actions
                             if a != ("cancel", f"{TASK_PREFIX}{tid}")]
            return
        if state in ("NAV_TO_DROPOFF", "PLACING") and not self.alloc.committed(tid):
            self.alloc.commit(tid, now)
            self._log(f"task {tid}: item picked up, committed")
        elif state == "DONE":
            self._retry.pop(tid, None)
            self.alloc.complete(tid, now)
            self._log(f"task {tid} done")
        elif state == "FAILED":
            until = now + float(self.cfg["failed_backoff_s"])
            if self.alloc.committed(tid):
                # The item is on us: nobody else can finish this. Try the
                # dropoff leg again after the backoff, keeping ownership.
                self._retry[tid] = until
                self._log(f"task {tid} failed with the item on board; "
                          f"retrying the dropoff in {until - now:.0f} s")
            else:
                self.failed[(t.pickup, t.dropoff)] = until
                self.alloc.release(tid, now)
                # Our own view is now OPEN with no rebid round pending.
                # Schedule one for when the backoff ends, or a task only we
                # can do would stay open forever once the other robots have
                # stopped bidding.
                t.rebid_at = until
                self._log(f"task {tid} failed here; released for another robot")
        self._flush()

    def _adopt(self, st: dict, now: float) -> None:
        """First task_status after OUR start: task_manager may have kept
        working through a restart of this node. A task with the item on
        board is ours again (committed: peers accept the claim whatever they
        think). Anything else it holds is cancelled; the fleet re-auctions
        it and we bid fresh."""
        tid = self._fleet_tid(st.get("task_id"))
        state = st.get("state")
        if tid is not None and state in ("NAV_TO_DROPOFF", "PLACING"):
            # task_manager puts the current task's names under "task" (its
            # summary()); top-level keys are the older shape.
            cur = st.get("task") if isinstance(st.get("task"), dict) else st
            pick, drop = cur.get("pickup"), cur.get("dropoff")
            wps = self.waypoints()
            t = TaskView(tid, state=CLAIMED, owner=self.num, owner_cost=COMMITTED_COST,
                         last_renew=now, claimed_at=now)
            if pick in wps and drop in wps:
                self.names[tid] = (pick, drop)
                t.pickup, t.dropoff = self._xy(pick, wps), self._xy(drop, wps)
            self.alloc.tasks[tid] = t
            self.alloc.out.append({"type": "Claim", "task_id": tid, "cost": COMMITTED_COST,
                                   "lease_s": self.alloc.p.task_lease_s})
            self._log(f"adopted task {tid} after restart: item on board")
        elif tid is not None and state not in ("DONE", "FAILED", "CANCELLED", "IDLE"):
            self._cancel_local(tid)
            self._log(f"cancelled task {tid} after restart; the fleet re-auctions it")
        for q in st.get("queue") or []:
            # Summaries ({"task_id": ...}) from task_manager; bare ids from
            # the older shape.
            qt = self._fleet_tid(q.get("task_id") if isinstance(q, dict) else q)
            if qt is not None:
                self._cancel_local(qt)
                self._log(f"cancelled queued task {qt} after restart")

    def drive(self, dt: float, moving: bool) -> None:
        """Simulated battery (configs/fleet_tasks.yaml battery.sim_start_pct)
        for the Isaac track, which has none: drains only while driving, with
        the same rate the allocator's gate assumes."""
        if self.battery_pct is not None and moving:
            self.battery_pct = max(
                0.0, self.battery_pct - dt * self.alloc.p.battery_drain_pct_per_s)

    # ---- gateway ------------------------------------------------------------

    def request(self, pickup: str, dropoff: str, now: float) -> int | None:
        self.now = now
        wps = self.waypoints()
        if pickup not in wps or dropoff not in wps:
            self._log(f"rejected task: unknown waypoint in {pickup!r} -> {dropoff!r}")
            self._flush()
            return None
        # Seeded from the wall clock so a restarted gateway never reuses an ID:
        # robots ignore a re-announce of a task they already know, so a reused
        # ID would make the new task invisible to them.
        tid = self._next_id = max(self._next_id + 1, int(time.time() * 10))
        self.names[tid] = (pickup, dropoff)
        self._announced[tid] = now
        self.alloc.announce(tid, self._xy(pickup, wps), self._xy(dropoff, wps), now)
        self._log(f"announced task {tid}: {pickup} -> {dropoff}")
        self._flush()
        return tid

    def cancel(self, tid: int, now: float) -> None:
        self.now = now
        if tid in self.alloc.tasks:
            self.alloc.cancel(tid, now)
            self._log(f"cancelled task {tid}")
        self._flush()

    def board(self) -> dict:
        """The operator's view (module docstring). Robots are everyone the
        gateway has ever heard a heartbeat from; `alive` is the allocator's
        own silence judgement, not a fresh age cut-off, so the GUI agrees with
        the fleet about who is gone."""
        a, now = self.alloc, self.now
        tasks, robots = [], []
        for tid, t in sorted(a.tasks.items()):
            pick, drop = self.names.get(tid, (None, None))
            tasks.append({"task_id": tid, "pickup": pick, "dropoff": drop,
                          "state": t.state, "owner": robot_name(t.owner),
                          "committed": a.committed(tid),
                          "age_s": round(now - self._announced[tid], 1)
                          if tid in self._announced else None})
        for num in sorted(set(a.last_status) | set(self.peer_info)):
            info = self.peer_info.get(num, {})
            mine = [t for t in a.tasks.values() if t.state == CLAIMED and t.owner == num]
            last = a.last_status.get(num)
            robots.append({"id": robot_name(num), "alive": num not in a.silent,
                           "last_heard_s": round(now - last, 1) if last is not None else None,
                           "battery_pct": info.get("battery_pct"), "mode": info.get("mode"),
                           "owned": [t.task_id for t in mine],
                           "carrying": [t.task_id for t in mine
                                        if t.owner_cost <= COMMITTED_COST]})
        # An alarm leaves the board once its condition is gone AND it was
        # acknowledged — either alone keeps it visible (annunciator rule).
        self.alarms = [al for al in self.alarms if al["active"] or not al["acked"]][-100:]
        return {"t": round(now, 2), "robots": robots, "tasks": tasks, "alarms": self.alarms}

    def ack(self, alarm_id: int) -> None:
        for al in self.alarms:
            if al["id"] == alarm_id:
                al["acked"] = True

    def _raise(self, kind: str, text: str, *, robot: str | None = None,
               task_id: int | None = None, oneshot: bool = False) -> None:
        """Latch an alarm (gateway). A repeat of a still-active one is ignored;
        a one-shot has no condition to clear, so it is born inactive and
        shows only until acknowledged."""
        if not self.gateway:
            return
        for al in self.alarms:
            if al["active"] and (al["kind"], al["robot"], al["task_id"]) == (kind, robot, task_id):
                return
        self._alarm_seq += 1
        self.alarms.append({"id": self._alarm_seq, "kind": kind, "robot": robot,
                            "task_id": task_id, "t": round(self.now, 2),
                            "active": not oneshot, "acked": False, "text": text})

    def _clear(self, kind: str, *, robot: str | None = None, task_id: int | None = None) -> None:
        for al in self.alarms:
            if al["active"] and al["kind"] == kind and (robot is None or al["robot"] == robot) \
                    and (task_id is None or al["task_id"] == task_id):
                al["active"] = False

    def _xy(self, name: str, wps: dict | None = None) -> tuple[float, float]:
        # The waypoint itself: tasks are nav-only, so the robot parks on the
        # pin (no dock offset, unlike the retired stations).
        w = (wps or self.waypoints())[name]
        return (round(w.x, 3), round(w.y, 3))

    # ---- clock --------------------------------------------------------------

    def tick(self, now: float) -> None:
        self.now = now
        if self.pose is not None and self._pose_at is not None \
                and now - self._pose_at > float(self.cfg["pose_stale_s"]):
            self.pose = None  # lost localization: a bid from the old pose would be a lie
            self._log("pose stale; not bidding until TF is back")
        for tid, due in list(self._retry.items()):
            if now >= due and self.alloc.committed(tid) and tid in self.names:
                del self._retry[tid]
                pick, drop = self.names[tid]
                self._actions.append(("dispatch", {"task_id": f"{TASK_PREFIX}{tid}",
                                                   "pickup": pick, "dropoff": drop,
                                                   "carrying": True}))
                self._log(f"retrying dropoff of task {tid}")
        self.alloc.tick(now)
        if len(self.names) > len(self.alloc.tasks):  # the allocator pruned finished tasks
            self.names = {k: v for k, v in self.names.items() if k in self.alloc.tasks}
        # An echo that never came (task_manager had already finished it) must
        # not pin the id forever.
        self._cancelling &= set(self.alloc.tasks)
        if self.gateway:
            if now >= self._next_heartbeat:
                # Robots with no peer in range hear nobody but us; without a
                # heartbeat they would call themselves isolated and stop bidding.
                self._next_heartbeat = now + 1.0 / float(self.cfg["gateway_status_hz"])
                self.alloc.out.append({"type": "Status", "owned": []})
            if now >= self._next_reannounce:
                # The topic is volatile: a robot that starts after a task was
                # announced never hears of it. Unowned tasks are repeated;
                # robots that already know one ignore the repeat.
                self._next_reannounce = now + float(self.cfg["reannounce_s"])
                for t in self.alloc.tasks.values():
                    if t.state in (OPEN, BIDDING) and t.pickup is not None:
                        self.alloc.out.append({"type": "TaskAnnounce", "task_id": t.task_id,
                                               "pickup": t.pickup, "dropoff": t.dropoff})
            stalled = float(self.cfg["stalled_after_s"])
            for t in self.alloc.tasks.values():
                tid = t.task_id
                if t.state in (OPEN, BIDDING) and now - self._announced.get(tid, now) > stalled:
                    self._raise("task_stalled", f"task {tid} unclaimed for {stalled:.0f} s",
                                task_id=tid)
                else:
                    self._clear("task_stalled", task_id=tid)
                if not t.owner_missing:  # the owner renewed, finished, or was cancelled
                    self._clear("owner_missing", task_id=tid)
            for tid in [tid for tid in self._announced if tid not in self.alloc.tasks]:
                del self._announced[tid]
                self._clear("task_stalled", task_id=tid)
                self._clear("owner_missing", task_id=tid)
        self._flush()

    # ---- outputs ------------------------------------------------------------

    def _log(self, text: str) -> None:
        self._actions.append(("log", text))

    def _cancel_local(self, tid: int) -> None:
        self._cancelling.add(tid)
        self._actions.append(("cancel", f"{TASK_PREFIX}{tid}"))

    def _flush(self) -> None:
        out, events = self.alloc.drain()
        for m in out:
            if m["type"] == "TaskAnnounce" and m["task_id"] in self.names:
                pick, drop = self.names[m["task_id"]]
                m = {**m, "pickup_station": pick, "dropoff_station": drop}
            self._actions.append(("publish", encode(self.num, m)))
        for e in events:
            self._on_event(e)
        if not self.gateway:
            state = {"owned": [[tid, c] for tid, c in self.alloc.owned()],
                     "battery_pct": self.battery_pct, "isolated": self.alloc.isolated}
            raw = json.dumps(state)
            if raw != self._last_alloc:
                self._last_alloc = raw
                self._actions.append(("alloc", state))

    def _on_event(self, e: dict) -> None:
        kind, tid = e["event"], e.get("task_id")
        if kind == "claimed":
            if tid not in self.names:
                # Won a task announced without waypoint names (e.g. by a
                # commsim stub). task_manager drives to named waypoints only.
                self.alloc.release(tid, self.now)
                self._log(f"released task {tid}: no waypoint names")
                return
            pick, drop = self.names[tid]
            self._actions.append(("dispatch", {"task_id": f"{TASK_PREFIX}{tid}",
                                               "pickup": pick, "dropoff": drop}))
            self._log(f"won task {tid} ({pick} -> {drop}), cost {e.get('cost'):.1f}")
        elif kind in ("task_yielded", "stop_task"):
            self._cancel_local(tid)
            self._retry.pop(tid, None)
            why = e.get("reason") or f"out-bid by {robot_name(e.get('to'))}"
            if e.get("committed"):
                why += " WITH THE ITEM ON BOARD: unload it by hand"
            self._log(f"stopped task {tid}: {why}")
        elif kind in ("isolated", "rejoined", "hold_safe", "lease_expired",
                      "robot_silent", "robot_back", "battery_gated", "declined",
                      "committed", "owner_missing"):
            detail = {k: v for k, v in e.items() if k not in ("event", "robot_id", "t")}
            self._log(f"{kind} {detail}" if detail else kind)
            peer = robot_name(e.get("peer"))
            if kind == "robot_silent":
                self._raise("robot_silent", f"{peer} silent for "
                            f"{self.alloc.p.status_timeout_s:.0f} s", robot=peer)
            elif kind == "robot_back":
                self._clear("robot_silent", robot=peer)
            elif kind == "owner_missing":
                self._raise("owner_missing", f"task {tid}: owner {robot_name(e.get('owner'))} "
                            f"silent with the item on board", robot=robot_name(e.get("owner")),
                            task_id=tid)
            elif kind == "lease_expired":
                self._raise("lease_expired", f"task {tid}: lease of "
                            f"{robot_name(e.get('owner'))} expired, re-auctioned",
                            robot=robot_name(e.get("owner")), task_id=tid, oneshot=True)

    def drain(self) -> list[tuple]:
        actions, self._actions = self._actions, []
        return actions


# -- ROS 2 node -------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument("--gateway", action="store_true",
                        help="inject tasks and publish the task board; never bids")
    parser.add_argument("--no-bid", action="store_true",
                        help="follow the protocol but never bid (robot cannot pick)")
    parser.add_argument("--config", type=Path, default=DEFAULT_CFG)
    parser.add_argument(
        "--platform", default=os.environ.get("SORTBOTS_PLATFORM", "sim"),
        help="which platform's waypoint set to read from data/nav_waypoints.json "
             "(sim or real), the same one task_manager uses; default: "
             "$SORTBOTS_PLATFORM, else sim")
    args = parser.parse_args(argv)

    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
    )
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    import tf2_ros

    import fleet_radio

    cfg = load_config(args.config)
    params = Params.from_config(cfg)
    rid = args.robot_id

    # Reliable and deep: a lost Claim or Release costs a lease timeout, and a
    # burst of announces plus every robot's bids arrives at once.
    TASKS_QOS = QoSProfile(depth=200, reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.VOLATILE,
                           history=QoSHistoryPolicy.KEEP_LAST)
    STATUS_QOS = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            durability=QoSDurabilityPolicy.VOLATILE,
                            history=QoSHistoryPolicy.KEEP_LAST)
    LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)

    class FleetTasksNode(Node):
        def __init__(self):
            name = "fleet_gateway" if args.gateway else f"{rid}_fleet_tasks"
            super().__init__(name)
            self.core = FleetTasks(None if args.gateway else rid, params,
                                   lambda: load_waypoints(args.platform),
                                   gateway=args.gateway, can_bid=not args.no_bid,
                                   cfg=cfg, now=time.monotonic())
            self.sim_battery = (not args.gateway
                                and cfg["battery"].get("sim_start_pct") is not None)
            if self.sim_battery:
                self.core.battery_pct = float(cfg["battery"]["sim_start_pct"])
            self._moving = False
            self._last_tick = time.monotonic()

            self.tasks_pub = self.create_publisher(String, TASKS_TOPIC, TASKS_QOS)
            self.create_subscription(String, TASKS_TOPIC, self._on_tasks, TASKS_QOS)
            self.create_subscription(String, fleet_radio.STATUS_TOPIC, self._on_status,
                                     STATUS_QOS)
            if args.gateway:
                self.board_pub = self.create_publisher(String, BOARD_TOPIC, LATCHED)
                self.create_subscription(String, REQUEST_TOPIC, self._on_request, 10)
                self.create_subscription(String, CANCEL_TOPIC, self._on_cancel, 10)
                self.create_subscription(String, ALARM_ACK_TOPIC, self._on_ack, 10)
            else:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
                self.dispatch_pub = self.create_publisher(
                    String, f"/{rid}/dispatch_task", 10)
                self.cancel_pub = self.create_publisher(String, f"/{rid}/cancel_task", 10)
                self.alloc_pub = self.create_publisher(String, f"/{rid}/task_alloc", LATCHED)
                # LATCHED, matching task_manager's publisher: a volatile
                # subscriber never gets the last status replayed, so restart
                # adoption (_adopt) would wait for task_manager's NEXT change.
                self.create_subscription(String, f"/{rid}/task_status",
                                         self._on_task_status, LATCHED)
                self.create_subscription(BatteryState, f"/{rid}/battery_state",
                                         self._on_battery, 10)
                if self.sim_battery:
                    self.create_subscription(Odometry, f"/{rid}/odom", self._on_odom, 10)
                # The fused world map (nodes/map_merge.py), latched; bids
                # measure drive distance along it instead of a straight line.
                self.create_subscription(OccupancyGrid, "/map", self._on_map, LATCHED)
            # Steady clock: see the module docstring. Under use_sim_time a
            # node-clock timer would stretch the 1 s bid window by the
            # real-time factor (CLAUDE.md, "A ROS timer follows use_sim_time").
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(0.05, self._tick, clock=steady)
            if args.gateway:
                self.create_timer(1.0, self._publish_board, clock=steady)
            else:
                self.create_timer(0.5, self._update_pose, clock=steady)
            role = "gateway" if args.gateway else ("follower" if args.no_bid else "bidder")
            self.get_logger().info(
                f"fleet_tasks {role} on {TASKS_TOPIC} (wire id {self.core.num}, "
                f"{len(self.core.waypoints())} {args.platform} waypoints, "
                f"sim battery {self.sim_battery})")

        # -- inputs --
        def _on_tasks(self, msg):
            self.core.on_wire(msg.data, time.monotonic())
            self._act()

        def _on_status(self, msg):
            st = fleet_radio.parse_status(msg.data)
            if st is not None:
                self.core.on_peer_status(st["robot_id"], time.monotonic(),
                                         battery_pct=st["battery_pct"], mode=st["mode"])
                self._act()

        def _on_ack(self, msg):
            try:
                self.core.ack(int(json.loads(msg.data)["id"]))
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                self.get_logger().error(f"bad alarm ack {msg.data!r}: {exc}")

        def _on_task_status(self, msg):
            self.core.on_task_status(msg.data, time.monotonic())
            self._act()

        def _on_battery(self, msg):
            if not math.isnan(msg.percentage):
                self.sim_battery = False  # a real battery overrides the simulated one
                self.core.battery_pct = 100.0 * float(msg.percentage)

        def _on_map(self, msg):
            info = msg.info
            self.core.grid = GridDistance(
                info.width, info.height, info.resolution,
                (info.origin.position.x, info.origin.position.y), msg.data)

        def _on_odom(self, msg):
            tw = msg.twist.twist
            self._moving = abs(tw.linear.x) > 0.02 or abs(tw.angular.z) > 0.05

        def _on_request(self, msg):
            try:
                d = json.loads(msg.data)
                self.core.request(str(d["pickup"]), str(d["dropoff"]), time.monotonic())
            except (json.JSONDecodeError, KeyError, TypeError) as exc:
                self.get_logger().error(f"bad task request {msg.data!r}: {exc}")
            self._act()

        def _on_cancel(self, msg):
            try:
                self.core.cancel(int(json.loads(msg.data)["task_id"]), time.monotonic())
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                self.get_logger().error(f"bad cancel {msg.data!r}: {exc}")
            self._act()

        def _update_pose(self):
            try:
                tr = self.tf_buffer.lookup_transform(
                    "map", f"{rid}/base_link", rclpy.time.Time())
            except Exception:
                return  # keep the last pose; None until localized -> no bids
            t = tr.transform.translation
            self.core.set_pose((t.x, t.y), time.monotonic())

        def _tick(self):
            now = time.monotonic()
            if self.sim_battery:
                self.core.drive(now - self._last_tick, self._moving)
            self._last_tick = now
            self.core.tick(now)
            self._act()

        def _publish_board(self):
            self.board_pub.publish(String(data=json.dumps(self.core.board())))

        # -- outputs --
        def _act(self):
            log = self.get_logger()
            for kind, val in self.core.drain():
                if kind == "publish":
                    self.tasks_pub.publish(String(data=val))
                elif kind == "dispatch":
                    self.dispatch_pub.publish(String(data=json.dumps(val)))
                elif kind == "cancel":
                    self.cancel_pub.publish(String(data=val))
                elif kind == "alloc":
                    self.alloc_pub.publish(String(data=json.dumps(val)))
                elif kind == "log":
                    log.info(val)

    rclpy.init(args=argv)
    node = FleetTasksNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # SIGTERM (launch stopping us) already shut the context down; a
        # second rclpy.shutdown() raises and buries the real log in a trace.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
