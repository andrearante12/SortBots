"""Decentralized task allocation: auction + leases, no central coordinator.

Pure python with an injected clock: every handler takes `now`, the robot's
OWN clock. Leases are measured from when *this* robot last heard the owner,
never from message stamps, so a clock offset between robots cannot expire a
lease early. The same class runs on the robots (nodes/fleet_tasks.py), in the
in-memory harness (core/loopback.py) and in the DES, so all three test one
implementation.

Handlers append to `self.out` (messages to publish: dicts with a "type") and
`self.events` (metric events, e.g. robot_silent, lease_expired, task_yielded);
callers drain both with `drain()`.

Task states: OPEN -> BIDDING -> CLAIMED -> DONE, plus CANCELLED. Release or
lease expiry (+ margin) returns a task to OPEN.

Committed ownership: once the owner has picked the item up it calls
`commit()`. From then on the task is physically on that robot, so nothing
may move it elsewhere: the owner never yields it to a cheaper claim, keeps
working it while isolated, and peers whose lease on it expires do not
reclaim it (they report `owner_missing` for the operator instead). The only
way it leaves the owner is `complete`, the owner's own `release`, or a
cancel. On the wire a committed owner claims with COMMITTED_COST, below any
real travel cost, so the existing tie-break does the rest.

Majority quorum (Params.quorum_nodes > 0): a robot claims new tasks and
reclaims expired leases only while it hears a majority of the fleet, itself
counted. An owner that has gone a task lease without a majority stops its
uncommitted tasks, which is before the majority side's lease + margin runs
out. At most one side of a split holds a majority, so a split can no longer
put two robots on one task: the cut-off side waits instead of reclaiming.

Trust: every peer on the fleet topic is trusted. Any sender may complete or
cancel any task, and anything on the gateway's request topic is a task.
The mesh key (commsim/hw) is the boundary; nothing here authenticates.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Callable, Optional

OPEN, BIDDING, CLAIMED, DONE, CANCELLED = "open", "bidding", "claimed", "done", "cancelled"
COMPLETE, RELEASE, CANCEL = 0, 1, 2  # TaskEvent.kind, mirrors commsim_msgs/TaskEvent
# Owner cost of a committed task: below any real bid (travel seconds >= 0),
# so a committed owner wins every tie-break without a new message type.
COMMITTED_COST = -1.0


@dataclass
class Params:
    bid_window_s: float = 1.0
    bid_radius_m: float = 15.0
    rebid_delay_s: tuple = (0.2, 1.0)
    rebid_radius_growth: float = 2.0
    rebid_max_rounds: int = 3
    task_lease_s: float = 10.0
    reclaim_margin_s: float = 2.0
    # 6 s, not the spec's ~3 s: a measured reroute (1.95 s) plus TCP retry
    # backoff gaps Status for ~3.5-4 s, so 3 s declared healthy robots silent
    # after a single link loss (commsim/docs/sim_log.md steps 9 and 14).
    status_timeout_s: float = 6.0
    hold_safe_timeout_s: float = 15.0
    # Tasks a robot may own at once; it works them in claim order. Above 1 a
    # bid carries the backlog: the time to finish what it already owns plus
    # the drive from its last dropoff (2026-10-19, team decision: up to 3 on
    # the robots). Without that a loaded robot next to a pickup out-bids an
    # idle one across the room and the task waits behind its queue.
    max_owned_tasks: int = 1
    # Done and cancelled tasks are forgotten this long after they finished.
    # Kept a while so a late Status or re-announce of a finished task is
    # recognised, not re-run; dropped so a node running for days does not
    # grow (and scan on every tick) without bound.
    finished_retention_s: float = 300.0
    # Fleet-wide lease renewal. With neighbor-only Status (V3+), robots outside
    # an owner's neighbor cells never saw its Status, so leases expired at them
    # and they reclaimed tasks still being carried (sim_log finding 1). An owner
    # now also sends its owned-task list fleet-wide at this rate. 0 = off
    # (spec as originally written).
    lease_beacon_hz: float = 1.0
    # Cap on tasks a robot bids on at once. 0 = no cap (spec, and this
    # default so spec_original.yaml reproduces it). A 30-task burst made every
    # idle robot bid on every task; configs/base.yaml and the robots
    # (configs/fleet_tasks.yaml) set 3 — sim_log step 17.
    max_open_bids: int = 0
    # Every node in the fleet list, gateway included; 0 = no quorum (spec, and
    # this default so spec_original.yaml reproduces it). Team decision
    # 2026-10-20: no duplicate ownership, even across a network split — the
    # accepted alternative let both halves work a task until the split healed
    # (docs/plan.md question 11). Must match the real fleet: too small and a
    # minority can reclaim; too large and a healthy fleet cannot claim.
    quorum_nodes: int = 0
    # Battery. Only applies to robots given a battery_fn; without one a robot
    # bids on travel time alone. Two effects:
    #  - gate: no bid if driving the WHOLE task (to pickup, then dropoff) would
    #    leave less than the reserve, so a winner can always finish what it
    #    claimed and never strands a carried task mid-aisle;
    #  - penalty: up to battery_penalty_s added to the bid, linear from 0 at
    #    100 % to the full amount at the reserve, so a fuller robot beats a
    #    slightly closer, drained one. 30 s = 15 m of travel at 0.5 m/s.
    battery_drain_pct_per_s: float = 0.05  # placeholder: ~33 min driving per charge
    battery_reserve_pct: float = 20.0      # placeholder: margin to reach a charger
    battery_penalty_s: float = 30.0        # placeholder: open question, tune in DES

    @classmethod
    def from_config(cls, cfg: dict) -> "Params":
        p = cfg.get("protocol", cfg)
        kw = {k: p[k] for k in cls.__dataclass_fields__ if k in p}
        if "rebid_delay_s" in kw:
            kw["rebid_delay_s"] = tuple(kw["rebid_delay_s"])
        return cls(**kw)


@dataclass
class TaskView:
    task_id: int
    pickup: Optional[tuple] = None
    dropoff: Optional[tuple] = None
    state: str = OPEN
    owner: Optional[int] = None
    owner_cost: float = math.inf
    last_renew: float = 0.0
    bids: dict = field(default_factory=dict)
    window_end: Optional[float] = None
    round: int = 0
    rebid_at: Optional[float] = None
    claimed_at: float = 0.0  # when WE claimed it: the order we work our backlog
    finished_at: Optional[float] = None  # DONE/CANCELLED at: pruned after retention
    owner_missing: bool = False  # committed owner's lease expired; reported once
    reclaim_blocked: bool = False  # lease expired without a quorum; reported once


# cost_fn(pickup, dropoff[, start]) -> (travel time to pickup s, straight-line
# distance to pickup m[, drive time for the whole task s]), or None if this
# robot cannot take the task at all. The third value feeds the battery gate;
# without it the gate only sees the pickup leg. `start` is passed only when
# the robot already owns a task (max_owned_tasks > 1): measure from there,
# not from the robot's position.
CostFn = Callable[..., Optional[tuple]]
# battery_fn() -> state of charge in percent, or None if unknown.
BatteryFn = Callable[[], Optional[float]]


def _beats(a: tuple, b: tuple) -> bool:
    """(cost, robot_id) ordering: lower cost wins, ties to lower robot ID."""
    return a < b


class Allocator:
    def __init__(self, robot_id: int, params: Params, cost_fn: CostFn | None = None,
                 rng: random.Random | None = None, is_gateway: bool = False,
                 battery_fn: BatteryFn | None = None):
        self.id = robot_id
        self.p = params
        self.cost_fn = cost_fn
        self.battery_fn = battery_fn
        self.rng = rng or random.Random(robot_id)
        self.is_gateway = is_gateway
        self.tasks: dict[int, TaskView] = {}
        self.last_status: dict[int, float] = {}
        self.silent: set[int] = set()
        self.last_peer_heard: Optional[float] = None
        self.peer_heard: dict[int, float] = {}  # per peer, for the quorum
        self._quorum_since = -math.inf  # when the quorum last came back
        self._had_quorum = False
        self.isolated = False
        self.holding = False
        self.out: list[dict] = []
        self.events: list[dict] = []
        self._next_beacon: Optional[float] = None

    # ---- helpers ---------------------------------------------------------

    def drain(self) -> tuple[list[dict], list[dict]]:
        out, ev = self.out, self.events
        self.out, self.events = [], []
        return out, ev

    def _event(self, kind: str, now: float, **kw) -> None:
        self.events.append({"event": kind, "robot_id": self.id, "t": now, **kw})

    def owned(self) -> list[tuple[int, float]]:
        """(task_id, cost) pairs for Status — renews leases and carries the
        tie-break key so a healed partition converges without a re-Claim."""
        return [(t.task_id, t.owner_cost) for t in self.tasks.values()
                if t.state == CLAIMED and t.owner == self.id]

    def committed(self, task_id: int) -> bool:
        t = self.tasks.get(task_id)
        return t is not None and t.state == CLAIMED and t.owner_cost <= COMMITTED_COST

    def commit(self, task_id: int, now: float) -> None:
        """The item is on this robot: the task can no longer move (module
        docstring). Idempotent; ignored unless we own the task."""
        t = self.tasks.get(task_id)
        if t is None or t.state != CLAIMED or t.owner != self.id:
            return
        if t.owner_cost <= COMMITTED_COST:
            return
        t.owner_cost = COMMITTED_COST
        self.out.append({"type": "Claim", "task_id": task_id, "cost": COMMITTED_COST,
                         "lease_s": self.p.task_lease_s})
        self._event("committed", now, task_id=task_id)

    def _radius(self, rnd: int) -> float:
        if rnd >= self.p.rebid_max_rounds:
            return math.inf
        return self.p.bid_radius_m * self.p.rebid_radius_growth ** rnd

    def _can_take_new(self, now: float) -> bool:
        return (not self.is_gateway and not self.isolated and self.cost_fn is not None
                and len(self.owned()) < self.p.max_owned_tasks and self.has_quorum(now))

    def _quorum_at(self) -> float:
        """When we last heard a majority (ourselves counted): the time of the
        most recent message from the k-th most recently heard peer. No
        start-up grace, unlike isolation: until a majority has been heard
        there is no evidence this side is the one allowed to claim."""
        need = self.p.quorum_nodes // 2
        if need == 0:
            return math.inf
        heard = sorted(self.peer_heard.values(), reverse=True)
        return heard[need - 1] if len(heard) >= need else -math.inf

    def has_quorum(self, now: float) -> bool:
        return now - self._quorum_at() <= self.p.status_timeout_s

    def _backlog(self) -> tuple[Optional[tuple], float]:
        """(where we will be, how many seconds from now) once every task we
        own is done, in claim order. (None, 0) with nothing owned. The task
        in progress is counted from where we stand, as if not started: a
        small overestimate that cannot make a loaded robot look idle."""
        start, ahead = None, 0.0
        mine = sorted((t for t in self.tasks.values()
                       if t.state == CLAIMED and t.owner == self.id and t.pickup is not None),
                      key=lambda t: t.claimed_at)
        for t in mine:
            est = self.cost_fn(t.pickup, t.dropoff) if start is None else \
                self.cost_fn(t.pickup, t.dropoff, start)
            if est is None:
                continue
            ahead += est[2] if len(est) > 2 else est[0]
            start = t.dropoff if t.dropoff is not None else t.pickup
        return start, ahead

    def bid_cost(self, travel_s: float, task_s: float) -> Optional[float]:
        """Travel time adjusted for battery, or None if the battery gate
        forbids bidding. The adjusted value is what goes on the wire and into
        owner_cost, so every robot tie-breaks on the same number."""
        soc = self.battery_fn() if self.battery_fn is not None else None
        if soc is None:
            return travel_s
        reserve = self.p.battery_reserve_pct
        if soc - task_s * self.p.battery_drain_pct_per_s < reserve:
            return None
        depleted = (100.0 - soc) / max(100.0 - reserve, 1e-9)
        return travel_s + self.p.battery_penalty_s * min(max(depleted, 0.0), 1.0)

    def _heard(self, sender: int, now: float) -> None:
        if sender != self.id:
            self.last_peer_heard = now
            self.peer_heard[sender] = now

    def heard_network(self, now: float, peer: Optional[int] = None) -> None:
        """Connectivity evidence that isn't a message — e.g. the local zenohd
        still holds a live session. With neighbor-only status (V3+) a robot
        with no neighbors hears no Status at all; without this it would call
        itself isolated and stop taking tasks while perfectly connected.

        Pass the time data was last actually RECEIVED, not the time liveness
        was checked: the latter adds up to one transport lease of lag, and an
        isolated owner then stops after its lease instead of within it.

        `peer`, when the evidence is a session to one known peer, also counts
        toward the quorum: with neighbor-only Status that is the only way to
        hear a distant robot that owns nothing."""
        if self.last_peer_heard is None or now > self.last_peer_heard:
            self.last_peer_heard = now
        if peer is not None and peer != self.id and now > self.peer_heard.get(peer, -math.inf):
            self.peer_heard[peer] = now

    def forget(self, peer: int) -> None:
        """Stop silence-tracking a peer we unsubscribed from (it left our
        neighbor cells). Its silence is expected, not a failure."""
        self.last_status.pop(peer, None)
        self.silent.discard(peer)

    # ---- auction ---------------------------------------------------------

    def _open_bidding(self, t: TaskView, now: float) -> None:
        t.state, t.bids, t.rebid_at = BIDDING, {}, None
        t.owner, t.owner_cost = None, math.inf
        t.window_end = now + self.p.bid_window_s
        if not self._can_take_new(now) or t.pickup is None:
            return
        start, ahead = self._backlog()
        est = self.cost_fn(t.pickup, t.dropoff) if start is None else \
            self.cost_fn(t.pickup, t.dropoff, start)
        if est is None:
            return
        # Both the bid and the battery gate include the backlog: the gate must
        # see everything we have promised, or a third task could be claimed
        # that we can only start, not finish.
        travel, dist = est[0] + ahead, est[1]
        if dist > self._radius(t.round):
            return
        cost = self.bid_cost(travel, (est[2] if len(est) > 2 else est[0]) + ahead)
        if cost is None:
            self._event("battery_gated", now, task_id=t.task_id)
            return
        if self.p.max_open_bids and sum(
                1 for v in self.tasks.values()
                if v.state == BIDDING and self.id in v.bids) >= self.p.max_open_bids:
            return
        t.bids[self.id] = float(cost)
        self.out.append({"type": "Bid", "task_id": t.task_id, "cost": float(cost)})

    def announce(self, task_id: int, pickup: tuple, dropoff: tuple, now: float) -> None:
        """Gateway/operator side: inject a task."""
        self.out.append({"type": "TaskAnnounce", "task_id": task_id,
                         "pickup": tuple(pickup), "dropoff": tuple(dropoff)})
        self.on_announce(self.id, task_id, pickup, dropoff, now)

    def on_announce(self, sender: int, task_id: int, pickup: tuple, dropoff: tuple,
                    now: float) -> None:
        self._heard(sender, now)
        t = self.tasks.get(task_id)
        if t is None:
            t = self.tasks[task_id] = TaskView(task_id, tuple(pickup), tuple(dropoff))
            self._open_bidding(t, now)
        elif t.pickup is None:  # learned of it from a Status before the announce
            t.pickup, t.dropoff = tuple(pickup), tuple(dropoff)

    def on_bid(self, sender: int, task_id: int, cost: float, now: float) -> None:
        self._heard(sender, now)
        t = self.tasks.get(task_id)
        if t is None:
            return
        if t.state == OPEN:  # a peer's rebid / reclaim round started first: join it
            self._open_bidding(t, now)
        if t.state == BIDDING:
            t.bids[sender] = float(cost)

    def _close_window(self, t: TaskView, now: float) -> None:
        if not t.bids:
            t.state, t.round, t.window_end = OPEN, t.round + 1, None
            lo, hi = self.p.rebid_delay_s
            t.rebid_at = now + self.rng.uniform(lo, hi)
            self._event("no_winner", now, task_id=t.task_id, round=t.round)
            return
        winner, cost = min(((rid, c) for rid, c in t.bids.items()),
                           key=lambda rc: (rc[1], rc[0]))
        if winner == self.id and (len(self.owned()) >= self.p.max_owned_tasks
                                  or not self.has_quorum(now)):
            # Won while already at capacity (we bid on several tasks at once and
            # another window closed first). Claiming anyway left robots "owning"
            # tasks they never worked — renewed forever, never done (found in a
            # heavy-load DES run, 2026-10-03). Decline: peers holding us as the
            # provisional winner reopen on the Release instead of waiting out a
            # lease. Same if the quorum went during the window: a claim from the
            # minority side of a split is exactly the duplicate it prevents.
            self.out.append({"type": "TaskEvent", "task_id": t.task_id, "kind": RELEASE})
            self._event("declined", now, task_id=t.task_id)
            self._open_bidding(t, now)
            return
        t.state, t.owner, t.owner_cost, t.last_renew = CLAIMED, winner, cost, now
        t.window_end, t.round = None, 0
        if winner == self.id:
            t.claimed_at = now
            self.out.append({"type": "Claim", "task_id": t.task_id, "cost": cost,
                             "lease_s": self.p.task_lease_s})
            self._event("claimed", now, task_id=t.task_id, cost=cost)
            if len(self.owned()) >= self.p.max_owned_tasks:
                # At capacity now: withdraw our other open bids (existing
                # Release message) so those windows pick the next-best bidder
                # in the same round instead of us winning and declining later.
                for v in self.tasks.values():
                    if v.state == BIDDING and self.id in v.bids:
                        v.bids.pop(self.id)
                        self.out.append({"type": "TaskEvent", "task_id": v.task_id,
                                         "kind": RELEASE})
        # Otherwise the expected winner is provisional: if it never claims or
        # renews, lease expiry reopens the task — no extra timer needed.

    # ---- ownership -------------------------------------------------------

    def _assert_owner(self, t: TaskView, sender: int, cost: float, now: float) -> None:
        if t.state in (DONE, CANCELLED):
            return
        if t.state == CLAIMED and t.owner is not None and t.owner != sender:
            if not _beats((cost, sender), (t.owner_cost, t.owner)):
                return  # incumbent wins the tie-break; sender will yield when it hears us
            if t.owner == self.id:
                self._event("task_yielded", now, task_id=t.task_id, to=sender)
        t.state, t.owner, t.owner_cost, t.last_renew = CLAIMED, sender, cost, now
        t.window_end = t.rebid_at = None
        t.bids = {}
        t.owner_missing = t.reclaim_blocked = False

    def on_claim(self, sender: int, task_id: int, cost: float, now: float) -> None:
        self._heard(sender, now)
        t = self.tasks.setdefault(task_id, TaskView(task_id))
        mine = t.bids.get(self.id)
        if t.state == BIDDING and mine is not None and _beats((mine, self.id), (cost, sender)):
            # Windows open on local receipt of the announce, so a peer can
            # close early without having seen our cheaper bid. Keep our
            # window; our Claim will beat theirs and they yield on receipt.
            t.bids[sender] = float(cost)
            return
        self._assert_owner(t, sender, float(cost), now)

    def on_status(self, sender: int, owned: list, now: float) -> None:
        self._heard(sender, now)
        self.last_status[sender] = now
        if sender in self.silent:
            self.silent.discard(sender)
            self._event("robot_back", now, peer=sender)
        self.on_lease_renewal(sender, owned, now)

    def on_lease_renewal(self, sender: int, owned: list, now: float) -> None:
        """Ownership half of Status, without silence tracking. Used alone by a
        fleet-wide lease beacon when Status itself is neighbor-only."""
        self._heard(sender, now)
        for task_id, cost in owned:
            t = self.tasks.setdefault(task_id, TaskView(task_id))
            self._assert_owner(t, sender, float(cost), now)

    def on_task_event(self, sender: int, task_id: int, kind: int, now: float) -> None:
        self._heard(sender, now)
        t = self.tasks.get(task_id)
        if t is None:
            return
        if kind == COMPLETE:
            t.state, t.finished_at = DONE, now
        elif kind == CANCEL:
            if t.owner == self.id and t.state == CLAIMED:
                self._event("stop_task", now, task_id=task_id, reason="cancelled",
                            committed=t.owner_cost <= COMMITTED_COST)
            t.state, t.finished_at = CANCELLED, now
        elif kind == RELEASE and t.owner == sender and t.state == CLAIMED:
            self._open_bidding(t, now)
        elif kind == RELEASE and t.state == BIDDING and sender in t.bids:
            t.bids.pop(sender)  # a bidder that cannot take it after all

    def cancel(self, task_id: int, now: float) -> None:
        """Gateway/operator side: withdraw a task fleet-wide."""
        self.out.append({"type": "TaskEvent", "task_id": task_id, "kind": CANCEL})
        self.on_task_event(self.id, task_id, CANCEL, now)

    def complete(self, task_id: int, now: float) -> None:
        self.tasks[task_id].state = DONE
        self.tasks[task_id].finished_at = now
        self.out.append({"type": "TaskEvent", "task_id": task_id, "kind": COMPLETE})

    def release(self, task_id: int, now: float) -> None:
        t = self.tasks[task_id]
        t.state, t.owner, t.owner_cost = OPEN, None, math.inf
        self.out.append({"type": "TaskEvent", "task_id": task_id, "kind": RELEASE})

    # ---- timers ----------------------------------------------------------

    def tick(self, now: float) -> None:
        if self.last_peer_heard is None:
            self.last_peer_heard = now  # grace from start-up, not an instant isolation
        quiet = now - self.last_peer_heard

        was_isolated = self.isolated
        self.isolated = quiet > self.p.status_timeout_s
        if self.isolated and not was_isolated:
            self._event("isolated", now)
        elif was_isolated and not self.isolated:
            self._event("rejoined", now)
            self.holding = False
        if self.isolated and not self.holding and quiet > self.p.hold_safe_timeout_s:
            self.holding = True
            self._event("hold_safe", now)

        q = self.has_quorum(now)
        if q and not self._had_quorum:
            self._quorum_since = now
        self._had_quorum = q

        owned = self.owned()
        if self.p.lease_beacon_hz > 0 and owned:
            if self._next_beacon is None or now >= self._next_beacon:
                self.out.append({"type": "LeaseBeacon", "owned": owned})
                self._next_beacon = now + 1.0 / self.p.lease_beacon_hz
        elif not owned:
            self._next_beacon = None  # first renewal goes out as soon as we own again

        for peer, last in self.last_status.items():
            if peer not in self.silent and now - last > self.p.status_timeout_s:
                self.silent.add(peer)
                self._event("robot_silent", now, peer=peer)

        expiry = self.p.task_lease_s + self.p.reclaim_margin_s
        for t in self.tasks.values():
            if t.state == BIDDING and now >= t.window_end:
                self._close_window(t, now)
            elif t.state == OPEN and t.rebid_at is not None and now >= t.rebid_at:
                self._open_bidding(t, now)
            elif t.state == CLAIMED and t.owner == self.id:
                # An isolated owner must stop before ANY peer could reclaim:
                # peers wait lease + margin from when they last heard us, and
                # we stop at lease from when we last heard anyone. Not once
                # committed: the item is on board, nobody else can do it.
                # Without a majority for a lease the majority side may reclaim
                # at lease + margin, so the same deadline applies.
                if (quiet > self.p.task_lease_s or now - self._quorum_at() > self.p.task_lease_s) \
                        and t.owner_cost > COMMITTED_COST:
                    t.state, t.owner, t.owner_cost = OPEN, None, math.inf
                    self._event("stop_task", now, task_id=t.task_id, reason="lease_lost")
            elif t.state == CLAIMED and now - t.last_renew > expiry:
                if t.owner_cost <= COMMITTED_COST:
                    # Reclaiming would send a robot to an empty shelf. Report
                    # once; the owner finishes when it is back, or an operator
                    # cancels.
                    if not t.owner_missing:
                        t.owner_missing = True
                        self._event("owner_missing", now, task_id=t.task_id, owner=t.owner)
                    continue
                if not q:
                    # Minority side of a split: the owner may be working it on
                    # the majority side. Wait; a renewal or the heal settles it.
                    if not t.reclaim_blocked:
                        t.reclaim_blocked = True
                        self._event("reclaim_blocked", now, task_id=t.task_id, owner=t.owner)
                    continue
                if t.reclaim_blocked and now - self._quorum_since < self.p.reclaim_margin_s:
                    # Just healed: the owner's lease beacon may be a second
                    # away. Reclaiming first re-auctioned tasks the owner had
                    # committed during the split (DES partition, n=10,
                    # 2026-10-20); the beacon's committed cost settles it.
                    continue
                self._event("lease_expired", now, task_id=t.task_id, owner=t.owner)
                self._open_bidding(t, now)
        for tid in [tid for tid, t in self.tasks.items() if t.finished_at is not None
                    and now - t.finished_at > self.p.finished_retention_s]:
            del self.tasks[tid]


def deliver(alloc: Allocator, sender: int, m: dict, now: float) -> bool:
    """Route one received message to its handler. False if the type is not
    part of the task protocol (callers may carry other traffic alongside)."""
    typ = m.get("type")
    if typ == "TaskAnnounce":
        alloc.on_announce(sender, m["task_id"], m["pickup"], m["dropoff"], now)
    elif typ == "Bid":
        alloc.on_bid(sender, m["task_id"], m["cost"], now)
    elif typ == "Claim":
        alloc.on_claim(sender, m["task_id"], m["cost"], now)
    elif typ == "TaskEvent":
        alloc.on_task_event(sender, m["task_id"], m["kind"], now)
    elif typ == "Status":
        alloc.on_status(sender, m["owned"], now)
    elif typ == "LeaseBeacon":
        alloc.on_lease_renewal(sender, m["owned"], now)
    else:
        return False
    return True
