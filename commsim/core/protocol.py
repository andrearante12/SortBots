"""Decentralized task allocation: auction + leases, no central coordinator.

Pure python with an injected clock: every handler takes `now`, the robot's
OWN clock. Leases are measured from when *this* robot last heard the owner,
never from message stamps, so a clock offset between robots cannot expire a
lease early. The same class runs inside the ROS 2 stub (stubs/robot_stub.py),
the in-memory harness (core/loopback.py) and the DES, so all three test one
implementation.

Handlers append to `self.out` (messages to publish: dicts with a "type") and
`self.events` (metric events, e.g. robot_silent, lease_expired, task_yielded);
callers drain both with `drain()`.

Task states: OPEN -> BIDDING -> CLAIMED -> DONE, plus CANCELLED. Release or
lease expiry (+ margin) returns a task to OPEN.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Callable, Optional

OPEN, BIDDING, CLAIMED, DONE, CANCELLED = "open", "bidding", "claimed", "done", "cancelled"
COMPLETE, RELEASE, CANCEL = 0, 1, 2  # TaskEvent.kind, mirrors commsim_msgs/TaskEvent


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
    max_owned_tasks: int = 1
    # Fleet-wide lease renewal. With neighbor-only Status (V3+), robots outside
    # an owner's neighbor cells never saw its Status, so leases expired at them
    # and they reclaimed tasks still being carried (sim_log finding 1). An owner
    # now also sends its owned-task list fleet-wide at this rate. 0 = off
    # (spec as originally written).
    lease_beacon_hz: float = 1.0
    # EXPERIMENT (not adopted): cap on tasks a robot bids on at once. 0 = no
    # cap (spec). A 30-task burst made every idle robot bid on every task.
    max_open_bids: int = 0

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


# cost_fn(pickup) -> (estimated travel time s, straight-line distance m), or
# None if this robot cannot take the task at all.
CostFn = Callable[[tuple], Optional[tuple]]


def _beats(a: tuple, b: tuple) -> bool:
    """(cost, robot_id) ordering: lower cost wins, ties to lower robot ID."""
    return a < b


class Allocator:
    def __init__(self, robot_id: int, params: Params, cost_fn: CostFn | None = None,
                 rng: random.Random | None = None, is_gateway: bool = False):
        self.id = robot_id
        self.p = params
        self.cost_fn = cost_fn
        self.rng = rng or random.Random(robot_id)
        self.is_gateway = is_gateway
        self.tasks: dict[int, TaskView] = {}
        self.last_status: dict[int, float] = {}
        self.silent: set[int] = set()
        self.last_peer_heard: Optional[float] = None
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

    def _radius(self, rnd: int) -> float:
        if rnd >= self.p.rebid_max_rounds:
            return math.inf
        return self.p.bid_radius_m * self.p.rebid_radius_growth ** rnd

    def _can_take_new(self) -> bool:
        return (not self.is_gateway and not self.isolated and self.cost_fn is not None
                and len(self.owned()) < self.p.max_owned_tasks)

    def _heard(self, sender: int, now: float) -> None:
        if sender != self.id:
            self.last_peer_heard = now

    def heard_network(self, now: float) -> None:
        """Connectivity evidence that isn't a message — e.g. the local zenohd
        still holds a live session. With neighbor-only status (V3+) a robot
        with no neighbors hears no Status at all; without this it would call
        itself isolated and stop taking tasks while perfectly connected.

        Pass the time data was last actually RECEIVED, not the time liveness
        was checked: the latter adds up to one transport lease of lag, and an
        isolated owner then stops after its lease instead of within it."""
        if self.last_peer_heard is None or now > self.last_peer_heard:
            self.last_peer_heard = now

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
        if not self._can_take_new() or t.pickup is None:
            return
        est = self.cost_fn(t.pickup)
        if est is None:
            return
        cost, dist = est
        if dist > self._radius(t.round):
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
        if winner == self.id and len(self.owned()) >= self.p.max_owned_tasks:
            # Won while already at capacity (we bid on several tasks at once and
            # another window closed first). Claiming anyway left robots "owning"
            # tasks they never worked — renewed forever, never done (found in a
            # heavy-load DES run, 2026-10-03). Decline: peers holding us as the
            # provisional winner reopen on the Release instead of waiting out a
            # lease.
            self.out.append({"type": "TaskEvent", "task_id": t.task_id, "kind": RELEASE})
            self._event("declined", now, task_id=t.task_id)
            self._open_bidding(t, now)
            return
        t.state, t.owner, t.owner_cost, t.last_renew = CLAIMED, winner, cost, now
        t.window_end, t.round = None, 0
        if winner == self.id:
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
            t.state = DONE
        elif kind == CANCEL:
            if t.owner == self.id and t.state == CLAIMED:
                self._event("stop_task", now, task_id=task_id, reason="cancelled")
            t.state = CANCELLED
        elif kind == RELEASE and t.owner == sender and t.state == CLAIMED:
            self._open_bidding(t, now)
        elif kind == RELEASE and t.state == BIDDING and sender in t.bids:
            t.bids.pop(sender)  # a bidder that cannot take it after all

    def complete(self, task_id: int, now: float) -> None:
        self.tasks[task_id].state = DONE
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
                # we stop at lease from when we last heard anyone.
                if quiet > self.p.task_lease_s:
                    t.state, t.owner, t.owner_cost = OPEN, None, math.inf
                    self._event("stop_task", now, task_id=t.task_id, reason="lease_lost")
            elif t.state == CLAIMED and now - t.last_renew > expiry:
                self._event("lease_expired", now, task_id=t.task_id, owner=t.owner)
                self._open_bidding(t, now)
