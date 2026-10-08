"""In-memory fleet: real Allocator logic over an idealized network.

No radio, no airtime — a fixed delivery delay, optional per-pair blocking
(partitions, isolation, dead robots) and per-robot clock skew. It exists to
test protocol *correctness* (no duplicate owners, no abandoned tasks, timer
ordering) in milliseconds; the DES adds the medium on top of the same logic.
"""
from __future__ import annotations

import heapq
import itertools
import math
import random
from dataclasses import dataclass, field

from .protocol import Allocator, Params, CLAIMED


@dataclass
class Robot:
    alloc: Allocator
    pos: tuple = (0.0, 0.0)
    speed_mps: float = 0.5
    clock_offset_s: float = 0.0
    clock_rate: float = 1.0
    alive: bool = True

    def now(self, t: float) -> float:
        return self.clock_offset_s + self.clock_rate * t


@dataclass
class LoopbackNet:
    params: Params
    delay_s: float = 0.02
    status_hz: float = 3.0
    tick_s: float = 0.05
    seed: int = 1
    t: float = 0.0
    robots: dict = field(default_factory=dict)
    blocked: set = field(default_factory=set)
    log: list = field(default_factory=list)
    # optional (sender, dst, msg) -> bool; False drops that delivery (e.g. to
    # model neighbor-only Status, where far robots never get an owner's Status)
    deliver_filter: object = None
    _q: list = field(default_factory=list)
    _seq: itertools.count = field(default_factory=itertools.count)
    _next_status: dict = field(default_factory=dict)

    def add(self, rid: int, pos=(0.0, 0.0), gateway=False, **kw) -> Robot:
        r = Robot(alloc=None, pos=tuple(pos), **kw)

        def cost_fn(pickup, r=r):
            d = math.dist(r.pos, pickup)
            return d / r.speed_mps, d

        r.alloc = Allocator(rid, self.params, None if gateway else cost_fn,
                            random.Random(self.seed * 1000 + rid), is_gateway=gateway)
        self.robots[rid] = r
        # Stagger status phases like real robots booting at different times.
        self._next_status[rid] = self.t + random.Random(rid).uniform(0, 1 / self.status_hz)
        return r

    def link_up(self, a: int, b: int) -> bool:
        return frozenset((a, b)) not in self.blocked

    def partition(self, *groups) -> None:
        self.blocked = {frozenset((a, b)) for g1 in groups for g2 in groups if g1 is not g2
                        for a in g1 for b in g2}

    def isolate(self, rid: int) -> None:
        self.blocked |= {frozenset((rid, o)) for o in self.robots if o != rid}

    def heal(self) -> None:
        self.blocked = set()

    def _publish(self, sender: int, msgs: list[dict]) -> None:
        for m in msgs:
            for dst in self.robots:
                if dst != sender:
                    heapq.heappush(self._q, (self.t + self.delay_s, next(self._seq),
                                             dst, sender, m))

    def _deliver(self, dst: int, sender: int, m: dict) -> None:
        r = self.robots[dst]
        a, now = r.alloc, r.now(self.t)
        typ = m["type"]
        if typ == "TaskAnnounce":
            a.on_announce(sender, m["task_id"], m["pickup"], m["dropoff"], now)
        elif typ == "Bid":
            a.on_bid(sender, m["task_id"], m["cost"], now)
        elif typ == "Claim":
            a.on_claim(sender, m["task_id"], m["cost"], now)
        elif typ == "TaskEvent":
            a.on_task_event(sender, m["task_id"], m["kind"], now)
        elif typ == "Status":
            a.on_status(sender, m["owned"], now)
        elif typ == "LeaseBeacon":
            a.on_lease_renewal(sender, m["owned"], now)

    def _flush(self, rid: int) -> None:
        out, ev = self.robots[rid].alloc.drain()
        self.log.extend(ev)
        self._publish(rid, out)

    def step(self) -> None:
        self.t += self.tick_s
        while self._q and self._q[0][0] <= self.t:
            _, _, dst, sender, m = heapq.heappop(self._q)
            # Blocking is checked at delivery: a link cut mid-flight drops it,
            # like a radio link going away under a queued frame.
            if self.deliver_filter and not self.deliver_filter(sender, dst, m):
                continue
            if self.robots[dst].alive and self.robots[sender].alive and self.link_up(dst, sender):
                self._deliver(dst, sender, m)
        for rid, r in self.robots.items():
            if not r.alive:
                continue
            if self.t >= self._next_status[rid]:
                self._next_status[rid] += 1 / self.status_hz
                r.alloc.out.append({"type": "Status", "owned": r.alloc.owned()})
            r.alloc.tick(r.now(self.t))
            self._flush(rid)

    def run(self, seconds: float) -> None:
        end = self.t + seconds
        while self.t < end:
            self.step()

    def announce(self, gw: int, task_id: int, pickup, dropoff) -> None:
        r = self.robots[gw]
        r.alloc.announce(task_id, pickup, dropoff, r.now(self.t))
        self._flush(gw)

    # ---- invariants ------------------------------------------------------

    def self_owners(self, task_id: int) -> list[int]:
        """Robots that believe THEY own the task (the duplicate-work check)."""
        return [rid for rid, r in self.robots.items() if r.alive
                and (t := r.alloc.tasks.get(task_id)) is not None
                and t.state == CLAIMED and t.owner == rid]

    def events(self, kind: str) -> list[dict]:
        return [e for e in self.log if e["event"] == kind]
