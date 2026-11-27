"""Map-tile subscriptions (`map_share.recipients: subscribers`, plan item B8)
and event-driven map updates (`map_share.events`, B9).

Subscriptions (2026-11-26; docs/fleetsim.md "Map sharing: subscriptions and
event-driven updates"):

  * Each robot subscribes to the tiles within `sub_radius_m` of it and to the
    tiles along the next `sub_path_m` of its planned path (`wanted`). It works
    the set out at every map tick and announces it, whole, to every robot
    (`mapsub`, reliable) when it changed, at most every `sub_min_interval_s`,
    and unchanged every `sub_keepalive_s`. The whole set, not the change: a
    lost announcement is healed by the next one, and a version number drops
    one that arrives late.
  * A robot sends its patches and refresh snapshots of a tile to the tile's
    subscribers as it has heard them. Its own map of who subscribes is never
    ground truth.
  * A robot that subscribes to a new tile PULLS its state from one peer
    (`mappull` -> `mapans`), not from every robot that has seen it. Every
    current subscriber of a tile has been getting every patch of it, so any
    one of them holds the fleet's view of it; all of them answering would cost
    k copies of the same tile. The asker picks the one nearest itself (the
    fewest radio hops), preferring one subscribed long enough to have had its
    own answer; failing that, the peer it heard leave the tile last.
  * What the answer carries. With a shared prior (map_share.prior truth, a
    localize-on-saved-map run) every robot starts from the same map, so only
    the cells that have ever differed from it in the answerer's map, with
    their current state (a pallet that has gone comes back as free):
    typically a pallet's faces, not a 4 kB tile. Without one, the cells
    observed since the asker last knew the tile: when it last left it, less
    two publish periods (patches still on their way when it left went
    elsewhere), or everything known if it never had it. It also carries the
    answerer's sequence numbers for the tile (`Belief.seq_vector`), so the
    asker's next delta from each origin is not taken for a gap (each would
    otherwise cost a full MapResync snapshot). While a pull is open, gaps on
    that tile ask for no resync.
  * Answers come out of the answerer's own rate budget (`rate_cap_bps`), at
    once if it has room, else at its next map tick before the refresh.
  * A patch that carries a cell the sender's own sensing flipped (free <->
    occupied: a pallet came or went) goes to EVERY robot (`sub_flips_to:
    all`), the rest to the tile's subscribers. Subscriptions keep news local,
    but the auction prices tasks anywhere: a robot far from a new pallet bid
    on its stale map, won a task past it, and only learned of it once its
    path subscribed (map_sharing, seed 1, 2026-11-26: 91 s for a delivery
    the fleet-wide run made in 81 s). Flips are rare and small, so telling
    everyone costs little. A robot not subscribed to the tile asks no resync
    for gaps there: it is not owed the tile's whole stream.
  * Sequence numbers, MapResync and the round-robin refresh are unchanged; a
    patch of a tile nobody subscribes to is not sent at all (and takes no
    sequence number), since no one is owed it.

What it costs is its own traffic kinds in the `net` stream: `mapsub`
(announcements), `mappull` (pull requests) and `mapans` (answers), beside
`map` (patches, refresh) and `resync`.
"""
from __future__ import annotations

import math
from collections import OrderedDict, defaultdict

import numpy as np

from .belief import PRIOR_STAMP

SUB_HEADER_B = 16   # robot id, version, count
SUB_TILE_B = 2      # tile id
PULL_HEADER_B = 16
PULL_TILE_B = 6     # tile id + since (float32)
VV_ENTRY_B = 3      # origin id + sequence number, per origin in an answer
POSE_FRESH_S = 10.0  # a peer's Status pose this old still says roughly where it is


class _Robot:
    """One robot's side: its own subscription, and what it has heard of its
    peers' (by node id)."""

    def __init__(self):
        self.tiles: frozenset = frozenset()   # as last announced
        self.ver = 0
        self.next_ok = 0.0                    # sub_min_interval_s
        self.next_keep = 0.0                  # sub_keepalive_s
        self.left: dict[int, float] = {}      # tile -> when it announced dropping it
        self.pulling: dict[int, float] = {}   # tile -> until when a gap asks no resync
        self.view: dict[int, tuple] = {}      # peer node -> (ver, tiles, heard at)
        self.subs_of: dict[int, dict] = defaultdict(dict)  # tile -> {peer node: since}
        self.last_left: dict[int, tuple] = {}  # tile -> (when, peer node) it heard leave
        self.answers: OrderedDict = OrderedDict()  # (asker node, tile) -> since
        self.alive_until = 0.0
        self.path = None                      # the path the set was last worked out for


class Subscriptions:
    def __init__(self, sim):
        c = sim.cfg["map_share"]
        self.sim = sim
        self.radius = float(c["sub_radius_m"])
        self.ahead = None if c["sub_path_m"] is None else float(c["sub_path_m"])
        self.min_iv = float(c["sub_min_interval_s"])
        self.keep = float(c["sub_keepalive_s"])
        self.expire = 2.5 * self.keep
        self.margin = 2.0 * float(c["publish_s"])
        self.flips_all = c["sub_flips_to"] == "all"
        # An answer can wait a publish period for budget, then travels like
        # any reliable message (up to the transport lease).
        self.pull_wait = float(sim.net.lease_s) + 2.0 * float(c["publish_s"])
        prior = c["prior"] == "truth" or sim.cfg["info"]["map"] == "truth"
        self.prior_since = float(PRIOR_STAMP) if prior else -math.inf
        self.st = [_Robot() for _ in sim.robots]
        for r in sim.robots:
            r.belief.track_touched()  # no-op without a shared prior
        self.robot_nodes = np.arange(1, sim.n + 1)
        self.counts = {"announces": 0, "pulls": 0, "pulled_tiles": 0, "answers": 0,
                       "answer_cells": 0, "no_source": 0}

    # ---- what a robot subscribes to ----------------------------------------

    def wanted(self, r) -> frozenset:
        b = r.belief
        tiles = self._disc(b, r.x, r.y, self.radius)
        if r.goal is not None and r.path:
            tiles |= self._along(b, r)
        return frozenset(tiles)

    @staticmethod
    def _disc(b, x: float, y: float, rad: float) -> set:
        """Tiles whose square meets the disc of radius rad around (x, y)."""
        g = b.grid
        tm = b.T * g.res
        tx0 = max(0, int(math.floor((x - rad - g.ox) / tm)))
        tx1 = min(b.ntx - 1, int(math.floor((x + rad - g.ox) / tm)))
        ty0 = max(0, int(math.floor((y - rad - g.oy) / tm)))
        ty1 = min(b.nty - 1, int(math.floor((y + rad - g.oy) / tm)))
        out = set()
        for ty in range(ty0, ty1 + 1):
            y0 = g.oy + ty * tm
            dy = max(y0 - y, 0.0, y - (y0 + tm))
            for tx in range(tx0, tx1 + 1):
                x0 = g.ox + tx * tm
                dx = max(x0 - x, 0.0, x - (x0 + tm))
                if dx * dx + dy * dy <= rad * rad:
                    out.add(ty * b.ntx + tx)
        return out

    def _along(self, b, r) -> set:
        """Tiles the next `ahead` metres of the path pass through, or come
        within a body of: a pallet half in the next tile still blocks it."""
        pts = np.array([(r.x, r.y)] + list(r.path[r.seg + 1:]), dtype=float)
        if len(pts) < 2:
            return set()
        seg = np.hypot(*np.diff(pts, axis=0).T)
        cum = np.r_[0.0, np.cumsum(seg)]
        end = cum[-1] if self.ahead is None else min(cum[-1], self.ahead)
        s = np.r_[np.arange(0.0, end, 0.5), end]
        xy = np.c_[np.interp(s, cum, pts[:, 0]), np.interp(s, cum, pts[:, 1])]
        g = b.grid
        pad = r.body_r + g.res
        off = np.array([(0, 0), (pad, 0), (-pad, 0), (0, pad), (0, -pad)])
        q = (xy[:, None, :] + off[None, :, :]).reshape(-1, 2)
        tm = b.T * g.res
        tx = np.clip(np.floor((q[:, 0] - g.ox) / tm).astype(np.int64), 0, b.ntx - 1)
        ty = np.clip(np.floor((q[:, 1] - g.oy) / tm).astype(np.int64), 0, b.nty - 1)
        return set(np.unique(ty * b.ntx + tx).tolist())

    # ---- the map tick -------------------------------------------------------

    def tick(self, r) -> None:
        """At r's map tick, before its patches: announce (and pull) if due."""
        now = self.sim.now
        m = self.st[r.k]
        m.alive_until = now
        self._expire(m, now)
        m.path = r.path
        want = self.wanted(r)
        if (want != m.tiles and now >= m.next_ok) or now >= m.next_keep:
            self._announce(r, m, want, now)

    def step(self, r) -> None:
        """Every step: a new path subscribes at once (within the minimum
        interval), not at the next map tick. A robot given a task plans
        before it has heard of the tiles on its way; waiting for the tick, one
        drove 1.5 s along a path through a pallet the fleet had known about
        for 160 s before its pull answer came (map_sharing, 2026-11-26)."""
        m = self.st[r.k]
        if r.path is m.path or not r.path or self.sim.now < m.next_ok:
            return
        m.path = r.path
        want = self.wanted(r)
        if want - m.tiles:
            self._announce(r, m, want, self.sim.now)

    def _expire(self, m: _Robot, now: float) -> None:
        old = [p for p, (_v, _t, at) in m.view.items() if now - at > self.expire]
        for p in old:
            for t in m.view.pop(p)[1]:
                m.subs_of[t].pop(p, None)

    def _announce(self, r, m: _Robot, want: frozenset, now: float) -> None:
        added = want - m.tiles
        for t in m.tiles - want:
            m.left[t] = now
        m.tiles, m.ver = want, m.ver + 1
        m.next_ok, m.next_keep = now + self.min_iv, now + self.keep
        self.counts["announces"] += 1
        dsts = self.robot_nodes[self.robot_nodes != r.node]
        self.sim._send("mapsub", r.node, dsts, SUB_HEADER_B + SUB_TILE_B * len(want), True,
                       self._deliver_announce, (r.node, m.ver, want))
        if added:
            self._pull(r, m, added, now)

    def _deliver_announce(self, node: int, payload) -> None:
        peer, ver, tiles = payload
        o = self.sim.robots[node - 1]
        if not o.alive:
            return
        m = self.st[o.k]
        old = m.view.get(peer)
        if old is not None and old[0] >= ver:
            return  # older than what we have (reliable is in order, but a lease can drop one)
        now = self.sim.now
        before = old[1] if old is not None else frozenset()
        for t in before - tiles:
            m.subs_of[t].pop(peer, None)
            m.last_left[t] = (now, peer)
        for t in tiles - before:
            m.subs_of[t][peer] = now
        m.view[peer] = (ver, tiles, now)

    def dsts(self, r, tile: int, flip: bool = False) -> np.ndarray:
        if flip and self.flips_all:
            return self.robot_nodes[self.robot_nodes != r.node]
        subs = self.st[r.k].subs_of.get(tile)
        if not subs:
            return np.zeros(0, dtype=np.int64)
        return np.array(sorted(p for p in subs if p != r.node), dtype=np.int64)

    # ---- pulls ------------------------------------------------------------

    def _source(self, r, m: _Robot, tile: int, now: float) -> int | None:
        """The one peer to ask: a current subscriber, nearest us first (the
        cheapest answer on the radio), preferring one subscribed long enough
        to have had its own answer; else whoever left the tile last."""
        subs = {p: t for p, t in m.subs_of.get(tile, {}).items() if p != r.node}
        sim = self.sim
        if subs:
            def key(p):
                j = p - 1
                d = math.inf
                if now - sim.seen_t[r.k, j] <= POSE_FRESH_S:
                    d = math.hypot(sim.seen_xy[r.k, j, 0] - r.x, sim.seen_xy[r.k, j, 1] - r.y)
                return (now - subs[p] < self.pull_wait, d, subs[p], p)
            return min(subs, key=key)
        left = m.last_left.get(tile)
        if left is not None and left[1] != r.node:
            return left[1]
        return None

    def _pull(self, r, m: _Robot, added: frozenset, now: float) -> None:
        by_src: dict[int, list] = defaultdict(list)
        for t in sorted(added):
            src = self._source(r, m, t, now)
            if src is None:
                self.counts["no_source"] += 1
                continue
            since = max(m.left[t] - self.margin, self.prior_since) if t in m.left \
                else self.prior_since
            by_src[src].append((t, since))
            m.pulling[t] = now + self.pull_wait
        for src, items in by_src.items():
            self.counts["pulls"] += 1
            self.counts["pulled_tiles"] += len(items)
            self.sim._send("mappull", r.node, np.array([src]),
                           PULL_HEADER_B + PULL_TILE_B * len(items), True,
                           self._deliver_pull, (r.node, items))

    def _deliver_pull(self, node: int, payload) -> None:
        asker, items = payload
        o = self.sim.robots[node - 1]
        if not o.alive:
            return
        m = self.st[o.k]
        for t, since in items:
            key = (asker, t)
            m.answers[key] = min(since, m.answers.get(key, math.inf))
        self.flush(o)

    def flush(self, o) -> None:
        """Send queued answers, oldest first, while the budget lasts (after
        own patches at a tick; at once when a pull arrives)."""
        m = self.st[o.k]
        while m.answers:
            (asker, t), since = next(iter(m.answers.items()))
            if not self._answer(o, t, since, asker):
                return  # over the cap: the rest wait for the next tick
            m.answers.popitem(last=False)

    def _answer(self, o, tile: int, since: float, asker: int) -> bool:
        sim, c = self.sim, self.sim.cfg["map_share"]
        idx, st, ts = o.belief.answer_payload(tile, since)
        vv = o.belief.seq_vector(tile, o.node)
        per_chunk = max(1, int((float(c["chunk_bytes"]) - sim._map_bytes(0))
                               / max(sim.grid.res ** 2 * float(c["bytes_per_m2"]), 1e-9)))
        chunks = [slice(i, i + per_chunk) for i in range(0, max(len(idx), 1), per_chunk)]
        sizes = [sim._map_bytes(len(idx[s])) for s in chunks]
        sizes[-1] += VV_ENTRY_B * len(vv)
        if o.tokens < sum(sizes):
            return False
        o.tokens -= sum(sizes)
        seq = o.belief.out_seq.get(tile, 0)
        for n, (s, size) in enumerate(zip(chunks, sizes)):
            last = n == len(chunks) - 1
            sim._send("mapans", o.node, np.array([asker]), size, False, self._deliver_answer,
                      (o.node, tile, seq, (idx[s], st[s], ts[s]), vv if last else None))
        self.counts["answers"] += 1
        self.counts["answer_cells"] += len(idx)
        return True

    def _deliver_answer(self, node: int, payload) -> None:
        src, tile, seq, cells, vv = payload
        r = self.sim.robots[node - 1]
        if not r.alive:
            return
        r.belief.check_seq(src, tile, seq, True)
        changed = r.belief.merge(*cells)
        r.counters["shared_changed"] += len(changed)
        r.on_belief_change(changed, "shared")
        if vv is not None:
            r.belief.adopt_seq(tile, vv, r.node)
            self.st[r.k].pulling.pop(tile, None)

    def owed(self, r, tile: int) -> bool:
        """Does a gap in a tile's deltas call for a MapResync? Only while
        subscribed (news of a tile it doesn't subscribe to comes fleet-wide,
        so gaps are the norm there), and not while a pull of it is open (the
        answer is about to fill it)."""
        m = self.st[r.k]
        if tile not in m.tiles:
            return False
        until = m.pulling.get(tile)
        return until is None or self.sim.now >= until

    def on_revive(self, r) -> None:
        """Back from the dead: deaf since its last tick, so it has left every
        tile then and subscribes (and pulls) afresh at its next tick."""
        m = self.st[r.k]
        for t in m.tiles:
            m.left[t] = m.alive_until
        m.tiles = frozenset()
        m.next_ok = m.next_keep = self.sim.now
        m.answers.clear()
        m.pulling.clear()

    def summary(self) -> dict:
        return dict(self.counts)


def build(sim) -> Subscriptions | None:
    if sim.cfg["map_share"]["recipients"] != "subscribers":
        return None
    return Subscriptions(sim)
