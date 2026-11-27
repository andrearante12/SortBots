"""Recording only: how long a blockage takes to reach each robot's own map.

The `map_learn` stream (read by the `map_spread` metric). For every blockage
that appears, and every robot alive then, one event when the robot's OWN map
first has it (any of its cells occupied): `after_s` from the appearance,
`how` it got there (`sensed`: its camera; `shared`: a peer's patch, refresh,
resync or pull answer; `truth`: info.map truth; `already`: its map had those
cells occupied before), and `path_s`: when (from the appearance) the robot's
planned path first ran into it (within a body of it) while it didn't know,
null if it never did. Those are the robots for which how fast the news
spreads matters, and from then on is when it matters: a robot given a task
past the pallet ten minutes after it appeared needs it then, not before
(2026-11-26). For every
blockage that clears, the same for the robots whose map had it then: how
long until none of its cells is occupied in their map, since until then they
detour round a pallet that has left. A robot whose map did NOT have it then
can still take it in afterwards, from a peer that hasn't seen it go (a
refresh, a pull answer): a ghost. That is recorded too, as a clear event
with `ghost_s`, when (from the clear) its map took the ghost in.

A pair that never lands is recorded with `after_s: null` when the blockage
changes again or the run ends. This reads the robots' maps and paths and
changes nothing they do, so a run is the run without it plus these events
(test_map_learn_only_adds_its_events).
"""
from __future__ import annotations

import math

import numpy as np

from .planner import OCC


class LearnTracker:
    def __init__(self, sim):
        self.sim = sim
        g = sim.grid
        # A path cell this close to a blockage cell runs into it: the
        # planner's lethal band (footprint + half a cell), in whole cells.
        self.pad = int(math.ceil((float(sim.cfg["robots"]["footprint_radius_m"])
                                  + 0.5 * g.res) / g.res))
        self.cells: dict[tuple, np.ndarray] = {}  # (blockage, on) -> flat cells it changed
        self.near: dict[int, np.ndarray] = {}     # blockage -> cells a path must avoid
        self.t0: dict[tuple, float] = {}
        # robot k -> {(blockage, on): path_s for an appearance, ghost_s for a clear}
        self.pending: dict[int, dict] = {}
        self.watch: dict[int, set] = {}           # robot k -> cleared blockages it lacks

    def blockage(self, i: int, sl: tuple, on: bool) -> None:
        """Blockage i just came (on) or went; the true grid already shows it."""
        sim, g = self.sim, self.sim.grid
        self._close(i, not on)
        ys = np.arange(sl[0].start, sl[0].stop)
        xs = np.arange(sl[1].start, sl[1].stop)
        idx = (ys[:, None] * g.width + xs[None, :]).ravel().astype(np.int64)
        free = ~sim.base_static.reshape(-1)[idx]
        key = (i, on)
        self.t0[key] = sim.now
        if on:
            self.cells[key] = idx[free]
            p = self.pad
            ys = np.arange(max(0, sl[0].start - p), min(g.height, sl[0].stop + p))
            xs = np.arange(max(0, sl[1].start - p), min(g.width, sl[1].stop + p))
            self.near[i] = (ys[:, None] * g.width + xs[None, :]).ravel().astype(np.int64)
        else:
            # What it freed: a cell another standing blockage still covers is not.
            self.cells[key] = idx[free & ~sim.static_flat[idx]]
        cells = self.cells[key]
        if not len(cells):
            return
        for r in sim.robots:
            knows = bool((r.belief.s[cells] == OCC).any())
            if on:
                if not r.alive:
                    continue
                self.pending.setdefault(r.k, {})[key] = 0.0 if self._crosses(r, i) else None
                if knows:
                    self._done(r, key, "already")
            elif knows:  # a dead robot too: it comes back with the pallet in its map
                self.pending.setdefault(r.k, {})[key] = None
            else:
                self.watch.setdefault(r.k, set()).add(i)

    def _crosses(self, r, i: int) -> bool:
        """Does what is left of r's planned path run into blockage i?"""
        # r.path empty: _clear_goal or a failed plan, called while the old
        # path_idx is still set.
        if not r.path or not r.path_idx or i not in self.near:
            return False
        g = self.sim.grid
        p = np.asarray(r.path_idx, dtype=np.int64)
        cx = (p % g.width + 0.5) * g.res + g.ox
        cy = (p // g.width + 0.5) * g.res + g.oy
        here = int(np.argmin(np.hypot(cx - r.x, cy - r.y)))
        return bool(np.isin(p[here:], self.near[i]).any())

    def on_path(self, r) -> None:
        """r has a new path: does it now run into a blockage it doesn't know?"""
        pend = self.pending.get(r.k)
        if not pend:
            return
        for key, path_s in pend.items():
            if key[1] and path_s is None and self._crosses(r, key[0]):
                pend[key] = round(self.sim.now - self.t0[key], 3)

    def on_belief(self, r, changed: np.ndarray, cause: str) -> None:
        watch = self.watch.get(r.k)
        if watch:
            for i in sorted(watch):
                cells = self.cells[(i, False)]
                if not np.isin(changed, cells).any():
                    continue
                # Occupied in its map where the floor is now free (a later
                # blockage on the same spot is not a ghost of this one).
                if ((r.belief.s[cells] == OCC) & ~self.sim.static_flat[cells]).any():
                    watch.discard(i)
                    self.pending.setdefault(r.k, {})[(i, False)] = \
                        round(self.sim.now - self.t0[(i, False)], 3)
        pend = self.pending.get(r.k)
        if not pend:
            return
        for key in list(pend):
            cells = self.cells[key]
            if not np.isin(changed, cells).any():
                continue
            if bool((r.belief.s[cells] == OCC).any()) == key[1]:
                self._done(r, key, cause)

    def _done(self, r, key: tuple, how: str | None) -> None:
        when = self.pending[r.k].pop(key)
        after = None if how is None else round(self.sim.now - self.t0[key], 3)
        self.sim.record({"s": "map_learn", "rid": r.rid, "blockage": key[0], "on": key[1],
                         "after_s": after, "how": how,
                         **({"path_s": when} if key[1] else {"ghost_s": when})})

    def _close(self, i: int, on: bool) -> None:
        """Blockage i changed again: whoever never got its last change, didn't."""
        for k in sorted(self.pending):
            if (i, on) in self.pending[k]:
                self._done(self.sim.robots[k], (i, on), None)

    def flush(self) -> None:
        for k in sorted(self.pending):
            for key in list(self.pending[k]):
                self._done(self.sim.robots[k], key, None)
