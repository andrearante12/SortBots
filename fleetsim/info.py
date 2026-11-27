"""Perfect information: what a robot would do if it knew the truth.

Independent knobs under `info:` (fleetsim/configs/base.yaml), each one
channel of knowledge, all defaulting to the robots as they are:

  map: truth       every robot's belief and costmap follow the true grid,
                   blockages included, the moment it changes; a central
                   planner's drive times do too (dispatch/costs.py).
  peers: truth     peer poses for the local layer, and path intents, come
                   from ground truth every step instead of /fleet/status.
  dispatch: truth  /fleet/tasks is delivered instantly and never lost (the
                   auction's whole protocol, or a central dispatcher's
                   Dispatch/Cancel/Report), and a central dispatcher reads
                   each robot's true pose and liveness (sim.py, dispatch/).

The full perfect-information reference is all three at truth plus
`network.model: ideal` with `network.ideal.delay_s: 0`. Measuring a method
with and without them is the information gap; the coordination gap is a
central method against the auction under the same information.

This module holds the first two. They change only how robots KNOW things:
a robot still drives, senses, bumps and talks over the network as before,
so map patches, Status and notices still cost airtime (turn map sharing
off with map_share.enabled: false to take its traffic out as well).
"""
from __future__ import annotations

import numpy as np

from .planner import FREE, OCC


class PerfectInfo:
    def __init__(self, sim):
        c = sim.cfg["info"]
        self.sim = sim
        self.map_truth = c["map"] == "truth"
        self.peers_truth = c["peers"] == "truth"
        self._intent_built: dict[int, float] = {}

    def map_changed(self, sl: tuple) -> None:
        """The true grid changed in the window `sl` (a blockage came or
        went). That is the only place it ever changes, and sensing and
        patches only ever write the truth, so syncing here keeps every
        belief equal to the truth at every step."""
        if not self.map_truth:
            return
        sim, w = self.sim, self.sim.grid.width
        ys = np.arange(sl[0].start, sl[0].stop)
        xs = np.arange(sl[1].start, sl[1].stop)
        idx = (ys[:, None] * w + xs[None, :]).ravel().astype(np.int64)
        truth = np.where(sim.static_flat[idx], OCC, FREE).astype(np.int8)
        for r in sim.robots:
            b = r.belief
            changed = idx[b.s[idx] != truth]
            b.s[idx] = truth
            b.ts[idx] = np.float32(sim.now)
            r.on_belief_change(changed, "truth")

    def step(self, now: float) -> None:
        if not self.peers_truth:
            return
        sim = self.sim
        pos = np.array([(r.x, r.y) for r in sim.robots])
        # Everyone, the dead included: a dead robot is still in the way.
        sim.seen_t[:] = now
        sim.seen_xy[:] = pos[None, :, :]
        if not sim.cfg["intent"]["enabled"]:
            return
        for r in sim.robots:
            m = getattr(r, "intent_msg", None)
            if m is None or self._intent_built.get(r.k) == m["built"]:
                continue
            self._intent_built[r.k] = m["built"]
            age = now - m["built"]
            msg = {**m, "pts": [[x, y, round(dt - age, 2)] for x, y, dt in m["pts"]]}
            for o in sim.robots:
                if o is not r and o.alive:
                    o.on_intent(msg, now, truth=True)


def build(sim) -> PerfectInfo | None:
    c = sim.cfg["info"]
    if c["map"] == "shared" and c["peers"] == "heard":
        return None  # nothing to do: not even a per-step call
    return PerfectInfo(sim)
