"""Drive times for a central planner: distance fields on the TRUE map.

A warehouse's layout is known to its operator, so a central planner prices
a robot -> pickup leg on the true static floor at `robots.max_speed_mps`,
not on any robot's belief. What it does NOT know is a blockage that
appeared since: that reaches it only through what robots report (a leg that
fails), unless perfect map information is on (`info.map: truth`), in which
case the fields follow the live true grid and are rebuilt when it changes.

One A* per robot-task pair per solve is far too slow at 20 x 20 (each is a
flood of the floor in pure python), so the cost is a distance field per
waypoint instead: one Dijkstra from the waypoint over the cells a robot's
centre may occupy (the same lethal band the robots plan with: body radius
+ half a cell from any obstacle), built lazily on first use and cached.
Any robot is then one array lookup. A field costs ~40 ms on the default
60 x 40 m floor at 0.2 m cells (2026-11-26, i9-14900HX), and a run needs
at most one per waypoint a task names.

Distances are 8-connected cell steps (orthogonal 1, diagonal sqrt 2, no
corner cutting past a lethal cell, as planner.py's A*), so they are the
length of the shortest grid path, a little longer than the smoothed path
a robot actually drives; every robot is priced the same way, which is what
an assignment needs.
"""
from __future__ import annotations

import heapq
import time

import numpy as np

from ..planner import FREE, OCC, Costmap

NEIGHBOURS = ((1, 0, 1000), (-1, 0, 1000), (0, 1, 1000), (0, -1, 1000),
              (1, 1, 1414), (1, -1, 1414), (-1, 1, 1414), (-1, -1, 1414))
UNREACHED = -1


class DriveTimes:
    def __init__(self, sim, live: bool):
        """live: follow sim.static (blockages included) instead of the
        layout the run started with (sim.base_static)."""
        self.sim = sim
        self.live = live
        self.grid = sim.grid
        self.speed = float(sim.cfg["robots"]["max_speed_mps"])
        self.goal_tol = float(sim.cfg["nav"]["goal_tolerance_m"])
        self.body_r = float(sim.cfg["robots"]["footprint_radius_m"])
        self.nav = sim.cfg["nav"]
        self._fields: dict[str, np.ndarray] = {}
        self._version = None
        self.builds = 0
        self.build_wall_s = 0.0
        self._rebuild_mask()

    # ---- the traversable mask ---------------------------------------------

    def _occ(self) -> np.ndarray:
        return self.sim.static if self.live else self.sim.base_static

    def _rebuild_mask(self) -> None:
        cm = Costmap(self.grid, np.where(self._occ(), OCC, FREE).astype(np.int8), self.nav,
                     self.body_r)
        self.cm = cm
        self.free = np.isfinite(np.frombuffer(cm.mult, dtype=np.float32))
        self._free_list = self.free.tolist()
        self._fields.clear()
        self._version = self.sim.static_version if self.live else None

    def _check(self) -> None:
        if self.live and self._version != self.sim.static_version:
            self._rebuild_mask()

    # ---- lookups ------------------------------------------------------------

    def snap(self, xy) -> int | None:
        """The traversable cell a robot at xy starts from: its own cell, or
        the nearest free one within 1 m (planner.plan's escape)."""
        self._check()
        i = self.grid.index(*xy)
        if i is None:
            return None
        if self.free[i]:
            return i
        return self.cm.nearest_free(i, 1.0)

    def field(self, name: str) -> np.ndarray:
        """Flat int array: grid-path length from waypoint `name` in
        1/1000 cells, UNREACHED where no path exists."""
        self._check()
        f = self._fields.get(name)
        if f is None:
            t0 = time.perf_counter()
            f = self._fields[name] = self._dijkstra(name)
            self.builds += 1
            self.build_wall_s += time.perf_counter() - t0
        return f

    def seconds(self, name: str, cells) -> np.ndarray:
        """Drive time from each of `cells` (snapped flat indices, None =
        off the floor) to waypoint `name`, inf where unreachable."""
        f = self.field(name)
        idx = np.array([-1 if c is None else c for c in cells], dtype=np.int64)
        d = np.where(idx >= 0, f[np.maximum(idx, 0)], UNREACHED)
        scale = self.grid.res / 1000.0 / self.speed
        return np.where(d >= 0, d * scale, np.inf)

    def between(self, a: str, b: str) -> float:
        """Drive time waypoint a -> waypoint b."""
        return float(self.seconds(b, [self.snap(self.sim.waypoint_xy(a))])[0])

    # ---- the field ------------------------------------------------------------

    def _dijkstra(self, name: str) -> np.ndarray:
        g = self.grid
        w, h = g.width, g.height
        out = np.full(g.size, UNREACHED, dtype=np.int64)
        src = g.index(*self.sim.waypoint_xy(name))
        if src is None:
            return out
        if not self.free[src]:
            # A pin close to a shelf sits in the lethal band; robots snap the
            # goal the same way (planner.plan, nav.goal_tolerance_m).
            src = self.cm.nearest_free(src, self.goal_tol)
            if src is None:
                return out
        fl = self._free_list
        d = [-1] * g.size
        d[src] = 0
        heap = [(0, src)]
        pop, push = heapq.heappop, heapq.heappush
        while heap:
            dd, i = pop(heap)
            if dd > d[i]:
                continue
            x, y = i % w, i // w
            for dx, dy, c in NEIGHBOURS:
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                j = ny * w + nx
                if not fl[j]:
                    continue
                if c > 1000 and (not fl[y * w + nx] or not fl[ny * w + x]):
                    continue
                nd = dd + c
                old = d[j]
                if old < 0 or nd < old:
                    d[j] = nd
                    push(heap, (nd, j))
        out[:] = d
        return out
