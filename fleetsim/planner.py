"""Per-robot costmap and global A* on the robot's OWN belief grid.

Modelled on what the robots run (configs/nav2_params.yaml), not invented:

  * Lethal: a belief-occupied cell, and every cell whose centre is within the
    robot's body radius (+ half a cell, so a robot BETWEEN two cell centres
    still clears) of one. The robot is then planned as a point.
  * Inflation: beyond lethal, a cost decaying as exp(-cost_scaling * d) out to
    `inflation_m` (Nav2: inflation_radius 0.65, cost_scaling_factor 2.5), so
    paths keep to the middle of an aisle but still fit a doorway.
  * Unknown cells cost `unknown_cost` extra (0 = optimistic, the team's
    choice for now — Nav2's allow_unknown: true). A cautious policy is a
    number here, not new code.
  * A* over 8 neighbours, octile heuristic (exact on an open grid, never an
    overestimate, so the path found is the cheapest), no diagonal step past
    a lethal cell. Step cost = length x (1 + cost_weight * inflation cost).

An `overlay` mask adds short-lived lethal cells on top of the belief: the
local layer's "something is in the way" (peers, movers) when it replans
around a blockage. It never enters the belief, so it is never shared — the
same split the real stack keeps between /map and the costmap obstacle layer.
"""
from __future__ import annotations

import heapq
import math
from array import array

import numpy as np

from .grid import Grid

UNKNOWN, FREE, OCC = -1, 0, 100
INF = float("inf")
NEIGHBOURS_INT = ((1, 0, 1000), (-1, 0, 1000), (0, 1, 1000), (0, -1, 1000),
                  (1, 1, 1414), (1, -1, 1414), (-1, 1, 1414), (-1, -1, 1414))


class Costmap:
    def __init__(self, grid: Grid, state: np.ndarray, nav: dict, body_r: float):
        self.grid = grid
        self.state = state  # (h, w) int8, the belief's own array (shared, not copied)
        self.overlay = np.zeros((grid.height, grid.width), dtype=bool)
        self.r_lethal = body_r + 0.5 * grid.res
        self.infl = max(float(nav["inflation_m"]), self.r_lethal)
        self.scaling = float(nav["cost_scaling"])
        self.weight = float(nav["cost_weight"])
        self.unknown_cost = float(nav["unknown_cost"])
        self.offsets = sorted(grid.disc_offsets(self.infl), key=lambda o: o[2])
        self.R = max(max(abs(dx), abs(dy)) for dx, dy, _ in self.offsets)
        self.dist = np.full((grid.height, grid.width), np.inf, dtype=np.float32)
        # Flat mirror of the step multiplier for A*: array('f') indexes from
        # python at list speed with a quarter of a list's memory.
        self.mult = array("f", bytes(4 * grid.size))
        # The same multiplier as integers (x1000; 0 = lethal) for A*'s inner
        # loop, which runs on ints only. On CPython 3.14.4 the float version
        # intermittently raised "'<' not supported between 'Costmap' and
        # 'float'" on `ng < gscore[j]` with every local a float — once in
        # five to ten full runs, at a different run each time, never under a
        # standalone stress test (2026-11-02). Integer arithmetic allocates
        # no floats in the loop and the TypeError stopped, but segfaults
        # elsewhere since suggest the fault (interpreter or CPU, unresolved)
        # only moved; see fleetsim/README.md "Known issues".
        self.wint = array("l", bytes(array("l").itemsize * grid.size))
        self.version = 0
        self.rebuild()

    # ---- building -------------------------------------------------------

    def rebuild(self) -> None:
        self.update_window(0, 0, self.grid.width, self.grid.height)

    def update_cells(self, idx: np.ndarray) -> None:
        """Recompute around changed cells (flat indices): their bounding box
        grown by the inflation radius."""
        if idx is None or len(idx) == 0:
            return
        w = self.grid.width
        xs, ys = idx % w, idx // w
        self.update_window(int(xs.min()) - self.R, int(ys.min()) - self.R,
                           int(xs.max()) + self.R + 1, int(ys.max()) + self.R + 1)

    def update_window(self, x0: int, y0: int, x1: int, y1: int) -> None:
        g = self.grid
        x0, y0 = max(0, x0), max(0, y0)
        x1, y1 = min(g.width, x1), min(g.height, y1)
        if x0 >= x1 or y0 >= y1:
            return
        R = self.R
        src = (self.state == OCC) | self.overlay
        # Source window padded by R with "no obstacle" beyond the map edge.
        pad = np.zeros((y1 - y0 + 2 * R, x1 - x0 + 2 * R), dtype=bool)
        sy0, sy1 = max(0, y0 - R), min(g.height, y1 + R)
        sx0, sx1 = max(0, x0 - R), min(g.width, x1 + R)
        pad[sy0 - (y0 - R): sy1 - (y0 - R), sx0 - (x0 - R): sx1 - (x0 - R)] = src[sy0:sy1, sx0:sx1]
        h, w = y1 - y0, x1 - x0
        cur = np.full((h, w), np.inf, dtype=np.float32)
        for dx, dy, d in self.offsets:
            sub = pad[R + dy: R + dy + h, R + dx: R + dx + w]
            np.minimum(cur, np.where(sub, np.float32(d), np.float32(np.inf)), out=cur)
        self.dist[y0:y1, x0:x1] = cur
        with np.errstate(over="ignore", invalid="ignore"):
            m = 1.0 + self.weight * np.exp(-self.scaling * (cur - self.r_lethal))
        m = np.where(cur > self.infl, 1.0, m)
        if self.unknown_cost:
            m = m + self.unknown_cost * (self.state[y0:y1, x0:x1] == UNKNOWN)
        m = np.where(cur <= self.r_lethal, np.inf, m).astype(np.float32)
        wi = np.where(np.isinf(m), 0, np.round(m * 1000.0)).astype(np.dtype(array("l").typecode))
        for r in range(h):
            base = (y0 + r) * g.width + x0
            self.mult[base: base + w] = array("f", m[r].tobytes())
            self.wint[base: base + w] = array("l", wi[r].tobytes())
        self.version += 1

    def lethal(self, idx: int) -> bool:
        return self.mult[idx] == INF

    def lethal_xy(self, x: float, y: float) -> bool:
        i = self.grid.index(x, y)
        return i is None or self.mult[i] == INF

    # ---- search ---------------------------------------------------------

    def nearest_free(self, idx: int, within_m: float) -> int | None:
        g = self.grid
        cx, cy = idx % g.width, idx // g.width
        for dx, dy, d in sorted(g.disc_offsets(within_m), key=lambda o: o[2]):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < g.width and 0 <= ny < g.height:
                j = ny * g.width + nx
                if self.mult[j] != INF:
                    return j
        return None

    def plan(self, start, goal, *, goal_tol_m: float = 0.5,
             max_expansions: int = 200_000) -> list[tuple[float, float]] | None:
        """Cell-centre path start -> goal (world xy), or None. A goal inside
        lethal space snaps to the nearest free cell within goal_tol_m (a pin
        placed close to a shelf); a start inside it (the robot just learned
        of a wall beside it) first steps to the nearest free cell within
        1 m, the escape Nav2's recovery would do."""
        g = self.grid
        si, gi = g.index(*start), g.index(*goal)
        if si is None or gi is None:
            return None
        if self.mult[gi] == INF:
            gi = self.nearest_free(gi, goal_tol_m)
            if gi is None:
                return None
        prefix = []
        if self.mult[si] == INF:
            esc = self.nearest_free(si, 1.0)
            if esc is None:
                return None
            prefix = [g.centre_of(si)]
            si = esc
        if si == gi:
            return prefix + [g.centre_of(gi)]
        w, h, wint = g.width, g.height, self.wint
        gx, gy = gi % w, gi // w
        # Integer costs: a step is 1000 (orthogonal) or 1414 (diagonal) per
        # cell, times the cell's x1000 multiplier; the heuristic is octile in
        # the same units with multiplier 1000 (the floor), so it never
        # overestimates. See the note on self.wint.
        gscore = [-1] * g.size
        came = [-1] * g.size
        gscore[si] = 0
        sx, sy = si % w, si // w
        ax, ay = abs(sx - gx), abs(sy - gy)
        h0 = 1000 * (1000 * ax + 414 * ay) if ax > ay else 1000 * (1000 * ay + 414 * ax)
        heap = [(h0, 0, si)]
        pop, push = heapq.heappop, heapq.heappush
        expansions = 0
        found = False
        while heap:
            f, gs, i = pop(heap)
            if i == gi:
                found = True
                break
            if gs > gscore[i]:
                continue
            expansions += 1
            if expansions > max_expansions:
                return None
            x, y = i % w, i // w
            for dx, dy, step in NEIGHBOURS_INT:
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                j = ny * w + nx
                wj = wint[j]
                if wj == 0:
                    continue
                if step > 1000 and (wint[y * w + nx] == 0 or wint[ny * w + x] == 0):
                    continue
                ng = gs + step * wj
                old_g = gscore[j]
                if old_g < 0 or ng < old_g:
                    gscore[j] = ng
                    came[j] = i
                    ax, ay = abs(nx - gx), abs(ny - gy)
                    if ax > ay:
                        hh = 1000 * (1000 * ax + 414 * ay)
                    else:
                        hh = 1000 * (1000 * ay + 414 * ax)
                    push(heap, (ng + hh, ng, j))
        if not found:
            return None
        cells = [gi]
        while cells[-1] != si:
            cells.append(came[cells[-1]])
        cells.reverse()
        return prefix + [g.centre_of(c) for c in cells]


def path_length(pts) -> float:
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
