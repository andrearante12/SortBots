"""Drive distance between two points along an occupancy grid.

fleet_tasks.py bids with this instead of the straight line: a rack between
robot and shelf made the line an underestimate, and with several queued
tasks the error compounded along the chain. The fused /map is downsampled
to coarse cells (a cell is blocked if any fine cell in it is occupied, then
blocked cells are grown by one so the robot's body fits) and the path is
Dijkstra over 8-neighbours. A few thousand cells: milliseconds per bid.

Unknown space counts as blocked, so a target outside the explored map has no
path; callers fall back to the straight line (fleet_tasks._path_m).
Pure Python, no ROS: the OccupancyGrid fields are passed in.
"""
from __future__ import annotations

import heapq
import math

SQRT2 = math.sqrt(2.0)


class GridDistance:
    def __init__(self, width: int, height: int, resolution: float, origin_xy: tuple,
                 data, cell_m: float = 0.25, occupied_from: int = 50, grow: int = 1):
        self.res = float(resolution)
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        self.k = max(1, int(round(cell_m / self.res)))  # fine cells per coarse cell
        self.cell = self.k * self.res
        self.w = (int(width) + self.k - 1) // self.k
        self.h = (int(height) + self.k - 1) // self.k
        blocked = bytearray(self.w * self.h)
        for y in range(int(height)):
            row = y * int(width)
            cy = (y // self.k) * self.w
            for x in range(int(width)):
                v = data[row + x]
                if v < 0 or v >= occupied_from:
                    blocked[cy + x // self.k] = 1
        if grow:
            self.blocked = bytearray(blocked)
            for cy in range(self.h):
                for cx in range(self.w):
                    if blocked[cy * self.w + cx]:
                        for dy in range(-grow, grow + 1):
                            for dx in range(-grow, grow + 1):
                                nx, ny = cx + dx, cy + dy
                                if 0 <= nx < self.w and 0 <= ny < self.h:
                                    self.blocked[ny * self.w + nx] = 1
        else:
            self.blocked = blocked

    def _cell(self, xy) -> tuple[int, int] | None:
        cx = int((float(xy[0]) - self.ox) / self.cell)
        cy = int((float(xy[1]) - self.oy) / self.cell)
        if 0 <= cx < self.w and 0 <= cy < self.h:
            return cx, cy
        return None

    def free(self, xy) -> bool:
        c = self._cell(xy)
        return c is not None and not self.blocked[c[1] * self.w + c[0]]

    def path_length(self, a, b) -> float | None:
        """Metres along free cells from a to b, or None if either is outside
        the map or blocked, or no path exists. Endpoint cells are allowed
        even when blocked by growth: docks sit right next to the shelf."""
        ca, cb = self._cell(a), self._cell(b)
        if ca is None or cb is None:
            return None
        if ca == cb:
            return math.dist(a, b)
        w, blocked = self.w, self.blocked
        goal = cb[1] * w + cb[0]
        start = ca[1] * w + ca[0]
        dist = {start: 0.0}
        heap = [(0.0, start)]
        while heap:
            d, i = heapq.heappop(heap)
            if i == goal:
                return d * self.cell
            if d > dist.get(i, math.inf):
                continue
            x, y = i % w, i // w
            for dx, dy, step in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                                 (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)):
                nx, ny = x + dx, y + dy
                if not (0 <= nx < w and 0 <= ny < self.h):
                    continue
                j = ny * w + nx
                if blocked[j] and j != goal:
                    continue
                if step > 1.0 and (blocked[y * w + nx] or blocked[ny * w + x]):
                    continue  # no cutting corners through a blocked cell
                nd = d + step
                if nd < dist.get(j, math.inf):
                    dist[j] = nd
                    heapq.heappush(heap, (nd, j))
        return None
