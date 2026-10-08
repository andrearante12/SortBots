"""Floor layouts: rack rectangles, aisle routing, obstruction counting.

The real lab mock-up and target warehouse dimensions are open questions
(plan §11 q2). `warehouse()` is a PLACEHOLDER generic layout — parallel rack
rows, aisles between them, cross-aisles at both ends — so the model has
something rack-shaped to attenuate through. Replace it when the team supplies
a floor plan.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field


@dataclass
class Layout:
    width: float
    height: float
    racks: list = field(default_factory=list)      # (x0, y0, x1, y1), strong attenuation
    shelves: list = field(default_factory=list)    # wire shelving, mild attenuation
    aisles_y: list = field(default_factory=list)   # aisle centerlines
    cross_x: tuple = (2.5, 47.5)                   # cross-aisle centerlines
    gateway_pos: tuple = (25.0, 0.5)

    def crossings(self, a: tuple, b: tuple) -> tuple[int, int]:
        return (sum(_seg_hits_rect(a, b, r) for r in self.racks),
                sum(_seg_hits_rect(a, b, r) for r in self.shelves))

    def random_spot(self, rng: random.Random) -> tuple:
        y = rng.choice(self.aisles_y)
        x = rng.uniform(self.cross_x[0] + 3, self.cross_x[1] - 3)
        return (x, y)

    def route(self, a: tuple, b: tuple) -> list:
        """Waypoints along aisles: straight if same aisle, else via the
        cheaper cross-aisle."""
        if abs(a[1] - b[1]) < 1e-6 or not self.racks:
            return [a, b]
        xc = min(self.cross_x, key=lambda c: abs(a[0] - c) + abs(b[0] - c))
        return [a, (xc, a[1]), (xc, b[1]), b]


def warehouse(width=50.0, height=32.0, aisle=3.0, rack_depth=2.4, end_gap=5.0) -> Layout:
    racks, aisles = [], []
    y = 0.0
    while y + aisle <= height:
        aisles.append(y + aisle / 2)
        y += aisle
        if y + rack_depth + aisle > height:
            break
        racks.append((end_gap, y, width - end_gap, y + rack_depth))
        y += rack_depth
    return Layout(width, height, racks=racks, aisles_y=aisles,
                  cross_x=(end_gap / 2, width - end_gap / 2),
                  gateway_pos=(width / 2, aisles[0]))


def open_floor(width=30.0, height=20.0) -> Layout:
    """No obstructions — sanity baseline and closest stand-in for the lab
    until its shelving positions are known."""
    return Layout(width, height, aisles_y=[height * f for f in (0.2, 0.4, 0.6, 0.8)],
                  cross_x=(1.0, width - 1.0), gateway_pos=(width / 2, 0.5))


LAYOUTS = {"warehouse": warehouse, "open_floor": open_floor}


def path_length(pts: list) -> float:
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def _seg_hits_rect(a, b, r) -> bool:
    """Liang–Barsky: does segment a-b pass through the interior of rect r?"""
    x0, y0, x1, y1 = r
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - x0), (dx, x1 - a[0]), (-dy, a[1] - y0), (dy, y1 - a[1])):
        if p == 0:
            if q < 0:
                return False
            continue
        t = q / p
        if p < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return False
    return t1 - t0 > 1e-9
