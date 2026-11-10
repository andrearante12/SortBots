"""Floor layouts: rack rectangles, aisle routing, obstruction counting.

The real lab mock-up and target warehouse dimensions are open questions
(plan §11 q2). `warehouse()` is a PLACEHOLDER generic layout — parallel rack
rows, aisles between them, cross-aisles at both ends — so the model has
something rack-shaped to attenuate through. Replace it when the team supplies
a floor plan.

`from_occupancy()` is the other source: the racks of a real explored map
(nodes/map_merge.py's /map), for the operator view's comms overlay.
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
    # Optional, parallel to `racks`: which obstacle each rectangle belongs to.
    # from_occupancy() covers one wall with many rectangles; a link through
    # that wall must cost ONE rack's attenuation, not one per rectangle.
    groups: list = field(default_factory=list)
    _group_box: dict = field(default_factory=dict, repr=False)

    def crossings(self, a: tuple, b: tuple) -> tuple[int, int]:
        if self.groups:
            racks = self._groups_hit(a, b)
        else:
            racks = sum(_seg_hits_rect(a, b, r) for r in self.racks)
        return racks, sum(_seg_hits_rect(a, b, r) for r in self.shelves)

    def _groups_hit(self, a: tuple, b: tuple) -> int:
        if not self._group_box:
            for r, g in zip(self.racks, self.groups):
                x0, y0, x1, y1 = self._group_box.get(g, r)
                self._group_box[g] = (min(x0, r[0]), min(y0, r[1]), max(x1, r[2]), max(y1, r[3]))
        hit = set()
        for r, g in zip(self.racks, self.groups):
            # Cheap reject on the whole obstacle's box before its rectangles.
            if g in hit or not _seg_hits_rect(a, b, self._group_box[g]):
                continue
            if _seg_hits_rect(a, b, r):
                hit.add(g)
        return len(hit)

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


def from_occupancy(occ: str, w: int, h: int, cell: float, *, min_area_m2: float = 0.5,
                   rack_min_area_m2: float = 2.0, gateway_pos: tuple | None = None) -> Layout:
    """Racks from an occupancy grid: every connected blob of occupied cells.

    `occ` is row-major, row 0 at the BOTTOM (the OccupancyGrid convention),
    one character per `cell`-metre square: "1" occupied, anything else not.
    The caller downsamples (webui/operator_comms.js: a cell is occupied if
    any map cell in it is). Coordinates come out in metres from the grid's
    corner, so the model's [0, width] x [0, height] floor IS the map.

    Each blob (4-connected) becomes one obstacle group, covered by rectangles
    (runs of occupied cells per row, merged down while the run is identical).
    A blob's bounding box is NOT used as the rack: the warehouse's outer wall
    is one blob, and its box is the whole floor — every link would cross it.
    Blobs under `min_area_m2` are dropped: scan speckle must not attenuate.

    Only blobs of `rack_min_area_m2` or more are racks (rack_db, 15 dB at 2.4 GHz).
    Smaller ones (table legs, boxes, clutter) are light shelving (shelf_db, 3 dB), one
    rectangle each (their bounding box: they are small, so the box is the blob). Live
    2026-10-24: with every blob a rack, the clutter round a table cost a link 4 racks
    (60 dB) and the overlay called a robot 6.6 m from the gateway cut off.
    """
    if len(occ) != w * h:
        raise ValueError(f"occupancy has {len(occ)} cells, expected {w}x{h}")
    filled = [c == "1" for c in occ]
    label = [-1] * (w * h)
    blobs = []
    for start in range(w * h):
        if not filled[start] or label[start] >= 0:
            continue
        gid, stack, cells = len(blobs), [start], []
        label[start] = gid
        while stack:
            i = stack.pop()
            cells.append(i)
            x, y = i % w, i // w
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if 0 <= nx < w and 0 <= ny < h:
                    j = ny * w + nx
                    if filled[j] and label[j] < 0:
                        label[j] = gid
                        stack.append(j)
        blobs.append(cells)
    area = {g: len(cells) * cell * cell for g, cells in enumerate(blobs)}
    keep = {g for g, a in area.items() if a >= rack_min_area_m2}
    shelves = []
    for g, cells in enumerate(blobs):
        if min_area_m2 <= area[g] < rack_min_area_m2:
            xs, ys = [i % w for i in cells], [i // w for i in cells]
            shelves.append((min(xs) * cell, min(ys) * cell, (max(xs) + 1) * cell, (max(ys) + 1) * cell))
    # Row runs per blob, then merge each run into the rectangle directly below
    # it when that one spans exactly the same columns.
    racks, groups = [], []
    open_rects: dict = {}  # (x0, x1, gid) -> index into racks, for the previous row
    for y in range(h):
        nxt = {}
        x = 0
        while x < w:
            g = label[y * w + x]
            if g < 0 or g not in keep:
                x += 1
                continue
            x0 = x
            while x < w and label[y * w + x] == g:
                x += 1
            key = (x0, x, g)
            if key in open_rects:
                k = open_rects[key]
                r = racks[k]
                racks[k] = (r[0], r[1], r[2], (y + 1) * cell)
            else:
                k = len(racks)
                racks.append((x0 * cell, y * cell, x * cell, (y + 1) * cell))
                groups.append(g)
            nxt[key] = k
        open_rects = nxt
    width, height = w * cell, h * cell
    return Layout(width, height, racks=racks, groups=groups, shelves=shelves,
                  aisles_y=[height / 2], cross_x=(cell, width - cell),
                  gateway_pos=tuple(gateway_pos) if gateway_pos else (width / 2, height / 2))


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
