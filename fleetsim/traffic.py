"""Traffic rules a robot applies on its OWN map (docs/fleetsim.md "Traffic:
keep right and single-lane passages", 2026-11-26). Both are off by default.

Both read the same corridor geometry: for every free cell of the belief, the
free run through it along x and along y. The shorter run is the corridor's
cross-section: its length is the free width, its axis says which way the
corridor runs, and the cell's place in it is its offset from the centre line.
On this axis-aligned floor that is the distance-to-obstacle field measured
across the corridor; its gradient points at the centre line, so "the left
half" below is exactly cross(move, gradient) < 0 (test_traffic.py checks it
against the Euclidean field). One run extraction per axis gives width, axis
and offset at once, where a Euclidean distance transform and a width estimate
from it would cost ~250 shifted passes. The price: a corridor more than ~40
degrees off the grid reads wider than it is, so it gets no lanes (and no
passage) rather than wrong ones.

Keep right (`traffic.keep_right`, B10). Most predicted conflicts with a
moving peer were head-on meetings along an aisle (2026-11-17), and nothing
kept the two robots apart: shelves sit on the aisle's centre line, so a leg
from shelf to shelf pulled taut ran down the middle, and a longer one hugged
whichever edge of the flat band the taut line met. The local layer then
stopped both, though a 2.4 m aisle fits two. A step's cost gets a lane term,
per direction, in corridor cells (two-way width, longer than wide; junctions
and open floor read as wide and are untouched):

    weight * corridor * along * clamp((sigma + lane_offset_m) / (2 * lane_offset_m), 0, 1)

sigma is the cell's offset from the centre line, positive to the LEFT of the
move; along is how much the move runs with the corridor (1 along, 0.71
diagonal, 0 across, so crossing an aisle costs nothing extra). It is zero
from lane_offset_m right of the centre outwards, so the right lane costs
what the aisle cost before and routes don't change, and the full weight from
lane_offset_m left of it. In a 2.4 m aisle the flat band's outer cells are
0.5 m either side of the centre: robots going opposite ways run 1.0 m apart,
past the local layer's 0.6 m (body + body + clearance margin), so they pass
without either stopping. The smoother may not shortcut further left than
the A* path it replaces was at the same point (`Lanes.follows`).

The field follows the belief (`Lanes.on_window`, from the costmap's own
window updates), but its rebuild waits for a replan and at most every
`refresh_s`: a scan that changes cells re-cuts whole rows and columns of
runs, and the lane term only steers, while lethal cells stay exact.

Single-lane passages (`traffic.passages`, B11): connected regions of free
cells whose width is at most `single_lane_m`, where two robots can't pass;
a dead-end bay is one you leave the way you came. See PassageAgent.
"""
from __future__ import annotations

import math
from array import array

import numpy as np

from .grid import inflate_bool
from .planner import FREE, NEIGHBOURS_INT, OCC

# Corridor weight fades over this much width past max_width_m, and over
# length / width from 1 to 3: a 2.4 m square room is no aisle.
WIDTH_TAPER_M = 0.4
CLAIM_B = 16  # a claim on Status: passage box (4 x int16 dm), claim time, priority, flags


def row_runs(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The runs of True along axis 1, in row-major order: (row, first
    column, length)."""
    h, w = mask.shape
    pad = np.zeros((h, w + 2), np.int8)
    pad[:, 1:-1] = mask
    d = np.diff(pad, axis=1).ravel()
    s, e = np.flatnonzero(d == 1), np.flatnonzero(d == -1)  # w + 1 diffs per row
    return s // (w + 1), s % (w + 1), e - s


def run_lengths(free: np.ndarray) -> np.ndarray:
    """Along axis 1: each free cell's run length (cells), 0 off the mask."""
    n = np.zeros(free.size, np.int32)
    _r, _c, ln = row_runs(free)
    if len(ln):
        n[free.ravel()] = np.repeat(ln, ln)
    return n.reshape(free.shape)


def free_runs(free: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Along axis 1: each free cell's run length (cells) and its offset from
    the run's centre (cells: 0 on an odd run's middle cell, +-0.5 beside an
    even run's middle). Zero off the mask."""
    h, w = free.shape
    n = np.zeros(h * w, np.int32)
    off = np.zeros(h * w, np.float32)
    _r, _c, ln = row_runs(free)
    if len(ln):
        # Free cells in row-major order are the runs' cells in run order.
        per = np.repeat(ln, ln)
        pos = np.arange(per.size) - np.repeat(np.cumsum(ln) - ln, ln)
        flat = free.ravel()
        n[flat] = per
        off[flat] = pos - (per - 1) / 2.0
    return n.reshape(h, w), off.reshape(h, w)


def corridor_geometry(rx, ry, px, py, res: float):
    """-> (width m, length m, offset m, along_x bool). The shorter run is the
    cross-section; along_x: the corridor runs along x (cross-section in y)."""
    along_x = rx > ry
    width = np.where(along_x, ry, rx) * res
    length = np.where(along_x, rx, ry) * res
    off = np.where(along_x, py, px) * res
    return width, length, off, along_x


def _frombuffer(a: array, shape) -> np.ndarray:
    return np.frombuffer(a, dtype=np.dtype(a.typecode)).reshape(shape)


class Lanes:
    """The keep-right term of one robot's costmap. `wdir[k]` is the x1000
    step multiplier for a step in direction NEIGHBOURS_INT[k]: the costmap's
    `wint` plus the lane term, 0 where lethal. Arrays are array.array (A*
    indexes them from python at list speed) with numpy views for updates."""

    def __init__(self, cm, cfg: dict, single_lane_m: float):
        g = self.g = cm.grid
        self.cm = cm
        self.weight = float(cfg["weight"])
        self.max_w = float(cfg["max_width_m"])
        self.min_w = float(single_lane_m)
        self.offset = max(float(cfg["lane_offset_m"]), 1e-3)
        self.refresh_s = float(cfg["refresh_s"])
        h, w = self.shape = (g.height, g.width)
        self.occ = np.zeros(self.shape, bool)  # the OCC mask the runs were cut from
        self.rx, self.px = np.zeros(self.shape, np.int32), np.zeros(self.shape, np.float32)
        self.ry, self.py = np.zeros(self.shape, np.int32), np.zeros(self.shape, np.float32)
        # The field, flat for the smoother's per-sample reads, with views:
        # corridor weight, signed offset from the centre line along x / y (m,
        # one of the two is 0), and 1 where the corridor runs along x.
        self.cf, self.oxf, self.oyf = (array("f", bytes(4 * g.size)) for _ in range(3))
        self.axf = array("b", bytes(g.size))
        self.c, self.ox, self.oy = (_frombuffer(a, self.shape) for a in (self.cf, self.oxf, self.oyf))
        self.ax = _frombuffer(self.axf, self.shape)
        # The lane term x1000 of a step east, west, north and south; a
        # diagonal's is 0.707 x the sum of its two (one of them is 0: a cell
        # lies in a corridor along x or along y).
        self._base = [np.zeros(self.shape, np.float32) for _ in range(4)]
        assert [(dx, dy) for dx, dy, _s in NEIGHBOURS_INT] == [
            (1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        self.wdir = [array("i", bytes(4 * g.size)) for _ in NEIGHBOURS_INT]
        self._wdir = [_frombuffer(a, self.shape) for a in self.wdir]
        self._wint = _frombuffer(cm.wint, self.shape)
        self._dirty_rows = np.zeros(h, bool)
        self._dirty_cols = np.zeros(w, bool)
        self.next_refresh = -math.inf
        self.refreshes = 0

    # ---- the costmap's hooks ----------------------------------------------

    def on_window(self, x0: int, y0: int, x1: int, y1: int) -> None:
        """The costmap rebuilt this window: note belief walls that changed
        (their rows and columns need new runs), and rewrite the direction
        arrays from the new wint with the lane term as it stands."""
        occ = self.cm.state[y0:y1, x0:x1] == OCC
        diff = occ != self.occ[y0:y1, x0:x1]
        if diff.any():
            ys, xs = np.nonzero(diff)
            self._dirty_rows[y0 + ys] = True
            self._dirty_cols[x0 + xs] = True
            self.occ[y0:y1, x0:x1] = occ
        self._write(y0, y1, x0, x1)

    def refresh(self, now: float, force: bool = False) -> bool:
        """Re-cut the runs of changed rows and columns, then the field and
        direction arrays there. At most every refresh_s unless forced."""
        if not (self._dirty_rows.any() or self._dirty_cols.any()):
            return False
        if not force and now < self.next_refresh:
            return False
        self.next_refresh = now + self.refresh_s
        free = ~self.occ
        h, w = self.shape
        regions = []
        rows = np.flatnonzero(self._dirty_rows)
        if len(rows):
            r0, r1 = int(rows[0]), int(rows[-1]) + 1
            self.rx[r0:r1], self.px[r0:r1] = free_runs(free[r0:r1])
            regions.append((r0, r1, 0, w))
        cols = np.flatnonzero(self._dirty_cols)
        if len(cols):
            c0, c1 = int(cols[0]), int(cols[-1]) + 1
            ry, py = free_runs(free[:, c0:c1].T)
            self.ry[:, c0:c1], self.py[:, c0:c1] = ry.T, py.T
            regions.append((0, h, c0, c1))
        self._dirty_rows[:] = False
        self._dirty_cols[:] = False
        for y0, y1, x0, x1 in regions:
            self._field(y0, y1, x0, x1)
            self._write(y0, y1, x0, x1)
        self.cm.version += 1  # bid caches (robot.PathOracle) key off it
        self.refreshes += 1
        return True

    # ---- building -----------------------------------------------------------

    def _field(self, y0, y1, x0, x1) -> None:
        sl = (slice(y0, y1), slice(x0, x1))
        width, length, off, along_x = corridor_geometry(self.rx[sl], self.ry[sl], self.px[sl],
                                                        self.py[sl], self.g.res)
        cw = np.clip((self.max_w + WIDTH_TAPER_M - width) / WIDTH_TAPER_M, 0.0, 1.0)
        cw[width <= self.min_w] = 0.0  # single-lane: nobody passes there anyway (passages)
        with np.errstate(divide="ignore", invalid="ignore"):
            ce = np.clip((length / np.maximum(width, 1e-6) - 1.0) / 2.0, 0.0, 1.0)
        c = np.where(~self.occ[sl], cw * ce, 0.0)
        self.c[sl] = c
        self.ox[sl] = ox = np.where(along_x, 0.0, off)
        self.oy[sl] = oy = np.where(along_x, off, 0.0)
        self.ax[sl] = along_x
        # weight x corridor x clamp((sigma + lane_offset) / (2 lane_offset)),
        # sigma the offset LEFT of the move: +oy going west... see the module
        # docstring. A move across the corridor (east in a north-south one)
        # has along = 0 and costs nothing extra.
        k = 1000.0 * self.weight * c
        f = lambda v: np.clip((v + self.offset) / (2.0 * self.offset), 0.0, 1.0)  # noqa: E731
        for base, (sel, v) in zip(self._base, ((along_x, oy), (along_x, -oy),
                                               (~along_x, -ox), (~along_x, ox))):
            base[sl] = np.where(sel, k * f(v), 0.0)

    def _write(self, y0, y1, x0, x1) -> None:
        sl = (slice(y0, y1), slice(x0, x1))
        wi = self._wint[sl]
        if not self.c[sl].any():  # no corridor here: every direction is wint
            for a in self._wdir:
                a[sl] = wi
            return
        live = wi > 0
        e, w_, n, s_ = (b[sl] for b in self._base)
        d = math.sqrt(0.5)
        for a, pen in zip(self._wdir, (e, w_, n, s_, d * (e + n), d * (e + s_), d * (w_ + n),
                                       d * (w_ + s_))):
            a[sl] = np.where(live, wi + np.rint(pen), 0)

    # ---- the smoother's check -------------------------------------------------

    def sigma(self, j, mx: float, my: float) -> float:
        """Cell j's offset left of its corridor's centre line for a move
        (mx, my), in metres; -inf where it is no corridor or the move runs
        more across it than along it."""
        if j is None or self.cf[j] < 0.5:
            return -math.inf
        along = abs(mx) if self.axf[j] else abs(my)
        if along < 0.38:  # steeper than ~68 degrees to the corridor
            return -math.inf
        return (self.oxf[j] * -my + self.oyf[j] * mx) / along

    def direction(self, a, b):
        d = math.dist(a, b)
        return None if d < 1e-9 else ((b[0] - a[0]) / d, (b[1] - a[1]) / d)

    def follows(self, j, j_ref, m) -> bool:
        """May a smoother shortcut in direction m pass cell j, where the A*
        path it replaces was at cell j_ref (the same fraction along)? Not
        more than half a cell further left than A* was there, or anywhere in
        the zero-cost right lane. Bounding it by the shortcut's ends was not
        enough (2026-11-26): a shelf sits on the aisle's centre line, so a
        leg from shelf to shelf pulled taut ran straight down the middle and
        the lanes A* chose were gone. With the half cell also allowed past
        the lane's edge, lanes drifted 0.2 m toward the centre."""
        s = self.sigma(j, m[0], m[1])
        if s == -math.inf:
            return True
        return s <= max(self.sigma(j_ref, m[0], m[1]) + 0.5 * self.g.res, -self.offset) + 1e-6


def attach_lanes(cm, cfg: dict, single_lane_m: float) -> Lanes:
    """Give a costmap the keep-right term (built now, from its belief)."""
    ln = Lanes(cm, cfg, single_lane_m)
    cm.lanes = ln
    cm.use_step_arrays(ln.wdir)
    ln.on_window(0, 0, cm.grid.width, cm.grid.height)
    ln.refresh(0.0, force=True)
    return ln


# ---------------------------------------------------------------------------
# single-lane passages
# ---------------------------------------------------------------------------

class PassageMap:
    """Single-lane passages on one robot's own map: connected (8-way)
    regions of cells seen free whose free width is at most single_lane_m,
    with at least one cell the robot can stand on. `labels` holds 1 + the passage's
    index; `boxes[i]` its extent in metres, which is how robots name a
    passage to each other: two robots' maps may cut one passage slightly
    differently, and overlapping boxes still match."""

    def __init__(self, grid, state: np.ndarray, single_lane_m: float, r_lethal: float):
        self.g = grid
        self.state = state
        self.single = float(single_lane_m)
        self.r_lethal = float(r_lethal)
        self.labels = np.zeros((grid.height, grid.width), np.int32)
        self.flat = self.labels.reshape(-1)
        self.boxes: list[tuple] = []
        self.occ = None
        self.version = 0

    def refresh(self) -> bool:
        occ = self.state == OCC
        if self.occ is not None and np.array_equal(occ, self.occ):
            return False
        self.occ = occ
        free = ~occ
        width = np.minimum(run_lengths(free), run_lengths(free.T.copy()).T) * self.g.res
        # Seen free, not just not seen: unknown counts as free for the width
        # (as for the planner), but a blind robot that had seen both faces of
        # a 1.2 m rack read its unknown inside as a 0.8 m single lane, and
        # claimed and held for slivers like that all over fleet_scale
        # (2026-11-26).
        single = (self.state == FREE) & (width <= self.single + 1e-6)
        drivable = ~inflate_bool(occ, self.g, self.r_lethal) if single.any() else None
        self._label(single, drivable)
        self.version += 1
        return True

    def _label(self, mask: np.ndarray, drivable) -> None:
        """Components of mask by runs: label each row's runs, join runs of
        adjacent rows that touch (8-way), union-find over runs."""
        h, _w = mask.shape
        rr, rc, rl = row_runs(mask)
        runs = list(zip(rr.tolist(), rc.tolist(), (rc + rl - 1).tolist()))  # (row, x0, x1)
        by_row: list[list[int]] = [[] for _ in range(h)]
        for i, (y, _a, _b) in enumerate(runs):
            by_row[y].append(i)
        parent = list(range(len(runs)))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        for y in range(1, h):
            prev, cur = by_row[y - 1], by_row[y]
            if not prev or not cur:
                continue
            i = j = 0
            while i < len(prev) and j < len(cur):
                _, a0, b0 = runs[prev[i]]
                _, a1, b1 = runs[cur[j]]
                if a1 <= b0 + 1 and a0 <= b1 + 1:
                    ra, rb = find(prev[i]), find(cur[j])
                    if ra != rb:
                        parent[max(ra, rb)] = min(ra, rb)
                if b0 < b1:
                    i += 1
                else:
                    j += 1
        comps: dict[int, list] = {}
        for i, run in enumerate(runs):
            comps.setdefault(find(i), []).append(run)
        self.labels[:] = 0
        self.boxes = []
        g = self.g
        for root in sorted(comps):
            rs = comps[root]
            if not any(drivable[y, a:b + 1].any() for y, a, b in rs):
                continue  # a notch nobody can stand in is no passage
            k = len(self.boxes) + 1
            for y, a, b in rs:
                self.labels[y, a:b + 1] = k
            ys = [r[0] for r in rs]
            xa, xb = min(r[1] for r in rs), max(r[2] for r in rs)
            self.boxes.append((round(g.ox + xa * g.res, 2), round(g.oy + min(ys) * g.res, 2),
                               round(g.ox + (xb + 1) * g.res, 2),
                               round(g.oy + (max(ys) + 1) * g.res, 2)))

    def label_at(self, x: float, y: float) -> int:
        i = self.g.index(x, y)
        return 0 if i is None else int(self.flat[i])


def boxes_overlap(a, b, tol: float = 0.0) -> bool:
    return a[0] < b[2] + tol and b[0] < a[2] + tol and a[1] < b[3] + tol and b[1] < a[3] + tol


def claim_key(c: dict, k: int) -> tuple:
    """Who holds a passage: a robot inside it first (it can't wait outside),
    then the earliest claim, then priority (task class at claim time), then
    robot index. Every robot compares the same numbers, so every robot
    reaches the same order; clock skew only changes who wins."""
    return (0 if c["inside"] else 1, c["t0"], c["prio"], k)


def path_crossings(path: list, labels_flat, grid, step: float) -> tuple[list, list]:
    """Where a polyline runs through passages, sampled every `step` metres:
    ([(label, s_in, s_out | None)], cumulative vertex lengths). s is the
    distance along the path; s_out None: the path ends inside."""
    cum = [0.0]
    for a, b in zip(path[:-1], path[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    out: list = []
    if not path:
        return out, cum
    total = cum[-1]
    cur, s_in, seg = 0, 0.0, 0
    for t in range(int(math.ceil(total / step)) + 1):
        s = min(total, t * step)
        while seg < len(path) - 2 and cum[seg + 1] < s:
            seg += 1
        a = path[seg]
        b = path[seg + 1] if seg + 1 < len(path) else a
        ln = cum[seg + 1] - cum[seg] if seg + 1 < len(cum) else 0.0
        f = (s - cum[seg]) / ln if ln > 1e-12 else 0.0
        j = grid.index(a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
        lab = 0 if j is None else int(labels_flat[j])
        if lab != cur:
            if cur:
                out.append((cur, s_in, s))
            s_in, cur = s, lab
    if cur:
        out.append((cur, s_in, None))
    return out, cum


class PassageAgent:
    """One robot's side of single-lane passage reservation (B11).

      * Its passage map comes from its own belief (PassageMap), re-cut at
        most every refresh_s when walls change.
      * Claim: when the path enters a passage within claim_m, or the robot
        stands in one, it claims it: (box, claim time, priority, inside).
        Claims ride on its /fleet/status, every one, while they last; a
        Status without the claim releases it, and a peer's claims lapse
        ttl_s after its last Status (a dead holder frees its passage).
      * Hold: the robot may enter only if its claim comes first among the
        live claims on that passage (claim_key) and is settle_s old (time
        to hear an earlier one). Otherwise it stops standoff_m before the
        entry, never inside a passage, until it comes first.
      * The claim covers in and out: it stands while the robot is inside,
        whatever its path or task does (a dead-end bay's leg ends inside),
        and goes when it is out and its path no longer enters within
        claim_m (a new path that avoids the passage releases it too).
      * Bounded: waiting more than hold_max_s on one holder stops deferring
        to that holder for as long (a robot parked inside, a prediction gone
        wrong); the local layer takes over, as without reservations.
    """

    def __init__(self, robot, cfg: dict, single_lane_m: float):
        self.r = robot
        c = self.cfg = cfg
        self.claim_m = float(c["claim_m"])
        self.standoff = float(c["standoff_m"])
        self.settle_s = float(c["settle_s"])
        self.ttl_s = float(c["ttl_s"])
        self.hold_max_s = float(c["hold_max_s"])
        self.refresh_s = float(c["refresh_s"])
        self.map = PassageMap(robot.sim.grid, robot.belief.state, single_lane_m,
                              robot.cost.r_lethal)
        self.map.refresh()
        self.next_refresh = 0.0
        self.claims: dict[tuple, dict] = {}   # box -> {"t0", "prio", "inside"}
        self.peers: dict[int, dict] = {}      # k -> {"sent", "at", "claims": [...]}
        self.cool: dict[tuple, float] = {}    # (peer k, box) -> deferring to it again from
        self.inside = None                    # box of the passage we stand in
        self._cross = None                    # crossings of the current path
        self._cross_ver = None                # the map version they were cut against

    def reset(self) -> None:
        """A reboot: no claims, no memory of peers'."""
        self.claims.clear()
        self.peers.clear()
        self.cool.clear()
        self.inside = None
        self._cross_ver = None

    def path_changed(self) -> None:
        self._cross_ver = None

    # ---- wire --------------------------------------------------------------

    def for_status(self) -> list:
        return [[list(b), round(c["t0"], 3), c["prio"], int(c["inside"])]
                for b, c in self.claims.items()]

    def on_claims(self, k: int, sent: float, claims: list, now: float) -> None:
        p = self.peers.get(k)
        if p is not None and sent < p["sent"]:
            return  # an older Status overtaken by a newer one
        self.peers[k] = {"sent": sent, "at": now,
                         "claims": [{"box": tuple(b), "t0": t0, "prio": pr, "inside": bool(ins)}
                                    for b, t0, pr, ins in claims]}

    # ---- decisions -----------------------------------------------------------

    def first_claim(self, box: tuple, now: float):
        """(k, claim) of the claim that holds the passage `box` now, ours
        included (k = our index), or None if nobody claims it."""
        r = self.r
        best = None
        own = self.claims.get(box)
        if own is not None:
            best = (claim_key(own, r.k), r.k, own)
        for k, p in self.peers.items():
            if now - p["at"] > self.ttl_s:
                continue
            for c in p["claims"]:
                if not boxes_overlap(c["box"], box) or now < self.cool.get((k, box), -math.inf):
                    continue
                key = claim_key(c, k)
                if best is None or key < best[0]:
                    best = (key, k, c)
        return None if best is None else (best[1], best[2])

    def _crossings(self):
        r = self.r
        if self._cross_ver != self.map.version:
            self._cross_ver = self.map.version
            cr, cum = path_crossings(r.path, self.map.flat, r.sim.grid, 0.5 * r.sim.grid.res)
            lst = [{"box": self.map.boxes[lab - 1], "s_in": s_in, "s_out": s_out}
                   for lab, s_in, s_out in cr]
            # Hold standoff_m before the entry, never inside a passage and
            # never in the gap just past one: a hold point that lands there
            # is the earlier passage's own, so the robot waits before both
            # until both are its own. Holding in the 2.4 m crossing between
            # two narrow segments stood a robot in the mouth of each, and the
            # holder coming out could not get past it (2026-11-26).
            for i, c in enumerate(lst):
                h = max(0.0, c["s_in"] - self.standoff)
                for q in reversed(lst[:i]):
                    if q["s_out"] is not None and q["s_in"] - 1e-9 <= h <= q["s_out"] + r.body_r:
                        h = q["s_hold"]
                        break
                c["s_hold"] = h
            self._cross = (lst, cum)
        return self._cross

    def _progress(self, cum) -> float:
        r = self.r
        if not r.path:
            return 0.0
        seg = min(r.seg, len(r.path) - 1)
        return cum[seg] + math.dist(r.path[seg], (r.x, r.y))

    def tick(self, now: float) -> None:
        r = self.r
        if now >= self.next_refresh:
            self.next_refresh = now + self.refresh_s
            if self.map.refresh():
                self._remap()
        lab = self.map.label_at(r.x, r.y)
        here = self.map.boxes[lab - 1] if lab else None
        if here != self.inside:
            if self.inside is not None:
                r.sim.record({"s": "nav", "rid": r.rid, "event": "pass_exit", "box": list(self.inside)})
            if here is not None:
                r.sim.record({"s": "nav", "rid": r.rid, "event": "pass_enter", "box": list(here)})
            self.inside = here
        if r.goal is not None and not r.path:
            # A goal and no path (a failed plan, retrying): keep what we
            # claimed. Releasing here and claiming again on the next plan put
            # the robot at the back of the queue every retry (2026-11-26).
            for b, c in self.claims.items():
                c["inside"] = b == here
            if here is not None and here not in self.claims:
                self.claims[here] = {"t0": now, "prio": r.prio()[0], "inside": True}
                r.sim.record({"s": "nav", "rid": r.rid, "event": "pass_claim", "box": list(here),
                              "inside": True})
            self._hold(now, here, [])
            return
        want: dict[tuple, bool] = {}
        if here is not None:
            want[here] = True
        ahead = []
        if r.goal is not None and r.path:
            lst, cum = self._crossings()
            s = self._progress(cum)
            for c in lst:
                if c["s_in"] > s + 1e-6 and c["box"] != here:
                    # Within claim_m of the entry, or of where we would hold
                    # for it (a passage chained behind another).
                    if min(c["s_in"], c["s_hold"] + self.standoff) - s <= self.claim_m:
                        want.setdefault(c["box"], False)
                    ahead.append((c, s))
        for b in [b for b in self.claims if b not in want]:
            del self.claims[b]
            r.sim.record({"s": "nav", "rid": r.rid, "event": "pass_release", "box": list(b)})
        cls = r.prio()[0]
        for b, inside in want.items():
            if b not in self.claims:
                self.claims[b] = {"t0": now, "prio": cls, "inside": inside}
                r.sim.record({"s": "nav", "rid": r.rid, "event": "pass_claim", "box": list(b),
                              "inside": inside})
            self.claims[b]["inside"] = inside
        self._hold(now, here, ahead)

    def _hold(self, now: float, here, ahead) -> None:
        r = self.r
        blocker = None
        if here is None:  # never wait inside a passage: that is how waits deadlock
            # Hold points don't decrease along the path, so the first one not
            # reached yet ends the check; a chained passage shares its hold
            # point with the one before and is checked with it.
            for c, s in ahead:
                if s < c["s_hold"] - 0.05 or c["box"] not in self.claims:
                    break
                first = self.first_claim(c["box"], now)
                if first is not None and first[0] != r.k:
                    blocker = (first[0], c["box"])
                    break
                if now - self.claims[c["box"]]["t0"] < self.settle_s:
                    blocker = (r.k, c["box"])  # our claim is new: let earlier ones arrive
                    break
        h = r.pass_hold
        if h is not None and (blocker is None or (h["peer"], h["box"]) != blocker):
            r.sim.record({"s": "nav", "rid": r.rid, "event": "pass_hold_end",
                          "after_s": round(now - h["since"], 2)})
            r.pass_hold = None
        if blocker is not None and r.pass_hold is None:
            if r.blocked_since is not None:
                # A held robot isn't checked for blockers, so a stop left open
                # would book the whole hold as a stop for a robot (nav
                # blocked_robot_s) when it ends. Close it here.
                r.sim.record({"s": "nav", "rid": r.rid, "event": "unblocked",
                              "after_s": round(now - r.blocked_since, 2)})
                r.blocked_since = r.blocked_by = r.block_replan_at = None
            r.pass_hold = {"peer": blocker[0], "box": blocker[1], "since": now}
            r.counters["pass_holds"] += 1
            r.sim.record({"s": "nav", "rid": r.rid, "event": "pass_hold",
                          "to": f"robot_{blocker[0]}", "box": list(blocker[1])})
        h = r.pass_hold
        if h is not None and h["peer"] != r.k and now - h["since"] >= self.hold_max_s:
            # Waited too long for one holder (it is parked inside, or stuck
            # in there): stop deferring to it for as long again.
            self.cool[(h["peer"], h["box"])] = now + self.hold_max_s
            r.sim.record({"s": "nav", "rid": r.rid, "event": "pass_hold_timeout",
                          "to": f"robot_{h['peer']}"})
            r.sim.record({"s": "nav", "rid": r.rid, "event": "pass_hold_end",
                          "after_s": round(now - h["since"], 2)})
            r.pass_hold = None

    def _remap(self) -> None:
        """The map was re-cut: carry claims over to the passage that now
        covers them, so a claim keeps its place in the queue."""
        new = {}
        for b, c in self.claims.items():
            for nb in self.map.boxes:
                if boxes_overlap(b, nb):
                    new.setdefault(nb, c)
                    break
        self.claims = new
        if self.inside is not None:  # the same passage, re-cut: no exit and entry to record
            self.inside = next((nb for nb in self.map.boxes if boxes_overlap(self.inside, nb)),
                               self.inside)
        self._cross_ver = None
