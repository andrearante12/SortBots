"""The floor as a grid of square cells: geometry, saved-map loading, and a
synthetic warehouse generator.

Every map in fleetsim is the same `Grid` at the run's `world.cell_m`: the true
world, each robot's belief, its costmap. Row 0 is at the origin's y (the
OccupancyGrid convention), so `occ[cy, cx]` is the cell whose lower-left
corner is (ox + cx * res, oy + cy * res).

Cell size is the main fidelity/cost knob (docs/fleetsim.md, "Resolution").
A saved map (maps/<name>/grid.pgm, 0.05 m) is downsampled by max-pooling: a
coarse cell is occupied if ANY fine cell in it is, the same rule
nodes/grid_dist.py uses. That keeps thin walls from vanishing, at the price of
thickening every obstacle by up to one coarse cell, which can close a passage
narrower than about footprint + 2 cells. `downsample()` reports how much free
floor the pooling cost so a run can say when that happened.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
MAPS_DIR = REPO_ROOT / "maps"


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    res: float
    ox: float
    oy: float

    @property
    def size(self) -> int:
        return self.width * self.height

    def cell(self, x: float, y: float) -> tuple[int, int] | None:
        cx = int(math.floor((x - self.ox) / self.res))
        cy = int(math.floor((y - self.oy) / self.res))
        if 0 <= cx < self.width and 0 <= cy < self.height:
            return cx, cy
        return None

    def index(self, x: float, y: float) -> int | None:
        c = self.cell(x, y)
        return None if c is None else c[1] * self.width + c[0]

    def centre(self, cx: int, cy: int) -> tuple[float, float]:
        return (self.ox + (cx + 0.5) * self.res, self.oy + (cy + 0.5) * self.res)

    def centre_of(self, idx: int) -> tuple[float, float]:
        return self.centre(idx % self.width, idx // self.width)

    def disc_offsets(self, radius_m: float) -> list[tuple[int, int, float]]:
        """(dx, dy, metres) for every cell whose centre is within radius_m of a
        cell centre — the stencil inflation and body rasters are built from."""
        r = int(math.ceil(radius_m / self.res))
        out = []
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                d = math.hypot(dx, dy) * self.res
                if d <= radius_m + 1e-9:
                    out.append((dx, dy, d))
        return out


# ---------------------------------------------------------------------------
# saved maps (maps/<name>/grid.pgm + grid.yaml, map_saver trinary output)
# ---------------------------------------------------------------------------

def _read_pgm(path: Path) -> np.ndarray:
    """Binary (P5) or ASCII (P2) PGM -> uint8 array, top row first."""
    raw = path.read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b"#":
            pos = raw.index(b"\n", pos) + 1
            continue
        start = pos
        while not raw[pos:pos + 1].isspace():
            pos += 1
        tokens.append(raw[start:pos])
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError(f"{path}: 16-bit PGM not supported")
    if magic == b"P5":
        data = np.frombuffer(raw, dtype=np.uint8, count=w * h, offset=pos + 1)
    elif magic == b"P2":
        data = np.array(raw[pos:].split()[: w * h], dtype=np.uint8)
    else:
        raise ValueError(f"{path}: not a PGM (magic {magic!r})")
    return data.reshape(h, w)


def resolve_map_dir(name_or_path: str) -> Path:
    p = Path(name_or_path)
    if p.is_dir():
        return p
    d = MAPS_DIR / name_or_path
    if not d.is_dir():
        raise FileNotFoundError(f"no saved map {name_or_path!r} (looked for {d})")
    return d


def load_saved_map(name_or_path: str) -> tuple[np.ndarray, Grid, dict]:
    """A library map -> (occupied bool array at its own resolution, Grid,
    waypoints {name: (x, y)}). UNKNOWN cells count as occupied: in a saved
    map they are space nobody has seen — outside the walls, or inside racks —
    and the true world must not let a robot drive there."""
    d = resolve_map_dir(name_or_path)
    meta = yaml.safe_load((d / "grid.yaml").read_text())
    img = _read_pgm(d / meta.get("image", "grid.pgm"))
    p = img.astype(np.float64) / 255.0
    if not meta.get("negate", 0):
        p = 1.0 - p
    free = p < float(meta.get("free_thresh", 0.196))
    occ = ~free  # occupied and unknown alike
    occ = occ[::-1, :].copy()  # PGM rows are top-first; the grid is origin-first
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    grid = Grid(occ.shape[1], occ.shape[0], float(meta["resolution"]), ox, oy)
    wps = {}
    wf = d / "waypoints.json"
    if wf.is_file():
        import json

        try:
            for w in json.loads(wf.read_text()).get("waypoints", []):
                wps[str(w["name"])] = (float(w["x"]), float(w["y"]))
        except (ValueError, KeyError, TypeError, AttributeError):
            wps = {}
    return occ, grid, wps


def downsample(occ: np.ndarray, grid: Grid, cell_m: float) -> tuple[np.ndarray, Grid, float]:
    """Max-pool to `cell_m` (rounded to a whole number of source cells).
    Returns (occ, grid, free_lost_frac): the share of the source's free floor
    that pooling turned into obstacle."""
    k = max(1, int(round(cell_m / grid.res)))
    if k == 1:
        return occ.copy(), grid, 0.0
    h, w = occ.shape
    H, W = (h + k - 1) // k, (w + k - 1) // k
    pad = np.ones((H * k, W * k), dtype=bool)  # outside the source map is wall
    pad[:h, :w] = occ
    out = pad.reshape(H, k, W, k).any(axis=(1, 3))
    src_free = float((~occ).sum()) * grid.res ** 2
    new_free = float((~out).sum()) * (grid.res * k) ** 2
    lost = 0.0 if src_free <= 0 else max(0.0, 1.0 - new_free / src_free)
    return out, Grid(W, H, grid.res * k, grid.ox, grid.oy), lost


# ---------------------------------------------------------------------------
# synthetic warehouse
# ---------------------------------------------------------------------------

SYNTHETIC_DEFAULTS = {
    "width_m": 60.0,
    "height_m": 40.0,
    "wall_m": 0.4,
    "staging_m": 8.0,       # open apron along the south wall: docks + robot homes
    "aisle_m": 2.4,         # two 0.4 m-wide robots pass with room for inflation
    "rack_depth_m": 1.2,
    "end_gap_m": 3.0,       # cross-aisle at each end of every rack row
    "cross_every_m": 15.0,  # a mid-row cross-aisle at least this often
    "cross_m": 2.4,
    "shelf_spacing_m": 5.0, # pickup waypoints along each aisle
    "docks": 12,            # dropoff waypoints along the apron
    # Traffic structure (C12 benchmark, 2026-11-26). All off by default.
    # Single-lane aisles: these aisles' free width is cut to narrow_m beside
    # every rack segment (the racks are deeper there); the crossings with the
    # cross-aisles keep their full width. Aisle 0 runs along the open apron
    # and has no rack to its south, so it can't be narrowed.
    "narrow_aisles": [],
    # 1.0 m: a robot is planned as a 0.7 m disc at 0.2 m cells, so 3 cell
    # centres across stay drivable, and two 0.5 m bodies can't be side by
    # side (passing needs ~1.3 m: two lethal bands + the local layer's 0.6 m
    # body-to-body). Rounded to whole cells.
    "narrow_m": 1.0,
    # [[aisle, segment, west|east], ...]: a wall across that aisle at that end
    # of that rack segment, so the segment is entered from its other end only.
    "dead_ends": [],
    # [[x0, y0, x1, y1], ...]: extra walls anywhere (closing a cross-aisle,
    # walling off a corner). A wall that cuts a waypoint off fails the scenario.
    "walls": [],
}


def synthetic_warehouse(cfg: dict, cell_m: float) -> tuple[np.ndarray, Grid, dict, list, tuple]:
    """Parallel rack rows (east-west), aisles between them, cross-aisles at
    the ends and every `cross_every_m`, a staging apron along the south wall.

    Returns (occ, grid, waypoints, homes, gateway_xy). Waypoints are
    `shelf_<aisle>_<k>` (pickups, in the aisles beside the racks) and
    `dock_<k>` (dropoffs, on the apron). Homes are a lattice of free apron
    spots, as many as fit, where idle robots park.

    Aisle `a` is the corridor at y = aisle_ys[a]; aisle 0 borders the apron.
    Rack segment `s` is the s-th run of rack from the west, between the
    cross-aisles; it is the same x span in every row. On the default floor
    there are aisles 0-8 (y 9.6 + 3.6 a) and segments 0-3 (x 3.4-14.9,
    17.3-28.8, 31.2-42.7, 45.1-56.6).
    """
    unknown = set(cfg or {}) - set(SYNTHETIC_DEFAULTS)
    if unknown:
        # The block is free-form in base.yaml, so a typo'd option would
        # otherwise build the plain floor without a word.
        raise ValueError(f"world.synthetic: unknown key(s) {sorted(unknown)} "
                         f"(have: {sorted(SYNTHETIC_DEFAULTS)})")
    c = {**SYNTHETIC_DEFAULTS, **(cfg or {})}
    W, H = float(c["width_m"]), float(c["height_m"])
    grid = Grid(int(math.ceil(W / cell_m)), int(math.ceil(H / cell_m)), cell_m, 0.0, 0.0)
    occ = np.zeros((grid.height, grid.width), dtype=bool)

    def fill(x0, y0, x1, y1):
        cx0, cy0 = max(0, int(math.floor(x0 / cell_m))), max(0, int(math.floor(y0 / cell_m)))
        cx1 = min(grid.width, int(math.ceil(x1 / cell_m)))
        cy1 = min(grid.height, int(math.ceil(y1 / cell_m)))
        occ[cy0:cy1, cx0:cx1] = True

    wall = float(c["wall_m"])
    fill(0, 0, W, wall)
    fill(0, H - wall, W, H)
    fill(0, 0, wall, H)
    fill(W - wall, 0, W, H)

    aisle, depth = float(c["aisle_m"]), float(c["rack_depth_m"])
    end_gap, cross = float(c["end_gap_m"]), float(c["cross_m"])
    x_lo, x_hi = wall + end_gap, W - wall - end_gap
    # Rack segments along each row, split by mid-row cross-aisles.
    span = x_hi - x_lo
    n_seg = max(1, int(round(span / max(float(c["cross_every_m"]), 1.0))))
    seg_len = (span - (n_seg - 1) * cross) / n_seg
    segments = [(x_lo + i * (seg_len + cross), x_lo + i * (seg_len + cross) + seg_len)
                for i in range(n_seg)]

    waypoints: dict[str, tuple[float, float]] = {}
    y = wall + float(c["staging_m"]) + aisle  # first aisle runs along the apron's edge
    row, aisle_ys = 0, [wall + float(c["staging_m"]) + aisle / 2]
    while y + depth + aisle <= H - wall + 1e-6:
        for sx0, sx1 in segments:
            fill(sx0, y, sx1, y + depth)
        y += depth
        aisle_ys.append(y + aisle / 2)
        y += aisle
        row += 1
    _traffic_structure(c, occ, grid, aisle_ys, segments, fill)
    spacing = float(c["shelf_spacing_m"])
    for ai, ay in enumerate(aisle_ys):
        k = 0
        for sx0, sx1 in segments:
            x = sx0 + spacing / 2
            while x < sx1 - 0.5:
                waypoints[f"shelf_{ai}_{k}"] = (round(x, 3), round(ay, 3))
                x += spacing
                k += 1

    staging_top = wall + float(c["staging_m"])
    n_docks = int(c["docks"])
    dock_y = wall + 1.2
    for k in range(n_docks):
        x = x_lo + (k + 0.5) * (x_hi - x_lo) / max(n_docks, 1)
        waypoints[f"dock_{k}"] = (round(x, 3), round(dock_y, 3))

    # Homes: a lattice on the apron between the docks and the first aisle.
    # 1.6 m apart: two parked 0.5 m discs leave 1.1 m, enough for a robot
    # (planned as a 0.7 m disc at 0.2 m cells) to pass between them. At
    # 1.2 m a parked fleet walled the docks off from the racks.
    homes = []
    hy = dock_y + 2.4
    while hy < staging_top - 0.6:
        hx = x_lo
        while hx < x_hi:
            homes.append((round(hx, 3), round(hy, 3)))
            hx += 1.6
        hy += 1.6
    gateway = (W / 2, staging_top)
    return occ, grid, waypoints, homes, gateway


def _aisle_index(a, aisle_ys: list, what: str) -> int:
    if isinstance(a, bool) or not isinstance(a, int) or not 1 <= a < len(aisle_ys):
        raise ValueError(f"world.synthetic.{what}: aisle {a!r} must be 1..{len(aisle_ys) - 1} "
                         "(aisle 0 borders the open apron)")
    return a


def _traffic_structure(c: dict, occ: np.ndarray, grid: Grid, aisle_ys: list, segments: list,
                       fill) -> None:
    """narrow_aisles, dead_ends and walls, on top of the plain floor."""
    cell, aisle, wall = grid.res, float(c["aisle_m"]), float(c["wall_m"])
    narrow = c["narrow_aisles"] or []
    if not isinstance(narrow, list):
        raise ValueError("world.synthetic.narrow_aisles: a list of aisle numbers")
    if narrow:
        k = int(round(float(c["narrow_m"]) / cell))
        if not 1 <= k < int(round(aisle / cell)):
            raise ValueError(f"world.synthetic.narrow_m: {c['narrow_m']} m must be at least a "
                             f"cell and less than aisle_m ({aisle} m)")
        for a in narrow:
            ay = aisle_ys[_aisle_index(a, aisle_ys, "narrow_aisles")]
            # Whole cells, centred on the aisle: placed in metres, a 1.0 m
            # opening on a 0.2 m grid came out 0.8 m, since fill() takes every
            # cell it touches. The epsilon stops 13.2 / 0.2 = 65.999... from
            # shifting the opening a cell south.
            lo = int(math.floor(ay / cell - k / 2.0 + 0.5 + 1e-6))
            y_lo, y_hi = lo * cell, (lo + k) * cell
            for sx0, sx1 in segments:
                fill(sx0, ay - aisle / 2 - cell, sx1, y_lo - 1e-6)
                fill(sx0, y_hi + 1e-6, sx1, ay + aisle / 2 + cell)
    for i, de in enumerate(c["dead_ends"] or []):
        if not (isinstance(de, (list, tuple)) and len(de) == 3 and de[2] in ("west", "east")):
            raise ValueError(f"world.synthetic.dead_ends[{i}]: expected [aisle, segment, west|east]")
        ay = aisle_ys[_aisle_index(de[0], aisle_ys, f"dead_ends[{i}]")]
        s = de[1]
        if isinstance(s, bool) or not isinstance(s, int) or not 0 <= s < len(segments):
            raise ValueError(f"world.synthetic.dead_ends[{i}]: segment {s!r} must be "
                             f"0..{len(segments) - 1}")
        sx0, sx1 = segments[s]
        x0 = sx0 if de[2] == "west" else sx1 - wall
        # Rack face to rack face, a cell into each, so no diagonal gap is left.
        fill(x0, ay - aisle / 2 - cell, x0 + wall, ay + aisle / 2 + cell)
    for i, r in enumerate(c["walls"] or []):
        if not (isinstance(r, (list, tuple)) and len(r) == 4
                and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in r)):
            raise ValueError(f"world.synthetic.walls[{i}]: expected [x0, y0, x1, y1]")
        x0, y0, x1, y1 = (float(v) for v in r)
        fill(min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def reach_mask(free: np.ndarray, seed: tuple[int, int]) -> np.ndarray:
    """Cells 4-connected to `seed` (cx, cy) through `free`. A* never steps
    diagonally past a lethal cell, so on a lethal-free mask this is exactly
    what it can reach. Spreads along whole row and column runs per pass, so
    a warehouse converges in about as many passes as a path has turns."""
    h, w = free.shape
    reached = np.zeros_like(free)
    if not free[seed[1], seed[0]]:
        return reached
    reached[seed[1], seed[0]] = True

    def spread(f: np.ndarray, r: np.ndarray) -> np.ndarray:
        # Runs of free cells along each row: a run is reached if any cell is.
        start = f & ~np.concatenate([np.zeros((f.shape[0], 1), bool), f[:, :-1]], axis=1)
        run = np.cumsum(start.ravel()).reshape(f.shape) * f
        hit = np.zeros(int(run.max()) + 1, dtype=bool)
        hit[run[r]] = True
        hit[0] = False
        return hit[run]

    while True:
        nxt = spread(free, reached)
        nxt = spread(free.T, nxt.T).T
        if (nxt == reached).all():
            return reached
        reached = nxt


def unreachable(occ: np.ndarray, grid: Grid, points: dict, start_xy, *, clearance_m: float,
                tol_m: float) -> list:
    """Keys of `points` ({key: (x, y)}) a robot planned as a disc of
    clearance_m can't drive to from start_xy: no drivable cell connected to
    the start within tol_m (nav.goal_tolerance_m, how far a goal snaps)."""
    free = ~inflate_bool(occ, grid, clearance_m)
    c = grid.cell(*start_xy)
    if c is None:
        return list(points)
    reached = reach_mask(free, c)
    near = grid.disc_offsets(tol_m)
    out = []
    for key, (x, y) in points.items():
        p = grid.cell(x, y)
        if p is None or not any(0 <= p[0] + dx < grid.width and 0 <= p[1] + dy < grid.height
                                and reached[p[1] + dy, p[0] + dx] for dx, dy, _d in near):
            out.append(key)
    return out


def synthetic_checked(cfg: dict, cell_m: float, *, clearance_m: float, tol_m: float):
    """synthetic_warehouse, then every waypoint must be drivable from the
    apron (ValueError naming the ones that aren't), and homes that a wall
    took are dropped. The options above can close passages; a run that only
    found out when a task to a sealed shelf failed 45 s later would read as a
    nav result, not a broken floor."""
    occ, grid, wps, homes, gw = synthetic_warehouse(cfg, cell_m)
    c = {**SYNTHETIC_DEFAULTS, **(cfg or {})}
    apron = (float(c["width_m"]) / 2, float(c["wall_m"]) + float(c["staging_m"]) / 2)
    if unreachable(occ, grid, {"apron": apron}, apron, clearance_m=clearance_m, tol_m=0.0):
        raise ValueError(f"world.synthetic: the apron centre {apron} is walled in; reachability "
                         "is checked from there (walls)")
    bad = unreachable(occ, grid, wps, apron, clearance_m=clearance_m, tol_m=tol_m)
    if bad:
        raise ValueError(f"world.synthetic: {len(bad)} waypoint(s) can't be reached from the "
                         f"apron at {cell_m} m cells: {', '.join(sorted(bad)[:8])}"
                         f"{' ...' if len(bad) > 8 else ''} (narrow_m, dead_ends or walls)")
    lost = set(unreachable(occ, grid, dict(enumerate(homes)), apron, clearance_m=clearance_m,
                           tol_m=0.0))
    return occ, grid, wps, [h for i, h in enumerate(homes) if i not in lost], gw


def free_spots(occ: np.ndarray, grid: Grid, n: int, rng, *, clearance_m: float,
               spacing_m: float, avoid: list | None = None) -> list[tuple[float, float]]:
    """Up to n random free cell centres at least clearance_m from any
    obstacle and spacing_m from each other (and from `avoid`). For spawns and
    task spots on a saved map that has no homes or waypoints of its own."""
    clear = ~inflate_bool(occ, grid, clearance_m)
    cand = np.flatnonzero(clear.ravel())
    rng.shuffle(cand)
    out: list[tuple[float, float]] = []
    taken = list(avoid or [])
    for idx in cand:
        p = grid.centre_of(int(idx))
        if all(math.dist(p, q) >= spacing_m for q in taken):
            out.append(p)
            taken.append(p)
            if len(out) >= n:
                break
    return out


def inflate_bool(occ: np.ndarray, grid: Grid, radius_m: float) -> np.ndarray:
    """Every cell whose centre is within radius_m of an occupied cell centre."""
    out = occ.copy()
    h, w = occ.shape
    for dx, dy, _d in grid.disc_offsets(radius_m):
        if dx == 0 and dy == 0:
            continue
        ys, yd = (slice(0, h - dy), slice(dy, h)) if dy >= 0 else (slice(-dy, h), slice(0, h + dy))
        xs, xd = (slice(0, w - dx), slice(dx, w)) if dx >= 0 else (slice(-dx, w), slice(0, w + dx))
        out[yd, xd] |= occ[ys, xs]
    return out
