"""The floor as a grid of square cells: geometry, saved-map loading, and a
synthetic warehouse generator.

Every map in fleetsim is the same `Grid` at the run's `world.cell_m`: the true
world, each robot's belief, its costmap. Row 0 is at the origin's y (the
OccupancyGrid convention), so `occ[cy, cx]` is the cell whose lower-left
corner is (ox + cx * res, oy + cy * res).

Cell size is the main fidelity/cost knob (fleetsim/README.md, "Resolution").
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
}


def synthetic_warehouse(cfg: dict, cell_m: float) -> tuple[np.ndarray, Grid, dict, list, tuple]:
    """Parallel rack rows (east-west), aisles between them, cross-aisles at
    the ends and every `cross_every_m`, a staging apron along the south wall.

    Returns (occ, grid, waypoints, homes, gateway_xy). Waypoints are
    `shelf_<aisle>_<k>` (pickups, in the aisles beside the racks) and
    `dock_<k>` (dropoffs, on the apron). Homes are a lattice of free apron
    spots, as many as fit, where idle robots park.
    """
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
