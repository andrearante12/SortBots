"""A robot's own map, and the bookkeeping that shares it.

Each cell holds a state (unknown / free / occupied) and the sim time it was
last OBSERVED, by anyone. Fusion is newest-observation-wins per cell. Not
nodes/map_fuse.py's "occupied beats free": in a map with no central owner,
that rule means a box that was moved stays in every robot's map forever,
because no later "free" can overrule an earlier "occupied".

Sharing follows the commsim design brief (commsim/docs/brief.md, "Obstacles
and maps"):
  * the floor is cut into square tiles; a robot publishes only the cells ITS
    OWN sensing changed (MapUpdate), per tile, with a per-(robot, tile)
    sequence number;
  * a receiver that sees a sequence gap asks the origin for that tile
    (MapResync) and gets a full snapshot back;
  * every robot also re-sends snapshots of the tiles it has observed, round
    robin, so a robot that was out of reach (or joined late) converges
    without having to notice what it missed.
A snapshot carries the sender's fused view of the tile, stamps included, so
merging one is the same newest-wins rule as merging a delta.
"""
from __future__ import annotations

import numpy as np

from .grid import Grid
from .planner import FREE, OCC, UNKNOWN

PRIOR_STAMP = np.float32(-1e9)  # a prior map loses to any real observation


class Belief:
    def __init__(self, grid: Grid, tile_m: float, prior: np.ndarray | None = None):
        self.grid = grid
        h, w = grid.height, grid.width
        self.state = np.full((h, w), UNKNOWN, dtype=np.int8)
        self.stamp = np.full((h, w), -np.inf, dtype=np.float32)
        if prior is not None:
            self.state[:] = np.where(prior, OCC, FREE)
            self.stamp[:] = PRIOR_STAMP
        self.s = self.state.reshape(-1)   # flat views onto the same memory
        self.ts = self.stamp.reshape(-1)
        self.T = max(1, int(round(tile_m / grid.res)))
        self.ntx = (w + self.T - 1) // self.T
        self.nty = (h + self.T - 1) // self.T
        self.dirty = np.zeros(h * w, dtype=bool)
        self.observed_tiles: list[int] = []  # in first-observed order, for refresh
        self._observed_set: set[int] = set()
        self.out_seq: dict[int, int] = {}
        self.in_seq: dict[tuple[int, int], int] = {}

    @property
    def n_tiles(self) -> int:
        return self.ntx * self.nty

    def tile_of(self, idx: np.ndarray) -> np.ndarray:
        w = self.grid.width
        return (idx % w) // self.T + ((idx // w) // self.T) * self.ntx

    def tile_centre(self, tile: int) -> tuple[float, float]:
        tx, ty = tile % self.ntx, tile // self.ntx
        g = self.grid
        return (g.ox + (tx + 0.5) * self.T * g.res, g.oy + (ty + 0.5) * self.T * g.res)

    def tile_cells(self, tile: int) -> np.ndarray:
        g = self.grid
        tx, ty = tile % self.ntx, tile // self.ntx
        xs = np.arange(tx * self.T, min(g.width, (tx + 1) * self.T))
        ys = np.arange(ty * self.T, min(g.height, (ty + 1) * self.T))
        return (ys[:, None] * g.width + xs[None, :]).ravel()

    # ---- own sensing ------------------------------------------------------

    def observe(self, free_idx: np.ndarray, occ_idx: np.ndarray, t: float) -> np.ndarray:
        """Write one scan. A cell seen both free (a ray passing it) and
        occupied (another ray ending on it) in the same scan is occupied.
        Returns the flat indices whose STATE changed."""
        if len(occ_idx):
            free_idx = np.setdiff1d(free_idx, occ_idx, assume_unique=False)
        idx = np.concatenate([free_idx, occ_idx]).astype(np.int64)
        if not len(idx):
            return idx
        vals = np.concatenate([np.full(len(free_idx), FREE, np.int8),
                               np.full(len(occ_idx), OCC, np.int8)])
        changed = idx[self.s[idx] != vals]
        self.s[idx] = vals
        self.ts[idx] = np.float32(t)
        if len(changed):
            self.dirty[changed] = True
        for tile in np.unique(self.tile_of(idx)).tolist():
            if tile not in self._observed_set:
                self._observed_set.add(tile)
                self.observed_tiles.append(tile)
        return changed

    def take_dirty(self) -> dict[int, np.ndarray]:
        """{tile: changed cell indices} since the last call, and clear them."""
        d = np.flatnonzero(self.dirty)
        if not len(d):
            return {}
        self.dirty[d] = False
        tiles = self.tile_of(d)
        order = np.argsort(tiles, kind="stable")
        d, tiles = d[order], tiles[order]
        cuts = np.flatnonzero(np.diff(tiles)) + 1
        return {int(t[0]): c for t, c in zip(np.split(tiles, cuts), np.split(d, cuts))}

    def cells_payload(self, idx: np.ndarray) -> tuple:
        return (idx.copy(), self.s[idx].copy(), self.ts[idx].copy())

    def snapshot(self, tile: int) -> tuple:
        """Every known cell of a tile, with its stamp."""
        idx = self.tile_cells(tile)
        idx = idx[self.s[idx] != UNKNOWN]
        return self.cells_payload(idx)

    def next_seq(self, tile: int) -> int:
        self.out_seq[tile] = self.out_seq.get(tile, 0) + 1
        return self.out_seq[tile]

    # ---- peers' data ------------------------------------------------------

    def merge(self, idx: np.ndarray, states: np.ndarray, stamps: np.ndarray) -> np.ndarray:
        """Newest observation wins. Returns indices whose state changed."""
        if not len(idx):
            return idx
        newer = stamps > self.ts[idx]
        if not newer.any():
            return idx[:0]
        i2, s2 = idx[newer], states[newer]
        changed = i2[self.s[i2] != s2]
        self.s[i2] = s2
        self.ts[i2] = stamps[newer]
        return changed

    def check_seq(self, origin: int, tile: int, seq: int, snapshot: bool) -> bool:
        """Record a received sequence number. True if a delta arrived with a
        gap before it (something from this origin for this tile was lost),
        i.e. a resync is due. A snapshot resets the counter."""
        key = (origin, tile)
        last = self.in_seq.get(key, 0)
        if seq > last:
            self.in_seq[key] = seq
        return (not snapshot) and seq > last + 1
