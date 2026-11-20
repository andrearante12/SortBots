"""Map-conflict cue: depth points that land where this robot has seen free space.

Frame differencing (nodes/dynamic_obstacle_filter.py) only sees things while
they move. A person who walks in and stops, or a cart parked in an aisle the
robot drove down a minute ago, stops being motion and gets painted into
RTAB-Map as a wall. That is the "mapped twice" ghost: the first copy where it
stood, the second where it stopped. This module keeps a rolling 2D count of
how often each cell's body-height band has been seen EMPTY, and flags any new
depth return that lands in a cell seen empty many times and recently.

It is our own grid on purpose, not RTAB-Map's /map: depth_static (which this
cue helps mask) is what builds /map, so reading /map back would feed the mask
into its own evidence.

Flags are prompts, not verdicts. nodes/dynamic_segment.py turns a conflict blob
into a whole-object SAM mask, and its tracks keep a stopped object masked once
this cue has faded (see `hit_decay`).

Pure numpy, no ROS, so tests/map_conflict_test.py runs anywhere.
"""
from __future__ import annotations

import numpy as np

DEFAULTS = {
    "cell_m": 0.10,
    "size_m": 24.0,         # rolling window, re-centred on the camera
    "min_height_m": 0.10,   # same body band as the depth filter's heights
    "max_height_m": 2.2,
    "max_range_m": 4.5,     # D435 depth past ~5 m is too noisy to call free
    # Free evidence a cell needs before a return in it counts as a conflict,
    # and how fresh the last free sighting must be. One count per frame per
    # cell, so 5 is five separate looks, not one ray bundle.
    "min_free": 5,
    "recent_s": 60.0,
    "free_cap": 20,
    # Each frame a cell is hit costs this much free evidence. A stopped object
    # stays a conflict for about (free_cap - min_free) / hit_decay frames
    # (15 s at 2 Hz with these numbers), then becomes ordinary map. A box that
    # was really put down for good should become map; a person who is only
    # pausing is kept masked by dynamic_segment's tracks instead.
    "hit_decay": 0.5,
    # Rays stop this short of their hit, so a wall's own cell is not marked
    # free by the rays that end on it (depth noise is a few cm at 3 m).
    "end_margin_m": 0.20,
    "ray_step_m": 0.05,
}


class FreeSpaceGrid:
    def __init__(self, cfg: dict | None = None):
        self.c = {**DEFAULTS, **(cfg or {})}
        self.n = int(round(self.c["size_m"] / self.c["cell_m"]))
        self.free = np.zeros((self.n, self.n), dtype=np.float32)
        self.last_free = np.full((self.n, self.n), -np.inf, dtype=np.float64)
        self.origin = None  # odom (x, y) of cell [0, 0]'s corner

    # -- geometry ----------------------------------------------------------
    def _cells(self, x: np.ndarray, y: np.ndarray):
        cell = self.c["cell_m"]
        fx = np.floor((x - self.origin[0]) / cell)
        fy = np.floor((y - self.origin[1]) / cell)
        ok = np.isfinite(fx) & np.isfinite(fy)
        ix = np.where(ok, fx, -1).astype(np.int64)
        iy = np.where(ok, fy, -1).astype(np.int64)
        inside = ok & (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        return ix, iy, inside

    def recentre(self, x: float, y: float) -> None:
        """Keep the camera within the middle half of the window. Shifts are
        whole cells, so evidence never gets resampled or smeared."""
        cell, half = self.c["cell_m"], self.n * self.c["cell_m"] / 2
        if self.origin is None:
            self.origin = np.array([x - half, y - half])
            return
        off = np.array([x, y]) - (self.origin + half)
        if np.all(np.abs(off) < half / 2):
            return
        shift = np.round(off / cell).astype(np.int64)
        self.origin = self.origin + shift * cell
        for axis, s in enumerate(shift):
            if s == 0:
                continue
            self.free = np.roll(self.free, -s, axis=axis)
            self.last_free = np.roll(self.last_free, -s, axis=axis)
            idx = [slice(None), slice(None)]
            idx[axis] = slice(self.n - s, None) if s > 0 else slice(0, -s)
            self.free[tuple(idx)] = 0
            self.last_free[tuple(idx)] = -np.inf

    # -- update ------------------------------------------------------------
    def update(self, pts: np.ndarray, cam: np.ndarray, now: float) -> np.ndarray:
        """Feed one frame of odom-frame points (Nx3, NaN rows allowed) seen from
        `cam` (odom xyz). Returns a bool per point: a conflict, judged against
        the evidence BEFORE this frame (a frame cannot conflict with itself)."""
        c = self.c
        cam = np.asarray(cam, dtype=np.float64)
        self.recentre(float(cam[0]), float(cam[1]))
        pts = np.asarray(pts, dtype=np.float64)
        finite = np.isfinite(pts).all(axis=1)
        rng = np.linalg.norm(pts - cam, axis=1)
        valid = finite & (rng <= c["max_range_m"])
        band = valid & (pts[:, 2] >= c["min_height_m"]) & (pts[:, 2] <= c["max_height_m"])

        conflict = np.zeros(len(pts), dtype=bool)
        hx, hy, hin = self._cells(pts[:, 0], pts[:, 1])
        hit = band & hin
        if hit.any():
            # The whole 3x3 neighbourhood must be seen free, not just the cell.
            # A surface sitting on a cell boundary (a shelf's side face) puts
            # returns into a cell whose free evidence came from rays passing
            # alongside it; the neighbour INSIDE the shelf was never traced, so
            # its count is 0 and the min catches it. A person on open floor has
            # free all round. Cost: someone pressed against a shelf conflicts
            # only on their open side, which is still a prompt for SAM.
            free3 = _min3(self.free)
            last3 = _min3(self.last_free)
            f = free3[hx[hit], hy[hit]]
            fresh = (now - last3[hx[hit], hy[hit]]) <= c["recent_s"]
            conflict[hit] = (f >= c["min_free"]) & fresh

        hit_cells = np.unique(hx[hit] * self.n + hy[hit])
        # Free space uses EVERY return, with long rays cut at max_range rather
        # than dropped: a wall 6 m away is too noisy to place, but the first
        # 4.5 m of the ray to it are certainly empty. Dropping them left a
        # camera at 1 m with body-height free space only where its floor rays
        # were still above min_height_m, about 3.5 m out.
        far = finite & ~valid
        ends = pts.copy()
        ends[far] = cam + (pts[far] - cam) * (c["max_range_m"] / rng[far])[:, None]
        lens = np.where(far, c["max_range_m"], rng)
        self._mark_free(ends[finite], lens[finite], cam, now, hit_cells)
        if hit_cells.size:
            ix, iy = hit_cells // self.n, hit_cells % self.n
            self.free[ix, iy] = np.maximum(self.free[ix, iy] - c["hit_decay"], 0)
        return conflict

    def _mark_free(self, pts, rng, cam, now, hit_cells) -> None:
        """+1 (capped) to every cell some ray crosses inside the body band,
        at most once per cell per frame, except cells hit this same frame.

        A 10 cm cell straddling a shelf's edge gets rays through its empty half
        AND returns off its full half in the same frame. Counting the rays made
        those edge cells "seen free" and the shelf itself a conflict from a
        moving camera; letting the hit win keeps boundary cells out."""
        c = self.c
        if len(pts) == 0:
            return
        steps = int(np.ceil(c["max_range_m"] / c["ray_step_m"]))
        frac = np.linspace(0.0, 1.0, steps, dtype=np.float64)[None, :]
        keep = np.clip((rng - c["end_margin_m"]) / np.maximum(rng, 1e-6), 0, 1)
        t = frac * keep[:, None]                        # N x S, stops short of the hit
        d = pts - cam
        sx = cam[0] + t * d[:, 0:1]
        sy = cam[1] + t * d[:, 1:2]
        sz = cam[2] + t * d[:, 2:3]
        inband = (sz >= c["min_height_m"]) & (sz <= c["max_height_m"])
        ix, iy, inside = self._cells(sx[inband], sy[inband])
        cells = np.unique(ix[inside] * self.n + iy[inside])
        cells = np.setdiff1d(cells, hit_cells, assume_unique=True)
        if cells.size == 0:
            return
        gx, gy = cells // self.n, cells % self.n
        self.free[gx, gy] = np.minimum(self.free[gx, gy] + 1, c["free_cap"])
        self.last_free[gx, gy] = now


def _min3(a: np.ndarray) -> np.ndarray:
    p = np.pad(a, 1, mode="edge")
    h, w = a.shape
    out = a.copy()
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            np.minimum(out, p[dy:dy + h, dx:dx + w], out=out)
    return out


def backproject_odom(depth: np.ndarray, K, R_odom_cam, t_odom_cam, stride: int) -> np.ndarray:
    """Decimated depth image → (h/stride * w/stride) x 3 odom points, row-major,
    NaN rows where depth is invalid. Row order matches depth[::stride, ::stride]."""
    fx, fy, cx, cy = K
    d = depth[::stride, ::stride].astype(np.float64)
    h, w = d.shape
    us = np.arange(w) * stride
    vs = np.arange(h) * stride
    x = (us[None, :] - cx) / fx * d
    y = (vs[:, None] - cy) / fy * d
    cam = np.stack([x, y, d], axis=-1).reshape(-1, 3)
    cam[~(np.isfinite(cam).all(axis=1) & (cam[:, 2] > 0))] = np.nan
    R = np.asarray(R_odom_cam, dtype=np.float64)
    return cam @ R.T + np.asarray(t_odom_cam, dtype=np.float64)
