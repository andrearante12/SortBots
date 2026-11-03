"""What one robot sees: a depth-camera cone ray-cast on the TRUE world.

Stands in for the D435 + RTAB-Map grid + obstacle_detector chain, at the
level the fleet layer cares about:

  * Static hits (walls, racks, a pallet dropped in an aisle) go into the
    robot's belief as occupied; every cell a ray crosses before its hit goes
    in as free. That is what RTAB-Map's grid does with depth.
  * Hits on a peer or a scripted mover do NOT enter the belief (that's
    nodes/dynamic_obstacle_filter.py's job on the robots: keep movers out of
    SLAM). They come back as detections for the local layer, the way
    obstacle_detector's tracks reach Nav2's obstacle layer. Rays stop at them,
    so they occlude what is behind.
  * Forward cone only (D435: ~87 deg, `range_m` from obstacle_detector's crop):
    a robot does not see what is behind it or around a corner. Peers it
    cannot see it knows only from their Status, as on the robots.

Rays are spaced so neighbouring rays are at most half a cell apart at full
range, and each is sampled every half cell, so no cell inside the cone is
skipped. Vectorised: one scan is a few numpy gathers, not a python loop.
"""
from __future__ import annotations

import math

import numpy as np

from .grid import Grid


class Sensor:
    def __init__(self, grid: Grid, cfg: dict):
        self.grid = grid
        self.range_m = float(cfg["range_m"])
        fov = math.radians(float(cfg["fov_deg"]))
        step = 0.5 * grid.res
        n_rays = max(3, int(math.ceil(self.range_m * fov / step)) + 1)
        self.rel = np.linspace(-fov / 2, fov / 2, n_rays)
        self.s = np.arange(step, self.range_m + 1e-9, step)
        self.ns = len(self.s)

    def scan(self, x: float, y: float, yaw: float, static_occ: np.ndarray,
             robot_owner: np.ndarray, actor_owner: np.ndarray, self_tag: int):
        """-> (free_idx, occ_idx, detections). Arrays are flat views of the
        world's (h, w) grids. `self_tag` is this robot's value in
        robot_owner (its own body is not an obstacle). Detections are
        [(kind, ident, x, y)] with kind "robot" | "actor", one per object,
        at the nearest point the cone saw of it."""
        g = self.grid
        ang = yaw + self.rel
        px = x + np.outer(np.cos(ang), self.s)
        py = y + np.outer(np.sin(ang), self.s)
        cx = np.floor((px - g.ox) / g.res).astype(np.int64)
        cy = np.floor((py - g.oy) / g.res).astype(np.int64)
        inb = (cx >= 0) & (cy >= 0) & (cx < g.width) & (cy < g.height)
        idx = np.where(inb, cy * g.width + cx, 0)
        static = np.where(inb, static_occ[idx], True)
        rob = np.where(inb, robot_owner[idx], 0)
        act = np.where(inb, actor_owner[idx], 0)
        dyn = ((rob != 0) & (rob != self_tag)) | (act != 0)
        hit = static | dyn
        first = np.argmax(hit, axis=1)
        rows = np.arange(len(first))
        has = hit[rows, first]
        first = np.where(has, first, self.ns)
        before = np.arange(self.ns)[None, :] < first[:, None]
        free_idx = np.unique(idx[before & inb])
        hr = rows[has]
        hc = first[has]
        st = static[hr, hc] & inb[hr, hc]
        occ_idx = np.unique(idx[hr[st], hc[st]])
        dets = []
        dr, dc = hr[~st], hc[~st]
        if len(dr):
            seen: dict[tuple, tuple] = {}
            for r, c in zip(dr.tolist(), dc.tolist()):
                if rob[r, c] != 0 and rob[r, c] != self_tag:
                    key = ("robot", int(rob[r, c]) - 1)
                elif act[r, c] != 0:
                    key = ("actor", int(act[r, c]) - 1)
                else:
                    continue
                if key not in seen or c < seen[key][0]:
                    seen[key] = (c, float(px[r, c]), float(py[r, c]))
            dets = [(k[0], k[1], v[1], v[2]) for k, v in seen.items()]
        return free_idx, occ_idx, dets
