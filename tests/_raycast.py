"""Tiny numpy raycaster of axis-aligned boxes, for the depth-filter tests.

The world is z-up (= <id>/odom). Boxes are ((x0, x1), (y0, y1), (z0, z1)).
The camera can actually move and the scene has real depth edges: a shelf in
front of a far wall is exactly what an unwarped frame diff flags as a mover.
"""
from __future__ import annotations

import numpy as np

# Optical (x right, y down, z fwd) -> map (x fwd, y left, z up).
R_MAP_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)

W, H = 160, 120
K_SMALL = (80.0, 80.0, W / 2, H / 2)
SHELF = ((2.5, 3.0), (0.3, 1.5), (0.0, 1.8))
WALL = ((6.0, 6.5), (-5.0, 5.0), (0.0, 3.0))
FLOOR = ((-1.0, 9.0), (-5.0, 5.0), (-1.0, 0.0))


def _person(y0):
    return ((3.5, 3.8), (y0, y0 + 0.4), (0.0, 1.7))


def _yaw(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def _cam(x, y, yaw):
    """World pose of the optical frame: body at (x, y, 1.0 m), yawed."""
    R = _yaw(yaw) @ R_MAP_OPT.astype(np.float64)
    return R, np.array([x, y, 1.0])


def _render(R, c, boxes, K=K_SMALL, h=H, w=W):
    fx, fy, cx, cy = K
    rx = (np.arange(w) - cx) / fx
    ry = (np.arange(h) - cy) / fy
    dirs = np.stack(np.broadcast_arrays(rx[None, :], ry[:, None], np.ones((1, 1))), -1)
    dw = dirs @ R.T  # optical z = 1, so the ray parameter IS the depth
    best = np.full((h, w), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for box in boxes:
            t0 = np.full((h, w), -np.inf)
            t1 = np.full((h, w), np.inf)
            for k, (lo, hi) in enumerate(box):
                a = (lo - c[k]) / dw[..., k]
                b = (hi - c[k]) / dw[..., k]
                t0 = np.maximum(t0, np.minimum(a, b))
                t1 = np.minimum(t1, np.maximum(a, b))
            hit = (t1 >= t0) & (t0 > 0)
            best = np.where(hit & (t0 < best), t0, best)
    return np.where(np.isfinite(best), best, np.nan).astype(np.float32)


def _rel(Rp, cp, Rc, cc):
    """prev <- cur optical transform."""
    return (Rp.T @ Rc).astype(np.float32), (Rp.T @ (cc - cp)).astype(np.float32)
