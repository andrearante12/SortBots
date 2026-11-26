#!/usr/bin/env python3
"""Find boxes standing on a table from depth alone. numpy and cv2, no ROS, no colour.

What the manipulation state looks at, and what lets a pick start from "whatever is on this
table" instead of a location written in a work order. Everything is done in the map frame,
where "up" is known:

    1. points   back-project every few depth pixels and move them to the map
    2. surface  the supporting surface is the height most points agree on: a flat top piles
                up in one height bin, a wall or a rack spreads over all of them
    3. extent   the largest connected patch at that height, as a rotated rectangle. The box
                hides part of it; a rectangle fills that back in
    4. objects  points above that rectangle, in connected clusters. Racks behind the table
                are outside the rectangle, however tall they are
    5. fit      a rotated rectangle round each cluster gives x, y, yaw and size; the highest
                points give the height

What this does not do: say which side of the box has the magnet (the plate is 4 mm proud,
below what a real D435 resolves; nodes/package_vision.py finds it), or separate two boxes
that touch. yaw is the rectangle's, so it is only known modulo 90 degrees.
"""
from __future__ import annotations

import math

import numpy as np

# Every key here is documented in configs/package_detector.yaml's `boxes:`, which may override any.
DEFAULTS = {
    "stride": 2,
    "min_range_m": 0.25,
    "max_range_m": 2.5,
    "min_surface_z": 0.3,
    "max_surface_z": 1.5,
    "bin_m": 0.01,
    "surface_tol_m": 0.012,
    "min_surface_pts": 300,
    "surface_cell_m": 0.02,
    "margin_m": 0.03,
    "clear_m": 0.015,
    "max_height_m": 0.4,
    # Coarse enough that a box top seen at a grazing angle stays one cluster: from 1.2 m its
    # depth rows land ~3 cm apart. The fit uses the points, so this costs no accuracy, only
    # the gap two boxes need between them to count as two.
    "cell_m": 0.02,
    "min_pts": 40,
    "min_side_m": 0.04,
    "max_side_m": 0.5,
    "min_height_m": 0.03,
}


def merged(cfg: dict | None) -> dict:
    return {**DEFAULTS, **(cfg or {})}


def map_points(depth_m: np.ndarray, K, T_map_cam: np.ndarray, c: dict):
    """(points (N, 3) in the map, us, vs) of the usable depth pixels, every `stride`-th."""
    fx, fy, cx, cy = K
    s = int(c["stride"])
    vs, us = np.mgrid[0:depth_m.shape[0]:s, 0:depth_m.shape[1]:s]
    z = depth_m[::s, ::s].astype(np.float64)
    ok = np.isfinite(z) & (z > c["min_range_m"]) & (z < c["max_range_m"])
    us, vs, z = us[ok], vs[ok], z[ok]
    cam = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1)
    return cam @ T_map_cam[:3, :3].T + T_map_cam[:3, 3], us, vs


def surface_height(z: np.ndarray, c: dict) -> float | None:
    """The height the most points agree on, or None if nothing is flat enough."""
    z = z[(z >= c["min_surface_z"]) & (z <= c["max_surface_z"])]
    if z.size < c["min_surface_pts"]:
        return None
    edges = np.arange(c["min_surface_z"], c["max_surface_z"] + c["bin_m"], c["bin_m"])
    counts, _ = np.histogram(z, bins=edges)
    i = int(np.argmax(counts))
    if counts[i] < c["min_surface_pts"]:
        return None
    near = z[np.abs(z - (edges[i] + c["bin_m"] / 2)) < c["surface_tol_m"]]
    return float(np.median(near))


def _cells(xy: np.ndarray, cell: float, close: int):
    """Rasterize points: (labels, cell index of each point) of the connected cells."""
    import cv2

    ij = np.floor((xy - xy.min(axis=0)) / cell).astype(int) + 1
    grid = np.zeros((ij[:, 1].max() + 2, ij[:, 0].max() + 2), np.uint8)
    grid[ij[:, 1], ij[:, 0]] = 1
    if close > 1:
        grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((close, close), np.uint8))
    n, labels = cv2.connectedComponents(grid, connectivity=8)
    return n, labels, ij


def _rect(xy: np.ndarray):
    """((cx, cy), (w, h), angle_rad) of the smallest rotated rectangle round the points."""
    import cv2

    (cx, cy), (w, h), a = cv2.minAreaRect(xy.astype(np.float32))
    return (cx, cy), (w, h), math.radians(a)


def _inside(xy: np.ndarray, rect, margin: float) -> np.ndarray:
    (cx, cy), (w, h), a = rect
    d = xy - (cx, cy)
    u = d[:, 0] * math.cos(a) + d[:, 1] * math.sin(a)
    v = -d[:, 0] * math.sin(a) + d[:, 1] * math.cos(a)
    return (np.abs(u) <= w / 2 + margin) & (np.abs(v) <= h / 2 + margin)


def find_boxes(depth_m: np.ndarray, K, T_map_cam: np.ndarray, cfg: dict | None = None) -> list[dict]:
    """Boxes on the surface in view, nearest-sized first: [{x, y, z (centre), yaw, size:
    [long, short, height], support_z, conf, px: [u, v, w, h]}], in the map frame."""
    c = merged(cfg)
    pts, us, vs = map_points(depth_m, K, T_map_cam, c)
    if not len(pts):
        return []
    z0 = surface_height(pts[:, 2], c)
    if z0 is None:
        return []
    flat = pts[np.abs(pts[:, 2] - z0) < c["surface_tol_m"], :2]
    n, labels, ij = _cells(flat, c["surface_cell_m"], 5)
    if n < 2:
        return []
    of_point = labels[ij[:, 1], ij[:, 0]]
    biggest = 1 + int(np.argmax(np.bincount(of_point, minlength=n)[1:]))
    surface = _rect(flat[of_point == biggest])

    up = (pts[:, 2] > z0 + c["clear_m"]) & (pts[:, 2] < z0 + c["max_height_m"])
    up &= _inside(pts[:, :2], surface, c["margin_m"])
    if int(up.sum()) < c["min_pts"]:
        return []
    above, au, av = pts[up], us[up], vs[up]
    n, labels, ij = _cells(above[:, :2], c["cell_m"], 3)
    of_point = labels[ij[:, 1], ij[:, 0]]
    out = []
    for k in range(1, n):
        mine = of_point == k
        if int(mine.sum()) < c["min_pts"]:
            continue
        (cx, cy), (w, h), yaw = _rect(above[mine, :2])
        long_m, short_m = max(w, h), min(w, h)
        height = float(np.percentile(above[mine, 2], 98)) - z0
        if (short_m < c["min_side_m"] or long_m > c["max_side_m"] or height < c["min_height_m"]):
            continue
        u, v = au[mine], av[mine]
        out.append({"x": round(float(cx), 4), "y": round(float(cy), 4), "z": round(z0 + height / 2, 4),
                    "yaw": round(math.atan2(math.sin(yaw), math.cos(yaw)), 4),
                    "size": [round(long_m, 3), round(short_m, 3), round(height, 3)],
                    "support_z": round(z0, 4), "conf": round(min(1.0, int(mine.sum()) / 200.0), 3),
                    "px": [int(u.min()), int(v.min()), int(np.ptp(u)), int(np.ptp(v))]})
    return sorted(out, key=lambda b: -b["conf"])
