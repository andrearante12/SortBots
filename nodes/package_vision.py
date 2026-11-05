#!/usr/bin/env python3
"""Find a package's magnet plate in one RGB + depth frame. numpy and cv2 only, no ROS.

The plate is what the arm has to reach, so that is what is detected: its centre and
its OUTWARD normal, in the map frame. Nothing about the box's own pose is needed, and
nothing about its identity either (the pick controller tells packages apart by where
the work order says they are, nodes/pick_planner.py).

Markerless, on purpose (no fiducial on the plate): the sim authors the box in
cardboard and the plate near-black (simtest/world.py DEFAULT_PACKAGE_COLOR and
PLATE_COLOR), and a real package would carry a dark steel plate on a brown box, so

    1. segment  colour thresholds in HSV: cardboard body, dark plate
    2. gate     a plate blob counts only if cardboard surrounds it: merely touching
                some is not enough, the gaps between cartons on a rack do that
                (2026-11-05, ~1500 false plates in one 38 s run of the full warehouse)
    3. fit      back-project the plate's depth pixels (the depth image is aligned to
                the colour one), least-squares plane through them, the normal turned
                to face the camera; refuse a fit that isn't flat
    4. check    in metres now the depth is known: plate-sized, and lying in the plane
                of the cardboard around it (a dark gap is behind its carton, a plate
                is on it)
    5. report   plate centre (the plane point under the mean, minus the plate's own
                thickness) and normal, in the camera's optical frame; to_map() moves
                them to the map frame

What this does not do: see a plate that faces away from the camera, tell two boxes of
the same colour apart, or cope with a plate so far off-axis that its depth pixels are
mostly the box's edge. The pick controller scans (turns in place) for the first, takes
the work order's location for the second, and docks close enough that the third
goes away: docs/manipulation.md.

Optical frame: x right, y down, z forward (what the camera publishes).
"""
from __future__ import annotations

import math

import numpy as np

# Every key here is documented in configs/package_detector.yaml, which may override any.
DEFAULTS = {
    # OpenCV HSV: H 0-179, S and V 0-255.
    "box": {"h": [6, 26], "s": [50, 230], "v": [70, 255]},
    "plate": {"v_max": 60, "s_max": 140},
    "plate_thickness_m": 0.004,
    "min_plate_px": 40,
    "ring_px": 5,              # width of the ring looked at around a plate blob
    "ring_frac": 0.5,          # how much of that ring has to be cardboard
    "ring_plane_m": 0.02,      # how far the ring's cardboard may lie off the plate's plane
    "min_plate_m": 0.03,       # the plate's long side...
    "max_plate_m": 0.2,        # ...and the ceiling on it
    "min_plate_short_m": 0.02,  # its short side: a thin dark strip is a rack edge
    "min_range_m": 0.3,
    "max_range_m": 4.0,
    "min_points": 25,
    "plane_rms_m": 0.012,
    "stride": 1,
}


def depth_to_metres(data: bytes, encoding: str, h: int, w: int) -> np.ndarray | None:
    """A depth image's bytes as float32 metres (the same two encodings
    nodes/obstacle_cloud.py reads), or None for anything else."""
    if encoding in ("16UC1", "mono16"):
        return np.frombuffer(data, dtype=np.uint16).reshape(h, w).astype(np.float32) * 0.001
    if encoding == "32FC1":
        return np.frombuffer(data, dtype=np.float32).reshape(h, w)
    return None


def merged(cfg: dict | None) -> dict:
    out = {k: (dict(v) if isinstance(v, dict) else v) for k, v in DEFAULTS.items()}
    for k, v in (cfg or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k].update(v)
        else:
            out[k] = v
    return out


def segment(rgb: np.ndarray, cfg: dict | None = None) -> tuple[np.ndarray, np.ndarray]:
    """(box_mask, plate_mask), both bool (H, W), from an RGB uint8 image."""
    import cv2

    c = merged(cfg)
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    b = c["box"]
    box = ((h >= b["h"][0]) & (h <= b["h"][1]) & (s >= b["s"][0]) & (s <= b["s"][1])
           & (v >= b["v"][0]) & (v <= b["v"][1]))
    p = c["plate"]
    plate = (v <= p["v_max"]) & (s <= p["s_max"])
    k = np.ones((3, 3), np.uint8)
    box = cv2.morphologyEx(box.astype(np.uint8), cv2.MORPH_OPEN, k).astype(bool)
    plate = cv2.morphologyEx(plate.astype(np.uint8), cv2.MORPH_OPEN, k).astype(bool)
    return box, plate


def _plate_blobs(box: np.ndarray, plate: np.ndarray, c: dict):
    """(mask, cardboard ring, bbox) of every dark blob big enough and ringed by cardboard.

    The ring is measured inside the image, so a plate cut off by the frame's edge is
    judged on the part of it that shows."""
    import cv2

    n, labels, stats, _ = cv2.connectedComponentsWithStats(plate.astype(np.uint8), connectivity=8)
    side = 2 * c["ring_px"] + 1
    ring_k = np.ones((side, side), np.uint8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < c["min_plate_px"]:
            continue
        blob = labels == i
        ring = cv2.dilate(blob.astype(np.uint8), ring_k).astype(bool) & ~blob
        cardboard = ring & box
        if int(cardboard.sum()) < c["ring_frac"] * int(ring.sum()):
            continue
        out.append((blob, cardboard, (int(x), int(y), int(w), int(h))))
    return out


def fit_plane(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """(centroid, unit normal, rms distance) of the least-squares plane through (N, 3)."""
    c = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - c, full_matrices=False)
    n = vt[2]
    rms = float(np.sqrt(np.mean(((points - c) @ n) ** 2)))
    return c, n, rms


def plate_extent(points: np.ndarray, centroid: np.ndarray, normal: np.ndarray) -> tuple[float, float]:
    """(long, short) side in metres of (N, 3) points on a plane. 2nd to 98th percentile,
    so a few stray edge pixels do not stretch it."""
    d = points - centroid
    d = d - np.outer(d @ normal, normal)
    _, _, vt = np.linalg.svd(d, full_matrices=False)
    sides = [float(np.ptp(np.percentile(d @ axis, [2, 98]))) for axis in vt[:2]]
    return max(sides), min(sides)


def _back_project(us, vs, z, K) -> np.ndarray:
    fx, fy, cx, cy = K
    return np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1)


def _ring_off_plane(ring, depth_m, K, centroid, normal, c) -> float | None:
    """Median distance of the cardboard ring's depth pixels from the plate's plane, or
    None if the ring has no usable depth (holes are not evidence either way)."""
    vs, us = np.nonzero(ring)
    z = depth_m[vs, us].astype(np.float64)
    ok = np.isfinite(z) & (z > c["min_range_m"]) & (z < c["max_range_m"])
    if not ok.any():
        return None
    pts = _back_project(us[ok], vs[ok], z[ok], K)
    return float(np.median(np.abs((pts - centroid) @ normal)))


def detect_plates(rgb: np.ndarray, depth_m: np.ndarray, K, cfg: dict | None = None) -> list[dict]:
    """Every plate found in the frame, in the camera's optical frame:
    [{c: (x, y, z), n: (nx, ny, nz) outward, conf, px: (u, v, w, h), rms, npts}].

    K = (fx, fy, cx, cy) of the image the depth is aligned to."""
    import cv2

    c = merged(cfg)
    box, plate = segment(rgb, c)
    out = []
    for blob, ring, bbox in _plate_blobs(box, plate, c):
        # the blob's edge pixels mix plate and cardboard depth: fit on its interior
        core = cv2.erode(blob.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        if core.sum() < c["min_points"]:
            core = blob
        vs, us = np.nonzero(core)
        if c["stride"] > 1:
            vs, us = vs[:: c["stride"]], us[:: c["stride"]]
        z = depth_m[vs, us].astype(np.float64)
        ok = np.isfinite(z) & (z > c["min_range_m"]) & (z < c["max_range_m"])
        if int(ok.sum()) < c["min_points"]:
            continue
        us, vs, z = us[ok], vs[ok], z[ok]
        pts = _back_project(us, vs, z, K)
        # a plate is one flat thing: depth pixels from behind it (the wall past the
        # box's edge) would tilt the fit, so keep the pixels near the median range
        med = np.median(z)
        keep = np.abs(z - med) < 0.08
        if int(keep.sum()) < c["min_points"]:
            continue
        centroid, n, rms = fit_plane(pts[keep])
        if rms > c["plane_rms_m"]:
            continue
        long_m, short_m = plate_extent(pts[keep], centroid, n)
        if not c["min_plate_m"] <= long_m <= c["max_plate_m"] or short_m < c["min_plate_short_m"]:
            continue
        off = _ring_off_plane(ring, depth_m, K, centroid, n, c)
        if off is not None and off > c["ring_plane_m"]:
            continue
        if n @ (-centroid) < 0:          # turn the normal to face the camera
            n = -n
        face = centroid - n * c["plate_thickness_m"]    # the plate stands proud of the box
        conf = max(0.1, 1.0 - rms / c["plane_rms_m"]) * min(1.0, int(keep.sum()) / 200.0)
        out.append({"c": tuple(float(v) for v in face), "n": tuple(float(v) for v in n),
                    "conf": float(conf), "px": bbox, "rms": rms, "npts": int(keep.sum())})
    return out


def to_map(plate: dict, T_map_cam: np.ndarray) -> dict:
    """A camera-frame plate as {x, y, z, nx, ny, nz, conf, px} in the frame T_map_cam
    leads to (4x4, map <- optical)."""
    r, t = T_map_cam[:3, :3], T_map_cam[:3, 3]
    p = r @ np.array(plate["c"]) + t
    n = r @ np.array(plate["n"])
    return {"x": float(p[0]), "y": float(p[1]), "z": float(p[2]),
            "nx": float(n[0]), "ny": float(n[1]), "nz": float(n[2]),
            "conf": plate["conf"], "px": list(plate["px"])}


class Tracker:
    """Plates across frames: associates a new detection with the nearest known plate
    (EMA on position and normal), and forgets a plate not seen for `drop_after_s`.

    The EMA keeps the published pose from jittering with depth noise at 10 Hz; the
    memory means a plate that has just left the camera's view is still reported, with
    its age, so the controller can keep docking on it. Times are whatever clock the
    caller passes: the node uses sim time."""

    def __init__(self, assoc_m: float = 0.15, ema: float = 0.4, drop_after_s: float = 2.0):
        self.assoc_m, self.ema, self.drop_after_s = assoc_m, ema, drop_after_s
        self.tracks: list[dict] = []

    def update(self, plates: list[dict], now: float) -> list[dict]:
        seen = set()
        for p in plates:
            best, bd = None, self.assoc_m
            for i, tr in enumerate(self.tracks):
                d = math.dist((p["x"], p["y"], p["z"]), (tr["x"], tr["y"], tr["z"]))
                if d < bd and i not in seen:
                    best, bd = i, d
            if best is None:
                self.tracks.append(dict(p, last=now, hits=1))
                seen.add(len(self.tracks) - 1)
                continue
            tr = self.tracks[best]
            a = self.ema
            for k in ("x", "y", "z", "nx", "ny", "nz"):
                tr[k] = (1 - a) * tr[k] + a * p[k]
            norm = math.sqrt(tr["nx"] ** 2 + tr["ny"] ** 2 + tr["nz"] ** 2) or 1.0
            tr["nx"], tr["ny"], tr["nz"] = tr["nx"] / norm, tr["ny"] / norm, tr["nz"] / norm
            tr["conf"], tr["px"], tr["last"] = p["conf"], p["px"], now
            tr["hits"] += 1
            seen.add(best)
        self.tracks = [t for t in self.tracks if now - t["last"] <= self.drop_after_s]
        return [dict(t, age_s=round(now - t["last"], 3)) for t in self.tracks]
