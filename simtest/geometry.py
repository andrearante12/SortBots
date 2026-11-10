"""2D shapes and distances for the test layer, in the `map` (== Isaac world) frame.

Pure python + math, no numpy, because this is imported from three different
interpreters: Isaac's venv (spawn_warehouse.py), system python3 (session.py,
the probe, the metrics) and whatever runs the offline tests. Everything is a
plain tuple so a shape round-trips through JSON (/sim/ground_truth, events.jsonl)
without a custom encoder.

A shape is either
    ("circle", (cx, cy), r)
    ("poly",   ((x0, y0), (x1, y1), ...))      convex, counter-clockwise
and distance() is the gap between two of them — 0 when they touch or overlap.
That gap is what every clearance-style metric is built on: a robot footprint
(a rectangle, Nav2's own footprint) against an actor (a cylinder's circle, a
box's rectangle).
"""
from __future__ import annotations

import math

Point = tuple[float, float]


def rect(cx: float, cy: float, yaw: float, sx: float, sy: float) -> tuple:
    """An sx x sy rectangle centred on (cx, cy), rotated by yaw, as a poly."""
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = sx / 2.0, sy / 2.0
    corners = ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))  # CCW
    return ("poly", tuple((cx + c * x - s * y, cy + s * x + c * y) for x, y in corners))


def footprint_poly(points, cx: float, cy: float, yaw: float) -> tuple:
    """A body-frame footprint polygon (Nav2's `footprint:` list) placed at a pose."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(cx + c * x - s * y, cy + s * x + c * y) for x, y in points]
    if _signed_area(pts) < 0:
        pts.reverse()
    return ("poly", tuple(pts))


def circle(cx: float, cy: float, r: float) -> tuple:
    return ("circle", (cx, cy), r)


def _signed_area(pts) -> float:
    return 0.5 * sum(pts[i][0] * pts[(i + 1) % len(pts)][1]
                     - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))


def _point_segment(p: Point, a: Point, b: Point) -> float:
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def point_in_poly(p: Point, pts) -> bool:
    """Convex CCW polygon containment (boundary counts as inside)."""
    n = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < -1e-12:
            return False
    return True


def point_poly_distance(p: Point, pts) -> float:
    if point_in_poly(p, pts):
        return 0.0
    n = len(pts)
    return min(_point_segment(p, pts[i], pts[(i + 1) % n]) for i in range(n))


def _segments_intersect(a, b, c, d) -> bool:
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return (o1 * o2 <= 0) and (o3 * o4 <= 0)


def poly_poly_distance(p1, p2) -> float:
    if any(point_in_poly(v, p2) for v in p1) or any(point_in_poly(v, p1) for v in p2):
        return 0.0
    n1, n2 = len(p1), len(p2)
    best = math.inf
    for i in range(n1):
        a, b = p1[i], p1[(i + 1) % n1]
        for j in range(n2):
            c, d = p2[j], p2[(j + 1) % n2]
            if _segments_intersect(a, b, c, d):
                return 0.0
            best = min(best, _point_segment(a, c, d), _point_segment(b, c, d),
                       _point_segment(c, a, b), _point_segment(d, a, b))
    return best


def distance(s1: tuple, s2: tuple) -> float:
    """Gap between two shapes; 0.0 when they touch or overlap."""
    if s1[0] == "circle" and s2[0] == "circle":
        return max(0.0, math.dist(s1[1], s2[1]) - s1[2] - s2[2])
    if s1[0] == "circle":
        s1, s2 = s2, s1
    if s2[0] == "circle":
        return max(0.0, point_poly_distance(s2[1], s1[1]) - s2[2])
    return poly_poly_distance(s1[1], s2[1])


def centre(shape: tuple) -> Point:
    if shape[0] == "circle":
        return shape[1]
    pts = shape[1]
    return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))


def bounds(shape: tuple) -> tuple[float, float, float, float]:
    """(xmin, ymin, xmax, ymax)."""
    if shape[0] == "circle":
        (cx, cy), r = shape[1], shape[2]
        return (cx - r, cy - r, cx + r, cy + r)
    xs = [p[0] for p in shape[1]]
    ys = [p[1] for p in shape[1]]
    return (min(xs), min(ys), max(xs), max(ys))


def contains(shape: tuple, p: Point, margin: float = 0.0) -> bool:
    """Is p inside the shape grown by `margin`?"""
    if shape[0] == "circle":
        return math.dist(shape[1], p) <= shape[2] + margin
    return point_poly_distance(p, shape[1]) <= margin


def from_json(obj) -> tuple:
    """Undo JSON's tuple -> list conversion (events.jsonl, /sim/ground_truth)."""
    if obj[0] == "circle":
        return ("circle", (float(obj[1][0]), float(obj[1][1])), float(obj[2]))
    return ("poly", tuple((float(x), float(y)) for x, y in obj[1]))
