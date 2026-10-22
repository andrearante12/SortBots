"""What the test layer needs to know about the robots, read from the files that
already own it — so a footprint change in Nav2 or a new roster entry can't
silently leave the metrics measuring a different robot than the one driving.

    roster()           configs/robots.yaml ids, in order
    spawn(scene)       {id: (x, y)} spawn poses for a scene
    footprint_points() Nav2's local_costmap footprint (body frame, metres)
    camera_hfov_rad()  configs/sensors/d435.json horizontal field of view
"""
from __future__ import annotations

import functools
import json
import math
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ROBOTS_YAML = REPO_ROOT / "configs" / "robots.yaml"
NAV2_PARAMS = REPO_ROOT / "configs" / "nav2_params.yaml"
D435_JSON = REPO_ROOT / "configs" / "sensors" / "d435.json"

# Fallback if nav2_params.yaml ever stops carrying a parseable footprint: the
# 0.5 x 0.4 m rectangle it has carried since Nav2 came up.
DEFAULT_FOOTPRINT = ((0.25, 0.2), (0.25, -0.2), (-0.25, -0.2), (-0.25, 0.2))


@functools.lru_cache(maxsize=1)
def _roster_doc() -> dict:
    with open(ROBOTS_YAML) as f:
        return yaml.safe_load(f)


def roster() -> list[str]:
    return [r["id"] for r in _roster_doc()["robots"]]


def spawn(scene: str) -> dict[str, tuple[float, float]]:
    return {r["id"]: (float(r["spawn"][scene][0]), float(r["spawn"][scene][1]))
            for r in _roster_doc()["robots"] if scene in r.get("spawn", {})}


@functools.lru_cache(maxsize=1)
def footprint_points() -> tuple[tuple[float, float], ...]:
    """Nav2's footprint — the shape the planner itself keeps clear."""
    try:
        with open(NAV2_PARAMS) as f:
            doc = yaml.safe_load(f)
        raw = doc["/**/local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
        pts = yaml.safe_load(raw)  # "[[x, y], ...]" is valid YAML flow syntax
        out = tuple((float(x), float(y)) for x, y in pts)
        if len(out) >= 3:
            return out
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        pass
    return DEFAULT_FOOTPRINT


def footprint_radius() -> float:
    return max(math.hypot(x, y) for x, y in footprint_points())


@functools.lru_cache(maxsize=1)
def camera_hfov_rad() -> float:
    try:
        cfg = json.loads(D435_JSON.read_text())
        return 2.0 * math.atan(float(cfg["width"]) / 2.0 / float(cfg["fx"]))
    except (OSError, KeyError, ValueError, ZeroDivisionError):
        return math.radians(87.0)  # D435 datasheet
