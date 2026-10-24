"""Named nav waypoints: the working set the operator edits on the map.

Pure python, no rclpy: shared by nodes/task_manager.py (where to drive) and
nodes/fleet_tasks.py (where a task is, for auction bids), and testable offline
with the system python3. Split out of task_manager.py, which imports rclpy at
module scope, so fleet_tasks and its tests don't need a sourced ROS 2.

The file is keyed by platform ({"sim": [{name,x,y,yaw}, ...], "real": [...]})
and honours the same SORTBOTS_NAV_WAYPOINTS override webui/maps_lib.py uses so
tests and alternate layouts stay in lockstep. The override is read on every
call, not at import: a test that sets it after something already imported this
module must still get its own file.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def nav_waypoints_file() -> Path:
    env = os.environ.get("SORTBOTS_NAV_WAYPOINTS")
    return Path(env) if env else REPO_ROOT / "data" / "nav_waypoints.json"


class Waypoint:
    def __init__(self, spec: dict):
        self.name = spec["name"]
        self.x = float(spec["x"])
        self.y = float(spec["y"])
        self.yaw = float(spec.get("yaw", 0.0))


def load_waypoints(platform: str, path: Path | None = None) -> dict[str, Waypoint]:
    """Named destinations for `platform` from the working set, keyed by name.

    A missing/empty/malformed file yields no waypoints (tasks then fail
    validation until the operator places some) rather than crashing the node.
    Callers re-read it on every dispatch/request, so a pin placed on the map
    resolves without a restart — the stations file this replaced was load-once.
    """
    try:
        raw = json.loads((path or nav_waypoints_file()).read_text())
    except (OSError, ValueError):
        return {}
    points = raw.get(platform, []) if isinstance(raw, dict) else []
    out: dict[str, Waypoint] = {}
    for spec in points:
        try:
            wp = Waypoint(spec)
        except (KeyError, TypeError, ValueError):
            continue   # skip a malformed entry, keep the rest
        out[wp.name] = wp
    return out
