"""Named dispatch stations (configs/waypoints.yaml) and their dock poses.

Pure python, no rclpy: shared by nodes/task_manager.py (where to drive) and
nodes/fleet_tasks.py (where a task is, for auction bids), and testable
offline with the system python3.
"""
from __future__ import annotations

import math
from pathlib import Path

import yaml


class Station:
    def __init__(self, name: str, spec: dict):
        self.name = name
        self.kind = spec["kind"]
        self.x = float(spec["pose"]["x"])
        self.y = float(spec["pose"]["y"])
        self.yaw = float(spec["pose"]["yaw"])
        self.dock_offset_m = float(spec.get("dock_offset_m", 0.0))
        self.dock_lateral_m = float(spec.get("dock_lateral_m", 0.0))

    def dock_pose(self) -> tuple[float, float, float]:
        """Stop `dock_offset_m` short of the station, still facing it.

        The station pose is the pick/place point itself (shelf edge, bin);
        backing off along the reverse of the facing direction keeps the
        robot's arm workspace centered on that point instead of colliding
        with it.

        `dock_lateral_m` shifts the dock sideways, +90 degrees from the facing
        direction. The first arm is mounted off base_link's centreline (URDF:
        `Rotation` origin y=-0.0452, and the whole arm hangs to one side), so
        parking head-on does NOT put the station in the middle of the arm's
        workspace. Defaults to 0.0, which is the previous behaviour exactly.
        The sign that helps is an empirical question — jog the arm at the shelf
        and see which way it wants to go.
        """
        dx = self.dock_offset_m * math.cos(self.yaw)
        dy = self.dock_offset_m * math.sin(self.yaw)
        lx = -self.dock_lateral_m * math.sin(self.yaw)
        ly = self.dock_lateral_m * math.cos(self.yaw)
        return (self.x - dx + lx, self.y - dy + ly, self.yaw)


def load_stations(path: Path) -> dict[str, Station]:
    with open(path) as f:
        raw = yaml.safe_load(f)
    return {name: Station(name, spec) for name, spec in raw["stations"].items()}
