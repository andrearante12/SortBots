"""Per-robot configuration for Phase 2 import + drive tests.

Kept separate from conftest.py so test modules can import it as
`from _robot_configs import ROBOT_CONFIGS` (pytest doesn't expose the
conftest directory on sys.path during test-module collection).
"""
from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
GENERATED = REPO_ROOT / "assets" / "generated"
CONFIGS = REPO_ROOT / "configs" / "physics_overrides"

CARTER_URDF = Path(
    "/home/andre/isaacsim/venv/lib/python3.11/site-packages/isaacsim/exts"
    "/isaacsim.asset.importer.urdf/data/urdf/robots/carter/urdf/carter.urdf"
)
XLEROBOT_URDF = (
    REPO_ROOT
    / "third_party"
    / "XLeRobot"
    / "simulation"
    / "Maniskill"
    / "assets"
    / "xlerobot"
    / "xlerobot.urdf"
)


ROBOT_CONFIGS = {
    "carter": {
        "urdf": CARTER_URDF,
        "out": GENERATED / "carter.usd",
        "override": CONFIGS / "carter.json",
        "fix_base": False,
        "drive_mode": "diff_drive",
        "wheel_dof_names": ["left_wheel", "right_wheel"],
        "wheel_radius": 0.240,
        "wheel_base": 0.6284,
        "linear_velocity": 1.0,
        "spawn_position": (0.0, 0.0, 0.5),
        "explosion_z": 2.0,
    },
    "xlerobot": {
        "urdf": XLEROBOT_URDF,
        "out": GENERATED / "xlerobot.usd",
        "override": CONFIGS / "xlerobot.json",
        "fix_base": True,
        "drive_mode": "prismatic",
        "drive_dof_name": "root_x_axis_joint",
        "target_velocity": 0.5,
        "spawn_position": (0.0, 0.0, 0.0),
        "explosion_z": 2.0,
    },
}
