#!/usr/bin/env python3
"""Open Isaac Sim with a visible viewport so you can watch the sim.

Two modes:

    # 1. Just open a USD and orbit the camera (click Play in Kit to step physics)
    python scripts/view_sim.py inspect assets/generated/carter.usd

    # 2. Spawn the robot AND drive it forward continuously
    python scripts/view_sim.py drive carter
    python scripts/view_sim.py drive xlerobot   [--velocity 0.5]

The drive subcommand mirrors the headless regression in
`tests/isaac/test_drive_forward.py` but runs with a viewport and loops
indefinitely. Close the Kit window (or Ctrl+C in the terminal) to quit.

Requires `source scripts/activate_isaac.sh` and a display server
(X11 / Wayland). On a hybrid-graphics laptop, the activate script's
`__NV_PRIME_RENDER_OFFLOAD=1` / `__GLX_VENDOR_LIBRARY_NAME=nvidia` should
already point Kit at the dGPU.
"""
from __future__ import annotations

import argparse
import signal
import sys
from pathlib import Path

# Sibling modules for shared helpers and per-robot configs.
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "isaac"))

from _isaac_utils import guard_against_ros2  # noqa: E402

guard_against_ros2()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Open Isaac Sim with a viewport (inspect a USD or drive a robot)."
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    p_inspect = sub.add_parser("inspect", help="Open a USD in the viewport and idle.")
    p_inspect.add_argument("usd", help="Path to a .usd / .usda file (e.g. assets/generated/carter.usd)")

    p_drive = sub.add_parser("drive", help="Spawn and drive a configured robot continuously.")
    p_drive.add_argument("robot", choices=["carter", "xlerobot"])
    p_drive.add_argument(
        "--velocity",
        type=float,
        default=None,
        help="Override the configured linear velocity (m/s)",
    )

    return p.parse_args()


args = parse_args()  # parse before SimulationApp so --help is fast

# Construct SimulationApp with a visible viewport. Everything below this point
# MUST come after the SimulationApp constructor (Kit modules are not importable
# before app startup).
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {"headless": False, "renderer": "RayTracedLighting"}
)


def _ctrlc(signum, frame):  # noqa: ARG001
    print("\nCtrl+C — closing SimulationApp...", flush=True)
    simulation_app.close()
    sys.exit(0)


signal.signal(signal.SIGINT, _ctrlc)


def _setup_world():
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.api.objects.ground_plane import GroundPlane
    from pxr import Sdf, UsdLux

    World.clear_instance()
    omni.usd.get_context().new_stage()

    world = World(stage_units_in_meters=1.0)
    GroundPlane(prim_path="/World/groundPlane", z_position=0.0)

    stage = omni.usd.get_context().get_stage()
    light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
    light.CreateIntensityAttr(2000)
    return world


def _aim_camera(at_x: float = 0.0, at_y: float = 0.0, at_z: float = 0.3):
    """Move the persp camera so it looks at the spawn point from a useful angle."""
    try:
        from omni.kit.viewport.utility.camera_state import ViewportCameraState
        from pxr import Gf

        cam = ViewportCameraState("/OmniverseKit_Persp")
        cam.set_position_world(Gf.Vec3d(at_x + 3.0, at_y - 3.0, at_z + 2.0), True)
        cam.set_target_world(Gf.Vec3d(at_x, at_y, at_z), True)
    except Exception:
        # Non-fatal — only affects the initial camera framing.
        pass


def cmd_inspect(args: argparse.Namespace) -> int:
    from isaacsim.core.utils.stage import add_reference_to_stage

    usd_path = Path(args.usd).resolve()
    if not usd_path.is_file():
        print(f"ERROR: USD not found: {usd_path}", file=sys.stderr)
        return 1

    _setup_world()
    add_reference_to_stage(usd_path=str(usd_path), prim_path="/World/robot")
    _aim_camera()

    print(
        f"Opened {usd_path}. Use the Kit UI (Play button, viewport orbit) to inspect.\n"
        "Close the window or Ctrl+C to quit."
    )
    while simulation_app.is_running():
        simulation_app.update()
    return 0


def cmd_drive(args: argparse.Namespace) -> int:
    import numpy as np
    from _robot_configs import ROBOT_CONFIGS

    cfg = ROBOT_CONFIGS[args.robot]
    usd_path = Path(cfg["out"])
    if not usd_path.is_file():
        print(
            f"ERROR: {usd_path} missing. Run scripts/import_urdf.py for {args.robot} first.",
            file=sys.stderr,
        )
        return 1

    world = _setup_world()
    sx, sy, sz = cfg["spawn_position"]
    _aim_camera(at_x=sx, at_y=sy, at_z=sz)

    if cfg["drive_mode"] == "diff_drive":
        return _drive_diff(world, cfg, usd_path, args.velocity, args.robot)
    if cfg["drive_mode"] == "prismatic":
        return _drive_prismatic(world, cfg, usd_path, args.velocity, args.robot)
    print(f"ERROR: unknown drive_mode {cfg['drive_mode']!r}", file=sys.stderr)
    return 1


def _drive_diff(world, cfg, usd_path: Path, vel_override: float | None, name: str) -> int:
    import numpy as np
    from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
        DifferentialController,
    )
    from isaacsim.robot.wheeled_robots.robots import WheeledRobot

    wb = WheeledRobot(
        prim_path="/World/robot",
        name=name,
        wheel_dof_names=cfg["wheel_dof_names"],
        create_robot=True,
        usd_path=str(usd_path),
        position=np.array(cfg["spawn_position"]),
    )
    world.scene.add(wb)
    world.reset()
    ctrl = DifferentialController(
        name="diff", wheel_radius=cfg["wheel_radius"], wheel_base=cfg["wheel_base"]
    )
    vel = vel_override if vel_override is not None else cfg["linear_velocity"]
    print(f"Driving {name} forward at {vel} m/s. Close window or Ctrl+C to stop.")
    while simulation_app.is_running():
        wb.apply_wheel_actions(ctrl.forward(np.array([vel, 0.0])))
        world.step(render=True)
    return 0


def _drive_prismatic(world, cfg, usd_path: Path, vel_override: float | None, name: str) -> int:
    import numpy as np
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction

    add_reference_to_stage(usd_path=str(usd_path), prim_path="/World/robot")
    rb = Robot(prim_path="/World/robot", name=name, position=np.array(cfg["spawn_position"]))
    world.scene.add(rb)
    world.reset()

    dof_names = list(rb.dof_names)
    dof_idx = dof_names.index(cfg["drive_dof_name"])
    n_dof = len(dof_names)
    vel = vel_override if vel_override is not None else cfg["target_velocity"]
    print(
        f"Driving {name} via {cfg['drive_dof_name']} at {vel} m/s. "
        "Close window or Ctrl+C to stop."
    )
    while simulation_app.is_running():
        velocities = np.zeros(n_dof)
        velocities[dof_idx] = vel
        rb.apply_action(ArticulationAction(joint_velocities=velocities))
        world.step(render=True)
    return 0


try:
    if args.cmd == "inspect":
        rc = cmd_inspect(args)
    elif args.cmd == "drive":
        rc = cmd_drive(args)
    else:
        rc = 1
except Exception as e:
    print(f"ERROR: {type(e).__name__}: {e}", file=sys.stderr)
    import traceback

    traceback.print_exc()
    rc = 1
finally:
    simulation_app.close()

sys.exit(rc)
