#!/usr/bin/env python3
"""Open the warehouse, spawn 2 XLeRobot instances, publish per-robot ROS 2.

End state (one terminal):
    source scripts/activate_isaac.sh
    python scripts/spawn_warehouse.py --duration 30

Both robots curve gently in the warehouse, each one publishing:

    /<robot>/odom              nav_msgs/Odometry
    /<robot>/camera/rgb        sensor_msgs/Image
    /<robot>/camera/depth      sensor_msgs/Image (32FC1, meters)
    /<robot>/tf                tf2_msgs/TFMessage  (full articulation tree)

with `<robot>` ∈ {robot_0, robot_1}. From a separate shell with ROS 2 Jazzy
in scope (or the bundled-lib env-var recipe in docs/isaac_sim_phase3.md),
`ros2 topic list` and `ros2 topic echo /robot_0/odom` confirm liveness.

Use `--no-headless` for the GUI viewport (slow on hybrid graphics; the
Phase 3 test runs headless). `--forever` disables the watchdog timeout
and runs until the window closes or you Ctrl+C.

Exit codes: 0 OK, 1 exception, 2 timeout. Per-script result file at
``/tmp/isaac_spawn_warehouse_result.txt``.
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
from pathlib import Path

# Sibling for shared helpers and the warehouse spawn-position table.
SCRIPTS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS_DIR))

from _isaac_utils import (  # noqa: E402
    cancel_timeout,
    emit_factory,
    gpu_name,
    guard_against_ros2,
    install_timeout,
)

guard_against_ros2()

REPO_ROOT = SCRIPTS_DIR.parent

XLEROBOT_USD = REPO_ROOT / "assets" / "generated" / "xlerobot.usd"
WAREHOUSE_USD = REPO_ROOT / "scenes" / "warehouse_v0.usd"
D435_CONFIG = REPO_ROOT / "configs" / "sensors" / "d435.json"
MPU6050_CONFIG = REPO_ROOT / "configs" / "sensors" / "mpu6050.json"
RESULT_FILE = os.environ.get(
    "ISAAC_SPAWN_WAREHOUSE_RESULT", "/tmp/isaac_spawn_warehouse_result.txt"
)


def _load_sensor_configs():
    import json

    with open(D435_CONFIG) as f:
        d435 = json.load(f)
    with open(MPU6050_CONFIG) as f:
        mpu = json.load(f)
    return d435, mpu


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Spawn 2 XLeRobots in the warehouse and publish per-robot ROS 2 topics."
    )
    p.add_argument("--headless", dest="headless", action="store_true")
    p.add_argument("--no-headless", dest="headless", action="store_false")
    p.set_defaults(headless=True)
    g = p.add_mutually_exclusive_group()
    g.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Sim seconds to run before clean shutdown (default 30)",
    )
    g.add_argument(
        "--forever",
        action="store_true",
        help="Run until Ctrl+C or window close. Disables the watchdog timeout.",
    )
    p.add_argument(
        "--drive",
        choices=("cmd_vel", "circle", "straight", "still"),
        default="cmd_vel",
        help=(
            "Per-robot motion model. `cmd_vel` (default) waits for external "
            "ROS 2 Twist messages on /robot_<n>/cmd_vel. The other modes are "
            "for ROS-2-free smoke tests."
        ),
    )
    return p.parse_args()


args = parse_args()

# SimulationApp MUST be constructed before any other isaacsim / omni / pxr imports.
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {"headless": args.headless, "renderer": "RayTracedLighting"}
)

# The ROS 2 bridge does NOT auto-load on pip-install Isaac Sim 5.1 — enable
# it explicitly AND tick the app several times so its OmniGraph node types
# (ROS2PublishOdometry, ROS2CameraHelper, ...) finish registering before
# the first og.Controller.edit that references them.
import omni.kit.app  # noqa: E402

_ext_manager = omni.kit.app.get_app().get_extension_manager()
_ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
for _ in range(10):
    simulation_app.update()

emit = emit_factory(RESULT_FILE)

if not args.forever:
    install_timeout(
        seconds=max(int(args.duration * 6), 240),
        hint=(
            "Spawn loop took too long. Common causes: Kit fell back to llvmpipe,\n"
            "asset reference failed to resolve, or one of the ROS 2 publisher\n"
            "nodes raised silently. Inspect /tmp/isaac_spawn_warehouse_result.txt."
        ),
        close_app=simulation_app.close,
    )


def _ctrlc(signum, frame):  # noqa: ARG001
    print("\nCtrl+C — closing SimulationApp...", flush=True)
    try:
        simulation_app.close()
    finally:
        sys.exit(0)


signal.signal(signal.SIGINT, _ctrlc)


def _ensure_xlerobot_usd() -> Path:
    if XLEROBOT_USD.is_file():
        return XLEROBOT_USD
    urdf = (
        REPO_ROOT
        / "third_party"
        / "XLeRobot"
        / "simulation"
        / "Maniskill"
        / "assets"
        / "xlerobot"
        / "xlerobot.urdf"
    )
    overrides = REPO_ROOT / "configs" / "physics_overrides" / "xlerobot.json"
    cmd = [
        sys.executable,
        str(SCRIPTS_DIR / "import_urdf.py"),
        "--urdf",
        str(urdf),
        "--out",
        str(XLEROBOT_USD),
        "--physics-overrides",
        str(overrides),
        "--fix-base",
    ]
    emit(f"  XLeRobot USD missing; subprocess: {' '.join(cmd)}")
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    if r.returncode != 0 or not XLEROBOT_USD.is_file():
        raise RuntimeError(
            f"import_urdf.py failed (rc={r.returncode}).\n"
            f"stdout:\n{r.stdout}\nstderr:\n{r.stderr}"
        )
    return XLEROBOT_USD


def _ensure_warehouse_usd() -> Path:
    if WAREHOUSE_USD.is_file():
        return WAREHOUSE_USD
    cmd = [
        sys.executable,
        str(SCRIPTS_DIR / "build_warehouse.py"),
        "--out",
        str(WAREHOUSE_USD),
    ]
    emit(f"  warehouse USD missing; subprocess: {' '.join(cmd)}")
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    if r.returncode != 0 or not WAREHOUSE_USD.is_file():
        raise RuntimeError(
            f"build_warehouse.py failed (rc={r.returncode}).\n"
            f"stdout:\n{r.stdout}\nstderr:\n{r.stderr}"
        )
    return WAREHOUSE_USD


def _spawn_robot(world, name: str, position, xlerobot_usd: Path, d435: dict, mpu: dict):
    import numpy as np
    import omni.kit.commands
    import omni.usd
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.sensors.camera import Camera
    from pxr import Gf, UsdGeom

    prim_path = f"/World/{name}"
    add_reference_to_stage(usd_path=str(xlerobot_usd), prim_path=prim_path)

    # `Robot(position=...)` doesn't translate fixed-base articulations — the
    # `root` link's USD position takes precedence. Set the prim's USD
    # translate directly. `XformCommonAPI` fails when the referenced USD has
    # a matrix transform op ("incompatible xformable"), so clear the existing
    # op order and add a fresh translate.
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*position))

    robot = Robot(prim_path=prim_path, name=name)
    world.scene.add(robot)

    # Camera at the URDF's head optical frame, D435-spec resolution.
    cam_path = f"{prim_path}/head_camera_rgb_optical_frame/cam"
    cam = Camera(
        prim_path=cam_path,
        resolution=(d435["width"], d435["height"]),
        frequency=d435["fps"],
    )

    # D435 intrinsics on the underlying USD Camera prim — the bridge's
    # ROS2CameraInfoHelper derives K/P/R/D from these attributes.
    cam_prim = stage.GetPrimAtPath(cam_path)
    ucam = UsdGeom.Camera(cam_prim)
    ucam.CreateFocalLengthAttr(float(d435["focal_length_mm"]))
    ucam.CreateHorizontalApertureAttr(float(d435["horizontal_aperture_mm"]))
    ucam.CreateVerticalApertureAttr(float(d435["vertical_aperture_mm"]))
    ucam.CreateClippingRangeAttr(
        Gf.Vec2f(*[float(v) for v in d435["clipping_range_m"]])
    )

    # MPU 6050 IMU under base_link. `IsaacSensorCreateImuSensor` authors the
    # IsaacImuSensor schema + sets the sensor period from `sensor_period`
    # (seconds). Translation is the mount offset; rotation is identity by
    # default (overridden via mpu["mount_rotation_wxyz"] if non-identity).
    parent_path = f"{prim_path}/base_link"
    imu_path = f"{parent_path}/imu_sensor"
    sensor_period = 1.0 / float(mpu["internal_rate_hz"])
    ok, _ = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path="/imu_sensor",
        parent=parent_path,
        sensor_period=sensor_period,
        translation=Gf.Vec3d(*[float(v) for v in mpu["mount_offset_m"]]),
        orientation=Gf.Quatd(
            float(mpu["mount_rotation_wxyz"][0]),
            float(mpu["mount_rotation_wxyz"][1]),
            float(mpu["mount_rotation_wxyz"][2]),
            float(mpu["mount_rotation_wxyz"][3]),
        ),
        linear_acceleration_filter_size=int(mpu["linear_acceleration_filter_size"]),
        angular_velocity_filter_size=int(mpu["angular_velocity_filter_size"]),
        orientation_filter_size=int(mpu["orientation_filter_size"]),
    )
    if not ok:
        raise RuntimeError(f"IsaacSensorCreateImuSensor failed for {imu_path}")

    return robot, cam, cam_path, imu_path


def _orient_camera_to_ros(cam) -> None:
    """Apply a 180° local rotation about X so the camera's USD axes align
    with ROS optical-frame convention (Z forward, X right, Y down).

    USD Camera defaults to looking toward -Z with +Y up. Rotating 180°
    about X flips both Y and Z without disturbing X, giving us Z forward
    and Y down at the optical frame.
    """
    import numpy as np

    # (w, x, y, z) — Isaac Sim quaternion convention.
    q_x180 = np.array([0.0, 1.0, 0.0, 0.0])
    cam.set_local_pose(orientation=q_x180)


def _drive_velocities(name: str, mode: str, n_dof: int):
    """Return an np.ndarray of length n_dof with velocity targets.

    XLeRobot DOFs are ordered with the holonomic base first:
        idx 0: root_x_axis_joint     (prismatic, m/s)
        idx 1: root_y_axis_joint     (prismatic, m/s)
        idx 2: root_z_rotation_joint (continuous, rad/s)
        idx 3..16: arm + head joints (locked by xlerobot.json overrides)

    `cmd_vel` mode returns zeros — the step loop overrides these from the
    OG SubscribeTwist outputs.
    """
    import numpy as np

    v = np.zeros(n_dof)
    if mode in ("still", "cmd_vel"):
        return v
    v[0] = 0.15  # forward
    if mode == "circle":
        v[2] = 0.10 if name == "robot_0" else -0.10
    return v


def _read_cmd_vel(robot_name: str):
    """Read the most recent Twist from `/World/<robot>/CmdVelGraph`.

    Returns (linear: np.ndarray[3], angular: np.ndarray[3]). Defaults to
    zeros before the first message lands.
    """
    import numpy as np
    import omni.graph.core as og

    base = f"/World/{robot_name}/CmdVelGraph/SubscribeTwist.outputs"
    try:
        lin = og.Controller.attribute(f"{base}:linearVelocity").get()
        ang = og.Controller.attribute(f"{base}:angularVelocity").get()
    except Exception:
        return np.zeros(3), np.zeros(3)
    return np.array([float(lin[0]), float(lin[1]), float(lin[2])]), np.array(
        [float(ang[0]), float(ang[1]), float(ang[2])]
    )


def main() -> int:
    import numpy as np
    import omni.timeline
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction

    sys.path.insert(0, str(SCRIPTS_DIR))
    from _ros2_graphs import build_robot_graphs

    # Local import so the build_warehouse module's stdlib-only top-level still
    # works for documentation introspection.
    from build_warehouse import SPAWN_POSITIONS  # noqa: E402

    emit(f"spawn_warehouse | GPU: {gpu_name()}")
    emit(f"  headless={args.headless}  drive={args.drive}")
    emit(f"  duration={'forever' if args.forever else args.duration}")

    d435, mpu = _load_sensor_configs()
    emit(f"  d435: {d435['width']}x{d435['height']}@{d435['fps']}Hz")
    emit(f"  mpu6050: {mpu['publish_rate_hz']}Hz publish, mount={mpu['mount_offset_m']}")

    xlerobot_usd = _ensure_xlerobot_usd()
    warehouse_usd = _ensure_warehouse_usd()
    emit(f"  xlerobot_usd={xlerobot_usd}")
    emit(f"  warehouse_usd={warehouse_usd}")

    World.clear_instance()
    omni.usd.get_context().new_stage()
    world = World(stage_units_in_meters=1.0)

    add_reference_to_stage(usd_path=str(warehouse_usd), prim_path="/World/Warehouse")

    robots: dict[str, tuple] = {}
    for name, pos in SPAWN_POSITIONS.items():
        robot, cam, cam_path, imu_path = _spawn_robot(
            world, name, pos, xlerobot_usd, d435, mpu
        )
        robots[name] = (robot, cam, cam_path, imu_path)
        emit(f"  spawned {name} at {pos}, cam={cam_path}, imu={imu_path}")

    world.reset()

    # Camera.initialize() must come after world.reset() so the underlying
    # render product is bindable.
    for name, (_, cam, _, _) in robots.items():
        cam.initialize()
        _orient_camera_to_ros(cam)

    # Build per-robot graphs.
    for name, (_, _, cam_path, imu_path) in robots.items():
        build_robot_graphs(
            robot_prim_path=f"/World/{name}",
            chassis_subpath="base_link",
            camera_prim_path=cam_path,
            namespace=name,
            imu_prim_path=imu_path,
            rgb_resolution=(d435["width"], d435["height"]),
        )
        emit(f"  graphs built for {name}")

    # Global /clock publisher so external ROS 2 nodes can `use_sim_time:=true`.
    # Bridges sim time → /clock at every playback tick.
    import omni.graph.core as og  # noqa: E402

    og.Controller.edit(
        {"graph_path": "/World/ClockGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )
    emit("  /clock publisher built")

    # Let the bridge finish registering OG node types — first edit can race.
    for _ in range(3):
        simulation_app.update()

    # OnPlaybackTick fires only when the timeline is playing.
    omni.timeline.get_timeline_interface().play()
    emit("  timeline.play()")

    # Cache DOF counts post-reset.
    dof_counts = {name: len(robot.dof_names) for name, (robot, _, _, _) in robots.items()}
    emit(f"  dof_counts={dof_counts}")

    dt = 1.0 / 60.0
    n_steps = None if args.forever else int(args.duration / dt)
    step = 0
    while simulation_app.is_running():
        if n_steps is not None and step >= n_steps:
            break
        for name, (robot, _, _, _) in robots.items():
            v = _drive_velocities(name, args.drive, dof_counts[name])
            if args.drive == "cmd_vel":
                lin, ang = _read_cmd_vel(name)
                v[0] = lin[0]   # forward (m/s) — root_x_axis_joint
                v[1] = lin[1]   # strafe  (m/s) — root_y_axis_joint
                v[2] = ang[2]   # yaw    (rad/s) — root_z_rotation_joint
            robot.apply_action(ArticulationAction(joint_velocities=v))
        world.step(render=True)
        step += 1
        if step % 60 == 0:
            elapsed = step * dt
            r0_q = robots["robot_0"][0].get_joint_positions()
            r1_q = robots["robot_1"][0].get_joint_positions()
            emit(
                f"  t={elapsed:5.1f}s  "
                f"r0 x={r0_q[0]:+.3f} yaw={r0_q[2]:+.3f}  "
                f"r1 x={r1_q[0]:+.3f} yaw={r1_q[2]:+.3f}"
            )

    emit(f"spawn_warehouse: ran {step} steps ({step * dt:.1f} sim seconds)")
    emit("spawn_warehouse: OK")
    return 0


try:
    rc = main()
except Exception as e:
    print(f"ERROR: {type(e).__name__}: {e}", file=sys.stderr)
    import traceback

    traceback.print_exc()
    rc = 1
finally:
    if not args.forever:
        cancel_timeout()
    simulation_app.close()

sys.exit(rc)
