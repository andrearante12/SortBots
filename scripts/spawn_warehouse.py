#!/usr/bin/env python3
"""Open the warehouse, spawn 2 XLeRobot instances, publish per-robot ROS 2.

End state (one terminal):
    source scripts/activate_isaac.sh
    python scripts/spawn_warehouse.py --duration 30

Both robots curve gently in the warehouse, each one publishing:

    /<robot>/odom              nav_msgs/Odometry
    /<robot>/camera/rgb        sensor_msgs/Image
    /<robot>/camera/depth      sensor_msgs/Image (32FC1, meters)
    /<robot>/camera/chase/rgb   sensor_msgs/Image (3rd-person chase cam; first N robots)
    /<robot>/camera/chase/depth sensor_msgs/Image (32FC1, meters; first N robots)
    /<robot>/tf                tf2_msgs/TFMessage  (full articulation tree)

and subscribing:

    /<robot>/cmd_vel           geometry_msgs/Twist  (--drive cmd_vel; holonomic x/y/yaw)
    /<robot>/pick_cmd          std_msgs/String      ("attach"/"detach"; robot_0 only,
                                                      see nodes/task_manager.py)
    /<robot>/head_cmd          sensor_msgs/JointState (head_pan_joint/head_tilt_joint
                                                      position targets, radians; robot_0 only)

pick_cmd/head_cmd are plain rclpy subscribers, not OmniGraph — see
`_start_extra_ros_subscribers`. `<robot>` ∈ {robot_0, robot_1}. From a
separate shell with ROS 2 Jazzy
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

# NVIDIA's fully-assetted warehouse, streamed from the Isaac asset server
# (resolved via get_assets_root_path() after SimulationApp boots). The
# `full_warehouse` variant is stocked shelves + props; `warehouse` is the
# bare shell fallback if the full variant can't be reached.
NVIDIA_WAREHOUSE_REL = "Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
NVIDIA_WAREHOUSE_FALLBACK_REL = "Isaac/Environments/Simple_Warehouse/warehouse.usd"

D435_CONFIG = REPO_ROOT / "configs" / "sensors" / "d435.json"
MPU6050_CONFIG = REPO_ROOT / "configs" / "sensors" / "mpu6050.json"
WAYPOINTS_CONFIG = REPO_ROOT / "configs" / "waypoints.yaml"
ROBOTS_CONFIG = REPO_ROOT / "configs" / "robots.yaml"
RESULT_FILE = os.environ.get(
    "ISAAC_SPAWN_WAREHOUSE_RESULT", "/tmp/isaac_spawn_warehouse_result.txt"
)

# Milestone 1 mock pick: the URDF link name of the first arm's gripper tip
# (see third_party/XLeRobot/simulation/Maniskill/assets/xlerobot/xlerobot.urdf,
# `Fixed_Jaw_tip`). Isaac's URDF importer parents links as flat children of
# the articulation root (matches head_camera/imu path conventions above), so
# this is `/World/<robot>/Fixed_Jaw_tip`. Single-robot (robot_0) scope only.
GRIPPER_LINK = "Fixed_Jaw_tip"

PACKAGE_SIZE_M = 0.08

# Height of the arm's shoulder (the `Rotation` joint) above base_link, summed
# straight out of xlerobot.urdf: arm_base_joint z=+0.7600 then Rotation
# z=+0.0165. The arm is mounted on a mast, NOT at chassis height — which is the
# single most important number for placing anything the robot has to pick from.
#
# Everything reachable lives in a sphere of roughly 0.40-0.50 m around this
# point, so a pick surface wants to be somewhat BELOW it (reach forward and
# slightly down), not near the floor. A deck at 0.30 m asks the arm to reach
# half a metre straight down, which is the edge of its envelope pointing the
# wrong way.
ARM_SHOULDER_Z_ABOVE_BASE_M = 0.7765

# First arm's 6 non-fixed joints. `nodes/scripted_pick.py` commands these
# by name. DOF INDICES ARE NOT HARDCODED — Isaac's actual articulation DOF
# order does NOT follow xlerobot.urdf's declaration order (verified live:
# it interleaves arm1/arm2 by joint type — Rotation, Rotation_2, head_pan,
# Pitch, Pitch_2, head_tilt, Elbow, ... — not arm1-then-arm2-then-head as
# the raw URDF text would suggest). A prior version of this file hardcoded
# `ARM_JOINT_INDICES = [3,4,5,6,7,8]` and `HEAD_JOINT_INDICES = [15,16]`
# from the URDF's file order; [15,16] turned out to be Jaw/Jaw_2 (the
# GRIPPER), not the head, which is why head_cmd silently did nothing —
# no error, it was just moving the wrong joints. See `_dof_indices` below;
# indices are looked up from `robot.dof_names` at runtime instead.
ARM_JOINT_NAMES = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]
HEAD_JOINT_NAMES = ["head_pan_joint", "head_tilt_joint"]
HEAD_JOINT_LIMITS = {"head_pan_joint": (-1.57, 1.57), "head_tilt_joint": (-0.76, 1.45)}

# From xlerobot.urdf. head_cmd has always been clipped against its limits
# below, but arm commands went straight into ArticulationAction unclamped —
# so the dashboard's arm pad was the ONLY thing enforcing them, and a UI bug
# could drive a joint past its stop with nothing reporting it. Clamp here too.
ARM_JOINT_LIMITS = {
    "Rotation": (-2.1, 2.1),
    "Pitch": (-0.1, 3.45),
    "Elbow": (-0.2, 3.14159),
    "Wrist_Pitch": (-1.8, 1.8),
    "Wrist_Roll": (-3.14159, 3.14159),
    "Jaw": (0.0, 1.7),
}


def _dof_indices(dof_names: list[str], joint_names: list[str]) -> list[int]:
    """Map joint names to their actual DOF index in this articulation.

    Isaac's URDF import order is importer-specific, not the URDF file's
    declaration order — don't hardcode indices, look them up. See the
    ARM_JOINT_NAMES comment above for how this bit us once already.
    """
    return [dof_names.index(n) for n in joint_names]


def _load_sensor_configs():
    import json

    with open(D435_CONFIG) as f:
        d435 = json.load(f)
    with open(MPU6050_CONFIG) as f:
        mpu = json.load(f)
    return d435, mpu


def _load_station_props():
    """Stations in waypoints.yaml that carry a `prop:` block.

    Returns [(name, kind, x, y, yaw, prop), ...] in file order.

    Replaces the old `_load_first_pickup_station()`, whose (x, y)-only return
    couldn't carry the deck height the package z is now derived from. Extra
    keys in waypoints.yaml are inert everywhere else: task_manager.Station
    reads only kind/pose/dock_offset_m/dock_lateral_m, and the dashboard reads
    only `kind`.
    """
    import yaml

    with open(WAYPOINTS_CONFIG) as f:
        stations = yaml.safe_load(f)["stations"]
    out = []
    for name, spec in stations.items():
        prop = spec.get("prop")
        if not prop:
            continue
        pose = spec["pose"]
        out.append((
            name,
            spec["kind"],
            float(pose["x"]),
            float(pose["y"]),
            float(pose.get("yaw", 0.0)),
            prop,
        ))
    return out


def _load_spawn_positions(scene: str) -> dict[str, tuple[float, float, float]]:
    """Fleet roster spawn poses for `scene`, in configs/robots.yaml order.

    configs/robots.yaml is the single source of truth for robot ids + spawn
    poses — also read by build_warehouse.py (primitive-scene authoring) and
    launch/sortbots_bringup.launch.py (static map -> <id>/map anchors, which
    must match this pose for RTAB-Map's per-robot frame to align with the
    merged /map).
    """
    import yaml

    with open(ROBOTS_CONFIG) as f:
        roster = yaml.safe_load(f)
    return {r["id"]: tuple(r["spawn"][scene]) for r in roster["robots"]}


def _roster_size() -> int:
    import yaml

    with open(ROBOTS_CONFIG) as f:
        return len(yaml.safe_load(f)["robots"])


def _package_z(deck_height_m: float) -> float:
    """Package centre z so its bottom face rests on a deck at `deck_height_m`.

    UsdGeom.Cube is origin-centred, so a size-S cube sitting on a surface at
    height H has its centre at H + S/2. Derived rather than stored as a second
    constant on purpose: the previous PACKAGE_HEIGHT_M = 0.85 literal had no
    relationship to any geometry in the scene, and nothing would have told us
    when it drifted.
    """
    return deck_height_m + PACKAGE_SIZE_M / 2.0


def parse_args() -> argparse.Namespace:
    max_robots = _roster_size()
    p = argparse.ArgumentParser(
        description=(
            f"Spawn up to {max_robots} XLeRobots (configs/robots.yaml) in the "
            "warehouse and publish per-robot ROS 2 topics."
        )
    )
    p.add_argument("--headless", dest="headless", action="store_true")
    p.add_argument("--no-headless", dest="headless", action="store_false")
    p.set_defaults(headless=True)
    p.add_argument(
        "--scene",
        choices=("nvidia", "primitive"),
        default="nvidia",
        help=(
            "Which warehouse to load. `nvidia` (default) streams NVIDIA's "
            "fully-assetted Simple_Warehouse from the Isaac asset server; "
            "`primitive` uses the hand-built scenes/warehouse_v0.usd."
        ),
    )
    p.add_argument(
        "--warehouse-url",
        default=None,
        help=(
            "Override the warehouse USD path/URL outright (skips asset-server "
            "resolution). Useful for an offline Nucleus mirror."
        ),
    )
    p.add_argument(
        "--robots",
        type=int,
        default=min(2, max_robots),
        choices=tuple(range(1, max_robots + 1)),
        help=f"How many XLeRobots to spawn, in configs/robots.yaml order "
        f"(default {min(2, max_robots)}, max {max_robots}). Use 1 to save "
        "VRAM for the single-robot teleop + RTAB-Map demo.",
    )
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
    p.add_argument(
        "--no-station-props",
        dest="station_props",
        action="store_false",
        help=(
            "Skip the shelves authored at every configs/waypoints.yaml station "
            "carrying a `prop:` block. Escape hatch for when a shelf lands "
            "inside NVIDIA warehouse geometry — the station poses are estimates "
            "and have never been confirmed against the real rack layout. The "
            "package then spawns on the floor at the first pickup pose instead."
        ),
    )
    p.add_argument(
        "--no-chase-cam",
        action="store_true",
        help="Shorthand for --chase-cam-robots 0.",
    )
    p.add_argument(
        "--chase-cam-robots",
        type=int,
        default=None,
        metavar="N",
        help=(
            "How many robots (first N in configs/robots.yaml order) get a "
            "3rd-person chase camera. Purely cosmetic (the dashboard's main/"
            "PiP view) but each one costs a second render product per robot "
            "on top of its own head camera — useful for isolating "
            "render-product problems, or on a GPU that's already saturated. "
            "Default: every spawned robot (matches --robots)."
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
    # The NVIDIA warehouse streams ~hundreds of MB from the asset server on
    # the first run (cached locally afterward), so give it a much higher
    # floor than the primitive scene's local load.
    _timeout_floor = 600 if args.scene == "nvidia" else 240
    install_timeout(
        seconds=max(int(args.duration * 6), _timeout_floor),
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


def _resolve_nvidia_warehouse() -> str:
    """Resolve NVIDIA's Simple_Warehouse USD URL on the Isaac asset server.

    Must be called AFTER `SimulationApp` constructs (needs the Kit runtime).
    Prefers the stocked `full_warehouse`; falls back to the bare `warehouse`
    shell if the full variant can't be reached. Raises with an actionable
    message when the asset root itself is unreachable (no network / no local
    Nucleus) so the failure isn't a silent llvmpipe-style hang.
    """
    import omni.client
    from isaacsim.storage.native import get_assets_root_path

    root = get_assets_root_path()
    if not root:
        raise RuntimeError(
            "Isaac asset root is unreachable — get_assets_root_path() returned "
            "None. The NVIDIA warehouse streams from NVIDIA's S3/Nucleus server, "
            "so this machine needs network access (or a local Omniverse Nucleus "
            "mount). Re-run with `--scene primitive` to use the local "
            "scenes/warehouse_v0.usd instead."
        )
    for rel in (NVIDIA_WAREHOUSE_REL, NVIDIA_WAREHOUSE_FALLBACK_REL):
        url = f"{root}/{rel}"
        res, _ = omni.client.stat(url)
        if str(res) == "Result.OK":
            emit(f"  resolved NVIDIA warehouse: {url}")
            return url
        emit(f"  NVIDIA warehouse not reachable ({res}): {url}")
    raise RuntimeError(
        f"No Simple_Warehouse variant reachable under {root}. "
        "Re-run with `--scene primitive`."
    )


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


def _base_to_prim_transform(stage, base_prim_path: str, child_prim_path: str):
    """Fixed transform of `child` expressed in `base`'s frame.

    Returns ((tx, ty, tz), (qx, qy, qz, qw)) — translation + IJKR quaternion,
    the form ROS2PublishRawTransformTree wants. Used to publish
    base_link -> camera_optical so the prefixed frame the camera images are
    stamped with actually exists in the TF tree.
    """
    from pxr import Gf, UsdGeom

    xc = UsdGeom.XformCache()
    base_w = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(base_prim_path))
    child_w = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(child_prim_path))
    # USD is row-vector (p_world = p_local * L2W), so child-in-base = child_w * base_w^-1.
    rel = child_w * base_w.GetInverse()
    t = rel.ExtractTranslation()
    q = rel.ExtractRotationQuat()
    img = q.GetImaginary()
    return (
        (float(t[0]), float(t[1]), float(t[2])),
        (float(img[0]), float(img[1]), float(img[2]), float(q.GetReal())),
    )


def _orient_camera_to_ros(cam) -> None:
    """Apply a local rotation so the camera's USD axes align with ROS
    optical-frame convention (Z forward, X right, Y down).

    USD Camera defaults to looking toward -Z with +Y up. A pure 180° flip
    about X gives Z-forward/Y-down IF the parent frame's own axes are
    already exactly ROS-optical (which `head_camera_rgb_optical_frame`
    should be — the URDF's `head_camera_rgb_optical_joint` already applies
    the standard camera_link->optical_frame rpy).

    `camera_axes` MUST be "usd" here. `Camera.set_local_pose`'s default
    (`camera_axes="world"`, i.e. omitting the kwarg entirely) silently
    runs the supplied quaternion through a hidden `W_U_TRANSFORM`
    conversion before applying it — found by reading
    isaacsim.sensors.camera.camera.py's source after two live tests with
    that default produced wrong results in different ways (one rolled
    ~90°, one pointed straight up), neither matching what the raw
    quaternion math below predicts. `camera_axes="usd"` is the only mode
    that applies the quaternion as a plain local-to-parent rotation with
    no hidden conversion, matching this function's derivation.
    """
    import numpy as np

    # (w, x, y, z). 180° about X: right(+X) unchanged, up(+Y)->down,
    # forward(-Z)->+Z — converts USD's (-Z fwd, +Y up) default to ROS
    # optical (+Z fwd, +Y down) when the parent is already ROS-optical.
    q_x180 = np.array([0.0, 1.0, 0.0, 0.0])
    cam.set_local_pose(orientation=q_x180, camera_axes="usd")


CHASE_CAM_OFFSET_M = (-2.2, 0.0, 1.5)  # behind + above, in base_link's local frame
# Orientation quaternion (w,x,y,z), camera_axes="usd" (see _orient_camera_to_ros
# for why "usd" and not the misleading default). base_link's axes (X
# forward matching root_x_axis_joint's forward-drive convention, Y left
# per REP-103, Z up) verified live across three iterations: level (robot
# only a sliver at the bottom edge) -> 25° tilt at 1.3m/1.1m (whole cart
# visible but the head/camera-mast cropped off the top) -> this one,
# pulled back further (2.2m) and higher (1.5m) with a gentler 16° tilt so
# the taller head mast fits in frame too. Verified by explicit
# rotation-matrix construction + reconstruction + orthonormality check
# (not hand-waved) each time; only the distance/tilt magnitude needed
# empirical tuning against a live snapshot, not the axis mapping itself.
CHASE_CAM_ORIENTATION = (0.5647, 0.4255, -0.4255, -0.5647)


def _spawn_chase_camera(robot_prim_path: str, d435: dict):
    """3rd-person 'chase cam' rigidly mounted behind+above base_link.

    Unlike the head camera (a pre-existing URDF-authored prim,
    `head_camera_rgb_optical_frame`, whose parent orientation already does
    most of the work), this is a brand new child prim under `base_link`
    with no inherited pose — both translation AND orientation must be set
    explicitly, or it defaults to sitting at base_link's own origin
    looking along base_link's local -Z.
    """
    from isaacsim.sensors.camera import Camera
    from pxr import Gf, UsdGeom
    import omni.usd

    cam_path = f"{robot_prim_path}/base_link/chase_cam"
    cam = Camera(
        prim_path=cam_path,
        resolution=(d435["width"], d435["height"]),
        frequency=d435["fps"],
    )

    stage = omni.usd.get_context().get_stage()
    cam_prim = stage.GetPrimAtPath(cam_path)
    ucam = UsdGeom.Camera(cam_prim)
    ucam.CreateFocalLengthAttr(float(d435["focal_length_mm"]))
    ucam.CreateHorizontalApertureAttr(float(d435["horizontal_aperture_mm"]))
    ucam.CreateVerticalApertureAttr(float(d435["vertical_aperture_mm"]))
    ucam.CreateClippingRangeAttr(
        Gf.Vec2f(*[float(v) for v in d435["clipping_range_m"]])
    )
    return cam, cam_path


def _orient_chase_camera(cam) -> None:
    import numpy as np

    cam.set_local_pose(
        translation=np.array(CHASE_CAM_OFFSET_M),
        orientation=np.array(CHASE_CAM_ORIENTATION),
        camera_axes="usd",
    )


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


def _start_extra_ros_subscribers(robot_name: str):
    """Plain rclpy subscribers for topics with no working OmniGraph node,
    or where a rock-solid tested path was worth more than another
    speculative OmniGraph attribute name (see below).

    `/<robot_name>/pick_cmd` (std_msgs/String) is NOT an OmniGraph node:
    verified live against Isaac Sim 5.1 (isaacsim.ros2.bridge 4.12.4) that
    `ROS2SubscribeString` doesn't exist in this build (unlike
    `ROS2SubscribeTwist`/`ROS2SubscribeJointState`, both confirmed present
    in the bridge extension's own test suite).

    `/<robot_name>/head_cmd` (sensor_msgs/JointState: head_pan_joint,
    head_tilt_joint) COULD go through the same ArmCmdGraph/ROS2SubscribeJointState
    mechanism as arm_joint_cmd, but that graph's exact output attribute
    names (`outputs:jointNames`/`outputs:positionCommand`) are still
    unverified against a live session (only confirmed the *node type*
    exists, not `_read_arm_cmd`'s attribute names) — added here instead to
    reuse the pick_cmd path already proven end-to-end live.

    Isaac's Python process already has `rclpy` loaded for the bridge
    extension's own internal use, so plain subscribers work without a
    separate ROS 2 install inside the Isaac venv — just poll them each
    tick the same way the OmniGraph-attribute reads elsewhere in this file
    are polled.

    Returns (node, pick_state, head_state):
      - pick_state["data"]: last pick_cmd string, "" before the first one.
      - pick_state["seq"]: message counter, so the caller can distinguish a
        newly-arrived "detach" from one it already acted on.
      - head_state["pan"]/["tilt"]: last head_cmd values, 0.0 before the
        first one (matches the physics_overrides default of held-at-0).
    Call `rclpy.spin_once(node, timeout_sec=0.001)` once per sim tick — NOT
    `timeout_sec=0.0`; that's a known rclpy/rmw flakiness trap where a
    literal zero can skip actually polling the middleware depending on the
    executor, so pending messages don't reliably get processed. Verified
    live: `head_cmd` worked once right after this mechanism was first
    written, then silently stopped moving the head on a later restart with
    no code change in between — exactly the signature of relying on a
    zero timeout's undefined-ish polling behavior rather than a real bug
    in the callback logic itself.
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    try:
        rclpy.init()
    except RuntimeError:
        pass  # already initialized (e.g. by Isaac's own bridge internals)

    node = Node(f"{robot_name}_extra_ros_bridge")
    # `seq` makes the release edge-triggerable. "attach" wants to be level-
    # triggered (re-teleport the package every tick while held), but "detach"
    # is a one-shot event — the main loop fires the drop only when `seq`
    # changes, so it can't re-place the box every tick afterwards.
    pick_state = {"data": "", "seq": 0}
    head_state = {"pan": 0.0, "tilt": 0.0}

    def _on_pick_cmd(msg):
        pick_state["data"] = msg.data
        pick_state["seq"] += 1

    node.create_subscription(String, f"/{robot_name}/pick_cmd", _on_pick_cmd, 10)

    def _on_head_cmd(msg):
        by_name = dict(zip(msg.name, msg.position))
        if "head_pan_joint" in by_name:
            head_state["pan"] = float(by_name["head_pan_joint"])
        if "head_tilt_joint" in by_name:
            head_state["tilt"] = float(by_name["head_tilt_joint"])

    node.create_subscription(JointState, f"/{robot_name}/head_cmd", _on_head_cmd, 10)

    return node, pick_state, head_state


def _read_arm_cmd(robot_name: str):
    """Read the last JointState from `/World/<robot>/ArmCmdGraph`.

    Returns an np.ndarray of length `len(ARM_JOINT_NAMES)`, ordered to
    match `ARM_JOINT_NAMES`, or None before the first message lands / if
    the received `jointNames` don't match what we expect / on bridge error.
    """
    import numpy as np
    import omni.graph.core as og

    base = f"/World/{robot_name}/ArmCmdGraph/SubscribeJointState.outputs"
    try:
        names = list(og.Controller.attribute(f"{base}:jointNames").get())
        positions = list(og.Controller.attribute(f"{base}:positionCommand").get())
    except Exception:
        return None
    if not names or len(names) != len(positions):
        return None
    by_name = dict(zip(names, positions))
    if not all(n in by_name for n in ARM_JOINT_NAMES):
        return None
    # Clamp: nothing else on this path enforces the URDF limits (see
    # ARM_JOINT_LIMITS). A teleop pad is exactly the thing that will one day
    # send a joint past its stop.
    return np.array([
        float(np.clip(by_name[n], *ARM_JOINT_LIMITS[n])) for n in ARM_JOINT_NAMES
    ])


def _station_materials(stage):
    """OmniPBR materials for the hand-built station props, made once.

    Reuses the texture pipeline build_warehouse already has (the same
    assets/textures/*.jpg the primitive scene uses). Without this the shelves
    render flat white: _author_box makes UV-mapped geometry but binds nothing,
    and UsdPreviewSurface doesn't render textures in Kit's RTX path anyway —
    OmniPBR is the only thing that does here.

    Returns {} if the textures aren't present (fetch_warehouse_textures.py was
    never run), so a missing texture downgrades to untextured rather than
    killing the spawn.
    """
    from build_warehouse import TEXTURE_ROLES, TEXTURES_DIR, _make_texture_material

    from pxr import UsdGeom

    materials = {}
    UsdGeom.Scope.Define(stage, "/World/StationLooks")
    for role in ("shelves", "boxes"):
        tex = TEXTURES_DIR / TEXTURE_ROLES[role]
        if not tex.is_file():
            continue
        materials[role] = _make_texture_material(
            stage, f"/World/StationLooks/{role}", tex
        )
    return materials


def _spawn_shelf(stage, prim_path: str, x: float, y: float, yaw_rad: float,
                 deck_height_m: float, size, materials=None):
    """A deck slab on four legs, as static colliders, centred on (x, y).

    The NVIDIA warehouse has nothing this arm can pick from at a sane height,
    and editing a streamed asset reference is not something we want to own. So
    the demo brings its own shelf and places it at the station pose. See
    `type: usd` in configs/waypoints.yaml for referencing a real NVIDIA prop
    instead; this stays the offline/no-network fallback and is what the
    primitive scene needs.

    Reuses build_warehouse._author_box, which authors a UV-mapped Mesh rather
    than a UsdGeom.Cube — Kit's RTX delegate won't generate UVs for a Cube, so
    Cubes render flat grey. Static: CollisionAPI only, no RigidBodyAPI, so it
    never moves and costs the solver nothing.
    """
    import math

    from build_warehouse import _author_box, _bind

    yaw_deg = math.degrees(yaw_rad)
    sx, sy, sz = (float(v) for v in size)
    materials = materials or {}

    deck = _author_box(
        stage, f"{prim_path}/Deck", (sx, sy, sz),
        (x, y, deck_height_m - sz / 2.0), rotate_xyz=(0.0, 0.0, yaw_deg),
    )
    if "shelves" in materials:
        _bind(deck, materials["shelves"])

    leg = 0.05
    leg_h = max(deck_height_m - sz, 0.01)
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    inset_x = sx / 2.0 - leg
    inset_y = sy / 2.0 - leg
    for i, (ox, oy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        # Leg offsets are in the shelf's own frame, rotated into world by yaw.
        lx = x + (ox * inset_x) * cos_y - (oy * inset_y) * sin_y
        ly = y + (ox * inset_x) * sin_y + (oy * inset_y) * cos_y
        leg_prim = _author_box(
            stage, f"{prim_path}/Leg_{i}", (leg, leg, leg_h),
            (lx, ly, leg_h / 2.0), rotate_xyz=(0.0, 0.0, yaw_deg),
        )
        if "shelves" in materials:
            _bind(leg_prim, materials["shelves"])


def _spawn_usd_prop(stage, prim_path: str, x: float, y: float, yaw_rad: float,
                    ref: str, scale: float = 1.0, z: float = 0.0) -> bool:
    """Reference a real USD asset (an NVIDIA warehouse prop) at the station.

    `ref` is either a path relative to the Isaac asset root — e.g.
    `Isaac/Props/KLT_Bin/small_KLT_visual_collision.usd` — or an absolute
    URL/filesystem path, which is passed through untouched. Relative refs
    resolve through the same get_assets_root_path() the warehouse itself uses,
    so they stream and cache exactly like full_warehouse.usd and add no new
    dependency beyond what `--scene nvidia` already needs.

    Assets worth knowing about (verified present on the 5.0 asset server, and
    whether they ship colliders):

      Isaac/Props/KLT_Bin/small_KLT_visual_collision.usd   collider  tote/bin
      Isaac/Environments/Simple_Warehouse/Props/SM_RackShelf_01.usd
                                                          collider  rack shelf
      Isaac/Props/PackingTable/packing_table.usd           collider  (14 MB)
      Isaac/Props/Pallet/pallet.usd                                  pallet
      Isaac/Props/Blocks/red_block.usd                    NO physics graspable
      Isaac/Props/YCB/Axis_Aligned/061_foam_brick.usd                graspable

    HEIGHTS ARE NOT VERIFIED. These were authored for human-scale warehouses
    and Franka-class arms; this arm's shoulder is at ~0.83 m above the floor
    with a ~0.45 m envelope around it. Check what a prop actually looks like
    next to the robot before trusting it — `deck_height_m` in waypoints.yaml
    stays the pick height regardless of what the asset's own geometry does,
    so a mismatched prop is a cosmetic problem, not a silent wrong-pose one.

    Returns False if the asset root can't be resolved, so the caller can fall
    back to the hand-built shelf rather than spawning nothing.
    """
    import math

    from isaacsim.core.utils.stage import add_reference_to_stage
    from pxr import Gf, UsdGeom

    if "://" not in ref and not ref.startswith("/"):
        try:
            from isaacsim.storage.native import get_assets_root_path

            root = get_assets_root_path()
        except Exception:
            root = None
        if not root:
            return False
        ref = f"{root.rstrip('/')}/{ref.lstrip('/')}"

    add_reference_to_stage(usd_path=ref, prim_path=prim_path)
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), float(z)))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(math.degrees(yaw_rad))))
    if scale != 1.0:
        xform.AddScaleOp().Set(Gf.Vec3f(float(scale), float(scale), float(scale)))
    return True


def _spawn_package(stage, prim_path: str, x: float, y: float, z: float):
    """A free Cube the mock pick teleports onto the gripper.

    No rigid-body physics: the box only has to visibly ride the gripper and
    stay put once released. `_place_package_on_surface` handles the release.
    Real grasp physics is the successor to all of this — see that function.
    """
    from pxr import Gf, UsdGeom

    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.CreateSizeAttr(PACKAGE_SIZE_M)
    xformable = UsdGeom.Xformable(cube)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(x, y, z))


def _sync_package_to_gripper(stage, robot_name: str, package_prim_path: str) -> bool:
    """Teleport the package prim onto the gripper tip's current world pose.

    Returns False (no-op) if the gripper link isn't found — e.g. `GRIPPER_LINK`
    doesn't match the actual imported prim name, which nobody has confirmed
    against a live session yet (see the `Fixed_Jaw_tip` comment above).
    """
    from pxr import Usd, UsdGeom

    gripper_prim = stage.GetPrimAtPath(f"/World/{robot_name}/{GRIPPER_LINK}")
    if not gripper_prim.IsValid():
        return False
    world_xform = UsdGeom.Xformable(gripper_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    package_prim = stage.GetPrimAtPath(package_prim_path)
    xformable = UsdGeom.Xformable(package_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(world_xform)
    return True


def _place_package_on_surface(stage, package_prim_path: str, decks) -> bool:
    """Drop the package onto whatever it's currently over. Called on release.

    Without this the box freezes in mid-air wherever the gripper let go — the
    single most obviously-fake moment in the demo. `decks` is
    [(x, y, half_sx, half_sy, deck_z), ...] from the station props; if the
    package's XY is over one it lands on that deck, otherwise on the floor.

    Station-agnostic on purpose: it works for any dropoff, and for a release
    part-way through a run, without spawn_warehouse needing to know which
    station task_manager navigated to (it deliberately doesn't).

    Authors a Translate, not a Transform, so the wrist rotation the gripper
    baked into the package's transform is dropped and the box lands square.

    Still a teleport, same honesty caveat as the attach side. The successor is
    real physics: give the package RigidBodyAPI + CollisionAPI, keep
    physics:kinematicEnabled true while held and clear it on release so it
    actually falls. That is deliberately NOT on the demo path — it depends on
    the shelf colliders being right, on the gripper not interpenetrating, and
    on the Jaw drive, which at stiffness 100 / max_force 100 (vs the arm's
    1000/1000, see configs/physics_overrides/xlerobot.json) cannot hold
    anything at all.
    """
    from pxr import Gf, Usd, UsdGeom

    package_prim = stage.GetPrimAtPath(package_prim_path)
    if not package_prim.IsValid():
        return False
    xformable = UsdGeom.Xformable(package_prim)
    world_xform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = world_xform.ExtractTranslation()

    rest_z = PACKAGE_SIZE_M / 2.0  # on the floor
    for dx, dy, half_sx, half_sy, deck_z in decks:
        if abs(pos[0] - dx) <= half_sx and abs(pos[1] - dy) <= half_sy:
            rest_z = _package_z(deck_z)
            break

    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), rest_z))
    return True


def main() -> int:
    import numpy as np
    import omni.timeline
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction

    sys.path.insert(0, str(SCRIPTS_DIR))
    from _ros2_graphs import build_chase_camera_graph, build_robot_graphs

    emit(f"spawn_warehouse | GPU: {gpu_name()}")
    emit(f"  scene={args.scene}  robots={args.robots}")
    emit(f"  headless={args.headless}  drive={args.drive}")
    emit(f"  duration={'forever' if args.forever else args.duration}")

    d435, mpu = _load_sensor_configs()
    emit(f"  d435: {d435['width']}x{d435['height']}@{d435['fps']}Hz")
    emit(f"  mpu6050: {mpu['publish_rate_hz']}Hz publish, mount={mpu['mount_offset_m']}")

    xlerobot_usd = _ensure_xlerobot_usd()
    emit(f"  xlerobot_usd={xlerobot_usd}")

    # Pick the warehouse + matching spawn-point table for the chosen scene.
    if args.warehouse_url:
        warehouse_ref = args.warehouse_url
        spawn_positions = _load_spawn_positions("nvidia")
        emit(f"  warehouse_ref={warehouse_ref} (override)")
    elif args.scene == "nvidia":
        warehouse_ref = _resolve_nvidia_warehouse()
        spawn_positions = _load_spawn_positions("nvidia")
    else:
        warehouse_ref = str(_ensure_warehouse_usd())
        spawn_positions = _load_spawn_positions("primitive")
        emit(f"  warehouse_ref={warehouse_ref}")

    # Spawn only the first `--robots` entries (single-robot teleop saves VRAM).
    spawn_positions = dict(list(spawn_positions.items())[: args.robots])

    World.clear_instance()
    omni.usd.get_context().new_stage()
    world = World(stage_units_in_meters=1.0)

    add_reference_to_stage(usd_path=warehouse_ref, prim_path="/World/Warehouse")

    robots: dict[str, tuple] = {}
    for name, pos in spawn_positions.items():
        robot, cam, cam_path, imu_path = _spawn_robot(
            world, name, pos, xlerobot_usd, d435, mpu
        )
        robots[name] = (robot, cam, cam_path, imu_path)
        emit(f"  spawned {name} at {pos}, cam={cam_path}, imu={imu_path}")

    # 3rd-person chase camera for the first N robots (configs/robots.yaml
    # order). Kept out of the `robots` dict tuple — that shape is unpacked at
    # several call sites already and a 5th element would need touching all of
    # them. A real GPU cost (a second render product per robot, on top of its
    # own head camera). --no-chase-cam is shorthand for N=0; default N is
    # every spawned robot. Fleet runs usually want N=1: the dashboard only
    # shows ROBOT_ID's chase feed, so robot_1's chase cam is pure waste.
    if args.no_chase_cam:
        n_chase = 0
    elif args.chase_cam_robots is None:
        n_chase = len(robots)
    else:
        n_chase = max(0, min(args.chase_cam_robots, len(robots)))
    chase_names = list(robots)[:n_chase]
    chase_cams: dict[str, tuple] = {}  # name -> (Camera, prim_path)
    if not chase_names:
        reason = "--no-chase-cam" if args.no_chase_cam else f"--chase-cam-robots {n_chase}"
        emit(f"  chase_cam SKIPPED ({reason})")
    else:
        for name in chase_names:
            cam, cam_path = _spawn_chase_camera(f"/World/{name}", d435)
            chase_cams[name] = (cam, cam_path)
            emit(f"  chase_cam spawned for {name} at {cam_path}")
        skipped = [n for n in robots if n not in chase_cams]
        if skipped:
            emit(f"  chase_cam skipped for {', '.join(skipped)} (--chase-cam-robots {n_chase})")

    world.reset()

    # Camera.initialize() must come after world.reset() so the underlying
    # render product is bindable.
    for name, (_, cam, _, _) in robots.items():
        cam.initialize()
        _orient_camera_to_ros(cam)
    for name, (cam, _) in chase_cams.items():
        cam.initialize()
        _orient_chase_camera(cam)

    # IMU mount is authored relative to base_link from the MPU config, so the
    # base_link -> imu_link transform is exactly (offset, rotation). wxyz -> ijkr.
    imu_wxyz = [float(v) for v in mpu["mount_rotation_wxyz"]]
    base_to_imu = (
        tuple(float(v) for v in mpu["mount_offset_m"]),
        (imu_wxyz[1], imu_wxyz[2], imu_wxyz[3], imu_wxyz[0]),
    )

    # Build per-robot graphs.
    stage = omni.usd.get_context().get_stage()
    for name, (_, _, cam_path, imu_path) in robots.items():
        # TF for the frame the images are stamped with must be the ROS-optical
        # parent prim (head_camera_rgb_optical_frame), NOT the cam prim inside
        # it: _orient_camera_to_ros gives the cam prim an extra 180°-about-X so
        # its USD axes (-Z fwd, +Y up) render correctly, but depth consumers
        # (RTAB-Map) reproject assuming ROS optical axes (+Z fwd, +Y down).
        # Publishing the cam prim's pose here baked that 180° flip into TF and
        # mirrored the whole SLAM cloud about the camera's horizontal plane —
        # floor became a ceiling slab at z≈2.4, walls hung downward.
        optical_path = cam_path.rsplit("/", 1)[0]
        base_to_camera = _base_to_prim_transform(
            stage, f"/World/{name}/base_link", optical_path
        )
        emit(f"  {name} base->optical t={base_to_camera[0]} q={base_to_camera[1]}")
        build_robot_graphs(
            robot_prim_path=f"/World/{name}",
            chassis_subpath="base_link",
            camera_prim_path=cam_path,
            namespace=name,
            imu_prim_path=imu_path,
            rgb_resolution=(d435["width"], d435["height"]),
            base_to_camera=base_to_camera,
            base_to_imu=base_to_imu,
        )
        emit(f"  graphs built for {name}")

    for name, (_, cam_path) in chase_cams.items():
        build_chase_camera_graph(
            graph_path=f"/World/{name}/ChaseCameraGraph",
            camera_prim_path=cam_path,
            namespace=name,
            rgb_topic="camera/chase/rgb",
            depth_topic="camera/chase/depth",
            camera_info_topic="camera/chase/camera_info",
            width=d435["width"],
            height=d435["height"],
            frame_id=f"{name}/chase_camera_optical",
        )
        emit(f"  chase_cam graph built (/{name}/camera/chase/*)")

    # Station props + the mock pick package (robot_0 only). Every station in
    # configs/waypoints.yaml carrying a `prop:` block gets a shelf authored at
    # its pose, and the package starts resting on the first pickup station's
    # deck — so the scene the robot navigates to actually has something at a
    # reachable height to pick from. `nodes/task_manager.py` attaches/detaches
    # the package via a plain rclpy pick_cmd subscriber (see
    # _start_extra_ros_subscribers); the same node carries head_cmd.
    package_prim_path = None
    extra_ros_node, pick_cmd_state, head_cmd_state = None, None, None
    decks = []
    if "robot_0" in robots:
        props = _load_station_props() if args.station_props else []
        station_materials = _station_materials(stage) if props else {}
        for name, kind, sx, sy, syaw, prop in props:
            ptype = prop.get("type", "shelf")
            deck_h = float(prop["deck_height_m"])
            size = prop.get("size", [0.90, 0.40, 0.06])
            prim = f"/World/station_{name}"

            if ptype == "usd":
                ok = _spawn_usd_prop(
                    stage, prim, sx, sy, syaw,
                    ref=prop["ref"],
                    scale=float(prop.get("scale", 1.0)),
                    z=float(prop.get("z", 0.0)),
                )
                if ok:
                    emit(f"  usd prop for {name} ({kind}) at ({sx}, {sy}): {prop['ref']}")
                else:
                    # Asset root unreachable (offline / no Nucleus). Falling
                    # back keeps the pick geometry intact — the demo still
                    # works, it just looks like a white slab again.
                    emit(f"  usd prop for {name} unreachable, falling back to built-in shelf")
                    _spawn_shelf(stage, prim, sx, sy, syaw, deck_h, size, station_materials)
            elif ptype == "shelf":
                _spawn_shelf(stage, prim, sx, sy, syaw, deck_h, size, station_materials)
                emit(f"  shelf for {name} ({kind}) at ({sx}, {sy}) deck_z={deck_h}")
            else:
                emit(f"  station {name}: unknown prop type {ptype!r}, skipped")
                continue

            # deck_height_m is the pick height whatever the prop looks like,
            # so the release footprint is the same either way.
            decks.append((sx, sy, float(size[0]) / 2.0, float(size[1]) / 2.0, deck_h))

        pickup = next((p for p in props if p[1] == "pickup"), None)
        package_prim_path = "/World/package_0"
        if pickup is not None:
            _, _, pkg_x, pkg_y, _, pkg_prop = pickup
            pkg_z = _package_z(float(pkg_prop["deck_height_m"]))
        else:
            # No prop'd pickup station (e.g. --no-station-props): fall back to
            # the first pickup pose with the box on the floor, so the mock pick
            # still has something to grab.
            import yaml

            with open(WAYPOINTS_CONFIG) as f:
                stations = yaml.safe_load(f)["stations"]
            first = next(s for s in stations.values() if s["kind"] == "pickup")
            pkg_x, pkg_y = float(first["pose"]["x"]), float(first["pose"]["y"])
            pkg_z = PACKAGE_SIZE_M / 2.0
        _spawn_package(stage, package_prim_path, pkg_x, pkg_y, pkg_z)
        emit(f"  package_0 spawned at ({pkg_x}, {pkg_y}, {pkg_z:.3f})")

        extra_ros_node, pick_cmd_state, head_cmd_state = _start_extra_ros_subscribers("robot_0")
        emit("  pick_cmd/head_cmd rclpy subscriber started")

        # GRIPPER_LINK has never been confirmed against a live session, and
        # _sync_package_to_gripper fails silently when it's wrong. Dump the
        # articulation's children once so the real link names are in the log
        # forever, and say up front whether the one we want resolves.
        gripper_path = f"/World/robot_0/{GRIPPER_LINK}"
        if stage.GetPrimAtPath(gripper_path).IsValid():
            emit(f"  gripper link OK: {gripper_path}")
        else:
            names = [p.GetName() for p in stage.GetPrimAtPath("/World/robot_0").GetChildren()]
            emit(f"  WARNING: gripper link {gripper_path} NOT FOUND — the mock "
                 f"pick will silently do nothing. /World/robot_0 children: {names}")

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
    arm_joint_indices = None
    head_joint_indices = None
    if "robot_0" in robots:
        dof_names = robots["robot_0"][0].dof_names
        emit(f"  robot_0 dof_names={dof_names}")
        arm_joint_indices = _dof_indices(dof_names, ARM_JOINT_NAMES)
        head_joint_indices = _dof_indices(dof_names, HEAD_JOINT_NAMES)
        emit(f"  arm_joint_indices={arm_joint_indices} head_joint_indices={head_joint_indices}")

    dt = 1.0 / 60.0
    n_steps = None if args.forever else int(args.duration / dt)
    step = 0
    # Last pick_cmd message acted on, so "detach" fires once per message
    # rather than every tick (see the release block below).
    last_pick_seq = pick_cmd_state["seq"] if pick_cmd_state else 0
    warned_gripper = False
    while simulation_app.is_running():
        if n_steps is not None and step >= n_steps:
            break
        if extra_ros_node is not None:
            import rclpy

            # Spin BEFORE applying head_cmd this tick, not after — the
            # previous ordering (spin at the bottom of the loop) meant
            # every apply used the PREVIOUS tick's state, a permanent
            # one-tick lag. Also: timeout_sec=0.0 is a known rclpy/rmw
            # flakiness trap — a literal zero can skip actually polling
            # the middleware depending on the executor, so pending
            # messages don't reliably get processed. A tiny positive
            # timeout reliably triggers the check; see
            # reference_isaac_sim_quirks.md-style caveats elsewhere in
            # this file for the same "verify against a live session"
            # lesson.
            rclpy.spin_once(extra_ros_node, timeout_sec=0.001)
        for name, (robot, _, _, _) in robots.items():
            v = _drive_velocities(name, args.drive, dof_counts[name])
            if args.drive == "cmd_vel":
                lin, ang = _read_cmd_vel(name)
                # cmd_vel is BODY-frame (ROS convention), but root_x/root_y are
                # WORLD-frame prismatic joints: they sit BEFORE root_z_rotation in
                # the kinematic chain (root -> x -> y -> yaw -> base_link), so a raw
                # lin[0] drives along world +x regardless of heading — "forward"
                # ignores which way the robot faces. Rotate the body command into
                # the world frame by the current yaw (== root_z_rotation position,
                # DOF idx 2, the only rotation in the base chain) before applying.
                # Fixes manual teleop AND Nav2, which also emits body-frame cmd_vel.
                yaw = float(robot.get_joint_positions()[2])
                cos_y, sin_y = np.cos(yaw), np.sin(yaw)
                v[0] = cos_y * lin[0] - sin_y * lin[1]  # world vx -> root_x_axis_joint
                v[1] = sin_y * lin[0] + cos_y * lin[1]  # world vy -> root_y_axis_joint
                v[2] = ang[2]                           # yaw rate -> root_z_rotation_joint
            robot.apply_action(ArticulationAction(joint_velocities=v))
            # Arm/head joints are position-driven (physics_overrides/xlerobot.json),
            # so velocity commands above are no-ops for them; a second,
            # joint_indices-scoped action carries scripted_pick.py's targets.
            # joint_indices are looked up from the live dof_names, not
            # hardcoded — see _dof_indices' docstring for why.
            if name == "robot_0":
                arm_positions = _read_arm_cmd(name)
                if arm_positions is not None:
                    robot.apply_action(
                        ArticulationAction(
                            joint_positions=arm_positions,
                            joint_indices=np.array(arm_joint_indices),
                        )
                    )
                if extra_ros_node is not None:
                    pan_lo, pan_hi = HEAD_JOINT_LIMITS["head_pan_joint"]
                    tilt_lo, tilt_hi = HEAD_JOINT_LIMITS["head_tilt_joint"]
                    head_positions = np.array([
                        float(np.clip(head_cmd_state["pan"], pan_lo, pan_hi)),
                        float(np.clip(head_cmd_state["tilt"], tilt_lo, tilt_hi)),
                    ])
                    robot.apply_action(
                        ArticulationAction(
                            joint_positions=head_positions,
                            joint_indices=np.array(head_joint_indices),
                        )
                    )
        if package_prim_path is not None:
            # "attach" is level-triggered: state["data"] holds the last
            # message, so re-teleporting every tick is idempotent, not a
            # repeated trigger — that's what makes the box ride the gripper.
            if pick_cmd_state["data"] == "attach":
                if not _sync_package_to_gripper(stage, "robot_0", package_prim_path):
                    # Returns False when GRIPPER_LINK doesn't resolve. Logged
                    # once rather than every tick; the startup check above
                    # already warned, this catches a mid-run regression.
                    if not warned_gripper:
                        emit("  WARNING: _sync_package_to_gripper found no "
                             f"/World/robot_0/{GRIPPER_LINK} — package not following")
                        warned_gripper = True
            # "detach" is edge-triggered: releasing is a one-shot event, so it
            # fires only on a NEW message. Level-triggering it would re-drop
            # the box onto the deck every tick and pin it there.
            elif (pick_cmd_state["data"] == "detach"
                  and pick_cmd_state["seq"] != last_pick_seq):
                _place_package_on_surface(stage, package_prim_path, decks)
                emit("  package released")
            last_pick_seq = pick_cmd_state["seq"]
        world.step(render=True)
        step += 1
        if step % 60 == 0:
            elapsed = step * dt
            pose = " ".join(
                f"{n} x={robots[n][0].get_joint_positions()[0]:+.3f} "
                f"yaw={robots[n][0].get_joint_positions()[2]:+.3f}"
                for n in robots
            )
            debug_line = f"  t={elapsed:5.1f}s  {pose}"
            if head_cmd_state is not None:
                qpos = robots["robot_0"][0].get_joint_positions()
                pan_i, tilt_i = head_joint_indices
                debug_line += (
                    f"  head_cmd_state={head_cmd_state}"
                    f" actual[pan,tilt]=({qpos[pan_i]:+.3f},{qpos[tilt_i]:+.3f})"
                )
            emit(debug_line)

    if extra_ros_node is not None:
        extra_ros_node.destroy_node()  # not rclpy.shutdown() — Isaac's own bridge may share the context

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
