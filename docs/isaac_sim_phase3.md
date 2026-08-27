# Isaac Sim Phase 3 — warehouse + 2 XLeRobot + ROS 2 bridge

End state: `pytest -x tests/isaac/test_ros2_topics.py` spawns two XLeRobot
instances in a hand-built warehouse and confirms that each robot publishes
distinct ROS 2 topics over the bundled `isaacsim.ros2.bridge`:

```
/robot_0/odom              nav_msgs/Odometry
/robot_0/camera/rgb        sensor_msgs/Image
/robot_0/camera/depth      sensor_msgs/Image  (32FC1, meters)
/robot_0/tf                tf2_msgs/TFMessage  (full articulation tree)
/robot_1/...               same set under the robot_1 namespace
```

This is the first phase that actually exercises the "ROS 2 sim-to-real"
justification for adopting Isaac Sim. Phase 4 (cuVSLAM) consumes these
topics.

This page assumes [`quickstart.md`](quickstart.md) and
[`isaac_sim_phase2.md`](isaac_sim_phase2.md) are done.

## What the pieces do

- **`scripts/build_warehouse.py`** — pure `pxr.Usd` authoring; writes
  `scenes/warehouse_v0.usd` (floor, 4 walls, 4 shelves, 2 boxes, dome +
  distant lights, and named `/World/PickupZone` / `/World/DropoffZone` /
  `/World/SpawnPoints/robot_{0,1}` Xform prims). The output is gitignored
  — the script is the source of truth. Re-running with the same path is
  a no-op unless you pass `--force`.
- **`scripts/spawn_warehouse.py`** — references the warehouse + 2
  XLeRobot USDs onto a single Kit stage, attaches a Camera to each
  robot's head optical frame, builds three OmniGraph publishers per
  robot, and drives them forward. Defaults to `--headless`,
  `--duration 30`, `--drive circle`. Use `--no-headless` for the
  viewport, `--forever` to run until Ctrl+C.
- **`scripts/_ros2_graphs.py`** — shared `build_robot_graphs(...)` that
  authors three single-purpose Action Graphs per robot:
  - `OdometryGraph` — `IsaacComputeOdometry` on `base_link` →
    `ROS2PublishOdometry`.
  - `CameraGraph` — `IsaacCreateRenderProduct` → two `ROS2CameraHelper`
    nodes (`type="rgb"` and `type="depth"`) sharing one render product.
  - `TfGraph` — `ROS2PublishTransformTree` on the robot subtree,
    parented at `base_link`.
- **`tests/isaac/test_ros2_topics.py`** — pytest using same-process
  `rclpy` (the bridge's bundled rclpy from
  `.../isaacsim.ros2.bridge/jazzy/rclpy`). Subscribes to all 8 expected
  topics (6 + 2 `/tf`), steps the sim for ~12 wall-clock seconds, and
  asserts:
  1. Every expected topic has ≥ 1 message.
  2. `frame_id` headers differ between `robot_0` and `robot_1` on both
     odom and the RGB camera.
  3. `robot_0`'s odom position moves > 0.01 m across the test window
     (proves the prismatic-joint base motion is reaching the ROS 2 layer).

## Running it

After `source scripts/activate_isaac.sh`:

```bash
# Author the warehouse (one-time; gitignored output)
python scripts/build_warehouse.py --out scenes/warehouse_v0.usd

# Pytest regression (headless, ~25 s)
python -m pytest tests/isaac/test_ros2_topics.py -v

# Manual smoke (visible viewport, runs until you Ctrl+C)
python scripts/spawn_warehouse.py --no-headless --forever
```

The `activate_isaac.sh` wrapper exports `ROS_DISTRO=jazzy`,
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and prepends the bridge's bundled
lib dir to `LD_LIBRARY_PATH`. Without those three vars, the bridge
extension fails to load with `[Error] ROS2 Bridge startup failed` and
every `og.Controller.edit("...ROS2PublishOdometry...")` raises
`OmniGraphError: unrecognized type`.

## Multi-robot namespacing

We use the explicit `inputs:nodeNamespace` knob on every publisher node
rather than the USD `isaac:namespace` attribute. Each robot's graph is
authored with `nodeNamespace="robot_0"` (or `"robot_1"`) which prepends
the namespace to the published topic. The graph still publishes to
`tf` with namespace prepended, so each robot's TF tree lands at
`/robot_<n>/tf`. Downstream nav stacks remap to `/tf` via standard ROS 2
launch-file remaps.

## Inspecting topics from a separate shell

The bridge ships its own ROS 2 Jazzy libs. To use `ros2 topic list` /
`ros2 topic echo` *without* sourcing your system `/opt/ros/jazzy`:

```bash
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/isaacsim/venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib

# From another terminal while spawn_warehouse.py is running:
ros2 topic list
ros2 topic echo /robot_0/odom --once
```

If you DO source `/opt/ros/jazzy`, the bundled rclpy collides with the
system one and pytest segfaults. `_isaac_utils.guard_against_ros2()`
fails fast on `AMENT_PREFIX_PATH` for this reason.

## Camera attachment + orientation

For each robot we attach an `isaacsim.sensors.camera.Camera` as a child
of the URDF's `head_camera_rgb_optical_frame` Xform — that prim already
sits at the right pose for the head camera, but it inherits the empty
TF-frame convention from the URDF.

USD cameras default to looking down `-Z` with `+Y` up; ROS optical
frames are `+Z` forward, `+Y` down. We apply a 180° local rotation about
the X axis (`orientation=(0, 1, 0, 0)` in wxyz) so the camera's USD
look-axis aligns with the ROS optical look-axis. Without this, the
published RGB image looks at the back of the robot's head.

Resolution / frequency: `spawn_warehouse.py` uses 640×480 @ 30 Hz. The
pytest test drops to 320×240 @ 15 Hz to stay safely inside the 8 GB
VRAM budget on an RTX 4070 Laptop with two cameras live.

## Why XLeRobot needs --fix-base and a mass fix-up

XLeRobot's URDF models its mobile base as a ManiSkill-style holonomic
chain: three serial joints (X prismatic, Y prismatic, Z continuous)
from a virtual `root` link with `mass=0` down to `base_link`. To make
that work in Isaac Sim:

- **`--fix-base`** (Phase 2): welds `root` to the world. Without it,
  applying velocity to the prismatic pushes both `root_arm_1_link_1`
  forward and reacts-pushes `root` backward, leaving the robot stuck.
- **Mass fix-up** (Phase 3): `import_urdf.py` now bumps any link with
  zero or missing mass to a 0.5 kg minimum (`scripts/import_urdf.py`'s
  `fix_zero_masses()`). Without it, spawning two XLeRobot articulations
  in one PhysX scene leaves the inverse-mass matrix ill-conditioned and
  joints saturate at their ±20 m limits within a few steps. The fix-up
  doesn't meaningfully alter kinematics — `base_link` is 70 kg, so a
  0.5 kg bump on auxiliary links is noise.

## Troubleshooting

- **`OmniGraphError: unrecognized type 'isaacsim.ros2.bridge.ROS2*'`** —
  the bridge isn't loaded. Re-source `activate_isaac.sh` and confirm
  `ROS_DISTRO=jazzy`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and
  `LD_LIBRARY_PATH` includes `.../isaacsim.ros2.bridge/jazzy/lib`.
  Conftest enables the extension via `set_extension_enabled_immediate`
  + 10 `app.update()` ticks; if you replicate the pattern in a new
  script make sure you tick after enabling.
- **Both robots' joints saturate at ±20 m** — the mass fix-up wasn't
  applied. Delete `assets/generated/xlerobot.usd` and re-run
  `scripts/import_urdf.py` (or `pytest tests/isaac/` which subprocesses
  the import). Check `/tmp/isaac_import_result.txt` for
  `mass-fix: bumped N link(s)`.
- **`'World' object has no attribute '_scene'`** — same Phase 2 gotcha;
  use `World.clear_instance()` between tests, not
  `SimulationContext.clear_instance()`. The `world` fixture in
  `tests/isaac/conftest.py` does this correctly.
- **rclpy import fails inside Kit** — confirm the bundled rclpy path is
  on `sys.path` after the bridge enable. Check the printed
  `rclpy.__file__` in `/tmp/isaac_ros2_topics_result.txt` — it should
  point into `.../isaacsim.ros2.bridge/jazzy/rclpy`.
- **Test hits the 12 s wall-clock budget without all topics** — most
  likely the timeline never started. The spawn flow calls
  `omni.timeline.get_timeline_interface().play()` after building
  graphs; if you reorder, make sure `play()` comes after
  `world.reset()` and *before* the step loop.

## Known limitations

- The "circle" drive mode in `spawn_warehouse.py` is a misnomer — the
  xlerobot.json physics override locks `root_z_rotation_joint` as a
  position drive, so applying yaw via `joint_velocities[2]` is fought
  by the high-stiffness position drive and the robots end up driving
  straight. To make them actually curve, change `root_z_rotation_joint`
  to `drive_type: "velocity"` in `configs/physics_overrides/xlerobot.json`.
  Out of scope for Phase 3.
- The visible "raskog wheels" on XLeRobot are visual-only STLs — there's
  no real wheel physics. The base translates via the prismatic joint
  instead. See `[[project-isaac-sim]]` for the sim-to-real implications.
- `/tf` is published *per robot* under each namespace
  (`/robot_0/tf`, `/robot_1/tf`). Standard multi-robot ROS 2 pattern;
  Nav2 etc. remap to `/tf` at launch time.
