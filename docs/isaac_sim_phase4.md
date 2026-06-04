# Isaac Sim Phase 4 — D435-spec cameras + cmd_vel + per-robot RTAB-Map

End state: the warehouse demo's cameras match the team's planned
RealSense D435, an InvenSense MPU 6050 IMU publishes from each robot,
and an external ROS 2 node can drive each robot via `/robot_<n>/cmd_vel`.
The first SLAM consumer wired up is **RTAB-Map**, running natively on
ROS 2 Jazzy (no Docker, no Isaac ROS yet). Phase 5+ swaps in cuVSLAM
and the Jetson-parity Docker path.

This page assumes [`setup.md`](setup.md),
[`isaac_sim_phase2.md`](isaac_sim_phase2.md), and
[`isaac_sim_phase3.md`](isaac_sim_phase3.md) are done.

## Sensor stack (matches the planned hardware)

| Sensor | Sim publisher | Output topic | Spec source |
|---|---|---|---|
| Intel RealSense **D435** RGB-D | `ROS2CameraHelper`×2 + `ROS2CameraInfoHelper` | `/robot_<n>/camera/{rgb,depth,camera_info}` | `configs/sensors/d435.json` |
| InvenSense **MPU 6050** 6-DOF IMU | `IsaacReadIMU` → `ROS2PublishImu` | `/robot_<n>/imu` | `configs/sensors/mpu6050.json` |
| SparkFun **PAA5160E1** optical odom | (placeholder: noiseless `IsaacComputeOdometry`) | `/robot_<n>/odom` | Phase 5 will inject noise |

The D435 specs reproduce the depth-aligned color stream at 848×480,
30 Hz. Distortion is all-zero because `rs2::align` rectifies the
aligned output. The intrinsic JSON drives both Camera prim attributes
(focal length, apertures) and the `ROS2CameraInfoHelper` which derives
`K`, `P`, `R`, `D` from those attributes.

The MPU 6050 sim ships at 60 Hz publish (well above what RTAB-Map's
IMU input needs). Mount offset defaults to `[0, 0, 0.1]` from
`base_link` — placeholder until CAD pins the IMU board location.

**Honest disclosure for sim-to-real**: the sim IMU publishes a fused
orientation that the real MPU 6050 does not provide (the real chip
gives only raw accel + gyro; orientation is integrated by software
like `imu_filter_madgwick`). The Phase 4 default leaves
`publish_orientation: true` because it's convenient for RTAB-Map's
gravity prior. Phase 5+ sim-to-real work should flip it off and run a
matching filter node in software.

## The warehouse scene

`spawn_warehouse.py` loads one of two environments via `--scene`:

| `--scene` | What loads | Source |
|---|---|---|
| `nvidia` (default) | NVIDIA's fully-assetted **Simple_Warehouse** (`full_warehouse.usd` — stocked shelves, pallets, barrels, props) | Streamed from the Isaac asset server via `get_assets_root_path()`; cached locally after first run |
| `primitive` | The hand-built `scenes/warehouse_v0.usd` (textured boxes/cylinders) | Local, offline; what the pytest suite uses |

The NVIDIA warehouse is the default because it gives RTAB-Map far more
ORB texture/geometry to track than the primitive boxes. It needs network
(or a local Omniverse Nucleus mount) on the first run; if the asset root
is unreachable the script raises a clear error pointing you at
`--scene primitive`. Robots spawn in the open front aisle — `robot_0`
sits exactly where NVIDIA places Nova Carter in
`carter_warehouse_navigation.usd` (`-6, -1`), a verified-drivable spot.

The pytest suite (below) deliberately stays on the **primitive** scene so
CI is offline and deterministic; only the interactive demo uses NVIDIA's.

## Running the full Phase 4 demo (single-robot teleop + live SLAM)

You'll need three terminals. For the teleop-one-robot demo, spawn a
single XLeRobot (`--robots 1`) to keep VRAM free for rviz + RTAB-Map on
the 8 GB RTX 4070.

**Terminal 1 — Isaac Sim in NVIDIA's warehouse with one robot:**

```bash
source scripts/activate_isaac.sh
python scripts/spawn_warehouse.py --no-headless --forever --robots 1
```

`--scene nvidia` is the default. The first launch streams the warehouse
(~hundreds of MB) and can take a few minutes; later launches hit the
local cache. The default `--drive cmd_vel` mode waits for external Twist
messages; the robot stands still until terminal 3 sends commands. Pass
`--scene primitive` to fall back to the local box warehouse offline.

**Terminal 2 — RTAB-Map on robot_0:**

```bash
sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-teleop-twist-keyboard  # one-time
source /opt/ros/jazzy/setup.bash
ros2 launch ./launch/sortbots_rtabmap_robot.launch.py robot_id:=robot_0
```

rviz opens with RTAB-Map's panels. It will wait for the first IMU
message, then start consuming RGB-D + odom + TF.

**Terminal 3 — drive robot_0 around (WASD):**

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/wasd_teleop.py --robot-id robot_0
```

`scripts/wasd_teleop.py` is a friendlier replacement for
`teleop_twist_keyboard` (whose defaults are the awkward i/j/l/, cluster).
Keys: **w/s** forward/back, **a/d** turn, **q/e** strafe (the XLeRobot base
is holonomic), **space** stop, **z/x** slower/faster, **k** quit. It
streams the Twist continuously and zeroes on stop, so the OmniGraph
SubscribeTwist never holds a stale non-zero command. It auto-re-execs
under `/usr/bin/python3` (ROS 2's interpreter) if you launch it from a
conda/pyenv shell. rviz fills in the dense map as you explore.

For a scripted demo without the keyboard:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --rate 10 /robot_0/cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.2}, angular: {z: 0.1}}'
```

(Both robots can be driven simultaneously by running terminal 2 and 3
duplicates with `robot_id:=robot_1` and `/robot_1/cmd_vel` respectively.)

## What changed under the hood

Compared to Phase 3:

- **`scripts/_ros2_graphs.py`** — `build_robot_graphs(...)` now takes
  optional `camera_info_topic`, `cmd_vel_topic`, `imu_prim_path`,
  `imu_topic`. The `CameraGraph` got a `ROS2CameraInfoHelper`. Two
  new standalone graphs: `CmdVelGraph` (subscriber) and `ImuGraph`
  (publisher).
- **`scripts/spawn_warehouse.py`** — loads `d435.json` + `mpu6050.json`
  at startup, sets Camera prim attributes from D435 specs, authors an
  `IsaacImuSensor` under each `base_link`, and adds a `--drive cmd_vel`
  mode (now the default) that reads the OG SubscribeTwist outputs.
- **`configs/physics_overrides/xlerobot.json`** —
  `root_z_rotation_joint` flipped from position-drive to velocity-drive
  so `cmd_vel.angular.z` actually rotates the robot. Phase 3's "circle
  drive misnomer" is now fixed.
- **`launch/sortbots_rtabmap_robot.launch.py`** — thin wrapper around
  `rtabmap_launch/rtabmap.launch.py` with `/robot_<n>/*` remaps,
  `wait_imu_to_init: true`, `visual_odometry: false` (use sim's
  noiseless odom as the motion prior), and `use_sim_time: true`
  (REQUIRED — see below).

### TF + sim-time plumbing (required for RTAB-Map to build a map)

Getting RTAB-Map to actually map against the sim needs three things that
bit hard the first time the pipeline was run end-to-end:

1. **`use_sim_time:=true`.** The Isaac ROS 2 bridge stamps every message
   with *simulation* time (seconds since sim start), not wall clock. With
   `use_sim_time=false`, RTAB-Map treats all data as ~decades old and
   every TF lookup fails — the map stays empty. The launch file defaults
   it to `true` and the global `/clock` publisher (from `spawn_warehouse`)
   feeds it.
2. **A complete, prefixed TF chain.** `ROS2PublishTransformTree` can only
   emit raw USD prim names (no namespace prefix), and it never publishes
   an `odom→base_link` link at all. So `_ros2_graphs.py` adds
   `ROS2PublishRawTransformTree` publishers for the frames RTAB-Map needs,
   with the prefixed names the data streams already use:
   `robot_<n>/odom → robot_<n>/base_link` (dynamic, from the odom node),
   `robot_<n>/base_link → robot_<n>/camera_optical` (camera extrinsic,
   computed from USD), and `robot_<n>/base_link → robot_<n>/imu_link`
   (from the MPU mount config).
3. **TF on the *global* `/tf`, not `/robot_<n>/tf`.** ROS 2 `tf2`
   TransformListeners subscribe to the absolute `/tf` regardless of node
   namespace, so a namespaced tf topic has zero consumers. The raw TF
   publishers above use `nodeNamespace=""` → global `/tf`; the frame_ids
   stay prefixed (`robot_<n>/...`) so it remains multi-robot-safe.

## Tests

Three sim-side pytest cases run in CI (no Docker, no rtabmap install
needed — the topics are validated by their shape):

```bash
source scripts/activate_isaac.sh
python -m pytest tests/isaac/test_camera_info.py \
                 tests/isaac/test_cmd_vel.py \
                 tests/isaac/test_imu_topic.py -v
```

- `test_camera_info.py` — asserts the camera_info message's K/D/frame_id
  match the D435 JSON.
- `test_cmd_vel.py` — publishes a constant Twist, asserts the robot
  moves >0.3 m in odom; also confirms `angular.z=0.5` produces
  measurable yaw change (regression for the xlerobot.json drive-type
  flip).
- `test_imu_topic.py` — subscribes to `/robot_0/imu`, asserts gravity
  shows up in `linear_acceleration.z` and the stationary robot has
  near-zero angular velocity.

## Troubleshooting

- **`Isaac asset root is unreachable — get_assets_root_path() returned
  None`** — the NVIDIA warehouse streams from NVIDIA's S3/Nucleus server,
  so the machine needs network access on the first run. Re-run with
  `--scene primitive` to use the local box warehouse offline, or point
  `--warehouse-url` at a local Nucleus mirror.
- **First NVIDIA-scene launch hangs for minutes / watchdog fires** — it's
  streaming `full_warehouse.usd` and its dependencies. The watchdog floor
  is raised to 600 s for `--scene nvidia` to allow this; the download is
  cached under `~/.cache/ov/` so subsequent launches are fast.
- **`OmniGraphError: Could not create node using unrecognized type
  'isaacsim.ros2.bridge.ROS2CameraInfoHelper'`** — same fix as Phase 3:
  the bridge extension needs `app.update()` ticks after enable. The
  `sim_app` fixture already handles this.
- **RTAB-Map exits immediately with `"could not connect to camera_info"`** —
  the topic isn't being published. Confirm with
  `ros2 topic list | grep camera_info`. If missing, your USDs were
  regenerated under a Phase 3 build of `_ros2_graphs.py`. Delete
  `assets/generated/xlerobot.usd` and re-run `spawn_warehouse.py` to
  regenerate the cached graph.
- **rviz panel is empty / no map shows** — RTAB-Map is waiting for the
  first IMU sample (we set `wait_imu_to_init: true`). Confirm with
  `ros2 topic hz /robot_0/imu` that the sim is publishing at ~60 Hz.
  If silent, the OG `IsaacReadIMU` couldn't bind its `imuPrim` — usually
  means the path is wrong; check `/tmp/isaac_spawn_warehouse_result.txt`
  for the `spawned robot_0 ... imu=/World/robot_0/...` line.
- **`cmd_vel.angular.z` does nothing** — the `xlerobot.json` drive-type
  edit didn't land. Confirm the file has `"drive_type": "velocity"`
  under `root_z_rotation_joint`, then delete and regenerate
  `assets/generated/xlerobot.usd`.
- **The robot drives slightly when no cmd_vel is being published** —
  Phase 4 reads the OG SubscribeTwist outputs unconditionally; the
  default (no message received) is the zero vector. If you see drift,
  some other Twist publisher is on the network — `ros2 topic info
  /robot_0/cmd_vel --verbose` lists the publishers.
- **`apt install ros-jazzy-rtabmap-ros` fails with "no installation
  candidate"** — the package may be in `ros2 testing` only. Add the
  testing repo or build from source per https://github.com/introlab/rtabmap_ros.

## Known limitations / Phase 5+ backlog

- **No PAA5160E1 noise injection** — `/robot_<n>/odom` is currently
  noiseless. Phase 5 should add a synthesized noise model (σ ≈ 0.5
  mm/sample on Δx, Δy from the datasheet RMSE).
- **MPU 6050 publishes orientation** — sim is "richer than real". Sim
  2 real work should flip `publish_orientation: false` and run
  `imu_filter_madgwick` to compute orientation in software, matching
  the on-chip i2c reality.
- **D435 intrinsics are aperture-derived, not per-unit calibrated** —
  `fx ≈ 600` from the JSON vs `fx ≈ 605` typical-EEPROM. Phase 5 can
  sweep the JSON values to match a real D435's calibration if RTAB-Map
  shows excessive depth misalignment.
- **No multi-agent map fusion** — each robot's RTAB-Map runs
  independently. Phase 5+ will explore coarse frame alignment (known
  spawn poses) or true collaborative SLAM (Kimera-Multi / GTSAM).
- **No cuVSLAM yet** — the Jetson-parity story for VSLAM is deferred.
  The Phase 4 sim publishers (RGB, depth, camera_info, IMU) are
  already shaped for cuVSLAM, so the swap is "change the SLAM node,
  not the sim side."
- **rviz GPU contention** — when Kit (~3.5 GB VRAM) and rviz (~500 MB)
  both run on the same 8 GB RTX 4070, peak memory can spike under
  high-traffic mapping. Drop the camera resolution if either crashes.
