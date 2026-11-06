# The real robot (Jetson)

## Jetson

JetPack 6 is Ubuntu 22.04 (Humble), but this repo is Jazzy, so the stack runs in a
Jazzy container (`docker/jetson/`). **Never source the host's `/opt/ros/humble`,**
and never run its RealSense node: only one process can own the D435.

```
phone ──:8081/:9090/:8080──▶ Jetson ─ docker sortbots-jetson:jazzy
                                 ├─ run_console.sh --platform real
                                 └─ run_robot.sh → sortbots_bringup.launch.py platform:=real
                                      realsense2_camera → /robot_0/camera/*, RTAB-Map visual odometry
```

**Host setup (once):**

1. Add yourself to the docker group, and check `docker info | grep nvidia`.
2. Install the RealSense udev rules (`99-realsense-libusb.rules`).
3. Plug the D435 into a USB **3** port. `lsusb -t` must show `5000M`; at 480M the
   driver falls back to 640×480@15.
4. Set the power mode: `sudo nvpmodel -m 2 && sudo jetson_clocks`.

```bash
scripts/jetson.sh build      # first time, 10–20 min
scripts/jetson.sh console    # dashboard console in container 'sortbots'
scripts/jetson.sh status | shell | logs | stop
```

Open `http://jetson-orin.<tailnet>.ts.net:8081/` and start `real_map_fresh`. Move
the robot slowly: fast turns and blank walls lose visual odometry.

**Differences from the sim:**

- `use_sim_time:=false`.
- Visual odometry on raw depth instead of Isaac's perfect odometry.
- No IMU (`wait_imu_to_init:=false`).
- Depth is 16UC1 mm.
- The map anchor is `spawn.real`.
- Nav2, the explorer and task_manager are **off** until a base driver exists.
- DDS is localhost-only, so a desktop sim on the LAN can't cross-talk with
  `/robot_0/*`.
- `SORTBOTS_PLATFORM=real` must be set inside the container. `jetson.sh` sets it.

The camera mount in `configs/sensors/d435_real.yaml` is still a placeholder. Measure
it. A 1.2 MB D435 frame doesn't fit Fast DDS's default 512 KB shared-memory segment,
and silently dropped UDP fragments starved odometry. `configs/dds/fastdds_large_shm.xml`
(set by `jetson.sh`) fixes that.

**Viewing on the Jetson's own screen:** snap browsers need `SNAP_REEXEC=0`. For WebGL
without a GPU, add `--disable-gpu --enable-unsafe-swiftshader --ignore-gpu-blocklist`.
Viewing from another device is better.

### SLAM offload

On the Orin Nano, mapping competes with odometry and obstacle detection. With
`slam_role` set to `robot`, the Jetson keeps the camera driver, odometry, the live
obstacle cloud and 2 Hz compressed keyframes. The workstation runs RTAB-Map. On a
bench at 2× speed, this cut SLAM trajectory error from 2.8–4.7 m to about 0.5 m and
obstacle latency p50 from about 80 ms to about 35 ms, using about 1 Mb/s of link.

```bash
# workstation
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=<jetson>
ros2 launch ./launch/sortbots_slam_workstation.launch.py robot_id:=robot_0
# jetson
SORTBOTS_WORKSTATION=<workstation> scripts/jetson.sh console   # then scenario real_map_offload
```

Without a workstation, the default `--slam-role all` maps on the Jetson at `nice 10`.
`docker/slam_bench/` and `scripts/slam_bench/` reproduce the bench.

**Next hardware steps:**

- A base driver (`cmd_vel` → motors, `odom`), after which Nav2, the explorer and
  task_manager can be enabled on the robot.
- An MPU6050 driver feeding `imu_filter_madgwick`.
- cuVSLAM, which needs JetPack 7.2.
- Mesh radios (see [Fleet comms](commsim.md)).

## Gaussian splats

The 3D panel's **splat** mode shows a photoreal Gaussian splat of a saved map. It
is trained after the run, **on the workstation GPU only, never the Jetson**, in its
own conda env `splat`. The pipeline: `rtabmap-export` dumps keyframes and poses,
gsplat trains, and the result is anchored into the fleet `map` frame.

```bash
scripts/install_splat.sh [--check]     # once (pinned torch 2.4 / gsplat 1.5.3 prebuilt wheels)
scripts/splat.sh serve                 # worker on :8082
scripts/sim_ctl.sh stop --save-map NAME --splat                     # sim
scripts/splat.sh build NAME --from http://jetson-orin.<tailnet>:8081 --wait   # real robot's map
scripts/splat.sh status | log JOB
```

A 1,157-keyframe map takes about 4 min and produces about 42 MB. Jobs wait while
Isaac holds the GPU. Output goes to `~/.cache/sortbots/splats/`. On the robot, set
`worker_url` in `configs/splat.yaml` to the workstation.
