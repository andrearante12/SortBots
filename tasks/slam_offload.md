# SLAM on the Jetson: why it's slow and drifts, and what we changed (issue #2)

**Issue:** "SLAM Reconstruction (Jetson) is slow, inaccurate, and extremely
prone to drift." The constraint is a Jetson Orin Nano: 6 CPU cores and 8 GB of
memory shared between CPU and GPU. Reconstruction must not get priority over
obstacle detection. Suggestion from the issue: move SLAM reconstruction onto
the workstation, alongside Gaussian Splatting.

**Status (2026-10-09):** implemented on branch `liam/slam-offload`. Everything
was measured on an offline bench (below). **Nothing has run on the robot yet.**
The robot checks are in "What to verify on the robot".

## Summary

1. **Images may never reach odometry.** Fast DDS's default shared-memory segment
   (512 KB) is smaller than one D435 frame (1.2 MB). Each frame then falls back
   to fragmented UDP, and the default `net.core.rmem_max` (212 KB) drops most of
   the fragments. On the bench, 1.6 Hz of 11.7 Hz arrived, and odometry
   processed 23 of 832 frames. Nothing reports an error. This fits the
   perception plan's L11 note that `ros2 topic hz` "is capped by big images".
   **Fixed** with `configs/dds/fastdds_large_shm.xml`. Still to confirm on the
   Jetson.
2. **Odometry can't keep up.** With mapping on the same CPU budget, odometry ran
   at 3–3.6 Hz and processed 10–24% of the frames. With fewer frames there is
   more motion between them, more lost tracking, and drift that compounds. At
   2× speed the trajectory error reached 2.9–4.8 m. This reproduces the
   issue's "extremely prone to drift".
3. **Mapping crowds out obstacles.** With RTAB-Map sharing the robot's CPU, the
   live obstacle path had a 60–100 ms median latency and 200–300 ms p99. With
   mapping moved off the robot, the median was 6–9 ms.
4. **Camera-only odometry drifts by construction.** Every configuration
   showed about 7% of distance travelled. Only more sensors fix that: wheel
   odometry and an IMU (open items).

**What changed:** mapping moves to the workstation (`slam_role`). The robot
keeps odometry, a new live obstacle cloud, and a half-resolution keyframe
stream. Odometry switches to frame-to-frame. On-robot mapping, the fallback,
runs at lower CPU priority. The DDS profile is fixed. RTAB-Map's database
exports straight to COLMAP for Gaussian Splatting.

## The bench (how we measured without the robot)

The Jetson isn't reachable from the workstation, and no D435 recordings exist
(perception plan L13). So: `scripts/slam_bench/`, `docker/slam_bench/`.

- **Data:** TUM RGB-D `freiburg2_pioneer_360` and `pioneer_slam`: a Kinect on a
  Pioneer ground robot, with motion-capture ground truth.
  `tum_player.py` replays them as `/<id>/camera/{rgb,depth,camera_info}`:
  16UC1 mm depth aligned to color, system-time stamps, as the D435 driver
  publishes them. The real launch files therefore run unchanged.
  `--rate 2` replays at twice the speed, which is closer to a hand-carried
  robot than the Pioneer's slow drive.
- **Jetson CPU budget:** the robot-side container is limited to 0.6 of an
  E-core (cores 12–17 of the i9-13900H). That reproduces what was measured on
  the Orin with the shipped stack: odometry at 3.8 Hz with 0.17 s per update,
  against 3–5 Hz and 0.16 s on the robot (docs/jetson.md). The player, logger
  and "workstation" run on other cores.
  - A CFS quota throttles the whole container, so `nice` has no effect under
    it (batch 4).
  - Priority was therefore measured separately, on one whole core with no
    quota, which behaves like the Orin's real cores.
- **Memory:** every container is capped with `--memory` and `--memory-swap`,
  totalling at most 8 GB. A watchdog stops the bench below 1.5 GB available.
  Each run has a private 1 GB `/dev/shm` (see "Shared-memory leak" below).
- **Scoring** (`score.py`):
  - ATE: RMS position error after the best rigid alignment.
  - Drift: median translation error per metre travelled, over 1 s windows.
  - Frames tracked: the share of camera frames odometry produced a pose for.
  - Obstacle latency: depth stamp to the cloud arriving (same clock).
  - Link: bytes per second of compressed keyframes.
- **Repeats:** single runs vary a lot because loop closures either happen or
  don't. Rows below give the mean of 2–3 runs where marked (n).

## Results (`pioneer_slam`, 155 s, robot-side budget 0.6 core)

| | speed | odom Hz | frames tracked | odom ATE m | SLAM ATE m | obstacle p50 / p99 ms | link Mb/s |
|---|---|---|---|---|---|---|---|
| shipped: all on robot | 1× | 3.4–3.6 | 22–24% | 0.53–1.84 | 0.29–0.38 | 70 / 215–222 | — |
| offload (frame-to-map) | 1× | 7.5–7.7 | 51–53% | 0.30–0.34 | 0.19–0.32 | 7–9 / 135–177 | 2.7 |
| offload + frame-to-frame (n=2) | 1× | 10.1 | 69% | 0.43 | 0.19 | 7 / 82–120 | 2.7 |
| shipped: all on robot | 2× | 3.0–3.3 | 9–10% | 2.9–4.8 | 2.8–4.7 | 78–86 / 190–197 | — |
| offload (frame-to-map, n=3) | 2× | 6.6 | 22% | 2.37 | 2.55 | 44–64 / 170–236 | 2.9 |
| **offload + frame-to-frame (n=3)** | 2× | 9.9 | 33% | **1.33** | **0.69** | 34–55 / 156–286 | 2.9 |
| offload + frame-to-frame + half-res keyframes (n=2) | 2× | 10.8 | 36% | 1.41 | 1.15 | 33–35 / 155–172 | **0.95** |
| frame-to-frame, mapping still on robot (n=2) | 2× | 4.6 | 15% | 2.43 | 2.40 | 86–101 / 210–229 | — |

**Priority on one whole core (no quota, `pioneer_slam` 1×):** with on-robot
mapping at `nice 10` (what the launch file now does):
- odometry: 7.3 → 9.3 Hz;
- obstacle latency: p50 16 → 11 ms, p99 32 → 23.5 ms.

So when a workstation isn't available, the fallback still favors obstacles.

On that same whole core, offload gave 9.5 Hz and 11 / 24.4 ms, almost the same.
With correct priorities and some CPU to spare, `nice` alone recovers most of
the obstacle latency. Offloading still pays off in three ways:
- SLAM ATE is better (0.26 vs 0.34 m);
- it wins clearly when the CPU budget is tight (the quota rows above);
- RTAB-Map's growing graph, Grid/3D and clouds no longer occupy the Orin's
  CPU and its 8 GB of shared memory. Those are needed for Nav2 and the
  obstacle classifier (issue #3).

**Confirmed through the real launch files** (`launch_offload` = `slam_role`
robot + workstation with the new defaults: frame-to-frame odometry,
half-resolution keyframes):

| | speed | odom Hz | frames tracked | odom ATE m | SLAM ATE m | obstacle p50 / p99 ms | link Mb/s |
|---|---|---|---|---|---|---|---|
| launch_offload, 0.6-core quota (n=2) | 2× | 10.7 | 36% | 1.19 | **0.51** | 37–40 / 159–165 | 0.95 |
| launch_offload, 0.6-core quota | 1× | 10.1 | 69% | 0.54 | **0.24** | 7 / 76 | 0.89 |
| launch_offload, one whole core | 2× | 17.7 | 61% | 0.97 | 0.35 | 12 / 21.5 | 0.96 |
| launch (on-robot mapping at nice 10), one whole core | 2× | 16.9 | 58% | 0.63 | 0.52 | 9 / 20.5 | — |

Compare the shipped on-robot setup at 2×: 2.9–4.8 m odometry ATE, 2.8–4.7 m
SLAM ATE.

**Rejected:**
- `Reg/Force3DoF`: drift doubled (15.8%), lost frames doubled. On TUM it
  depends on a level camera mount the player only assumes. Retry on the robot
  after mount calibration (perception plan Phase 3).
- Quarter-resolution odometry (`Odom/ImageDecimation 4`): 13–21 Hz, but 60–70
  lost frames and 14–30% drift.
- Optical-flow matching (`Vis/CorType 1`): no clear gain.
- Fewer features (300): no gain.

## Shared-memory leak, found and fixed along the way

Each Fast DDS participant keeps its shared-memory segment in `/dev/shm`, which
is RAM. A participant that is killed (`pkill -9`, `docker stop`) never removes
its segment. After about 30 bench runs, 1,058 leaked segments held 5.2 GB.
That probably contributed to the workstation shutting down on 2026-10-08.

`run_robot.sh stop` kills the pipeline with `pkill -9` inside a container that
shares the host's `/dev/shm`. So the Jetson leaks too, and its 8 GB is also the
GPU's memory.

**Fixes:**
- `run_robot.sh stop` and `jetson.sh stop` run `fastdds shm clean`, which
  removes only segments that no live process holds.
- The profile's segment is 16 MB (about 12 frames in flight).
- Each bench run gets a private, size-capped IPC namespace.

## What runs where now

| | Jetson (`slam_role:=robot`) | Workstation (`sortbots_slam_workstation.launch.py`) |
|---|---|---|
| work | D435 driver; frame-to-frame visual odometry; `nodes/obstacle_cloud.py` (raw depth → base_link points at camera rate, no SLAM or odometry needed); `rgbd_sync` keyframes (2 Hz, half resolution, JPEG + PNG) | RTAB-Map graph, loop closure, Grid/3D, reconstruction clouds, the database |
| sends | `/<id>/rgbd_image/compressed` (~1 Mb/s), `/<id>/odom`, TF | `/<id>/map`, `<id>/map → <id>/odom`, clouds |

Obstacle detection never waits on SLAM. It needs only the static camera mount,
so it keeps working when odometry is lost, the workstation is unreachable, or
the mesh is congested. Nav2's local costmap should read `/<id>/obstacles` once
a base driver exists.

**How to run it:**
- Workstation: `ros2 launch ./launch/sortbots_slam_workstation.launch.py`.
- Robot: `scripts/run_robot.sh --slam-role robot`, or the `real_map_offload`
  scenario.
- DDS crosses machines only via `ROS_STATIC_PEERS`
  (`SORTBOTS_WORKSTATION=<ip> scripts/jetson.sh console`). Everything else
  stays localhost-only.
- Without a workstation, `slam_role:=all` (the default) maps on the Jetson as
  before, with RTAB-Map at nice 10.

**Mesh budget** (commsim's airtime model, 2.4 GHz at 24 Mb/s):
- full-resolution keyframes: about 27% of the channel per robot per hop, so
  two robots exceed the 50% busy line where the fleet model starts to fail;
- half resolution: about 7% per robot.

That is why `keyframe_decimation` defaults to 2. Set it to 1 on a wired or
5 GHz-only link.

## Gaussian Splatting

The workstation's RTAB-Map database already holds every keyframe image with
its loop-closed pose. `scripts/recon/rtabmap_to_colmap.py` turns an
`rtabmap-export --images --poses --poses_format 10 --cloud` export into a
COLMAP text model that splat trainers read (gsplat, nerfstudio `splatfacto`):
- PINHOLE cameras;
- world→camera poses in the optical frame;
- seed points from the colored cloud.

No second structure-from-motion pass is needed.

**Checked on a bench database:**
- cloud points re-projected into the exported cameras match the image colour
  (mean difference 7–17, against 26–54 for random pixels);
- unit tests cover the frame conventions.

A splat hasn't been trained yet.

- Pose format 1 ("RGBD-SLAM") is in motion-capture/optical axes, not ROS
  axes. Using it scored the wrong frame until this was caught.
- Train from a session recorded with **full-resolution** keyframes
  (`keyframe_decimation:=1` on a wired link, or a copy of the robot's own
  recording). Half resolution is fine for SLAM but costs splat detail.

## What to verify on the robot

1. **Frame delivery.** Compare frames per second actually used with the
   camera's 30:
   `ros2 topic echo /robot_0/odom_info --field header.stamp.sec | uniq -c`.
   Also note `sysctl net.core.rmem_max`. Do this with and without the new
   profile: unset `FASTRTPS_DEFAULT_PROFILES_FILE` for the "without" run.
2. **Offload end to end.** The keyframe link rate
   (`ros2 topic bw /robot_0/rgbd_image/compressed`, expect about 1 Mb/s), the
   map on the dashboard, and the Jetson's CPU with and without mapping.
3. **Obstacles.** Cover the lens, then drop an object in front of a still
   camera. `/robot_0/obstacles` should change within one frame.
4. **Shared-memory leak.** `ls /dev/shm | grep -c fastrtps` after a few
   start/stop cycles should stay flat.

## Not done / next

- **Faster odometry on the GPU.** Isaac ROS 4.6 (August 2026) supports Jetson
  Orin on **JetPack 7.2 + ROS 2 Jazzy**, so cuVSLAM can run natively. That
  also removes the need for the Jazzy container. It needs a reflash from
  JetPack 6, and Orin Nano 8 GB support should be confirmed on NVIDIA's list
  first. PyCuVSLAM on JetPack 6 is the no-reflash alternative (perception plan,
  2026-10-02 spike).
- **Wheel odometry and an IMU**, fused through `robot_localization`. Visual
  odometry drift was about 7% in every configuration; only other sensors
  change that.
- **Calibrate the camera mount** (plan Phase 3), then retry `Reg/Force3DoF`.
- **Power mode:** the Orin is at 15 W; MAXN SUPER needs sudo
  (`docs/jetson.md`).
- **Record real D435 data** (plan Phase 6). `tum_player.py` would replay it
  through the same bench.
