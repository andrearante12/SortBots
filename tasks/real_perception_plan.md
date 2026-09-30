# Real-robot perception: limitations found on 2026-09-28/30, and a plan

**Status:** proposal for review. Nothing here is implemented yet.
**Scope:** the camera → odometry → SLAM → dashboard chain on the Jetson Orin
Nano + RealSense D435 (`platform: real`, `docs/jetson.md`). Motion control
(base driver, Nav2 on hardware) is out of scope except where it feeds perception.

## What we hit

| # | Symptom seen | Root cause | Evidence |
|---|---|---|---|
| L1 | 3D reconstruction froze for a whole run while the camera feed looked live | Visual odometry lost on essentially every frame from launch. RTAB-Map drops frames while odom is lost, and the default `Odom/ResetCountdown=0` never recovers | 11,160 `Registration failed … 0/20 inliers` in 20 min; covariance 9999 |
| L2 | "Odometry at ~4.5 Hz" was reported as healthy while it was dead | `publish_null_when_lost=true` publishes empty odom when lost; we checked rate, not quality | same run |
| L3 | Odometry on the filtered depth never tracked | `dynamic_obstacle_filter.py` runs at ~7.8 Hz vs the camera's 30, about 0.5 s behind, and blanks pixels frame to frame | raw-depth odometry tracked at once (quality ~180, 0 failures). **Fixed in `d4a24c0`** (odom on raw depth + auto-reset) |
| L4 | Obstacles put in front of a still camera never appear | RTAB-Map adds a keyframe only after ~10 cm / ~6° of motion. It's a SLAM map, not a live obstacle view | map updated once the camera moved |
| L5 | Odometry is slow and CPU-heavy | CPU `rgbd_odometry` at 848×480: ~0.16 s per update, so ~3–5 Hz; container used ~3 of 6 cores | `Odom: quality … update time=0.16s` |
| L6 | Tracking breaks on fast turns, blank walls, low light | Camera only: no IMU and no wheel odometry to bridge bad frames | known failure mode; the room image was dim/grainy |
| L7 | Map floor tilt and height are unreliable | Camera mount in `configs/sensors/d435_real.yaml` is a placeholder (1 m, level); no gravity reference (plain D435, no IMU) | placeholder visible in TF: `base_link → optical` z = 1.000 |
| L8 | Camera silently degraded to 640×480@15 | D435 on a USB 2 hub; the driver only logs a warning | "connected using a 2.1 port"; fixed by moving to USB 3 |
| L9 | Frame timestamps are host time, not camera time | ROS-packaged librealsense uses the V4L2 backend on JetPack's stock uvcvideo (no RealSense metadata patches) | driver falls back to system-time stamps |
| L10 | The GPU does nothing for perception | RTAB-Map/odometry are CPU-only; cuVSLAM (Isaac ROS 3.x) targets Humble while the container is Jazzy | version check still pending |
| L11 | Health checks lie (pgrep self-match, `ros2 topic hz` capped by big images, `topic echo` without a type) | tooling pitfalls hit repeatedly while diagnosing | several false "running" / "timeout" results this session |
| L12 | Robot map picker would offer sim warehouse maps | map manifests don't record which platform built them | `real_localize` lists every `maps/` entry |
| L13 | No way to reproduce a real-perception bug offline | no recorded real D435 data; every check needed the live robot | this whole session |

## Plan

Ordered so each phase makes the next one measurable. Effort is rough, in
focused working days.

### Phase 1: make perception state visible and trustworthy (L1, L2, L8, L11) · ~1.5 d

The worst part of L1 wasn't the bug. It ran for 20 minutes and nothing said so.

1. **`nodes/perception_health.py`**, one per robot, publishes `/<id>/perception_health`
   (`std_msgs/String` JSON, like the rest of the protocol):
   - odom state `tracking | lost | reset`, from `/<id>/odom` covariance (≥ 9999 = lost) and
     the `rtabmap_msgs/OdomInfo` topic (`lost`, `inliers`, `features`, update time);
   - real rates of rgb/depth/depth_static/odom/info, measured from header stamps in
     the node itself (no `ros2 topic hz`, see L11);
   - camera: USB type and active profile, parsed once from the driver's parameters.
2. **Dashboard**: replace the binary "SLAM: stale" badge with the reason: *odom
   lost 12 s*, *depth filter 7 Hz*, *camera USB 2 (640×480@15)*. Keep it in the
   existing badge style; a tooltip carries the numbers.
3. **`run_robot.sh`** readiness: wait for `odom state = tracking`, not just
   camera_info. **Refuse to start (ERROR:) on USB 2** unless `--allow-usb2`, so
   L8 can't recur silently.
4. **Tooling**: add `scripts/robot_health.sh` (inside the container) that prints the
   health JSON. It replaces ad-hoc pgrep/hz/echo checks, and docs/jetson.md points at it.

*Verify:* cover the lens (odom lost), and the badge says so within 2 s. Unplug and
re-plug into USB 2: `run_robot.sh` refuses with a clear message. Offline: unit-test
the covariance/OdomInfo → state mapping the way `tests/*_test.py` does for other nodes.

### Phase 2: real-time obstacles, separate from the SLAM map (L4, L3 follow-up) · ~2 d

RTAB-Map's map is *where things were when we passed by*. The robot and the
operator also need *what is in front of the camera now*.

1. **Live obstacle layer.** `depth_image_proc/point_cloud_xyz` on raw depth,
   decimated (every 4th pixel) and cropped to 0.3–4 m and floor+5 cm…1.5 m. Then:
   - the Nav2 costmap obstacle layer (already the sim design; used as soon as Nav2 runs on hardware);
   - the dashboard as a separate, fading "live" point layer in the 3D panel, over the
     persistent reconstruction.
2. **Speed up `dynamic_obstacle_filter.py`** (L3): it only needs to keep up for
   SLAM now. Process on a 2× or 4× decimated image and upsample the mask, and replace the
   pure-numpy connected components with `cv2.connectedComponentsWithStats`. Target ≥ 15 Hz
   at < 0.5 core. On a single real robot there are no peer robots to remove, so also add
   a `filter: off` switch for `platform: real` and measure whether SLAM quality changes.
3. Optionally lower RTAB-Map's `RGBD/LinearUpdate` / `AngularUpdate` for real (for
   example 0.05 m / 0.05 rad) so the map fills in during slow manual motion. Measure the RAM
   and CPU cost before keeping it.

*Verify:* drop an object in front of a still camera. It appears in the live layer
within 0.5 s, and appears in the SLAM map after a small camera move. Filter rate from
Phase 1's health topic ≥ 15 Hz.

### Phase 3: geometry you can trust (L7, L6 partial) · ~1 d

1. **Ground-plane calibration script** `scripts/calibrate_mount.py`: with the robot on
   a flat floor, RANSAC-fit the floor plane in the depth cloud and solve for camera
   height, pitch and roll. Write them into `d435_real.yaml` `mount:` with the date. Yaw and
   x/y still come from a tape measure (documented).
2. **Gravity without an IMU (interim):** feed that fitted pitch/roll as the static
   mount, and set RTAB-Map `Reg/Force3DoF=true` for the real robot. A ground robot
   can't pitch or roll, so this removes floor-tilt drift outright. Revisit when the
   IMU (Phase 5) supplies real gravity.

*Verify:* the fitted floor lands at z = 0 ± 2 cm in `map`. Walk a 10 m loop and the
floor stays level (compare map z extent before/after).

### Phase 4: GPU odometry with cuVSLAM (L5, L6, L10) · ~3–4 d, gated on a version check

1. **Version check first (½ d):** confirm which Isaac ROS release has
   `isaac_ros_visual_slam` for JetPack 6.2 / R36.4. Expected: Isaac ROS 3.x on
   **Humble**. If an Orin-supported Jazzy build exists, use it and skip step 2.
2. **Humble sidecar container** (`docker/jetson-isaac/`) from NVIDIA's Isaac ROS
   base image, same `--network host --ipc host`, localhost DDS. It runs only
   cuVSLAM and publishes `/<id>/odom` + `odom → base_link`. Humble↔Jazzy on one host
   using only std messages is not officially supported but generally works; the
   Phase 1 health topic is how we'd notice if it doesn't.
3. **Camera input:** cuVSLAM wants the D435 **IR stereo pair** with the **emitter off**,
   but SLAM depth wants the emitter on. Use the driver's emitter on/off
   interleave (`depth_module.emitter_on_off`) and split frames, so emitter-off IR goes to
   cuVSLAM and emitter-on depth goes to RTAB-Map (Isaac ROS ships a splitter for this).
   Fallback: emitter off everywhere and accept worse depth on blank walls.
4. `camera_odometry:=false` + new `odometry_source: cuvslam | rtabmap` in the real
   bringup; keep `rtabmap` (today's fix) as the no-GPU fallback.

*Verify:* same handheld loop recorded as a bag (Phase 6), with both sources replayed:
odom rate (target ≥ 30 Hz), lost-frame count, loop-closure error, CPU cores.
Decide on numbers, not impressions.

### Phase 5: more sensors (L6, L7) · depends on hardware arriving

- **MPU6050** → `/<id>/imu/data_raw` → `imu_filter_madgwick` (already in the image) →
  `/<id>/imu`. Then `wait_imu_to_init:=true`, feed the IMU to odometry (cuVSLAM supports
  it; so does RTAB-Map), and drop the `Force3DoF` crutch if tilt stays bounded.
- **Wheel odometry** from `nodes/base_driver.py`, fused with visual odometry via
  `robot_localization` EKF, so a lost visual frame no longer freezes the pose at all.
- The IMU gives the orientation half. Extrinsics come from Phase 3's calibration plus CAD.

### Phase 6: reproducible perception offline (L13, L9, L12) · ~1.5 d

1. **Record real D435 bags** (`scripts/record_dashboard_bag.sh` already exists): one
   still scene, one slow handheld loop, one fast-turn/blank-wall stress run. Store them
   out of git (they are large); `maps/`-style manifest.
2. **`tests/real_odometry_replay`** (manual, needs ROS, not CI): replay each bag
   through the real bringup with the camera driver off, and assert lost-frame count and
   loop-closure error under thresholds. Every fix above gets a before/after number.
3. **Timestamps (L9):** measure camera-to-host latency jitter from the bags. Only if it
   hurts odometry: build librealsense with `FORCE_RSUSB_BACKEND` in the image for
   hardware timestamps and metadata (costs a source build; no kernel patching on the Jetson).
4. **Map provenance (L12):** `scripts/maps.sh save` records `platform` (and camera
   serial) in `map.json`; the Scenarios map picker filters to the console's platform.

## Suggested order

Phase 1 → 2 → 3 first (about a week total, no new hardware, all CPU-side). They make
the current camera-only robot usable and give us the measurements Phase 4 needs.
Phase 6's bag recording should start alongside Phase 1, since every later
comparison depends on it. Phase 4 starts with its half-day version check whenever
convenient; Phase 5 waits on hardware.

## Open questions for review

1. Is the robot expected to see *other SortBots* on the real floor soon? If not,
   `dynamic_obstacle_filter` can be off on hardware for now (Phase 2.2).
2. Is `Reg/Force3DoF` acceptable? It assumes flat floors (true for a warehouse).
3. D435**i** swap vs MPU6050: the D435i's IMU is factory-calibrated to the camera,
   which removes a whole extrinsics problem. Is the swap an option?
4. Where should real bags live (size: roughly 1–2 GB per minute at 848×480@30)?
