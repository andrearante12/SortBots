# Running SortBots on the robot (Jetson + RealSense D435)

The same dashboard drives both tracks. On the desktop, `scripts/run_console.sh`
serves it and the Scenarios tab launches Isaac Sim. On the robot's Jetson, the
same console runs inside a ROS 2 Jazzy container, and the tab launches the real
camera + SLAM stack instead. Nothing in `webui/` knows which one it is talking
to: the D435 driver is remapped onto the sim's topic names.

```
 phone / laptop ──http :8081──▶ Jetson host (JetPack 6, Ubuntu 22.04)
                  ws  :9090     └─ docker: sortbots-jetson:jazzy (Ubuntu 24.04)
                  http:8080        ├─ run_console.sh   rosbridge · web_video_server · serve.py --platform real
                                   └─ run_robot.sh     sortbots_bringup.launch.py platform:=real
                                        ├─ realsense2_camera  → /robot_0/camera/{rgb,depth,camera_info}
                                        ├─ dynamic_obstacle_filter → depth_static
                                        ├─ RTAB-Map (visual odometry → /robot_0/odom)
                                        └─ map_merge, fleet_radio, recon relays
```

## Why a container

JetPack 6 is Ubuntu 22.04, and the ROS 2 for 22.04 is Humble. This repo is
Jazzy. You can't upgrade Ubuntu in place on a Jetson, because the L4T BSP (kernel,
GPU driver, CUDA) is tied to the release. So Jazzy runs in
`docker/jetson/Dockerfile` on top of NVIDIA's supported host. The host's
`/opt/ros/humble` is **never** used for SortBots: don't source it, and don't run
its `realsense2_camera_node` (only one process can own the D435).

## Host setup (once)

1. **Docker + NVIDIA runtime:** both ship with JetPack. `docker info | grep nvidia`
   should list the runtime. Add yourself to the docker group
   (`sudo usermod -aG docker $USER`, then log in again).
2. **RealSense udev rules:** these make the camera's device nodes world-readable,
   so the container (running as your uid) can open them.
   Check with `ls /lib/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules`.
   The host's `ros-humble-librealsense2` package installs them. If neither file exists,
   install them from librealsense (`config/99-realsense-libusb.rules` →
   `/etc/udev/rules.d/`, then `sudo udevadm control --reload && sudo udevadm trigger`).
3. **USB 3:** `lsusb -t` must show the D435 at `5000M`. At `480M` (a USB 2
   port or hub, or a USB 2 cable) the driver logs "connected using a 2.1 port",
   rejects 848×480@30, and falls back to 640×480@15. That still works, but it's
   half the frame rate and not the sim's resolution. This happened on 2026-09-28
   behind a USB 2 hub. Moved to a USB 3 port on 2026-09-30, the driver reports
   "USB type: 3.2" and opens 848×480@30 for both streams.
4. **Power mode:** `sudo nvpmodel -m 2 && sudo jetson_clocks` (MAXN SUPER on the
   Orin Nano Super). RTAB-Map + visual odometry is CPU-bound on this board.

The ROS-packaged librealsense (`ros-jazzy-librealsense2`) uses the **V4L2**
backend. It opens the host kernel's `uvcvideo` nodes (`/dev/video*`,
`/dev/media*`), which `scripts/jetson.sh` passes into the container. JetPack's
stock `uvcvideo` handles color + depth. It lacks the RealSense metadata patches,
so the driver timestamps frames with system time, which is fine for RTAB-Map.

## Run

```bash
scripts/jetson.sh build      # first time, ~10-20 min
scripts/jetson.sh console    # dashboard console, detached in container 'sortbots'
scripts/jetson.sh status     # container / camera / dashboard
```

Open `http://<jetson>:8081/`, go to **Scenarios**, and start:

- **`real_map_fresh`:** camera + RTAB-Map with visual odometry, fresh map. Carry
  or push the robot slowly. Fast turns and blank walls lose visual odometry.
- **`real_localize`:** read-only against a saved map, loaded by copy as in sim.

Save a map with the dashboard's map-save button while the stack is up, or
`scripts/maps.sh save NAME` from `scripts/jetson.sh shell`.

```bash
scripts/jetson.sh shell      # ROS sourced, inside the container
  ros2 topic hz /robot_0/camera/rgb                   # ~30 Hz
  ros2 topic echo --once /robot_0/camera/camera_info  # K ≈ [~605, 0, ~424, ...]
  ros2 run tf2_ros tf2_echo map robot_0/camera_color_optical_frame   # full chain resolves
scripts/jetson.sh logs       # console output; the bringup log is /tmp/sortbots_robot_bringup.log
scripts/jetson.sh stop       # pipeline + console + container
```

## Viewing it remotely

Reach the dashboard over **Tailscale**, never a public tunnel. `serve.py
--control` binds 0.0.0.0 with no authentication and can launch processes, so
the tailnet's ACLs are its only perimeter. The Jetson is already on the tailnet
(`jetson-orin`). From any device signed in to the same tailnet (phone, laptop,
anywhere), open:

    http://jetson-orin.tail0d28f9.ts.net:8081/     (IP fallback: http://100.123.112.59:8081/)

`scripts/jetson.sh console` and `status` print this URL. The container borrows
the host's tailscale CLI and socket read-only so `webui_url.py` can print it
from inside too. Use plain `http`: the page talks to rosbridge (`ws://…:9090`)
and web_video_server (`…:8080`) on the same host, and an https origin would
make the browser block those as mixed content.

### On the Jetson's own screen: snap browsers need `SNAP_REEXEC=0`

`chromium` / `firefox` on this Jetson die with "snap-confine is packaged
without necessary permissions … cap_dac_override not found". The cause is the
snapd **snap** (2.76.3). It re-executes its own `snap-confine`, which gets its
privileges from file capabilities stored as squashfs xattrs, and the L4T kernel
is built with `# CONFIG_SQUASHFS_XATTR is not set`, so the capabilities vanish.
The Ubuntu package's copy (`/usr/lib/snapd/snap-confine`, on ext4) keeps them,
and `SNAP_REEXEC=0` makes snap use that one (verified 2026-09-30):

    SNAP_REEXEC=0 chromium --app=http://localhost:8081/

To make it permanent for your account, put `SNAP_REEXEC=0` in
`~/.config/environment.d/90-snap-reexec.conf` (desktop and terminals, after you
log in again) and `export SNAP_REEXEC=0` in `~/.profile` (SSH). For every user on
the machine: `echo SNAP_REEXEC=0 | sudo tee -a /etc/environment`. The snap
browser still can't reach the Jetson's GPU (`GLDisplayEGL::Initialize failed`),
so pages render on the CPU. Prefer viewing the 3D panel from another device.

**3D reconstruction tab says "needs WebGL"** — with no GPU, headed Chromium
gives up on WebGL entirely (headless doesn't, which makes it confusing to test).
Force software WebGL (SwiftShader) AND turn off GPU compositing; verified
2026-10-02, `NOGL` → `OK WebGL 2.0`:

    SNAP_REEXEC=0 chromium --disable-gpu --enable-unsafe-swiftshader \
      --ignore-gpu-blocklist --app=http://localhost:8081/

`--disable-gpu` is not optional polish. With `--use-gl=angle --use-angle=
swiftshader` instead (WebGL works, but the whole page is composited through
SwiftShader) Chromium's GPU process ate ~175% CPU on this CPU-bound board; the
page's main thread stalled for 2-4 s at a time and ticked ~3 times in 10 s, so
the camera tiles ran ~1.4 fps and read as a badly lagging feed even though the
camera, ROS and web_video_server were ~60 ms behind. With `--disable-gpu` the
same page ticks at full rate (worst stall <60 ms) and WebGL still works.

Fully quit the old window first, or the flags are ignored by the running
instance. Expect a slow, CPU-bound 3D view (it defaults to points here).

### Photoreal 3D: Gaussian splats are built on the workstation

The 3D panel's **splat** mode shows a Gaussian splat of a saved map. It is
NEVER trained on this board. After a run, save the map as usual. Then, on the
workstation, run
`scripts/splat.sh build NAME --from http://jetson-orin.tail0d28f9.ts.net:8081`.
The workstation pulls the pose graph once over the tailnet (`GET
/api/maps/NAME/db`) and trains on its own GPU. The browser then fetches the
splat straight from the workstation, so the Jetson never stores or serves it.
Set `worker_url` in `configs/splat.yaml` to the workstation's tailnet name, and
view the result from a laptop, not this screen. See `docs/splat.md`.

## What differs from sim, and where it's configured

| | sim | real |
|---|---|---|
| launcher | `scripts/run_demo.sh` | `scripts/run_robot.sh` |
| scenarios | `platform: sim` (default) | `platform: real`, allowlist `REAL_RUN_FLAGS` in `webui/session.py` |
| camera | Isaac render product | `launch/sortbots_realsense.launch.py`, `configs/sensors/d435_real.yaml` |
| depth encoding | 32FC1 m | 16UC1 mm (the depth filter converts) |
| odometry | Isaac, perfect | `rgbd_odometry` on RAW depth (`camera_odometry:=true`); SLAM maps from filtered `depth_static` |
| IMU | sim MPU6050 | none (plain D435), `wait_imu_to_init:=false` |
| time | `use_sim_time:=true` | `use_sim_time:=false` |
| map anchor | `configs/robots.yaml` `spawn.nvidia` | `spawn.real` (origin) |
| Nav2 / explorer / task_manager | on | off: no base driver yet |
| rates seen live (USB 3) | — | 848×480@30 from the driver; factory K ≈ [607.6, 421.2, 246.7] vs the sim's [605, 424, 240]; visual odom ~4.5 Hz; container ~1 GB / ~3 of 6 cores |
| DDS | domain 0, LAN | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |

**Measure the camera mount.** `configs/sensors/d435_real.yaml` → `mount` is a
placeholder (1 m straight up, level). Put in the real `base_link → camera_link`
offset, or the map's floor plane and obstacle heights will be off by the error.

**Localhost-only DDS** is deliberate. The dashboard runs on the Jetson, so
nothing needs to leave it, and a desktop sim on the same network (which also
publishes `/robot_0/*`) can't cross-talk with the robot. Remove it when robots
need to talk to each other over the LAN.

## Next hardware steps

- **Base driver** (`nodes/base_driver.py`): `/<id>/cmd_vel` → motors, publishing
  `/<id>/odom` + `odom → base_link`. Then set `camera_odometry` back to false for
  real (two odom sources fight over one TF edge). After that, turn on Nav2, the
  explorer and task_manager in `run_robot.sh`, and add their keys to `REAL_RUN_FLAGS`.
- **MPU6050:** a driver publishing raw `/<id>/imu/data_raw`, plus
  `imu_filter_madgwick` (already in the image) to `/<id>/imu`. Then
  `wait_imu_to_init:=true`.
- **cuVSLAM** (Isaac ROS) in place of RTAB-Map's visual odometry.
