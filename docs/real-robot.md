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
   driver falls back to 640×480@15. Put the arm, base and magnet serial boards
   on the **USB-A** ports too. **Never boot with a device on the USB-C port.**
   With a serial board on USB-C, the port comes up in host mode and the Jetson
   hangs in early boot. This was seen 2026-11-16: the kernel stopped 12.7 s in,
   right after `cdc_acm 1-1`. Hot-plugging the arm board into USB-C after boot
   worked in a controlled test, so USB-C is usable if the board is unplugged
   before every reboot or power-on. The USB-C driver also logs
   `VDD_5V0_SYS: Underflow of regulator enable count` on unplug, so treat that
   port as fragile. Its normal use is the laptop recovery cable.
4. Set the power mode: `sudo nvpmodel -m 2 && sudo jetson_clocks`.

```bash
scripts/jetson.sh build      # first time, 10–20 min
scripts/jetson.sh console    # dashboard console in container 'sortbots'
scripts/jetson.sh status | shell | logs | stop
```

Open `http://jetson-orin.<tailnet>.ts.net:8081/` and start `real_map_fresh`. Move
the robot slowly: fast turns and blank walls lose visual odometry.

**Power-on order:** the battery feeds the Jetson and the servo boards together. Switch on the
**servo boards first**, wait 3–5 s, then the **Jetson**. Both at once, the servos' inrush sags the
supply while the Jetson boots, and the boot sometimes hangs: a UEFI firmware call that never
returns (`Workqueue: efi_rts_wq` in the kernel log), Tailscale stuck behind it, the robot alive
but unreachable (seen 2026-11-16). Shutting down: Rest, then the Jetson, then the servo boards.
A separate regulator or a delay relay for the Jetson would remove the ordering rule.

**Unattended (survives reboots and power cuts):** once the Wi-Fi profile works,

```bash
sudo scripts/jetson_host_setup.sh     # once; rerun after it changes
```

It makes Wi-Fi retry forever with power save off, keeps the journal across
reboots, holds tailscaled until the network is up, adds a watchdog (Wi-Fi and
Tailscale, every 30 s), and installs `sortbots-console.service`, which runs
`jetson.sh console` at boot. A boot never pulls: the Jetson comes back on the
branch and commit it was running, so a power blip can't swap a branch under test
for `main`. The dashboard is up about a minute after power-on with nobody logged
in. Also disable key expiry for the Jetson in the Tailscale
admin console, or it falls off the tailnet when the key expires.

**Updating the code** is by hand only. The checkout is only ever fast-forwarded:

```bash
scripts/jetson.sh update          # pull main, rebuild the image if docker/jetson changed, restart the console
scripts/jetson.sh update BRANCH   # same, for a branch under test
```

It refuses if the checkout has uncommitted changes or has diverged from origin.
Develop elsewhere and push. Don't commit on the Jetson.

**No network:** with the USB-C port plugged into a laptop, the Jetson is at
`ssh andre@192.168.55.1`. If eduroam rejects the saved password (`error 691` in
`journalctl -u wpa_supplicant`), store the new one:
`sudo nmcli con modify eduroam 802-1x.password "$(read -rsp 'password: ' p; echo "$p")"`.

**Differences from the sim:**

- `use_sim_time:=false`.
- Visual odometry on raw depth instead of Isaac's perfect odometry.
- No IMU (`wait_imu_to_init:=false`).
- Depth is 16UC1 mm.
- The map anchor is `spawn.real`.
- Nav2, the explorer and task_manager run only with `--nav` (`real_explore`, `real_nav`):
  Nav2 is real CPU on the Orin next to RTAB-Map. `nodes/base_driver.py` turns `cmd_vel`
  into wheel speeds. magnet_pick never runs: the real pick is `nodes/arm_driver.py`'s.
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

### Point-to-point tasks

Operator task → `/robot_0/dispatch_task` → `nodes/task_manager.py` → Nav2
`navigate_to_pose` → `cmd_vel` → `nodes/base_driver.py` → wheels. The same path as the sim.
The only real-robot difference is that the whole chain is behind `run_robot.sh --nav`.

1. **Map the space** with `real_map_fresh` (or `real_explore`), then save it from the Map
   button. Waypoints are poses in the map frame, and a fresh map's frame starts wherever
   the robot stood, so pins only carry over between runs on a saved map.
2. **Start `real_nav`** and pick that map. Start the robot somewhere it has mapped, with the
   camera on something textured, and wait until it shows on the map, localized. Nav2 can't
   plan before that, and task_manager retries a rejected first leg for 90 s only.
3. **Place waypoints** on the map (they save with the map), then in **Tasks** queue a move
   (`goto`) or a pick-up → drop-off. Pick-up and drop-off are 2 s dwells on the robot: no
   arm. Task state streams on `/robot_0/task_status`.

From a shell in the container (`scripts/jetson.sh shell`), the same without the GUI:

```bash
ros2 topic pub --once /robot_0/dispatch_task std_msgs/String '{data: "{\"goto\": \"A\"}"}'
ros2 topic echo /robot_0/task_status
ros2 topic pub --once /robot_0/cancel_task std_msgs/String '{data: all}'
```

Speed is capped at 0.15 m/s (`configs/nav2_real.yaml`) under base_driver's own cap. The
E-STOP latches in base_driver and stops Nav2's output from reaching the wheels.

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

- Wheel odometry from the base driver (`odom`): the base drives open-loop today, and
  Nav2's odometry is RTAB-Map's visual odometry alone.
- An MPU6050 driver feeding `imu_filter_madgwick`.
- cuVSLAM, which needs JetPack 7.2.
- Mesh radios (see [Fleet comms](commsim.md)).

## The arm, pick and place, and the Robot view on the Jetson

Everything in `docs/drive.md` and `docs/head_camera.md` runs on the Jetson as it does on the
laptop: same robot, same boards, same calibration. The Jetson is just the computer it's plugged
into. Three things differ: the lerobot env is miniforge's, the ROS console wants the same camera
and port, and the browser is on another machine.

**Once per Jetson:**

1. The `lerobot` env is at `~/miniforge3/envs/lerobot` (not miniconda). `scripts/arm.sh` and
   the Robot view find either. It has lerobot, numpy, PyYAML and OpenCV.
2. **It has no `pyrealsense2` yet** (checked 2026-11-16), and `handeye pick` and the Robot view's
   camera both need it. Try `~/miniforge3/envs/lerobot/bin/pip install pyrealsense2`; if pip has
   no aarch64 wheel for Python 3.12, build librealsense with `-DBUILD_PYTHON_BINDINGS=ON` against
   that env's python. The ROS container has its own RealSense driver; it doesn't help the host env.
3. Put the calibration where the tools look: `scripts/calibration.sh install`. The Jetson had an
   older arm calibration (2026-11-06); install keeps it as `sortbots_arm.before-install-*.json`.
4. The boards are found by USB serial, as on the laptop: arm `5B91029024`, base `5B79016064`,
   magnet ESP32 on the CP2102 (`ttyUSB0`). Check with `ls -l /dev/serial/by-id/`. A board that
   comes back `root:dialout 660` after a re-plug needs `sudo chmod 666` (docs/arm.md).

**Each session:**

```bash
# on the Jetson: the ROS console holds the D435 and port 8081, so stop it first
scripts/jetson.sh stop
~/miniforge3/envs/lerobot/bin/python webui/serve.py --drive     # host, NOT in the container

# on the laptop (where the browser and the gamepad are):
ssh -N -L 8081:localhost:8081 jetson-orin.tail0d28f9.ts.net
# then open http://localhost:8081/ -> Robot
```

The SSH tunnel is the point, not a workaround: the drive routes answer only a browser on the
robot's own machine, and refuse anything through Tailscale Serve/Funnel (serve.py has no
authentication). Through the tunnel the request arrives on the Jetson from localhost. The
gamepad stays plugged into the laptop: the browser reads it there and sends commands over the
tunnel.

The Robot view and the ROS stack (mapping, localization) can't run at once on the Jetson: both
need the D435. Stop one before starting the other. The console starts by itself at every boot
(`sortbots-console.service`, a one-shot: `jetson.sh stop` sticks until the next reboot), so
after a reboot stop it again before starting the Robot view.

## Gaussian splats

The operator view's **3D cloud** view has a **Splat** mode (`webui/operator_splat.js`): a
photoreal Gaussian splat of a saved map, next to the live point cloud. It is trained after
the run, **on a workstation GPU only, never the Jetson**, by the splat worker
(`splat/worker.py`, :8082). The pipeline: the map's RTAB-Map DB becomes keyframes + poses
(`splat/extract.py`), gsplat trains, and the result is anchored into the fleet `map` frame.

```bash
scripts/install_splat.sh [--check]     # once: conda env `splat`, or a uv venv where there is no conda
scripts/splat.sh serve                 # worker on :8082
scripts/sim_ctl.sh stop --save-map NAME --splat                     # sim
scripts/splat.sh build NAME --from http://jetson-orin.<tailnet>:8081 --wait   # real robot's map
scripts/splat.sh status | log JOB
```

Or from the GUI: 3D cloud → **Splat**, pick a saved map, **Build**. Progress shows in the
bar and the splat loads when it's done. `?recon=splat` opens straight into that mode.
The "waiting for the robots" card is hidden in splat mode: a saved map needs no robot online.

**The env.** `requirements-splat.txt` is the one package list (Python 3.10, torch 2.4.1+cu124,
gsplat 1.5.3: the newest pair gsplat ships prebuilt CUDA wheels for, so no `nvcc` is
needed). `install_splat.sh` uses conda when it exists and otherwise `uv` into
`~/.local/share/sortbots/splat-venv` (`scripts/_splat_env.sh`; `SORTBOTS_SPLAT_PYTHON`
overrides). A bare venv also needs `packaging` and `setuptools<70`, or gsplat's CUDA
backend fails to import; conda's pip happened to bring them.

**No ROS needed.** `extract.backend` in `configs/splat.yaml` (`auto`, the default) is
`splat/rtabmap_db.py` on every machine: it reads the sqlite DB directly (numpy + Pillow
+ scipy). Jazzy has no Ubuntu 26.04 packages, so on the RTX 4070 laptop it is the only
way, and it is the default where Jazzy exists too: `rtabmap-export` (opt in with
`backend: rtabmap-export`) always exports the optimized graph, which on the sim map below
was 6x less self-consistent than the poses the reader picked. It re-solves the pose graph (the DB keeps
only odometry poses), and with `dataset.poses: auto` it trains on whichever pose set,
optimized or odometry, agrees better with the keyframes' own depth. On
`fleet_warehouse_test` (Isaac) odometry won, 0.60 cm median disagreement vs 3.81 cm,
because that map's loop closures were ~10 cm off. A real robot's drifting visual odometry
should pick the optimized graph. The job log says which one it used.

**The real robot.** Save the map on the Jetson as usual. Open the Jetson's operator view
from the machine running the worker: `worker_url: http://localhost:8082` then reaches it,
and **Build** sends the Jetson's address as the source, so the worker pulls the DB over
the tailnet (`GET /api/maps/<name>/db`, sha256-checked). From any other browser, set
`?splat_worker=http://<workstation>:8082` or `worker_url`. The worker answers Chrome's
private-network preflight, which a Jetson-origin page calling `localhost` needs.

**Measured** (RTX 4070 Laptop, 2026-11-17): `fleet_warehouse_test`, 79 keyframes, 3 s
extract + 186 s training (15k steps), 1.42 M gaussians, 29.1 dB on the training views,
45 MB `.splat`. Earlier: the 1,157-keyframe `warehouse`, about 4 min and 42 MB. Jobs wait
while Isaac holds the GPU. Output goes to `~/.cache/sortbots/splats/`.

```bash
/usr/bin/python3 -m pytest tests/splat_*_test.py   # extract, dataset, convert, worker (no GPU)
node webui/tests/splat_test.mjs                     # renderer + Splat mode in headless Chrome
```
