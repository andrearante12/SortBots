# Quickstart — fresh device to a live fleet

One linear path: clone → install → a collaborating two-robot fleet exploring a
warehouse in the browser. This is the only setup doc; everything else
([`running.md`](running.md), the `isaac_sim_phase*.md` notes) is reference you
reach for after this works.

The **primary track is Isaac Sim** (mobile / multi-robot / ROS 2 / SLAM). The
GUI you drive is the **web dashboard** at `http://localhost:8081/` — not rviz,
not Isaac's own Kit window. rviz is off by default; you never touch the Kit UI
for a normal run. The **ManiSkill** arm-manipulation track is optional and
lives at the end.

## 0. Requirements

- **OS:** Ubuntu 22.04 or 24.04 (verified on 24.04), **X11** session
  (`echo $XDG_SESSION_TYPE` → `x11`; Kit and SAPIEN misbehave on Wayland).
- **GPU:** NVIDIA with RT cores, **≥ 8 GB VRAM** (8 GB is the documented floor
  and is tight with the GUI up). Driver **R535–R580**.
- **Disk:** ~30 GB free on `$HOME` (Isaac wheels + Kit cache). +~20 GB for the
  optional ManiSkill track.
- **Python 3.11** for the Isaac venv: `python3.11` on PATH (deadsnakes PPA) or
  `~/miniconda3/envs/lerobot/bin/python3.11` (exists if you did ManiSkill).

> **Never `source /opt/ros/jazzy/setup.bash` in a shell that runs Isaac or
> conda**, and never put it in `~/.bashrc`. It injects a Python / library path
> that shadows both the Isaac venv and the conda env (symptoms:
> `numpy.core.multiarray failed to import`, Kit falling back to llvmpipe).
> `install_isaac_sim.sh`, `activate_isaac.sh` and `install.sh` all refuse to
> run when ROS 2 is sourced. The `run_*.sh` scripts source it in their own
> subshells; you only source it by hand in a dedicated shell.

## 1. System packages

```bash
sudo apt-get update
sudo apt-get install -y \
    libvulkan1 vulkan-tools libglvnd-dev mesa-utils \
    libegl1 libgl1 libxext6 libx11-6 \
    libxcb-cursor0 libxkbcommon-x11-0 libglu1-mesa \
    build-essential git curl ca-certificates git-lfs

git lfs install    # once per machine — the saved-map library ships .db via LFS
```

Confirm Vulkan finds the NVIDIA GPU, not just `llvmpipe`:

```bash
vulkaninfo --summary | grep -i nvidia    # must list your GPU
```

If only software/llvmpipe appears, the NVIDIA Vulkan ICD isn't registered —
reinstall `nvidia-driver-*`.

## 2. Clone

```bash
git clone --recurse-submodules <this-repo-url> sortbots_ws
cd sortbots_ws
# already cloned flat?  git submodule update --init --recursive
git lfs pull        # replace the maps/*.db pointer files with the real blobs
```

A clone without git-lfs leaves ~130-byte pointer files in `maps/`; they look
present but the dashboard disables those maps and `run_demo.sh` refuses to load
them. The fix is always `git lfs pull`.

## 3. Install Isaac Sim 5.1

Clean shell — no conda env active, no ROS 2 sourced:

```bash
bash scripts/install_isaac_sim.sh                   # preflight + plan, installs nothing
bash scripts/install_isaac_sim.sh --accept-download # ~15 min, idempotent
```

It preflights GPU / RT cores / VRAM / driver / Vulkan / Ubuntu / disk, locates
Python 3.11, creates `~/isaacsim/venv`, installs `isaacsim[all]==5.1.0` from
`https://pypi.nvidia.com`, and smoke-tests `import isaacsim`. Re-running with
`--accept-download` prints "skipping" for finished stages.

Verify:

```bash
source scripts/activate_isaac.sh
python scripts/verify_isaac_sim.py                  # steps an empty stage 100x
```

`activate_isaac.sh` sets the PRIME-offload vars (dGPU on hybrid-graphics
laptops), pre-accepts the Kit EULA/privacy prompts, and activates the venv; it
refuses inside a conda env or a ROS-sourced shell. Verify prints
`verify_isaac_sim: OK` (exit 0). A hang that times out at 120 s means Kit fell
back to llvmpipe — re-source from a clean shell. Then close this shell; don't
reuse it for the ROS side.

## 4. Install ROS 2 Jazzy + the demo stack

The demo runs RTAB-Map SLAM + Nav2 and serves the dashboard from **system
ROS 2 Jazzy** (separate from Isaac's bundled bridge in the venv). Follow the
[official Jazzy apt guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
to add the repo, then:

```bash
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-web-video-server \
    ros-jazzy-depth-image-proc
```

`desktop` gives rclpy/rviz2; `rtabmap-ros` is SLAM; `navigation2` +
`nav2-bringup` are Nav2; `rosbridge-suite` + `web-video-server` are what the
dashboard talks to; `depth-image-proc` feeds RTAB-Map. Sanity check, then
close the shell:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix rtabmap_launch nav2_bringup rosbridge_server web_video_server
```

## 5. First run — the fleet, from the dashboard

From a **fresh clean shell** (no ROS, no conda, no Isaac — the scripts source
what each half needs):

```bash
scripts/run_console.sh
```

Leave that terminal running. It brings up rosbridge (9090), web_video_server
(8080) and `serve.py` (8081), then prints:

```
SortBots dashboard console is UP
  * Dashboard : http://localhost:8081/   (or the tailnet URL)
```

1. Open **`http://localhost:8081/`**.
2. Switch the header toggle to **scenarios**.
3. Pick **`explore_fleet`** and hit **Start**.

`explore_fleet` spawns **two robots** at their `configs/robots.yaml` positions,
each running its own RTAB-Map, fused into one world-anchored `/map`
(`nodes/map_merge.py`) so both explorers pick frontiers against space *either*
robot has mapped. They deconflict over mesh radio (`/fleet/intent` +
`/fleet/status`), not a TF oracle. This is the project's default operating
mode; `explore_fresh` is the single-robot variant.

The tab streams the launcher output and a phase readout:

```
starting Isaac Sim → loading the warehouse → sim publishing
  → starting RTAB-Map + Nav2 → ROS 2 stack up → running
```

The **first** run streams the NVIDIA warehouse assets — allow a few minutes at
"loading the warehouse" (cached afterward). At **running** the map view starts
filling in and both robots explore hands-off. Chase cams are off by default in
fleet runs (the 3rd-person render product costs ~30% of real-time factor); the
dashboard shows robot_0's head cam. Tick **chase_cam** on the card to get the
3rd-person view back.

- **camera / map** buttons (header) swap the stage between camera and the
  occupancy grid. In map mode, drag on the grid to send a nav goal.
- **Robot switcher** (header) reloads the page scoped to the other robot.
- **W/A/S/D** drive, **Q/E** rotate, **Space** stops — anywhere on the page.
  Don't drive while exploration or a task is active; Nav2 will fight you.
- **Dispatch & task queue** (right column): **Start / Stop** for exploration
  and the pickup→dropoff form.

## 6. Stop

- **Stop** in the scenarios tab — tears the run down, leaves the console and
  your open page up for the next run.
- `scripts/run_console.sh stop` — takes down the console *and* any running sim.
- Ctrl-C in the console terminal — stops the console only; a running sim keeps
  going.

## CLI / scriptable path

The dashboard is optional. `run_demo.sh` brings the whole pipeline up from one
clean-shell command and also defaults to the **2-robot fleet**:

```bash
scripts/run_demo.sh                 # fleet + dashboard, no exploration
scripts/run_demo.sh --explore       # ...and explore autonomously
scripts/run_demo.sh --robots 1      # single robot instead
scripts/run_demo.sh stop            # tear it all down
```

Everything the scenarios tab does is also `scripts/sim_ctl.sh`, with exit codes
as the interface (`0` ok, `1` failure, `3` no console, `4` nothing running,
`124` timeout):

```bash
scripts/sim_ctl.sh console start
scripts/sim_ctl.sh start explore_fleet headless=true
scripts/sim_ctl.sh wait running --timeout 420    # first run streams assets
scripts/sim_ctl.sh status
scripts/sim_ctl.sh stop
```

Coding agents should use the **`sim-run` skill**, which carries this runbook
plus the failure modes. Full run reference — every scenario, map lifecycle,
exploration tuning, remote access, dashboard tests — is in
[`running.md`](running.md). Implementation notes per phase:
[`isaac_sim_phase2.md`](isaac_sim_phase2.md) (URDF→USD),
[`isaac_sim_phase3.md`](isaac_sim_phase3.md) (ROS 2 bridge),
[`isaac_sim_phase4.md`](isaac_sim_phase4.md) (sensors + RTAB-Map TF).

## Troubleshooting

- **`vulkaninfo` lists only llvmpipe / verify hangs at 120 s** — the NVIDIA
  Vulkan ICD isn't picked. Reinstall `nvidia-driver-*`; confirm
  `__NV_PRIME_RENDER_OFFLOAD=1 vulkaninfo --summary` lists the dGPU; re-source
  `activate_isaac.sh` from a clean shell.
- **GUI OOM on 8 GB** — Kit alone takes ~6 GB. Close browsers / other GPU apps,
  run `--robots 1`, and drop the camera resolution in
  `configs/sensors/d435.json` if Kit spikes.
- **Kit aborts with a driver mismatch** — R535–R580 is the tested range for 5.1.
- **Disk creeps up** — Kit caches in `~/.cache/ov` and `~/.nvidia-omniverse/`;
  safe to delete, regenerated on next run.
- **Window black / never appears** — likely Wayland. Log in via "Ubuntu on
  Xorg", or prefix with `XDG_SESSION_TYPE=x11`.
- **Dashboard loads but no camera / "stale" badge** — check the console
  terminal for errors; `ss -tlnp | grep -E ':8080|:8081|:9090'` should show all
  three listening.
- **A map card is greyed out with "run `git lfs pull`"** — step 1/2 git-lfs was
  skipped.
- **`rosbridge_websocket` crashes with `No module named
  'rclpy._rclpy_pybind11'`** — conda's Python is on PATH; run from a shell where
  `which python3` is `/usr/bin/python3`.

---

## Optional: ManiSkill (arm-manipulation track)

A separate conda env for the VLA / arm-manipulation half of the project. Skip
unless you're working on manipulation. **End state:** `bash
scripts/verify_sim.sh` opens two SAPIEN windows showing Fetch and XLeRobot
moving in the ReplicaCAD apartment.

### Miniconda

```bash
curl -fsSL -o /tmp/miniconda.sh \
    https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash /tmp/miniconda.sh -b -p "$HOME/miniconda3"
"$HOME/miniconda3/bin/conda" init bash       # then open a new terminal
# Recent Miniconda requires accepting Anaconda's ToS before env creation:
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
```

### Bootstrap + verify

```bash
bash scripts/install.sh        # conda env `lerobot` + mani-skill + ReplicaCAD (~1.6 GB) + XLeRobot overlay
conda activate lerobot
bash scripts/verify_sim.sh
```

`install.sh` is idempotent (~3–10 min first run) and refuses if ROS 2 is
sourced. The §1 apt deps already cover ManiSkill. Demo catalog and
shader/Wayland tips: [`running.md`](running.md).
