# Setup

End state: `./scripts/run_demo.sh` brings up the full warehouse SLAM demo —
Isaac Sim (NVIDIA warehouse + XLeRobot), RTAB-Map + rviz, and WASD teleop.

The primary track is **Isaac Sim** (mobile / multi-robot / ROS 2 / SLAM). The
**ManiSkill** arm-manipulation track is optional and lives at the end.

Isaac Sim installs into `~/isaacsim/` with its own Python 3.11 venv, separate
from everything else. **Never activate two stacks in one shell, and never
source ROS 2 in an Isaac or conda shell** — it rewrites `PYTHONPATH` /
`LD_LIBRARY_PATH` and breaks both (see [ROS 2 isolation](#ros-2-isolation-important)).

## Requirements

- **OS:** Ubuntu 22.04 or 24.04 (verified on 24.04), X11 session
  (`echo $XDG_SESSION_TYPE` → `x11`; Kit and SAPIEN misbehave on Wayland).
- **GPU:** NVIDIA with RT cores, **≥ 8 GB VRAM** (8 GB is the documented floor —
  headless is fine, but GUI + rviz + RTAB-Map together are tight). Driver
  **R535+**, with the Vulkan ICD pointing at the NVIDIA GPU.
- **Disk:** ~30 GB free on `$HOME` (Isaac wheels + Kit cache). Add ~20 GB if you
  also do the optional ManiSkill track.
- **Python 3.11** for the Isaac venv — either `python3.11` on PATH (deadsnakes
  PPA) or `~/miniconda3/envs/lerobot/bin/python3.11` (which exists if you set up
  ManiSkill).

## 1. Apt prerequisites

```bash
sudo apt-get update
sudo apt-get install -y \
    libvulkan1 vulkan-tools libglvnd-dev mesa-utils \
    libegl1 libgl1 libxext6 libx11-6 \
    libxcb-cursor0 libxkbcommon-x11-0 libglu1-mesa \
    build-essential git curl ca-certificates
```

Confirm Vulkan finds the NVIDIA GPU (not just `llvmpipe`):

```bash
vulkaninfo --summary | grep -i nvidia    # must list your GPU
```

If only software/llvmpipe appears, the NVIDIA Vulkan ICD isn't registered —
reinstall `nvidia-driver-*`.

## 2. Clone with submodules

```bash
git clone --recurse-submodules <this-repo-url> sortbots_ws
cd sortbots_ws
# already cloned flat?  git submodule update --init --recursive
```

## 3. Install Isaac Sim 5.1

From the repo root, in a clean shell (no conda env active, no ROS 2 sourced):

```bash
bash scripts/install_isaac_sim.sh                   # prints the plan, exits 0
bash scripts/install_isaac_sim.sh --accept-download # actually installs
```

It preflights GPU / RT cores / VRAM / driver / Vulkan / Ubuntu / disk, locates
Python 3.11, creates `~/isaacsim/venv`, installs `isaacsim[all]==5.1.0` from
`https://pypi.nvidia.com`, and smoke-tests `import isaacsim`. Idempotent —
re-running with `--accept-download` prints "skipping" for finished stages.

## 4. Activate + verify

```bash
source scripts/activate_isaac.sh
python scripts/verify_isaac_sim.py
```

`activate_isaac.sh` sets the PRIME-offload vars (route Kit at the dGPU on
hybrid-graphics laptops), pre-accepts the Kit EULA/privacy prompts, and
activates the venv; it refuses inside a conda env or a ROS-sourced shell.
Verify steps an empty stage 100× and prints `verify_isaac_sim: OK` (exit 0). A
hang that times out at 120 s means Kit fell back to llvmpipe — re-source from a
clean shell.

## 5. ROS 2 Jazzy + RTAB-Map (for the SLAM demo)

The demo drives the robot from a system ROS 2 node and runs RTAB-Map + rviz, so
it needs **ROS 2 Jazzy** + **RTAB-Map** installed system-wide (separate from
Isaac's bundled bridge, which lives in the Isaac venv). Skip this if you only
want the headless verify above.

Install ROS 2 Jazzy per the
[official guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
or run its apt-repo setup:

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-rtabmap-ros
```

`ros-jazzy-desktop` brings rclpy, rviz2, and rmw_fastrtps_cpp; the WASD teleop
(`scripts/wasd_teleop.py`) is plain rclpy, no extra package. Sanity check:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix rtabmap_launch    # prints a path if installed
```

Then close that shell — don't reuse it for Isaac.

## 6. Run the demo

From a clean shell (the script sources Isaac vs. ROS itself):

```bash
./scripts/run_demo.sh          # Isaac Sim + RTAB-Map + rviz + WASD teleop
./scripts/run_demo.sh stop     # tear it all down
```

Drive with WASD; the map fills in live in rviz. The first run streams the
NVIDIA warehouse (a few minutes; cached after). Implementation notes per phase:
[`isaac_sim_phase2.md`](isaac_sim_phase2.md) (URDF→USD import),
[`isaac_sim_phase3.md`](isaac_sim_phase3.md) (ROS 2 bridge),
[`isaac_sim_phase4.md`](isaac_sim_phase4.md) (sensors + RTAB-Map TF plumbing).

## ROS 2 isolation (important)

Do **not** `source /opt/ros/jazzy/setup.bash` in `~/.bashrc`. It injects
Python 3.12 / `AMENT_PREFIX_PATH` that shadow both the Isaac venv and the conda
env (symptoms: `numpy.core.multiarray failed to import`, Kit falling back to
llvmpipe). `install_isaac_sim.sh`, `activate_isaac.sh`, and `install.sh` all
refuse to run when ROS 2 is sourced. Source it only in its own shell — or let
`run_demo.sh`, which sources it in subshells, handle it.

## Troubleshooting

- **`vulkaninfo` lists only llvmpipe / verify hangs at 120 s:** the NVIDIA
  Vulkan ICD isn't picked. Reinstall `nvidia-driver-*`; confirm
  `__NV_PRIME_RENDER_OFFLOAD=1 vulkaninfo --summary` lists the dGPU; re-source
  `activate_isaac.sh` from a clean shell.
- **GUI OOM on 8 GB:** Kit alone takes ~6 GB. Close browsers / other GPU apps,
  run the demo with one robot, and drop the camera resolution in
  `configs/sensors/d435.json` if Kit + rviz spike.
- **Kit aborts with a driver mismatch:** R535–R580 is the tested range for 5.1.
- **Disk creeps up:** Kit caches in `~/.cache/ov` and `~/.nvidia-omniverse/` —
  safe to delete; regenerated on next run.
- **Window black / never appears (any track):** likely Wayland — log in via
  "Ubuntu on Xorg", or prefix with `XDG_SESSION_TYPE=x11`.

---

## Optional: ManiSkill (arm-manipulation track)

ManiSkill drives the VLA / arm-manipulation half of the project in a separate
conda env. Skip unless you're working on manipulation.

**End state:** `bash scripts/verify_sim.sh` opens two SAPIEN windows showing
Fetch and XLeRobot moving in the ReplicaCAD apartment.

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
