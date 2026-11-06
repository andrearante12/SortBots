# Setup

## Install

**Requirements:** Ubuntu 22.04 or 24.04 on an **X11** session (Kit and SAPIEN
misbehave on Wayland). An NVIDIA RTX GPU with ≥ 8 GB VRAM and driver R535–R580.
About 30 GB free (50 GB with ManiSkill). Python 3.11 for the Isaac venv.

```bash
# 1. system packages
sudo apt-get install -y libvulkan1 vulkan-tools libglvnd-dev mesa-utils libegl1 libgl1 \
    libxext6 libx11-6 libxcb-cursor0 libxkbcommon-x11-0 libglu1-mesa \
    build-essential git curl ca-certificates git-lfs
git lfs install
vulkaninfo --summary | grep -i nvidia        # must list your GPU, not just llvmpipe

# 2. clone (git-lfs pulls the saved maps; without it they are 130-byte pointer files)
git clone --recurse-submodules <repo-url> sortbots_ws && cd sortbots_ws
git lfs pull

# 3. Isaac Sim 5.1, in a clean shell (no conda, no ROS)
bash scripts/install_isaac_sim.sh                     # preflight only
bash scripts/install_isaac_sim.sh --accept-download   # ~15 min, idempotent → ~/isaacsim/venv
source scripts/activate_isaac.sh && python scripts/verify_isaac_sim.py   # "OK"; close the shell after

# 4. ROS 2 Jazzy (add the apt repo per docs.ros.org first)
sudo apt install -y ros-jazzy-desktop ros-jazzy-rtabmap-ros ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup ros-jazzy-rosbridge-suite ros-jazzy-web-video-server \
    ros-jazzy-depth-image-proc
```

The first run auto-builds the robot USD (`scripts/import_urdf.py`) and streams the
NVIDIA warehouse assets, which takes a few minutes. Both are cached after that.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `vulkaninfo` shows only llvmpipe / verify hangs at 120 s | NVIDIA Vulkan ICD missing: reinstall `nvidia-driver-*`, re-source `activate_isaac.sh` from a clean shell |
| `numpy.core.multiarray failed to import`, Kit on llvmpipe | ROS 2 is sourced in this shell. Open a new one |
| `No module named 'rclpy._rclpy_pybind11'` | conda's python3 on PATH: prepend `/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin` |
| Black / missing Kit window | Wayland: log in with "Ubuntu on Xorg" |
| GUI out of memory on 8 GB | Kit alone takes ~6 GB: close other GPU apps, `--robots 1`, run headless |
| Map greyed out "run `git lfs pull`" | git-lfs wasn't installed at clone time |
| Dashboard up, no camera / "stale" | `ss -tlnp \| grep -E ':8080\|:8081\|:9090'`: all three should be listening |
| Cameras die around sim t≈120 s | something polls `/snapshot` without `qos_profile=sensor_data` |
| A node "hangs" with sim time on | its timer runs on the sim clock (see Timers in §5) |
| Disk creeps up | Kit caches in `~/.cache/ov` and `~/.nvidia-omniverse/` are safe to delete |

---
