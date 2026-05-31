# First-time setup

End state: an activated conda env called `lerobot` where `bash scripts/verify_sim.sh` opens two SAPIEN windows showing Fetch and XLeRobot moving inside the ReplicaCAD apartment.

## Requirements

- Linux (verified on Ubuntu 24.04, kernel 6.x). Other distros likely work; commands below assume `apt`.
- NVIDIA GPU with the proprietary driver installed. Verified on an RTX 4070 Laptop, 8 GB VRAM, driver 595.x. CPU-only is possible but slow.
- ~20 GB free disk for the conda env, mani-skill wheels, and the ReplicaCAD scene dataset.
- An X11 desktop session (`echo $XDG_SESSION_TYPE` should print `x11`). SAPIEN + GLFW can render black or fail to create a window under Wayland.

## 1. Apt prerequisites

```bash
sudo apt-get update
sudo apt-get install -y \
    libvulkan1 vulkan-tools libglvnd-dev mesa-utils \
    libegl1 libgl1 libxext6 libx11-6 \
    build-essential git curl ca-certificates
```

Verify Vulkan can find the NVIDIA ICD:

```bash
vulkaninfo --summary | head
# Expect to see "GPU0: NVIDIA GeForce ..." in the output
```

If `vulkaninfo` lists only software/llvmpipe, your NVIDIA Vulkan ICD isn't registered — usually a reinstall of `nvidia-driver-*` fixes this.

## 2. Miniconda

Install non-interactively (skip if `conda --version` already works):

```bash
curl -fsSL -o /tmp/miniconda.sh \
    https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash /tmp/miniconda.sh -b -p "$HOME/miniconda3"
"$HOME/miniconda3/bin/conda" init bash
# Open a new terminal so `conda` is on PATH.
```

Recent Miniconda releases require explicit acceptance of Anaconda's Terms of Service before env creation will succeed. Accept the two channels used by the base installer:

```bash
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
```

## 3. Clone with submodules

```bash
git clone --recurse-submodules <this-repo-url> sortbots_ws
cd sortbots_ws
```

If you already cloned without `--recurse-submodules`:

```bash
git submodule update --init --recursive
```

## 4. Bootstrap

```bash
bash scripts/install.sh
```

This is idempotent — re-running is safe. It:

1. Verifies no ROS 2 source has polluted `PYTHONPATH` / `AMENT_PREFIX_PATH` in this shell (see "ROS 2 caveat" below).
2. `git submodule update --init --recursive`.
3. Creates the conda env `lerobot` from `environment.yml` (Python 3.11 + `mani-skill`, `torch`, `pygame`, `rerun-sdk`). Skipped if the env already exists.
4. `pip install --upgrade` of the same pip deps as a defensive re-pin.
5. Downloads the ReplicaCAD scene dataset into `~/.maniskill/data/` (~1.6 GB, 872 files). Skipped if already present.
6. Runs `scripts/overlay_xlerobot.py` to wire the submodule's XLeRobot files into ManiSkill's `site-packages`. See [`overlay.md`](overlay.md) for what this does.

Expect a clean 3–10 minute run on a typical machine, faster on subsequent runs.

## 5. Verify

```bash
conda activate lerobot
bash scripts/verify_sim.sh
```

Successful run prints:

- `mani_skill 3.0.1, torch 2.12.0+cu130, cuda=True`
- `Xlerobot.uid = 'xlerobot'`
- `Headless OK; action space dim = 16`

Then opens two SAPIEN windows in sequence:

- **Demo 1:** Fetch wandering ReplicaCAD with random actions and the `rt-fast` ray-traced shader. Close the window to advance.
- **Demo 2:** XLeRobot (dual-arm) in ReplicaCAD with preset actions and the `default` shader.

If either window fails to appear, see "Display issues" below.

## ROS 2 caveat

If ROS 2 (e.g. Jazzy at `/opt/ros/jazzy`) is installed on this machine, do **not** source it in `~/.bashrc`. The `setup.bash` injects Python 3.12 paths into `PYTHONPATH` and `AMENT_PREFIX_PATH` that shadow the conda env's stdlib and break `numpy` with `ImportError: numpy.core.multiarray failed to import`. `install.sh` and `verify_sim.sh` refuse to run when those variables are set.

When you need ROS 2, source it explicitly in a separate shell:

```bash
source /opt/ros/jazzy/setup.bash
```

## Display issues

- **Window opens black or never appears:** likely Wayland. Log out and log back in via "Ubuntu on Xorg", or set `XDG_SESSION_TYPE=x11 SDL_VIDEODRIVER=x11` before running.
- **GPU OOM or stutter on the `rt-fast` shader (8 GB VRAM machines):** drop to `--shader="default"` for both demos. Edit `scripts/verify_sim.sh`, or run the standalone commands in [`running.md`](running.md) with `--shader="default"`.
