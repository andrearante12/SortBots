# Isaac Sim 5.1 setup

End state: `python scripts/verify_isaac_sim.py` runs headlessly, steps an empty
stage 100 times, and exits 0. Isaac Sim lives in `~/isaacsim/` with its own
Python 3.11 venv — entirely separate from the `lerobot` conda env that drives
ManiSkill.

This is Phase 1 of the broader Isaac Sim plan in
[`../tasks/issacsim_setup.md`](../tasks/issacsim_setup.md). Phase 2 (URDF →
USD import pipeline + regression test) is documented in
[`isaac_sim_phase2.md`](isaac_sim_phase2.md). Phase 3 (warehouse + 2-robot
ROS 2 spawn) is documented in [`isaac_sim_phase3.md`](isaac_sim_phase3.md).
Phase 4 (cuVSLAM hook-up) is still deferred.

## Why Isaac Sim alongside ManiSkill

ManiSkill stays put for VLA / arm manipulation work — the conda env, scripts,
and demos under `scripts/install.sh` are untouched. Isaac Sim is added for the
mobile-navigation / multi-robot / SLAM / ROS-2 half of the SortBots project,
where its native ROS 2 bridge and shared toolchain with Isaac ROS (cuVSLAM,
Nav2) make sim-to-real for perception nearly free.

## Requirements

- Ubuntu 22.04 or 24.04. (Isaac Sim 4.5 is 22.04-only; 6.0 demands 16 GB VRAM.
  5.1 is the version that fits this machine.)
- NVIDIA GPU with RT cores. A100 / H100 are explicitly unsupported. 8 GB VRAM
  is the documented minimum — at that floor, headless verification works but
  RTX-interactive GUI on large scenes will OOM.
- NVIDIA driver R535 or newer.
- Vulkan ICD pointing at the NVIDIA GPU
  (`vulkaninfo --summary | grep -i nvidia` must list it).
- ~30 GB free on `$HOME` (5-8 GB wheels + Kit cache headroom).
- Python 3.11. The install script will pick it up from one of:
  - `python3.11` on PATH (e.g. installed via deadsnakes PPA), or
  - `~/miniconda3/envs/lerobot/bin/python3.11` (which already exists if you
    set up ManiSkill first).

## 1. Apt prerequisites

If you haven't run the ManiSkill setup, install the shared graphics deps:

```bash
sudo apt-get update
sudo apt-get install -y \
    libvulkan1 vulkan-tools libglvnd-dev mesa-utils \
    libegl1 libgl1 libxext6 libx11-6 \
    libxcb-cursor0 libxkbcommon-x11-0 libglu1-mesa
```

The trailing three (`libxcb-cursor0`, `libxkbcommon-x11-0`, `libglu1-mesa`)
are extra Kit dependencies that ManiSkill does not need.

## 2. Install

From the repo root, with no conda env active and no ROS 2 sourced:

```bash
bash scripts/install_isaac_sim.sh                   # prints the install plan, exits 0
bash scripts/install_isaac_sim.sh --accept-download # actually installs
```

The script:

1. Refuses to run if ROS 2 is sourced or the `lerobot` conda env is active.
2. Preflights GPU, RT cores, VRAM, driver, Vulkan ICD, Ubuntu version, disk.
3. Locates Python 3.11.
4. Prints the plan and exits, unless `--accept-download` was passed.
5. Creates `~/isaacsim/venv` and installs `isaacsim[all]==5.1.0` from
   `https://pypi.nvidia.com`.
6. Smoke-tests `import isaacsim`.

Re-running with `--accept-download` is idempotent — finished stages print
"skipping" lines.

## 3. Activate

```bash
source scripts/activate_isaac.sh
```

The wrapper sets `__NV_PRIME_RENDER_OFFLOAD=1` /
`__GLX_VENDOR_LIBRARY_NAME=nvidia` (so hybrid-graphics laptops route Kit at
the dGPU, not the iGPU or llvmpipe), exports `OMNI_KIT_ACCEPT_EULA=YES` /
`PRIVACY_CONSENT=Y` (Kit otherwise prompts for both on first import and hangs
under non-interactive shells), then activates the venv. It refuses to
activate inside the `lerobot` conda env or inside a ROS 2-sourced shell.

## 4. Verify

```bash
python scripts/verify_isaac_sim.py
```

Expected output:

```
isaacsim 5.1.0 | GPU: NVIDIA GeForce RTX 4070 Laptop GPU
Stepped 100/100 frames in 1.2s | sim_time=1.667s
verify_isaac_sim: OK
```

Exit code 0. The 100-step loop is wrapped in a 120 s timeout — if Kit silently
falls back to llvmpipe, the script aborts with a clear message instead of
hanging.

## ROS 2 isolation

Same rule as the ManiSkill side: do **not** source `/opt/ros/jazzy/setup.bash`
in an Isaac Sim shell. Kit ships its own Python 3.11 and the ROS 2 environment
rewrites `PYTHONPATH` / `LD_LIBRARY_PATH` in ways that conflict.
`install_isaac_sim.sh` and `activate_isaac.sh` both refuse to run with
`AMENT_PREFIX_PATH` or `ROS_DISTRO` set.

The Isaac Sim ROS 2 bridge (used in Phase 3) runs inside the Isaac process and
does not require sourcing ROS 2 in the launching shell.

## Coexistence with ManiSkill

One shell, one sim. Never activate both in the same shell:

| Stack     | Activation                              |
|-----------|------------------------------------------|
| ManiSkill | `conda activate lerobot`                |
| Isaac Sim | `source scripts/activate_isaac.sh`      |

Open separate terminals if you need both side by side. The two installs share
no files — the conda env lives in `~/miniconda3/envs/lerobot/`, Isaac Sim in
`~/isaacsim/`.

## Troubleshooting

- **`vulkaninfo` lists only `llvmpipe`:** the NVIDIA Vulkan ICD is not
  registered. Reinstall `nvidia-driver-*` (purge then reinstall). On hybrid
  graphics, the install script's preflight may pass while interactive runs
  miss the dGPU — `activate_isaac.sh` sets the PRIME-offload env vars to fix
  this, but a missing ICD will still fail.
- **Kit aborts on startup with a driver mismatch:** the installed
  `isaacsim` wheels require a specific driver range. R535-R580 is the tested
  range for 5.1; newer drivers usually work but are not guaranteed.
- **`verify_isaac_sim.py` hangs and times out at 120 s:** Kit is using
  llvmpipe instead of the dGPU. Confirm with
  `__NV_PRIME_RENDER_OFFLOAD=1 vulkaninfo --summary` that the NVIDIA ICD is
  picked, and re-source `scripts/activate_isaac.sh` from a clean shell.
- **OOM on the GUI (`./isaac-sim.sh` or any interactive workflow):** 8 GB
  VRAM is the documented floor; Kit allocates ~6 GB before any scene loads.
  Stay headless for Phase 1; for Phase 2+ close other GPU consumers (browsers
  especially) before launching.
- **Disk fills up over time:** Kit caches shaders and assets in
  `~/.cache/ov` and `~/.nvidia-omniverse/`. Safe to delete; will be
  regenerated.

## Updating

```bash
source scripts/activate_isaac.sh
pip install --upgrade "isaacsim[all]==<new-version>" --extra-index-url https://pypi.nvidia.com
```

To wipe and reinstall from scratch:

```bash
rm -rf ~/isaacsim
bash scripts/install_isaac_sim.sh --accept-download
```
