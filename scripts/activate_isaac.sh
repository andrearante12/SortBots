# Sourced wrapper that activates the Isaac Sim 5.1 venv.
# Counterpart to `conda activate lerobot` for the ManiSkill side.
# Usage:  source scripts/activate_isaac.sh
# shellcheck shell=bash

# Refuse to run as a child process — env vars wouldn't propagate.
if [[ "${BASH_SOURCE[0]:-}" == "${0}" ]]; then
    echo "ERROR: activate_isaac.sh must be sourced, not executed." >&2
    echo "       Run:  source scripts/activate_isaac.sh" >&2
    exit 1
fi

# Refuse to activate on top of conda lerobot to prevent PYTHONPATH / numpy shadow.
if [[ "${CONDA_DEFAULT_ENV:-}" == "lerobot" ]]; then
    echo "ERROR: lerobot conda env is active; deactivate before sourcing Isaac." >&2
    echo "       Run:  conda deactivate" >&2
    return 1
fi

# Refuse to activate inside a ROS 2-sourced shell. We check AMENT_PREFIX_PATH
# (the canonical "ROS 2 is sourced" signal) but NOT ROS_DISTRO — Isaac Sim's
# bundled ROS 2 bridge requires ROS_DISTRO to be set to the bundled distro
# (we set it below to "jazzy"), so its presence isn't a conflict by itself.
if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
    echo "ERROR: ROS 2 is sourced in this shell (AMENT_PREFIX_PATH set);" >&2
    echo "       open a fresh terminal and do not source /opt/ros/*/setup.bash." >&2
    return 1
fi

export ISAACSIM_HOME="${ISAACSIM_HOME:-$HOME/isaacsim}"

if [[ ! -d "$ISAACSIM_HOME/venv" ]]; then
    echo "ERROR: $ISAACSIM_HOME/venv does not exist. Run scripts/install_isaac_sim.sh first." >&2
    return 1
fi

# Ubuntu 25.10 ("questing") ships only libxml2-16 (soname libxml2.so.16);
# the pip-installed Isaac Sim's URDF importer plugin is linked against the
# older libxml2.so.2 and there is no apt package that provides it here
# (diagnosed 2026-08-27 — spawn_warehouse's XLeRobot import failed with
# "Failed to acquire interface: ...urdf::Urdf" because the plugin's native
# lib silently no-op'd on the missing dependency). compat-libs/ holds a real
# libxml2.so.2 (+ its libicuuc.so.70/libicudata.so.70 deps) lifted from the
# gnome-42-2204 snap's jammy base — an actual matching build, not a symlink
# to the newer major version. Scoped to this venv's launches only.
_ISAAC_COMPAT_LIBS="$ISAACSIM_HOME/compat-libs"
if [[ -d "$_ISAAC_COMPAT_LIBS" ]]; then
    export LD_LIBRARY_PATH="$_ISAAC_COMPAT_LIBS${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
unset _ISAAC_COMPAT_LIBS

# Hints for hybrid-graphics laptops so Kit picks the NVIDIA dGPU, not iGPU/llvmpipe.
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export KIT_RENDERER=rtx

# Pre-accept NVIDIA EULA + privacy prompt so Kit doesn't block on stdin under
# non-interactive shells (CI, headless verify, etc).
export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y

# Phase 3+: Isaac Sim's bundled ROS 2 bridge (isaacsim.ros2.bridge v4.12.4)
# fails to load with "ROS2 Bridge startup failed" unless ROS_DISTRO,
# RMW_IMPLEMENTATION, and LD_LIBRARY_PATH point at the bundled distro libs.
# These libs live under .../isaacsim.ros2.bridge/jazzy/lib on Ubuntu 24.04.
# We default to FastDDS (the bridge's recommended RMW for the bundled stack).
_ISAAC_ROS2_LIB="$ISAACSIM_HOME/venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"
if [[ -d "$_ISAAC_ROS2_LIB" ]]; then
    export ROS_DISTRO=jazzy
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$_ISAAC_ROS2_LIB"
fi
unset _ISAAC_ROS2_LIB

# shellcheck disable=SC1091
source "$ISAACSIM_HOME/venv/bin/activate"

echo "Isaac Sim venv active: $(python --version) at $(which python)"
