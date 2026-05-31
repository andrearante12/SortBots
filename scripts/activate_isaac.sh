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

# Refuse to activate inside a ROS 2-sourced shell.
if [[ -n "${AMENT_PREFIX_PATH:-}" || -n "${ROS_DISTRO:-}" ]]; then
    echo "ERROR: ROS 2 is sourced in this shell; open a fresh terminal." >&2
    return 1
fi

export ISAACSIM_HOME="${ISAACSIM_HOME:-$HOME/isaacsim}"

if [[ ! -d "$ISAACSIM_HOME/venv" ]]; then
    echo "ERROR: $ISAACSIM_HOME/venv does not exist. Run scripts/install_isaac_sim.sh first." >&2
    return 1
fi

# Hints for hybrid-graphics laptops so Kit picks the NVIDIA dGPU, not iGPU/llvmpipe.
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export KIT_RENDERER=rtx

# Pre-accept NVIDIA EULA + privacy prompt so Kit doesn't block on stdin under
# non-interactive shells (CI, headless verify, etc).
export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y

# shellcheck disable=SC1091
source "$ISAACSIM_HOME/venv/bin/activate"

echo "Isaac Sim venv active: $(python --version) at $(which python)"
