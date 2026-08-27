#!/usr/bin/env bash
# Idempotent bootstrap for the SortBots simulation environment.
# Assumes apt prereqs + Miniconda are already installed; see README for the one-time steps.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# --- Guard: ROS 2 must not be sourced in this shell --------------------------
# Jazzy sources Python 3.12 paths that shadow the lerobot env's numpy.
if [[ -n "${AMENT_PREFIX_PATH:-}" || -n "${ROS_DISTRO:-}" ]]; then
    echo "ERROR: ROS 2 appears to be sourced in this shell (AMENT_PREFIX_PATH or ROS_DISTRO set)." >&2
    echo "       Open a fresh terminal that does NOT source /opt/ros/*/setup.bash and re-run." >&2
    exit 1
fi
if [[ -n "${PYTHONPATH:-}" ]]; then
    echo "WARN: PYTHONPATH is set ('$PYTHONPATH'). This may shadow conda env packages." >&2
    echo "      Continuing, but if you see numpy import errors, unset PYTHONPATH and retry." >&2
fi

# --- Conda ------------------------------------------------------------------
if ! command -v conda >/dev/null 2>&1; then
    if [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
        # shellcheck disable=SC1091
        source "$HOME/miniconda3/etc/profile.d/conda.sh"
    else
        echo "ERROR: conda not found. Install Miniconda (see README) and re-run." >&2
        exit 1
    fi
else
    # shellcheck disable=SC1091
    source "$(conda info --base)/etc/profile.d/conda.sh"
fi

ENV_NAME="lerobot"

# --- Submodules -------------------------------------------------------------
echo "==> Updating submodules"
git submodule update --init --recursive

# --- Conda env --------------------------------------------------------------
if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    echo "==> Conda env '$ENV_NAME' already exists, skipping creation"
else
    echo "==> Creating conda env '$ENV_NAME' from environment.yml"
    conda env create -f environment.yml
fi

conda activate "$ENV_NAME"
echo "==> Active python: $(python --version) at $(which python)"

# --- Pip deps (idempotent fallback; environment.yml already declares them) --
echo "==> Ensuring pip deps are up to date"
pip install --upgrade "mani-skill>=3.0.0b22" torch pygame rerun-sdk

# --- ReplicaCAD asset (~5 GB) -----------------------------------------------
REPLICA_DIR="$HOME/.maniskill/data/scene_datasets/replica_cad_dataset"
if [[ -d "$REPLICA_DIR" ]]; then
    echo "==> ReplicaCAD already at $REPLICA_DIR, skipping download"
else
    echo "==> Downloading ReplicaCAD (~5 GB) into ~/.maniskill/data/"
    python -m mani_skill.utils.download_asset -y "ReplicaCAD"
fi

# --- Overlay XLeRobot into ManiSkill ----------------------------------------
echo "==> Running XLeRobot overlay"
python "$REPO_ROOT/scripts/overlay_xlerobot.py"

echo
echo "Bootstrap complete. Next:  conda activate $ENV_NAME && bash scripts/verify_sim.sh"
