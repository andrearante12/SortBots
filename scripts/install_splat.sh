#!/usr/bin/env bash
# Create (or verify) the `splat` conda env from environment-splat.yml.
# Idempotent. ~6 GB on disk (torch + CUDA runtime wheels).
#
#   scripts/install_splat.sh            # create if missing, then self-check
#   scripts/install_splat.sh --check    # self-check only
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_NAME=splat

# Same guard as install.sh: a sourced Jazzy puts python3.12 site-packages on
# PYTHONPATH and they shadow the env's numpy.
if [[ -n "${AMENT_PREFIX_PATH:-}" || -n "${ROS_DISTRO:-}" ]]; then
    echo "ERROR: ROS 2 is sourced in this shell; open a clean one." >&2
    exit 1
fi

CONDA_BASE="$(conda info --base 2>/dev/null || echo "$HOME/miniconda3")"
# shellcheck disable=SC1091
source "$CONDA_BASE/etc/profile.d/conda.sh"

if [[ "${1:-}" != "--check" ]]; then
    if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
        echo "==> conda env '$ENV_NAME' exists; updating from environment-splat.yml"
        conda env update -n "$ENV_NAME" -f "$REPO_ROOT/environment-splat.yml"
    else
        echo "==> creating conda env '$ENV_NAME'"
        conda env create -f "$REPO_ROOT/environment-splat.yml"
    fi
fi

# The import that matters is gsplat's CUDA extension: a wheel built for the
# wrong torch silently falls back to a JIT build and dies looking for nvcc.
# The env's python by path, not `conda run`: conda run doesn't forward stdin,
# so a heredoc check silently runs nothing and "passes".
"$CONDA_BASE/envs/$ENV_NAME/bin/python" - <<'PY'
import torch, gsplat
from gsplat.cuda._backend import _C
assert torch.cuda.is_available(), "torch sees no GPU"
assert _C is not None, "gsplat CUDA extension did not load (prebuilt wheel mismatch?)"
print(f"ok: torch {torch.__version__}, gsplat {gsplat.__version__}, "
      f"{torch.cuda.get_device_name(0)}")
PY
