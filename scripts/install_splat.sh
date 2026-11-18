#!/usr/bin/env bash
# Create (or verify) the splat trainer's env. Idempotent. ~6 GB on disk (torch +
# CUDA runtime wheels). With conda: the `splat` conda env from
# environment-splat.yml. Without: a uv venv at ~/.local/share/sortbots/splat-venv
# (scripts/_splat_env.sh). Same packages either way: requirements-splat.txt.
#
#   scripts/install_splat.sh            # create if missing, then self-check
#   scripts/install_splat.sh --check    # self-check only
#   scripts/install_splat.sh --uv       # use uv even where conda exists
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/_splat_env.sh
source "$REPO_ROOT/scripts/_splat_env.sh"

CHECK_ONLY=false USE_UV=false
for a in "$@"; do
  case "$a" in
    --check) CHECK_ONLY=true;;
    --uv)    USE_UV=true;;
    *) echo "usage: $0 [--check] [--uv]" >&2; exit 2;;
  esac
done

# Same guard as install.sh: a sourced Jazzy puts python3.12 site-packages on
# PYTHONPATH and they shadow the env's numpy.
if [[ -n "${AMENT_PREFIX_PATH:-}" || -n "${ROS_DISTRO:-}" ]]; then
    echo "ERROR: ROS 2 is sourced in this shell; open a clean one." >&2
    exit 1
fi

if ! $CHECK_ONLY; then
  if ! $USE_UV && CONDA_BASE="$(splat_conda_base)"; then
    # shellcheck disable=SC1091
    source "$CONDA_BASE/etc/profile.d/conda.sh"
    cd "$REPO_ROOT"   # environment-splat.yml's `-r requirements-splat.txt` is relative
    if conda env list | awk '{print $1}' | grep -qx splat; then
      echo "==> conda env 'splat' exists; updating from environment-splat.yml"
      conda env update -n splat -f environment-splat.yml
    else
      echo "==> creating conda env 'splat'"
      conda env create -f environment-splat.yml
    fi
    PY="$CONDA_BASE/envs/splat/bin/python"
  elif command -v uv >/dev/null 2>&1; then
    if [[ ! -x "$SPLAT_UV_VENV/bin/python" ]]; then
      echo "==> creating uv venv $SPLAT_UV_VENV (python 3.10)"
      mkdir -p "$(dirname "$SPLAT_UV_VENV")"
      uv venv --python 3.10 "$SPLAT_UV_VENV"
    fi
    echo "==> installing requirements-splat.txt into $SPLAT_UV_VENV"
    # unsafe-best-match: torch's +cu124 build is only on the pytorch index, but
    # uv's default stops at the first index that has ANY version of a package.
    uv pip install --python "$SPLAT_UV_VENV/bin/python" --index-strategy unsafe-best-match \
      -r "$REPO_ROOT/requirements-splat.txt"
    PY="$SPLAT_UV_VENV/bin/python"
  else
    echo "ERROR: neither conda nor uv is installed (curl -LsSf https://astral.sh/uv/install.sh | sh)" >&2
    exit 1
  fi
fi

PY="${PY:-$(splat_python)}" || { echo "ERROR: no splat env — run $0" >&2; exit 1; }

# The import that matters is gsplat's CUDA extension: a wheel built for the
# wrong torch silently falls back to a JIT build and dies looking for nvcc.
# The env's python by path, not `conda run`: conda run doesn't forward stdin,
# so a heredoc check silently runs nothing and "passes".
"$PY" - <<'PY'
import sys, torch, gsplat, scipy
from gsplat.cuda._backend import _C
assert torch.cuda.is_available(), "torch sees no GPU"
assert _C is not None, "gsplat CUDA extension did not load (prebuilt wheel mismatch?)"
print(f"ok: {sys.executable}: torch {torch.__version__}, gsplat {gsplat.__version__}, "
      f"{torch.cuda.get_device_name(0)}")
PY
