#!/usr/bin/env bash
# Idempotent install of Isaac Sim 5.1 into ~/isaacsim/, parallel to the ManiSkill conda env.
# First run prints the install plan and exits 0; pass --accept-download to actually install.
# See docs/quickstart.md for the bigger picture.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

ISAACSIM_HOME="${ISAACSIM_HOME:-$HOME/isaacsim}"
VENV_DIR="$ISAACSIM_HOME/venv"
STAMP_DIR="$ISAACSIM_HOME/.stamps"
ISAAC_PIP_PACKAGE="${ISAAC_PIP_PACKAGE:-isaacsim[all]==5.1.0}"
ISAAC_PIP_INDEX="${ISAAC_PIP_INDEX:-https://pypi.nvidia.com}"
MIN_VRAM_MIB=7800           # 8 GB cards report ~8188 MiB; below this is a hard fail
MIN_DRIVER_MAJOR=535
MIN_FREE_GB=30

ACCEPT_DOWNLOAD=0
for arg in "$@"; do
    case "$arg" in
        --accept-download) ACCEPT_DOWNLOAD=1 ;;
        -h|--help)
            echo "Usage: $0 [--accept-download]"
            echo "  Without --accept-download: prints the install plan and exits."
            echo "  With --accept-download: installs Isaac Sim into \$ISAACSIM_HOME (default ~/isaacsim/)."
            exit 0 ;;
        *) echo "ERROR: unknown arg '$arg'" >&2; exit 1 ;;
    esac
done

# --- Guard: ROS 2 must not be sourced ---------------------------------------
if [[ -n "${AMENT_PREFIX_PATH:-}" || -n "${ROS_DISTRO:-}" ]]; then
    echo "ERROR: ROS 2 appears to be sourced in this shell." >&2
    echo "       Open a fresh terminal that does NOT source /opt/ros/*/setup.bash." >&2
    exit 1
fi

# --- Guard: not on top of conda lerobot -------------------------------------
if [[ "${CONDA_DEFAULT_ENV:-}" == "lerobot" ]]; then
    echo "ERROR: lerobot conda env is active. Run 'conda deactivate' first." >&2
    echo "       Isaac Sim uses its own venv at $VENV_DIR." >&2
    exit 1
fi

# --- Guard: stale PYTHONPATH ------------------------------------------------
if [[ -n "${PYTHONPATH:-}" ]]; then
    echo "WARN: PYTHONPATH is set ('$PYTHONPATH'). Isaac Sim may import the wrong packages." >&2
    echo "      Continuing; unset PYTHONPATH if imports misbehave." >&2
fi

# ============================================================================
# Stage 1/6: GPU + driver preflight
# ============================================================================
echo "==> 1/6 GPU + driver preflight"
if ! command -v nvidia-smi >/dev/null 2>&1; then
    echo "ERROR: nvidia-smi not found. Install the NVIDIA proprietary driver." >&2
    exit 1
fi
GPU_LINE="$(nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader,nounits | head -n1 | sed 's/^ *//; s/, */|/g')"
GPU_NAME="$(echo "$GPU_LINE" | cut -d'|' -f1)"
GPU_VRAM_MIB="$(echo "$GPU_LINE" | cut -d'|' -f2)"
GPU_DRIVER="$(echo "$GPU_LINE" | cut -d'|' -f3)"
GPU_DRIVER_MAJOR="$(echo "$GPU_DRIVER" | cut -d'.' -f1)"
echo "    GPU: $GPU_NAME ($GPU_VRAM_MIB MiB), driver $GPU_DRIVER"

# RT cores: name contains "RTX" (consumer 20xx+, RTX A-series, RTX PRO).
if ! [[ "$GPU_NAME" =~ RTX ]]; then
    echo "ERROR: GPU '$GPU_NAME' does not appear to be an RTX card." >&2
    echo "       Isaac Sim requires RT cores. A100/H100 are explicitly unsupported." >&2
    exit 1
fi

# Driver: hard-fail below MIN_DRIVER_MAJOR, warn above 580 (Isaac Sim 5.1's tested ceiling).
if [[ "$GPU_DRIVER_MAJOR" -lt "$MIN_DRIVER_MAJOR" ]]; then
    echo "ERROR: NVIDIA driver $GPU_DRIVER < R$MIN_DRIVER_MAJOR.x required by Isaac Sim 5.x." >&2
    exit 1
fi
if [[ "$GPU_DRIVER_MAJOR" -gt 580 ]]; then
    echo "WARN: driver R$GPU_DRIVER_MAJOR.x is newer than Isaac Sim 5.1's tested matrix (R535-R580)." >&2
    echo "      Anecdotally fine; if Kit crashes on startup, consider rolling back." >&2
fi

# VRAM: hard-fail below MIN_VRAM_MIB, warn at the 8 GB floor.
if [[ "$GPU_VRAM_MIB" -lt "$MIN_VRAM_MIB" ]]; then
    echo "ERROR: VRAM $GPU_VRAM_MIB MiB is below Isaac Sim 5.x minimum (~8 GB)." >&2
    exit 1
fi
if [[ "$GPU_VRAM_MIB" -lt 12000 ]]; then
    echo "WARN: VRAM $GPU_VRAM_MIB MiB is at the documented Isaac Sim 5.x minimum (8 GB)." >&2
    echo "      Headless verification will work; large scenes / RTX-interactive may OOM." >&2
fi

# ============================================================================
# Stage 2/6: Vulkan ICD + OS sanity
# ============================================================================
echo "==> 2/6 Vulkan ICD + OS sanity"
if ! command -v vulkaninfo >/dev/null 2>&1; then
    echo "ERROR: vulkaninfo not found. Install: sudo apt install vulkan-tools libvulkan1" >&2
    exit 1
fi
if ! vulkaninfo --summary 2>/dev/null | grep -qi nvidia; then
    echo "ERROR: vulkaninfo does not list an NVIDIA ICD." >&2
    echo "       On hybrid-graphics laptops, retry with:" >&2
    echo "       __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia vulkaninfo --summary" >&2
    exit 1
fi
UBUNTU_VER="$(. /etc/os-release && echo "${VERSION_ID:-unknown}")"
if [[ "$UBUNTU_VER" != "22.04" && "$UBUNTU_VER" != "24.04" ]]; then
    echo "WARN: Ubuntu $UBUNTU_VER is outside Isaac Sim 5.x's tested set (22.04, 24.04)." >&2
fi

# ============================================================================
# Stage 3/6: Disk space
# ============================================================================
echo "==> 3/6 Disk space"
FREE_GB="$(df --output=avail -B1G "$HOME" | tail -n1 | tr -d ' ')"
echo "    $FREE_GB GB free on $HOME"
if [[ "$FREE_GB" -lt "$MIN_FREE_GB" ]]; then
    echo "ERROR: need at least ${MIN_FREE_GB} GB free, have ${FREE_GB} GB." >&2
    exit 1
fi

# ============================================================================
# Stage 4/6: Locate Python 3.11
# ============================================================================
echo "==> 4/6 Locate Python 3.11"
PYTHON311=""
if command -v python3.11 >/dev/null 2>&1; then
    PYTHON311="$(command -v python3.11)"
elif [[ -x "$HOME/miniconda3/envs/lerobot/bin/python3.11" ]]; then
    PYTHON311="$HOME/miniconda3/envs/lerobot/bin/python3.11"
fi
if [[ -z "$PYTHON311" ]]; then
    echo "ERROR: no Python 3.11 found." >&2
    echo "       Install via:  sudo add-apt-repository ppa:deadsnakes/ppa && sudo apt install python3.11 python3.11-venv" >&2
    echo "       Or rely on the existing miniconda3 lerobot env (Python 3.11 is bundled there)." >&2
    exit 1
fi
echo "    Using $PYTHON311 ($("$PYTHON311" --version))"

# ============================================================================
# Print plan; bail unless --accept-download was given.
# ============================================================================
cat <<EOF

Install plan
------------
Target dir          : $ISAACSIM_HOME
Venv                : $VENV_DIR
Python              : $PYTHON311
Pip package         : $ISAAC_PIP_PACKAGE
Pip extra index     : $ISAAC_PIP_INDEX
Approx download     : ~5-8 GB wheels (NVIDIA pypi)
Disk needed         : ~$MIN_FREE_GB GB (with cache headroom)
GPU                 : $GPU_NAME, $GPU_VRAM_MIB MiB, driver $GPU_DRIVER

EOF

if [[ "$ACCEPT_DOWNLOAD" -ne 1 ]]; then
    echo "Re-run with --accept-download to install."
    exit 0
fi

# ============================================================================
# Stage 5/6: Create venv + pip install Isaac Sim
# ============================================================================
echo "==> 5/6 Venv + pip install"
mkdir -p "$ISAACSIM_HOME" "$STAMP_DIR"

if [[ -x "$VENV_DIR/bin/python" ]]; then
    echo "    Venv already exists at $VENV_DIR, skipping creation"
else
    "$PYTHON311" -m venv "$VENV_DIR"
fi

# Activate venv in this script only.
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
python -m pip install --upgrade pip

if [[ -f "$STAMP_DIR/pip_installed" ]]; then
    echo "    Isaac Sim wheels already installed (per stamp); use --force to redo (not implemented)"
else
    echo "    Installing $ISAAC_PIP_PACKAGE from $ISAAC_PIP_INDEX (this may take 10-20 min)..."
    pip install --upgrade "$ISAAC_PIP_PACKAGE" --extra-index-url "$ISAAC_PIP_INDEX"
    touch "$STAMP_DIR/pip_installed"
fi

# ============================================================================
# Stage 6/6: Smoke import
# ============================================================================
echo "==> 6/6 Smoke import"
# Kit prompts for EULA + privacy on first import; pre-accept so we don't hang.
export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y
python - <<'PY'
import importlib, sys
try:
    isaacsim = importlib.import_module("isaacsim")
    print(f"    isaacsim package: {getattr(isaacsim, '__version__', 'unknown')}")
except Exception as e:
    print(f"ERROR: 'import isaacsim' failed: {e}", file=sys.stderr)
    sys.exit(1)
PY

cat <<EOF

Install complete. Next:

    source scripts/activate_isaac.sh
    python scripts/verify_isaac_sim.py
EOF
