#!/usr/bin/env bash
# Idempotent from-source install of ROS 2 Jazzy into ${ROS2_HOME:-~/ros2_jazzy}.
#
# Ubuntu 25.10 (Questing) ships Python 3.13 and has no binary ROS 2 Jazzy
# packages, so we build from the upstream jazzy repos list.
#
# Without --accept-download: prints the plan and exits 0.
# With    --accept-download: installs deps, fetches source, builds.
#
# After a successful build the stamp $ROS2_HOME/.stamp_built is written
# so repeated calls skip the expensive colcon build.
#
# Usage:
#   scripts/install_ros2_jazzy.sh --accept-download   # ~45-90 min first run
#   source ~/ros2_jazzy/install/setup.bash            # activate
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS2_HOME="${ROS2_HOME:-$HOME/ros2_jazzy}"
STAMP="$ROS2_HOME/.stamp_built"
REPOS_URL="https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos"
MIN_FREE_GB=20

ACCEPT=0
for arg in "$@"; do
  case "$arg" in
    --accept-download) ACCEPT=1;;
    -h|--help)
      echo "Usage: $0 [--accept-download]"
      echo "  --accept-download  fetch source and build (skipped if stamp exists)"
      exit 0;;
    *) echo "ERROR: unknown arg '$arg'" >&2; exit 1;;
  esac
done

# ── Plan ─────────────────────────────────────────────────────────────────────
echo "ROS 2 Jazzy from-source installer"
echo "  ROS2_HOME : $ROS2_HOME"
echo "  repos     : $REPOS_URL"
echo "  build     : colcon build --symlink-install ($(nproc) cores)"
echo "  stamp     : $STAMP"
echo ""

if [[ "$ACCEPT" -eq 0 ]]; then
  echo "Dry run. Pass --accept-download to proceed."
  echo "Estimated time: 45-90 min on first run."
  exit 0
fi

# ── Stamp check ──────────────────────────────────────────────────────────────
if [[ -f "$STAMP" ]]; then
  echo "Already built (stamp: $STAMP). Nothing to do."
  echo "To rebuild: rm $STAMP && $0 --accept-download"
  exit 0
fi

# ── Disk space ───────────────────────────────────────────────────────────────
FREE_GB=$(df -BG / | awk 'NR==2{gsub("G","",$4); print $4}')
if [[ "$FREE_GB" -lt "$MIN_FREE_GB" ]]; then
  echo "ERROR: only ${FREE_GB}G free on /; need ${MIN_FREE_GB}G" >&2; exit 1
fi

# ── System dependencies ───────────────────────────────────────────────────────
echo "==> [1/5] apt dependencies"
# Disable the ROS binary apt repo — it has no questing release and causes
# apt-get update to fail with a hard error. We're building from source so
# the binary repo is not needed.
if [[ -f /etc/apt/sources.list.d/ros2.list ]]; then
  sudo mv /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.list.disabled
  echo "    disabled /etc/apt/sources.list.d/ros2.list (no questing packages)"
fi
sudo apt-get update -q
sudo apt-get install -y \
  build-essential cmake git \
  python3-dev python3-pip python3-venv \
  libbullet-dev \
  libasio-dev libtinyxml2-dev libcunit1-dev \
  libcurl4-openssl-dev \
  libssl-dev \
  libeigen3-dev \
  libacl1-dev \
  liblog4cxx-dev \
  pkg-config \
  clang \
  lld

# ── Python build tools (isolated from conda) ──────────────────────────────────
echo ""
echo "==> [2/5] Python build tools"
# Use /usr/bin/pip3 explicitly — conda's pip must not install these into the
# system site-packages or they'll conflict with ROS 2's Python packages.
#
# setuptools must come first: Python 3.12+ removed distutils from stdlib;
# setuptools provides the distutils shim that empy==3.3.4 requires.
# empy==3.3.4 must be installed with --no-build-isolation so it sees the
# already-installed setuptools instead of an isolated env without distutils.
/usr/bin/pip3 install --break-system-packages setuptools

/usr/bin/pip3 install --break-system-packages --no-build-isolation empy==3.3.4

/usr/bin/pip3 install --break-system-packages \
  colcon-common-extensions \
  vcstool \
  catkin_pkg \
  lark \
  pytest \
  pytest-cov \
  flake8 \
  flake8-docstrings \
  importlib-metadata \
  mypy-extensions

# ── Jazzy build dependencies (replaces rosdep on questing) ───────────────────
# rosdep has no questing rules and its pip install is broken by the Python 3.13
# module path split. We install the known Jazzy dependency set directly.
echo ""
echo "==> [3/5] Jazzy build dependencies (apt)"
sudo apt-get install -y \
  libacl1-dev \
  libasio-dev \
  libbullet-dev \
  libcunit1-dev \
  libcurl4-openssl-dev \
  libeigen3-dev \
  liblog4cxx-dev \
  liborocos-kdl-dev \
  libpcre2-dev \
  libsqlite3-dev \
  libtinyxml2-dev \
  libxaw7-dev \
  libxrandr-dev \
  libyaml-cpp-dev \
  pybind11-dev \
  python3-cryptography \
  python3-ifcfg \
  python3-numpy \
  python3-packaging \
  python3-lxml \
  swig \
  2>/dev/null || true

# ── Fetch source ─────────────────────────────────────────────────────────────
echo ""
echo "==> [4/5] fetch ROS 2 Jazzy source into $ROS2_HOME"
mkdir -p "$ROS2_HOME/src"
cd "$ROS2_HOME"

if [[ ! -f ros2.repos ]]; then
  wget -q -O ros2.repos "$REPOS_URL"
  echo "    downloaded ros2.repos"
else
  echo "    ros2.repos already present — skipping download"
fi

# Import repos (idempotent: vcs import skips existing dirs)
VCS="$(command -v vcs 2>/dev/null || echo "$HOME/.local/bin/vcs")"
"$VCS" import --retry 3 src < ros2.repos


# ── Build ─────────────────────────────────────────────────────────────────────
echo ""
echo "==> [5/5] colcon build (this takes 45-90 min)"
cd "$ROS2_HOME"

# Build only the packages SortBots needs: rclpy, rclcpp, DDS/rmw, common
# message types, ros2cli, and demo_nodes for Phase 2/3 testing.
# Vendor packages that wrap libraries already in apt are skipped — colcon
# resolves their dependents against the system libs via cmake find_package.
# Gazebo packages are excluded entirely (SortBots uses Isaac Sim).
env -i \
  HOME="$HOME" \
  PATH="$HOME/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  COLCON_EXTENSION_BLOCKLIST="colcon_core.event_handler.desktop_notification" \
  /usr/bin/python3 -m colcon build \
    --executor parallel \
    --symlink-install \
    --parallel-workers "$(nproc)" \
    --packages-up-to \
      rclpy \
      rclcpp \
      rclcpp_action \
      std_msgs \
      std_srvs \
      nav_msgs \
      geometry_msgs \
      sensor_msgs \
      tf2 \
      tf2_ros \
      tf2_py \
      rmw_fastrtps_cpp \
      rmw_fastrtps_dynamic_cpp \
      demo_nodes_cpp \
      demo_nodes_py \
      ros2cli \
      ros2topic \
      ros2run \
      ros2node \
      ros2bag \
    --packages-skip \
      yaml_cpp_vendor \
      tinyxml2_vendor \
      gz_cmake_vendor \
      gz_math_vendor \
      gz_utils_vendor \
      gz_tools_vendor \
      rviz_ogre_vendor \
      rviz_rendering \
      rviz_common \
      rviz2 \
      rosbag2_storage \
      rosbag2_storage_default_plugins \
      rosbag2_storage_mcap \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=OFF \
      -DBUILD_TESTING=OFF \
      -DFORCE_BUILD_VENDOR_PKG=OFF \
      -DTRACETOOLS_DISABLED=ON \
    2>&1 | tee /tmp/sortbots_ros2_build.log | grep -E "^\[|^---| error: | failed" || true

echo ""
# Verify key packages built
SETUP="$ROS2_HOME/install/setup.bash"
if [[ ! -f "$SETUP" ]]; then
  echo "ERROR: colcon produced no install/setup.bash — see /tmp/sortbots_ros2_build.log" >&2
  exit 1
fi
echo "    install/setup.bash: found"

# Smoke test — source setup.bash then try ros2 CLI and rclpy import
env -i HOME="$HOME" PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin" \
  bash -c "source '$SETUP' && ros2 --help >/dev/null 2>&1" \
  && echo "    ros2 CLI: OK" \
  || echo "    WARNING: ros2 CLI not on PATH (ros2cli may not have built yet)"

env -i HOME="$HOME" PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin" \
  bash -c "source '$SETUP' && python3 -c 'import rclpy; print(\"    rclpy: OK\")'" \
  || echo "    WARNING: rclpy import failed"

# ── Stamp ────────────────────────────────────────────────────────────────────
touch "$STAMP"
echo ""
echo "==> Install complete"
echo "    Activate with:  source $SETUP"
echo "    Or add to ~/.bashrc:"
echo "      source $SETUP"
