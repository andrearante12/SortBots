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
# Same deal for deadsnakes: it hasn't published a questing release either
# (diagnosed 2026-08-27 — its 404 on `apt-get update` trips this script's
# `set -e` before step 1 even finishes, well before anything ROS-related
# runs). Ubuntu 25.10 already ships Python 3.13 by default, so this script
# has no need for it regardless.
DEADSNAKES_SRC="/etc/apt/sources.list.d/deadsnakes-ubuntu-ppa-questing.sources"
if [[ -f "$DEADSNAKES_SRC" ]]; then
  sudo mv "$DEADSNAKES_SRC" "${DEADSNAKES_SRC}.disabled"
  echo "    disabled $DEADSNAKES_SRC (no questing packages)"
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

# rosbridge_suite + web_video_server are NOT in ros2.repos (they're separate
# RobotWebTools repos, normally pulled in as apt binaries — ros-jazzy-*-server
# — which don't exist for questing). Their own deps, diagnosed 2026-08-27 when
# the console's rosbridge_websocket failed to launch ("package 'rosbridge_server'
# not found"): rosbridge_library needs bson/cbor2/pil/ujson, rosbridge_server
# needs tornado, web_video_server needs ffmpeg's libav*/libswscale + boost.
sudo apt-get install -y \
  python3-tornado \
  python3-bson \
  python3-cbor2 \
  python3-pil \
  python3-ujson \
  libavcodec-dev \
  libavformat-dev \
  libavutil-dev \
  libswscale-dev \
  libboost-dev \
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

# rosbridge_suite + the web_video_server chain, cloned directly since neither
# is in ros2.repos (see the apt block above for why). `jazzy` is
# rosbridge_suite's real per-distro branch; web_video_server and its
# async_web_server_cpp dependency each carry a single `ros2` branch spanning
# every ROS 2 distro. Plain `git clone` (not vcs import) since these aren't
# listed in any .repos file — skip if already cloned, matching vcs import's
# own idempotency.
clone_if_missing() {
  local dir="$1" url="$2" branch="$3"
  if [[ -d "$dir" ]]; then
    echo "    $dir already present — skipping clone"
  else
    git clone --branch "$branch" --depth 1 "$url" "$dir"
  fi
}
clone_if_missing "$ROS2_HOME/src/rosbridge_suite" \
  https://github.com/RobotWebTools/rosbridge_suite jazzy
# GT-RAIL/async_web_server_cpp (upstream) is the ros2 branch's canonical
# home, but it's stale and doesn't build against Boost 1.88 (questing's
# default): boost::asio::io_service and boost::filesystem::path::leaf() were
# both fully removed, not just deprecated, by that version (diagnosed
# 2026-08-27 — a wall of "does not name a type" errors from http_connection.hpp
# and http_reply.cpp). fkie's fork already modernized both call sites and was
# last updated 2026-05.
clone_if_missing "$ROS2_HOME/src/async_web_server_cpp" \
  https://github.com/fkie/async_web_server_cpp ros2-releases
clone_if_missing "$ROS2_HOME/src/web_video_server" \
  https://github.com/RobotWebTools/web_video_server ros2

# ── Build ─────────────────────────────────────────────────────────────────────
echo ""
echo "==> [5/5] colcon build (this takes 45-90 min)"
cd "$ROS2_HOME"

# Build only the packages SortBots needs: rclpy, rclcpp, DDS/rmw, common
# message types, ros2cli, and demo_nodes for Phase 2/3 testing.
# Vendor packages that wrap libraries already in apt are skipped — colcon
# resolves their dependents against the system libs via cmake find_package.
# Gazebo packages are excluded entirely (SortBots uses Isaac Sim).
# tinyxml2_vendor is NOT skipped (unlike the others here): pluginlib — a
# transitive dep of ros2launch, added 2026-08-27 — looks for its colcon
# env-hook package.sh at build time, not just headers/libs, so skipping the
# vendor package breaks the dependent even though the actual symbols would
# resolve against the system libtinyxml2-dev fine. It's a thin wrapper,
# seconds to build.
#
# ros2bag was tried alongside ros2launch in the same 2026-08-27 pass but
# dropped: it pulls in the full rosbag2_storage_{sqlite3,mcap,default_plugins}
# stack, each hitting the same env-hook problem one level deeper, and none of
# it is needed for the mesh scenario to run (only scripts/record_*_bag.sh use
# ros2bag, for the offline-dashboard-fixture workflow). It was already
# unbuilt before this pass — not a regression — just still a gap.
# `set +e` here, not `... | grep ... || true` below: grep exiting 1 on "no
# match" (expected on a clean build) must not be mistaken for colcon failing,
# but `-e` has to be off for the whole pipeline or it aborts on that before
# PIPESTATUS can even be read. Re-enabled right after, with colcon's real
# code (PIPESTATUS[0]) checked explicitly.
set +e
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
      ros2launch \
      ros2interface \
      ros2service \
      image_transport \
      rosbridge_msgs \
      rosbridge_library \
      rosapi_msgs \
      rosapi \
      rosbridge_server \
      async_web_server_cpp \
      web_video_server \
    --packages-skip \
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
    2>&1 | tee /tmp/sortbots_ros2_build.log | grep -E "^\[|^---| error: | failed"
COLCON_RC="${PIPESTATUS[0]}"
set -e
# A build interrupted mid-run (e.g. Ctrl-C) must fail loudly here, not fall
# through to the stamp: on 2026-08-22 a SIGINT'd build left 54 packages
# (incl. launch, ros2cli, ros2run) without a local_setup.bash, but the old
# blanket `|| true` swallowed that and the run still stamped it as complete.
if [[ "$COLCON_RC" -ne 0 ]]; then
  echo "ERROR: colcon build exited $COLCON_RC — see /tmp/sortbots_ros2_build.log" >&2
  exit 1
fi

echo ""
# Verify key packages built
SETUP="$ROS2_HOME/install/setup.bash"
if [[ ! -f "$SETUP" ]]; then
  echo "ERROR: colcon produced no install/setup.bash — see /tmp/sortbots_ros2_build.log" >&2
  exit 1
fi
echo "    install/setup.bash: found"

# Smoke test — source setup.bash then try ros2 CLI and rclpy import. These
# must hard-fail (not just warn) so a partial build can never reach the
# stamp below and be mistaken for done on the next run.
env -i HOME="$HOME" PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin" \
  bash -c "source '$SETUP' && ros2 --help >/dev/null 2>&1" \
  && echo "    ros2 CLI: OK" \
  || { echo "ERROR: ros2 CLI not on PATH after build — see /tmp/sortbots_ros2_build.log" >&2; exit 1; }

# `ros2 --help` succeeds even with zero verb extensions registered — it only
# parses the base CLI — so it didn't catch ros2launch missing from the
# --packages-up-to list above (diagnosed 2026-08-27: the mesh bringup's
# `ros2 launch ...` calls failed with "invalid choice: 'launch'" despite this
# smoke test passing). Check the actual verb SortBots depends on.
env -i HOME="$HOME" PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin" \
  bash -c "source '$SETUP' && ros2 launch --help >/dev/null 2>&1" \
  && echo "    ros2 launch: OK" \
  || { echo "ERROR: 'ros2 launch' verb missing after build — see /tmp/sortbots_ros2_build.log" >&2; exit 1; }

env -i HOME="$HOME" PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin" \
  bash -c "source '$SETUP' && python3 -c 'import rclpy; print(\"    rclpy: OK\")'" \
  || { echo "ERROR: rclpy import failed after build" >&2; exit 1; }

# rosbridge_server/web_video_server going missing is exactly what silently
# breaks the dashboard (2026-08-27: console came up, webui/serve.py answered
# on 8081, but rosbridge never listened on 9090 and the page just spun on
# "disconnected — retrying" with nothing in run_console.sh's own output to
# say why — the real error was buried in run_demo.sh's/run_console.sh's own
# log files). Catch it here instead, at build time.
env -i HOME="$HOME" PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin" \
  bash -c "source '$SETUP' && ros2 pkg prefix rosbridge_server >/dev/null 2>&1 && ros2 pkg prefix web_video_server >/dev/null 2>&1" \
  && echo "    rosbridge_server / web_video_server: OK" \
  || { echo "ERROR: rosbridge_server or web_video_server missing after build — see /tmp/sortbots_ros2_build.log" >&2; exit 1; }

# ── Stamp ────────────────────────────────────────────────────────────────────
touch "$STAMP"
echo ""
echo "==> Install complete"
echo "    Activate with:  source $SETUP"
echo "    Or add to ~/.bashrc:"
echo "      source $SETUP"
