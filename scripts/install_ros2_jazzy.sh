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

# rtabmap (SLAM) + the Nav2 stack are the other big silent gap (diagnosed
# 2026-08-27: both explore_fresh and explore_fleet_mesh's per-robot bringups
# were crashing at launch-description build time — "package 'rtabmap_launch'
# not found" — before a single node started, so NEITHER SLAM NOR Nav2 was
# ever actually running; run_demo.sh's own "ROS 2 stack up" banner prints
# unconditionally and never caught it). Nav2 itself (navigation2 repo) is
# already in ros2.repos, just never built; rtabmap needs the first four
# system libs, and Nav2's own nav2_map_server (pulled in transitively, not
# explicitly requested below) needs GraphicsMagick++ to read map images.
# libxsimd-dev is nav2_mppi_controller's alone: MPPI vectorises its rollouts
# through xsimd, find_package(xsimd REQUIRED) fails the configure step without
# it, and it is the controller plugin configs/nav2_params.yaml selects — so
# missing it costs the whole controller_server, not just some SIMD speedup
# (2026-08-27).
sudo apt-get install -y \
  libpcl-dev \
  liboctomap-dev \
  libtbb-dev \
  libproj-dev \
  libgraphicsmagick++1-dev \
  libxsimd-dev \
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

# rtabmap_ros's dependency chain: none of these are in ros2.repos either.
# rtabmap/rtabmap_ros use their own per-distro `jazzy-devel` branch;
# perception_pcl (pcl_conversions/pcl_ros — rtabmap_util/_odom's only
# REQUIRED find_package, unlike the ones below) and octomap_msgs (needed for
# the octomap_* topics nodes/rtabmap_cloud_pump.py relays) carry a single
# branch spanning multiple ROS 2 distros, same as rosbridge_suite's siblings
# above. Deliberately NOT cloned: grid_map_ros, apriltag_msgs, aruco_msgs,
# aruco_opencv_msgs — all four are plain (non-REQUIRED) find_package() calls
# in rtabmap_ros's CMakeLists, i.e. auto-detected optional marker/costmap
# features this demo never uses; skipping them avoids grid_map_ros's own
# rosbag2_cpp dependency (rosbag2_storage is deliberately unbuilt below,
# same reasoning as the ros2bag drop).
clone_if_missing "$ROS2_HOME/src/rtabmap" \
  https://github.com/introlab/rtabmap jazzy-devel
clone_if_missing "$ROS2_HOME/src/rtabmap_ros" \
  https://github.com/introlab/rtabmap_ros jazzy-devel
clone_if_missing "$ROS2_HOME/src/perception_pcl" \
  https://github.com/ros-perception/perception_pcl jazzy
clone_if_missing "$ROS2_HOME/src/octomap_msgs" \
  https://github.com/OctoMap/octomap_msgs ros2
# perception_pcl's own pcl_conversions hard-requires this (2026-08-27:
# "Could not find a package configuration file provided by pcl_msgs") — a
# separate tiny message-only repo, not bundled inside perception_pcl itself.
clone_if_missing "$ROS2_HOME/src/pcl_msgs" \
  https://github.com/ros-perception/pcl_msgs ros2

# Drop Nav2's blanket -Werror (2026-08-27). nav2_package.cmake hardcodes
# `-Werror -Wnull-dereference`, and GCC 15 — which questing ships, years newer
# than the GCC 13 Jazzy was cut against — raises a false "potential null
# pointer dereference" inside rosidl's *generated* Time::operator==, reached
# by inlining Path != Path in nav2_behavior_tree. The offending code is
# generated, not Nav2's own, so there is nothing upstream to fix locally and
# no narrower flag wins: add_compile_options lands after CMAKE_CXX_FLAGS on
# the command line, so a -Wno-error=... passed via --cmake-args is re-armed by
# the -Werror that follows it. Warnings still print; they just stop being
# fatal. sed is idempotent — a second run finds no -Werror left to strip.
NAV2_PKG_CMAKE="$ROS2_HOME/src/navigation2/nav2_common/cmake/nav2_package.cmake"
if [ -f "$NAV2_PKG_CMAKE" ]; then
  sed -i 's/ -Werror//' "$NAV2_PKG_CMAKE"
  echo "    patched out Nav2 -Werror (GCC 15 false positive)"
fi

# Drop rtabmap_util's REQUIRED dependency on RTABMap's `gui` component
# (2026-08-27). WITH_QT=OFF below means rtabmap builds no gui library, so this
# line fails the configure step with "Unsupported or not found required
# component: gui" — and rtabmap_util is not optional for us: both rtabmap_slam
# and rtabmap_launch depend on it. The requirement is spurious, not something
# WITH_QT=OFF actually breaks: line 31 is the only mention of `gui` in the
# whole CMakeLists, no target links a gui target, and no source under src/ or
# include/ pulls in an rtabmap/gui or Qt header. Cheaper to delete the line
# than to drag all of Qt5 in to satisfy a component nothing calls.
RTABMAP_UTIL_CMAKE="$ROS2_HOME/src/rtabmap_ros/rtabmap_util/CMakeLists.txt"
if [ -f "$RTABMAP_UTIL_CMAKE" ]; then
  sed -i '/find_package(RTABMap COMPONENTS gui REQUIRED)/d' "$RTABMAP_UTIL_CMAKE"
  echo "    patched out rtabmap_util's unused RTABMap gui component"
fi

# Drop rtabmap_launch's exec_depend on the two GUI packages (2026-08-27).
# rtabmap_viz and rtabmap_rviz_plugins are in --packages-skip below, so colcon
# refuses to build rtabmap_launch at all ("Check that the following packages
# have been built") — it fails in 0.01s, before any compilation. These are
# exec_depends on a runtime path this demo never takes: upstream's
# rtabmap.launch.py gates both behind its `rviz`/`rtabmap_viz` arguments, and
# launch/sortbots_rtabmap_robot.launch.py pins BOTH to false under the unified
# bringup (the web dashboard replaces rviz). Consequence to know about: a
# standalone `rviz:=true` run now fails at node-spawn time instead of at build
# time — already true before this patch, since neither package was ever built.
RTABMAP_LAUNCH_PKG="$ROS2_HOME/src/rtabmap_ros/rtabmap_launch/package.xml"
if [ -f "$RTABMAP_LAUNCH_PKG" ]; then
  sed -i -e '/<exec_depend>rtabmap_viz<\/exec_depend>/d' \
         -e '/<exec_depend>rtabmap_rviz_plugins<\/exec_depend>/d' \
         "$RTABMAP_LAUNCH_PKG"
  echo "    patched out rtabmap_launch's GUI exec_depends"
fi

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
#
# The rtabmap WITH_* flags below are all auto-detect-if-found in rtabmap's
# own CMakeLists (2026-08-27): g2o/GTSAM are large, historically finicky
# graph-optimization backends neither of which is in ros2.repos or apt here,
# and rtabmap falls back to its bundled TORO optimizer with both off —
# acceptable for this demo's warehouse-scale maps. WITH_QT off +
# BUILD_APP/TOOLS/EXAMPLES off drop rtabmap's desktop Qt GUI/CLI tools, which
# this demo never launches (rviz:=false; the web dashboard replaces both).
# These are colcon-global --cmake-args like FORCE_BUILD_VENDOR_PKG above —
# harmless "not used by the project" warnings on every package that isn't
# rtabmap itself.
#
# nav2_mppi_controller / nav2_navfn_planner / depth_image_proc are listed
# explicitly because nothing pulls them in transitively (2026-08-27).
# controller_server and planner_server are only the *servers*: the algorithms
# they run are pluginlib plugins in separate packages, named in
# configs/nav2_params.yaml, and a plugin package is a runtime dependency that
# --packages-up-to on the server cannot see. Built without them, Nav2 comes up
# and then dies in on_configure ("... does not exist. Declared types are ...")
# — so the lifecycle manager never reaches active and no goal is ever
# followed, which looks identical to a planning failure from the dashboard.
# Same story for depth_image_proc, one step earlier in the pipeline:
# sortbots_nav2.launch.py composes its PointCloudXyzNode to turn Isaac's raw
# depth Image into the PointCloud2 obstacle_layer subscribes to, so without it
# the local costmap stays empty and the robot drives straight through racks.
# Keep this list in sync with the `plugin:` keys in configs/nav2_params.yaml.
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
      pcl_msgs \
      pcl_conversions \
      pcl_ros \
      octomap_msgs \
      rtabmap \
      rtabmap_msgs \
      rtabmap_conversions \
      rtabmap_sync \
      rtabmap_util \
      rtabmap_odom \
      rtabmap_slam \
      rtabmap_launch \
      nav2_controller \
      nav2_planner \
      nav2_smoother \
      nav2_behaviors \
      nav2_bt_navigator \
      nav2_waypoint_follower \
      nav2_velocity_smoother \
      nav2_lifecycle_manager \
      nav2_mppi_controller \
      nav2_navfn_planner \
      depth_image_proc \
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
      rtabmap_viz \
      rtabmap_rviz_plugins \
      rtabmap_demos \
      rtabmap_examples \
      rtabmap_python \
      rtabmap_costmap_plugins \
      nav2_rviz_plugins \
      nav2_simple_commander \
      rviz_assimp_vendor \
      rviz_default_plugins \
      rviz_rendering_tests \
      rviz_visual_testing_framework \
      qt_gui \
      qt_gui_cpp \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=OFF \
      -DBUILD_TESTING=OFF \
      -DFORCE_BUILD_VENDOR_PKG=OFF \
      -DWITH_OCTOMAP=ON \
      -DWITH_G2O=OFF \
      -DWITH_GTSAM=OFF \
      -DWITH_POINTMATCHER=OFF \
      -DWITH_QT=OFF \
      -DBUILD_APP=OFF \
      -DBUILD_TOOLS=OFF \
      -DBUILD_EXAMPLES=OFF \
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

# rtabmap_launch/nav2_bt_navigator missing is the same failure mode as
# rosbridge above, one layer deeper: the per-robot bringup's launch
# description fails to even construct ("package 'rtabmap_launch' not found"),
# so NO node in that robot's stack ever starts — not RTAB-Map, not Nav2, not
# the explorer/task_manager — while run_demo.sh's own "ROS 2 stack up" banner
# prints unconditionally regardless (diagnosed 2026-08-27: this had been
# silently true for every prior run, mesh or not).
env -i HOME="$HOME" PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin" \
  bash -c "source '$SETUP' && ros2 pkg prefix rtabmap_launch >/dev/null 2>&1 && ros2 pkg prefix nav2_bt_navigator >/dev/null 2>&1" \
  && echo "    rtabmap_launch / nav2_bt_navigator: OK" \
  || { echo "ERROR: rtabmap_launch or nav2_bt_navigator missing after build — see /tmp/sortbots_ros2_build.log" >&2; exit 1; }

# The Nav2 plugin packages fail later and quieter than a missing launch
# package: the bringup starts fine and only falls over inside on_configure, so
# check them here rather than finding out from a robot that never moves. Each
# name pairs with a `plugin:` entry in configs/nav2_params.yaml; depth_image_proc
# is the composable node that feeds the local costmap (2026-08-27).
# Check with `ros2 pkg prefix`, never `test -d install/<pkg>`: a package whose
# configure step fails still leaves an install/<pkg>/ containing nothing but
# colcon's own package.sh shims, so the directory exists while the package does
# not. `ros2 pkg prefix` reads the ament index marker, which only a completed
# install writes.
for pkg in nav2_mppi_controller nav2_navfn_planner nav2_waypoint_follower depth_image_proc; do
  env -i HOME="$HOME" PATH="$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin" \
    bash -c "source '$SETUP' && ros2 pkg prefix $pkg >/dev/null 2>&1" \
    || { echo "ERROR: $pkg missing after build — see /tmp/sortbots_ros2_build.log" >&2; exit 1; }
done
echo "    nav2 controller/planner/waypoint plugins + depth_image_proc: OK"

# ── Stamp ────────────────────────────────────────────────────────────────────
touch "$STAMP"
echo ""
echo "==> Install complete"
echo "    Activate with:  source $SETUP"
echo "    Or add to ~/.bashrc:"
echo "      source $SETUP"
