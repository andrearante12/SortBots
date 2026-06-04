#!/usr/bin/env bash
# SortBots warehouse SLAM demo launcher — brings up the whole pipeline with
# one command:
#   1. Isaac Sim  — NVIDIA warehouse + XLeRobot + ROS 2 publishers
#   2. RTAB-Map + rviz — live SLAM map
#   3. a WASD teleop terminal window
#
# Each piece needs a different environment (Isaac venv vs system ROS 2), so
# this script sources the right one for each component itself. Run it from a
# CLEAN terminal — do NOT `source /opt/ros/...` or activate Isaac first.
#
# Usage:
#   scripts/run_demo.sh [--robot-id robot_0] [--robots 1] [--scene nvidia]
#                       [--headless] [--no-teleop]
#   scripts/run_demo.sh stop      # tear the whole pipeline down
#
# Re-running it auto-stops any existing pipeline first, so it doubles as a
# "reset the environment" button.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
SIM_LOG="/tmp/sortbots_demo_sim.log"
SIM_RESULT="/tmp/isaac_spawn_warehouse_result.txt"
RTAB_LOG="/tmp/sortbots_demo_rtabmap.log"

stop_pipeline() {
  echo "[run_demo] stopping any running pipeline..."
  pkill -INT -f "sortbots_rtabmap_robot.launch.py" 2>/dev/null || true
  sleep 2
  for p in rtabmap_slam rtabmap_viz rtabmap_util point_cloud_xyzrgb rviz2 \
           wasd_teleop "spawn_warehouse.py"; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
  sleep 1
}

if [[ "${1:-}" == "stop" ]]; then
  stop_pipeline
  echo "[run_demo] stopped."
  exit 0
fi

# ---- args ----
ROBOT_ID=robot_0; ROBOTS=1; SCENE=nvidia; HEADLESS="--no-headless"; TELEOP=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-id) ROBOT_ID="$2"; shift 2;;
    --robots)   ROBOTS="$2";   shift 2;;
    --scene)    SCENE="$2";    shift 2;;
    --headless) HEADLESS="--headless"; shift;;
    --no-teleop) TELEOP=0; shift;;
    -h|--help) sed -n '2,20p' "$0"; exit 0;;
    *) echo "[run_demo] unknown arg: $1"; exit 2;;
  esac
done

# Must be a clean shell — Isaac's activate refuses if ROS 2 is sourced.
if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "ERROR: this shell has ROS 2 sourced (AMENT_PREFIX_PATH set)."
  echo "       Open a fresh terminal and run again WITHOUT sourcing ROS first."
  exit 1
fi

# Always reset first so re-running is a clean restart.
stop_pipeline

# ---- 1. Isaac Sim ----
echo "[run_demo] launching Isaac Sim ($SCENE, $ROBOTS robot(s), $HEADLESS)..."
rm -f "$SIM_RESULT"
setsid bash -c "source '$REPO_ROOT/scripts/activate_isaac.sh' >/dev/null 2>&1; \
  exec python '$REPO_ROOT/scripts/spawn_warehouse.py' $HEADLESS --forever \
       --robots $ROBOTS --scene $SCENE --drive cmd_vel" \
  >"$SIM_LOG" 2>&1 &

echo "[run_demo] waiting for the warehouse to load"
echo "           (first run streams assets from NVIDIA — can take a few minutes)..."
READY=0
for _ in $(seq 1 72); do
  if grep -q "timeline.play()" "$SIM_RESULT" 2>/dev/null; then READY=1; break; fi
  if grep -qiE "ERROR:|Traceback|RuntimeError" "$SIM_LOG" 2>/dev/null; then
    echo "[run_demo] Isaac Sim failed to start — see $SIM_LOG"; exit 1
  fi
  sleep 5
done
if [[ $READY -ne 1 ]]; then
  echo "[run_demo] WARNING: sim not confirmed ready after timeout; continuing anyway (check $SIM_LOG)."
else
  echo "[run_demo] Isaac Sim is up and publishing."
fi

# ---- 2. RTAB-Map + rviz ----
echo "[run_demo] launching RTAB-Map + rviz..."
setsid bash -c "source '$ROS_SETUP'; \
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0; \
  exec ros2 launch '$REPO_ROOT/launch/sortbots_rtabmap_robot.launch.py' \
       robot_id:=$ROBOT_ID use_sim_time:=true" \
  >"$RTAB_LOG" 2>&1 &
sleep 12
echo "[run_demo] RTAB-Map + rviz up."

# ---- 3. WASD teleop (its own terminal window) ----
TELEOP_CMD="source '$ROS_SETUP'; \
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0; \
  python3 '$REPO_ROOT/scripts/wasd_teleop.py' --robot-id $ROBOT_ID"
if [[ $TELEOP -eq 1 ]]; then
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="SortBots WASD teleop" -- \
      bash -c "$TELEOP_CMD; echo; echo 'teleop exited — press Enter to close'; read" \
      >/dev/null 2>&1 &
    echo "[run_demo] opened a WASD teleop window."
  else
    echo "[run_demo] no gnome-terminal; drive from a new terminal with:"
    echo "    source $ROS_SETUP && python3 $REPO_ROOT/scripts/wasd_teleop.py --robot-id $ROBOT_ID"
  fi
fi

cat <<EOF

============================================================
  SortBots warehouse SLAM demo is UP
    * Isaac Sim window  : NVIDIA warehouse + $ROBOT_ID
    * rviz / rtabmap_viz: live SLAM map (drive around to build it)
    * WASD teleop       : w/s fwd-back  a/d turn  q/e strafe
                          space stop  z/x slower/faster  k quit
  Logs : $SIM_LOG
         $RTAB_LOG
  Stop : scripts/run_demo.sh stop
============================================================
EOF
