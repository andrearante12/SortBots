#!/usr/bin/env bash
# SortBots warehouse SLAM demo launcher — brings up the whole pipeline with
# one command:
#   1. Isaac Sim — NVIDIA warehouse + XLeRobot + ROS 2 publishers (own venv)
#   2. ROS 2 side, one launch (launch/sortbots_bringup.launch.py):
#        RTAB-Map SLAM + Nav2 + the web dashboard (rosbridge + web_video_server
#        + serve.py) + nodes/task_manager.py
#   3. (optional) a WASD teleop terminal window — off by default now that the
#        dashboard has its own drive pad; enable with --teleop.
#
# Drive and dispatch from the dashboard (URL printed on startup); rviz is off
# by default since the dashboard replaces it.
#
# Each piece needs a different environment (Isaac venv vs system ROS 2), so
# this script sources the right one for each component itself. Run it from a
# CLEAN terminal — do NOT `source /opt/ros/...` or activate Isaac first.
#
# Usage:
#   scripts/run_demo.sh [--robot-id robot_0] [--robots 1] [--scene nvidia]
#                       [--headless] [--teleop] [--localize] [--map PATH]
#   scripts/run_demo.sh stop      # tear the whole pipeline down
#
# Re-running it auto-stops any existing pipeline first, so it doubles as a
# "reset the environment" button.
#
# Map lifecycle — build a map once, then navigate on it afterwards:
#
#   scripts/run_demo.sh --teleop     # drive around; map builds from scratch
#   scripts/run_demo.sh stop         # map persists in ~/.ros/sortbots_<id>.db
#   scripts/run_demo.sh --localize   # reopen that map, localize, send nav goals
#
# Without --localize every run starts from an empty database, so a mapping run
# always means what it says. --localize never deletes.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
SIM_LOG="/tmp/sortbots_demo_sim.log"
SIM_RESULT="/tmp/isaac_spawn_warehouse_result.txt"
BRINGUP_LOG="/tmp/sortbots_demo_bringup.log"
DASHBOARD_PORT=8081

stop_pipeline() {
  echo "[run_demo] stopping any running pipeline..."
  # SIGINT the top-level launch first so it can shut its children (Nav2
  # lifecycle nodes, rtabmap, rosbridge) down cleanly, then hard-kill leftovers.
  pkill -INT -f "sortbots_bringup.launch.py" 2>/dev/null || true
  pkill -INT -f "sortbots_rtabmap_robot.launch.py" 2>/dev/null || true
  sleep 2
  for p in rtabmap_slam rtabmap_viz rtabmap_util point_cloud_xyzrgb rviz2 \
           controller_server planner_server smoother_server behavior_server \
           bt_navigator waypoint_follower velocity_smoother lifecycle_manager \
           component_container rosbridge_websocket web_video_server \
           "webui/serve.py" webui_url.py task_manager.py rtabmap_cloud_pump.py \
           web_video_watchdog.sh wasd_teleop "spawn_warehouse.py"; do
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
# Teleop defaults OFF — the dashboard drive pad replaces the WASD window.
ROBOT_ID=robot_0; ROBOTS=1; SCENE=nvidia; HEADLESS="--no-headless"; TELEOP=0
LOCALIZE=false; MAP_DB=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-id) ROBOT_ID="$2"; shift 2;;
    --robots)   ROBOTS="$2";   shift 2;;
    --scene)    SCENE="$2";    shift 2;;
    --headless) HEADLESS="--headless"; shift;;
    --teleop)    TELEOP=1; shift;;
    --no-teleop) TELEOP=0; shift;;  # accepted for back-compat (already the default)
    --localize) LOCALIZE=true; shift;;
    --map)      MAP_DB="$2";  shift 2;;
    -h|--help) sed -n '2,33p' "$0"; exit 0;;
    *) echo "[run_demo] unknown arg: $1"; exit 2;;
  esac
done

# Default matches launch/sortbots_rtabmap_robot.launch.py's database_path.
[[ -z "$MAP_DB" ]] && MAP_DB="$HOME/.ros/sortbots_${ROBOT_ID}.db"

# Localizing against a map that was never built is a confusing failure mode —
# RTAB-Map comes up, publishes nothing, and every nav goal is rejected for want
# of a map. Catch it here instead.
if [[ "$LOCALIZE" == "true" && ! -f "$MAP_DB" ]]; then
  echo "ERROR: --localize needs an existing map, but $MAP_DB does not exist."
  echo "       Build one first:  scripts/run_demo.sh --teleop"
  exit 1
fi

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

# ---- 2. ROS 2 side: SLAM + Nav2 + dashboard + task manager (one launch) ----
if [[ "$LOCALIZE" == "true" ]]; then
  echo "[run_demo] LOCALIZATION mode — reusing the map at $MAP_DB (not modified)."
else
  echo "[run_demo] MAPPING mode — building a fresh map into $MAP_DB."
fi
echo "[run_demo] launching RTAB-Map + Nav2 + web dashboard + task manager..."
# Prepend the system path so rosbridge_websocket's `#!/usr/bin/env python3`
# shebang resolves to /usr/bin/python3 even if this script was started from a
# conda-active shell (conda's python breaks rclpy's compiled extension — see
# launch/sortbots_webui.launch.py's docstring).
setsid bash -c "source '$ROS_SETUP'; \
  export PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin':\"\$PATH\"; \
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0; \
  exec ros2 launch '$REPO_ROOT/launch/sortbots_bringup.launch.py' \
       robot_id:=$ROBOT_ID use_sim_time:=true rviz:=false \
       dashboard_port:=$DASHBOARD_PORT \
       localization:=$LOCALIZE database_path:='$MAP_DB'" \
  >"$BRINGUP_LOG" 2>&1 &
sleep 12
echo "[run_demo] ROS 2 stack up (see $BRINGUP_LOG)."

# Echo the dashboard URL to this console too (the launch prints it into the
# bringup log; webui_url.py needs no ROS, just tailscale).
python3 "$REPO_ROOT/scripts/webui_url.py" --port "$DASHBOARD_PORT" 2>/dev/null || \
  echo "[run_demo] dashboard: http://localhost:$DASHBOARD_PORT/"

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
    * Web dashboard     : http://localhost:$DASHBOARD_PORT/  (or the tailnet URL above)
        - live SLAM map + trail, Nav2 path/costmap overlays, SLAM status
        - drag on the map to send a Nav2 goal; drive pad; pickup->dropoff dispatch
    * Nav2 + RTAB-Map   : running headless (rviz off; the dashboard replaces it)
    * Map               : $MAP_DB $( [[ "$LOCALIZE" == "true" ]] && echo "(localization — read-only)" || echo "(mapping — rebuilt this run)" )
$( [[ $TELEOP -eq 1 ]] && echo "    * WASD teleop       : w/s fwd-back  a/d turn  q/e strafe  space stop  k quit" )
  Logs : $SIM_LOG
         $BRINGUP_LOG
  Stop : scripts/run_demo.sh stop
============================================================
EOF
