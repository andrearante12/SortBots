#!/usr/bin/env bash
# SortBots real-robot pipeline launcher — run_demo.sh's counterpart on the
# Jetson. Brings up, in one `ros2 launch` (sortbots_bringup.launch.py
# platform:=real):
#   * the RealSense D435 driver, remapped onto the sim's camera topics
#     (launch/sortbots_realsense.launch.py)
#   * RTAB-Map with visual odometry (no base/IMU yet) + the depth filter,
#     map_merge, fleet_radio and the recon-cloud relays — the same graph the
#     dashboard reads in sim
#
# Runs INSIDE docker/jetson's container (scripts/jetson.sh console starts the
# dashboard console there; the Scenarios tab then runs this for you).
#
# Usage:
#   scripts/run_robot.sh [--robot-id robot_0] [--localize | --resume]
#                        [--map PATH] [--keep-console]
#   scripts/run_robot.sh stop [--keep-console]
#
# Map lifecycle is the same three modes as run_demo.sh: default = fresh map,
# --resume = keep mapping into the existing DB, --localize = read-only. --map
# into maps/ is loaded by COPY (scripts/_map_db.sh). Save with the dashboard's
# map-save button or `scripts/maps.sh save NAME` while this is up.
#
# Not started, on purpose: Nav2, task_manager, scripted_pick and the explorer.
# There is no base driver yet, so nothing would move; the dashboard's drive
# pad publishes /<id>/cmd_vel into the void until nodes/base_driver.py exists.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
BRINGUP_LOG="/tmp/sortbots_robot_bringup.log"
DASHBOARD_PORT=8081

# Same split as run_demo.sh: the console stack (owned by run_console.sh) is
# spared under --keep-console.
CONSOLE_PATTERNS=(rosbridge_websocket web_video_server "webui/serve.py" \
                  webui_url.py web_video_watchdog.sh)
# EVERY process the real bringup starts must appear here — one survivor keeps
# the whole `ros2 launch` parent alive into the next run (see run_demo.sh's
# PIPELINE_PATTERNS for the incident). realsense2_camera_node matters most: a
# leftover one keeps the D435 claimed and the next run's driver can't open it.
PIPELINE_PATTERNS=(realsense2_camera_node rtabmap_slam rgbd_odometry rtabmap_viz \
                   rtabmap_util point_cloud_xyzrgb rviz2 \
                   controller_server planner_server smoother_server behavior_server \
                   bt_navigator waypoint_follower velocity_smoother lifecycle_manager \
                   collision_monitor component_container \
                   task_manager.py scripted_pick.py explorer.py \
                   rtabmap_cloud_pump.py recon_cloud_relay.py \
                   map_merge.py static_transform_publisher \
                   fleet_radio.py dynamic_obstacle_filter.py recon_cloud_merge.py)
KEEP_CONSOLE=false

stop_pipeline() {
  if [[ "$KEEP_CONSOLE" == "true" ]]; then
    echo "[run_robot] stopping any running pipeline (keeping the dashboard console)..."
  else
    echo "[run_robot] stopping any running pipeline..."
  fi
  # SIGINT the launch first: rtabmap only flushes its database on a clean
  # shutdown, and on the robot there is no sim to re-run the session from.
  pkill -INT -f "sortbots_bringup.launch.py" 2>/dev/null || true
  sleep 3
  PATTERNS=("${PIPELINE_PATTERNS[@]}")
  [[ "$KEEP_CONSOLE" == "true" ]] || PATTERNS+=("${CONSOLE_PATTERNS[@]}")
  for p in "${PATTERNS[@]}"; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
  sleep 1
}

if [[ "${1:-}" == "stop" ]]; then
  [[ "${2:-}" == "--keep-console" ]] && KEEP_CONSOLE=true
  stop_pipeline
  echo "[run_robot] stopped."
  exit 0
fi

ROBOT_ID=robot_0; LOCALIZE=false; RESUME=false; MAP_DB=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-id) ROBOT_ID="$2"; shift 2;;
    --localize) LOCALIZE=true; shift;;
    --resume)   RESUME=true; shift;;
    --map)      MAP_DB="$2"; shift 2;;
    --keep-console) KEEP_CONSOLE=true; shift;;
    -h|--help) sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'; exit 0;;
    *) echo "[run_robot] unknown arg: $1"; exit 2;;
  esac
done

if [[ "$LOCALIZE" == "true" && "$RESUME" == "true" ]]; then
  echo "ERROR: --localize and --resume are mutually exclusive (read-only vs. keep-mapping)."
  exit 2
fi

if ! [[ -f "$ROS_SETUP" ]]; then
  echo "ERROR: $ROS_SETUP not found — run this inside the robot container"
  echo "       (scripts/jetson.sh console / shell), not on the Jetson host."
  exit 1
fi

source "$SCRIPT_DIR/_map_db.sh"
prepare_map_db "[run_robot]" || exit $?

# Fail fast and legibly on the commonest hardware problem. The driver would
# otherwise sit in wait_for_device forever and the tab would say "starting".
if ! lsusb 2>/dev/null | grep -qi "8086:0b07"; then
  echo "ERROR: no RealSense D435 (8086:0b07) on USB — check the cable, and that"
  echo "       the container was started by scripts/jetson.sh (USB passthrough)."
  exit 1
fi

stop_pipeline

# The "[run_robot] ..." progress lines below are parsed by webui/session.py
# (PHASE_PATTERNS) to drive the Scenarios tab's phase readout. Reword them
# freely, but update that table in the same commit.
DELETE_DB_ON_START=true
if [[ "$LOCALIZE" == "true" ]]; then
  echo "[run_robot] LOCALIZATION mode — reusing the map at $MAP_DB (not modified)."
elif [[ "$RESUME" == "true" ]]; then
  DELETE_DB_ON_START=false
  echo "[run_robot] RESUME mapping — extending the existing map at $MAP_DB."
else
  echo "[run_robot] MAPPING mode — building a fresh map into $MAP_DB."
fi

WEBUI=true; [[ "$KEEP_CONSOLE" == "true" ]] && WEBUI=false
ROS_ENV="source '$ROS_SETUP'; \
  export PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin':\"\$PATH\"; \
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0"

echo "[run_robot] launching RTAB-Map + RealSense D435 for $ROBOT_ID..."
# use_sim_time:=false — there is no /clock on hardware, and with it true every
# node's clock would sit at zero waiting for one.
setsid bash -c "$ROS_ENV; \
  exec ros2 launch '$REPO_ROOT/launch/sortbots_bringup.launch.py' \
       platform:=real robot_id:=$ROBOT_ID robot_ids:=$ROBOT_ID \
       use_sim_time:=false rviz:=false \
       nav2:=false task_manager:=false scripted_pick:=false explore:=false \
       webui:=$WEBUI dashboard_port:=$DASHBOARD_PORT \
       localization:=$LOCALIZE database_path:='$MAP_DB' \
       delete_db_on_start:=$DELETE_DB_ON_START" \
  >"$BRINGUP_LOG" 2>&1 &

# Wait for the camera for real rather than sleeping a fixed time: the D435
# takes a few seconds to enumerate and occasionally resets once on open. The
# message type is REQUIRED here: without it `ros2 topic echo` can't resolve a
# topic nobody publishes yet and exits at once ("could not determine the
# type"), so this reported a timeout while the camera came up 2 s later
# (seen live 2026-09-28).
if bash -c "$ROS_ENV; timeout 60 ros2 topic echo --once \
     /$ROBOT_ID/camera/camera_info sensor_msgs/msg/CameraInfo" >/dev/null 2>&1; then
  echo "[run_robot] camera publishing on /$ROBOT_ID/camera/*."
else
  echo "[run_robot] WARNING: no /$ROBOT_ID/camera/camera_info after 60 s — continuing;"
  echo "            see $BRINGUP_LOG for the realsense driver's output."
fi
echo "[run_robot] ROS 2 stack up (see $BRINGUP_LOG)."

python3 "$REPO_ROOT/scripts/webui_url.py" --port "$DASHBOARD_PORT" 2>/dev/null || \
  echo "[run_robot] dashboard: http://localhost:$DASHBOARD_PORT/"

cat <<EOF

============================================================
  SortBots robot is UP  ($ROBOT_ID, RealSense D435)
    * Web dashboard : http://localhost:$DASHBOARD_PORT/  (or the tailnet URL above)
        - head camera, live SLAM map, 3D reconstruction
        - drive pad / nav goals do nothing yet: no base driver
    * RTAB-Map      : visual odometry, no IMU
    * Map           : $MAP_DB $( [[ "$LOCALIZE" == "true" ]] && echo "(localization — read-only)" || { [[ "$RESUME" == "true" ]] && echo "(resumed — extending existing map)" || echo "(mapping — rebuilt this run)"; } )
  Log  : $BRINGUP_LOG
  Stop : scripts/run_robot.sh stop
============================================================
EOF
