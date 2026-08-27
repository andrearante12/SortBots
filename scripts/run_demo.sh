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
#   scripts/run_demo.sh [--robot-id robot_0] [--robots 2] [--robot-ids IDS]
#                       [--scene nvidia]
#                       [--headless] [--teleop] [--localize | --resume]
#                       [--no-chase-cam | --chase-cam-robots N]
#                       [--map PATH] [--explore] [--keep-console]
#   scripts/run_demo.sh stop [--keep-console]   # tear the pipeline down
#
# --robots N spawns N robots in Isaac (configs/robots.yaml order) AND brings
# up a full RTAB-Map + Nav2 + task_manager + explorer stack for each — the
# same N, always, because the ROS side derives --robot-ids from --robots
# against that same roster unless you pass --robot-ids explicitly. Every
# robot's own SLAM grid gets fused into one world-anchored /map (see
# nodes/map_merge.py) that Nav2 and every explorer plan against by default,
# so robots explore collaboratively rather than as independent single-robot
# runs sharing a warehouse. --robot-id (singular) still names which ONE
# robot --localize/--resume/--map/--teleop apply to.
#
# Default is --robots 2: SortBots is a fleet project, so the plain demo comes
# up as a collaborative fleet. Pass --robots 1 for the single-robot case
# (map-lifecycle / teleop work below is written that way).
#
# --keep-console leaves the dashboard stack (rosbridge, web_video_server,
# webui/serve.py) alone and tells the bringup not to start its own copy. It is
# what scripts/run_console.sh + the dashboard's Scenarios tab use, so the page
# you clicked "Start" on survives the run it launched. Not needed by hand.
#
# --explore starts nodes/explorer.py, which autonomously maps the warehouse
# with no human input (frontier-based: drives toward the nearest unexplored
# opening until none remain). Combine with --localize for hands-off
# task_manager dispatch on an already-built map (nodes/task_manager.py and
# nodes/explorer.py arbitrate over the shared Nav2 goal server — see
# task_manager.py's module docstring) — NOT for continuing to explore new
# territory, since --localize is read-only and never grows the map. For
# that, use --resume --explore (see below).
#
# Re-running it auto-stops any existing pipeline first, so it doubles as a
# "reset the environment" button.
#
# Map lifecycle — three modes, build once and reuse:
#
#   scripts/run_demo.sh --robots 1 --teleop            # drive around; map builds from scratch
#   scripts/run_demo.sh stop                           # map persists in ~/.ros/sortbots_<id>.db
#   scripts/run_demo.sh --robots 1 --localize          # read-only: reopen that map, send nav goals
#   scripts/run_demo.sh --robots 1 --resume --explore  # keep MAPPING: extend that map, autonomously
#
# Default (neither flag) starts every run from an empty database, so a plain
# mapping run always means what it says. --localize never modifies the map at
# all. --resume is the middle ground: still mapping (RTAB-Map keeps growing
# the pose graph), just starting from the existing database instead of an
# empty one — "continue exploring from where a prior session left off".
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
SIM_LOG="/tmp/sortbots_demo_sim.log"
SIM_RESULT="/tmp/isaac_spawn_warehouse_result.txt"
BRINGUP_LOG="/tmp/sortbots_demo_bringup.log"
DASHBOARD_PORT=8081

# The dashboard stack (rosbridge, web_video_server, webui/serve.py) is listed
# separately because scripts/run_console.sh can own it INSTEAD of the bringup:
# in console mode those processes outlive individual sim runs, which is what
# lets the Scenarios tab start and stop the pipeline from the browser without
# killing the page it's served from. --keep-console spares them.
CONSOLE_PATTERNS=(rosbridge_websocket web_video_server "webui/serve.py" \
                  webui_url.py web_video_watchdog.sh)
# EVERY process the bringup starts must appear here. This list is not just
# belt-and-braces over the SIGINT above: `ros2 launch` waits for all of its
# ExecuteProcess children before exiting, so ONE unlisted node keeps the whole
# launch parent alive, and the entire subtree survives into the next run. The
# next run then brings up a SECOND copy of every node under the same node
# names, and duplicate lifecycle node names in particular make
# /<robot>/<node>/change_state ambiguous. Found 2026-08-09 with two full
# generations of bringup live at once (fleet_radio, dynamic_obstacle_filter and
# collision_monitor x2 per robot) after several "stopped" runs — the four
# newest nodes at the time had never been added here.
PIPELINE_PATTERNS=(rtabmap_slam rtabmap_viz rtabmap_util point_cloud_xyzrgb rviz2 \
                   controller_server planner_server smoother_server behavior_server \
                   bt_navigator waypoint_follower velocity_smoother lifecycle_manager \
                   collision_monitor \
                   component_container task_manager.py scripted_pick.py explorer.py \
                   rtabmap_cloud_pump.py recon_cloud_relay.py wasd_teleop \
                   map_merge.py static_transform_publisher \
                   fleet_radio.py dynamic_obstacle_filter.py recon_cloud_merge.py \
                   "spawn_warehouse.py")
# Note scripts/save_map.sh --watch is deliberately absent from that list. Its
# checkpoint loop has to OUTLIVE teardown: rtabmap gets pkill -9'd only 2 s
# after SIGINT below, and the last checkpoint is exactly the one worth
# keeping. The loop ends itself once /map stops answering.
KEEP_CONSOLE=false

stop_pipeline() {
  if [[ "$KEEP_CONSOLE" == "true" ]]; then
    echo "[run_demo] stopping any running pipeline (keeping the dashboard console)..."
  else
    echo "[run_demo] stopping any running pipeline..."
  fi
  # SIGINT the top-level launch first so it can shut its children (Nav2
  # lifecycle nodes, rtabmap, rosbridge) down cleanly, then hard-kill leftovers.
  pkill -INT -f "sortbots_bringup.launch.py" 2>/dev/null || true
  pkill -INT -f "sortbots_rtabmap_robot.launch.py" 2>/dev/null || true
  sleep 2
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
  echo "[run_demo] stopped."
  exit 0
fi

# ---- args ----
# Teleop defaults OFF — the dashboard drive pad replaces the WASD window.
# ROBOTS defaults to 2 — SortBots is a fleet project, so the bare demo comes up
# as a collaborative fleet (fused /map, mesh radio). --robots 1 for single-robot.
ROBOT_ID=robot_0; ROBOTS=2; ROBOT_IDS=""; SCENE=nvidia; HEADLESS="--no-headless"; TELEOP=0
# Cosmetic 3rd-person cam = a second render product per robot. --no-chase-cam
# drops all of them; --chase-cam-robots N keeps them on the first N robots
# only (dashboard only shows robot_id's feed, so fleet runs usually want 1).
CHASE_CAM_ARGS=""
LOCALIZE=false; RESUME=false; MAP_DB=""; EXPLORE=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-id) ROBOT_ID="$2"; shift 2;;
    --robots)   ROBOTS="$2";   shift 2;;
    --robot-ids) ROBOT_IDS="$2"; shift 2;;
    --scene)    SCENE="$2";    shift 2;;
    --headless) HEADLESS="--headless"; shift;;
    --no-chase-cam) CHASE_CAM_ARGS="--no-chase-cam"; shift;;
    --chase-cam-robots) CHASE_CAM_ARGS="--chase-cam-robots $2"; shift 2;;
    --teleop)    TELEOP=1; shift;;
    --no-teleop) TELEOP=0; shift;;  # accepted for back-compat (already the default)
    --localize) LOCALIZE=true; shift;;
    --resume)   RESUME=true; shift;;
    --map)      MAP_DB="$2";  shift 2;;
    --explore)  EXPLORE=true; shift;;
    # Set by webui/session.py when the Scenarios tab launches a run: the
    # dashboard console is already up and owns rosbridge/web_video_server, so
    # don't kill it on teardown and don't start a second copy in the bringup.
    --keep-console) KEEP_CONSOLE=true; shift;;
    -h|--help) sed -n '2,57p' "$0"; exit 0;;
    *) echo "[run_demo] unknown arg: $1"; exit 2;;
  esac
done

if [[ "$LOCALIZE" == "true" && "$RESUME" == "true" ]]; then
  echo "ERROR: --localize and --resume are mutually exclusive (read-only vs. keep-mapping)."
  exit 2
fi

# --robot-ids threads straight through to the bringup's robot_ids:= (which
# brings up a full RTAB-Map + Nav2 + task_manager + explorer stack per id —
# see launch/sortbots_bringup.launch.py). Left unset (the common case), it's
# derived from --robots against configs/robots.yaml's order — the same
# roster scripts/spawn_warehouse.py --robots N uses to place robots in Isaac,
# so "N robots spawned" and "N robots brought up" can't drift apart.
if [[ -z "$ROBOT_IDS" ]]; then
  ROBOT_IDS=$(python3 -c "
import yaml
with open('$REPO_ROOT/configs/robots.yaml') as f:
    roster = yaml.safe_load(f)
ids = [r['id'] for r in roster['robots']][:$ROBOTS]
print(','.join(ids))
")
fi

# Default matches launch/sortbots_rtabmap_robot.launch.py's database_path.
[[ -z "$MAP_DB" ]] && MAP_DB="$HOME/.ros/sortbots_${ROBOT_ID}.db"

# COPY-ON-USE for the saved-map library (maps/, see maps/README.md).
#
# A library entry is a COMMITTED artifact. RTAB-Map opens its sqlite file
# read-write even under --localize (Mem/IncrementalMemory=false stops it
# LEARNING, not WRITING), so pointing the stack straight at maps/ would dirty a
# tracked ~150-500 MB git-lfs object on every demo. Copy to the working DB and
# run on that instead; --resume then extends the COPY, and keeping that result
# is always an explicit `scripts/maps.sh save`.
MAP_DB="$(readlink -f -- "$MAP_DB" 2>/dev/null || echo "$MAP_DB")"
if [[ "$MAP_DB" == "$REPO_ROOT/maps/"* ]]; then
  if [[ ! -f "$MAP_DB" ]]; then
    echo "ERROR: no library map at $MAP_DB  (scripts/maps.sh list)"
    exit 1
  fi
  # A clone without git-lfs leaves a ~130-byte pointer file here. It LOOKS
  # present, so RTAB-Map opens it and fails deep inside sqlite; say the real
  # thing instead.
  if head -c 23 -- "$MAP_DB" 2>/dev/null | grep -q "^version https://git-lfs"; then
    echo "ERROR: $MAP_DB is an unfetched git-lfs pointer, not a database."
    echo "       Fetch it first:  git lfs pull"
    exit 1
  fi
  if [[ "$LOCALIZE" != "true" && "$RESUME" != "true" ]]; then
    echo "ERROR: --map into maps/ needs --localize or --resume; plain mapping"
    echo "       mode would wipe the copy on start (delete_db_on_start:=true)."
    exit 2
  fi
  WORK_DB="$HOME/.ros/sortbots_${ROBOT_ID}.db"
  echo "[run_demo] library map $(basename "$(dirname "$MAP_DB")") -> $WORK_DB (library entry untouched)"
  mkdir -p "$HOME/.ros"
  cp -f -- "$MAP_DB" "$WORK_DB" || exit 1
  MAP_DB="$WORK_DB"
fi

# Localizing/resuming against a map that was never built is a confusing
# failure mode — RTAB-Map comes up, publishes nothing, and every nav goal is
# rejected for want of a map. Catch it here instead.
if [[ ( "$LOCALIZE" == "true" || "$RESUME" == "true" ) && ! -f "$MAP_DB" ]]; then
  echo "ERROR: --localize/--resume need an existing map, but $MAP_DB does not exist."
  echo "       Build one first:  scripts/run_demo.sh --teleop"
  exit 1
fi

# --resume passes delete_db_on_start:=false explicitly below (bringup's
# default is true otherwise) — mapping mode (RTAB-Map keeps growing the pose
# graph, unlike --localize's read-only Mem/IncrementalMemory=false), started
# from the EXISTING database instead of wiping it. This is the "continue
# exploring from where a prior session left off" mode.

# Must be a clean shell — Isaac's activate refuses if ROS 2 is sourced.
if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "ERROR: this shell has ROS 2 sourced (AMENT_PREFIX_PATH set)."
  echo "       Open a fresh terminal and run again WITHOUT sourcing ROS first."
  exit 1
fi

# Always reset first so re-running is a clean restart.
stop_pipeline

# The "[run_demo] ..." progress lines below are parsed by webui/session.py
# (PHASE_PATTERNS) to drive the Scenarios tab's phase readout. Reword them
# freely, but update that table in the same commit.

# ---- 1. Isaac Sim ----
echo "[run_demo] launching Isaac Sim ($SCENE, $ROBOTS robot(s), $HEADLESS)..."
rm -f "$SIM_RESULT"
setsid bash -c "source '$REPO_ROOT/scripts/activate_isaac.sh' >/dev/null 2>&1; \
  exec python '$REPO_ROOT/scripts/spawn_warehouse.py' $HEADLESS --forever \
       --robots $ROBOTS --scene $SCENE --drive cmd_vel $CHASE_CAM_ARGS" \
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
DELETE_DB_ON_START=true
if [[ "$LOCALIZE" == "true" ]]; then
  echo "[run_demo] LOCALIZATION mode — reusing the map at $MAP_DB (not modified)."
elif [[ "$RESUME" == "true" ]]; then
  DELETE_DB_ON_START=false
  echo "[run_demo] RESUME mapping — extending the existing map at $MAP_DB."
else
  echo "[run_demo] MAPPING mode — building a fresh map into $MAP_DB."
fi
echo "[run_demo] launching RTAB-Map + Nav2 + web dashboard + task manager..."
# Prepend the system path so rosbridge_websocket's `#!/usr/bin/env python3`
# shebang resolves to /usr/bin/python3 even if this script was started from a
# conda-active shell (conda's python breaks rclpy's compiled extension — see
# launch/sortbots_webui.launch.py's docstring).
# In console mode the dashboard stack is already up and outside this pipeline,
# so the bringup must NOT start a second one — two rosbridges would fight over
# port 9090 and the loser dies silently.
WEBUI=true; [[ "$KEEP_CONSOLE" == "true" ]] && WEBUI=false
setsid bash -c "source '$ROS_SETUP'; \
  export PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin':\"\$PATH\"; \
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0; \
  exec ros2 launch '$REPO_ROOT/launch/sortbots_bringup.launch.py' \
       robot_id:=$ROBOT_ID robot_ids:=$ROBOT_IDS scene:=$SCENE \
       use_sim_time:=true rviz:=false \
       webui:=$WEBUI dashboard_port:=$DASHBOARD_PORT \
       localization:=$LOCALIZE database_path:='$MAP_DB' \
       delete_db_on_start:=$DELETE_DB_ON_START \
       explore:=$EXPLORE" \
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
    * Isaac Sim window  : NVIDIA warehouse + $ROBOT_IDS
    * Web dashboard     : http://localhost:$DASHBOARD_PORT/  (or the tailnet URL above)
        - live SLAM map + trail, Nav2 path/costmap overlays, SLAM status
        - drag on the map to send a Nav2 goal; drive pad; pickup->dropoff dispatch
    * Nav2 + RTAB-Map   : running headless (rviz off; the dashboard replaces it)
    * Map               : $MAP_DB $( [[ "$LOCALIZE" == "true" ]] && echo "(localization — read-only)" || { [[ "$RESUME" == "true" ]] && echo "(resumed — extending existing map)" || echo "(mapping — rebuilt this run)"; } )
$( [[ $EXPLORE == "true" ]] && echo "    * Explorer          : autonomous frontier exploration — no human input needed" )
$( [[ $TELEOP -eq 1 ]] && echo "    * WASD teleop       : w/s fwd-back  a/d turn  q/e strafe  space stop  k quit" )
  Logs : $SIM_LOG
         $BRINGUP_LOG
  Stop : scripts/run_demo.sh stop
============================================================
EOF
