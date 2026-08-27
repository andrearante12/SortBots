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
#   scripts/run_demo.sh [--robot-id robot_0] [--robots 1] [--robot-ids IDS]
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
# Leaving out --robots (and --chase-cam-robots) sizes both to this machine's
# detected VRAM/RAM instead of defaulting to 1 — see scripts/_hw_budget.py.
# An explicit value here always wins over that auto-pick.
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
#   scripts/run_demo.sh --teleop              # drive around; map builds from scratch
#   scripts/run_demo.sh stop                  # map persists in ~/.ros/sortbots_<id>.db
#   scripts/run_demo.sh --localize            # read-only: reopen that map, send nav goals
#   scripts/run_demo.sh --resume --explore    # keep MAPPING: extend that map, autonomously
#
# Default (neither flag) starts every run from an empty database, so a plain
# mapping run always means what it says. --localize never modifies the map at
# all. --resume is the middle ground: still mapping (RTAB-Map keeps growing
# the pose graph), just starting from the existing database instead of an
# empty one — "continue exploring from where a prior session left off".
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS2_HOME="${ROS2_HOME:-$HOME/ros2_jazzy}"
ROS_SETUP="${ROS2_HOME}/install/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  ROS_SETUP="/opt/ros/jazzy/setup.bash"  # fallback for binary install
fi
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
PIPELINE_PATTERNS=(rtabmap_slam rtabmap_viz rtabmap_util point_cloud_xyzrgb rviz2 \
                   controller_server planner_server smoother_server behavior_server \
                   bt_navigator waypoint_follower velocity_smoother lifecycle_manager \
                   component_container task_manager.py scripted_pick.py explorer.py \
                   rtabmap_cloud_pump.py recon_cloud_relay.py wasd_teleop \
                   map_merge.py static_transform_publisher \
                   "spawn_warehouse.py" \
                   ns3_mesh_bridge fastdds \
                   fast-discovery-server) # the "fastdds" wrapper execs this as a
                   # grandchild whose argv never contains "fastdds" itself, so it
                   # survived every pkill -f fastdds and kept UDP 11811 bound
                   # across runs (diagnosed 2026-08-27, port-allocation failures).
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
  # Tear down mesh netns/taps if they exist (idempotent, no-op otherwise)
  if sudo -n true 2>/dev/null && [[ -f "$REPO_ROOT/scripts/mesh_netns_teardown.sh" ]]; then
    sudo "$REPO_ROOT/scripts/mesh_netns_teardown.sh" \
      --robots "${ROBOTS:-1}" --robot-ids "${ROBOT_IDS:-}" 2>/dev/null || true
  fi
}

if [[ "${1:-}" == "stop" ]]; then
  [[ "${2:-}" == "--keep-console" ]] && KEEP_CONSOLE=true
  stop_pipeline
  echo "[run_demo] stopped."
  exit 0
fi

# ---- args ----
# Teleop defaults OFF — the dashboard drive pad replaces the WASD window.
# ROBOTS starts unset (not "1"): that's the signal, below, that the caller
# didn't pin a count and scripts/_hw_budget.py should size it to this
# machine's VRAM/RAM instead. --robots on the CLI (or a scenario's pinned
# run.robots, which arrives here the same way) always overrides it.
ROBOT_ID=robot_0; ROBOTS=""; ROBOT_IDS=""; SCENE=nvidia; HEADLESS="--no-headless"; TELEOP=0; MESH=0
NS3_HOME="${NS3_HOME:-$HOME/ns-3-dev}"
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
    --mesh)     MESH=1; shift;;
    # Set by webui/session.py when the Scenarios tab launches a run: the
    # dashboard console is already up and owns rosbridge/web_video_server, so
    # don't kill it on teardown and don't start a second copy in the bringup.
    --keep-console) KEEP_CONSOLE=true; shift;;
    -h|--help) sed -n '2,53p' "$0"; exit 0;;
    *) echo "[run_demo] unknown arg: $1"; exit 2;;
  esac
done

if [[ "$LOCALIZE" == "true" && "$RESUME" == "true" ]]; then
  echo "ERROR: --localize and --resume are mutually exclusive (read-only vs. keep-mapping)."
  exit 2
fi

# No --robots on the CLI (see the ROBOTS="" comment above) -> size the run to
# this machine's VRAM/RAM instead of always defaulting to 1. Explicit
# --chase-cam-robots/--no-chase-cam still wins outright; only fills in the
# count when the caller left both robots AND chase-cam-robots unset.
if [[ -z "$ROBOTS" ]]; then
  _HW_MAX_ROBOTS=$(python3 -c "
import yaml
with open('$REPO_ROOT/configs/robots.yaml') as f:
    print(len(yaml.safe_load(f)['robots']))
")
  read -r ROBOTS _HW_CHASE_N _HW_NOTE < <(python3 "$SCRIPT_DIR/_hw_budget.py" --pick --max-robots "$_HW_MAX_ROBOTS")
  [[ -z "$CHASE_CAM_ARGS" ]] && CHASE_CAM_ARGS="--chase-cam-robots $_HW_CHASE_N"
  echo "[run_demo] hw-budget: $_HW_NOTE"
fi
ROBOTS="${ROBOTS:-1}"   # detection itself failed (no python3/nvidia-smi) — old default

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

# ---- 0. 802.11s mesh bridge (optional) ----
DDS_PROFILE_DIR="/tmp/sortbots_dds"
if [[ $MESH -eq 1 ]]; then
  echo "[run_demo] --mesh: setting up 802.11s mesh (ns-3 + Linux netns)"

  # netns, veth, tap — requires sudo
  sudo "$REPO_ROOT/scripts/mesh_netns_setup.sh" \
    --robots "$ROBOTS" --robot-ids "$ROBOT_IDS"

  # Render FastDDS XML profiles (no sudo needed)
  python3 "$REPO_ROOT/network/render_dds_profile.py" \
    --output-dir "$DDS_PROFILE_DIR" \
    --ds-host "127.0.0.1" --ds-port 11811 \
    --robot-ids "$ROBOT_IDS" --robots "$ROBOTS"

  # FastDDS Discovery Server in the root netns (background)
  # fastdds CLI is provided by ros-jazzy-fastrtps; source ROS inside subshell
  # so we don't pollute this clean shell with AMENT_PREFIX_PATH.
  setsid bash -c "source '$ROS_SETUP'; \
    export PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin':\"\$PATH\"; \
    exec fastdds discovery -i 0 -l 0.0.0.0 -p 11811" \
    >"/tmp/sortbots_fastdds.log" 2>&1 &
  sleep 1
  echo "[run_demo] FastDDS Discovery Server started (log: /tmp/sortbots_fastdds.log)"

  # ns-3 mesh bridge — waits for all TapBridge attachments before returning
  "$REPO_ROOT/scripts/run_mesh.sh" \
    --robots "$ROBOTS" --robot-ids "$ROBOT_IDS" \
    --config "$REPO_ROOT/network/mesh_config.yaml"
  echo "[run_demo] ns-3 mesh bridge up (log: /tmp/sortbots_ns3_mesh.log)"

  # --- Move taps to netns and assign mesh IPs ---
  # Must happen BEFORE MAC alignment below. Doing the down/address/up cycle
  # on a tap still in the root ns and then immediately netns-moving it stacks
  # two live-FD state transitions back to back — a combination test_mesh_phase1.sh
  # never exercises and which left taps NO-CARRIER in practice (diagnosed live,
  # 2026-08-27). test_mesh_phase1.sh's proven-safe order is: move to netns
  # first (mesh_tap_to_netns.sh's own FD-survives-the-move guarantee), then
  # align the MAC inside the netns as its own, single down/up cycle.
  sudo "$REPO_ROOT/scripts/mesh_tap_to_netns.sh" \
    --robots "$ROBOTS" --robot-ids "$ROBOT_IDS"

  # --- MAC alignment (inside each netns, after the move) ---
  # ns-3 TapBridge assigns its own MAC to the taps. The Linux TAP driver's
  # default MAC may differ, causing the kernel to drop inbound frames as
  # PACKET_OTHERHOST (Phase 1 lesson). Read the MACs ns-3 chose from its log
  # and apply them to the tap devices, now that they're in their final netns.
  echo "[run_demo] aligning tap MACs to ns-3 mesh MACs..."
  _MESH_LOG="/tmp/sortbots_ns3_mesh.log"
  IFS=',' read -ra _TAP_IDS <<< "$ROBOT_IDS"
  declare -a _MESH_MACS=()
  for _tid in "${_TAP_IDS[@]}"; do
    _TAP="tap-${_tid}"
    _NETNS="ns-${_tid}"
    _M=$(grep "node-mac ${_TAP}" "$_MESH_LOG" 2>/dev/null | awk '{print $4}')
    if [[ -z "$_M" ]]; then
      echo "[run_demo] WARNING: could not read ns-3 MAC for ${_TAP} — skipping MAC alignment"
      _M=""
    else
      sudo ip netns exec "$_NETNS" ip link set "$_TAP" down
      sudo ip netns exec "$_NETNS" ip link set "$_TAP" address "$_M"
      sudo ip netns exec "$_NETNS" ip link set "$_TAP" up
      echo "[run_demo]   ${_TAP} MAC → $_M"
    fi
    _MESH_MACS+=("$_M")
  done

  # --- Static ARP inside each netns (bypass HWMP ARP timing) ---
  # HWMP peer tables take time to converge. Pre-populating ARP entries lets
  # DDS send the very first SPDP frame without waiting for ARP resolution,
  # which would time out before HWMP has a path (Phase 1 lesson).
  _I=0
  for _rid in "${_TAP_IDS[@]}"; do
    _NETNS="ns-${_rid}"
    _J=0
    for _rid2 in "${_TAP_IDS[@]}"; do
      if [[ "$_rid" != "$_rid2" ]]; then
        _PEER_IP="10.66.0.$((_J + 1))"
        _PEER_MAC="${_MESH_MACS[$_J]}"
        _TAP="tap-${_rid}"
        [[ -n "$_PEER_MAC" ]] && \
          sudo ip netns exec "$_NETNS" ip neigh replace "$_PEER_IP" lladdr "$_PEER_MAC" dev "$_TAP" 2>/dev/null || true
      fi
      _J=$((_J + 1))
    done
    _I=$((_I + 1))
  done
  echo "[run_demo] waiting 15s for HWMP mesh routes to converge..."
  sleep 15
fi

# ---- 1. Isaac Sim ----
echo "[run_demo] launching Isaac Sim ($SCENE, $ROBOTS robot(s), $HEADLESS)..."
rm -f "$SIM_RESULT"
ISAAC_EXTRA_ENV=""
if [[ $MESH -eq 1 ]]; then
  ISAAC_EXTRA_ENV="export FASTRTPS_DEFAULT_PROFILES_FILE='$DDS_PROFILE_DIR/profile_root.xml'; \
    export ROS_DISCOVERY_SERVER='127.0.0.1:11811'; "
fi
setsid bash -c "$ISAAC_EXTRA_ENV source '$REPO_ROOT/scripts/activate_isaac.sh' >/dev/null 2>&1; \
  exec python '$REPO_ROOT/scripts/spawn_warehouse.py' $HEADLESS --forever \
       --robots $ROBOTS --scene $SCENE --drive cmd_vel $CHASE_CAM_ARGS" \
  >"$SIM_LOG" 2>&1 &
SIM_PID=$!

echo "[run_demo] waiting for the warehouse to load"
echo "           (first run streams assets from NVIDIA — can take a few minutes)..."
# Liveness, not log text, decides failure here: optional extensions
# (asset_converter/mjcf/urdf importers) log an ERROR + Traceback on Kit
# startup whenever the host is missing their bundled libxml2.so.2 (true on
# Ubuntu 25.10, which only ships libxml2-16 — diagnosed 2026-08-27). Those
# are non-fatal and were tripping this loop's old `grep -qiE "ERROR:|..."`
# check well before the warehouse ever got a chance to load. `setsid ...
# exec python` keeps SIM_PID valid across the setsid->bash->python chain,
# so kill -0 is the same "is it actually still running" check CLAUDE.md
# already prescribes (pgrep -f spawn_warehouse.py) — just PID-scoped.
READY=0
for _ in $(seq 1 72); do
  if grep -q "timeline.play()" "$SIM_RESULT" 2>/dev/null; then READY=1; break; fi
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "[run_demo] Isaac Sim process exited before becoming ready — see $SIM_LOG"; exit 1
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

if [[ $MESH -eq 1 ]]; then
  # Mesh mode (Phase 4): each robot stack runs inside its own network namespace
  # (ns-robot_i). Isolation is REAL: the robot process can only reach the mesh
  # tap (10.66.0.x, for inter-robot DDS) and its own veth (10.77.i.2 → DS/Isaac
  # in root ns). iptables REJECT rules block the cross-robot veth shortcuts,
  # forcing all inter-robot DDS traffic through the ns-3 simulated mesh tap.
  # DS + map_merge + dashboard stay in root ns with the root profile.
  IFS=',' read -ra _MESH_IDS <<< "$ROBOT_IDS"
  _RID_IDX=0
  for _rid in "${_MESH_IDS[@]}"; do
    _PROFILE="$DDS_PROFILE_DIR/profile_${_rid}.xml"
    _LOG="/tmp/sortbots_bringup_${_rid}.log"
    # DS is reachable via the veth host end (10.77.{i}.1) in the robot's netns.
    _DS_ADDR="10.77.${_RID_IDX}.1:11811"
    # Write launch script to /tmp to avoid bash quoting issues across ip-netns-exec
    # (Phase 3 lesson: heredoc quoting with nested quotes inside a setsid/bash -c
    # string causes subtle eval errors that are hard to spot in logs).
    _LAUNCH_SH="/tmp/sortbots_mesh_launch_${_rid}.sh"
    cat >"$_LAUNCH_SH" <<LAUNCHEOF
#!/bin/bash
# sudo ip netns exec runs this as root, which resets HOME to /root — Python's
# user-site lookup then misses lark/catkin_pkg/etc., installed under this
# user's site because pip has no permission to write system-wide (diagnosed
# 2026-08-27: "ros2 launch" failed in-netns with "No module named 'lark'"
# despite working fine outside the namespace). Pin HOME back explicitly
# rather than sudo -E, to avoid pulling in the rest of the caller's env.
export HOME='$HOME'
source '$ROS_SETUP' 2>/dev/null
export PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin':\$PATH
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE='${_PROFILE}'
export ROS_DISCOVERY_SERVER='${_DS_ADDR}'
exec ros2 launch '$REPO_ROOT/launch/sortbots_bringup.launch.py' \
     robot_id:=${_rid} robot_ids:=${_rid} scene:=$SCENE \
     use_sim_time:=true rviz:=false webui:=false \
     localization:=$LOCALIZE database_path:='$MAP_DB' \
     delete_db_on_start:=$DELETE_DB_ON_START \
     explore:=$EXPLORE
LAUNCHEOF
    chmod +x "$_LAUNCH_SH"
    setsid sudo ip netns exec "ns-${_rid}" bash "$_LAUNCH_SH" >"$_LOG" 2>&1 &
    echo "[run_demo] launched mesh bringup for $_rid in ns-${_rid} (log: $_LOG)"
    _RID_IDX=$((_RID_IDX + 1))
  done

  # map_merge + dashboard: root profile (loopback only, sees all robots via DS)
  setsid bash -c "source '$ROS_SETUP'; \
    export PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin':\"\$PATH\"; \
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0; \
    export FASTRTPS_DEFAULT_PROFILES_FILE='$DDS_PROFILE_DIR/profile_root.xml'; \
    export ROS_DISCOVERY_SERVER='127.0.0.1:11811'; \
    exec ros2 launch '$REPO_ROOT/launch/sortbots_bringup.launch.py' \
         robot_id:=$ROBOT_ID robot_ids:=$ROBOT_IDS scene:=$SCENE \
         use_sim_time:=true rviz:=false \
         webui:=$WEBUI dashboard_port:=$DASHBOARD_PORT \
         localization:=$LOCALIZE database_path:='$MAP_DB' \
         delete_db_on_start:=$DELETE_DB_ON_START \
         explore:=false" \
    >"$BRINGUP_LOG" 2>&1 &
else
  # Standard (no mesh): single bringup for all robots on the host netns
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
fi
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
