#!/usr/bin/env bash
# The saved-map library: save an explored environment under a name, list what's
# saved, and check that an entry is still intact.
#
# A library map lets a demo or test run start from an already-explored
# warehouse instead of re-exploring for twenty minutes. See maps/README.md for
# the layout and scripts/maps_lib.py for the manifest schema.
#
# Usage:
#   scripts/maps.sh list [--json]
#   scripts/maps.sh show NAME [--json]
#   scripts/maps.sh save NAME [--title T] [--description D] [--robot-id ID]
#                             [--robot-ids a,b] [--scene nvidia]
#                             [--no-live-db] [--no-vacuum] [--force]
#   scripts/maps.sh verify NAME
#   scripts/maps.sh rm NAME [--force]
#
# `save` IS IDEMPOTENT AND STACK-AWARE — run the same command whenever. The two
# artifacts have opposite requirements:
#   * the OCCUPANCY GRID comes off the live, world-anchored /map (the fused grid
#     nodes/map_merge.py publishes, same topic at any robot count), so it can
#     only be captured while the stack is UP.
#   * the RTAB-MAP POSE GRAPH is a live sqlite file. Copying one mid-write gives
#     a torn database, so it is only safe while the stack is DOWN — or through
#     RTAB-Map's own /<rid>/rtabmap/backup service, which closes the file first.
# So a mid-run save may land as db_state "pending" (grid in, pose graph not yet)
# and a second run after teardown promotes it to "complete". One gesture does
# both:  scripts/sim_ctl.sh stop --save-map NAME
#
# NEEDS SYSTEM ROS 2 SOURCED — the exact opposite of run_demo.sh, which exits 1
# when AMENT_PREFIX_PATH is set. This talks to a running graph rather than
# launching one, same as scripts/save_map.sh:
#
#   bash -c 'source /opt/ros/jazzy/setup.bash
#     export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH
#     scripts/maps.sh save warehouse_full --title "NVIDIA warehouse"'
#
# (`list`, `show`, `verify` and `rm` touch no ROS and work in any shell.)
#
# EXIT CODES (stable — branch on these, not on the text):
#   0   success
#   1   failure
#   2   usage error
#   5   saved, but db_state is "pending" — re-run after `sim_ctl.sh stop`
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MAPS_LIB="$SCRIPT_DIR/maps_lib.py"
MAPS_DIR="${SORTBOTS_MAPS_DIR:-$REPO_ROOT/maps}"

# Pinned rather than left to map_saver's defaults: unspecified thresholds make
# it warn and emit whatever it feels like, and this repo already has saved
# yamls disagreeing (free_thresh 0.25 vs 0.196) for exactly that reason.
# scripts/map_coverage.py then has to defend against the ambiguity, so pin it
# here instead. 0.65 matches explorer.yaml's occupied_thresh and
# nav2_params.yaml's costmap threshold. Same values as scripts/save_map.sh.
OCC=0.65; FREE=0.25

# System python3, never conda's: conda's init hook puts its bin dir at the
# front of PATH even after `conda deactivate`, and maps_lib.py needs the same
# interpreter the rest of the ROS-free tooling uses.
PY="$(command -v /usr/bin/python3 || command -v python3)"

die()   { echo "[maps] $*" >&2; exit 1; }
usage() { sed -n '2,44p' "$0"; exit "${1:-2}"; }

# --------------------------------------------------------------------------

# True while any RTAB-Map node is alive. Match the NODE executables, not the
# bare string "rtabmap", and never match this script or its parent shell: a
# loose `pgrep -f rtabmap` also matches any command line that merely MENTIONS
# the database path — including the shell that invoked this script — so the
# guard fires with nothing actually running. (Lifted from scripts/save_map.sh,
# where that bug was diagnosed.)
stack_running() {
  local n
  n=$(pgrep -f "rtabmap_slam|rtabmap_viz|rtabmap_util|rgbd_odometry" \
      | grep -vx -e "$$" -e "$PPID" | wc -l)
  [[ "$n" -gt 0 ]]
}

require_ros() {
  command -v ros2 >/dev/null 2>&1 || die \
    "ros2 not on PATH — this subcommand needs system ROS 2 sourced:
       source /opt/ros/jazzy/setup.bash
       export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:\$PATH"
}

# Wait for a file to exist and stop growing — the backup service returns before
# the copy has necessarily landed, and a half-written file would fail the
# integrity check for no good reason.
wait_for_stable_file() {
  local path="$1" timeout="${2:-120}" last=-1 size stable=0 waited=0
  while (( waited < timeout )); do
    if [[ -f "$path" ]]; then
      size=$(stat -c %s "$path" 2>/dev/null || echo 0)
      if [[ "$size" == "$last" && "$size" -gt 0 ]]; then
        stable=$((stable + 1))
        (( stable >= 2 )) && return 0
      else
        stable=0
      fi
      last="$size"
    fi
    sleep 1; waited=$((waited + 1))
  done
  return 1
}

save_grid() {
  local name="$1" out="$MAPS_DIR/$name/grid"
  mkdir -p "$MAPS_DIR/$name"
  echo "[maps] capturing /map -> $out.{pgm,yaml}"
  # /map, not /<rid>/map: nodes/map_merge.py's fused, world-anchored grid is
  # the same topic no matter how many robots are running.
  if ! ros2 run nav2_map_server map_saver_cli \
        -t "/map" -f "$out" \
        --occ "$OCC" --free "$FREE" --fmt pgm --mode trinary \
        --ros-args -p use_sim_time:=true -p save_map_timeout:=5.0 \
        >"$MAPS_DIR/$name/.save_map.log" 2>&1; then
    echo "[maps] map_saver_cli failed (see $MAPS_DIR/$name/.save_map.log)" >&2
    return 1
  fi
  [[ -f "$out.pgm" ]] || { echo "[maps] map_saver wrote no pgm" >&2; return 1; }
  "$PY" "$MAPS_LIB" add-grid "$name" --yaml "$out.yaml"
}

# Ask RTAB-Map to snapshot its own database. CoreWrapper::backupDatabaseCallback
# closes the DB, copies it to <path>.back, and re-inits on the same path — so
# the copy is taken while the file is CLOSED (not torn), and the live session
# keeps its own database and carries on. It does cost a multi-second stall while
# memory reloads, which is why --no-live-db exists.
save_db_live() {
  local name="$1" rid="$2" vac="$3"
  local live="$HOME/.ros/sortbots_${rid}.db"
  local back="${live}.back"

  if ! ros2 service type "/${rid}/rtabmap/backup" >/dev/null 2>&1; then
    echo "[maps] no /${rid}/rtabmap/backup service — leaving the pose graph pending" >&2
    return 1
  fi
  rm -f "$back"
  echo "[maps] asking rtabmap to back up its database (this stalls mapping briefly)"
  if ! ros2 service call "/${rid}/rtabmap/backup" std_srvs/srv/Empty "{}" >/dev/null 2>&1; then
    echo "[maps] backup service call failed — leaving the pose graph pending" >&2
    return 1
  fi
  if ! wait_for_stable_file "$back" 180; then
    echo "[maps] $back never settled — leaving the pose graph pending" >&2
    return 1
  fi
  "$PY" "$MAPS_LIB" add-db "$name" --src "$back" --robot-id "$rid" $vac || return 1
  rm -f "$back"
}

save_db_offline() {
  local name="$1" rid="$2" vac="$3"
  local live="$HOME/.ros/sortbots_${rid}.db"
  [[ -f "$live" ]] || { echo "[maps] no database at $live" >&2; return 1; }
  "$PY" "$MAPS_LIB" add-db "$name" --src "$live" --robot-id "$rid" $vac
}

# --------------------------------------------------------------------------

cmd_save() {
  local name="" title="" desc="" rid="robot_0" rids="" scene="nvidia"
  local live_db=true vac="" force=false
  [[ $# -gt 0 ]] || usage 2
  name="$1"; shift
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --title)       title="$2"; shift 2;;
      --description) desc="$2";  shift 2;;
      --robot-id)    rid="$2";   shift 2;;
      --robot-ids)   rids="$2";  shift 2;;
      --scene)       scene="$2"; shift 2;;
      --no-live-db)  live_db=false; shift;;
      --no-vacuum)   vac="--no-vacuum"; shift;;
      --force)       force=true; shift;;
      *) echo "[maps] unknown arg: $1" >&2; exit 2;;
    esac
  done

  "$PY" "$MAPS_LIB" init "$name" \
      ${title:+--title "$title"} ${desc:+--description "$desc"} \
      --scene "$scene" --robot-id "$rid" ${rids:+--robot-ids "$rids"} || exit 1

  if stack_running; then
    require_ros
    save_grid "$name" || exit 1
    if [[ "$live_db" == true ]]; then
      save_db_live "$name" "$rid" "$vac" || true
    else
      echo "[maps] --no-live-db: pose graph deferred until the stack is down"
    fi
  else
    echo "[maps] stack is down — grid kept as-is, copying the pose graph"
    if [[ ! -f "$MAPS_DIR/$name/grid.yaml" && "$force" != true ]]; then
      die "$name has no grid and the stack is down, so one can't be captured.
       Start the sim and save again, or pass --force to keep a DB-only entry."
    fi
    save_db_offline "$name" "$rid" "$vac" || exit 1
  fi

  local state
  state=$("$PY" "$MAPS_LIB" show "$name" --json | "$PY" -c \
          'import json,sys; print(json.load(sys.stdin)["db_state"])')
  if [[ "$state" != "complete" ]]; then
    echo "[maps] $name saved with db_state=$state — re-run after 'scripts/sim_ctl.sh stop'"
    echo "       (or use: scripts/sim_ctl.sh stop --save-map $name)"
    exit 5
  fi
  echo "[maps] $name saved (db_state=complete)"
  echo "       load it: scripts/sim_ctl.sh start library_localize map=$MAPS_DIR/$name/map.db"
}

cmd_rm() {
  local name="${1:-}" force=false
  [[ -n "$name" ]] || usage 2
  shift
  [[ "${1:-}" == "--force" ]] && force=true
  if [[ "$force" != true ]]; then
    read -r -p "[maps] permanently remove $MAPS_DIR/$name? [y/N] " reply
    [[ "$reply" == [yY]* ]] || { echo "[maps] cancelled"; exit 0; }
  fi
  "$PY" "$MAPS_LIB" rm "$name"
}

case "${1:-}" in
  list)   shift; exec "$PY" "$MAPS_LIB" list "$@";;
  show)   shift; exec "$PY" "$MAPS_LIB" show "$@";;
  verify) shift; exec "$PY" "$MAPS_LIB" verify "$@";;
  save)   shift; cmd_save "$@";;
  rm)     shift; cmd_rm "$@";;
  -h|--help) usage 0;;
  *) usage 2;;
esac
