#!/usr/bin/env bash
# Save the occupancy grid RTAB-Map is publishing, into data/runs/<name>/map/.
#
# Two things are worth saving off a mapping run, and they have different
# lifetimes:
#   * the OCCUPANCY GRID (.pgm + .yaml) — what Nav2 and the dashboard render,
#     and what scripts/map_coverage.py measures. Written from the live,
#     world-anchored /map topic (nodes/map_merge.py's fused grid — the same
#     topic regardless of robot count), so it can only be captured while the
#     stack is up. --robot-id doesn't affect this half.
#   * the RTAB-MAP DATABASE (.db) — one robot's own full pose graph, needed
#     to resume ITS session later (run_demo.sh --resume --robot-id <rid> --map
#     <path>). A live sqlite file, so it can only be copied safely once the
#     stack is DOWN (--db); --robot-id selects which robot's DB.
#
# Usage:
#   scripts/save_map.sh --run NAME [--label final]     # one shot, stack running
#   scripts/save_map.sh --run NAME --watch 3 &         # checkpoint every 3 min
#   scripts/save_map.sh --stop-watch                   # end that loop
#   scripts/save_map.sh --run NAME --db                # after run_demo.sh stop
#
# A typical exploration run:
#   scripts/save_map.sh --run nvidia_explore_20260802 --watch 3 &
#   ... explore until "exploration done" ...
#   scripts/save_map.sh --run nvidia_explore_20260802 --label final
#   scripts/save_map.sh --stop-watch
#   scripts/run_demo.sh stop
#   scripts/save_map.sh --run nvidia_explore_20260802 --db
#   scripts/map_coverage.py data/runs/nvidia_explore_20260802/map/final.yaml \
#       --reference data/runs/nvidia_explore_20260801_145700/map/checkpoint_resume002.yaml
#
# Needs system ROS 2 sourced (unlike run_demo.sh, which refuses it) — this
# talks to a running graph rather than launching one.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ROBOT_ID=robot_0; RUN=""; LABEL="final"; WATCH=0; STOP_WATCH=false; DB=false
# Passed explicitly rather than left to map_saver's defaults: unspecified
# thresholds make it warn and emit whatever it feels like, and this repo
# already has saved yamls disagreeing (free_thresh 0.25 vs 0.196) for exactly
# that reason. scripts/map_coverage.py then has to defend against the
# ambiguity, so pin it here instead. 0.65 matches explorer.yaml's
# occupied_thresh and nav2_params.yaml's costmap threshold.
OCC=0.65; FREE=0.25

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-id)  ROBOT_ID="$2"; shift 2;;
    --run)       RUN="$2";      shift 2;;
    --label)     LABEL="$2";    shift 2;;
    --watch)     WATCH="$2";    shift 2;;
    --stop-watch) STOP_WATCH=true; shift;;
    --db)        DB=true;       shift;;
    --occ)       OCC="$2";      shift 2;;
    --free)      FREE="$2";     shift 2;;
    -h|--help)   sed -n '2,30p' "$0"; exit 0;;
    *) echo "[save_map] unknown arg: $1" >&2; exit 2;;
  esac
done

PIDFILE="/tmp/sortbots_map_watch_${ROBOT_ID}.pid"

if [[ "$STOP_WATCH" == true ]]; then
  if [[ -f "$PIDFILE" ]] && kill "$(cat "$PIDFILE")" 2>/dev/null; then
    echo "[save_map] stopped checkpoint loop (pid $(cat "$PIDFILE"))"
  else
    echo "[save_map] no checkpoint loop running"
  fi
  rm -f "$PIDFILE"
  exit 0
fi

if [[ -z "$RUN" ]]; then
  echo "[save_map] --run NAME is required" >&2
  exit 2
fi

if [[ ! "$WATCH" =~ ^[0-9]+$ ]]; then
  echo "[save_map] --watch takes a whole number of minutes, got '$WATCH'" >&2
  exit 2
fi

OUT_DIR="$REPO_ROOT/data/runs/$RUN/map"
mkdir -p "$OUT_DIR"

save_once() {
  local label="$1"
  # /map: the fused, world-anchored grid nodes/map_merge.py publishes — the
  # same topic no matter how many robots are running or which --robot-id was
  # passed (that flag only steers the --db branch below).
  ros2 run nav2_map_server map_saver_cli \
      -t "/map" -f "${OUT_DIR}/${label}" \
      --occ "$OCC" --free "$FREE" --fmt pgm --mode trinary \
      --ros-args -p use_sim_time:=true -p save_map_timeout:=5.0 \
      >"${OUT_DIR}/.save_map.log" 2>&1
  local rc=$?
  if [[ $rc -eq 0 && -f "${OUT_DIR}/${label}.pgm" ]]; then
    echo "[save_map] $(date +%H:%M:%S) wrote ${OUT_DIR}/${label}.{pgm,yaml}"
    return 0
  fi
  echo "[save_map] $(date +%H:%M:%S) FAILED to save '${label}' (see ${OUT_DIR}/.save_map.log)" >&2
  return 1
}

if [[ "$DB" == true ]]; then
  # A live sqlite file copied mid-write gives a torn database, and RTAB-Map
  # writes continuously, so refuse rather than produce something that looks
  # fine until it's resumed.
  # Match the NODE executables, not the bare string "rtabmap", and never
  # match this script or its parent shell. A loose `pgrep -f rtabmap` also
  # matches any command line that merely MENTIONS the database path —
  # including the shell that invoked this script — so the guard fired with
  # nothing actually running and refused a legitimate copy.
  RUNNING=$(pgrep -f "rtabmap_slam|rtabmap_viz|rtabmap_util|rgbd_odometry" \
            | grep -vx -e "$$" -e "$PPID" | wc -l)
  if [ "$RUNNING" -gt 0 ]; then
    echo "[save_map] rtabmap is still running — copying its sqlite DB now would" >&2
    echo "           produce a torn file. Run 'scripts/run_demo.sh stop' first." >&2
    exit 1
  fi
  SRC="$HOME/.ros/sortbots_${ROBOT_ID}.db"
  if [[ ! -f "$SRC" ]]; then
    echo "[save_map] no database at $SRC" >&2
    exit 1
  fi
  # .backup takes sqlite's own consistent snapshot — belt and braces in case
  # the pgrep above was fooled by a process still flushing.
  if command -v sqlite3 >/dev/null 2>&1; then
    sqlite3 "$SRC" ".backup '${OUT_DIR}/rtabmap.db'"
  else
    cp -a "$SRC" "${OUT_DIR}/rtabmap.db"
  fi
  echo "[save_map] copied $SRC -> ${OUT_DIR}/rtabmap.db ($(du -h "${OUT_DIR}/rtabmap.db" | cut -f1))"
  exit 0
fi

if [[ "$WATCH" -eq 0 ]]; then
  save_once "$LABEL"
  exit $?
fi

# Checkpoint loop. It exists because `run_demo.sh stop` pkill -9's rtabmap
# only 2 s after SIGINT, which can take the last minutes of mapping with it —
# and because the checkpoint series doubles as a coverage-versus-time curve
# (feed the .yamls to scripts/map_coverage.py).
#
# Deliberately NOT matched by any of run_demo.sh's PIPELINE_PATTERNS: this
# loop must OUTLIVE teardown so the last checkpoint can land. It ends itself
# after three consecutive failures — /map going away means the pipeline is
# gone — so it can't be orphaned indefinitely either.
echo $$ > "$PIDFILE"
trap 'rm -f "$PIDFILE"; exit 0' INT TERM
echo "[save_map] checkpointing /${ROBOT_ID}/map every ${WATCH} min into ${OUT_DIR}"
n=0; fails=0
while :; do
  n=$((n + 1))
  if save_once "$(printf 'checkpoint_%03d' "$n")"; then
    fails=0
  else
    fails=$((fails + 1))
    if [[ $fails -ge 3 ]]; then
      echo "[save_map] 3 consecutive failures — pipeline is gone, ending checkpoint loop"
      break
    fi
  fi
  sleep $((WATCH * 60))
done
rm -f "$PIDFILE"
