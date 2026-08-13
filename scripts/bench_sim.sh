#!/usr/bin/env bash
# Measure Isaac's real-time factor (RTF) with NOTHING else running.
#
# Why this exists: the sim was observed at 0.18-0.25x real time (2026-08-09)
# against the ~0.45x that CLAUDE.md documents, and "the sim is slow" is
# unactionable until you know WHICH half is slow. run_demo.sh starts Isaac AND
# a full RTAB-Map + Nav2 + explorer stack per robot on the same 16 cores, so a
# low RTF there could be Isaac's render loop or it could be the ROS side
# stealing CPU. This runs spawn_warehouse.py ALONE, so the number it prints is
# Isaac's own ceiling — any full-stack run can only be slower than this.
#
# RTF is read from the spawn loop's own "t=<sim seconds>" progress line (every
# 60 steps) versus wall clock, so it needs no ROS, no /clock subscriber, and no
# instrumentation inside the sim.
#
# Baseline measured 2026-08-13, RTX + 16 cores, --scene nvidia, Isaac alone:
#
#   robots  chase  render-every   RTF     phys/s   GPU    what it tells you
#   2       1      1              0.33x   20       100%   what run_demo gives
#   2       0      1              0.47x   28       100%   one render product off
#   2       1      2              0.36x   21        66%   halving RENDER RATE
#   2       1      600            1.20x   72         9%   physics with no pixels
#
# Read those four rows together, because two of them are traps:
#   * Rendering is ~85% of the cost (0.33x -> 1.20x with pixels off), so it is
#     tempting to render less often. Don't: row 3 halved the render rate for a
#     9% gain, because the cost of each render roughly doubles when you do.
#     What scales is the NUMBER OF RENDER PRODUCTS (row 2), not their rate.
#   * The ~0.45x in CLAUDE.md is row 2, not row 1. The chase camera is a third
#     render product on top of the two head cams and costs ~30% of real time
#     all by itself, which is the whole gap between the documented figure and
#     what a fleet run actually does.
# The full stack (Isaac + 2x RTAB-Map + Nav2 + explorers) then runs ~0.25x,
# i.e. the ROS side costs far less than any of the above.
#
#   scripts/bench_sim.sh                                  # current defaults
#   scripts/bench_sim.sh --robots 1 --chase-cam-robots 0  # isolate camera cost
#   scripts/bench_sim.sh --label "render every 2nd tick"  # annotate a run
#
# Must be run from a shell with NO ROS 2 sourced (activate_isaac.sh enforces).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ROBOTS=2
CHASE_CAM=1
RENDER_EVERY=2
SCENE=nvidia
# Isaac needs to load the warehouse, import 2 URDFs and let the RTX pipeline
# settle; the first seconds after the timeline starts are not representative.
WARMUP_S=25
WINDOW_S=45
STARTUP_TIMEOUT_S=400
LABEL=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robots) ROBOTS="$2"; shift 2 ;;
    --chase-cam-robots) CHASE_CAM="$2"; shift 2 ;;
    --render-every) RENDER_EVERY="$2"; shift 2 ;;
    --scene) SCENE="$2"; shift 2 ;;
    --warmup) WARMUP_S="$2"; shift 2 ;;
    --window) WINDOW_S="$2"; shift 2 ;;
    --label) LABEL="$2"; shift 2 ;;
    -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "ERROR: ROS 2 is sourced here; Isaac won't start. Use a clean shell." >&2
  exit 1
fi
# Match on --drive, not on the bare script name: a plain
# `pgrep -f spawn_warehouse.py` also matches any shell or editor tooling that
# merely MENTIONS the file (a py_compile, a grep, this script's own wrapper),
# which false-positived here on 2026-08-13. Only a real sim invocation passes
# --drive. The same pattern guards the pkill below, where a loose match would
# be worse than a false positive.
SIM_PATTERN='spawn_warehouse\.py .*--drive'
if pgrep -f "$SIM_PATTERN" >/dev/null 2>&1; then
  echo "ERROR: a sim is already running — measuring now would time a contended" >&2
  echo "       machine. Run: scripts/sim_ctl.sh stop" >&2
  exit 1
fi

RESULT="$(mktemp -t bench_sim_result.XXXXXX)"
LOG="$(mktemp -t bench_sim_log.XXXXXX)"
trap 'rm -f "$RESULT" "$LOG"' EXIT

echo "[bench] ${LABEL:-baseline}: robots=$ROBOTS chase_cam=$CHASE_CAM" \
     "render_every=$RENDER_EVERY scene=$SCENE"
echo "[bench] starting Isaac alone (no ROS pipeline)..."

ISAAC_SPAWN_WAREHOUSE_RESULT="$RESULT" setsid bash -c "
  source '$REPO_ROOT/scripts/activate_isaac.sh' >/dev/null 2>&1
  exec python '$REPO_ROOT/scripts/spawn_warehouse.py' --headless --forever \
       --robots $ROBOTS --scene $SCENE --drive cmd_vel \
       --chase-cam-robots $CHASE_CAM --render-every $RENDER_EVERY" >"$LOG" 2>&1 &
SESSION_PID=$!

cleanup() {
  kill -- -"$SESSION_PID" 2>/dev/null
  sleep 3
  pkill -9 -f "$SIM_PATTERN" 2>/dev/null
  rm -f "$RESULT" "$LOG"
}
trap cleanup EXIT INT TERM

# The sim is "up" once the loop emits its first t= line (timeline playing).
deadline=$((SECONDS + STARTUP_TIMEOUT_S))
until grep -qE '^\s*t=' "$RESULT" 2>/dev/null; do
  if ! kill -0 "$SESSION_PID" 2>/dev/null; then
    echo "[bench] Isaac died during startup. Tail of its log:" >&2
    tail -25 "$LOG" >&2
    exit 1
  fi
  if (( SECONDS > deadline )); then
    echo "[bench] timed out after ${STARTUP_TIMEOUT_S}s waiting for the step loop." >&2
    tail -25 "$LOG" >&2
    exit 1
  fi
  sleep 2
done
echo "[bench] stepping; warming up ${WARMUP_S}s..."
sleep "$WARMUP_S"

ISAAC_PID="$(pgrep -f "$SIM_PATTERN" | head -1)"

# Sample sim time, wall time and the process' own CPU jiffies together, so the
# CPU number covers exactly the window the RTF is computed over.
sample() {
  local t_sim wall cpu
  t_sim="$(grep -oE '^\s*t=\s*[0-9.]+' "$RESULT" | tail -1 | grep -oE '[0-9.]+')"
  wall="$(date +%s.%N)"
  cpu="$(awk '{print $14 + $15}' "/proc/$ISAAC_PID/stat" 2>/dev/null || echo 0)"
  echo "$t_sim $wall $cpu"
}

read -r t1 w1 c1 <<<"$(sample)"
echo "[bench] measuring over ${WINDOW_S}s..."
GPU_SAMPLES="$(mktemp)"
( for _ in $(seq 1 $((WINDOW_S / 5))); do
    nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits 2>/dev/null
    sleep 5
  done > "$GPU_SAMPLES" ) &
sleep "$WINDOW_S"
read -r t2 w2 c2 <<<"$(sample)"

/usr/bin/python3 - "$t1" "$w1" "$c1" "$t2" "$w2" "$c2" "$GPU_SAMPLES" <<'PY'
import sys, os
t1, w1, c1, t2, w2, c2, gpu_file = sys.argv[1:]
sim = float(t2) - float(t1)
wall = float(w2) - float(w1)
# utime+stime are in clock ticks; 100/s on every Linux we care about.
cpu_s = (float(c2) - float(c1)) / os.sysconf("SC_CLK_TCK")
rtf = sim / wall if wall else 0.0
print()
print(f"  sim advanced   {sim:6.1f} s")
print(f"  wall elapsed   {wall:6.1f} s")
print(f"  real-time factor  {rtf:5.2f}x")
print(f"  physics rate      {sim * 60 / wall:5.1f} steps/s  (target 60)")
print(f"  isaac CPU         {cpu_s / wall * 100:5.0f}%  of one core"
      f"  ({cpu_s / wall:.1f} cores busy)")
try:
    vals = [int(x) for x in open(gpu_file).read().split() if x.strip().isdigit()]
    if vals:
        print(f"  GPU util          {sum(vals) / len(vals):5.0f}%  (n={len(vals)})")
except OSError:
    pass
print()
PY
rm -f "$GPU_SAMPLES"
echo "[bench] done; stopping Isaac."
