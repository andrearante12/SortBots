#!/usr/bin/env bash
# One entry point for driving the SortBots sim without reading any launch file.
#
# This is the interface to use from scripts, CI, and coding agents. Everything
# it does is also doable by hand (see docs/running.md), but this wraps the two
# things that are easy to get wrong: the console has to be started detached and
# from a ROS-free shell, and "is the sim up yet" is a phase you have to poll.
#
# Usage:
#   scripts/sim_ctl.sh console start        # bring the dashboard up (detached)
#   scripts/sim_ctl.sh console stop         # dashboard + any running sim
#   scripts/sim_ctl.sh list                 # scenarios and their status
#   scripts/sim_ctl.sh dry-run NAME [k=v...]  # print the run_demo.sh command; launches nothing
#   scripts/sim_ctl.sh start NAME [k=v...]  # start a scenario (returns immediately)
#   scripts/sim_ctl.sh wait [PHASE] [--timeout S]   # block until PHASE (default: running)
#   scripts/sim_ctl.sh status               # one line: state, phase, scenario, elapsed
#   scripts/sim_ctl.sh log [--lines N]      # tail the current session's log
#   scripts/sim_ctl.sh stop                 # tear the sim down, keep the console
#   scripts/sim_ctl.sh stop --save-map NAME # ...saving the map into maps/ first
#
# Saved maps (the library at maps/, see maps/README.md) are managed by
# scripts/maps.sh; `stop --save-map NAME` is the one-gesture wrapper, since a
# complete entry needs the grid captured while the stack is up and the pose
# graph copied once it's down. Load one back with:
#   scripts/sim_ctl.sh start library_localize map=$PWD/maps/NAME/map.db
#
# Typical unattended run:
#   scripts/sim_ctl.sh console start
#   scripts/sim_ctl.sh start explore_fresh headless=true
#   scripts/sim_ctl.sh wait running --timeout 420   # first run streams NVIDIA assets
#   ...
#   scripts/sim_ctl.sh stop
#
# EXIT CODES (stable — branch on these, not on the text):
#   0   success / requested state reached
#   1   command-level failure (bad scenario, start rejected, run went to failed)
#   3   the dashboard console is not running (run: sim_ctl.sh console start)
#   4   no session exists — from `status` and `wait`. Not an error condition:
#          `status` returning 4 is exactly how you ask "is anything running?"
#   124 timed out waiting
# `stop` and `log` are idempotent: with no session they do nothing and exit 0.
# `list` and `dry-run` never touch the console and work with nothing running.
#
# Run from a CLEAN shell: no ROS sourced, no Isaac venv, no conda env active.
# `console start` refuses otherwise, for the same reason run_demo.sh does.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PORT="${SORTBOTS_PORT:-8081}"
API="http://127.0.0.1:${PORT}/api"
CONSOLE_STDOUT="/tmp/sortbots_console_stdout.log"

# $1 message, $2 exit code — note "$1" not "$*", or the code lands in the text.
die() { echo "[sim_ctl] $1" >&2; exit "${2:-1}"; }

# Every control endpoint answers 503 when serve.py is running without
# --control (i.e. it belongs to a bringup, not to a console), and curl fails
# outright when nothing is listening. Both mean "no console" to a caller.
api_get() {
  local body code
  body="$(curl -s -w '\n%{http_code}' "${API}/$1" 2>/dev/null)" || return 3
  code="${body##*$'\n'}"
  body="${body%$'\n'*}"
  [[ "$code" == "503" ]] && return 3
  [[ "$code" == "200" ]] || { echo "$body" >&2; return 1; }
  printf '%s' "$body"
}

api_post() {
  local body code
  body="$(curl -s -w '\n%{http_code}' -X POST -H 'Content-Type: application/json' \
          -d "${2:-{\}}" "${API}/$1" 2>/dev/null)" || return 3
  code="${body##*$'\n'}"
  body="${body%$'\n'*}"
  [[ "$code" == "503" ]] && return 3
  [[ "$code" == "200" ]] || { echo "$body" | jq -r '.error // .' >&2; return 1; }
  printf '%s' "$body"
}

require_console() {
  curl -sf -o /dev/null "${API}/session" 2>/dev/null || \
    die "the dashboard console is not running — start it with: scripts/sim_ctl.sh console start" 3
}

# k=v pairs -> a JSON object, built by python3 rather than string-concatenated
# so a value with a quote in it can't break out of the payload.
overrides_json() {
  python3 -c '
import json, sys
out = {}
for item in sys.argv[1:]:
    key, _, value = item.partition("=")
    if not _:
        raise SystemExit(f"override must be key=value, got: {item}")
    low = value.lower()
    out[key] = True if low == "true" else False if low == "false" else (
        int(value) if value.lstrip("-").isdigit() else value)
print(json.dumps(out))' "$@"
}

cmd_console() {
  case "${1:-}" in
    start)
      if curl -sf -o /dev/null "${API}/session" 2>/dev/null; then
        echo "[sim_ctl] console already up: http://localhost:${PORT}/"
        return 0
      fi
      [[ -n "${AMENT_PREFIX_PATH:-}" ]] && \
        die "this shell has ROS 2 sourced (AMENT_PREFIX_PATH set) — use a fresh terminal" 1
      # Detached: run_console.sh is foreground by design (Ctrl-C stops it), which
      # is wrong for an unattended caller. setsid+nohup gives it its own session
      # so it survives the shell that launched it.
      setsid nohup bash "$REPO_ROOT/scripts/run_console.sh" --port "$PORT" \
        >"$CONSOLE_STDOUT" 2>&1 &
      for _ in $(seq 1 60); do
        curl -sf -o /dev/null "${API}/session" 2>/dev/null && {
          echo "[sim_ctl] console up: http://localhost:${PORT}/  (log: $CONSOLE_STDOUT)"
          return 0
        }
        sleep 1
      done
      die "console did not come up within 60s — see $CONSOLE_STDOUT" 1
      ;;
    stop)
      bash "$REPO_ROOT/scripts/run_console.sh" stop
      ;;
    *) die "usage: sim_ctl.sh console {start|stop}" 1;;
  esac
}

cmd_list() {
  # Reads configs/scenarios/ directly — deliberately works with no console up.
  python3 "$REPO_ROOT/webui/session.py" --list
}

cmd_dry_run() {
  local name="${1:-}"; shift || true
  [[ -n "$name" ]] || die "usage: sim_ctl.sh dry-run NAME [k=v...]" 1
  local args=()
  for kv in "$@"; do args+=(--set "$kv"); done
  python3 "$REPO_ROOT/webui/session.py" --print-argv "$name" "${args[@]}"
}

cmd_start() {
  local name="${1:-}"; shift || true
  [[ -n "$name" ]] || die "usage: sim_ctl.sh start NAME [k=v...]" 1
  require_console
  local ov; ov="$(overrides_json "$@")" || die "bad override" 1
  local payload; payload="$(python3 -c '
import json, sys
print(json.dumps({"scenario": sys.argv[1], "overrides": json.loads(sys.argv[2])}))' \
    "$name" "$ov")"
  local out; out="$(api_post session/start "$payload")" || exit $?
  echo "$out" | jq -r '"[sim_ctl] started \(.scenario) (session \(.session_id))"'
}

cmd_status() {
  local out rc
  out="$(api_get session)"; rc=$?
  [[ $rc -eq 3 ]] && die "console not running" 3
  [[ $rc -ne 0 ]] && die "could not read session status" 1
  echo "$out" | jq -r '
    if .state == "idle" then "state=idle  (no session)"
    else "state=\(.state) phase=\(.phase) scenario=\(.scenario) elapsed=\(.elapsed_s // 0)s"
         + (if .error then " error=\(.error)" else "" end)
    end'
  [[ "$(echo "$out" | jq -r .state)" == "idle" ]] && return 4
  return 0
}

cmd_log() {
  local lines=40
  [[ "${1:-}" == "--lines" ]] && lines="${2:-40}"
  require_console
  local out; out="$(api_get 'session/log?offset=-1')" || exit $?
  echo "$out" | jq -r .text | tail -n "$lines"
}

cmd_wait() {
  local target="running" timeout=600
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --timeout) timeout="$2"; shift 2;;
      *) target="$1"; shift;;
    esac
  done
  require_console
  local deadline=$(( SECONDS + timeout )) last=""
  while [[ $SECONDS -lt $deadline ]]; do
    local out; out="$(api_get session)" || exit $?
    local state phase
    state="$(echo "$out" | jq -r .state)"
    phase="$(echo "$out" | jq -r .phase)"
    if [[ "$state:$phase" != "$last" ]]; then
      echo "[sim_ctl] $state / $phase"
      last="$state:$phase"
    fi
    case "$state" in
      failed|exited) echo "[sim_ctl] run failed — last 30 log lines:" >&2
                     cmd_log --lines 30 >&2; exit 1;;
      idle)          die "no session is running" 4;;
    esac
    # `running` is both a phase and the terminal healthy state.
    [[ "$phase" == "$target" || "$state" == "$target" ]] && { echo "[sim_ctl] reached $target"; return 0; }
    sleep 3
  done
  die "timed out after ${timeout}s waiting for $target (last: $last)" 124
}

# scripts/maps.sh needs system ROS 2 sourced — the opposite of everything else
# here, which must stay ROS-free for run_demo.sh's sake. Give it its own shell
# rather than contaminating ours, and prepend the system PATH so `ros2` and
# map_saver_cli don't pick up conda's python3.
run_maps_sh() {
  bash -c "source /opt/ros/jazzy/setup.bash
           export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:\$PATH
           exec '$REPO_ROOT/scripts/maps.sh' \"\$@\"" _ "$@"
}

cmd_stop() {
  local save_map=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --save-map) save_map="${2:-}"; shift 2;;
      *) die "stop: unknown arg: $1" 1;;
    esac
  done
  require_console

  # Two saves, either side of teardown, because the two artifacts have
  # opposite requirements: the occupancy grid comes off the live /map (stack
  # UP), and a safe copy of the sqlite pose graph needs the file closed (stack
  # DOWN, or via the backup service the first call tries). maps.sh is
  # idempotent, so the first call captures what it can — exit 5 meaning
  # "pending" is expected here, not a failure — and the second completes it.
  if [[ -n "$save_map" ]]; then
    echo "[sim_ctl] saving map '$save_map' (grid, stack still up)"
    run_maps_sh save "$save_map" || true
  fi

  api_post session/stop >/dev/null || exit $?
  # stop is asynchronous (run_demo.sh's teardown sleeps between sweeps).
  local stopped=false
  for _ in $(seq 1 60); do
    local state; state="$(api_get session | jq -r .state)"
    if [[ "$state" == "stopped" || "$state" == "idle" ]]; then
      echo "[sim_ctl] stopped"; stopped=true; break
    fi
    sleep 2
  done
  [[ "$stopped" == true ]] || die "stop did not complete within 120s" 1

  if [[ -n "$save_map" ]]; then
    echo "[sim_ctl] completing map '$save_map' (pose graph, stack down)"
    run_maps_sh save "$save_map" --force || \
      die "map '$save_map' did not complete — scripts/maps.sh show $save_map" 1
  fi
  return 0
}

case "${1:-}" in
  console)  shift; cmd_console "$@";;
  list)     shift; cmd_list "$@";;
  dry-run)  shift; cmd_dry_run "$@";;
  start)    shift; cmd_start "$@";;
  status)   shift; cmd_status "$@";;
  log)      shift; cmd_log "$@";;
  wait)     shift; cmd_wait "$@";;
  stop)     shift; cmd_stop "$@";;
  -h|--help|"") sed -n '2,45p' "$0" | sed 's/^# \{0,1\}//'; exit 0;;
  *) die "unknown command: $1  (try --help)" 1;;
esac
