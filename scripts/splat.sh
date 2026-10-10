#!/usr/bin/env bash
# Gaussian-splat reconstruction of a saved map, on the WORKSTATION's GPU.
#
# The splat worker (splat/worker.py) turns maps/<name>/ — the RTAB-Map pose
# graph already holds every keyframe and its optimized pose — into a splat the
# dashboard's 3D panel shows in "splat" mode. Same pipeline for sim and real;
# only where the pose graph comes from differs. Never run this on the Jetson.
# See docs/splat.md.
#
# Usage:
#   scripts/splat.sh serve                 # start the worker, detached (:8082)
#   scripts/splat.sh stop                  # stop it (a running job is lost)
#   scripts/splat.sh status                # worker health + recent jobs
#   scripts/splat.sh build NAME [--from URL] [--iters N] [--force] [--wait]
#       sim:   scripts/splat.sh build warehouse --wait
#       robot: scripts/splat.sh build warehouse --from http://jetson-orin.tail0d28f9.ts.net:8081
#   scripts/splat.sh log JOB_ID            # a job's log
#
# One-time setup: scripts/install_splat.sh (creates the `splat` conda env).
#
# EXIT CODES (stable):
#   0   success (build --wait: job done)
#   1   failure (job failed/cancelled, request refused)
#   2   usage error
#   3   the worker is not running (run: scripts/splat.sh serve)
#   124 build --wait timed out
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PORT="${SORTBOTS_SPLAT_PORT:-8082}"
API="http://127.0.0.1:${PORT}/api"
LOG="/tmp/sortbots_splat_worker.log"
# The `[s]` keeps pgrep/pkill from matching a shell whose own command line
# merely mentions the worker (e.g. `bash -c '... splat/worker.py ...'`).
WORKER_PATTERN="[s]plat/worker.py"
# System python3 for the small JSON helpers below: conda's leads PATH.
PY=/usr/bin/python3

die() { echo "[splat] $1" >&2; exit "${2:-1}"; }

json_field() { "$PY" -c "import json,sys; d=json.load(sys.stdin); print(d$1)"; }

up() { curl -s -m 2 "${API}/health" >/dev/null 2>&1; }

cmd_serve() {
  up && { echo "[splat] worker already up on :${PORT}"; return 0; }
  # Same guard as install_splat.sh: a sourced Jazzy's PYTHONPATH shadows the
  # env's numpy; the worker sources ROS itself, only for rtabmap-export.
  [[ -n "${AMENT_PREFIX_PATH:-}" ]] && die "ROS 2 is sourced in this shell; use a clean one" 1
  local conda_base py
  conda_base="$(conda info --base 2>/dev/null || echo "$HOME/miniconda3")"
  py="$conda_base/envs/splat/bin/python"
  [[ -x "$py" ]] || die "no \`splat\` conda env — run scripts/install_splat.sh" 1
  cd "$REPO_ROOT" || exit 1
  setsid "$py" splat/worker.py --port "$PORT" >"$LOG" 2>&1 </dev/null &
  for _ in $(seq 1 40); do
    up && { echo "[splat] worker up: http://$(hostname):${PORT}/ (log: $LOG)"; return 0; }
    sleep 0.25
  done
  tail -5 "$LOG" >&2
  die "worker did not come up" 1
}

cmd_stop() {
  pkill -f "$WORKER_PATTERN" && echo "[splat] worker stopped" || echo "[splat] worker not running"
}

cmd_status() {
  up || die "worker not running (scripts/splat.sh serve)" 3
  curl -s "${API}/health"; echo
  curl -s "${API}/jobs" | "$PY" -c "$(cat <<'EOF'
import json, sys
for j in json.load(sys.stdin)["jobs"][:8]:
    p, r = j.get("progress") or {}, j.get("result") or {}
    extra = ""
    if p:
        extra = " %s/%s %s dB" % (p.get("step"), p.get("iters"), p.get("psnr"))
    elif j["state"] == "done":
        extra = " %s gaussians, %s dB" % (r.get("gaussians"), r.get("psnr"))
    if j.get("error"):
        extra += "  (%s)" % j["error"]
    print("  %-36s %-16s%s" % (j["id"], j["state"], extra))
EOF
)"
}


cmd_build() {
  local name="" from="local" iters="" force=false wait=false timeout=3600
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --from)    from="${2:-}"; shift 2;;
      --iters)   iters="${2:-}"; shift 2;;
      --force)   force=true; shift;;
      --wait)    wait=true; shift;;
      --timeout) timeout="${2:-}"; shift 2;;
      -*)        die "unknown option $1" 2;;
      *)         [[ -z "$name" ]] && name="$1" || die "extra argument $1" 2; shift;;
    esac
  done
  [[ -n "$name" ]] || die "usage: splat.sh build NAME [--from URL] [--iters N] [--force] [--wait]" 2
  up || die "worker not running (scripts/splat.sh serve)" 3
  local body resp code id
  body="$("$PY" -c 'import json,sys
d = {"map": sys.argv[1], "source": sys.argv[2], "force": sys.argv[4] == "true"}
if sys.argv[3]: d["iters"] = int(sys.argv[3])
print(json.dumps(d))' "$name" "$from" "$iters" "$force")" || die "bad --iters" 2
  resp="$(curl -s -w '\n%{http_code}' -X POST -H 'Content-Type: application/json' -d "$body" "${API}/jobs")"
  code="${resp##*$'\n'}"; resp="${resp%$'\n'*}"
  [[ "$code" == "201" ]] || die "refused: $(echo "$resp" | json_field '["error"]' 2>/dev/null || echo "$resp")" 1
  id="$(echo "$resp" | json_field '["id"]')"
  echo "[splat] queued $id"
  $wait || return 0
  local deadline=$((SECONDS + timeout)) state="" last=""
  while (( SECONDS < deadline )); do
    resp="$(curl -s "${API}/jobs/${id}")"
    state="$(echo "$resp" | json_field '["state"]')"
    local line="$state$(echo "$resp" | json_field '.get("progress") and " %s/%s" % (d["progress"]["step"], d["progress"]["iters"]) or ""')"
    [[ "$line" != "$last" ]] && { echo "[splat] $id: $line"; last="$line"; }
    case "$state" in
      done) echo "[splat] done: http://$(hostname):${PORT}/splats/${name}/${id}.splat"; return 0;;
      failed|cancelled) echo "$resp" | json_field '["error"]' >&2; return 1;;
    esac
    sleep 5
  done
  die "timed out waiting for $id (still $state)" 124
}

cmd_log() {
  [[ -n "${1:-}" ]] || die "usage: splat.sh log JOB_ID" 2
  up || die "worker not running (scripts/splat.sh serve)" 3
  curl -s "${API}/jobs/$1/log?offset=0" | json_field '["text"]'
}

case "${1:-}" in
  serve)  shift; cmd_serve "$@";;
  stop)   shift; cmd_stop "$@";;
  status) shift; cmd_status "$@";;
  build)  shift; cmd_build "$@";;
  log)    shift; cmd_log "$@";;
  *)      sed -n '2,27p' "$0"; exit 2;;
esac
