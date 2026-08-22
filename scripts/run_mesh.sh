#!/usr/bin/env bash
# Launch the ns-3 802.11s mesh bridge in the background and wait until all
# TapBridge devices are attached. Called by run_demo.sh --mesh.
#
# Env vars consumed:
#   NS3_HOME   (default: ~/ns-3-dev)
#   MESH_LOG   (default: /tmp/sortbots_ns3_mesh.log)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS3_HOME="${NS3_HOME:-$HOME/ns-3-dev}"
MESH_LOG="${MESH_LOG:-/tmp/sortbots_ns3_mesh.log}"

ROBOTS=2
ROBOT_IDS=""
CONFIG="$REPO_ROOT/network/mesh_config.yaml"
PCAP_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robots)    ROBOTS="$2";    shift 2;;
    --robot-ids) ROBOT_IDS="$2"; shift 2;;
    --config)    CONFIG="$2";    shift 2;;
    --pcap-dir)  PCAP_DIR="$2";  shift 2;;
    *) echo "[run_mesh] unknown arg: $1" >&2; exit 1;;
  esac
done

if [[ -z "$ROBOT_IDS" ]]; then
  ROBOT_IDS=$(python3 -c "
import yaml
with open('$REPO_ROOT/configs/robots.yaml') as f:
    roster = yaml.safe_load(f)
ids = [r['id'] for r in roster['robots']][:$ROBOTS]
print(','.join(ids))
")
fi

NS3_BIN="$NS3_HOME/ns3"
if [[ ! -x "$NS3_BIN" ]]; then
  echo "[run_mesh] ERROR: ns3 wrapper not found at $NS3_BIN" >&2
  echo "           Run scripts/install_ns3.sh first." >&2
  exit 1
fi

SCRATCH="$NS3_HOME/scratch/ns3_mesh_bridge.cc"
if [[ ! -f "$SCRATCH" ]]; then
  echo "[run_mesh] ERROR: scratch file missing: $SCRATCH" >&2
  echo "           Re-run scripts/install_ns3.sh to restore the symlink." >&2
  exit 1
fi

PCAP_ARG=""
[[ -n "$PCAP_DIR" ]] && PCAP_ARG="--pcap-dir=$PCAP_DIR"

NS3_ARGS="ns3_mesh_bridge --robots=$ROBOTS --config=$CONFIG $PCAP_ARG"

echo "[run_mesh] launching ns-3 mesh (robots=$ROBOTS, log=$MESH_LOG)"
rm -f "$MESH_LOG"

# setsid so signals to run_demo.sh don't cascade here prematurely
setsid bash -c "cd '$NS3_HOME' && ./ns3 run '$NS3_ARGS'" \
  >"$MESH_LOG" 2>&1 &

# Wait for all TapBridge attachments — one "TapBridge: attached" line per robot
echo "[run_mesh] waiting for TapBridge to attach $ROBOTS tap device(s)..."
ATTACHED=0
for _ in $(seq 1 60); do
  ATTACHED=$(grep -c "TapBridge: attached" "$MESH_LOG" 2>/dev/null || true)
  if [[ "$ATTACHED" -ge "$ROBOTS" ]]; then
    echo "[run_mesh] all $ROBOTS TapBridge device(s) attached."
    exit 0
  fi
  if grep -qiE "ERROR|Traceback|Aborted|Segmentation fault" "$MESH_LOG" 2>/dev/null; then
    echo "[run_mesh] ERROR: ns-3 failed — see $MESH_LOG" >&2
    tail -20 "$MESH_LOG" >&2
    exit 1
  fi
  sleep 2
done

echo "[run_mesh] WARNING: only $ATTACHED/$ROBOTS tap(s) attached after timeout." >&2
echo "           Continuing anyway — check $MESH_LOG" >&2
