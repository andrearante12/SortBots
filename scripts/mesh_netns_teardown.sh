#!/usr/bin/env bash
# Reverse mesh_netns_setup.sh + mesh_tap_to_netns.sh. Idempotent.
# Must run as root. Kill ns-3 before running this.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROBOTS=2
ROBOT_IDS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robots)    ROBOTS="$2";    shift 2;;
    --robot-ids) ROBOT_IDS="$2"; shift 2;;
    *) echo "[mesh_teardown] unknown arg: $1" >&2; exit 1;;
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

IFS=',' read -ra IDS <<< "$ROBOT_IDS"

for rid in "${IDS[@]}"; do
  TAP="tap-${rid}"
  NETNS="ns-${rid}"

  # Delete netns — this moves its interfaces (including the tap) back to root ns.
  if ip netns list 2>/dev/null | grep -q "^${NETNS}\b"; then
    ip netns del "$NETNS" 2>/dev/null || true
    echo "[mesh_teardown] deleted netns $NETNS"
  fi

  # Delete tap whether it's still in root ns (never moved) or was moved back.
  if ip link show "$TAP" >/dev/null 2>&1; then
    ip link del "$TAP" 2>/dev/null || true
    echo "[mesh_teardown] removed $TAP"
  fi
done

echo "[mesh_teardown] done."
