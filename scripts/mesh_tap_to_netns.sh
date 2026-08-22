#!/usr/bin/env bash
# Phase B of mesh setup: move tap devices from root namespace into their robot
# network namespaces and assign mesh IPs. Must be called AFTER run_mesh.sh
# has confirmed TapBridge is attached (ns-3 must already hold the tap FD open).
#
# The Linux kernel's TUN/TAP driver keeps the open file descriptor valid when
# the underlying net_device is moved to another namespace, so ns-3 continues
# to read/write frames after the move.
#
# Must run as root. Idempotent.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROBOTS=2
ROBOT_IDS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robots)    ROBOTS="$2";    shift 2;;
    --robot-ids) ROBOT_IDS="$2"; shift 2;;
    *) echo "[tap_to_netns] unknown arg: $1" >&2; exit 1;;
  esac
done

if [[ $EUID -ne 0 ]]; then
  echo "[tap_to_netns] ERROR: must run as root" >&2
  exit 1
fi

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

i=0
for rid in "${IDS[@]}"; do
  TAP="tap-${rid}"
  NETNS="ns-${rid}"
  MESH_IP="10.66.0.$((i + 1))"
  MTU=1400

  if ip netns exec "$NETNS" ip link show "$TAP" >/dev/null 2>&1; then
    echo "[tap_to_netns] $TAP already in $NETNS ($MESH_IP) — skipping"
  else
    ip link set "$TAP" netns "$NETNS"
    ip netns exec "$NETNS" ip addr add "${MESH_IP}/24" dev "$TAP"
    ip netns exec "$NETNS" ip link set "$TAP" mtu $MTU
    ip netns exec "$NETNS" ip link set "$TAP" promisc on
    ip netns exec "$NETNS" ip link set "$TAP" up
    echo "[tap_to_netns] moved $TAP → $NETNS ($MESH_IP/24)"
  fi

  i=$((i + 1))
done

echo "[tap_to_netns] done."
