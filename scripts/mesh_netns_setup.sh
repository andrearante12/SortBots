#!/usr/bin/env bash
# Create tap devices in the ROOT namespace for ns-3 TapBridge, and create
# per-robot network namespaces. Idempotent. Must run as root.
#
# Two-phase design:
#   Phase A (this script): taps are created in root ns WITHOUT an IP so that
#     ns-3 can open them with TapBridge::UseLocal. ns-3 must start while the
#     taps are still in root ns (tap-creator runs in root ns).
#   Phase B (scripts/mesh_tap_to_netns.sh): called AFTER ns-3 attaches the
#     TapBridges. Moves each tap into its robot netns and assigns the mesh IP.
#     The open TapBridge FD remains valid after the move because the kernel
#     TUN/TAP driver references the net_device object directly, not by namespace.
#
# After Phase B:
#   ns-robot_0: tap-robot_0  10.66.0.1/24  (robot_0's DDS mesh interface)
#   ns-robot_1: tap-robot_1  10.66.0.2/24  (robot_1's DDS mesh interface)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROBOTS=2
ROBOT_IDS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robots)    ROBOTS="$2";    shift 2;;
    --robot-ids) ROBOT_IDS="$2"; shift 2;;
    *) echo "[mesh_setup] unknown arg: $1" >&2; exit 1;;
  esac
done

if [[ $EUID -ne 0 ]]; then
  echo "[mesh_setup] ERROR: must run as root (sudo $0 $*)" >&2
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

# ── Preflight: tap-creator must be setuid root ────────────────────────────
_USER_HOME="$(getent passwd "${SUDO_USER:-$USER}" | cut -d: -f6)"
NS3_HOME="${NS3_HOME:-${_USER_HOME}/ns-3-dev}"
TAP_CREATOR="$NS3_HOME/build/src/tap-bridge/ns3-dev-tap-creator-default"
if [[ ! -f "$TAP_CREATOR" ]]; then
  echo "[mesh_setup] ERROR: tap-creator not found: $TAP_CREATOR" >&2
  echo "             Run scripts/install_ns3.sh first." >&2
  exit 1
fi
TAP_CREATOR_PERMS="$(stat -c '%a' "$TAP_CREATOR")"
TAP_CREATOR_OWNER="$(stat -c '%U' "$TAP_CREATOR")"
if [[ "$TAP_CREATOR_OWNER" != "root" || "$TAP_CREATOR_PERMS" != "4755" ]]; then
  echo "[mesh_setup] ERROR: tap-creator is not setuid root." >&2
  echo "             Fix with:  sudo chown root:root '$TAP_CREATOR'" >&2
  echo "                        sudo chmod 4755 '$TAP_CREATOR'" >&2
  exit 1
fi

IFS=',' read -ra IDS <<< "$ROBOT_IDS"

for rid in "${IDS[@]}"; do
  TAP="tap-${rid}"
  NETNS="ns-${rid}"
  MTU=1400

  # Create tap in ROOT namespace, NO IP — ns-3 opens it here.
  # IP is added after ns-3 attaches (Phase B: mesh_tap_to_netns.sh).
  if ip link show "$TAP" >/dev/null 2>&1; then
    echo "[mesh_setup] tap $TAP already exists in root ns — skipping"
  elif ip netns exec "$NETNS" ip link show "$TAP" >/dev/null 2>&1; then
    echo "[mesh_setup] tap $TAP already exists in netns $NETNS — skipping"
  else
    ip tuntap add mode tap "$TAP"
    ip link set "$TAP" mtu $MTU
    ip link set "$TAP" promisc on
    ip link set "$TAP" up
    echo "[mesh_setup] created tap $TAP (no IP, mtu $MTU) in root namespace"
  fi

  # Create network namespace
  if ip netns list 2>/dev/null | grep -q "^${NETNS}\b"; then
    echo "[mesh_setup] netns $NETNS already exists — skipping"
  else
    ip netns add "$NETNS"
    ip netns exec "$NETNS" ip link set lo up
    echo "[mesh_setup] created netns $NETNS"
  fi
done

echo "[mesh_setup] done. ${#IDS[@]} tap(s) + netns ready (Phase A). Run mesh_tap_to_netns.sh after ns-3 attaches."
