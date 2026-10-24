#!/usr/bin/env bash
# zenoh-bridge-ros2dds: the ONLY way ROS traffic leaves a host for the mesh.
#
#   scripts/fleet_bridge.sh install          # fetch the pinned binary (laptop; the Jetson image has it)
#   scripts/fleet_bridge.sh run   BUNDLE     # foreground
#   scripts/fleet_bridge.sh start BUNDLE     # background, log in /tmp/sortbots_fleet_bridge.log
#   scripts/fleet_bridge.sh stop | status
#
# BUNDLE is one node's folder from commsim.hw.deploy (commsim/hw/README.md):
# its bridge.json5 forwards /fleet/status, /fleet/intent and /fleet/tasks and
# nothing else, over the zenoh endpoints and lease the emulation measured.
# fleet.env says which ROS domain the local stack is on and which
# --robot-id this host must run.
#
# Robots: run it INSIDE the Jazzy container (scripts/jetson.sh bridge ...).
# The 1.10.1 binary needs glibc 2.38; the JetPack 6 host has 2.35.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION=1.10.1
# sha256 of the release's standalone zips, checked 2026-10-15. The aarch64 one
# is repeated in docker/jetson/Dockerfile (outside its build context).
declare -A SHA256=(
  [x86_64]=f3ea8ac4345be22716925269bfd3e350e7cb36a8f056e835468c8299341d29b0
  [aarch64]=fdb64d942d4b6beccbe9b1f8a359a01a2fca95a023c27535bf97b233603fad5f
)
INSTALL_DIR="${SORTBOTS_BRIDGE_DIR:-$HOME/.local/share/sortbots/zenoh-bridge-ros2dds-$VERSION}"
LOG=/tmp/sortbots_fleet_bridge.log
PIDFILE=/tmp/sortbots_fleet_bridge.pid

find_bin() {
  local c
  for c in "${SORTBOTS_ZENOH_BRIDGE:-}" /opt/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds \
           "$INSTALL_DIR/zenoh-bridge-ros2dds"; do
    [[ -n "$c" && -x "$c" ]] && { echo "$c"; return 0; }
  done
  echo "ERROR: no zenoh-bridge-ros2dds: scripts/fleet_bridge.sh install" >&2
  return 1
}

check_bundle() {
  local b="$1"
  [[ -f "$b/bridge.json5" && -f "$b/fleet.env" ]] || {
    echo "ERROR: $b has no bridge.json5/fleet.env — regenerate it with commsim.hw.deploy" >&2; exit 1; }
  # The bridge listens on this host's fleet address; without it (netplan not
  # applied, router unplugged) zenoh fails to bind with a less obvious error.
  local ip
  ip=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["listen"]["endpoints"][0].split("/")[1].rsplit(":",1)[0])' "$b/bridge.json5")
  ip route get "$ip" 2>/dev/null | grep -q '^local ' || {
    echo "ERROR: this host has no $ip — run the bundle's apply-host.sh and plug into the router" >&2; exit 1; }
}

run_bridge() {
  local b="$1" bin
  bin=$(find_bin) || exit 1
  check_bundle "$b"
  # The bridge's own Cyclone DDS must sit on loopback. The robot stack runs
  # Fast DDS with discovery LOCALHOST, and the bridge's LOCALHOST mode alone
  # still advertises the default interface (the campus Wi-Fi), which Fast DDS
  # never answers: nothing crossed until this was set (2026-10-15). The
  # bridge appends this to its own CYCLONEDDS_URI.
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
  # Else it assumes 'iron' for the ros_discovery_info format.
  export ROS_DISTRO=jazzy
  echo "[fleet_bridge] $(grep -h '^SORTBOTS_ROBOT_ID=\|^ROS_DOMAIN_ID=' "$b/fleet.env" | tr '\n' ' ')"
  exec "$bin" -c "$b/bridge.json5"
}

running() { [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; }

case "${1:-}" in
  install)
    arch=$(uname -m)
    [[ -n "${SHA256[$arch]:-}" ]] || { echo "ERROR: no pinned build for $arch"; exit 1; }
    zip="zenoh-plugin-ros2dds-$VERSION-$arch-unknown-linux-gnu-standalone.zip"
    tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
    curl -fsSL -o "$tmp/$zip" \
      "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/$VERSION/$zip" || exit 1
    echo "${SHA256[$arch]}  $tmp/$zip" | sha256sum -c --quiet - || { echo "ERROR: checksum mismatch"; exit 1; }
    mkdir -p "$INSTALL_DIR" && unzip -oq "$tmp/$zip" -d "$INSTALL_DIR" || exit 1
    "$INSTALL_DIR/zenoh-bridge-ros2dds" --version 2>/dev/null | tail -1
    ;;
  run)
    [[ -n "${2:-}" ]] || { echo "usage: $0 run BUNDLE_DIR"; exit 2; }
    run_bridge "$2"
    ;;
  start)
    [[ -n "${2:-}" ]] || { echo "usage: $0 start BUNDLE_DIR"; exit 2; }
    running && { echo "[fleet_bridge] already running (pid $(cat "$PIDFILE"))"; exit 0; }
    find_bin >/dev/null || exit 1
    check_bundle "$2"
    setsid "$SCRIPT_DIR/fleet_bridge.sh" run "$2" > "$LOG" 2>&1 < /dev/null &
    echo $! > "$PIDFILE"
    sleep 2
    running || { echo "ERROR: bridge exited:"; tail -20 "$LOG"; exit 1; }
    echo "[fleet_bridge] running (pid $(cat "$PIDFILE"), log $LOG)"
    ;;
  stop)
    running && kill "$(cat "$PIDFILE")"
    rm -f "$PIDFILE"
    echo "[fleet_bridge] stopped"
    ;;
  status)
    if running; then echo "[fleet_bridge] running (pid $(cat "$PIDFILE"))"; else echo "[fleet_bridge] not running"; exit 1; fi
    ;;
  *)
    sed -n 2,7p "$0"; exit 2
    ;;
esac
