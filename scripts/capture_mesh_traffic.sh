#!/usr/bin/env bash
# Capture inter-robot ROS 2 (DDS/RTPS) traffic for Wireshark.
#
# Where the traffic actually is, and why this script exists: under --mesh each
# robot's ROS 2 stack lives in its own netns, so the inter-robot packets are on
# tap-robot_N *inside* ns-robot_N and are invisible to a capture run in the root
# namespace. Every capture here is therefore wrapped in `ip netns exec`.
#
# Three vantage points, because they answer different questions:
#   --robot N      tap-robot_N  (10.66.0.x)  inter-robot DDS over the 802.11s
#                               mesh — this is "are the two robots talking?"
#   --discovery N  veth${N}     (10.77.N.x)  robot ↔ FastDDS Discovery Server
#                               on :11811 — this is "did they find each other?"
#   --loopback     lo (root ns)              non-mesh runs, where both stacks
#                               share the host loopback
#
# Run: bash scripts/capture_mesh_traffic.sh --robot 0 --seconds 30
#      bash scripts/capture_mesh_traffic.sh --robot 0 --live
#
# See docs/mesh.md, "Watching the robots talk (Wireshark)".
set -euo pipefail

ROBOT=0
MODE="mesh"
SECONDS_ARG=30
OUT=""
LIVE=0
OPEN=0
FILTER="udp"

usage() {
  cat <<EOF
Usage: $0 [options]
  --robot N        capture inter-robot DDS on tap-robot_N (default: 0)
  --discovery N    capture robot_N <-> Discovery Server on veth\${N}
  --loopback       capture on root-namespace lo (non-mesh runs)
  --seconds N      capture duration (default: $SECONDS_ARG); ignored with --live
  --out FILE       output path (default: /tmp/sortbots_rtps_<iface>.pcapng)
  --live           stream straight into the Wireshark GUI instead of a file
  --open           open the finished capture in Wireshark
  --filter EXPR    pcap capture filter (default: "$FILTER")
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot) MODE="mesh"; ROBOT="$2"; shift 2;;
    --discovery) MODE="discovery"; ROBOT="$2"; shift 2;;
    --loopback) MODE="loopback"; shift;;
    --seconds) SECONDS_ARG="$2"; shift 2;;
    --out) OUT="$2"; shift 2;;
    --live) LIVE=1; shift;;
    --open) OPEN=1; shift;;
    --filter) FILTER="$2"; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "ERROR: unknown argument: $1" >&2; usage >&2; exit 1;;
  esac
done

case "$MODE" in
  mesh)      NETNS="ns-robot_${ROBOT}"; IFACE="tap-robot_${ROBOT}";;
  discovery) NETNS="ns-robot_${ROBOT}"; IFACE="veth${ROBOT}";;
  loopback)  NETNS=""; IFACE="lo";;
esac
OUT="${OUT:-/tmp/sortbots_rtps_${IFACE}.pcapng}"

# dumpcap, not tshark: dumpcap is the capture engine and ships in the base
# wireshark install, while tshark is a separate package that is frequently
# absent (2026-08-27 — it was missing on this host). Nothing here needs
# tshark's dissection, only Wireshark does, and that happens at read time.
if ! command -v dumpcap >/dev/null 2>&1; then
  echo "ERROR: dumpcap not found. Install: sudo apt install wireshark" >&2
  exit 1
fi
if [[ $LIVE -eq 1 || $OPEN -eq 1 ]] && ! command -v wireshark >/dev/null 2>&1; then
  echo "ERROR: wireshark not found (needed for --live/--open)." >&2
  echo "       Install: sudo apt install wireshark" >&2
  exit 1
fi

# A missing netns means no mesh run is up. Say so here rather than letting
# `ip netns exec` fail with a bare ENOENT that reads like a script bug.
if [[ -n "$NETNS" ]] && ! sudo ip netns list | grep -qw "$NETNS"; then
  echo "ERROR: namespace $NETNS does not exist — no mesh run is up." >&2
  echo "       Start one first:  scripts/sim_ctl.sh start explore_fleet_mesh" >&2
  echo "       (or use --loopback to capture a non-mesh run)" >&2
  exit 1
fi

NS_PREFIX=()
[[ -n "$NETNS" ]] && NS_PREFIX=(ip netns exec "$NETNS")

if [[ $LIVE -eq 1 ]]; then
  echo "==> live capture on ${NETNS:+$NETNS:}$IFACE -> Wireshark (close Wireshark to stop)"
  # dumpcap writes the stream to stdout; Wireshark reads it as a live source.
  # Wireshark stays unprivileged on this side of the pipe — only the capture
  # half is root, which is the arrangement upstream recommends.
  # `|| true`: closing the Wireshark window breaks the pipe, dumpcap dies on
  # SIGPIPE, and with pipefail+errexit that normal exit would otherwise be
  # reported as a script failure.
  sudo "${NS_PREFIX[@]}" dumpcap -i "$IFACE" -f "$FILTER" -P -w - \
    | wireshark -k -i - 2>/dev/null || true
  exit 0
fi

echo "==> capturing ${SECONDS_ARG}s on ${NETNS:+$NETNS:}$IFACE (filter: $FILTER)"
sudo "${NS_PREFIX[@]}" dumpcap -i "$IFACE" -f "$FILTER" \
  -a "duration:$SECONDS_ARG" -w "$OUT"

# dumpcap ran under sudo, so the file lands root:root 0600 and Wireshark cannot
# open it as you — the failure looks like a corrupt capture rather than a
# permissions problem, which cost real time on 2026-08-27. Hand it back.
sudo chown "$(id -un):$(id -gn)" "$OUT"
chmod 0644 "$OUT"

echo "    wrote $OUT"
if command -v capinfos >/dev/null 2>&1; then
  capinfos -c -u "$OUT" 2>/dev/null | sed 's/^/    /' || true
fi

if [[ $OPEN -eq 1 ]]; then
  echo "==> opening in Wireshark (filter on 'rtps' to see only DDS)"
  wireshark "$OUT" >/dev/null 2>&1 &
else
  echo ""
  echo "    open it with:  wireshark $OUT"
  echo "    then set the display filter to:  rtps"
fi
