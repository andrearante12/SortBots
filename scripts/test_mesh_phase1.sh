#!/usr/bin/env bash
# Phase 1 mesh test: ns-3 802.11s mesh with tap-to-netns isolation.
# Run as your normal user: bash scripts/test_mesh_phase1.sh
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS3_HOME="${NS3_HOME:-$HOME/ns-3-dev}"
PCAP_DIR="/tmp/sortbots_mesh_test"
MESH_LOG="/tmp/sortbots_ns3_mesh.log"
CAP0="/tmp/sm_cap_r0.pcap"
CAP1="/tmp/sm_cap_r1.pcap"
TCPDUMP_PIDS=()

cleanup() {
  echo "--- cleanup ---"
  for p in "${TCPDUMP_PIDS[@]:-}"; do sudo kill "$p" 2>/dev/null || true; done
  pkill -f ns3_mesh_bridge 2>/dev/null || true
  sleep 1
  sudo rm -f "$CAP0" "$CAP1" 2>/dev/null || true
  sudo "$REPO_ROOT/scripts/mesh_netns_teardown.sh" --robots 2 2>/dev/null || true
}
trap cleanup EXIT

echo "==> 1. tap + netns setup (Phase A)"
sudo "$REPO_ROOT/scripts/mesh_netns_setup.sh" --robots 2

echo ""
echo "==> 2. launch ns-3 (TapBridge opens taps while still in root ns)"
export NS3_HOME MESH_LOG
mkdir -p "$PCAP_DIR"
"$REPO_ROOT/scripts/run_mesh.sh" --robots 2 \
  --config "$REPO_ROOT/network/mesh_config.yaml" \
  --pcap-dir "$PCAP_DIR"

echo ""
echo "==> 3. move taps into robot namespaces (Phase B — FD stays valid)"
sudo "$REPO_ROOT/scripts/mesh_tap_to_netns.sh" --robots 2

echo ""
echo "==> 3b. align tap MACs to ns-3 mesh node MACs"
# Frames arriving in ns-robot_N have dst_mac=M_N (the HWMP mesh MAC).  Linux
# marks frames with dst_mac != interface_mac as PACKET_OTHERHOST, which the IP
# stack drops even in promisc mode.  We change each tap's MAC to its ns-3 mesh
# MAC so the kernel sees them as PACKET_HOST and the IP stack processes them.
# The taps go DOWN briefly; ns-3's FD survives because tun_net_close() does not
# generate EPOLLHUP — select() in the ReadThread just blocks until UP returns.
MESH_MAC_0=$(grep "node-mac tap-robot_0" "$MESH_LOG" 2>/dev/null | awk '{print $4}')
MESH_MAC_1=$(grep "node-mac tap-robot_1" "$MESH_LOG" 2>/dev/null | awk '{print $4}')
if [[ -z "$MESH_MAC_0" || -z "$MESH_MAC_1" ]]; then
  echo "ERROR: could not read ns-3 mesh node MACs from $MESH_LOG" >&2
  exit 1
fi
sudo ip netns exec ns-robot_0 ip link set tap-robot_0 down
sudo ip netns exec ns-robot_0 ip link set tap-robot_0 address "$MESH_MAC_0"
sudo ip netns exec ns-robot_0 ip link set tap-robot_0 up
sudo ip netns exec ns-robot_1 ip link set tap-robot_1 down
sudo ip netns exec ns-robot_1 ip link set tap-robot_1 address "$MESH_MAC_1"
sudo ip netns exec ns-robot_1 ip link set tap-robot_1 up
echo "    tap-robot_0 MAC -> $MESH_MAC_0"
echo "    tap-robot_1 MAC -> $MESH_MAC_1"

echo ""
echo "==> 4. waiting 15s for HWMP mesh routes to converge..."
sleep 15

echo ""
echo "==> 5. verify interfaces inside namespaces"
sudo ip netns exec ns-robot_0 ip -4 addr show tap-robot_0
sudo ip netns exec ns-robot_1 ip -4 addr show tap-robot_1

echo ""
echo "==> 6. add static ARP on both sides (bypass ARP timing; test pure ICMP path)"
# tap MACs now equal ns-3 mesh MACs so using tap MAC for ARP is correct.
TAP0_MAC=$(sudo ip netns exec ns-robot_0 cat /sys/class/net/tap-robot_0/address)
TAP1_MAC=$(sudo ip netns exec ns-robot_1 cat /sys/class/net/tap-robot_1/address)
sudo ip netns exec ns-robot_0 ip neigh replace 10.66.0.2 lladdr "$TAP1_MAC" dev tap-robot_0
sudo ip netns exec ns-robot_1 ip neigh replace 10.66.0.1 lladdr "$TAP0_MAC" dev tap-robot_1
echo "    robot_0 ARP: 10.66.0.2 -> $TAP1_MAC"
echo "    robot_1 ARP: 10.66.0.1 -> $TAP0_MAC"

echo ""
echo "==> 7. start tcpdump in each netns (trace where frames go)"
sudo ip netns exec ns-robot_0 tcpdump -ni tap-robot_0 -w "$CAP0" &
TCPDUMP_PIDS+=($!)
sudo ip netns exec ns-robot_1 tcpdump -ni tap-robot_1 -w "$CAP1" &
TCPDUMP_PIDS+=($!)
sleep 0.5

echo ""
echo "==> 8. ping from ns-robot_0 (10.66.0.1) to 10.66.0.2"
sudo ip netns exec ns-robot_0 ping -c 4 -W 5 10.66.0.2 && PING_OK=1 || PING_OK=0

sleep 1
for p in "${TCPDUMP_PIDS[@]}"; do sudo kill "$p" 2>/dev/null || true; done
TCPDUMP_PIDS=()
sleep 0.5

echo ""
echo "==> 9. frame trace"
echo "--- tap-robot_0 in ns-robot_0 ---"
sudo tcpdump -r "$CAP0" 2>/dev/null | head -20 || echo "(no capture)"
echo "--- tap-robot_1 in ns-robot_1 ---"
sudo tcpdump -r "$CAP1" 2>/dev/null | head -20 || echo "(no capture)"

echo ""
echo "==> 10. ns-3 wireless pcap data frames"
DATA=$(tcpdump -r "$PCAP_DIR/mesh-0-1.pcap" 2>/dev/null \
       | grep -v "Beacon\|Action\|Acknowledgment\|CF-End" | wc -l) || DATA=0
echo "    data frames in ns-3 pcap: $DATA"

echo ""
echo "==> 11. ns-3 log tail"
tail -8 "$MESH_LOG"

echo ""
if [[ "$PING_OK" == "1" ]]; then
  echo "PHASE 1 PASS — ping succeeded through ns-3 802.11s mesh"
elif [[ "$DATA" -gt 0 ]]; then
  echo "PHASE 1 PASS (L2) — data frames traversed ns-3 mesh; see frame trace above"
else
  echo "PHASE 1 FAIL"
  exit 1
fi
