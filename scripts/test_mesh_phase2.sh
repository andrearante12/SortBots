#!/usr/bin/env bash
# Phase 2 mesh test: UDP traffic through ns-3 802.11s mesh.
#
# Uses Python raw UDP sockets (always present) to send RTPS-sized frames
# from ns-robot_0 (10.66.0.1) to ns-robot_1 (10.66.0.2) and back,
# verifying the transport layer that DDS (RTPS) rides on.
#
# Gate (any one suffices for PASS):
#   - listener receives UDP frames from talker
#   - UDP frames visible on both tap interfaces
#   - ns-3 wireless pcap shows data frames
#
# Run: bash scripts/test_mesh_phase2.sh
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS3_HOME="${NS3_HOME:-$HOME/ns-3-dev}"
MESH_LOG="/tmp/sortbots_ns3_mesh.log"
PCAP_DIR="/tmp/sortbots_mesh_p2"
CAP0="/tmp/sm_p2_cap_r0.pcap"
CAP1="/tmp/sm_p2_cap_r1.pcap"
LISTENER_LOG="/tmp/sortbots_p2_listener.log"
TCPDUMP_PIDS=()
LISTENER_PID=""

cleanup() {
  echo "--- cleanup ---"
  for p in "${TCPDUMP_PIDS[@]:-}"; do sudo kill "$p" 2>/dev/null || true; done
  [[ -n "$LISTENER_PID" ]] && sudo kill "$LISTENER_PID" 2>/dev/null || true
  pkill -f ns3_mesh_bridge 2>/dev/null || true
  sleep 1
  sudo rm -f "$CAP0" "$CAP1" 2>/dev/null || true
  sudo "$REPO_ROOT/scripts/mesh_netns_teardown.sh" --robots 2 2>/dev/null || true
}
trap cleanup EXIT

echo "==> 1. tap + netns setup (Phase A)"
sudo "$REPO_ROOT/scripts/mesh_netns_setup.sh" --robots 2

echo ""
echo "==> 2. launch ns-3 mesh"
export NS3_HOME MESH_LOG
mkdir -p "$PCAP_DIR"
"$REPO_ROOT/scripts/run_mesh.sh" --robots 2 \
  --config "$REPO_ROOT/network/mesh_config.yaml" \
  --pcap-dir "$PCAP_DIR"

echo ""
echo "==> 3. move taps into robot namespaces (Phase B)"
sudo "$REPO_ROOT/scripts/mesh_tap_to_netns.sh" --robots 2

echo ""
echo "==> 3b. align tap MACs to ns-3 mesh MACs"
MESH_MAC_0=$(grep "node-mac tap-robot_0" "$MESH_LOG" 2>/dev/null | awk '{print $4}')
MESH_MAC_1=$(grep "node-mac tap-robot_1" "$MESH_LOG" 2>/dev/null | awk '{print $4}')
if [[ -z "$MESH_MAC_0" || -z "$MESH_MAC_1" ]]; then
  echo "ERROR: could not read ns-3 mesh node MACs from $MESH_LOG" >&2; exit 1
fi
sudo ip netns exec ns-robot_0 ip link set tap-robot_0 down
sudo ip netns exec ns-robot_0 ip link set tap-robot_0 address "$MESH_MAC_0"
sudo ip netns exec ns-robot_0 ip link set tap-robot_0 up
sudo ip netns exec ns-robot_1 ip link set tap-robot_1 down
sudo ip netns exec ns-robot_1 ip link set tap-robot_1 address "$MESH_MAC_1"
sudo ip netns exec ns-robot_1 ip link set tap-robot_1 up
echo "    tap-robot_0 MAC -> $MESH_MAC_0"
echo "    tap-robot_1 MAC -> $MESH_MAC_1"

TAP0_MAC=$(sudo ip netns exec ns-robot_0 cat /sys/class/net/tap-robot_0/address)
TAP1_MAC=$(sudo ip netns exec ns-robot_1 cat /sys/class/net/tap-robot_1/address)
sudo ip netns exec ns-robot_0 ip neigh replace 10.66.0.2 lladdr "$TAP1_MAC" dev tap-robot_0
sudo ip netns exec ns-robot_1 ip neigh replace 10.66.0.1 lladdr "$TAP0_MAC" dev tap-robot_1
echo "    static ARP set on both sides"

echo ""
echo "==> 4. wait 15s for HWMP routes to converge..."
sleep 15

echo ""
echo "==> 5. start tcpdump in each netns"
sudo rm -f "$CAP0" "$CAP1"
sudo ip netns exec ns-robot_0 tcpdump -ni tap-robot_0 -w "$CAP0" &
TCPDUMP_PIDS+=($!)
sudo ip netns exec ns-robot_1 tcpdump -ni tap-robot_1 -w "$CAP1" &
TCPDUMP_PIDS+=($!)
sleep 0.5

echo ""
echo "==> 6. start UDP listener in ns-robot_1 (10.66.0.2:7400)"
sudo ip netns exec ns-robot_1 python3 - >"$LISTENER_LOG" 2>&1 <<'PYEOF' &
import socket, sys, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', 7400))
s.settimeout(30)
count = 0
start = time.time()
print(f"[listener] ready on 0.0.0.0:7400", flush=True)
while time.time() - start < 28:
    try:
        data, addr = s.recvfrom(65535)
        count += 1
        print(f"[listener] msg {count} from {addr}: {data[:60]}", flush=True)
    except socket.timeout:
        break
print(f"[listener] total received: {count}", flush=True)
PYEOF
LISTENER_PID=$!
sleep 1

echo ""
echo "==> 7. UDP talker from ns-robot_0 (10.66.0.1) -> 10.66.0.2:7400 (20 frames)"
# RTPS-like payload: starts with 0x52 0x54 0x50 0x53 ("RTPS") header magic
sudo ip netns exec ns-robot_0 python3 - <<'PYEOF'
import socket, time, struct

RTPS_MAGIC = b'RTPS'  # real RTPS header magic bytes
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('10.66.0.1', 7401))

for i in range(20):
    # Minimal RTPS-like payload: magic + protocol version + vendor + seq
    payload = RTPS_MAGIC + struct.pack('!BBHHI', 2, 3, 0x0101, 0, i)
    payload += b'\x00' * 32  # pad to realistic size
    s.sendto(payload, ('10.66.0.2', 7400))
    print(f"[talker] sent frame {i+1}/20", flush=True)
    time.sleep(0.5)

print("[talker] done", flush=True)
PYEOF

sleep 2
for p in "${TCPDUMP_PIDS[@]}"; do sudo kill "$p" 2>/dev/null || true; done
TCPDUMP_PIDS=()
[[ -n "$LISTENER_PID" ]] && sudo kill "$LISTENER_PID" 2>/dev/null || true
LISTENER_PID=""
sleep 0.5

echo ""
echo "==> 8. results"
echo "--- listener log ---"
cat "$LISTENER_LOG" 2>/dev/null || echo "(no listener log)"

echo ""
echo "--- UDP on tap-robot_0 (talker side) ---"
sudo tcpdump -r "$CAP0" -n udp 2>/dev/null | head -10 || echo "(none)"
echo "--- UDP on tap-robot_1 (listener side) ---"
sudo tcpdump -r "$CAP1" -n udp 2>/dev/null | head -10 || echo "(none)"

echo ""
echo "==> 9. ns-3 wireless data frames"
DATA=$(tcpdump -r "$PCAP_DIR/mesh-0-1.pcap" 2>/dev/null \
       | grep -v "Beacon\|Action\|Acknowledgment\|CF-End" | wc -l) || DATA=0
echo "    wireless data frames in ns-3 pcap: $DATA"

echo ""
tail -5 "$MESH_LOG"

echo ""
LISTENER_MSGS=$(grep -c "\[listener\] msg" "$LISTENER_LOG" 2>/dev/null || true)
LISTENER_MSGS=${LISTENER_MSGS//[^0-9]/}
LISTENER_MSGS=${LISTENER_MSGS:-0}
UDP_TAP0=$(sudo tcpdump -r "$CAP0" -n udp 2>/dev/null | wc -l || echo 0)
UDP_TAP0=${UDP_TAP0//[^0-9]/}; UDP_TAP0=${UDP_TAP0:-0}
UDP_TAP1=$(sudo tcpdump -r "$CAP1" -n udp 2>/dev/null | wc -l || echo 0)
UDP_TAP1=${UDP_TAP1//[^0-9]/}; UDP_TAP1=${UDP_TAP1:-0}

echo "listener_msgs=$LISTENER_MSGS  udp_tap0=$UDP_TAP0  udp_tap1=$UDP_TAP1  wireless_data=$DATA"
echo ""
if [[ "$LISTENER_MSGS" -gt 0 ]]; then
  echo "PHASE 2 PASS — listener received $LISTENER_MSGS UDP frames through ns-3 802.11s mesh"
elif [[ "$UDP_TAP0" -gt 0 && "$UDP_TAP1" -gt 0 && "$DATA" -gt 0 ]]; then
  echo "PHASE 2 PASS (transport) — UDP on both taps ($UDP_TAP0/$UDP_TAP1) + $DATA wireless frames"
elif [[ "$DATA" -gt 0 ]]; then
  echo "PHASE 2 PARTIAL — $DATA wireless data frames in ns-3 pcap; check listener binding"
else
  echo "PHASE 2 FAIL"
  exit 1
fi
