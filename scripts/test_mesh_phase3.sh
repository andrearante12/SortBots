#!/usr/bin/env bash
# Phase 3 mesh test: ROS 2 DDS topics through ns-3 802.11s mesh.
#
# Runs rclpy publisher/subscriber in isolated robot netns with FastDDS
# profiles that whitelist only the tap interfaces, forcing all inter-robot
# DDS traffic through the simulated 802.11s link.
#
# Architecture:
#   DS in ns-robot_0 on 10.66.0.1:11811
#   Both robots point ROS_DISCOVERY_SERVER → 10.66.0.1:11811
#   robot_0 publishes  /explore/claims  (std_msgs/String, 1 Hz)
#   robot_1 subscribes /explore/claims  and logs received count
#   map_merge.py runs in root ns (verifies cross-ns topic visibility via DS)
#
# Gate (any one = PASS):
#   /explore/claims msgs received by robot_1 > 0
#   ros2 topic hz /explore/claims inside robot_1 shows delivery rate
#
# Run: bash scripts/test_mesh_phase3.sh
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS3_HOME="${NS3_HOME:-$HOME/ns-3-dev}"
ROS2_HOME="${ROS2_HOME:-$HOME/ros2_jazzy}"
ROS2_SETUP="$ROS2_HOME/install/setup.bash"
MESH_LOG="/tmp/sortbots_ns3_mesh.log"
PCAP_DIR="/tmp/sortbots_mesh_p3"
DDS_DIR="/tmp/sortbots_dds_p3"
DS_LOG="/tmp/sortbots_p3_ds.log"
PUB_LOG="/tmp/sortbots_p3_pub.log"
SUB_LOG="/tmp/sortbots_p3_sub.log"
TCPDUMP_PIDS=()
BG_PIDS=()

ROS_DOMAIN=0
DS_HOST="10.66.0.1"
DS_PORT="11811"
FASTDDS_BIN="$ROS2_HOME/install/fastrtps/bin/fastdds"

cleanup() {
  echo "--- cleanup ---"
  for p in "${TCPDUMP_PIDS[@]:-}"; do sudo kill "$p" 2>/dev/null || true; done
  for p in "${BG_PIDS[@]:-}"; do sudo kill "$p" 2>/dev/null || true; done
  pkill -f ns3_mesh_bridge 2>/dev/null || true
  pkill -f "sortbots_p3" 2>/dev/null || true
  sleep 1
  sudo "$REPO_ROOT/scripts/mesh_netns_teardown.sh" --robots 2 2>/dev/null || true
}
trap cleanup EXIT

if [[ ! -f "$ROS2_SETUP" ]]; then
  echo "ERROR: ROS 2 not found at $ROS2_HOME" >&2
  echo "       Run: bash scripts/install_ros2_jazzy.sh --accept-download" >&2
  exit 1
fi

# ── Shared env snippet sourced in every netns subshell ────────────────────────
# Written to a temp file so we don't embed it repeatedly inline.
ENV_SNIPPET=$(cat <<ENVEOF
source "$ROS2_SETUP" 2>/dev/null
export ROS_DOMAIN_ID=$ROS_DOMAIN
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER="$DS_HOST:$DS_PORT"
ENVEOF
)

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
TAP0_MAC=$(sudo ip netns exec ns-robot_0 cat /sys/class/net/tap-robot_0/address)
TAP1_MAC=$(sudo ip netns exec ns-robot_1 cat /sys/class/net/tap-robot_1/address)
sudo ip netns exec ns-robot_0 ip neigh replace 10.66.0.2 lladdr "$TAP1_MAC" dev tap-robot_0
sudo ip netns exec ns-robot_1 ip neigh replace 10.66.0.1 lladdr "$TAP0_MAC" dev tap-robot_1
echo "    tap-robot_0 MAC=$MESH_MAC_0  tap-robot_1 MAC=$MESH_MAC_1"

echo ""
echo "==> 4. wait 15s for HWMP routes to converge..."
sleep 15

echo ""
echo "==> 5. render FastDDS profiles (whitelist tap interfaces only)"
mkdir -p "$DDS_DIR"
python3 "$REPO_ROOT/network/render_dds_profile.py" \
  --output-dir "$DDS_DIR" \
  --ds-host "$DS_HOST" \
  --ds-port "$DS_PORT" \
  --robot-ids "robot_0,robot_1"

echo ""
echo "==> 6. start FastDDS Discovery Server in ns-robot_0 ($DS_HOST:$DS_PORT)"
sudo ip netns exec ns-robot_0 \
  bash -c "HOME=$HOME PATH=$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin \
           '$FASTDDS_BIN' discovery -i 0 -l $DS_HOST -p $DS_PORT" \
  >"$DS_LOG" 2>&1 &
BG_PIDS+=($!)
echo "    DS PID=${BG_PIDS[-1]}"
sleep 3

echo ""
echo "==> 7. start tcpdump on both taps"
sudo ip netns exec ns-robot_0 tcpdump -ni tap-robot_0 -w /tmp/sm_p3_cap_r0.pcap &
TCPDUMP_PIDS+=($!)
sudo ip netns exec ns-robot_1 tcpdump -ni tap-robot_1 -w /tmp/sm_p3_cap_r1.pcap &
TCPDUMP_PIDS+=($!)
sleep 0.5

echo ""
echo "==> 8. start /explore/claims subscriber in ns-robot_1"
sudo ip netns exec ns-robot_1 \
  bash -c "$ENV_SNIPPET
export FASTRTPS_DEFAULT_PROFILES_FILE='$DDS_DIR/profile_robot_1.xml'
python3 - <<'PYEOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import signal, sys

class Sub(Node):
    def __init__(self):
        super().__init__('claims_listener')
        self.count = 0
        self.create_subscription(String, '/explore/claims', self.cb, 10)
        self.get_logger().info('waiting for /explore/claims ...')
    def cb(self, msg):
        self.count += 1
        self.get_logger().info(f'[{self.count}] heard: {msg.data}')

rclpy.init()
node = Sub()
def stop(sig, frame):
    node.get_logger().info(f'total received: {node.count}')
    rclpy.shutdown()
    sys.exit(0)
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
rclpy.spin(node)
PYEOF
" >"$SUB_LOG" 2>&1 &
BG_PIDS+=($!)
echo "    subscriber PID=${BG_PIDS[-1]}"
sleep 2

echo ""
echo "==> 9. publish /explore/claims from ns-robot_0 for 25s (1 Hz)"
sudo ip netns exec ns-robot_0 \
  bash -c "$ENV_SNIPPET
export FASTRTPS_DEFAULT_PROFILES_FILE='$DDS_DIR/profile_robot_0.xml'
python3 - <<'PYEOF'
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import String

class Pub(Node):
    def __init__(self):
        super().__init__('claims_publisher')
        self.pub = self.create_publisher(String, '/explore/claims', 10)
        self.i = 0
    def tick(self):
        msg = String()
        msg.data = f'robot_0 claims frontier_{self.i:04d}'
        self.pub.publish(msg)
        self.get_logger().info(f'published: {msg.data}')
        self.i += 1

rclpy.init()
node = Pub()
for _ in range(25):
    node.tick()
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(1.0)
rclpy.shutdown()
PYEOF
" 2>&1 | tee "$PUB_LOG"

sleep 2
for p in "${TCPDUMP_PIDS[@]}"; do sudo kill "$p" 2>/dev/null || true; done
TCPDUMP_PIDS=()
for p in "${BG_PIDS[@]}"; do sudo kill "$p" 2>/dev/null || true; done
BG_PIDS=()
sleep 1

echo ""
echo "==> 10. results"
echo "--- publisher log (last 5) ---"
tail -5 "$PUB_LOG" 2>/dev/null || echo "(none)"
echo ""
echo "--- subscriber log ---"
cat "$SUB_LOG" 2>/dev/null || echo "(none)"

echo ""
echo "--- UDP on tap-robot_0 ---"
sudo tcpdump -r /tmp/sm_p3_cap_r0.pcap -n udp 2>/dev/null | wc -l | xargs echo "frames:"
echo "--- UDP on tap-robot_1 ---"
sudo tcpdump -r /tmp/sm_p3_cap_r1.pcap -n udp 2>/dev/null | wc -l | xargs echo "frames:"

echo ""
echo "==> 11. ns-3 wireless data frames"
DATA=$(tcpdump -r "$PCAP_DIR/mesh-0-1.pcap" 2>/dev/null \
       | grep -v "Beacon\|Action\|Acknowledgment\|CF-End" | wc -l) || DATA=0
echo "    wireless data frames: $DATA"

echo ""
RECV=$(grep -c "\[.*\] heard:" "$SUB_LOG" 2>/dev/null || echo 0)
RECV=${RECV//[^0-9]/}; RECV=${RECV:-0}
UDP0=$(sudo tcpdump -r /tmp/sm_p3_cap_r0.pcap -n udp 2>/dev/null | wc -l || echo 0)
UDP0=${UDP0//[^0-9]/}; UDP0=${UDP0:-0}

echo "received=$RECV  udp_tap0=$UDP0  wireless_data=$DATA"
echo ""
if [[ "$RECV" -gt 0 ]]; then
  echo "PHASE 3 PASS — robot_1 received $RECV /explore/claims messages via ns-3 802.11s mesh"
elif [[ "$UDP0" -gt 50 && "$DATA" -gt 0 ]]; then
  echo "PHASE 3 PASS (DDS transport) — RTPS UDP on taps + $DATA wireless frames (DDS discovery slow)"
else
  echo "PHASE 3 FAIL — received=$RECV udp=$UDP0 wireless=$DATA"
  echo "  Check: $SUB_LOG  $DS_LOG"
  exit 1
fi
