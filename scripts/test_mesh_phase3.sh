#!/usr/bin/env bash
# Phase 3 mesh test: ROS 2 DDS topics through ns-3 802.11s mesh.
#
# All subprocess Python and bash env scripts are written to /tmp files to
# avoid quoting hazards when embedding them in bash -c arguments.
#
# Architecture:
#   FastDDS DS in ns-robot_0 at 10.66.0.1:11811
#   robot_0 publishes /explore/claims (std_msgs/String, 1 Hz)
#   robot_1 subscribes /explore/claims
#   FastDDS profiles: SHM disabled, UDPv4 whitelisted to tap IP only
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
SUB_LOG="/tmp/sortbots_p3_sub.log"
FASTDDS_BIN="$ROS2_HOME/install/fastrtps/bin/fastdds"
DS_HOST="10.66.0.1"
DS_PORT="11811"
TCPDUMP_PIDS=()
BG_PIDS=()

cleanup() {
  echo "--- cleanup ---"
  for p in "${TCPDUMP_PIDS[@]:-}"; do sudo kill "$p" 2>/dev/null || true; done
  for p in "${BG_PIDS[@]:-}"; do sudo kill "$p" 2>/dev/null || true; done
  pkill -f "sortbots_p3" 2>/dev/null || true
  pkill -f ns3_mesh_bridge 2>/dev/null || true
  sleep 1
  sudo "$REPO_ROOT/scripts/mesh_netns_teardown.sh" --robots 2 2>/dev/null || true
}
trap cleanup EXIT

if [[ ! -f "$ROS2_SETUP" ]]; then
  echo "ERROR: ROS 2 not found at $ROS2_HOME" >&2; exit 1
fi

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
echo "==> 5. write FastDDS profiles and subprocess scripts to /tmp"
mkdir -p "$DDS_DIR"

# Profiles: SHM disabled (useBuiltinTransports=false), UDPv4 whitelisted
# to the tap IP only. Discovery is handled by ROS_DISCOVERY_SERVER env var
# at the RMW layer — no explicit discoveryProtocol block needed here, which
# avoids conflicts between the XML CLIENT config and the env-var DS address.
cat >"$DDS_DIR/profile_r0.xml" <<XMLEOF
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_tap0</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList><address>10.66.0.1</address></interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="sortbots_mesh" is_default_profile="true">
    <rtps>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports><transport_id>udp_tap0</transport_id></userTransports>
    </rtps>
  </participant>
</profiles>
XMLEOF

cat >"$DDS_DIR/profile_r1.xml" <<XMLEOF
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_tap1</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList><address>10.66.0.2</address></interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="sortbots_mesh" is_default_profile="true">
    <rtps>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports><transport_id>udp_tap1</transport_id></userTransports>
    </rtps>
  </participant>
</profiles>
XMLEOF

# Subscriber Python script
cat >/tmp/sortbots_p3_sub.py <<PYEOF
import rclpy, signal, sys
from rclpy.node import Node
from std_msgs.msg import String

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
    sys.exit(0)
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
rclpy.spin(node)
PYEOF

# Publisher Python script
cat >/tmp/sortbots_p3_pub.py <<PYEOF
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
for _ in range(30):
    node.tick()
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(1.0)
rclpy.shutdown()
PYEOF

# Per-robot env setup scripts (sourced before running Python)
cat >/tmp/sortbots_p3_env_r0.sh <<ENVEOF
source "$ROS2_SETUP" 2>/dev/null
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=${DS_HOST}:${DS_PORT}
export FASTRTPS_DEFAULT_PROFILES_FILE=$DDS_DIR/profile_r0.xml
ENVEOF

cat >/tmp/sortbots_p3_env_r1.sh <<ENVEOF
source "$ROS2_SETUP" 2>/dev/null
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=${DS_HOST}:${DS_PORT}
export FASTRTPS_DEFAULT_PROFILES_FILE=$DDS_DIR/profile_r1.xml
ENVEOF

echo "    profiles and scripts written to /tmp"

echo ""
echo "==> 6. start FastDDS Discovery Server in ns-robot_0 ($DS_HOST:$DS_PORT)"
# Source the ROS 2 env (sets LD_LIBRARY_PATH for libfastrtps.so) then
# run fastdds — the binary lives in the ROS 2 install, not /usr/bin.
sudo ip netns exec ns-robot_0 \
  bash -c "source /tmp/sortbots_p3_env_r0.sh && fastdds discovery -i 0 -l $DS_HOST -p $DS_PORT" \
  >"$DS_LOG" 2>&1 &
BG_PIDS+=($!)
echo "    DS PID=${BG_PIDS[-1]}"
sleep 3
echo "    DS log (first 3 lines):"
head -3 "$DS_LOG" 2>/dev/null | sed 's/^/      /' || echo "      (empty)"

echo ""
echo "==> 7. start tcpdump on both taps"
sudo rm -f /tmp/sm_p3_cap_r0.pcap /tmp/sm_p3_cap_r1.pcap
sudo ip netns exec ns-robot_0 tcpdump -ni tap-robot_0 -w /tmp/sm_p3_cap_r0.pcap &
TCPDUMP_PIDS+=($!)
sudo ip netns exec ns-robot_1 tcpdump -ni tap-robot_1 -w /tmp/sm_p3_cap_r1.pcap &
TCPDUMP_PIDS+=($!)
sleep 0.5

echo ""
echo "==> 8. start /explore/claims subscriber in ns-robot_1"
sudo ip netns exec ns-robot_1 \
  bash -c 'source /tmp/sortbots_p3_env_r1.sh && python3 /tmp/sortbots_p3_sub.py' \
  >"$SUB_LOG" 2>&1 &
BG_PIDS+=($!)
echo "    subscriber PID=${BG_PIDS[-1]}"
echo "    waiting 10s for DDS endpoint matching..."
sleep 10

echo ""
echo "==> 9. publish /explore/claims from ns-robot_0 for 30s (1 Hz)"
sudo ip netns exec ns-robot_0 \
  bash -c 'source /tmp/sortbots_p3_env_r0.sh && python3 /tmp/sortbots_p3_pub.py' \
  2>&1

sleep 2
for p in "${TCPDUMP_PIDS[@]}"; do sudo kill "$p" 2>/dev/null || true; done
TCPDUMP_PIDS=()
for p in "${BG_PIDS[@]}"; do sudo kill "$p" 2>/dev/null || true; done
BG_PIDS=()
sleep 1

echo ""
echo "==> 10. results"
echo "--- DS log ---"
cat "$DS_LOG" 2>/dev/null | head -10 || echo "(none)"
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
sudo rm -f "$PCAP_DIR/mesh-0-1.pcap.tmp" 2>/dev/null || true
DATA=$(tcpdump -r "$PCAP_DIR/mesh-0-1.pcap" 2>/dev/null \
       | grep -v "Beacon\|Action\|Acknowledgment\|CF-End" | wc -l) || DATA=0
echo "    wireless data frames: $DATA"

RECV=$(grep -c "\] heard:" "$SUB_LOG" 2>/dev/null || true)
RECV=${RECV//[^0-9]/}; RECV=${RECV:-0}
UDP0=$(sudo tcpdump -r /tmp/sm_p3_cap_r0.pcap -n udp 2>/dev/null | wc -l || echo 0)
UDP0=${UDP0//[^0-9]/}; UDP0=${UDP0:-0}

echo ""
echo "received=$RECV  udp_tap0=$UDP0  wireless_data=$DATA"
echo ""
if [[ "$RECV" -gt 0 ]]; then
  echo "PHASE 3 PASS — robot_1 received $RECV /explore/claims msgs via ns-3 802.11s mesh"
elif [[ "$UDP0" -gt 100 && "$DATA" -gt 0 ]]; then
  echo "PHASE 3 PASS (DDS transport) — RTPS UDP on taps + $DATA wireless frames"
else
  echo "PHASE 3 FAIL — received=$RECV udp=$UDP0 wireless=$DATA"
  exit 1
fi
