# SortBots 802.11s Mesh Bridge

Bridges the two-robot Isaac Sim simulation to an ns-3-simulated IEEE 802.11s
WiFi mesh so that inter-robot ROS 2 traffic (`/explore/claims`, `/map` fusion,
TF) traverses a realistic simulated PHY/MAC instead of the host loopback.

## How it works

Each robot's ROS 2 stack (RTAB-Map, Nav2, explorer, task_manager) runs inside
its own Linux network namespace (`ns-robot_0`, `ns-robot_1`). Inside each
namespace there is a tap device (`tap-robot_0`, `tap-robot_1`) that ns-3 bridges
to a simulated 802.11s mesh node. The only route from one robot's namespace to
another goes through the tap → ns-3 mesh → tap, so every DDS packet between
robots (RTPS heartbeats, user data, ACKs) experiences the simulated PHY
conditions.

Isaac Sim and the dashboard stay in the root namespace with GPU/Vulkan access
intact. Per-robot topics (cmd_vel, camera, odom) travel over a veth pair that
connects each namespace to the root, and all participants use a FastDDS
Discovery Server for unicast peer discovery (no multicast).

```
root netns
  Isaac Sim ──── veth-0-h ──── ns-robot_0 ── tap-robot_0 ──┐
                                                             │
  dashboard  ──── veth-1-h ──── ns-robot_1 ── tap-robot_1 ──┤
  map_merge                                                  │
  FastDDS DS                                          ns-3 (RealtimeSimulatorImpl)
                                                      802.11s + HWMP mesh
```

## Prerequisites

- Ubuntu 22.04 or 24.04
- ROS 2 Jazzy installed (`/opt/ros/jazzy/`)
- Isaac Sim 5.1 installed (`scripts/install_isaac_sim.sh --accept-download`)
- sudo access

## Fresh-clone bring-up

```bash
git clone <sortbots-repo> ~/SortBots
cd ~/SortBots

# 1. Install Isaac Sim (skip if already done)
scripts/install_isaac_sim.sh --accept-download

# 2. Install ns-3 (build ~5-15 min)
scripts/install_ns3.sh --accept-download

# 3. One-time: set tap-creator setuid root (required for TapBridge)
sudo scripts/install_ns3.sh --setup-privileged

# 4. Run a mesh scenario
scripts/sim_ctl.sh console start
scripts/sim_ctl.sh start explore_fleet_mesh
```

If `NS3_HOME` is set to a non-default location, export it before step 2-4:
```bash
export NS3_HOME=/path/to/ns-3-dev
```

## One-time setuid (if install_ns3.sh was already run)

```bash
sudo scripts/install_ns3.sh --setup-privileged
```

This sets the sticky bit on two ns-3 helper binaries so they can create tap
devices and raw sockets without running the whole ns-3 process as root.

## Running manually

```bash
# Standard run (no mesh)
scripts/run_demo.sh --robots 2 --explore

# Mesh run
scripts/run_demo.sh --mesh --robots 2 --explore
```

## Verification

### Phase 1: tap devices and ping

```bash
# After mesh_netns_setup.sh has run (done automatically by run_demo.sh --mesh):
sudo ip netns exec ns-robot_0 ip -4 addr show tap-robot_0
# expect: inet 10.66.0.1/24

sudo ip netns exec ns-robot_0 ping -c 3 10.66.0.2
# expect: 3 packets transmitted, 3 received
```

### Phase 2: DDS packets through the tap (not the shortcut)

In one terminal:
```bash
sudo ip netns exec ns-robot_0 tcpdump -ni tap-robot_0 -vv udp
```

In another:
```bash
sudo ip netns exec ns-robot_1 bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   export ROS_DISCOVERY_SERVER=10.77.1.1:11811 && \
   ros2 topic pub -r 1 /explore/claims std_msgs/msg/String '{data: hello}'"
```

Expect RTPS frames in the tcpdump output on `tap-robot_0`.

### Phase 3: pcap inspection

Add `--pcap-dir /tmp/mesh` to `run_mesh.sh` (or set in `network/mesh_config.yaml`),
then open `/tmp/mesh/mesh-*-0.pcap` in Wireshark. Frames should show IEEE 802.11
headers with RTPS payloads.

### Phase 4: fault injection

Increase path loss exponent to degrade the link:
```bash
# Edit network/mesh_config.yaml:
#   path_loss_exp: 4.5
# Then restart the mesh scenario.
scripts/sim_ctl.sh stop
scripts/sim_ctl.sh start explore_fleet_mesh
```

Observe in the dashboard:
- `/explore/claims` delivery rate drops (frontier collision counter rises)
- Merged `/map` update latency increases
- `explore_fleet_mesh` completion time inflates vs `explore_fleet`

To simulate a full partition (robot out of range), change `x` for `robot_1` to
`1000.0` in `network/mesh_config.yaml` — HWMP will report path failures and the
disconnected robot's `/map` will stop updating in the merged map.

## Mesh-off vs mesh-on comparison

```bash
# Baseline
scripts/sim_ctl.sh start explore_fleet
# ... wait for completion ...
scripts/sim_ctl.sh stop

# Mesh
scripts/sim_ctl.sh start explore_fleet_mesh
# ... wait for completion ...
scripts/sim_ctl.sh stop
```

Compare `~/.ros/sortbots_robot_0.db` sizes and dashboard session timings.

## Configuration

Edit `network/mesh_config.yaml` to tune:

| Key | Effect |
|---|---|
| `path_loss_exp` | Channel propagation (2=free space, 3=indoor, 4=dense) |
| `tx_power_dbm` | Per-node transmit power |
| `hwmp_root` | Force a root for proactive HWMP (ff:ff:ff:ff:ff:ff = reactive) |
| `robots[i].x/y` | Node positions in metres (affects link quality and hop count) |

Changes take effect on the next `scripts/sim_ctl.sh start explore_fleet_mesh`.

## Logs

| Log file | Contents |
|---|---|
| `/tmp/sortbots_ns3_mesh.log` | ns-3 output: HWMP peering, TapBridge attach |
| `/tmp/sortbots_fastdds.log` | FastDDS Discovery Server |
| `/tmp/sortbots_bringup_robot_0.log` | ROS 2 stack for robot_0 (in netns) |
| `/tmp/sortbots_bringup_robot_1.log` | ROS 2 stack for robot_1 (in netns) |
| `/tmp/sortbots_demo_bringup.log` | root-netns bringup (map_merge, dashboard) |

## Risks and known limitations

**SHM bypass** — if FastDDS SHM transport is active, robots on the same host
share `/dev/shm` across namespaces and bypass the tap silently. The XML profiles
rendered by `network/render_dds_profile.py` disable SHM (`useBuiltinTransports=false`).
If you suspect bypass: verify RTPS frames appear on the tcpdump above.

**MTU** — tap devices are set to MTU 1400 (802.11s header overhead). Large RTPS
fragments (e.g. high-resolution camera topics) may be dropped. Camera topics are
per-robot and stay on the veth path; the mesh only carries `/explore/claims`,
`/map`, and `/tf` which fit within 1400 bytes under typical ROS 2 QoS.

**Full partition** — a `RELIABLE` DDS writer (like `/explore/claims`) will stall
its thread if it can't get ACKs. This is intentional — it's the behavior the
mesh bridge is designed to surface. Under partition, explorer.py stops publishing
claims (no peers to coordinate with) and tasks may queue. This is correct
decentralized behavior.

**sudo prompts** — `mesh_netns_setup.sh` and the per-robot `ip netns exec` wraps
require root. If your system requires a password each time, pre-authenticate with
`sudo -v` before starting `run_demo.sh --mesh`.
