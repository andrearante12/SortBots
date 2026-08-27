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
  Isaac Sim ──── veth0-h ──── veth0 (ns-robot_0) ── tap-robot_0 ──┐
                                                                  │
  dashboard ──── veth1-h ──── veth1 (ns-robot_1) ── tap-robot_1 ──┤
  map_merge                                                       │
  FastDDS DS                                                      │
                                                                  ▼
                                                    ns-3 (RealtimeSimulatorImpl)
                                                         802.11s + HWMP mesh
```

## Prerequisites

- Ubuntu 22.04 or 24.04
- ROS 2 Jazzy installed (`~/ros2_jazzy/install/`, via
  `scripts/install_ros2_jazzy.sh --accept-download`). A binary install at
  `/opt/ros/jazzy/` also works, but questing ships no Jazzy packages, so
  from-source is the norm here — `run_demo.sh` prefers `$ROS2_HOME` and only
  falls back to `/opt/ros`.
- Isaac Sim 5.1 installed (`scripts/install_isaac_sim.sh --accept-download`)
- Wireshark, for inspecting mesh traffic (`sudo apt install wireshark`). Only
  needed to *look* at the mesh, not to run it — `install_ns3.sh` warns if it is
  missing rather than failing.
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
  "source ${ROS2_HOME:-$HOME/ros2_jazzy}/install/setup.bash && \
   export ROS_DISCOVERY_SERVER=10.77.1.1:11811 && \
   ros2 topic pub -r 1 /explore/claims std_msgs/msg/String '{data: hello}'"
```

Expect RTPS frames in the tcpdump output on `tap-robot_0`.

### Phase 3: pcap inspection

Two different captures, and they show different layers — don't confuse them:

- **ns-3's own wireless pcap** is the 802.11 side. Add `--pcap-dir /tmp/mesh` to
  `run_mesh.sh` (or set it in `network/mesh_config.yaml`), then open
  `/tmp/mesh/mesh-*-0.pcap`. Frames show IEEE 802.11 headers with RTPS payloads
  — this is where you confirm the *radio* is carrying the traffic.
- **The tap capture** is the IP side, and is what you want for reading actual
  robot-to-robot conversation. Use `scripts/capture_mesh_traffic.sh`; the full
  walkthrough is in [Watching the robots talk](#watching-the-robots-talk-wireshark)
  below.

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

## Watching the robots talk (Wireshark)

Where the packets are decides how you capture them. Under `--mesh`, each robot's
ROS 2 stack runs in its own network namespace, so **inter-robot traffic is on
`tap-robot_N` inside `ns-robot_N` and is invisible to a capture run in the root
namespace.** A plain `wireshark` on the host shows nothing and looks like the
robots are silent. `scripts/capture_mesh_traffic.sh` exists to get this right.

| Vantage point | Interface | Answers |
|---|---|---|
| `--robot N` | `tap-robot_N` (10.66.0.x) | are the two robots exchanging data? |
| `--discovery N` | `veth${N}` (10.77.N.x:11811) | did they ever find each other? |
| `--loopback` | `lo`, root ns | same, for a non-mesh run |

### Step by step: see robot_0 and robot_1 communicating

**1. Start a two-robot mesh run** and wait for it to come up.

```bash
scripts/sim_ctl.sh console start
scripts/sim_ctl.sh start explore_fleet_mesh
scripts/sim_ctl.sh wait running --timeout 600
```

**2. Confirm both namespaces exist.** If these are missing, the run isn't up and
every capture below will be empty.

```bash
sudo ip netns list | grep ns-robot
# expect: ns-robot_0, ns-robot_1
```

**3. Capture the inter-robot link for 30 s.** The robots only exchange
`/explore/claims`, `/map`, and `/tf` over the mesh, and claims are published on
frontier events — so let exploration actually run while this captures.

```bash
scripts/capture_mesh_traffic.sh --robot 0 --seconds 30 --open
```

This wraps `dumpcap` in `ip netns exec`, hands the file back to you (see
[gotchas](#gotchas) — this is the step people get wrong), and opens Wireshark.

To watch it stream live instead of capturing to a file:

```bash
scripts/capture_mesh_traffic.sh --robot 0 --live
```

**4. In Wireshark, set the display filter to `rtps`.** Wireshark's built-in RTPS
dissector decodes DDS automatically — no config or plugin needed. Everything
that remains is ROS 2 traffic.

**5. Confirm it is genuinely robot-to-robot over the mesh.** Tighten the filter
to the two tap addresses:

```
rtps && ip.addr == 10.66.0.1 && ip.addr == 10.66.0.2
```

Packets here prove the traffic crossed the simulated 802.11s PHY. **If this is
empty but `rtps` alone is busy, the robots are bypassing the mesh** — see
[SHM bypass](#risks-and-known-limitations); that filter is the fastest way to
catch it.

**6. Read the conversation.** Useful filters, narrowest last:

| Filter | Shows |
|---|---|
| `rtps` | all DDS traffic |
| `rtps.sm.id == 0x15` | DATA submessages — the actual published payloads |
| `rtps.sm.id == 0x07` | HEARTBEATs — a reliable writer chasing ACKs |
| `rtps.sm.id == 0x06` | ACKNACKs — the reader's side of that |
| `udp.port == 11811` | discovery-server traffic (use `--discovery N`) |

A note on topics: RTPS DATA submessages carry a writer GUID, not a topic name.
Topic names appear only in discovery (SEDP), so to attribute payloads to
`/explore/claims` specifically, capture discovery too and match the GUID. For
most debugging "is anything flowing, and in both directions" is the real
question, and the filters above answer it directly.

**7. Use the Statistics menu for the visual view** — this is where the traffic
becomes a picture rather than a packet list:

| Wireshark menu | What it gives you here |
|---|---|
| **Statistics → Conversations** (UDP tab) | which participants talk to which, with byte and packet counts — the quickest read on whether both robots are peering |
| **Statistics → I/O Graph** | packets/sec over time; discovery storms and link drop-outs are obvious |
| **Statistics → Flow Graph** | a per-packet sequence diagram between endpoints — best for discovery handshakes |
| **Statistics → Protocol Hierarchy** | how much is discovery vs. actual user data |

The I/O Graph is the one to watch while doing
[fault injection](#phase-4-fault-injection): raise `path_loss_exp` and the
throughput curve visibly collapses.

### Gotchas

- **The capture file is root-owned.** `dumpcap` runs under sudo, so a hand-rolled
  capture lands `root:root` mode `0600` and Wireshark fails to open it as you —
  which reads like a corrupt file, not a permissions problem.
  `capture_mesh_traffic.sh` chowns it back to you; if you capture by hand, do
  `sudo chown $USER <file>` yourself.
- **`tshark` is a separate package** from `wireshark` and is often absent. The
  script uses `dumpcap` (which ships with `wireshark`) precisely so it doesn't
  depend on `tshark`. Only install `tshark` if you want to script analysis.
- **Don't run the Wireshark GUI as root.** For `--live`, only the `dumpcap` half
  of the pipe is privileged. For root-namespace captures you can drop sudo
  entirely with `sudo dpkg-reconfigure wireshark-common` and
  `sudo usermod -aG wireshark $USER` (re-login required) — but netns captures
  still need root regardless of group membership.
- **An empty capture usually means the wrong namespace**, not silence. Check
  step 2 before concluding the robots aren't talking.

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
