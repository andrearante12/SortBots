# Prompt: Plan a simulation of our robot fleet communication system

You are helping a university capstone team plan a simulation of the communication system for a fleet of autonomous warehouse robots. **Produce a plan only; do not write implementation code yet.** Work in plan mode. Where a decision depends on information you do not have, list it as an open question instead of assuming. Mark every number below that is labeled "placeholder" or "estimate" as a parameter to be replaced by measurement, never as a fact.

---

## 1. What the simulation must answer

1. **Scale to failure.** Sweep the robot count n upward (2, 5, 10, 15, 20, … until failure) and find the largest n at which the system still meets its requirements, for each design variant in section 6. "Failure" is defined in section 7, not by eye.
2. **Link and node failures.** Inject failed links, degraded links, dead relay robots, a lost band, network partitions and a lost gateway, and show the system recovers correctly: no duplicate task ownership, no abandoned tasks, recovery within the timer budget.
3. **Requirement checks.** Verify the latency targets and the timer ordering in section 4 under load and under failure.
4. **Design comparison.** Quantify how much each design choice (bundling, neighbor-only topics, dual band, one-hop broadcast) moves the failure point.

The headline outputs are (a) a chart of channel busy time and latency vs n for each design variant, with the failure point marked, and (b) a table of failure scenarios with recovery times and correctness results.

---

## 2. System under test (simulate this design faithfully)

### Purpose
Robots autonomously choose tasks from a shared list, path to a package, and carry it to a drop-off, coordinating with each other with **no central coordinator**. A gateway with an operator GUI only injects tasks and displays status.

### Layers (top to bottom)

| Layer | Component | Runs on | Job |
| --- | --- | --- | --- |
| Application | ROS 2 nodes: task allocation, navigation, perception, GUI | Jetson / operator laptop | Decides what to say; acts on what it hears |
| Middleware | Zenoh via `rmw_zenoh_cpp`, one `zenohd` router per machine | Jetson / operator laptop | Delivers each message to every subscriber |
| Network | Static IPs on one flat subnet over `bat0` | Every node | One flat LAN; multi-hop is invisible above this |
| Mesh routing | batman-adv (B.A.T.M.A.N. IV) | Mesh router on each robot | Best next hop, band choice, relaying, self-healing |
| Link | 802.11s mesh point, HWMP forwarding off (`mesh_fwding 0`), SAE encryption | Mesh router | Encrypted radio links to neighbors in range |
| Physical | Two radios per router: 2.4 GHz and 5 GHz, OFDM, 20 MHz channels | Mesh router | Radio transmission |

### Nodes
- **Each robot:** a Jetson (ROS 2 + its own Zenoh router) connected by Ethernet to its own dual-band mesh router. The Jetson runs no mesh software; the router's Ethernet port is bridged with `bat0`, and batman-adv's translation table maps the Jetson's MAC to its router.
- **Gateway:** a mesh router with the same configuration, mounted high, with the operator laptop attached by Ethernet. The laptop runs its own `zenohd` and the GUI.
- The router model is **not yet chosen**; any dual-band router that runs vanilla OpenWrt with mesh-point mode on both radios qualifies.

### Network isolation: our own network, not school Wi-Fi
- The fleet runs on **its own dedicated network that the team controls**: our own mesh routers and gateway, our own SSID/mesh ID, our own subnet and static addresses. **Nothing depends on the school's Wi-Fi (eduroam)** for operation, discovery, time sync or testing.
- No DHCP, DNS, NTP or internet dependency on the campus network. The gateway laptop is the time source (chrony) for the fleet.
- If the operator laptop is also connected to eduroam for internet, Zenoh must listen and connect **only on the mesh (Ethernet) interface**, so no robot traffic ever crosses the campus network. The plan should include a check that enforces this.
- The development fallback network (a plain Wi-Fi access point used before the mesh is ready) is **also our own router**, not eduroam.
- eduroam and other campus networks appear in the simulation **only as background interference**: a configurable share of channel busy time on 2.4 GHz and 5 GHz, plus the option to model a congested 2.4 GHz channel. Include sweeps of this background load (for example 0%, 15%, 30%) because it directly reduces the airtime the fleet can use.

### Mesh routing details
- batman-adv B.A.T.M.A.N. IV, originator interval 500 ms (default 1000 ms; make it a parameter), hop penalty default.
- Both radios' mesh interfaces belong to the same `bat0`. batman-adv scores each link separately and can alternate bands along a multi-hop path, so a relay can receive on one band while forwarding on the other.
- MTU 1532 on mesh interfaces (32-byte batman-adv header); multicast rate raised to 12 Mbps; legacy 802.11b rates off.
- batman-adv floods ordinary broadcasts mesh-wide.

### Middleware details (Zenoh)
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp`. One `zenohd` per machine started before ROS nodes; nodes connect only to `tcp/localhost:7447`.
- Routers peer with explicit `connect` endpoints to every other robot and the gateway (generated from one fleet list); multicast scouting off; listen only on the mesh address.
- Router-to-router transport is TCP. Each subscriber router gets its own copy of each message (unicast fan-out). Zenoh batches small queued messages.
- Zenoh transport lease is a tunable timer (section 4).
- Fallback middleware for comparison: Cyclone DDS with multicast off and a fixed peer list.

### Interface contract (messages)
Every message has a header: `version` (uint8), `robot_id` (uint8), `seq` (uint32), `stamp` (synchronized time).

| Message | Payload (approx.) | Sent when | Delivery | Receivers |
| --- | --- | --- | --- | --- |
| TaskAnnounce | ~40 B | Operator adds a task | Reliable, transient-local | All robots |
| Bid | ~20 B | Bidding window open | Reliable | Robots near the pickup, gateway |
| Claim | ~24 B (includes cost, lease duration) | Robot wins | Reliable | All robots, gateway |
| Complete / Release / TaskCancel | ~16–18 B | Done / give up / operator cancels | Reliable | All robots, gateway |
| Status (bundled heartbeat) | ~100–300 B: pose, velocity, battery, owned tasks, path summary (next ~10 waypoints, only after replan), new dynamic obstacles | 3 Hz (placeholder) | Best-effort, newest wins | Neighbor cells at 3 Hz; gateway at 1 Hz |
| ObstacleEvent | ~24–32 B each | Appears / changes beyond threshold / clears | Reliable | All robots (fleet-wide) |
| MapUpdate | up to ~8 KB per chunk, compressed occupancy patch | Every 2–5 s, rate-capped (placeholder 100 kbps per robot) | Best-effort, chunked | Region subscribers |
| MapResync | ~12 B | Gap in map seq detected | Reliable | Region's reporting robot |
| Emergency | ~16 B | Triggered | Reliable, top priority | All robots |
| LinkHealth | ~50 B | 1 Hz | Best-effort | Gateway |

Topic layout: `fleet/tasks/…`, `fleet/status/<cell>` (robots subscribe to their own cell, neighboring cells and cells on their path; cell size is a placeholder), `fleet/obstacles` (fleet-wide), `fleet/map/<region>`, `fleet/emergency`, `fleet/link_health`.

### Task allocation protocol
- States per task: Open → Bidding → Claimed → Done (Release or lease expiry returns it to Open).
- Auction: on TaskAnnounce, robots within a set distance of the pickup bid their estimated travel time; bidding window 1 s; lowest cost wins, ties to lowest robot ID; winner publishes Claim with a lease (10 s).
- The owner renews the lease by listing the task in every Status message. Each robot measures lease time on its **own clock** from the last time it heard the owner.
- Conflicts: duplicate claims resolved by the same tie-break; no winner → rebid after random delay; partitions handled by leases (isolated owner must finish or stop before its lease ends; others reclaim only after lease expiry + margin).
- An isolated robot finishes or stops its current task within its lease, stops claiming new tasks, relies on local perception for collision avoidance, and holds safely after a hold-safe timeout.
- If the gateway is down, robots keep trading tasks already in the pool; they only stop receiving new ones.

### Obstacles and maps
- One owner per obstacle (closest robot); events, not timers; each dynamic obstacle carries velocity and expiry; static obstacles sent once plus a "cleared" event.
- Requires a shared lab coordinate frame and synchronized clocks. **The SLAM package is not yet chosen; do not assume one.** Model each robot's pose as ground truth plus configurable noise and drift.
- Camera images and point clouds never cross the mesh.
- Map updates share changes only, with per-region sequence numbers, MapResync on gaps, and periodic full refresh.

### Safety boundary
Network data is for planning only. Collision avoidance and emergency stop are local to each robot. The simulation does not need to model collisions physically, but must flag any case where a robot's planning view of another robot is staler than a configurable threshold.

---

## 3. Upgrade phases (simulate each as a variant)

| Phase | Change |
| --- | --- |
| 0 | Plain Wi-Fi access point (our own router), no mesh |
| 1 | Single-band mesh baseline: batman-adv + Zenoh, bundled status, neighbor-only topics, leases |
| 2 | Second band added to `bat0` |
| 3 | One-hop broadcast for status (sent once on the radio, outside batman-adv, best-effort) |
| 4 | Reliable broadcast for obstacles (sequence numbers + negative acknowledgment repair) |

---

## 4. Timers and requirements to verify

Required ordering: **batman-adv reroute time < Zenoh transport lease < status timeout (~3 s) < task lease (10 s)**, plus a reclaim margin after lease expiry. All values are parameters; reroute time will come from hardware measurement.

Latency requirement (robot to robot, control messages = status, bids, claims, obstacle events):
- 95% within 150 ms and 99% within 500 ms,
- across up to 3 hops, at 10 robots, with channel busy time under 50% per band,
- excluding reroute windows, which are measured separately.
- Map updates have a looser target (placeholder 1–2 s).

Report the number of samples behind each percentile (hundreds for p95, thousands for p99).

---

## 5. Approach the plan should evaluate

Evaluate and recommend a toolchain. The team's working assumption, to confirm or reject with reasons:

- **High-fidelity emulation:** Mininet-WiFi with the kernel's simulated radio driver (`mac80211_hwsim`) and its wireless medium simulator (wmediumd), so each simulated robot runs the **real** stack in its own network namespace: real 802.11s, real batman-adv, real `zenohd` and `rmw_zenoh`, and lightweight ROS 2 stub nodes. Two simulated radios per node for dual band.
- **Scale extension:** emulation will hit host CPU/memory limits at some n. Plan a second, lighter model (for example a discrete-event simulation in Python, or ns-3) calibrated against the emulation at small n, to extend the sweep to larger n. Show how the two are cross-validated where they overlap.
- **Calibration from hardware:** the lab testbed (2 robots + gateway, lab mock-up with metal wire shelving, all on our own network) supplies measured parameters: signal vs distance per band, packet loss, throughput, reroute time, background busy time from `iw dev <iface> survey dump`. The plan must say which simulation parameters each measurement replaces.

Also address:
- Host requirements (Linux kernel with `mac80211_hwsim`, root, CPU/RAM per emulated robot), and whether it runs on a team laptop, a lab desktop or a VM.
- How robots are represented: ROS 2 stub nodes that publish the message types in section 2 at configured rates and run the real auction and lease logic, without real SLAM, navigation or perception.
- Mobility: robots moving on a 2D floor map (lab mock-up layout and a larger warehouse layout with aisles and racks), with link quality driven by distance and obstruction. Racks attenuate strongly; wire shelving attenuates mildly. Use published attenuation figures for loaded racks, cited, rather than invented ones.
- Workload: a task generator with configurable arrival rate, pickup/drop-off locations, and burstiness.
- Background interference model for campus networks (section 2).

---

## 6. Design variants to compare

| Variant | Description |
| --- | --- |
| V1 Naive | Separate messages for status, position, path, obstacles, each sent to every robot |
| V2 Bundled | One bundled status message at 3 Hz, sent to every robot |
| V3 Bundled + neighbor-only | Status to neighbor cells only; auctions limited to robots near the pickup |
| V4 = V3 + dual band | Mesh on both 2.4 and 5 GHz |
| V5 = V4 + one-hop status broadcast | Phase 3 |
| V6 = V5 + reliable obstacle broadcast | Phase 4 |
| Middleware check | V3 on Cyclone DDS with fixed peers, to show the middleware is swappable |
| Phase 0 | Single access point (our own router), no mesh, for comparison |

Expected shape to confirm or refute: V1 grows roughly with n², V3 and later grow roughly linearly. Earlier hand estimates (to be replaced, not trusted): V1 crosses 50% busy near 11 robots, V2 near 19, V3 stays near 14% at 20 robots.

---

## 7. Failure criteria for the scale sweep

The system "fails" at the smallest n where any of these holds over a steady-state run (define warm-up and run length in the plan):

- Channel busy time > 50% on any band in any neighborhood,
- control-message latency p95 > 150 ms or p99 > 500 ms,
- status delivery ratio below a threshold (propose one),
- any false "robot silent" decision (status timeout fired on a healthy robot),
- any duplicate task ownership or abandoned task,
- auction completion time beyond a threshold (propose one),
- map divergence between robots beyond a threshold (propose a metric).

Report which criterion fails first for each variant, since that names the bottleneck.

---

## 8. Failure scenarios to inject

| Scenario | Injection | Pass condition |
| --- | --- | --- |
| Single link loss | Block one link (`iw dev <mesh-iface> station set <MAC> plink_action block`) on one or both bands | Traffic reroutes; no app-level decisions triggered |
| Degraded link | Raise loss / lower signal on one link gradually | batman-adv shifts to better path; latency bounded |
| Link flapping | Toggle a link at varying periods | No route oscillation storms; no false silent decisions |
| Relay robot dies | Kill a robot that relays for others | Reroute time measured; its task reclaimed only after lease + margin |
| Robot isolated but alive | Block all its links | It finishes or stops within its lease; no duplicate work |
| Band loss | Disable 2.4 GHz everywhere (or saturate with background load) | Traffic moves to 5 GHz; degraded capacity, no outage |
| Partition | Split the fleet into two groups, then heal | No duplicate ownership; duplicates resolved by tie-break on heal |
| Gateway loss | Kill the gateway | Robots continue existing tasks; GUI rebuilds view on return |
| Reboot | Restart a robot | Rejoins; receives open tasks via transient-local |
| Clock drift | Offset a robot's clock | Leases unaffected; obstacle extrapolation error reported |
| Burst load | Many tasks at once; many obstacles moving | Auctions complete; control traffic not starved by map data |
| Timer misordering | Deliberately set status timeout < reroute time | Show the false-silent failures this causes (negative control) |

Each scenario runs at several fleet sizes and design variants; the plan should propose which combinations are worth running.

---

## 9. Metrics and outputs

Per run, log to files (CSV or Parquet) with a run ID, seed and full configuration:
- Per band: channel busy time, overall and per neighborhood.
- Per message type: end-to-end latency (send stamp to receive time), delivery ratio, hop count.
- batman-adv: route changes, reroute time after each failure.
- Zenoh: connection drops/rebuilds, queue depth.
- Task layer: auction duration, duplicate claims, abandoned tasks, lease expiries, false silent decisions, task throughput.
- Obstacle/map layer: obstacle report age at receipt, map divergence.

Deliver: plots of busy time and latency vs n per variant with the failure point marked, recovery-time charts per failure scenario, and a summary table. All runs must be reproducible from a config file and seed; one fleet list generates every node's addresses and Zenoh peer config.

---

## 10. What the plan document must contain

1. Recommended toolchain with justification and rejected alternatives.
2. Simulation architecture: components, how real software and stubs fit together, how dual-band radios, broadcast variants and background interference are modeled.
3. Network isolation design: how the simulation and the hardware testbed both stay on our own network and never depend on eduroam, and how that is checked.
4. Parameter table: every parameter, its default, its source (measured / placeholder / literature), and which hardware test replaces it.
5. Experiment matrix: variants × fleet sizes × failure scenarios × background load, with run counts and estimated compute time.
6. Failure criteria and metrics definitions, including thresholds you propose.
7. Validation plan: cross-checks between emulation and the lighter model, and between simulation and lab measurements at n = 2–3.
8. Repository layout, configuration format, and run/automation scripts (described, not written).
9. Milestones across two semesters, aligned with the upgrade phases in section 3, with a minimal first milestone that produces a real number quickly.
10. Risks (for example host limits on emulated nodes, wmediumd realism, rmw_zenoh behavior in namespaces) and mitigations.
11. Open questions for the team.

Keep the plan concrete. Do not invent measured values, hardware models, or results.
