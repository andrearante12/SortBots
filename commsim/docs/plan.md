# Plan: Fleet communication simulation (`commsim/`)

## Context

[`brief.md`](brief.md) (the original brief) asks for a plan to simulate the SortBots fleet **communication** system: a dual-band 802.11s + batman-adv mesh, Zenoh middleware (`rmw_zenoh`), decentralized auction/lease task allocation, and obstacle/map sharing. The questions are: how many robots before it fails, does it recover correctly from link/node failures, does it meet the latency and timer requirements, and how much does each design choice move the failure point.

The simulation is **fully separate from the Isaac Sim / ROS 2 track**. It shares no code, shell, ROS install or launch files with it. Agreed decisions:
- It lives in a new top-level dir, `commsim/`, with its own README, environment and tests (the same arrangement as the ManiSkill track).
- Emulation runs in an **Ubuntu 24.04 VM on this laptop**. The host is Ubuntu 25.10 with no apt ROS, and Mininet-WiFi doesn't support 25.10. The VM keeps root and kernel-module work away from the Isaac setup.
- The scale extension is a **Python discrete-event simulation (SimPy)**.
- Messages use a **custom `.msg` package** with binary CDR encoding, so the wire sizes match the spec. This breaks the repo's "no custom msg" rule, but only inside `commsim/`.

**Jetson access is not needed.** Every emulated robot's `zenohd`, `rmw_zenoh` and ROS 2 stubs run in their own network namespace inside the x86 VM. A Jetson appears only in the 2-robot lab testbed, as an *optional* calibration input (zenohd/stack processing delay on Orin Nano). It is listed under open questions and is not on the critical path.

Host facts (checked read-only):
- 20 cores, 15 GB RAM, kernel 6.17.
- `mac80211_hwsim` and `batman-adv` modules are present (they'll be used inside the VM's own 24.04 kernel).
- ROS 2 is a source build in `~/ros2_jazzy` with no rmw_zenoh. It is not used.

All numbers below marked *placeholder* or *estimate* are parameters, not facts.

---

## 1. Recommended toolchain

**Confirmed: Mininet-WiFi + `mac80211_hwsim` + wmediumd for high-fidelity emulation, with a Python DES (SimPy) for scale, and both cross-validated at small n.**

| Layer | Emulation (real software) | DES (modeled) |
|---|---|---|
| PHY/MAC | hwsim radios. wmediumd in interference mode, using a per-link SNR matrix that our propagation module pushes at runtime | Per-band contention domains; per-frame airtime (preamble + MAC header + payload at rate + SIFS/ACK + DIFS/backoff); collision probability from a Bianchi-style model, calibrated against emulation |
| Link | Real 802.11s mesh point, `mesh_fwding 0`, SAE (wpa_supplicant) | Link up/down + loss(SNR) curve |
| Mesh routing | Real batman-adv IV, `batctl` | OGM flooding at originator interval, TQ/hop-penalty path choice, reroute via OGM loss |
| Middleware | Real `zenohd` + `rmw_zenoh_cpp`; Cyclone DDS for the middleware check | TCP unicast fan-out per subscriber router, batching window, retransmit on loss |
| App | rclpy stub nodes running the **real** auction/lease code | Imports the **same** pure-Python protocol code |

Rejected alternatives:
- **ns-3 only:** it has no batman-adv or Zenoh model, so the most important parts would be re-implemented.
- **Physical-only testing:** only 2 robots exist.
- **Mininet without hwsim (wired links + tc netem):** it can't produce shared-medium airtime or busy time, which is the main failure mode being studied.
- **ns-3 as the scale model:** the user chose the DES. ns-3 stays a possible later cross-check of the MAC model (`~/ns-3-dev` exists).

## 2. Simulation architecture

- **One fleet list → everything.**
  - `commsim/core/fleet.py` generates from a single YAML: the per-node mesh IP/MAC, hwsim radio assignments, every `zenohd` config (explicit `connect` to all peers, scouting off, `listen` on the mesh IP only, **SHM off** because namespaces share `/dev/shm`), and the Cyclone peer lists.
- **Node = namespace(s).**
  - Default ("compact") mode: one namespace per robot holds two hwsim radios (2.4 GHz / 5 GHz) on `bat0`, plus `zenohd` and the stubs.
  - "Faithful" mode (small n only): a second namespace stands in for the Jetson. It connects by veth to the router namespace, where `bat0` is bridged, so batman-adv's translation table is exercised.
  - The gateway is a robot-shaped node with a raised antenna (a height term in propagation), plus `gateway_stub` (task injection, GUI view, 1 Hz status sink, chrony source role).
- **Stubs** (`commsim/stubs/`):
  - `robot_stub.py` publishes every message type in §2 of the prompt at configured rates and sizes.
  - It runs `core/protocol.py`: task FSM, auction, tie-break, lease renewal through Status, lease expiry + reclaim margin, isolated-robot behavior, hold-safe timeout.
  - Mobility comes from `core/mobility.py`: robots follow task paths on a 2D grid layout; pose = ground truth + configurable noise and drift.
  - No SLAM, Nav2 or perception.
- **Topics:**
  - `fleet/status/<cell>`: subscriptions update as a robot moves (own cell + neighbor cells + cells on its path).
  - Also `fleet/tasks/…`, `fleet/obstacles`, `fleet/map/<region>`, `fleet/emergency`, `fleet/link_health`.
- **Variants:**
  - V1: separate status/position/path/obstacle messages to all robots.
  - V2: bundled status.
  - V3: neighbor-cell status + auctions limited to robots near the pickup.
  - V4: second hwsim radio added to `bat0`.
  - V5: `beacon` sidecar sends Status as a one-hop raw broadcast on the mesh interface, outside batman-adv.
  - V6: `rbcast` sidecar for obstacles (sequence numbers + NACK repair).
  - Middleware check: V3 on `rmw_cyclonedds_cpp` with fixed peers.
  - Phase 0: one AP namespace, robots as stations, no batman-adv.
- **Propagation** (`core/propagation.py`, shared by emulation and DES):
  - Log-distance path loss per band, plus per-crossing attenuation for racks (strong) and wire shelving (mild), found by ray-segment intersection against the layout polygons.
  - The emulation updates the wmediumd SNR matrix as robots move (period is a placeholder, e.g. 200 ms).
- **Background (campus) interference:**
  - Emulation: one "interferer" hwsim node per band and neighborhood transmits broadcast frames at a duty cycle equal to the configured busy share (0/15/30%). An optional congested 2.4 GHz channel profile adds more.
  - DES: the same share is removed from available airtime.
- **Busy-time measurement:**
  - hwsim's `survey dump` busy counters are not realistic. Instead, a monitor interface per band (`hwsim0`-style) captures every frame, and airtime is computed from length, rate and preamble.
  - Busy time = summed airtime / wall time, per band, per neighborhood (neighborhood = nodes within carrier-sense range of a reference node).
- **Failure injector** (`emu/inject.py`):
  - Link block via `iw ... station set <MAC> plink_action block`, per band.
  - Degrade via a wmediumd SNR ramp; flap via a toggle schedule.
  - Kill or restart a namespace's processes; disable all 2.4 GHz radios; partition via SNR matrix cuts.
  - Gateway kill; clock offset via app-level clock skew in the stubs (the namespaces share the host clock).
- **Collectors:**
  - Stub-side per-message send/recv logs (latency uses the shared host clock, which is exact in emulation).
  - `batctl o/tr` polling (route changes, hop count).
  - A 10 ms UDP probe stream per watched pair (reroute time = gap from injection to first delivery on the new path).
  - Zenoh admin space (sessions, drops) and queue depth.
  - Host CPU guard: a run is flagged invalid if VM CPU > 80% (placeholder) or steal time is high.

## 3. Network isolation design

- **Emulation:** all fleet traffic stays on hwsim radios and veths inside the VM. The VM's NAT NIC exists only for apt/git. Zenoh configs are generated with `listen`/`connect` limited to the fleet subnet and scouting off.
- **Hardware testbed:**
  - Our own routers, our own mesh ID, our own static subnet; the gateway laptop is the chrony source.
  - The dev fallback network is our own AP, never eduroam.
  - If the laptop is also on eduroam, `zenohd` listens on the mesh Ethernet IP only.
- **`commsim/scripts/check_isolation.sh`** (runs in both settings, exits nonzero on violation). It checks that:
  1. The generated configs contain no endpoint outside the fleet subnet and scouting is disabled.
  2. `ss -tlnp` shows `zenohd` bound only to the mesh IP or localhost.
  3. Established connections on 7447 have only fleet-subnet peers.
  4. The chrony source is the gateway.
  5. There are no DHCP/DNS dependencies in the generated network config.
  - Optional nftables rule on the laptop: drop 7447 on every interface except the mesh one.

## 4. Parameter table (abridged; the full table goes in `commsim/configs/base.yaml` comments + `commsim/docs/parameters.md`)

| Parameter | Default | Source | Replaced by |
|---|---|---|---|
| Path-loss exponent / ref loss, per band | ITU-R P.1238 indoor coefficients | literature | Lab signal-vs-distance per band |
| Loaded-rack attenuation per crossing | **TBD, must be cited** | literature (to source) | Lab mock-up measurement where possible |
| Wire-shelving attenuation | placeholder | placeholder | Lab mock-up measurement |
| Loss vs SNR curve | wmediumd default error model | literature | Lab packet-loss measurement |
| Link rate / MCS | 802.11 rate control default | placeholder | Lab throughput measurement |
| Multicast rate | 12 Mbps | design | — |
| batman OGM interval | 500 ms | design | — |
| Reroute time | emerges in emulation | measured (sim) | Lab link-kill measurement |
| Background busy share, per band | 0 / 15 / 30 % sweep | placeholder | `iw dev <if> survey dump` in lab |
| Zenoh transport lease | placeholder (between reroute and 3 s) | placeholder | tuned from reroute measurement |
| Status timeout | ~3 s | design | — |
| Task lease / reclaim margin | 10 s / placeholder | design / placeholder | — |
| Status rate (neighbors / gateway) | 3 Hz / 1 Hz | placeholder | — |
| Bidding window / bid radius | 1 s / placeholder | design / placeholder | — |
| Cell size, region size | placeholder | placeholder | — |
| Map update rate cap | 100 kbps/robot, 2–5 s | placeholder | — |
| Message sizes | from prompt table | estimate | serialized size of the `.msg` types, measured in M1 |
| Stack processing delay | measured on x86 VM | measured (sim) | Optional: Jetson Orin Nano measurement |
| Pose noise / drift | placeholder | placeholder | SLAM choice (open) |
| Staleness threshold | 1 s | placeholder | — |
| Task arrival rate / burstiness | placeholder | placeholder | team workload estimate |

## 5. Experiment matrix (compute estimates are estimates)

- **Run shape:** warm-up starts after all batman originators are visible and all Zenoh sessions are up, then 60 s more; 300 s of steady state after that; 3 seeds per point (5 for failure scenarios).
- **DES (full grid):**
  - Variants: 8.
  - Fleet sizes: n ∈ {2, 5, 10, 15, 20, 25, 30, 40, … until failure}.
  - Background load: {0, 15, 30}%.
  - Layouts: lab and warehouse.
  - Seeds: 3.
  - Size: roughly 8 × 9 × 3 × 2 × 3 ≈ 1300 runs. At under a minute each, that's ≤ 1 day, and it parallelizes across 20 cores.
- **Emulation (subset; real time, ~7 min per run including setup):**
  - Scale: Phase 0, V1, V2, V3, V4 (and V5/V6 once built) × n ∈ {2, 5, 8, 10, 12, 15 … up to the VM cap} × background {0, 30}% × 3 seeds, warehouse layout. About 180 runs ≈ 21 h (overnight batches).
  - Lab layout at n = 2–3 for hardware comparison.
- **Failure scenarios:**
  - Emulation: all 12 scenarios on V4 at n = 5 and 10, × 5 seeds ≈ 120 runs ≈ 10 h.
  - Band loss also on V3 (no second band; shows the outage).
  - Partition and timer misordering also on V3.
  - Clock drift and burst load also at the largest passing n.
  - The DES repeats all scenarios up to the DES failure point.

## 6. Failure criteria and metrics (proposed thresholds)

Failure is the smallest n where any of these holds in steady state:
- Busy time > 50% on any band in any neighborhood.
- Control-message latency p95 > 150 ms or p99 > 500 ms. Control messages are status, bid, claim and obstacle; up to 3 hops; reroute windows are excluded. Every run reports sample counts, and a p99 with fewer than 1000 samples is marked "insufficient".
- **Status delivery ratio** < 95% fleet-wide, or < 85% for any single subscribed pair over 10 s windows (proposed).
- Any false "robot silent" decision.
- Any duplicate ownership persisting past lease + margin, or any abandoned task.
- **Auction completion** p95 > bidding window + 0.5 s (1.5 s), or max > 3 s (proposed).
- **Map divergence:** for each region, a robot's copy is compared with the latest version published by the reporting robot. Two numbers are tracked: *version lag* and *cell disagreement %*.
  - Failure if p95 version lag > 2 refresh periods, or cell disagreement > 1% 5 s after a change (proposed).
- Staleness flags (planning view of a peer older than the threshold) are reported, but are not a failure criterion.
- Each run records **which criterion tripped first** (that names the bottleneck).
- Logged metrics follow §9 of the prompt. Format: Parquet per run, plus `run.json` (run ID = config hash + seed + timestamp, full resolved config, git SHA, host CPU stats).

## 7. Validation plan

- **Emulation ↔ DES at n = 2–12:**
  - Compare busy time per band, latency CDFs (KS distance), delivery ratio, reroute time, and auction duration.
  - Calibrate DES contention, processing delay and Zenoh batching against the emulation. Success means busy time within ±10% and p95 latency within ±20% (proposed), checked on held-out runs (n values not used for calibration).
  - Above the emulation cap, the DES carries the sweep; the report marks results there as "DES-only".
- **Simulation ↔ lab at n = 2–3:**
  - Run the lab-layout scenario in both. Compare RSSI vs distance, loss, throughput, reroute time after a link kill, and busy time from `survey dump`.
  - Measured values replace the §4 placeholders, and the scale sweep is re-run.
- **Expected-shape check:** fit busy-time vs n per variant to a·n² + b·n + c. V1 should be quadratic-dominated and V3+ linear. The old hand estimates (V1 ≈ 11, V2 ≈ 19, V3 ≈ 14% at 20) are compared against, not used.
- **Offline unit tests** (system python3, no ROS) cover `core/protocol.py`:
  - tie-break, duplicate-claim resolution, lease expiry with the robot's own clock and skew, partition heal, rebid after no winner.
  - Plus a fleet-list → config generation golden test and DES sanity tests (single link, airtime arithmetic).

## 8. Repository layout, config format, automation (described, not written yet)

```
commsim/
  README.md                 # what it is, VM setup, run commands; states it shares nothing with the Isaac track
  vm/provision_vm.sh        # inside the 24.04 VM: ros-jazzy-ros-base, rmw-zenoh-cpp, rmw-cyclonedds-cpp,
                            #   mininet-wifi (pinned commit), wmediumd, batctl, iw, wpa_supplicant, python deps
  ws/src/commsim_msgs/      # ament .msg package: Header(version u8, robot_id u8, seq u32, stamp), TaskAnnounce,
                            #   Bid, Claim, Complete, Release, TaskCancel, Status, ObstacleEvent, MapUpdate,
                            #   MapResync, Emergency, LinkHealth (colcon workspace scoped to commsim only)
  core/                     # pure python, no ROS: fleet.py, protocol.py, workload.py, mobility.py,
                            #   propagation.py, layouts.py, metrics_schema.py
  stubs/                    # rclpy: robot_stub.py, gateway_stub.py, beacon.py (V5), rbcast.py (V6)
  emu/                      # topology.py (Mininet-WiFi build), wmediumd_ctl.py, inject.py, collect/
  des/                      # SimPy model: medium.py, batman.py, zenoh.py, sim.py
  configs/base.yaml, variants/*.yaml, layouts/{lab_mockup,warehouse}.yaml,
          scenarios/*.yaml (failures), sweeps/*.yaml
  scripts/run_emu.sh, run_des.py, sweep.py, check_isolation.sh, analyze.py (plots + summary table)
  tests/                    # pytest, offline
  results/                  # gitignored
```

- Config: YAML layered as base → variant → layout → scenario → sweep, plus seed. `sweep.py` expands the matrix and runs it resumably (it skips run IDs that already have results).
- Outputs:
  - Busy time & latency vs n per variant, with the failure point and the first-tripped criterion marked.
  - Recovery-time charts per scenario.
  - A summary table (scenario × variant × n: recovery time, duplicate/abandoned counts, pass/fail).
- Isolation from the Isaac track:
  - Nothing outside `commsim/` changes, except `.gitignore` (results) and optionally a CI job running `commsim/tests` offline.
  - Nothing in `commsim/` sources the host ROS.
  - Sweeps are never run while an Isaac session is up; `scripts/sim_ctl.sh status` is checked first because of RAM contention.

## 9. Milestones (two semesters)

**Semester 1**
- **M0 (wk 1–2):**
  - Provision the VM.
  - 2 namespaces, 802.11s + batman-adv over hwsim, ping across.
  - `check_isolation.sh`.
- **M1 — first real number (wk 3–4):**
  - 3 nodes (2 robots + gateway), single band, `zenohd` + `robot_stub` publishing Status at 3 Hz.
  - Output: measured p50/p95 latency, busy time from airtime capture, and serialized message sizes.
  - Also measures RAM/CPU per emulated robot, which sets the VM cap.
- **M2:**
  - `core/protocol.py` + unit tests.
  - Full stub message set.
  - Phase 0 and Phase 1 (V1–V3) in emulation; lab + warehouse layouts.
  - Literature pass for cited rack attenuation.
- **M3:**
  - Failure injector + first 6 scenarios on V3.
  - DES v1, calibrated against M2 runs.
  - First scale curves for V1–V3 + Phase 0.

**Semester 2**
- **M4:** Phase 2 (V4, dual band); band-loss, partition, timer-misordering scenarios; Cyclone middleware check.
- **M5:** Lab testbed calibration (signal/loss/throughput/reroute/survey) → parameter table updated; re-run sweeps.
- **M6:** Phase 3 (V5 beacon) and Phase 4 (V6 reliable broadcast); remaining scenarios.
- **M7:** Full DES sweep, cross-validation report, final charts/tables.

## 10. Risks and mitigations

- **VM resource cap (15 GB host):**
  - The per-robot cost of `zenohd` + rclpy stub + wpa_supplicant is unknown; M1 measures it. Estimated cap: ~10–15 robots.
  - Mitigations: compact mode, a minimal stub process, giving the VM most of the RAM while Isaac is not running; the DES covers larger n.
- **Real-time distortion under CPU load:** the CPU guard invalidates runs; fewer seeds run in parallel.
- **wmediumd realism** (hidden terminals, capture effect, rate control):
  - Cross-check against the DES and lab data; report emulation results as "emulated", not "measured".
- **Per-band SNR in wmediumd with two radios per node:** verify in M0 that the SNR matrix can be set per interface (per band). Fallback: two separate wmediumd media (one per band).
- **rmw_zenoh in namespaces:**
  - Each namespace has its own loopback, so `localhost:7447` resolves per robot — but `/dev/shm` is shared, so SHM is disabled.
  - `ZENOH_ROUTER_CONFIG_URI` is set per namespace. Verified in M1.
- **SAE cost at scale on hwsim:** measure; if it dominates CPU, run large-n points without SAE, quantify the difference at small n, and report it.
- **Mininet-WiFi maintenance on 24.04 / Python 3.12:** pin a commit. Fallback: our own thin namespace + hwsim + wmediumd scripts (we use only a small part of Mininet-WiFi).
- **Shared host clock hides clock-sync problems:** clock drift is injected at the app level; on hardware, latency is corrected by chrony offset estimates.
- **Unsourced attenuation numbers:** rack attenuation stays "TBD" until a citation is chosen; no invented value.

## 11. Open questions for the team

1. Router model: affects rates, antennas, OpenWrt version, real reroute behavior.
2. Lab mock-up dimensions and shelving positions; target warehouse layout (aisle width, rack height and load).
3. Channels: 2.4 GHz channel choice; 5 GHz non-DFS vs DFS.
4. Values for: cell size, region size, bid radius, status timeout (exact), hold-safe timeout, reclaim margin, staleness threshold.
5. Task arrival rate and burstiness for a realistic shift.
6. Acceptance of the proposed thresholds in §6 (delivery 95%/85%, auction 1.5 s/3 s, map divergence metric).
7. Do you want the optional Jetson Orin Nano stack-delay measurement (requires Jetson access)? Without it, x86 VM processing delay is used.
8. VM sizing: how many cores/GB can be dedicated (and confirm sweeps never overlap with Isaac runs).
9. Lab testbed availability and schedule for calibration (M5).
10. SLAM choice: affects the pose noise/drift model only.
11. **Partition semantics (found while building `core/protocol.py`).**
    - The spec's owner rule ("isolated owner stops before its lease ends") only covers an owner that hears *nobody*.
    - In a partition, the owner still hears its own side, so it never stops. Meanwhile the other side reclaims after lease + margin, which means duplicate work until heal, when the tie-break resolves it.
    - Is that acceptable, given the §8 pass condition says both "no duplicate ownership" and "duplicates resolved by tie-break on heal"? If not, the options are:
      - (a) the owner stops when ANY known peer is unheard for the lease. Safe, but one dead robot makes every owner stop.
      - (b) majority quorum: only the side holding a majority of the fleet may reclaim.

## Verification (of the implementation, once approved)

- `/usr/bin/python3 -m pytest commsim/tests` passes offline on the host.
- M0: `check_isolation.sh` exits 0; `batctl o` in each namespace lists all originators.
- M1: `run_emu.sh configs/sweeps/m1.yaml` produces Parquet + `run.json` with nonzero latency samples and busy time per band; `analyze.py` renders the first latency CDF.
- From M3 on: DES vs emulation calibration report within the §7 tolerances at overlapping n.

## Status

This plan was written before any work started; what was actually built and
measured, and where it departed from the plan, is in
[`sim_log.md`](sim_log.md) (Results at the top). Notable departures: the
emulation drives `mac80211_hwsim` + `wmediumd` directly instead of through
Mininet-WiFi; results are CSV/JSON rather than Parquet; the scale model is a
custom discrete-event simulator in plain Python.
