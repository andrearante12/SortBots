# commsim — fleet communication simulation

**What:** tests the SortBots fleet's communication design before any radios
are bought: a self-healing Wi-Fi mesh (802.11s links, batman-adv routing),
Zenoh middleware, and a decentralized auction + lease protocol that decides
which robot owns which task.

**Why:** the robots have no central coordinator, so correctness depends on
the network. The questions it answers: how many robots before the network
fails, whether the fleet recovers correctly from broken links, dead robots
and partitions, whether latency and timer requirements hold, and which
design choices matter most.

**Results:** see the **Results** section at the top of
[`docs/sim_log.md`](docs/sim_log.md). In short, the design as first written
failed at every fleet size; after the fixes it supports about 20 robots in
simulation and passes 110 of 120 failure scenarios.

**Separate from the Isaac Sim track.** No shared code, shell, ROS install or
launch files with `nodes/`, `launch/` or `scripts/`. The emulation runs in its
own Ubuntu 24.04 VM; the model and tests run on the host with system Python.

## How it works

Two tools, cross-checked against each other:

| | Emulation (`emu/`) | Discrete-event model (`des/`) |
|---|---|---|
| What runs | The **real** 802.11s, batman-adv, zenohd and ROS 2 nodes, one Linux network namespace per robot, over simulated radios (`mac80211_hwsim` + `wmediumd`) | A Python model of the radio channel (CSMA/CA), batman-style routing and Zenoh-over-TCP sessions, with the **real** task-allocation code (`core/protocol.py`) in every robot |
| Good for | Ground truth: latency, overhead per message, reroute time, bugs only the real stack shows | Scale and failure sweeps: 20+ robots, 11 failure scenarios, many seeds, seconds per run |
| Limit | One VM, about 20 nodes | Simplified radio (one contention domain per band) |
| Where it ran | inside the VM, as root | on the host |

The model is calibrated on the emulation's measurements
([`measurements/`](measurements/README.md)): median latency agrees
within 3–10%, channel load within ~6%, and both put the overload point in
the same place.

## Layout

| Path | What it is |
|---|---|
| `docs/brief.md` | The original brief: what the simulation had to answer |
| `docs/plan.md` | The plan written from the brief (toolchain, metrics, milestones) |
| `docs/sim_log.md` | **Results at the top**, then the full run log: what was done, why, what came out, including mistakes |
| `docs/figures/` | Charts used in the log (`*_fixed.png` = final design) |
| `core/protocol.py` | Task allocation: auction, tie-break, leases, isolation, lease beacon. Pure Python, shared by every tool |
| `core/fleet.py` | One fleet list → every node's IP, radio MACs, Zenoh and UDP config, plus the isolation check |
| `core/timers.py` | Refuses timer settings the measurements say will misfire |
| `core/loopback.py` | Tiny in-memory fleet for protocol unit tests |
| `des/sim.py` | The discrete-event model (radio, routing, sessions, robots, metrics) |
| `des/radio.py`, `des/layout.py` | 802.11 timing and propagation; placeholder warehouse floor |
| `des/run.py`, `des/scenarios.py` | Fleet-size sweeps; failure-scenario runs with pass/fail judging |
| `des/crosscheck.py` | Model vs emulation at the same setup |
| `des/summarize.py`, `des/scen_table.py`, `des/plots.py` | Tables and charts from results (`plots.py` needs matplotlib) |
| `emu/mesh.py`, `emu/m1.py`, `emu/reroute.py` | Build the emulated mesh; run Status stubs and capture; measure reroute time |
| `stubs/status_stub.py` | ROS 2 node run inside each emulated robot (Zenoh or UDP Status) |
| `ws/src/commsim_msgs/` | The message definitions (`.msg`), so wire sizes are real |
| `configs/` | `base.yaml` = the recommended design; `spec_original.yaml` = the design as first written, for before/after runs; `ogm250.yaml`/`ogm500.yaml` = routing-interval comparisons |
| `measurements/` | Emulation measurements the model is checked against |
| `scripts/` | `fix_sweep.sh` reproduces the final comparison; `check_isolation.sh` proves traffic stays on the fleet subnet |
| `vm/` | Creating and provisioning the emulation VM |
| `tests/` | Offline tests (protocol, config generation, model sanity) |

Every value in `configs/base.yaml` tagged `placeholder` or `estimate` is a
parameter awaiting a hardware measurement, not a fact.

## Run it

Offline tests (host, seconds, no ROS):

```bash
/usr/bin/python3 -m pytest commsim/tests
```

Discrete-event model (host):

```bash
/usr/bin/python3 -m commsim.des.run --variants V3 V4 --n 10 15 20 --seeds 1 2 3 --until-fail --out commsim/results/mine
/usr/bin/python3 -m commsim.des.summarize commsim/results/mine
/usr/bin/python3 -m commsim.des.scenarios --variants V4 --n 10 --seeds 1 --out commsim/results/scen
/usr/bin/python3 -m commsim.des.crosscheck          # model vs emulation, 3-node line
/usr/bin/python3 -m commsim.des.crosscheck n10      # model vs emulation, 10-node warehouse
commsim/scripts/fix_sweep.sh               # the full before/after comparison (~1 h)
```

Add `--configs commsim/configs/spec_original.yaml` to any run to see the
design as first written.

Emulation (inside the VM, as root; see [`vm/README.md`](vm/README.md)):

```bash
sudo python3 -m commsim.emu.mesh up --n 3 --line
sudo python3 -m commsim.emu.m1 --n 10 --layout warehouse --out /run/commsim/m1
sudo python3 -m commsim.emu.reroute --trials 6
sudo python3 -m commsim.emu.mesh down
```

`multipass exec` joins its arguments into one remote shell string, so put
multi-step VM commands in a script under the gitignored `commsim/results/`
(mounted in the VM) and run that.

Fleet configs and the isolation check:

```bash
/usr/bin/python3 -m commsim.core.fleet gen commsim/configs/base.yaml --out /tmp/fleet
commsim/scripts/check_isolation.sh --subnet 10.42.0.0/24 --configs /tmp/fleet
```

## Decisions and open items

- **Partitions:** an owner that still hears its own half of a split network
  keeps working, so both halves may work the same task until the split heals;
  the tie-break then picks one owner in about 2 s. Accepted by design
  (`tests/protocol_test.py::test_partition_heal_converges_by_tie_break`).
- **Status recipients over UDP** are currently every robot (right for small
  fleets). Neighbor-only Status needs a small "which areas I listen to"
  announcement before scaling past ~10 robots.
- **Not yet measured on hardware:** link rates, rack attenuation, reroute
  time on real radios. The 2-robot testbed replaces those placeholders.
