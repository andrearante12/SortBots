# fleetsim — the headless fleet sim

Many robots, no Isaac, no ROS, no GPU. Each robot plans on **its own map**,
shares what it learns over a simulated network, and takes work through the
**real** fleet task auction. Runs at tens of times real time for a few
dozen robots, so comms and navigation protocols can be tested at fleet sizes
Isaac can't reach (2–4 robots on a typical machine). Isaac stays the place
to verify, calibrate and demo.

```bash
/usr/bin/python3 -m fleetsim list                       # the scenarios
/usr/bin/python3 -m fleetsim run delivery_smoke         # ~5 s
/usr/bin/python3 -m fleetsim run map_sharing --set map_share.enabled=false
/usr/bin/python3 -m fleetsim run fleet_scale --robots 30 --seed 2
/usr/bin/python3 -m fleetsim show walkers               # the fully resolved config
/usr/bin/python3 -m fleetsim rescore data/fleetsim/<dir>   # re-score with current metric code
/usr/bin/python3 -m fleetsim bench smoke                # a suite in parallel, one table (see Benchmarks)
/usr/bin/python3 -m pytest fleetsim/tests               # ~10 s
```

System python3 with numpy and PyYAML (`requirements-dev.txt`), from the repo
root. Results land in `data/fleetsim/<UTC>_<name>/` (gitignored):
`scenario.json` (exactly what ran), `events.jsonl.gz`, `summary.json`,
`score.json`, `system.jsonl` (the compute log; `crash.txt` after a crash). To watch a run, see [Watching a run](#watching-a-run). Exit code 0 = every pass criterion held, 1 = one failed,
2 = invalid scenario.

To run one without a terminal, use the operator view: Session panel, **Tests**
tab, **Run a fleet sim**. It offers the scenario, robots, duration, seed and a
fixed list of overrides (`webui/fleetsim_runner.py`, `SETTABLE`), needs the sim
console (`--control`), and runs one at a time. Any other `--set` is terminal only,
because the console is reachable from the internet and `world.map` takes a path.

## Where it sits

| | Isaac + ROS 2 | commsim DES / emulation | **fleetsim** |
|---|---|---|---|
| Robots | 2–4 | 20+ / ~20 | **50+** |
| Motion | physics, real Nav2 | points on aisle lines | grid A* on each robot's own map, local layer, bodies that collide |
| Perception | real D435 render, RTAB-Map, obstacle_detector | none | ray-cast depth cone on the true grid |
| Maps | per-robot RTAB-Map, fused centrally (`map_merge`) | message sizes only | **per-robot belief, shared as patches over the network** |
| Network | one DDS domain (free, perfect) | real CSMA/CA, TCP, zenoh (DES); real stack (emulation) | commsim's radio arithmetic per epoch, load factor, multi-hop routes |
| Task auction | `nodes/fleet_tasks.py` | `core/protocol.py` | `FleetTasks` from `nodes/fleet_tasks.py`, in-process |

**The point is decentralisation.** In Isaac every robot plans on one fused
`/map` that a central node assembles for free. Real robots won't have that:
the mesh bridge carries only `/fleet/status`, `/fleet/intent` and
`/fleet/tasks`. Here every robot plans only on what it has sensed itself
or heard from peers, and every byte of that travels the network model.

## What is real and what stands in

| Piece | fleetsim | Real thing |
|---|---|---|
| Task auction, leases, commit, quorum, confirmed picks and completes, retry, restart adoption | **real**: `FleetTasks` + `commsim/core/protocol.py`, imported | `nodes/fleet_tasks.py` |
| Bid cost | the bidder's own A* path on its own belief (`robot.PathOracle`), so bid and drive agree | `nodes/grid_dist.py` Dijkstra on fused `/map` |
| Task FSM | stand-in with task_manager's states, dwell (per robot type: pick and place times) and status JSON | `nodes/task_manager.py` |
| Global planner | A* on the robot's belief, Nav2-style lethal band + exponential inflation (`planner.py`) | Nav2 NavFn |
| Local layer | behind a seam, `nav.local` ([Local avoidance](#local-avoidance)). Default `stop_replan`: stop for what's ahead within `stop_distance_m`, replan around after `block_replan_s` (lower priority first), give up after `block_fail_s` (`local/stop_replan.py`). Or `orca` (`local/orca.py`) | Nav2 controller + costmaps |
| Perception | ray-cast cone, movers occlude and come back as detections with velocity, never mapped (`sensing.py`) | D435 + RTAB-Map + obstacle_detector + dynamic_obstacle_filter |
| Peer awareness | camera, `/fleet/status` over the network, obstacle notices; never ground truth | the same |
| Map sharing | tiles, deltas, sequence numbers, MapResync, round-robin refresh (`belief.py`, `sim.py`); optional tile subscriptions and event-driven publishes (`mapshare.py`, [below](#map-sharing-subscriptions-and-event-driven-updates)) | not implemented on the robots yet; this is the commsim brief's design |
| Network | ideal, or radio (`network.py`) | mesh + zenoh |
| Movers | simtest actors (inline in a scenario, or a `configs/worlds/` world) | scripted actors in Isaac |

## The decisions (2026-11-02) and what they mean in practice

1. **Resolution is configurable** (`world.cell_m`, default **0.2 m**). Issues:
   - Memory and A* time grow as 1/cell². At 0.2 m on the default 60 × 40 m
     floor that's 60 k cells and ~1.5 MB per robot. At 0.1 m it's 4×.
   - A robot is planned as a disc of `footprint_radius_m` + half a cell, so
     coarser cells cost clearance: 0.35 m at 0.2 m cells, 0.3 m at 0.1 m.
   - A saved 0.05 m map is max-pooled (any occupied fine cell → occupied),
     so walls thicken by up to a cell. The run reports in `notes` how much
     free floor that took when it's over 5 %.
   - Map traffic is priced per **m²** (`map_share.bytes_per_m2`), not per
     cell, so changing the resolution does not change radio load.
2. **Cell state** is trinary (unknown / free / occupied) plus the sim time it
   was last observed. Fusion is **newest observation wins**, not map_fuse's
   "occupied beats free", which in a map with no owner would keep a moved box
   in every map forever.
3. **Unknown is optimistic** (`nav.unknown_cost: 0`). A cautious policy is one
   number, not new code.
4. **Frames are aligned.** Every robot knows its true pose. Pose drift is
   a later, separate knob.
5. **Time:** messages are events at their exact arrival time, while motion,
   sensing and protocol ticks step every `sim.step_s` (0.1 s). Seeded and
   bit-for-bit reproducible (`test_a_seed_reproduces_a_run_exactly`).
6. **Network:** the commsim DES's radio maths per topology epoch. The
   docstring of `network.py` lists exactly what it simplifies and what that
   costs. Status is UDP, `/fleet/tasks` is reliable and in order per pair,
   map patches are best-effort.
7. **Capacity can be lifted** (`network.capacity`, radio model only).
   `enforced` (default) is the radio as modelled: past 95 % airtime the
   excess is lost or queued. `unlimited` asks what the protocol would do if
   the radio could carry everything offered: load is still measured
   (`comms.load_max` is then the offered load, and `over_cap_s` counts the
   seconds some node was past the cap) but costs no delay, loss or queueing.
   Path loss, frame errors, routing and failures still apply. Run a scenario
   both ways to split "the protocol is wrong" from "the radio can't carry it".
   The operator view's run form offers it as **Radio capacity**. It does not
   lift `map_share.rate_cap_bps`, which is the robots' own self-imposed
   budget rather than the radio's limit.

The body is a 0.25 m disc, half the length of the 0.5 × 0.4 m footprint.
The circumscribed 0.32 m disc made robots unable to pass each other in a
2.4 m aisle on a 0.2 m grid.

## Watching a run

The operator view replays any recorded run. Open the Session panel, go to the
**Tests** tab, and find the run under **Fleet sim runs**. Runs of a
`fleetsim bench` are under **Benchmarks** below them, a closed group per
bench; open one to list its runs. You can also go straight to one with
`http://<console>/#replay=<run dir>` (a bench run is
`#replay=bench/<bench dir>/<run dir>`). The replay covers the stage until you
close it. None of it is the live fleet.

- **Floor:** the true floor. Waypoints are drawn as tape, the gateway as a
  triangle, and a blockage as a red box while it is in place.
- **Robots:** to scale, coloured by state (to pickup, carrying, idle, off). A
  dashed red ring means the robot can't reach the gateway: over the radio
  links if any were recorded, otherwise through a partition or a cut link.
- **Paths:** every robot's planned path. The selected robot's path is drawn
  dashed.
- **Links:** one-hop radio links, darker for a stronger signal. Radio runs
  only.
- **Load:** colours each link and rings each robot by its channel's load
  instead. See [Comms traffic monitor](#comms-traffic-monitor).
- **Own map:** with a robot selected, its *own* map replaces the true floor.
  Cells where it disagrees with the truth at that moment are red, so a
  blockage it hasn't learned about shows up red. Only robots listed in
  `record.belief_maps` are recorded.
- **Timeline:** trouble is ticked along the top (contacts, blocks,
  failures) and deliveries along the bottom. Disturbances are shaded bands.
  Click a tick to jump to it.
- **Info panel:** the task board, network loss and load, and the selected
  robot's task, how much of the floor it knows, how far it is behind the
  fleet's map, its own traffic, and its recent events.
- **Traffic:** the comms monitor, charts of load and traffic over the run
  ([Comms traffic monitor](#comms-traffic-monitor)).

Keys: Space plays or pauses. ←/→ steps 1 s (with Shift, 10 s). `[`/`]` jumps
to the previous or next event (only the selected robot's, if one is
selected). T shows or hides the comms monitor. Esc closes. Drag to pan,
scroll to zoom, double-click to fit.

**What is recorded for it.** These go in `record:` in `fleetsim/configs/base.yaml`.
Only the paths feed a metric (`utilisation`, see [Benchmark metrics](#benchmark-metrics)).
The run's floor, waypoints, homes and gateway are
recorded once (`world`). Paths are recorded on every plan and replan. Radio
links are recorded every `links_s`. `belief_map` snapshots are taken every
`belief_map_s` for the robots in `belief_maps` (default `[robot_0]`; `all`
records every robot). Each node's load and traffic go into every `net`
sample while `net_nodes` is on (the default).

`fleetsim/replay.py` packs a run into one JSON document. Poses are stored as
flat arrays, and every stream is a list of time-ordered rows, so the page
finds the state at any moment by bisection. A 5-minute run of `fleet_scale`
(50 robots, radio) packs in 0.2 s to 3.4 MB, or 620 kB once gzipped; 0.9 MB
of that is the per-node traffic (2.5 MB and 430 kB without it).

Runs recorded before 2026-11-03 have no `world` event, so they are listed but
can't be replayed. Re-running the same scenario and seed reproduces them
exactly.

## Scenarios

`fleetsim/scenarios/*.yaml`. The schema is in `fleetsim/config.py`. Every
knob is in `fleetsim/configs/base.yaml`, with its source or tagged PLACEHOLDER.
Numbers below are seed 1 on an i9-14900HX laptop (single-threaded: fleetsim
uses one core).

| Scenario | Question | Seed-1 result |
|---|---|---|
| `delivery_smoke` | 8 robots, 12 tasks, ideal network: does it work at all? | 12/12 done, 0 contacts, 0 duplicates, ~80× real time |
| `map_sharing` | A pallet blocks an aisle; does sharing save the others finding it? | **on:** 0 stale cells, mean delivery 98 s · **off:** 147 stale cells, 107 s |
| `partition_heal` | 4 of 10 robots cut off for 60 s | all delivered, 0 duplicates |
| `walkers` | #18: walkers stepping out of rack aisles | robot-driven contacts 0–1 per run; **notices made no measurable difference** (seeds 1–4) |
| `fleet_scale` | 50 robots, radio, neighbour-only Status | 108/140 done, 9× real time; **fails** its criteria (finding 4) |
| `fleet_scale` + `--set status.recipients=all --set status.hz=5 --set status.session_liveness=false` | the same with today's fleet-wide Status | **0/140 done**: `/fleet/tasks` delivery 0.01 %, mean load 324 % |
| the same + `--set network.capacity=unlimited` | is that the radio, or the protocol? | 117/140 done, every kind delivered >99 %, offered load 265 % mean / 511 % peak: the radio. **2 duplicates** with near-perfect delivery, so not only the radio |

## Findings so far

These came out of building this, and each one is worth a decision.

1. **A pick inside the lease window sent a second robot. Fixed 2026-11-17**
   (confirmed picks, `commsim/core/protocol.py`). An owner cut off a few
   seconds before the shelf still "had a quorum" for `status_timeout_s`,
   picked, and became uncatchable: a committed task never yields, and the
   majority never heard the commit. Now the owner announces the pick and
   waits at the shelf (`AWAIT_PICK`) until a majority acks. Cut off, it
   gets no acks and gives the task up at its lease with the item still on
   the shelf (`test_no_pick_without_the_majority_knowing`; the old gap is
   kept as a control with the switch off).
2. **A completion lost in a split was never repeated. Fixed 2026-11-17**
   (confirmed completes). A Complete is now acked, and repeated to the
   peers that haven't acked until every peer the owner hears has. A node
   that knows how a task ended answers anyone still talking about it, and
   a node whose committed owner is missing asks with a `TaskQuery`
   (`test_a_completion_made_during_a_split_reaches_everyone`).

   What the two did at 50 robots (`fleet_scale`, seed 1, switches off →
   on): 108 → **122** delivered, duplicate-work episodes 17 → **2**,
   empty-shelf visits 8 → **0**, `/fleet/tasks` delivery 74 % → **97 %**,
   and less traffic (10.1 → 8.8 kB per robot per second), because the
   fleet stops redoing work. Small scenarios are unchanged. Two first cuts
   made it much worse, and are now protocol rules ("Addressing"): acks and
   repeats sent fleet-wide (n² copies per pick), and TaskQueries every 10 s
   answered fleet-wide, which is n³ exactly when the radio is full, since
   lost beacons are what make owners look missing. That run fell to 19
   deliveries.
3. **Fleet-wide Status does not scale.** At 50 robots, all-to-all Status
   at 5 Hz is 12,500 unicast copies a second. The radio model puts mean load
   at 324 %, and the fleet delivers nothing (`fleet_scale` with fleet-wide
   Status: 0/140). That matches the DES's ~20-robot ceiling. Scale needs
   neighbour-only Status, and then the quorum needs zenoh-session liveness
   (`status.session_liveness`), which **`nodes/fleet_tasks.py` doesn't do
   today**. With Status neighbour-only, a robot never hears a majority's
   Status and can't claim anything.
4. **Even neighbour-only, 50 robots saturate where they bunch up.** With
   Status at 3 Hz within 12 m, `fleet_scale` still peaks at ~300 % load on
   the apron where idle robots park, and the lease beacons are still
   fleet-wide (n² copies a second). Reliable `/fleet/tasks` delivery falls to
   74 % (p95 4 s), and the protocol responds with 61 cancellations, 17
   duplicate-work episodes and 8 empty-shelf visits. Next levers to try: lease
   beacons to the gateway plus neighbours only, sparser parking, and the
   DES's dual-band variant.
5. **Parked robots wall off docks.** At 1.2 m spacing, an idle fleet on the
   apron blocked the route to the docks. Homes are now 1.6 m apart.
   Decentralised parking is itself a protocol question.
6. **Dead-end aisles deadlock** when many robots share one pickup behind a
   blockage. There's no traffic coordination beyond "lower priority replans
   first". Path intents with yielding are now built (see "Path intents")
   but, as measured, don't pay yet.

## Path intents (2026-11-17): built, measured, off by default

`intent.enabled` (`fleetsim/intent.py`). Each robot announces the next 20 s
of its path as up to 8 timestamped points. Every 0.25 s it checks its own
predicted path against the intents of peers that outrank it: carrying beats
fetching beats going home, then time already spent yielding, then robot
index. If they would be within 0.9 m of each other at the same moment, and
there is a spot on its own path that keeps 1.1 m from the peer's whole
intent, and the wait there is under 8 s, it holds at that spot. Anything
else is left to the local layer. The points ride on `/fleet/status` once
per path change, hold or second (`intent.transport: status`).

What it did, seed 1, off → on:

| Scenario | Robot-caused stops | Seconds stopped | Yields | Done | Mean delivery |
|---|---|---|---|---|---|
| `delivery_smoke` | 44 → 41 | 36 → 31 | 2 (6 s) | 12 → 12 | 217 → 217 s |
| `walkers` | 186 → 218 | 179 → 224 | 6 (23 s) | 23 → 23 | 161 → 165 s |
| `fleet_scale` (radio) | 3778 → 3397 | 3956 → 3791 | 223 (690 s) | 122 → 121 | 202 → 210 s |

So as built it doesn't pay. Why, from counting every check (scratch
instrumentation, `delivery_smoke`): most predicted conflicts with a moving
peer are head-on meetings along one aisle, where the only spot clear of
the peer is back at the aisle mouth and the wait is the peer's whole
passage. Those are rightly left to the local layer's swerve, since a
2.4 m aisle lets two robots pass. A third to half of robot-caused stops
are for a peer that is standing still (blocked itself, parked, picking),
which intents don't cover. And a robot holding still is itself something
the others then stop for, which is why `walkers` got worse.

Two things learnt on the way, both now the default:
- **Intents ride on Status.** As their own message to robots within 20 m
  they took the 50-robot radio down (194 MB of intents; Status was lost, so
  every peer was then reported as an unknown mover, and obstacle notices
  went from 65 k to 1.3 M copies): 0 deliveries. Airtime is per frame far
  more than per byte.
- **Carried once per change, not on every Status:** on every 3 Hz Status
  the points added 40 % to the bytes; once per change, 2 %.

Next steps that would address what the stops actually are: lane discipline
(keep right of the aisle centre when travelling against a peer's intent,
so head-on meetings pass without stopping), and detours that treat a
higher-priority peer's intent as a temporary cost on the costmap (another
aisle, rather than waiting).

## Benchmarks and crash logs

`fleetsim bench` runs a suite: scenarios x robot counts x seeds x variants,
each run in its own process, many at once, and prints one table per metric.
"Many" is at most 12 by default (`--jobs` overrides it): 24-28 at once took
this laptop's CPU package to 98 degC, and heat is what speeds up the CPU fault
the crashes point at (Known issues).

```bash
/usr/bin/python3 -m fleetsim bench --list
/usr/bin/python3 -m fleetsim bench smoke                  # 36 runs, ~30 s on the 30-CPU laptop
/usr/bin/python3 -m fleetsim bench smoke --dry-run        # the runs and their commands
/usr/bin/python3 -m fleetsim bench smoke --only 'walkers_*' --jobs 8
/usr/bin/python3 -m fleetsim bench --rescore data/fleetsim/bench/<dir>   # current metric code
```

**Suites** are `fleetsim/suites/<name>.yaml`. The full schema is in the
docstring of `fleetsim/bench.py`:

```yaml
name: smoke
scenarios:                       # each entry at each of its robot counts
  - {scenario: delivery_smoke, robots: [2, 4]}
  - {scenario: walkers, robots: [5], duration_s: 240}
seeds: [1, 2, 3]
variants:                        # name -> overrides, the same syntax as --set
  default: {}
  intents: {intent.enabled: true}
headline: fleet_tasks.per_hour   # <metric id>.<value>
also: [fleet_tasks.mean_s, nav.blocked_robot_s]
```

An entry can also take `set:` (its own overrides, applied before the
variant's) and `id:` (the row label, needed when a scenario appears twice).
The bench resolves every scenario x count x variant before it starts, so a
bad knob or a misspelt metric fails at once. Metric values are read from
each run's `score.json` without a list of known names, so a new metric or
knob needs no change in the bench. The headline's metric must be in every
scenario's `metrics:`. A scenario without an `also` metric shows `-`.

**Output** goes to `data/fleetsim/bench/<UTC>_<suite>/` (gitignored):

| File | What |
|---|---|
| `table.md` | the printed tables: rows scenario x robots, columns variant, cells mean ± sd over seeds; `(2/3)` = a value from 2 of 3 seeds |
| `results.csv` | one row per run: scenario, robots, seed, variant, status, attempts, crashes and their cores, wall_s, sim_s, then every metric value as `<metric id>.<value>` |
| `summary.json` | the suite, the host (CPU, microcode, kernel, python, numpy, offline CPUs), every run with each attempt, the tables, crash counts. Rewritten as runs finish, so a bench cut short keeps what it finished |
| `<id>_r<N>_s<seed>_<variant>/` | one run: what `fleetsim run` writes, plus `log.txt`; a crashed attempt is moved into `attempt_<k>/` before the re-run |

The operator view lists bench runs under **Benchmarks** in its replay list
(Tests tab), a closed group per bench dir, newest first. Open a group to
list its runs in the order the bench planned them, with each run's variant
and seed. They replay like any run ([Watching a run](#watching-a-run)).

**What counts as done.** A run is complete when it exits 0 (every criterion
held) or 1 (one failed) and wrote `score.json`. Exit 2 is an invalid
scenario: it is reported and not retried. Anything else is a crash: a
signal, exit 1 without `score.json` (a Python exception), or `--timeout`. A
crash is re-run up to `--retries` times (default 2). Runs are seeded and
deterministic, so a re-run is the same run. Ctrl-C stops the bench, lets the
running runs close their recordings, and writes the tables for what finished
(exit 130). The bench exits 0 when every run completed, 1 otherwise.

**The compute log.** Every `fleetsim run` writes two more files into its
own dir (`fleetsim/sysmon.py`):

- `system.jsonl`: a sampler thread writes one JSON line every 2 s, and
  flushes and fsyncs each line, so the log survives a segfault or a hard
  lock-up. The first line holds the pid, the sampler's thread id, python and
  numpy versions, kernel, CPU model, microcode and the offline CPUs. Each
  sample holds wall and sim time, the run's phase, the CPU the main thread
  is on and its physical core, that CPU's frequency, the package, own-core
  and hottest-core temperatures (hwmon `coretemp`, else thermal zones), the
  load average, RSS and CPU time. An `end` line is written on a clean exit
  or a Python exception, so a run killed by a signal has none. Any probe
  that can't be read is `null`. The cost is too small to measure (6-robot
  radio run: 8.4-8.8 s without, 8.5-8.6 s with). The event stream is
  bit-identical with and without it.
- `crash.txt`: `faulthandler`'s stack of every thread on a fatal signal, or
  the traceback of an uncaught exception. It exists only after a crash.

**Which core a crash died on.** For a death by signal, the bench saves the
kernel's log lines for that pid in the run's time window to
`attempt_<k>/kernel.txt` (`journalctl -k`, else `dmesg`). Machine-check
lines are saved too. On this kernel a segfault line names the CPU: "likely
on CPU 10 (core 20, socket 0)". **But the kernel logs only faults that no
handler caught, and `faulthandler` catches them** (checked 2026-11-26), so
with it on there is no segfault line. The bench therefore reads the dead
process's `/proc/<pid>/stat` before it reaps it. Field 39 there is the CPU
the process died on, and it matched the kernel's line in 4 of 4 deliberate
faults. The last sample in `system.jsonl` is the fallback, but it can be up
to 2 s old (one test run moved from CPU 18 to CPU 8 in 0.4 s). In
`results.csv` such a CPU is marked `?`. `bench --no-faulthandler` gives up
the Python stack to get the kernel's line back, with the faulting
instruction's address. The printed summary and `summary.json` (`crashes`)
count crashed attempts by kind and by CPU and core, and how many runs
recovered on which attempt. Crashes that cluster on one core point at the
CPU.

## Benchmark metrics

The metrics the benchmark compares methods by ([plan](fleetsim-plan.md),
item A4). Throughput comes first, delivery time
second, and both are measured from the operator's request (the `announce`).
Throughput only separates methods when the fleet can't keep up, so `bounds`
says how loaded a scenario is. Every scenario lists `fleet_tasks`, `bounds`
and `utilisation`, with no pass criteria on the new values yet.

**`fleet_tasks` v3.** The v2 values keep their meaning. New values:

- `wait_assign_s`: announce to the task's first dispatch.
- `wait_pickup_s`: first dispatch to the item leaving the shelf (the first
  `NAV_TO_DROPOFF`). This includes time in a busy robot's queue and the empty
  drive to the shelf.
- `backlog_end`: announced tasks not delivered when the run stopped.

Both waits are means over delivered tasks, so `wait_assign_s +
wait_pickup_s` + the carry leg = `mean_s`.

`per_hour` divides by the time from the first announce to the end of the work.
The end of the work is the last DONE if every announced task was delivered,
else the end of the run. v2 divided by the whole run, so the idle tail after
the last delivery counted as time the fleet failed to deliver. The same
`delivery_smoke` fleet scored 112/h when stopped at `tasks_done`, and would
score 48/h if left to run its 900 s. v3 gives 113/h either way. Work left
undone still costs until the run stops, so a fleet that deadlocks early gets no
reward for it.

**`bounds` v1**, fed by one new event per announced task. The sim emits
`task_bound {task_id, lb_s, drive_m}` straight after the `announce`. `lb_s` is
the shortest drive pickup → dropoff at `robots.max_speed_mps`, plus both
`tasks.dwell_s`. The drive is A* on the base layout (`Sim._task_bound`) with
the robots' planning radius, by length only (no inflation cost), then pulled
taut, because the robots drive a smoothed path. Blockages are ignored: they
only make a drive longer. Lengths are cached per (pickup, dropoff). A new pair
costs 7 to 13 ms, and `fleet_scale` took 108.4 s of wall time instead of 108.3 s.
The values:

- `lb_mean_s`: the mean bound over announced tasks.
- `stretch`: the mean over delivered tasks of delivery time / its bound.
- `ub_per_hour`: robots × 3600 / `lb_mean_s`, what the fleet could deliver if
  every robot did nothing but back-to-back bound-length tasks. It ignores the
  empty drive between tasks, so it is optimistic.
- `arrival_per_hour`: (announced − 1) × 3600 / (last announce − first
  announce).
- `offered_load`: `arrival_per_hour / ub_per_hour`. Above 1, no method can keep
  up. Because `ub_per_hour` is optimistic, a fleet can also be overloaded below
  1.

For a fixed task list, `arrival_per_hour` is the rate of the burst, not a
steady rate.

**`utilisation` v1** books every robot-second to one phase: idle, to pickup,
at pickup (`AWAIT_PICK` + `PICKING`), carrying, placing, going home, or off
(killed). It reads the `task` states, and the `path` events with purpose
`home`. It also gives time lost by cause, in robot-seconds and as a share of
robot-time: stopped for a peer robot, a mover, a notice or a bump (the
`blocked` event's `by`), yielding to an intent, awaiting the pick ack, and no
path. No path runs from `plan_failed` to the next recorded path, a `gave_up`,
or the task's next state. The rules:

- Lost time counts only while a robot works a task. A robot stopped on its way
  home delays no delivery. For this reason `lost_robot_s` is below
  `nav.blocked_robot_s` (`fleet_scale`: 3189 vs 3956 s).
- One cause at a time. Yielding wins over a stop, because a held robot isn't
  checked for blockers. A stop wins over no path, because a replan around a
  blocker that finds no way out is the blocker's doing.
- A stop is booked to what stopped the robot first.
- A change of task state closes an open stop and a planning failure. The
  robot's `_clear_goal` records no `unblocked`.
- With `record.paths: false`, going home and recovered plans can't be seen.
  `home_frac` and `lost_no_path_*` then come out None, and the time spent going
  home is in `idle_frac`.
- Recovery pauses (`nav.hiccup_rate_hz`, 0 until calibrated) aren't booked,
  because their length isn't in the event.

**`bounds` v2** (2026-11-26) adds robot types: a task's bound is the best
type that can do it, `ub_per_hour` is also capped per capability, and
`unservable` counts tasks no robot can do. Runs without types score as in v1
([Robot types and capabilities](#robot-types-and-capabilities)).

Runs recorded before 2026-11-26 rescore. They have no `task_bound`, so every
`bounds` value except `arrival_per_hour` comes out None. The new
`fleet_tasks` and `utilisation` values are computed from streams those runs
already have. Only the new event changes the event stream: every other event
of `delivery_smoke`, `walkers` and `fleet_scale` is unchanged, and
`test_task_bounds_are_the_only_change_to_a_run` checks this.

Seed 1, 2026-11-26:

| | `delivery_smoke` | `walkers` | `fleet_scale` |
|---|---|---|---|
| `per_hour` / `arrival_per_hour` | 113 / 1165 (a burst) | 150 / 277 | 498.5 / 675 |
| `mean_s` = assign + pickup + carry | 217 = 2.9 + 136 + 78 | 161 = 2.3 + 85 + 74 | 203 = 3.1 + 83 + 116 |
| `backlog_end` | 0 | 11 | 18 |
| `lb_mean_s` / `stretch` | 74 s / 3.0 | 73 s / 2.65 | 90 s / 2.4 |
| `ub_per_hour` / `offered_load` | 390 / **3.0** | 595 / 0.47 | 2008 / **0.34** |
| busy / idle / home | 61 / 29 / 11 % | 59 / 38 / 3 % | 60 / 28 / 12 % |
| lost: all / peers / movers | 0.6 / 0.6 / 0 % | 3.4 / 2.4 / 0.8 % | 7.2 / 7.1 / 0 % |

What these say:

- **`fleet_scale` is not overloaded.** Its offered load is 0.34. The bound is
  optimistic, so that number alone proves nothing, but the robots are also idle
  or driving home 40 % of the time, and a task waits only 6 s on average in a
  busy robot's queue. The 18-task backlog is mostly tasks still in flight: 17
  of them were announced after t = 500 s, against a 203 s mean delivery. To
  make throughput separate methods there, the arrival rate has to go up (C12).
- **Peers cost the most lost time.** At 50 robots, stops for peers are 7.1 %
  of all robot-time and 12 % of the time spent on tasks. Notices, pick acks,
  yields and missing paths together are under 0.1 %.
- **The wait is after assignment, not before it.** Every scenario assigns a
  task within about 3 s, but the item leaves the shelf 83 to 136 s later. In
  `delivery_smoke`, the auction queues 4 of the 12 tasks behind a busy robot,
  for 174 s on average, while robots that finished early stand idle. For
  example, robot_0 is free from t = 65 s, but the task queued on robot_7 at
  t = 9 s starts only at t = 114 s.
- **The bound is close.** The time from reaching the shelf to DONE was at
  least 1.016 × `lb_s` for every delivered task in the three runs, so
  `stretch` is mostly the drive to the shelf and the queueing, not slack in
  the bound.

## Dispatchers and perfect information

Two switches (2026-11-26) for the benchmark's two gaps (docs/fleetsim-plan.md):
**who assigns tasks** (`dispatch.method`, the coordination gap) and **what
robots may know** (`info.*`, the information gap). Defaults are the robots
as they are, and a default run is bit-identical to one from before the
switches existed.

```bash
/usr/bin/python3 -m fleetsim run walkers --set dispatch.method=central_hungarian
# the perfect-information reference, for any method:
/usr/bin/python3 -m fleetsim run walkers --set dispatch.method=central_hungarian \
    --set info.map=truth --set info.peers=truth --set info.dispatch=truth \
    --set network.model=ideal --set network.ideal.delay_s=0
```

The operator view's run form offers the method as **Task assignment**.

### `dispatch.method`

| Method | Who decides | How |
|---|---|---|
| `auction` (default) | every robot | the real FleetTasks auction (`nodes/fleet_tasks.py`); the gateway only announces |
| `central_greedy` | one dispatcher at the gateway | on every event and every `resolve_period_s`, the oldest open task goes to the **idle** robot that reaches its pickup soonest (with [robot types](#robot-types-and-capabilities): an eligible one, that delivers it soonest), repeated while both remain; never reassigns |
| `central_hungarian` | one dispatcher at the gateway | on the same triggers, Kuhn-Munkres over robots × tasks not yet picked, minimising total drive time to pickup; a robot carrying an item counts as free at its dropoff after its remaining drive and dwell, so it can take one queued task |

The code is `fleetsim/dispatch/`. A robot talks to its task side through
one small interface (`RobotAgent` in `fleetsim/dispatch/__init__.py`:
`set_pose`, `tick`, `on_task_status`, `on_message`, `on_peer_status`,
`heard_network`, `owned`, `battery_pct`, `drive`, `drain`). FleetTasks is
one already, a central method gives robots a thin client instead, and the
decentralised methods still to come (SSI auctions, CBBA, the distributed
Hungarian method, #50) plug in at the same seam as per-robot agents that
exchange messages.

**A central dispatcher pays for its radio.** It sits at network node 0. A
Dispatch or Cancel travels gateway → robot as a reliable `/fleet/tasks`
message, and every task-status change travels back as a Report (one per
step at most). Poses come from the robots' `/fleet/status`, which the
gateway always receives. So under load its messages are late or lost, and
while the gateway is cut off it assigns nothing. Robots keep running the
same task FSM, without `AWAIT_PICK`: a central planner is its own single
source of truth, so it needs no confirmed-pick handshake. The rules that
keep it and the robots consistent, in brief (the `central.py` docstring
has them in full):

- A robot not heard for `status_timeout_s` (6 s) is lost. Its tasks that
  were not yet picked are re-planned. A task whose item is on board stays
  with it, because nobody else can deliver it. A lost robot keeps working
  what it has. When it is heard again, the dispatcher asks it (`Query`)
  what it holds before planning again.
- Whoever holds the item keeps the task. A robot refuses a Cancel for a
  task whose item is on board and reports instead. The dispatcher then
  takes that task back from whoever else it went to.
- A task whose robot is at the shelf (`PICKING`) is never moved. A Cancel
  sent then could arrive after the pick.
- Reports echo the sequence number of the last gateway message. A Query
  answer settles anything still unexplained: a lost Dispatch, a lost
  Report, or a reboot.

**Hysteresis.** The Hungarian dispatcher keeps a task with its current
robot unless another assignment saves at least `reassign_gain_s` (10 s).
Current pairs get that much off their cost, so a change must pay for
itself. Without it the assignment flips as robots move.

**Drive times** come from the true static map, because a warehouse's
layout is known to its operator. A blockage is known only through the legs
it makes fail, unless `info.map: truth`. One A* per robot-task pair per
solve is too slow at 20 × 20, so `fleetsim/dispatch/costs.py` builds a
Dijkstra distance field per waypoint on first use (~40 ms on the default
floor) and looks robots up in it. The field builds are almost all of the
dispatcher's cost: 0.5–1.6 s per run on `delivery_smoke` and `walkers`. The
Hungarian solve itself is under 1 ms at 20 × 20.

### `info.*`

| Knob | Default | `truth` |
|---|---|---|
| `info.map` | `shared` | every robot's belief and costmap, and a central planner's drive times, follow the true grid the moment it changes, blockages included |
| `info.peers` | `heard` | peer poses for the local layer, and path intents, come from ground truth every step, not from `/fleet/status` (a dead robot still counts: it is still in the way) |
| `info.dispatch` | `network` | `/fleet/tasks` is delivered instantly and never lost, through partitions too, and off the radio. A central dispatcher also reads each robot's true pose and liveness |

The full perfect-information reference is all three at `truth` plus
`network.model: ideal` with `network.ideal.delay_s: 0`. Status and map
patches still go over the network and still cost airtime, but robots no
longer depend on them. To take map traffic out as well, set
`map_share.enabled: false`. Movers (walkers) are not peers: a robot still
hears of one it can't see only through obstacle notices.

For `auction`, `info.dispatch: truth` means the auction's own protocol
runs on a perfect channel. Bids, claims, lease beacons, acks and completes
arrive at once, so nobody is ever isolated by a partition. This is the
auction's version of "the dispatcher's messages always arrive". It is the
same algorithm with perfect communication, which is what the information
gap compares against.

### Measured (seeds 1 and 2, i9-14900HX)

`robot stops` = the `nav` metric's `blocked_robot` (stops for a peer).
"perfect" = the reference settings above. `delivery_smoke`'s tasks and
homes are fixed and its network is ideal, so seeds 1 and 2 gave the same
numbers (only sub-millisecond timings differ). `walkers` has a Poisson
task stream and runs for its full 600 s, so its seeds differ (seed 1 / seed
2 below). Duplicates were 0 in every run.

| Scenario | Method | Info | Done | per_hour | mean_s | Robot stops |
|---|---|---|---|---|---|---|
| `delivery_smoke` | auction | network | 12/12 | 111.9 | 216.9 | 44 |
| | | perfect | 12/12 | 111.6 | 216.9 | 44 |
| | central_greedy | network | 12/12 | 116.3 | 195.8 | 92 |
| | | perfect | 12/12 | 116.3 | 195.8 | 92 |
| | central_hungarian | network | 12/12 | 127.5 | 183.5 | 80 |
| | | perfect | 12/12 | 127.5 | 183.5 | 80 |
| `walkers` | auction | network | 23/34 · 32/38 | 138 · 192 | 161 · 190 | 186 · 169 |
| | | perfect | 24/34 · 32/38 | 144 · 192 | 171 · 190 | 236 · 208 |
| | central_greedy | network | 25/34 · 35/38 | 150 · 210 | 138 · 171 | 236 · 293 |
| | | perfect | 27/34 · 37/38 | 162 · 222 | 142 · 164 | 186 · 259 |
| | central_hungarian | network | 27/34 · 37/38 | 162 · 222 | 142 · 151 | 199 · 260 |
| | | perfect | 27/34 · 37/38 | 162 · 222 | 142 · 158 | 208 · 304 |

Wall time per run, one at a time: `delivery_smoke` 5.7–6.7 s and `walkers`
13–16 s for every method. The central methods are slightly faster, because
nobody computes A* bids. On `fleet_scale` (50 robots, radio) it was
auction 174 s, greedy 122 s, Hungarian 123 s.

What this shows so far:
- On these ideal-network scenarios the **coordination gap is the bigger
  one**. Rolling Hungarian beats the auction by 14–17 % in deliveries per
  hour, and its mean delivery is 12–21 % shorter. Greedy sits between the
  two.
- **Perfect information barely moves anything here.** The network is
  already ideal, the maps start true, and nothing blocks an aisle. The
  information gap needs the radio and blockage scenarios (C12).
- **Central methods stop for peers more often** (`delivery_smoke`: 80–92
  vs 44). Why is not measured yet.
- **The rolling Hungarian can take a task off a robot with nothing in
  return** when there are more tasks than robots. The minimum total drive
  time prefers a nearer task, and the far one waits (`delivery_smoke`: task
  7 was dropped at 32 s and delivered at 318 s). That is the objective as
  specified. If p95 delivery time matters, it needs an age term.
- On `fleet_scale` (radio, 50 robots) the central methods delivered as
  much as the auction (119 and 123 vs 122). They also recorded 6–10
  duplicate episodes: robots the dispatcher lost when their Status was
  dropped kept driving to tasks that it had re-planned, until the Cancel
  arrived. There were 0 empty-shelf visits.

How "bit-identical" was checked: `delivery_smoke` and `partition_heal`
were run on the tree before the switches and after, and their
`events.jsonl.gz` compared line by line, ignoring `wall`, `wall_s` and
`summary`. Both were identical (3186 and 3325 events).
`test_defaults_reproduce_the_event_stream_from_before_the_dispatch_layer`
keeps a digest of the small test floor's run from before, on the numpy
version it was recorded with.

Not covered: `kill: gateway` doesn't kill the gateway. `_disturb` indexes
`robots[node - 1]`, so it kills the last robot. To take the gateway away,
cut it off with `partition: [[gateway], rest]`.

## Scenario editor

The operator view edits scenarios on their own floor. Open the Session panel,
go to the **Tests** tab, and find the scenario under **Fleet sim scenarios**:
**Edit** opens it, **New from** starts a new one as a copy under a free name.
The editor covers everything below the header until it is closed. None of it
is the live fleet.

- **Floor:** the true floor a run of the scenario gets: waypoints (named when
  zoomed in, or on hover), each robot's home (numbered), the gateway,
  blockages, the fixed tasks as pickup → dropoff lines, and the movers'
  paths. `fleetsim/preview.py` builds it with the sim's own `_build_world` and
  `_homes` (`Sim.world_only`), so synthetic options and saved maps look as
  they will run. It is rebuilt when the world options, the robot count or the
  seed change.
- **Timeline:** fixed tasks are ticks along the top, the task stream's window
  is a band, and each disturbance is a bar. Drag a bar to move it, or its
  right end to change how long it lasts. The slider shows the floor at that
  time: the blockages that are up, where the movers are, and the homes
  coloured by partition group (red: switched off).
- **Map tools:** **Tasks** (click a pickup, then a dropoff), **Blockage**
  (drag a rectangle; it appears at the timeline's time), **Mover** (click its
  path; Enter or a double-click ends it), **Gateway** (saved maps only; the
  synthetic warehouse puts its gateway on the apron). In **Pan**, select a
  pickup or dropoff field and click a waypoint to fill it in. Click a
  blockage, a path or a task's line to select it in the form.
- **Form:** name, title, description, seed, robots, duration and stop rule;
  the world (synthetic options as rows, or a map from the maps library);
  tasks; the Poisson stream; disturbances (blockage, partition groups written
  `robot_0, robot_1`, `0-3` or `rest`, switch off and on, cut link, and any
  kind added later as a YAML value); inline movers; settings (overrides of
  `base.yaml` by dotted name); metrics with pass criteria; and any other
  top-level key. Values in key/value rows are YAML, read like
  `--set key=value`, so `false`, `[robot_0]` and `0.5` mean what they mean
  in a terminal.

Keys: V, T, B, M and G pick a tool. Esc puts the tool down, and then closes
the editor. ←/→ move the timeline 1 s (with Shift, 10 s). Ctrl+S saves. Drag
to pan, scroll to zoom, double-click to fit.

**Checking.** The server checks every edit a moment after it is made. The
scenario goes through `config.resolve`, the validation `fleetsim run` does.
Then the floor is checked for what only the floor can show: a task that names
a waypoint that isn't there (fleet_tasks would reject it silently), a
`task_arrival` pattern that matches nothing, a blockage off the map, a
disturbance that names a robot the fleet doesn't have, more robots than
there is room to park. Errors are marked on their field and listed above the
buttons; click one to go to it. Warnings (a disturbance after the run ends,
a `world.gateway` the floor doesn't use) don't block a save.

**Saving** writes `fleetsim/scenarios/<name>.yaml` and nothing else, and
only from the sim console (`--control`), like launching a run. **Save as**
writes a new file and leaves the original as it is; it and **New from** copy
the original's untouched sections, as below. A file that changed on disk
after it was opened isn't overwritten. The leading comment block is
kept (edit it under **File**), and so is every top-level section that wasn't
changed, line for line, comments included. A changed section is rewritten in
the hand-written files' style and **loses its comments**. **File** lists the
comments a save would drop, so you can move them into the header or the
description first. **Run** saves, then picks the scenario in **Run a fleet
sim**.

**Untrusted input.** The console can be reachable from the internet, so
`webui/fleetsim_editor.py` treats every field as untrusted. The name must
match `config.NAME_RX`. `config.world.map` must name a map in the maps
library: fleetsim also takes a directory path there, and the console never
does. No other string under `config:` may look like a path. YAML values are
parsed with `safe_load`, and only plain data is accepted. A synthetic floor
is capped at 4 M cells. A `/usr/bin/python3` child builds the floor, one at
a time, with a time and memory limit. A saved scenario can still set any
knob in `base.yaml`; the run form's robot and duration caps and its Stop
still bound what a launch can cost.

From a terminal, `/usr/bin/python3 -m fleetsim.preview NAME` prints a
scenario's floor and the same problems.

## Benchmark scenarios (draft, for review)

`fleetsim/scenarios/bench_*.yaml` (C12 in `docs/fleetsim-plan.md`, 2026-11-26).
Ten drafts for the benchmark suite. Each one runs unchanged at
`--robots 2`, 5, 10 and 20, and each file's header says what it tests, which
load regime it targets and why it uses the network it does. They are drafts:
the edge cases are for the owner to review.

**Load regimes.** One robot on the default floor delivers about 22 tasks an
hour at most: 0.0062 tasks per robot per second. That is the second half of
900 s runs at twice that load, seed 1, at 5 and at 20 robots alike. So
`rate_per_robot_hz` 0.003 is about half the capacity (underloaded: every
method keeps up and only delivery time differs), 0.004 is about 0.65, and
0.012 is about 1.9 (overloaded: the backlog grows all run and throughput is
what differs). The underloaded drafts stop arrivals at 1200 s and run to
1500 s, so late tasks still finish. These ratios are measured, not
`offered_load`. Recalibrate them once A4's lower bound exists.

| Scenario | What it tests | Regime | Network | Duration | 5 robots, done / mean s (seed 1; seed 2) | 20 robots, done / mean s (seed 1; seed 2) |
|---|---|---|---|---|---|---|
| `bench_steady` | delivery time on the open floor | 0.003 /robot/s (~0.5) | ideal | 1500 s | 10/10, 165; 18/18, 185 | 65/65, 177; 73/73, 155 |
| `bench_rush` | throughput on the same floor | 0.012 /robot/s (~1.9) | ideal | 1200 s | 32/65, 96 /h; 37/73, 111 /h | 128/283, 384 /h; 131/290, 393 /h |
| `bench_burst` | assignment of a backlog: 3 tasks per robot at t=30, then nothing | burst | ideal | until done (≤1500 s) | 15/15, 297, makespan 601 s; 15/15, 304, 728 s | 60/60, 341, 1021 s; 60/60, 316, 791 s |
| `bench_narrow` | aisles 2, 4 and 6 one robot wide, every pickup in one | 0.004 (~0.65) | ideal | 1500 s | 15/15, 192; 23/23, 178 (plain floor 182; 177) | 84/84, 190; 94/94, 189 (plain 173; 166) |
| `bench_dead_end` | the same, with their end segments sealed into six one-lane bays | 0.004 (~0.65) | ideal | 1500 s | 15/15, 194; 23/23, 190 | **70/84**, 266, 91 legs given up; 92/94, 212, 20 given up |
| `bench_moved` | a full-width pallet that moves between aisles every 6 min, and a half-width one | 0.004 (~0.65) | radio, spatial | 1500 s | 12/15, 194; 19/23, 257 | 83/84, 162; 93/94, 174 |
| `bench_partition` | 40 % of the fleet cut off for 2 min; later the gateway and 30 % | 0.003 (~0.5) | ideal | 1500 s | 10/10, 165; 17/18, 213 | 64/65, 193; 72/73, 181 |
| `bench_dead_robot` | a robot carrying an item dead for 90 s, later a robot fetching dead for 120 s | 0.003 (~0.5) | ideal | 1500 s | 10/10, 172; 18/18, 191 | 65/65, 165; 72/73, 155 |
| `bench_dead_gateway` | the gateway down for 3 min; requests wait for it | 0.003 (~0.5) | radio, spatial | 1500 s | 8/10, 154; 15/18, 248 (from the request: 154; 254) | 64/65, 168; 73/73, 194 (from the request: 180; 207) |
| `bench_walkers` | the four walkers of `walkers` | 0.003 (~0.5) | ideal | 1500 s | 10/10, 168; 18/18, 188 | 65/65, 176; 73/73, 156 |

An eleventh draft, `bench_mixed` (a fleet of standard, fast and heavy robots;
A6), is measured in [Robot types and capabilities](#robot-types-and-capabilities).

Read `bench_rush`'s cells as deliveries per hour, not done/announced: the
undone tasks are the point. Up to 16 runs went in parallel. Even so, a
20-robot run took 40–105 s of wall time, and `bench_dead_end` took 170 s,
because its replans are expensive. A one-seed difference of 10–20 % is
noise: `bench_dead_robot` at 20 robots, seed 1, beat `bench_steady`. At 2
robots, seed 1 draws only 2–6 tasks per run, too few to read.

**New inputs.** Every default reproduces today's runs exactly. The event
streams of `delivery_smoke`, `map_sharing`, `partition_heal`, `walkers`
and 150 s of `fleet_scale` are identical before and after.
- `world.synthetic`:
  - `narrow_aisles: [a, ...]` and `narrow_m` (1.0) make single-lane aisles
    beside every rack segment. The crossings keep full width. Aisle 0 borders
    the apron and can't be narrowed.
  - `dead_ends: [[aisle, segment, west|east], ...]` walls off one end of a
    rack segment's aisle.
  - `walls: [[x0, y0, x1, y1], ...]` adds walls anywhere.
  - An unknown key in the block now fails validation, and so does a floor
    where some waypoint can't be driven to. Homes a wall covers are dropped.
- `task_arrival`:
  - `rate_per_robot_hz` is the alternative to `rate_hz`. Give exactly one.
  - `count_per_robot` is the alternative to `count`.
- `disturbances`:
  - A name that doesn't exist at the run's robot count fails validation.
    Before, `robot_3` at `--robots 2` crashed mid-run, and `kill: gateway`
    silently killed the last robot.
  - Partition groups can be fractions, `{first: 0.4}` and `{last: 0.3}`,
    mixed with names (`[gateway, {last: 0.3}]`).
  - `kill: {select: carrying | fetching | busy | any, within_s: S}` takes the
    lowest-index robot doing that when it fires. It waits up to `within_s`
    for one, then falls back to a busy robot, then to any robot. Its revival
    counts from the kill.
  - `kill: gateway` (`fresh` is rejected): while the gateway is down it hears
    and sends nothing. Requests wait for it and are announced when it is
    back, with `requested_s`.
  - The true floor is now the static map plus every blockage up now. Before,
    clearing a blockage deleted any other blockage that overlapped it.
  - A blockage waits until no robot or mover is under it. Before, one that
    appeared on top of a robot trapped it. The event records `waited_s`.
  - A revived robot no longer sends one Status and one map tick per step
    until its timers catch up.

**Findings from drafting them (2026-11-26).**
1. **Today's comms on the radio have a cliff at 20 robots.** With fleet-wide
   5 Hz Status and global contention (the defaults), 20 robots run at ~0.7
   load. A disturbance tips them past it for good. When the gateway came
   back, or a pallet moved, `/fleet/tasks` delivery went to 0 %, and 29 of
   65 and 18 of 84 tasks were done. With `network.capacity=unlimited` the
   same runs did 64/65 and 83/84. Spatial contention, or neighbour-only
   Status with session liveness, also holds (64/65 and 62/65 for the
   gateway run). The radio drafts use spatial contention for this reason.
2. **A 2-robot fleet on the radio can't hold a quorum in the far aisles.**
   A robot at `shelf_7_7` is out of range of both the gateway and the other
   robot. It loses its claim, the other robot gets the task, drives there,
   and loses it in turn, for the whole run. On the ideal network the same
   tasks are delivered first time. For this reason `bench_partition` uses
   the ideal network.
3. **A robot rebooted (`fresh: true`) with an item aboard** comes back
   IDLE. Its committed task is reclaimed, and robot after robot finds the
   shelf empty (`empty_shelf`, every one to two minutes, to the end of the run).
4. **A robot that never comes back is never mapped.** Robots see it as a
   mover, so each one sent past it stops. When it stands at a pickup, every
   leg there is given up after `block_fail_s`, again and again.
5. **`fleet_tasks.duplicates` counts what isn't duplicate work.** It counts
   two robots dispatched in the same step, where the loser cancels 3 ms
   later. It also counts a dead robot as still working its task. The
   drafts' pass criterion is `nav.empty_shelf == 0` instead (0 in every run).
   **Fixed in `fleet_tasks` v4:** a duplicate is now two live robots on one
   task for at least `dup_min_s` (2 s); the old count is kept as `overlaps`.
   v4 also measures delivery time from `requested_s` when a request waited
   out a gateway outage.
6. **Narrow aisles cost nothing while robots can avoid them.** With any
   shelf as a pickup, mean delivery at 20 robots matched the plain floor to
   0.1 s, so `bench_narrow` picks only in the narrow aisles.

**For the owner to decide.**
- Which comms the radio scenarios assume: today's robots with global
  contention (cliff at 20 robots), spatial contention (the drafts), or the
  neighbour-only design `fleet_scale` uses. Variants can override it with
  `--set`; the scenario default is the baseline.
- Whether the floor gets fixed relays or access points (not modelled), or a
  2-robot radio run stays a coverage test (finding 2).
- How many seeds or how long a run at 2 robots: ~7 tasks per 1200 s at
  0.003 per robot per second.
- Whether to spread homes across the apron for small fleets. Today robots
  park on the first n spots of the lattice, so 2 robots start at the west
  end.
- Whether held requests during a gateway outage should wait (the drafts)
  or be refused. Delivery time from the request needs A4's metric to read
  `requested_s`.
- Whether `bench_dead_end` at 20 robots (91 legs given up) is the right
  strength, and whether findings 3 and 4 should become scenarios once they
  are fixed.

## Comms traffic monitor

The replay's **Traffic** button (or T) opens a panel under the floor with two
charts over the whole run. Use it to tune the communication scheme: which
kind of message fills the channel, when the channel is past what the radio
can carry, and which robots lose traffic. The charts span the run on the
scrubber's own time axis, so the cursor (the replay's time) sits above the
slider's thumb. Hover a chart to read every series at that second. Press or
drag on a chart to move the replay there.

- **Channel load** (radio runs only): the busiest node's load and the fleet
  mean per 1 s sample, and the selected robot's own. The dashed red line is
  the 95 % cap (`LOAD_CAP`, `fleetsim/network.py`). Seconds in which some
  node was past it are shaded red above the cap, and the legend counts them
  (the `comms` metric's `over_cap_s`). Past the cap an enforced radio loses
  best-effort traffic and queues reliable traffic. With
  `network.capacity: unlimited` nothing is lost, and the shading is what a
  real radio could not have carried. Load is what a node's channel carries
  in its last topology epoch (`radio.topo_s`), so it is a 1 s average:
  bursts inside a second don't show.
- **Traffic by kind**, one line per kind (status, tasks, map, intent,
  obstacle, resync, and any other). **Sent** is copies sent per second.
  Every copy is unicast, so a fleet-wide message counts once per recipient.
  **Lost** is copies lost per second, **kB/s** is bytes offered, and
  **Airtime** is the airtime each kind offered as a share of one channel,
  every hop and retransmission included (radio only). Under Airtime the
  batman OGMs that carry routing are drawn too. With spatial contention the
  shares add over all nodes, so they can pass 100 %. The legend gives each
  kind's value at the cursor and the share of its copies lost. Click a kind
  in the legend to hide it, and the axis rescales: hide status to see what
  the map traffic does.
- **On the floor**, the **Load** checkbox colours each radio link by the
  busier of its two ends and rings each robot by its own channel's load:
  faint is idle, blue is near the cap, red is past it. The fleet has no
  per-link traffic figure (routes are only kept as totals per pair), so a
  link shows the channel it shares. With `contention: global` every node
  shares one channel, so every link and robot gets the same colour. Use
  `contention: spatial` to see where on the floor the channel fills.
- **The info panel**, with a robot selected, gives its load, the airtime its
  own sends took, and per kind the kB/s it sent and received and the share
  of its copies that were lost.

An ideal network has no airtime and no load, so the load chart and Airtime
are not shown. Its messages, deliveries and losses (`network.ideal.loss`)
are.

**What is recorded.** Each `net` sample (`record.net_s`, 1 s) has always had
the fleet's counters per kind. Since 2026-11-26 each kind also has `msgs`
(sends, as against copies) and, on the radio, `air_ms` (airtime offered).
With `record.net_nodes` on (the default), the sample also holds per node
(node 0 is the gateway, `robot_k` is node k + 1):

- `node_kinds`: `{kind: {sent, bytes, lost, rx, rx_bytes}}`, one value per
  node. The first three are counted at the sender, the last two at the
  receiver. A list that is all zeros is left out. Summed over nodes they
  equal the fleet's `sent`, `bytes`, `lost` and `delivered`.
- `node_load` (radio): each node's load factor, the one delay and loss are
  priced at.
- `node_air` (radio): the airtime the node's own sends took, in seconds per
  second, OGMs included.
- `ogm_air` (radio): the fleet's OGM airtime, in seconds per second.

Recording changes nothing else: a run's events are identical with
`net_nodes` on and off once the new fields are dropped
(`fleetsim/tests/test_replay.py`). `delivery_smoke`, `bench_moved` and
5 minutes of `fleet_scale` were also compared, event for event, against
recordings made before the change. At 50 robots a sample grows from 0.4 kB
to 3.7 kB, and a 5-minute `fleet_scale` `events.jsonl.gz` grows from 0.86 to
1.08 MB. A run recorded before the change still replays: the monitor draws
the fleet's charts and says that per-robot traffic was not recorded.

## Robot types and capabilities

Robots may differ (A6 in `docs/fleetsim-plan.md`, 2026-11-26). A robot type
sets a robot's speed, acceleration, pick and place times, battery drain and
capabilities. A task may require a capability. The default is one type with
today's values, and a default run is bit-identical to one from before types
existed.

```yaml
config:
  robots:
    types:
      - {name: standard}                       # no share or count: it takes the rest
      - {name: fast, share: 0.3, max_speed_mps: 0.75, accel_mps2: 0.75}
      - {name: heavy, share: 0.2, capabilities: [heavy], max_speed_mps: 0.4,
         accel_mps2: 0.3, pick_s: 8, place_s: 5}
tasks:
  - {at_s: 5, pickup: shelf_1_0, dropoff: dock_0, requires: heavy}   # or a list
task_arrival: {rate_per_robot_hz: 0.004, pickups: "shelf_*", dropoffs: "dock_*",
               requires: {heavy: 0.2}}
```

**Types.** `robots.types` is `null` (one type: the `robots.*` values, and
`tasks.dwell_s` to pick and to place) or a list. Each type has a `name` and
may set `max_speed_mps`, `accel_mps2`, `yaw_rate_rps`, `pick_s`, `place_s`,
`battery_drain_pct_per_s` and `capabilities`. What a type does not set comes
from `robots.*` and `tasks.dwell_s`. The battery drain only matters with a
battery model (`fleet_tasks.battery.sim_start_pct`). The footprint stays
fleet-wide, because every costmap and the floor check use it.

**How many of each.** A type gives `share: F`, `count: N`, or neither:

- `share: F` takes round(F × robots) robots, and at least one. A capability
  that a 20-robot fleet has is then also there at 2 robots.
- `count: N` is fixed: it does not scale with `--robots`.
- The one type with neither takes the rest. It can get none. Without such a
  type, the counts must add up to the fleet size, or validation fails.

bench_mixed's fleet at the benchmark sizes:

| robots | standard | fast (0.3) | heavy (0.2) |
|---:|---:|---:|---:|
| 2 | 0 | 1 | 1 |
| 5 | 2 | 2 | 1 |
| 10 | 5 | 3 | 2 |
| 20 | 10 | 6 | 4 |

Types are spread through the fleet, not given in blocks: each type's robots
take evenly spaced indices (`config.robot_profiles`). `robot_k` parks at the
k-th home and wins exact auction ties by its index, so a block would park
every heavy robot at one end of the apron and give one type all the
tie-breaks. The `world` event records each robot's type (`types: {robot_0:
standard, ...}`), only when types are set.

**Tasks that need a capability.** A fixed task takes `requires` (one name or
a list). `task_arrival.requires` gives the share of arrivals that need each
capability. Each arrival needs at most one, so the shares add up to at most 1.
The requirement draws use an RNG of their own, so a seed gives the same times,
pickups and dropoffs with or without `requires`. The `announce` event carries
`requires`. A capability that no type lists fails validation, because it is
most likely a typo. A type with no robots at this fleet size passes
validation: its tasks are the case below.

**A task no robot can do.** When no type that has the capability has a robot
(a `count: 0`, or the rest type got none), the task is still announced, as the
real gateway would announce it. Then:

- nobody is ever assigned it, and the gateway's `task_stalled` alarm shows it;
- the run's `notes` say so, and its `task_bound` has `able: 0` and no `lb_s`;
- `bounds.unservable` counts it, and `fleet_tasks` counts it as undone
  (`backlog_end`);
- `stop: {until: tasks_done}` does not wait for it, so the run still ends.

**The task FSM** waits `pick_s` at the shelf and `place_s` at the dock.
`AWAIT_PICK` is unchanged: `pick_s` starts when the pick permit arrives.

**The auction.** Each robot's FleetTasks gets its type's `speed_mps`,
`capabilities`, `service_s: {pick, place}` and battery drain.

- *Who bids.* The gateway's TaskAnnounce carries `requires` (a list) when the
  task has one, and only then. A robot whose `capabilities` lack one returns
  no cost for the task, so it does not bid, in the first round or in any rebid
  round. FleetTasks' cost function makes this decision (protocol.py: `None` =
  "this robot cannot take the task at all"). `commsim/core/protocol.py` is
  unchanged. Its `cost_fn` gets only coordinates, and two tasks from the same
  shelf to the same dock can need different robots. So `fleet_tasks._Allocator`
  overrides `_open_bidding`, where every bid and its backlog are priced, to
  say which task it prices.
- *The wire.* An announce without `requires` is byte-identical to before and
  open to every robot. A node that predates the field ignores it
  (`test_a_node_that_predates_requires_still_takes_the_announce`). A
  malformed `requires` gets no bid from anyone, and the gateway refuses a
  request that has one. The gateway's `/fleet/task_request` takes an optional
  `"requires"`, and the board shows it per task.
- *What a bid is.* With `service_s` set, a bid is the time until this robot
  would have delivered the task: the drive to the pickup, the pick, the carry
  and the place, all at its own speed and times, plus the same for what it
  already owns (the backlog). Without it, a bid is the drive to the pickup, as
  before. That ranks identical robots correctly, because the rest of the task
  costs them all the same. It cannot rank a fast robot against a slow one, or
  a slow picker against a quick one. fleetsim sets `service_s` on every robot
  when `robots.types` is set. The battery gate then counts the pick and place
  as driving, which slightly overestimates the drain. On the real robots, set
  `service_s` on every robot or on none, because bids are compared across the
  fleet.

**The central dispatchers.** A task that requires a capability goes only to
robots whose type has it, for both methods. Each robot's drive time uses its
own max speed, and the Hungarian "free at" offset uses its own pick and place
times. When the robots differ, a pair also costs the rest of the task: the
pick, the carry at that robot's speed, and the place. Greedy then picks the
robot that would deliver soonest, as the auction does. The Hungarian solve
takes that term relative to the task's best eligible robot. Added as it is, it
would be a constant per task, and that changes which tasks get robots when
there are more tasks than robots. With identical robots the term is zero.

**The lower bound** (`bounds` v2). A task's `lb_s` is the best robot this
fleet could send: the minimum, over the types that can do it and have a robot,
of drive / their max speed + their pick + place. A task with a requirement
also records `requires` and `able`, the number of robots that can do it.
`ub_per_hour` is the lowest of:

- robots × 3600 / `lb_mean_s`, as in v1;
- for each requirement R: `able` × 3600 × tasks / (the summed bounds of the
  tasks that need R or more). The robots that have R must do all of that work
  themselves.

It is still optimistic: it ignores the empty drive between tasks, and it
assumes every robot is as quick as the best type. A run without types, and
every run recorded before them, scores as in v1, with `unservable` 0.

**Per-task service times: not added.** A package's own handling time is the
same whichever robot takes it. So it shifts every candidate's cost by the same
amount and cannot change who should take the task. What does depend on the
pair (a heavy package needs a different gripper, which is slower) is a
capability on the task plus that type's own `pick_s` and `place_s`. A
per-task time would also need one more field on the wire and in the central
costs.

### Measured (2026-11-26, seeds 1-3, `fleetsim bench mixed --jobs 6`)

`fleetsim/suites/mixed.yaml`, 72 runs, no crashes, 8-66 s of wall time
each. Four rows per fleet size:

- `bench_mixed`: the mixed fleet above, a fifth of the tasks need `heavy`;
- `bench_mixed_uniform`: the same task stream, every robot a standard one
  that can lift heavy packages;
- `bench_steady_mixed`: `bench_steady` with bench_mixed's fleet and no
  requirements, so speed and handling differ but nothing is exclusive;
- `bench_steady` as it is.

Mean delivery time in s (p95 in brackets), mean ± sd over the seeds:

| scenario | robots | auction | central_greedy | central_hungarian |
|---|---:|---:|---:|---:|
| `bench_mixed` | 5 | 219 ± 71 (500) | 192 ± 83 (429) | 169 ± 50 (351) |
| | 20 | 169 ± 22 (383) | 152 ± 13 (294) | 154 ± 12 (300) |
| `bench_mixed_uniform` | 5 | 178 ± 10 (280) | 154 ± 7 (227) | 150 ± 8 (221) |
| | 20 | 160 ± 4 (274) | 151 ± 2 (239) | 147 ± 3 (232) |
| `bench_steady_mixed` | 5 | 131 ± 5 (179) | 124 ± 10 (184) | 119 ± 11 (175) |
| | 20 | 132 ± 9 (220) | 122 ± 4 (202) | 122 ± 9 (198) |
| `bench_steady` | 5 | 171 ± 12 (254) | 155 ± 6 (219) | 154 ± 6 (216) |
| | 20 | 161 ± 14 (284) | 149 ± 6 (230) | 148 ± 8 (231) |

Every run delivered at least 96 % of its tasks. The lowest was
`bench_mixed`, 20 robots, seed 1, auction: 81 of 84, with 3 still in flight
at 1500 s. Every run had 0 empty-shelf visits. There
was one contact in 72 runs, in `bench_mixed_uniform`. `bench_mixed`'s
`offered_load` was 0.38 ± 0.15 at 5 robots and 0.40 ± 0.16 at 20, against
0.26-0.28 for the same tasks on the uniform fleet. The difference is the cap
from the heavy robots.

Heavy tasks against the rest in `bench_mixed`, read from the runs' events
(not a metric yet). Means are over the seeds' means; shares and counts are
over all three seeds:

| robots | method | heavy tasks, mean s | other tasks, mean s | delivered by standard / fast / heavy | other tasks delivered by a heavy robot |
|---:|---|---:|---:|---|---:|
| 5 | auction | 366 | 150 | 12 / 61 / 27 % | 4 |
| | central_greedy | 362 | 118 | 22 / 56 / 22 % | 1 |
| | central_hungarian | 299 | 115 | 14 / 66 / 20 % | 0 |
| 20 | auction | 290 | 136 | 28 / 51 / 21 % | 5 |
| | central_greedy | 248 | 125 | 26 / 53 / 20 % | 1 |
| | central_hungarian | 243 | 127 | 23 / 57 / 20 % | 0 |

What this shows:

- **Faster robots, priced by delivery time, get most of the work.** With no
  exclusive capability (`bench_steady_mixed`), fast robots delivered 67-89 %
  of the tasks and heavy robots at most 1 %. Mean delivery fell by 18-23 %
  against the plain fleet for every method (auction 171 → 131 s at 5 robots,
  161 → 132 s at 20). The standard robots stand idle more.
- **The scarce capability is the bottleneck.** Heavy tasks took 1.9-3.1 × as
  long as the others, because the heavy robots are both few and slow. Against
  the uniform fleet on the same tasks, the mean rose 13-25 % at 5 robots
  (auction 178 → 219 s) and 1-6 % at 20. The p95 rose much more (auction at
  5 robots: 280 → 500 s).
- **The coordination gap widens with a mixed fleet at 5 robots**: the
  Hungarian dispatcher beat the auction by 23 % there (219 vs 169 s), against
  16 % on the uniform fleet. Most of it is on the heavy tasks (366 vs 299 s).
  At 20 robots the gap is 8-9 % either way.
- **The heavy robots were not wasted on other work**: 0-5 tasks over three
  seeds. Delivery-time pricing keeps a slow robot off a task that a quicker one
  can do, so no method had to reserve the heavy robots. A heavy type as fast as
  a standard one would take ordinary work and make heavy tasks wait. That case
  is not tested.
- **The task mix varies a lot between seeds.** The share of heavy tasks is a
  draw per arrival: at 20 robots seeds 1, 2 and 3 drew 29 %, 11 % and 22 %.
  That is most of the ± on `bench_mixed`, and at 5 robots one seed had only 2
  heavy tasks.

## Map sharing: subscriptions and event-driven updates

Two switches (2026-11-26, plan items B8 and B9), both off by default. A
default run is the run from before them plus one recording-only event kind,
`map_learn` (below).

```bash
/usr/bin/python3 -m fleetsim run bench_moved --set map_share.recipients=subscribers
/usr/bin/python3 -m fleetsim run bench_moved --set map_share.events=true
/usr/bin/python3 -m fleetsim bench map_sharing --jobs 6    # the comparison below, ~35 min
```

| `map_share.recipients` | A tile's patches and refresh snapshots go to |
|---|---|
| `all` | every robot |
| `radius` (default) | the robots within `radius_m` (20 m) of the tile when it is sent. A robot that drives in later has missed what came before, until a refresh or a resync reaches it |
| `subscribers` | the robots that subscribe to the tile, as the sender has heard them (`fleetsim/mapshare.py`) |

### Subscriptions (B8)

- **What a robot subscribes to.** The tiles that meet a disc of
  `sub_radius_m` (6 m, a little more than the camera's 4.5 m) around it. Also
  the tiles that the next `sub_path_m` (30 m; `null` is the whole path) of
  its planned path passes through or comes within a body of. It works the set
  out at every map tick, and at once when it gets a new path. Waiting for the
  tick, a robot given a task drove 1.5 s along a path through a pallet that
  the fleet had known about for 160 s.
- **Announcements.** The whole set with a version number: when it changes,
  at most every `sub_min_interval_s` (2 s), and unchanged every
  `sub_keepalive_s` (30 s). A lost announcement is healed by the next one,
  and a late one is dropped by its version. A robot not heard for 2.5
  keepalives drops out of the others' views, so a dead robot stops costing
  patches. Each robot routes by its own view, never by the true
  subscriptions.
- **How announcements travel** (`sub_transport`). `status` (default): on the
  robot's next two `/fleet/status`, to the Status recipients, the way intents
  ride. `message`: its own reliable message to every robot, as the plan said.
  As a message it took `fleet_scale` (50 robots, radio) past the cliff in all
  three seeds. Its `mapsub` copies (297 k in 900 s, seed 1) cut Status delivery to
  57–71 %. Every peer whose Status was lost then counted as an unknown mover,
  so obstacle notices went from 717 to 218 k copies (seed 1). `/fleet/tasks` delivery
  fell to 11–27 %, and deliveries to 35–68, against 96–122 with `radius`. This
  is the lesson of path intents again: on the radio, each frame costs
  airtime, whatever its size.
- **A new tile is pulled from one peer.** The plan said that every robot that
  has observed the tile sends its snapshot. That is k copies of nearly the
  same tile. On `bench_moved`, nearly every robot has seen nearly every tile,
  so it would be 9 snapshots of 4 kB for each new subscription at 10 robots.
  Every current subscriber of a tile has been getting all its patches, so one
  of them is enough. The new subscriber asks one (`mappull`, reliable): the
  one nearest itself, which is the cheapest answer on the radio. It prefers
  one subscribed for longer than a pull takes, so it doesn't ask a robot
  that is still waiting for its own answer. If nobody subscribes, it asks the
  peer that it last heard leave the tile.
- **The answer** (`mapans`, reliable) is paid for from the answerer's own
  `rate_cap_bps` budget. It goes at once if there is room, else at the
  answerer's next tick, before its refresh. What it carries:
  - With a shared prior (`prior: truth`, robots that localize on a saved
    map): only the cells that have ever differed from the prior in the
    answerer's map, with their current state. A pallet that has gone comes
    back as free. The first version sent the cells observed since the asker
    last knew the tile. Sensing re-stamps every cell it sees, so that was most
    of a busy tile: 34,064 cells in 149 answers on `map_sharing`, against 91
    cells now.
  - Without a prior: the known cells observed after the asker last left the
    tile, less two publish periods. If it never had the tile, every known
    cell.
  - The answerer's sequence numbers for the tile, so that the asker's next
    delta from each origin does not count as a gap. Each false gap costs a
    full MapResync snapshot.
- **World changes go to every robot** (`sub_flips_to: all`, default). A
  patch that carries a cell the sender's own sensing flipped (free ↔
  occupied) goes to every robot. The rest goes to the tile's subscribers. The
  first version kept flips with the subscribers. A robot far from a new
  pallet then bid on its stale map, won a task past the pallet, and learned
  of it only when its path subscribed (`map_sharing`: 91 s for a delivery
  that `all` made in 81 s). Subscriptions keep news local, but the auction
  prices tasks anywhere. Flips are rare and small. `sub_flips_to:
  subscribers` keeps the first version, for comparison.
- **Sequence numbers, MapResync and refresh** work as before, with three
  rules:
  - A patch of a tile that nobody subscribes to is not sent and takes no
    sequence number.
  - A gap on a tile with an open pull asks for no resync.
  - A delta of a tile that the robot doesn't subscribe to (fleet-wide news)
    asks for no resync and records no sequence number. A recorded number
    would hide a real gap once the robot subscribes.

  A revived robot has left every tile at the time it died, so it subscribes
  and pulls again.

### Event-driven publishes (B9)

With `map_share.events: true`, a tile goes out at once when the robot's own
sensing flips a known cell in it, not at the next `publish_s` tick. It goes
to the same recipients, from the same budget, at most every `event_gap_s`
(0.5 s). Over the cap it waits for the tick. With intents on, a robot whose
path changed with the flip rebuilds its intent. An extra Status carries it,
so the intent does not wait for the next scheduled Status. Each event
publish is recorded (`map_pub`).

The trigger is any free ↔ occupied flip, not only a flip on a known path:
- An event moves a publish earlier. It sends the cells that the tick would
  have sent, so the bytes are the same. The only extra is a message header,
  when an event splits a tick's patch in two.
- fleetsim's sensing is exact on the static world, so every flip is a real
  change: a pallet came or went. Unknown → known is discovery, and is not a
  trigger.
- Paths are only partly known: a robot knows its own, and its peers' only
  with intents on.

A real robot's sensor noise flips cells, so a filter to paths would earn its
place there.

### What measures it

- **`map_learn`** (`fleetsim/maplearn.py`), recording only. For every
  blockage that appears, one event per robot when its own map first has the
  blockage. The event gives the time since the appearance, how the map got
  it (`sensed`, `shared`, `truth`, `already`), and `path_s`: when the robot's
  path first ran into it while the robot didn't know. A blockage that clears
  gets the same for every robot whose map had it then. A robot that took a
  gone pallet in later from a stale peer is a ghost (`ghost_s`).
- **`map_spread`** (new metric), from those events. `on_path` counts the
  (blockage, robot) pairs that mattered. The values, all in sim seconds:
  - `late_mean_s`: how long a robot drove a path through a blockage it didn't
    know of.
  - `spread_mean_s`: from "some robot knows and it's on my path" until my map
    has it. This is propagation alone.
  - `missed`: robots whose own camera found a blockage that another robot
    already knew of.
  - `never`: robots on whose path the blockage stood but whose map never
    had it.
  - `clear_mean_s` and `clear_never`: for a cleared blockage, how long robots
    kept the phantom, and how many still had it at the end.
  - `ghosts`: robots that took a gone pallet in from a stale peer.

  From the appearance is the wrong clock: a robot given a task past the
  pallet ten minutes later needs the news then, not before.
- **`comms` v3** adds `map_bytes_per_robot_s` (all map kinds),
  `sub_bytes_per_robot_s` (announcements and pulls) and
  `map_copies_per_robot_s`. New `net` kinds: `mapsub`, `mappull` and
  `mapans`. On Status, an announcement costs bytes on the Status (4 B plus
  2 B per tile) and no frames.

### Measured (seeds 1–3, 2026-11-26)

`fleetsim bench map_sharing`, mean ± sd over seeds 1, 2 and 3.
- `subs`: subscriptions with the defaults above (on Status, flips to every
  robot).
- `subs_local`: flips to subscribers only.
- `subs_msg`: announcements as their own message.
- `_ev`: with `events: true`.
- `truth`: `info.map: truth` with the scenario's own recipients, so every map
  is perfect.

Map B/s and sub B/s are bytes per robot per second, from `comms` v3. A
missing value is a seed where nothing happened, for example no robot's path
ran into a pallet. `map_sharing`'s tasks and homes are fixed and its network
is ideal, so its three seeds gave the same numbers; its map bytes are from
seed 1, rescored with `comms`. The `subs`, `subs_ev` and `subs_local` rows
ran on 8cb8f6e, the others on 2bdae84. Between the two commits only the
Status transport for subscriptions was added, and the other rows don't use
it.

`map_sharing`, 10 robots, ideal network: a pallet in front of every pickup.

| variant | per h | mean s | stale cells | replans shared | late s | map B/s | sub B/s |
|---|---|---|---|---|---|---|---|
| `all` | 56.0 | 97.8 | 0 | 0 | 19.6 | 2421 | 0 |
| `all_ev` | 56.0 | 97.8 | 0 | 0 | 19.6 | 2422 | 0 |
| `radius` | 56.0 | 97.8 | 3 | 0 | 19.6 | 1941 | 0 |
| `radius_ev` | 56.0 | 97.8 | 3 | 0 | 19.6 | 1941 | 0 |
| `subs` | 56.0 | 97.8 | 0 | 0 | 19.6 | 1056 | 0.6 |
| `subs_ev` | 56.0 | 97.8 | 0 | 0 | 19.6 | 1057 | 0.6 |
| `subs_local` | 56.0 | 101.1 | 13 | 3 | 9.8 | 1146 | 0.6 |
| `subs_msg` | 56.0 | 97.8 | 0 | 0 | 19.6 | 1056 | 18.3 |
| `truth` | 55.5 | 93.0 | 0 | 0 | - | 2147 | 0 |

`bench_moved`, radio with spatial contention: a pallet that moves between
aisles.

| variant | robots | per h | mean s | stale | late s | missed | phantoms at end | ghosts | map B/s | sub B/s | map copies/s | load |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `all` | 10 | 115.2 ± 10.8 | 195.1 ± 9.0 | 1 | 14.4 (2/3) | 0 | 6.7 | 0 | 8892 | 0 | 2.25 | 0.139 |
| `radius` | 10 | 116.0 ± 10.4 | 197.8 ± 12.1 | 2 | 19.0 (2/3) | 0 | 6.0 | 1.3 | 4670 | 0 | 1.14 | 0.128 |
| `subs` | 10 | 116.0 ± 10.4 | 199.4 ± 14.4 | 2 | 14.2 (2/3) | 0 | 4.7 | 1.0 | 1987 | 2 | 0.67 | 0.121 |
| `subs_msg` | 10 | 116.0 ± 10.4 | 199.4 ± 14.4 | 2 | 14.2 (2/3) | 0 | 4.7 | 1.0 | 2035 | 51 | 2.12 | 0.123 |
| `truth` | 10 | 110.0 ± 16.6 | 192.2 ± 6.5 | 0 | - | 0 | 0 | 0 | 4778 | 0 | 1.17 | 0.126 |
| `all` | 20 | 219.1 ± 11.6 | 172.4 ± 7.6 | 1 | 32.0 ± 4.9 | 0.3 | 45.0 | 0 | 1 | 0 | 0.01 | 0.453 |
| `all_ev` | 20 | 218.3 ± 11.2 | 173.6 ± 8.0 | 1 | 31.8 ± 4.8 | 0.3 | 33.3 | 0 | 2 | 0 | 0.01 | 0.452 |
| `radius` | 20 | 218.3 ± 11.2 | 173.5 ± 10.9 | 10 | 32.0 ± 35.2 | 0.7 | 27.3 | 13.3 | 2208 | 0 | 0.54 | 0.459 |
| `subs` | 20 | 218.3 ± 11.2 | 172.6 ± 7.6 | 2 | 28.6 ± 5.8 | 0 | 38.7 | 0 | 4001 | 2 | 1.15 | 0.474 |
| `subs_ev` | 20 | 217.5 ± 10.8 | 173.1 ± 7.8 | 1 | 28.4 ± 6.0 | 0 | 33.7 | 0 | 4059 | 2 | 1.18 | 0.474 |
| `subs_local` | 20 | 218.3 ± 11.2 | 172.0 ± 7.6 | 15 | 31.2 ± 14.5 | 0 | 24.7 | 8.7 | 4012 | 2 | 1.15 | 0.476 |
| `subs_msg` | 20 | 217.4 ± 10.2 | 172.5 ± 7.4 | 2 | 28.6 ± 5.8 | 0 | 33.3 | 0 | 4098 | 93 | 3.92 | 0.482 |
| `truth` | 20 | 222.9 ± 15.5 | 167.3 ± 4.4 | 0 | 0 (2/3) | 0 | 0 | 0 | 2238 | 0 | 0.55 | 0.467 |

At 10 robots, every `_ev` variant and `subs_local` matched its base row
to within 1.5 % (`fleetsim/suites/map_sharing.yaml` reruns them).

`fleet_scale`, 50 robots, radio, starting blind, no blockage.

| variant | per h | mean s | `/fleet/tasks` delivery | stale | replans shared | map B/s | sub B/s | map copies/s | load |
|---|---|---|---|---|---|---|---|---|---|
| `all` | 395 ± 61 | 208 ± 8 | 0.52 ± 0.14 | 2910 | 105 | 564 | 0 | 2.94 | 0.84 |
| `radius` | 443 ± 57 | 213 ± 10 | 0.66 ± 0.28 | 32517 | 276 | 1106 | 0 | 2.52 | 0.77 |
| `subs` | 362 ± 143 | 226 ± 25 | 0.44 ± 0.22 | 41659 | 430 | 2929 | 2 | 1.40 | 0.93 |
| `subs_msg` | 196 ± 72 | 193 ± 5 | 0.17 ± 0.09 | 44018 | 239 | 1811 | 214 | 7.12 | 1.25 |
| `truth` | 378 ± 20 | 206 ± 7 | 0.45 ± 0.06 | 0 | 0 | 367 | 0 | 0.09 | 0.96 |

`fleet_scale` has no blockage, so its `_ev` rows equal their base rows. It
also sits at the radio's cliff. Per seed, `/fleet/tasks` delivery was
either about 0.95 or about 0.3, so its means show which seeds tipped over.
Three seeds can't separate the policies by throughput there; its bytes and
load can. With `refresh_s: 600` (seeds 1–3), `subs` delivered 490, 307
and 498 an hour on 425–526 map B/s, against 486, 311 and 337 an hour on
666–818 map B/s for `radius`.

What this says:
1. **On these scenarios, who gets the patches doesn't change throughput or
   delivery time beyond seed noise.** Few robots need the news: 1–3
   (blockage, robot) pairs per run had a path into a pallet they didn't
   know about. `bench_moved`'s pallets each lie between two shelves of a
   segment, so only through-traffic runs into them. Perfect maps (`truth`)
   are 3–5 % faster in delivery time (93.0 against 97.8 s; 167 against
   172 s). Most of that gap is the first robot's discovery (`late` 14–32 s,
   mostly that robot driving up to the pallet) and phantoms (below), and no
   sharing policy removes either.
2. **Subscriptions know what `all` knows where it matters.** With flips to
   every robot, `missed` is 0, stale is 1–2 cells and there are 0 ghosts.
   Where the refresh fits the budget, they do it for a fraction of the
   bytes: 44 % of `all` on `map_sharing`, and on `bench_moved` at 10 robots
   22 % of `all` and 43 % of `radius`, with the fewest copies and the lowest
   load.
3. **The refresh spends whatever the patches leave, up to `rate_cap_bps`.**
   A full tile with a shared prior is 4.1 kB. To 19 robots that is 78 kB,
   more than the token bucket holds (2 × 25 kB). So with `all` at 20 robots
   the refresh never goes out: it stalls on its first tile, and `all` sends
   only deltas (1 B/s). `radius` stalls the same way on crowded tiles.
   Subscriptions send each refresh to fewer robots, so their refresh goes
   out, and at 20 and 50 robots they send more map bytes than `radius`:
   4001 against 2208, and 2929 against 1106. On `fleet_scale` that is
   enough to tip more seeds over the cliff. "Map bytes" at these sizes
   measures the budget more than the policy.
4. **Announcements as messages break the 50-robot radio; on Status they
   don't.** 7.1 copies per robot-second against 1.4, load 1.25 against 0.93,
   and `/fleet/tasks` delivery 0.17 against 0.44.
5. **Flips have to go to every robot.** `subs_local` has 13–15 stale cells
   and 9 ghosts at 20 robots, and the stale-bid detour on `map_sharing`.
6. **Event-driven publishes change nothing measurable here.** There were 0–28
   event publishes per `bench_moved` run, 4–5 per `map_sharing` run and none
   on `fleet_scale`. Peers learn 3 ms after the discoverer instead of up to
   `publish_s` later (`test_an_event_publish_goes_out_when_the_camera_sees_the_blockage`).
   That is too little time to change a plan made 20 m away.
7. **Phantom pallets outlast every policy.** At 20 robots, 25–45 (pallet,
   robot) pairs still had a gone pallet in their map at the end of the run.
   Newest-observation-wins clears a pallet only when someone looks again,
   and the robots that believe it route round the spot. Ghosts (a gone
   pallet taken in from a stale peer's refresh) appear only where world
   changes don't reach everyone: 13 under `radius` and 9 under `subs_local`.

**How "identical" was checked.** `delivery_smoke`, `map_sharing` and
`bench_moved` (10 robots, radio, 1500 s) were run before the change
(c7bf064) and after every commit (da42b0b, 2bdae84, 8cb8f6e). Their
`events.jsonl.gz` were compared line by line, ignoring `wall`, `wall_s` and
`summary`, with the new `map_learn` events left out. All were identical:
3198, 2813 and 13660 events. `map_learn` added 0, 10 and 57 events.
`test_map_learn_only_adds_its_events` checks the same on the small test
floor, and `test_defaults_never_enter_the_new_code` fails a default run that
touches the subscription or event code. The dispatch layer's golden digest
(`test_defaults_reproduce_the_event_stream_from_before_the_dispatch_layer`)
still holds.

**Open issues.**
- A patch of a tile with no known subscriber is not sent and takes no
  sequence number. A robot whose subscription the sender hasn't heard yet
  misses it, and sees no gap, until a refresh. The window is usually
  milliseconds; after a lost Status it can last up to the keepalive.
- A revived robot keeps its old view of who subscribes, so it can pull
  from a peer that left meanwhile. A resync heals it, at the cost of
  traffic.
- An answer that waits longer than the pull window for budget lets the
  asker's gaps call for resyncs as well, so the tile is paid for twice.
- On Status, an announcement reaches only the Status recipients.
  `fleet_scale` has neighbour-only Status (12 m), so a far origin doesn't
  learn of a subscription along someone's path. Flips still reach every
  robot, and the pull covers the rest.
- `clear_mean_s` covers the robots that had the pallet when it went. The
  robots that took it in afterwards are counted as `ghosts`.

**For the owner to decide.**
- **The refresh.** It is a full snapshot that fills whatever budget is left,
  and it stalls when one tile to many robots doesn't fit the bucket. It could
  get a budget of its own, or become a digest: sequence numbers per tile, and
  a resync only for the robots that are behind. With a shared prior it could
  send only the cells that differ from the prior, as pull answers do. Until
  then, map bytes at 20 or more robots compare budgets, not policies.
- **Phantom pallets.** One option is an expiry for occupied cells that
  differ from the prior. Another is a robot that replans round a pallet
  nobody has seen for minutes and so goes to look. Either is a behaviour
  change, beyond map sharing.
- **The default.** Keep `radius` for now: subscriptions don't change
  deliveries here, and they cost fewer bytes only where the refresh fits
  the budget. Decide again after the refresh change.
- **`map_share.events`.** Keep it off. It costs nothing and gains nothing
  measurable with exact sensing. Revisit once sensing has noise.
- **A scenario where map news decides deliveries.** No shipped scenario has
  one. The candidate is `map_sharing`'s pattern (pallets in front of popular
  pickups) on the radio at 20 or more robots, with pallets that move.

## Traffic: keep right and single-lane passages

Two switches (2026-11-26, `fleetsim/traffic.py`; items B10 and B11 in
[the plan](fleetsim-plan.md)): `traffic.keep_right` puts a lane term on each
robot's A*, and `traffic.passages` makes robots claim single-lane passages
before they drive in. Both are off by default, and off, a run is
bit-identical to one from before they existed: the `events.jsonl.gz` of
`delivery_smoke`, `bench_narrow` and 600 s of `bench_dead_end` at 5 robots
were compared before and after, ignoring `wall`, `wall_s` and `summary`, and
were identical. `test_defaults_reproduce_the_event_stream_from_before_traffic`
keeps a digest of a small narrow-aisle run from before. The robots don't do
either yet.

```bash
/usr/bin/python3 -m fleetsim run bench_rush --robots 20 --set traffic.keep_right.enabled=true
/usr/bin/python3 -m fleetsim run bench_dead_end --robots 20 --set traffic.passages.enabled=true
/usr/bin/python3 -m fleetsim bench traffic --jobs 6     # off / keep_right / passages / both, 3 seeds
```

**What both read.** Each robot reads the corridors from its OWN belief, as
it plans on it. For every free cell, it takes the free run through it along
x and along y. The shorter run is the corridor's cross-section: its length
is the free width, its axis is the way the corridor runs, and the cell's
place in it is its offset from the centre line. On an axis-aligned floor
that is the distance to the walls measured across the corridor, so "left of
the centre line" is exactly cross(move, gradient of distance-to-obstacle)
< 0. `test_traffic.py` checks it against the Euclidean field. One run
extraction per axis gives width, axis and offset together. A distance
transform would take ~250 shifted passes and a width estimate on top. The
price: a corridor more than ~40 degrees off the grid reads wider than it is,
so it gets no lanes and no passage. Unknown cells count as free, as for the
planner.

### Keep right (`traffic.keep_right`)

A step's cost multiplier gets a lane term, per direction:

    weight * corridor * along * clamp((sigma + lane_offset_m) / (2 * lane_offset_m), 0, 1)

- `sigma` is the cell's offset from the centre line, positive to the left of
  the move. The term is 0 from `lane_offset_m` (0.4 m) right of the centre
  outwards and `weight` (1.0) from 0.4 m left of it. So the right lane costs
  what the aisle cost before, and routes don't change: only the side does.
- `along` is how much the move runs with the corridor: 1 along it, 0.71
  diagonal, 0 across. Crossing an aisle costs nothing extra.
- `corridor` is 1 where the free width is above `single_lane_m` (1.3 m) and
  at most `max_width_m` (3.2 m). It fades to 0 by 3.6 m, and with length /
  width from 3 down to 1. So the 2.4 m aisles and the 3.0 m end gaps get
  lanes. Junctions, the apron and single-lane aisles get none.

A* reads one multiplier array per direction (`Costmap.use_step_arrays`).
Off, all eight are the costmap's own `wint`, so the search is the one it
was. The lane term only adds, so the octile heuristic stays a lower bound.

**The smoother.** A shortcut may run no further left at any point than the
A* path it replaces was at the same fraction along, plus half a cell, unless
it stays inside the zero-cost lane (`Lanes.follows`). The first rule tried
bounded a shortcut by its two ends, and that was not enough. Shelves sit on
the aisle's centre line, so a leg from shelf to shelf pulled taut ran straight
down the middle, and the lanes A* had chosen were gone. With half a cell also
allowed past the lane's edge, lanes drifted 0.2 m toward the centre.

**The clearance check (`_blocking`'s `need`).** The local layer stops for an
obstacle whose centre comes within body + its radius + `clearance_margin_m`
= 0.6 m of the path ahead. In a 2.4 m aisle the outer cells of the flat band
are 0.5 m either side of the centre, so two robots going opposite ways run
1.0 m apart. For a peer known from its Status (its centre) that agrees. For a
peer the camera sees, it nearly does. A detection is the nearest point seen
on the peer's body, used as the centre of a body-sized disc, with a velocity
from consecutive such points. As the peer slides past, that point swings
around its body, and `predict_s` (0.5 s) throws it sideways toward our lane.
The meeting then costs one 0.1-0.2 s stop. With `nav.predict_s: 0` it costs
none. The sensor stand-in has this artefact in every run, not only with
lanes. A tracker that estimates the centre would remove it.

Two robots sent past each other down a 2.4 m aisle, four start/goal pairs
on and near the shelf line (scratch runs;
`test_two_robots_meeting_head_on_pass_without_a_stop` keeps the first):

| | stops | seconds stopped | both arrived by |
|---|---|---|---|
| off | 7-12 | 7.1-11.5 s | 26.4-29.5 s |
| keep right | 1-2 | 0.2-0.3 s | 20.3-23.1 s |

The field follows the belief. The costmap's window updates tell `Lanes`
which rows and columns have walls that changed. The runs there are re-cut
before the next plan, at most every `keep_right.refresh_s` (1 s). The lane
term only steers, and lethal cells stay exact.

### Single-lane passages (`traffic.passages`)

**What a passage is.** A connected (8-way) region of cells seen free whose
free width is at most `traffic.single_lane_m`, with at least one cell a robot
can stand on. Seen free, not just unknown: unknown counts as free for the
width, but a blind robot that has seen both faces of a 1.2 m rack read its
unknown inside as a 0.8 m lane. On `fleet_scale` robots claimed and held for
slivers like that (first bench, 2026-11-26). Why 1.3 m: the local layer
wants 0.6 m between centres, and each centre keeps a lethal band off the
walls, so the 1.0 m narrow aisles are single-lane and 1.4 m is not. On
`bench_narrow` that is the 12 narrow segments, each 11.6 x 1.0 m. On
`bench_dead_end`, six of them are bays. The plain floor has none. A passage
is named by its box (`PassageMap`). Two robots' maps can cut one passage
slightly differently, and claims match when the boxes overlap.

**The claim.** When a robot's path enters a passage within `claim_m` (5 m),
or the robot stands in one, it claims it: (box, claim time, task class at
claim time, inside). The claim that comes first holds the passage. A robot
inside comes first: it can't wait outside. After that the earliest claim
wins, then the task class (carrying, fetching, other), then the robot index.
Claim times are the sender's clock, not offsets re-based on arrival (as
intents are). With re-based times, two near-simultaneous claimants can each
see the other as earlier and both wait. Every robot must compare the same
numbers for the order to be the same everywhere. Clock skew only changes who
wins.

**On the wire.** Claims ride on `/fleet/status`, on every Status while they
last, best-effort, 16 B each. No frames of their own: intents sent as their
own message took the 50-robot radio down (Path intents, above), and airtime
is per frame far more than per byte. Not reliable: the claim has to be
repeated anyway so that a dead holder's claim can lapse. A lost one costs
one Status period (0.2 s at 5 Hz), and a reliable claim would still need
the refresh. A Status without the claim releases it at once. A peer's
claims lapse `ttl_s` (2 s, 10 Statuses) after its last Status. With
`status.recipients: radius`, only robots within `status.radius_m` hear a
claim. A holder at the far end of a passage and a claimant `claim_m` short
of the near end are the passage's length plus `claim_m` apart, so with
15 m and 5 m a passage longer than 10 m can be entered from both ends. The
bench floors' are 11.6 m, and their scenarios send Status fleet-wide. Under
`info.peers: truth`, claims are handed over every step instead.

**The hold.** A robot may enter only if its claim comes first and is
`settle_s` (1 s) old, so that an earlier claim has time to arrive.
Otherwise it stops `standoff_m` (2.5 m) along its path before the entry,
and stands still as an intent yield does (`Robot.pass_hold`). It never
stops inside a passage, or in the gap just past one. A hold point that lands
there is the earlier passage's, so the robot waits before both until both
are its own. At 1.2 m, a robot waited in the 2.4 m crossing between two
narrow segments, in the mouth that the holder had to come out of. The holder
couldn't get past it, gave its leg up after `block_fail_s`, and the waiter
waited 90 s.

**The cases:**
- **Two robots arrive at both ends at once.** Both claim, and both hear
  both claims. The order is the same at each robot, so one goes and one
  waits. Claims made in the same 0.1 s step tie on time and fall to task
  class and index.
- **A robot wants a passage it already stands in** (after a pick in a bay,
  a reboot, a lost claim). It claims as inside, and that comes first.
- **A dead-end bay** is entered and left the same way. The claim stands
  while the robot is inside, whatever its path or task does. The leg in
  ends inside, and the leg out exits, which releases it.
- **A dead holder.** Its Status stops, and its claim lapses `ttl_s` later.
  If it died inside, the passage is still physically blocked. The next
  robot meets it and is left to the local layer, as today (Benchmark
  scenarios, finding 4).
- **A partition between claimants.** Neither hears the other, so both go
  in. Head-on inside, the local layer has them as before reservations: stop,
  replan, give up after `block_fail_s`. No deadlock comes from the
  reservation: nobody waits on a claim it can't hear, and claims heard
  before the split lapse after `ttl_s`. A one-way loss works the same way.
  If the robot that hears is second, it waits; if it is first, both go.
- **The path changes after claiming.** A new path that no longer enters the
  passage within `claim_m` releases it. A failed plan (a goal and no path)
  keeps its claims. Releasing there put the robot at the back of the queue
  on every retry.
- **A holder that never leaves** (parked inside with `nav.return_home_s:
  null`, stuck). After `hold_max_s` (120 s) on one holder, the waiter stops
  deferring to it for as long and goes in. This is a backstop, not a
  schedule. A bay in and out is ~46 s at full speed plus the pick, and at
  60 s a waiter gave up 0.8 s before the holder came out.
- **A robot stopped by the local layer when its hold starts.** That stop is
  closed first. A held robot isn't checked for blockers, so the open stop
  would have booked the whole hold as a stop for a robot.

**Events and metrics.** `nav` events `pass_claim`, `pass_release`,
`pass_enter`, `pass_exit`, `pass_hold` (to the holder; to itself while the
claim settles), `pass_hold_end` and `pass_hold_timeout`, each with the
passage's `box`. The `nav` metric v3 adds `pass_holds`, `pass_hold_s` and
`pass_hold_timeouts`. `utilisation` v2 books held time as `lost_passage_s`;
like an intent yield, it wins over a stop. The replay doesn't mark passage
holds on its timeline yet.

### Cost

Measured with timers around each part, bench_rush 20 robots and
bench_dead_end 20 robots for 600 s, fleet_scale 50 robots for 300 s, both
on. Wall times on this shared laptop move 20-30 % from run to run, so the
per-call costs below are the measure.

- **A\*:** +5 % per plan on the plain floor (2.42 → 2.54 ms), +8 % on the
  50-robot floor. The scratch micro-benchmark (120 fixed shelf/dock legs,
  true map) gives +6 % on the plain floor and +10 % on the narrow one. On
  bench_dead_end it is cheaper per plan (7.5 → 5.0 ms), because fewer
  searches flood the floor and fail. There is no measurable cost with the
  switch off (4.50 → 4.50-4.52 ms).
- **Smoother:** 0.8-1.0 → 1.3-1.4 ms per path, 0.2-0.7 s more per 600 s at
  20 robots.
- **Upkeep with a true map** (every bench scenario): nothing after the
  first build, which takes ~22 ms per robot for the lanes and ~1.5 ms for
  the passages.
- **Upkeep with a blind start** (fleet_scale, prior none). The belief
  changes on almost every scan. Every costmap window update rewrites the
  direction arrays (0.04 ms, ~2.5 times a second per robot). The lanes are
  re-cut at replans, about every 12 s per robot (2.7-3.3 ms each). The
  passage map is re-cut every second (0.6 ms). Over 300 s of 50 robots,
  keep right cost ~7 s of CPU and passages ~11 s, on a 47 s run.
- **Memory with keep right:** 8 direction arrays, 4 lane terms, the field
  and the runs, ~4.7 MB per robot on the default floor (60 k cells), ~7.8 MB
  on `fleet_scale`'s (100 k).

### Measured

`fleetsim bench traffic --jobs 6`, seeds 1-3: 168 runs, no crashes
(`data/fleetsim/bench/20261127T021323Z_traffic_7151f6d`). `fleet_scale`'s
passages and both runs were re-run after the seen-free fix
(`..._c741974_fleet_scale`). Cells are means over the seeds, off → keep
right. "Stops" and "stopped s" are the `nav` metric's `blocked_robot` and
`blocked_robot_s`. "Lost" is `utilisation`'s `lost_robot_s`: stopped for a
peer while working a task, up to the next state change, so it also counts
stops that ended in a give-up. Contacts were 0 in every run except `walkers`
(11.7 → 9.0, walkers walking into robots) and `fleet_scale` (1.3 either
way).

**Keep right, open floor:**

| Scenario | Robots | per hour | mean s | stops | stopped s | lost s | gave up |
|---|---|---|---|---|---|---|---|
| `bench_steady` | 10 | 87.0 → 86.7 | 172 → 176 | 166 → 150 | 184 → 154 | 144 → 101 | 0.7 → 0.3 |
| `bench_steady` | 20 | 173 → 185 | 161 → 162 | 844 → 742 | 955 → 802 | 713 → 499 | 2.7 → 1.3 |
| `bench_steady` | 40 | 334 → 342 | 172 → 165 | 4044 → 3220 | 4307 → 3416 | 3297 → 2420 | 7.0 → 9.0 |
| `bench_rush` | 10 | 200 → 204 | 353 → 359 | 453 → 326 | 457 → 273 | 459 → 274 | 0 → 0 |
| `bench_rush` | 20 | 392 → 405 | 377 → 359 | 2011 → 1303 | 2032 → 1222 | 2030 → 1237 | 0 → 0.3 |
| `delivery_smoke` | 8 | 113.4 → 111.7 | 217 → 224 | 44 → 51 | 36 → 33 | 19 → 19 | 0 → 0 |
| `walkers` | 12 | 170 → 174 | 180 → 190 | 204 → 185 | 191 → 178 | 183 → 165 | 0 → 0 |
| `fleet_scale` (radio, blind) | 50 | 443 → 502 | 213 → 202 | 3798 → 3142 | 4219 → 3316 | 4434 → 2825 | 11.3 → 0.3 |

Seed-to-seed sd: `bench_rush` 20 throughput ±3.4 either way, `bench_steady`
20 ±17-19, `fleet_scale` ±56 → ±17. `delivery_smoke` has a fixed task list,
so its three seeds are one run.

**Single-lane floors, all four variants** (off / keep right / passages /
both). At 5 robots the variants are within 0.3 per hour and 4 s of each
other, and nothing is given up.

| Scenario | Robots | per hour | mean s | stops | lost s | held s | gave up |
|---|---|---|---|---|---|---|---|
| `bench_narrow` | 10 | 113 / 118 / 116 / 113 | 184 / 181 / 184 / 181 | 287 / 186 / 287 / 191 | 244 / 133 / 251 / 162 | 0 / 0 / 27 / 38 | 0 / 0.3 / 0.3 / 0.7 |
| `bench_narrow` | 20 | 233 / 236 / 229 / 236 | 186 / 175 / 188 / 176 | 1443 / 989 / 1404 / 975 | 1288 / 736 / 1283 / 750 | 0 / 0 / 259 / 213 | 1.0 / 0.7 / 0.7 / 1.0 |
| `bench_dead_end` | 10 | 114 / 117 / 114 / 115 | 194 / 190 / 196 / 183 | 341 / 215 / 299 / 232 | 421 / 310 / 320 / 220 | 0 / 0 / 81 / 59 | 3.0 / 2.3 / 1.0 / 0 |
| `bench_dead_end` | 20 | 195 / 204 / 228 / 231 | 239 / 208 / 211 / 184 | 1837 / 1311 / 1846 / 1174 | 5634 / 4367 / 2014 / 1027 | 0 / 0 / 344 / 288 | 66 / 55 / 1.0 / 0.3 |

`bench_dead_end` at 20 robots also ran faster with passages: 188 → 95-112 s
of wall time, because far fewer searches flood the floor and fail.

**Checks over the bench.** In the 66 runs with passages on: 1856 passage
entries, 332 holds, never two robots inside one passage at once, and no
hold timed out. On the floors without a single-lane passage, the switch
changes nothing: 24 runs are event-identical to `off`, and `both` is
event-identical to `keep_right`.

**What this shows:**
- **Keep right cuts robot stops wherever robots meet in two-way aisles.**
  Stops fall 9-37 % and time lost to peers 10-45 %. On `bench_rush` at 20
  robots, throughput rose 3.3 % (about 4 times the seed-to-seed sd) and mean
  delivery fell 5 %. In its seed 1, stops in junctions, cross-aisles and
  aisles fell from 1894 to 868, and on the apron they rose from 280 to 326.
  The 50-robot radio fleet gained the most: legs given up 11.3 → 0.3, and
  throughput +13 %.
- **It doesn't touch the apron**, which has no lanes, and the apron is where
  `delivery_smoke`'s stops are. There the cross-aisle stops fell from 14 to
  3, but a changed arrival order added 14 on the apron and 6 at junctions,
  and mean delivery rose 7 s. Underloaded (`bench_steady`), throughput is the
  arrival rate, and mean delivery moved within its noise.
- **Passages fix the dead-end bays at 20 robots.** Legs given up went 66 →
  1, throughput +17 % and mean delivery -12 %; with keep right as well,
  +19 % and -23 %. Holding cost 290-340 robot-s per run, against ~3600-4600
  robot-s of time lost to peers removed.
- **On `bench_narrow` they change little.** Robots rarely meet inside the
  narrow aisles: at 10 robots, 10 s of the 364 s of robot stops were in
  them, and most were at junctions and on the apron. Keep right does more
  there (-31 % stops at 20 robots).
- **Holding robots are obstacles too.** They wait in cross-aisles and
  junctions, and others stop for them. That is most of what is left in
  `bench_dead_end`'s stops with passages alone.

**For the owner to decide:**
- **The camera stand-in's detections.** A detection is the nearest point
  seen, with a velocity from consecutive such points. It costs every passing
  meeting a short stop, with lanes or without. Whether to report a centre
  estimate, or track it as obstacle_detector does, is a sensing-model
  change for all runs.
- **Lanes and passages need corridors near the grid axes.** A saved RTAB-Map
  map is in the robot's start frame, so its aisles can be at any angle. The
  robots would need a rotated or Euclidean corridor field.
- **A passage is held by one robot at a time**, even when the next robot
  goes the same way, and claims don't steer A*: a robot waits rather than
  take the long way round. Direction-aware claims, or a temporary cost on
  held passages, are the next levers.
- **On the robots** the claim would be a field on `/fleet/status`
  (`fleet_radio.encode_status`), and claim times compared across robots
  need clocks synced to well under the 1 s `settle_s`.
- The operator view's run form doesn't offer the two switches
  (`webui/fleetsim_runner.py`, `SETTABLE`), and the replay doesn't mark
  passage holds. Both are terminal-only for now.

## Local avoidance

One switch (2026-11-26, `fleetsim/local/`; item A7 in
[the plan](fleetsim-plan.md)): `nav.local` picks the local layer, the part
that decides each step what a robot does about what is near it.
`stop_replan` (the default) is the layer the robots always had in fleetsim,
moved out of `robot.py` unchanged. `orca` is Optimal Reciprocal Collision
Avoidance. The global planner, the path follower, the intent and passage
holds and the contact model stay where they were. With the default, a run
is bit-identical to one from before the seam: the `events.jsonl.gz` of
`delivery_smoke`, `walkers` and 600 s of `bench_dead_end` at 5 robots were
compared before and after, ignoring `wall`, `wall_s` and `summary`, and
were identical. `test_stop_replan_reproduces_the_event_stream_from_before_the_seam`
keeps a digest of a 6-robot run with a walker, stops and replans, from the
tree before (b85effd). The robots don't do ORCA.

```bash
/usr/bin/python3 -m fleetsim run bench_rush --robots 20 --set nav.local=orca
/usr/bin/python3 -m fleetsim bench local --jobs 4    # both, with and without keep right, 3 seeds
```

### The seam

While a robot drives a path and no hold, pause or recovery stops it,
`Robot._drive` asks `robot.local.decide(dt, now)` once per step and does
what the answer says:

| Decision | What the robot does |
|---|---|
| `DRIVE` | follows the path at the follower's own speed (`_follow`, as before) |
| `DRIVE` with `vel=(vx, vy)` | tracks that world-frame velocity (`_track`, below) |
| `STOP`, `by` | stands still this step. `by` (robot, actor, notice, bump) is the cause the metrics book |
| `STOP`, `by`, `around` | the same, after it marks the `around` discs lethal for `overlay_ttl_s` and replans around them |
| `GIVE_UP`, `by` | the leg fails, as a Nav2 abort |

The robot keeps the books, so both methods are measured the same way. A
`STOP` opens a stop if none is open (the `blocked` event), a `DRIVE` closes
it (`unblocked`), a `GIVE_UP` records `gave_up` cause blocked. A method is a
class per robot with `decide(dt, now)` and `reset()` (the robot calls it when
a stop closes and when its goal is cleared). It reads what the robot knows:
`perceive()` (camera detections with a velocity, bumps, peers heard on
Status, peers' obstacle notices), the pose, path and costmap, the path
follower's `preferred_velocity()`, and `track_step()`, the step a velocity
would really take. It never moves the robot or writes the costmap itself.
The docstring of `fleetsim/local/__init__.py` is the full contract, and how
to add a method.

`_track` drives a velocity the way a diff drive can. It turns toward it at
`yaw_rate_rps`, drives the part along its heading (none while more than 30
degrees off, the follower's rule) and accelerates at `accel_mps2`. The robot
then moves along its heading, not along the path, and its progress along the
path follows where it got to. It never drives from a cell its costmap holds
free into a lethal one. When that guard stops it, it replans from where it
stands (cause `guard`): off its path, the straight line to the next path
point can clip a rack's corner.

### ORCA (`nav.local: orca`)

van den Berg et al.'s ORCA in the RVO2 library's formulation (its
`computeNewVelocity`, `linearProgram1-3`), ported to pure python
(`fleetsim/local/orca.py`). RVO2 is C++, and fleetsim stays on the system
python3 with numpy. Each step:

1. **The preferred velocity** is the path follower's: toward the next path
   point, at the speed `_follow` would drive.
2. **Neighbours** are what the robot knows, never ground truth unless
   `info.peers: truth`. A peer heard on Status is its centre, carried along
   the twist that Status carries for the Status's age. `fleet_radio.encode_status`
   always carried `twist.vx` and the yaw, and the bytes were already priced;
   the sim now also delivers them (`Sim.seen_v`). A peer the camera sees
   whose Status is stale, and a mover, are the nearest point the camera saw,
   with the camera's velocity. Notices about movers count too. The nearest
   `max_neighbors` (10) within `neighbor_m` (5 m) are used.
3. **One half-plane per neighbour**, from the velocity obstacle for
   `time_horizon_s` (4 s). A peer moving faster than `reciprocal_mps` is taken
   to be avoiding too, and gets half the correction. A peer standing still
   (parked, picking, held, dead) and every mover get all of it: walkers never
   yield.
4. **Walls stay with the global planner.** The path is already clear of them,
   so ORCA only needs a guard against a dodge into one. Each occupied cell
   (or overlay) within `wall_m` (1 m), the nearest per 22.5 degrees, is a hard
   half-plane: no faster toward it than (distance - lethal radius) /
   `wall_horizon_s` (0.5 s). The bound is never tighter than the preferred
   velocity's own approach, so it never fights the path (a dock against a
   wall), and it always allows standing still. A bump is a half-plane too: no
   velocity toward it.
5. **The velocity** closest to the preferred one inside the max-speed circle
   that every half-plane allows (RVO2's 2D program). When nothing is allowed,
   RVO2's 3D fallback trades the neighbours' half-planes off and keeps the
   wall ones.

The knobs, under `nav.orca` (base.yaml):

| Key | Default | What |
|---|---|---|
| `time_horizon_s` | 4.0 | keep clear of peers and movers this far ahead |
| `wall_horizon_s` | 0.5 | and of walls (only a velocity off the path is held back) |
| `neighbor_m` | 5.0 | what is closer counts as a neighbour |
| `max_neighbors` | 10 | the nearest this many |
| `margin_m` | 0.1 | centres keep body + its radius + this apart (= `clearance_margin_m`) |
| `wall_m` | 1.0 | occupied cells and overlays this close are wall half-planes |
| `reciprocal_mps` | 0.1 | a peer moving faster than this takes half the correction |
| `track_s` | 1.0 | what the camera saw stays a neighbour this long after it left the cone |
| `stall_mps` | 0.05 | a chosen speed below this is a stop |
| `resume_mps` | 0.1 | stopped, drive on once the chosen speed is above this |

When nothing is near and the robot is on its path, it drives the path
exactly as `stop_replan` does. When the chosen speed falls under
`stall_mps` (and until it is back over `resume_mps`), the robot stands still
and books a stop for the neighbour that most excludes its preferred
velocity. From there on, the backstop is `stop_replan`'s: stopped
`block_replan_s`, it marks that neighbour and what is chained to it and
replans around them; stopped `block_fail_s`, it gives the leg up. That is
what clears a peer parked across a single-lane aisle, which no velocity can
pass.

**The diff-drive approximation.** ORCA assumes an agent that takes the new
velocity at once. Here the robot turns toward it and drives along its
heading, so it lags. `margin_m` (0.1 m) on top of both radii is the slack
for that lag: RVO2's radius inflation, the simplest form of NH-ORCA's
tracking-error bound. Two checks run on the step the robot will really take
(`track_step`), whatever ORCA asked for. It never closes on a neighbour to
within contact distance (body + its radius + 2 cm): it stands still
instead, booked as a stop for that neighbour. And `_track`'s guard keeps it
out of lethal cells. Inside the comfort distance (body + its radius +
`margin_m`), RVO2 would demand the whole separation within one step, a
sideways jump a diff drive can't make. Here the disc shrinks to just under
the distance instead, so ORCA forbids closing in and nothing more.

**Fixes found in testing (`walkers` and the first bench, 2026-11-26).**
- **Camera tracks.** The scan replaces the robot's detections every 0.2 s.
  A walker at the edge of the cone came and went every scan, so the robot
  turned toward its dodge and back toward its path in turn, and never moved
  off the walker's line: 4 driven contacts in seed 1. What the camera saw now
  stays a neighbour for `track_s` (1 s) after its last sighting, carried
  along its velocity, as obstacle_detector's tracks outlive a frame. Driven
  contacts: 4 → 1.
- **The contact veto.** A walker coming head-on at 1.2 m/s, faster than the
  robot can turn away, leaves no safe velocity. The fallback's least-bad
  one then took a small step into the walker. The veto above took the last
  driven contact of that run to 0. The first bench then found one more,
  with the walker in view: close up, the camera stand-in read a walker
  pacing along x as moving at (-2.05, +1.69) m/s, the next scan missed it,
  and the track it left was carried 0.3 m away from where the walker was.
  The veto now also checks each camera track where it was last seen.
- **Bumps with nothing in view.** A bump with no neighbour perceived fell
  through to plain path following, so the robot pushed on into what it had
  hit. A bump now always goes through the program (no velocity toward it).

**What it can't see, it can't avoid.** A walker crossing square on at 1.2 m/s
approaches a 0.5 m/s robot at 67 degrees off its heading, outside the
87-degree cone. Neither layer sees it until it is beside the robot, unless
a peer's notice reports it. `test_orca_avoids_a_walker_crossing_in_front`
has the walker cross diagonally from ahead, where the camera has it.

**Cost.** The neighbour loop and the linear programs are pure python on a
handful of floats. The wall candidates are cached per (cell, costmap
version) and binned with numpy. Under the profiler, `decide` took 4.3 s of
a 48 s `bench_narrow` run at 20 robots (600 s).

### Measured

`fleetsim bench local --jobs 4`, seeds 1-3: 108 runs, no crashes
(`data/fleetsim/bench/20261127T035711Z_local`, commit 8319c1e). The ORCA
runs of `bench_walkers` and `walkers` were run again after the last veto fix
(`..._20261127T041645Z_local`, 5053df2, 24 runs); the fix changed no run without
movers (`bench_rush` 5 robots, seed 1: event-identical). Cells are means over
the seeds, stop_replan / stop_replan with keep right / ORCA / ORCA with keep
right. "Robot stops" is the `nav` metric's `blocked_robot`. "Lost to robots"
and "lost" are `utilisation`'s `lost_robot_s` and `lost_s` (every cause).
"Driven" is `contacts.driven`. No run had a robot-robot or a robot-wall
contact. The stop_replan cells of `bench_steady` 20, `bench_rush` 20 and
`bench_narrow` 20 are the numbers the traffic bench measured
([above](#traffic-keep-right-and-single-lane-passages)), so the seam changed
nothing there either. "Wall s" is from the first bench, where every
variant ran side by side.

| Scenario | Robots | per hour | mean s | robot stops | lost to robots s | lost s | actor contacts | driven | gave up | wall s |
|---|---|---|---|---|---|---|---|---|---|---|
| `bench_steady` | 5 | 42.8 / 42.4 / 42.9 / 42.5 | 171 / 180 / 169 / 177 | 35 / 43 / 0 / 0 | 18 / 26 / 0 / 0 | 18 / 26 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 10 / 11 / 10 / 11 |
| `bench_steady` | 20 | 172.5 / 185.3 / 190.3 / 187.3 | 161 / 162 / 141 / 146 | 844 / 742 / 8 / 7 | 713 / 499 / 32 / 20 | 714 / 500 / 32 / 21 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 2.7 / 1.3 / 0 / 0 | 46 / 48 / 45 / 46 |
| `bench_rush` | 5 | 106.1 / 106.1 / 110.3 / 107.2 | 342 / 343 / 332 / 340 | 97 / 56 / 4 / 1 | 108 / 59 / 3 / 2 | 108 / 59 / 4 / 2 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 20 / 23 / 20 / 23 |
| `bench_rush` | 20 | 391.9 / 405.0 / 447.4 / 444.4 | 377 / 359 / 325 / 330 | 2011 / 1303 / 98 / 50 | 2030 / 1237 / 104 / 59 | 2031 / 1238 / 105 / 60 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0.3 / 0 / 0 | 83 / 84 / 86 / 88 |
| `bench_walkers` | 5 | 42.7 / 42.4 / 42.8 / 42.5 | 175 / 183 / 172 / 179 | 46 / 36 / 0 / 0 | 23 / 21 / 0 / 0 | 45 / 42 / 5 / 7 | 2.3 / 5.3 / 2.7 / 5.0 | 0.3 / 1.0 / 0.7 / 0.3 | 0 / 0 / 0 / 0 | 13 / 13 / 13 / 14 |
| `bench_walkers` | 20 | 187.2 / 177.9 / 190.1 / 189.3 | 162 / 164 / 144 / 149 | 833 / 751 / 25 / 34 | 614 / 582 / 45 / 30 | 712 / 694 / 67 / 52 | 25.3 / 34.0 / 20.3 / 27.3 | 2.0 / 2.7 / 0.7 / 3.0 | 1.3 / 1.3 / 0 / 0 | 47 / 50 / 48 / 50 |
| `walkers` | 5 | 94.3 / 88.0 / 96.3 / 92.1 | 255 / 260 / 246 / 254 | 46 / 44 / 0 / 0 | 44 / 38 / 0 / 0 | 71 / 68 / 7 / 3 | 5.3 / 5.3 / 3.0 / 3.7 | 0 / 0.3 / 0 / 0 | 0 / 0 / 0 / 0 | 15 / 12 / 12 / 13 |
| `walkers` | 20 | 192.5 / 194.7 / 199.0 / 194.8 | 157 / 155 / 142 / 146 | 345 / 295 / 27 / 4 | 295 / 207 / 52 / 18 | 357 / 265 / 64 / 30 | 9.7 / 15.0 / 11.7 / 13.0 | 0.3 / 0.3 / 0.3 / 0 | 0.3 / 0 / 0 / 0 | 22 / 23 / 22 / 22 |
| `bench_narrow` | 20 | 232.7 / 235.9 / 240.1 / 239.4 | 186 / 175 / 155 / 157 | 1443 / 989 / 68 / 70 | 1288 / 736 / 174 / 119 | 1289 / 737 / 175 / 120 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 1.0 / 0.7 / 0 / 0 | 62 / 62 / 57 / 60 |

Seed-to-seed sd of the throughput, in the same order: `bench_rush` 20
±3.4 / 3.4 / 5.5 / 2.5, `bench_narrow` 20 ±9.4 / 5.2 / 3.0 / 2.8,
`bench_steady` 20 ±17 / 19 / 16 / 13, `walkers` 20 ±34 / 32 / 29 / 32.
`bench_steady`, `bench_walkers` and `walkers` at 20 vary by arrivals far
more than by method. Wall time on the final code, side by side, seed 1:
`bench_walkers` 20 at 91 s either way, `bench_rush` 20 50 → 54 s.

**What this shows:**
- **ORCA removes almost every stop for a peer.** Robot stops fall 92-100 %
  at every size and floor, and time lost to peers 82-100 %. Robots pass
  each other, and parked or picking peers, without stopping and replanning.
  No leg was given up with ORCA in any run (stop_replan: up to 2.7 a run).
- **Overloaded, that is throughput.** `bench_rush` at 20 robots: 392 → 447
  deliveries per hour, +14 % (about 10 times the sd), and mean delivery 377
  → 325 s. Keep right alone gave +3.3 %. At 5 robots the floor is barely
  contended, and the gain (+4 %) is within the noise.
- **Underloaded, it is delivery time.** Mean delivery falls 10-17 % at 20
  robots on every floor: `bench_steady` 161 → 141 s, `bench_walkers` 162 →
  144 s, `walkers` 157 → 142 s, `bench_narrow` 186 → 155 s. At 5 robots,
  1-4 %. The throughput gains on the underloaded floors (`bench_steady` 20:
  +10 %) are within their seed-to-seed sd.
- **Single-lane aisles.** On `bench_narrow` a head-on meeting inside a
  narrow aisle still stalls both robots, and the backstop's replan around
  resolves it as before. The gain there is the two-way aisles and
  junctions between the narrow segments.
- **Keep right adds nothing on top of ORCA.** It cuts the few remaining
  stops further, but throughput is the same within noise and mean delivery
  is 2-8 s longer. ORCA already passes head-on meetings, so the lanes'
  extra distance is not paid back.
- **Walkers: no worse on safety.** Actor contacts are walkers walking into
  robots (they never yield), and are within noise across the four variants.
  Driven contacts are 0-3 a run for every variant. With ORCA, every one in
  the bench was a walker 44-99 degrees off the robot's heading, outside the
  camera's 43.5-degree half-cone, walking into the robot's side while it
  moved. `contacts.driven` books those to the robot when the walker's
  centre is in its front half-plane. Neither layer can avoid what it can't
  see.
- **Slow-downs are not booked as lost time.** `utilisation` counts stops.
  ORCA trades stops for slowing down and swerving, which show only as
  delivery time and throughput. Those are the fair comparison, and both
  improved.

**For the owner to decide:**
- **The robots don't run ORCA.** Their Nav2 controller and costmaps are what
  `stop_replan` stands in for. ORCA on the robots needs a controller that
  reads peers' twist from `/fleet/status` (already carried, at 5 Hz) and
  tracked movers with a velocity (obstacle_detector's tracks have one).
  These gains assume aligned frames and, in these benches, an ideal network.
  Status loss on a radio makes peers fall back to the camera's nearest
  point, which is a worse input for ORCA than for stop_replan.
- **Which local layer the other benchmarks use.** The default stays
  `stop_replan`, the robots as they are. Under ORCA, traffic costs far
  less, so the dispatchers (A1) would be compared on a different floor.
- **The camera stand-in's velocity.** ORCA leans on velocities more than
  stop_replan does, and the nearest-point stand-in's velocity is noise at
  close range (Known issues). A tracker that estimates the centre would
  help both layers.
- **A metric for slow-downs** (time driving well under the preferred speed)
  would book ORCA's cost the way stops are booked today.
- **The diff-drive mapping** is the simplest one: turn toward the velocity,
  drive along the heading, 30-degree rule, a radius margin. NH-ORCA's
  tracking-error bound, or choosing among velocities the robot can reach
  in one step, would be the next refinements.
- The operator view's run form doesn't offer `nav.local`
  (`webui/fleetsim_runner.py`, `SETTABLE`). It is terminal-only for now.

## Known issues

- **Intermittent interpreter faults on the development machine — cause not
  yet pinned down.** On the i9-14900HX laptop with CPython 3.14.4, long
  fleetsim runs occasionally fail inside the interpreter loop. One kind is
  a spurious `TypeError` (`'<' not supported between instances of 'Costmap'
  and 'float'` on `ng < gscore[j]`, with every local a float at that
  moment). The other is a segfault in `_PyEval_EvalFrameDefault`, seen in
  the full pytest run and in a genexpr in `planner.py`. Neither reproduces
  on demand. Both happen at the same simulation step only in some processes,
  and both are independent of `PYTHONHASHSEED` and `gc.disable()`.
  `PYTHONMALLOC=debug` found no heap corruption. Runs that don't fail are
  bit-identical, so the simulation itself is deterministic. The pytest
  suite without fleetsim never crashed (0/6 on `main`, 0/6 on this branch);
  with it, 6/20 — and two of those segfaults were inside commsim's own
  protocol tests (`protocol.py` `has_quorum`, pure python), which run
  before fleetsim's and only share a process with its imported modules.
  So the fault isn't in fleetsim's code; fleetsim's presence makes it show.
  Two explanations fit, and this machine alone can't separate them:
  1. a CPython 3.14 interpreter bug. [cpython#155145](https://github.com/python/cpython/issues/155145)
     (open, unconfirmed) reports the same symptoms on 3.14.6 from a pure
     stdlib hot loop, while 3.12 is clean;
  2. the CPU. 13th/14th-gen Intel Raptor Lake parts have a known
     voltage-degradation instability that shows up as exactly this kind of
     random crash under sustained single-core load. The #155145 reporter is
     also on an i9, and a contributor on AMD couldn't reproduce it.
  **2026-11-17: it happens under Python 3.12 too.** A scratch 3.12.15
  venv (numpy 2.5.5) segfaulted in 2 of 6 runs of commsim + fleet_tasks +
  fleetsim tests, both inside `planner.py`'s pure-Python A* loop, on the same
  machine where 3.14 faults at a similar rate. A CPython 3.14 bug can't
  explain a 3.12 crash, so the CPU (microcode 0x133 here; the update stops
  the degradation but can't undo it) is now the leading explanation. The
  remaining check is another machine.
  How to tell: run `python3 -m pytest fleetsim/tests` in a loop on another
  machine (CI uses 3.12). A* uses integer costs
  (`Costmap.wint`), which made the TypeError disappear in 15 runs. That may
  only have moved the fault elsewhere, and the segfaults since suggest it
  did.
  **2026-11-26: the kernel log points at one core.** Every python segfault
  the kernel logged on this laptop names core 20 (CPU 10 or 11): 10 of 10,
  2026-10-21 to 11-17 (`journalctl _TRANSPORT=kernel -g segfault`). Other
  programs' segfaults spread out: the 7 from `ros2` hit five other cores,
  which is what a software bug looks like. The other 6 (Firefox, libavcodec,
  apparmor_parser) put 3 on core 20. CPUs 10 and 11 have been offline since
  2026-11-26 03:32 (`/sys/devices/system/cpu/offline`). Since then a
  36-run and a 96-run bench crashed 0 times. The `fs-fake-segv*` lines of
  2026-11-26 are deliberate test faults, not crashes. Each run now records
  the core it is on and how hot it is, and `fleetsim bench` records the core
  every crashed run died on ([Benchmarks and crash logs](#benchmarks-and-crash-logs)).
  Check those against new crashes before reading the cause from the code.
- **Walkers never yield** (simtest's rule), so most walker contacts are a
  walker walking into a robot that stopped for it. `contacts.driven` counts
  only the ones where the robot moved into something ahead of it.
- **`map_divergence.final_stale` and `recipients: radius` or `subscribers`.**
  Stale counts every cell where a robot differs from the fleet's newest
  knowledge. With region-limited sharing a robot isn't *meant* to know
  distant tiles, so stale is large by design (`fleet_scale`: ~32 k cells per
  robot). Read it as "how far from one shared map", and use it as a pass
  criterion only with `recipients: all`. What region-limited sharing must get
  right is the robots a change matters to: `map_spread` measures that.
- **Notice extrapolation** is capped at 1 s. A full 3 s along a stale
  velocity planted phantom obstacles where a pacing walker never went.
- **A camera detection is the nearest point seen, not the body's centre**
  (2026-11-26). The local layer uses it as the centre of a body-sized disc,
  with a velocity from consecutive such points. As a peer slides past, that
  point swings around its body, and `nav.predict_s` throws it sideways.
  Two robots passing 1.0 m apart in keep-right lanes still stop for
  0.1-0.2 s; with `predict_s: 0` they don't
  ([Traffic](#traffic-keep-right-and-single-lane-passages)). ORCA leans on
  the velocity more, and close up it is noise: a walker pacing along x read
  as (-2.05, +1.69) m/s ([Local avoidance](#local-avoidance)).

## Not modelled yet

- Pose drift / localisation error, and with it the peer-masking radius.
- Intents on the robots (fleetsim's are above; the robots' `/fleet/intent`
  is a goal plus a straight corridor and `fleet_radio.should_yield`), and
  the explorer and exploration-from-scratch tasks (the explorer's logic still
  imports rclpy at module scope).
- Dual band, zenoh keepalives, hidden terminals (spatial contention is a
  load factor, not a medium).
- **Calibration against Isaac.** `nav.hiccup_rate_hz` (random recovery
  pauses) is 0 until Isaac task timings give it a value. Run the same task
  list in both and compare the `fleet_tasks` metric.
