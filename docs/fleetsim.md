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
/usr/bin/python3 -m pytest fleetsim/tests               # ~10 s
```

System python3 with numpy and PyYAML (`requirements-dev.txt`), from the repo
root. Results land in `data/fleetsim/<UTC>_<name>/` (gitignored):
`scenario.json` (exactly what ran), `events.jsonl.gz`, `summary.json`,
`score.json`. To watch a run, see [Watching a run](#watching-a-run). Exit code 0 = every pass criterion held, 1 = one failed,
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
| Task FSM | stand-in with task_manager's states, dwell and status JSON | `nodes/task_manager.py` |
| Global planner | A* on the robot's belief, Nav2-style lethal band + exponential inflation (`planner.py`) | Nav2 NavFn |
| Local layer | stop for what's ahead within `stop_distance_m`, replan around after `block_replan_s` (lower priority first), give up after `block_fail_s` (`robot.py`) | Nav2 controller + costmaps |
| Perception | ray-cast cone, movers occlude and come back as detections with velocity, never mapped (`sensing.py`) | D435 + RTAB-Map + obstacle_detector + dynamic_obstacle_filter |
| Peer awareness | camera, `/fleet/status` over the network, obstacle notices; never ground truth | the same |
| Map sharing | tiles, deltas, sequence numbers, MapResync, round-robin refresh (`belief.py`, `sim.py`) | not implemented on the robots yet; this is the commsim brief's design |
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
**Tests** tab, and find the run under **Fleet sim runs**. You can also go
straight to one with `http://<console>/#replay=<run dir>`. The replay
covers the stage until you close it. None of it is the live fleet.

- **Floor:** the true floor. Waypoints are drawn as tape, the gateway as a
  triangle, and a blockage as a red box while it is in place.
- **Robots:** to scale, coloured by state (to pickup, carrying, idle, off). A
  dashed red ring means the robot can't reach the gateway: over the radio
  links if any were recorded, otherwise through a partition or a cut link.
- **Paths:** every robot's planned path. The selected robot's path is drawn
  dashed.
- **Links:** one-hop radio links, darker for a stronger signal. Radio runs
  only.
- **Own map:** with a robot selected, its *own* map replaces the true floor.
  Cells where it disagrees with the truth at that moment are red, so a
  blockage it hasn't learned about shows up red. Only robots listed in
  `record.belief_maps` are recorded.
- **Timeline:** trouble is ticked along the top (contacts, blocks,
  failures) and deliveries along the bottom. Disturbances are shaded bands.
  Click a tick to jump to it.
- **Info panel:** the task board, network loss and load, and the selected
  robot's task, how much of the floor it knows, how far it is behind the
  fleet's map, and its recent events.

Keys: Space plays or pauses. ←/→ steps 1 s (with Shift, 10 s). `[`/`]` jumps
to the previous or next event (only the selected robot's, if one is
selected). Esc closes. Drag to pan, scroll to zoom, double-click to fit.

**What is recorded for it.** These go in `record:` in `fleetsim/configs/base.yaml`,
and no metric reads them. The run's floor, waypoints, homes and gateway are
recorded once (`world`). Paths are recorded on every plan and replan. Radio
links are recorded every `links_s`. `belief_map` snapshots are taken every
`belief_map_s` for the robots in `belief_maps` (default `[robot_0]`; `all`
records every robot).

`fleetsim/replay.py` packs a run into one JSON document. Poses are stored as
flat arrays, and every stream is a list of time-ordered rows, so the page
finds the state at any moment by bisection. A 5-minute run of `fleet_scale`
(50 robots, radio) packs in 0.2 s to 2.5 MB, or 420 kB once gzipped.

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
- **Walkers never yield** (simtest's rule), so most walker contacts are a
  walker walking into a robot that stopped for it. `contacts.driven` counts
  only the ones where the robot moved into something ahead of it.
- **`map_divergence.final_stale` and `recipients: radius`.** Stale counts
  every cell where a robot differs from the fleet's newest knowledge. With
  region-limited sharing a robot isn't *meant* to know distant tiles, so
  stale is large by design (`fleet_scale`: ~32 k cells per robot). Read it
  as "how far from one shared map", and use it as a pass criterion only
  with `recipients: all`.
- **Notice extrapolation** is capped at 1 s. A full 3 s along a stale
  velocity planted phantom obstacles where a pacing walker never went.

## Not modelled yet

- Pose drift / localisation error, and with it the peer-masking radius.
- Intents on the robots (fleetsim's are above; the robots' `/fleet/intent`
  is a goal plus a straight corridor and `fleet_radio.should_yield`), and
  the explorer and exploration-from-scratch tasks (the explorer's logic still
  imports rclpy at module scope).
- Region *subscriptions* for map tiles. `recipients: radius` picks receivers
  by distance to the tile at send time, so a robot that later drives into a
  tile has missed older updates until a refresh reaches it.
- Dual band, zenoh keepalives, hidden terminals (spatial contention is a
  load factor, not a medium).
- **Calibration against Isaac.** `nav.hiccup_rate_hz` (random recovery
  pauses) is 0 until Isaac task timings give it a value. Run the same task
  list in both and compare the `fleet_tasks` metric.
