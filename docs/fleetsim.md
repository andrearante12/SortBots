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
