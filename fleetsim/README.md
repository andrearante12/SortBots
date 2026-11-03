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
`score.json`. Exit code 0 = every pass criterion held, 1 = one failed,
2 = invalid scenario.

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
| Task auction, leases, commit, quorum, retry, restart adoption | **real**: `FleetTasks` + `commsim/core/protocol.py`, imported | `nodes/fleet_tasks.py` |
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

The body is a 0.25 m disc, half the length of the 0.5 × 0.4 m footprint.
The circumscribed 0.32 m disc made robots unable to pass each other in a
2.4 m aisle on a 0.2 m grid.

## Scenarios

`fleetsim/scenarios/*.yaml`. The schema is in `fleetsim/config.py`. Every
knob is in `configs/base.yaml`, with its source or tagged PLACEHOLDER.
Numbers below are seed 1 on an i9-14900HX laptop (single-threaded: fleetsim
uses one core).

| Scenario | Question | Seed-1 result |
|---|---|---|
| `delivery_smoke` | 8 robots, 12 tasks, ideal network: does it work at all? | 12/12 done, 0 contacts, 0 duplicates, ~80× real time |
| `map_sharing` | A pallet blocks an aisle; does sharing save the others finding it? | **on:** 0 stale cells, mean delivery 98 s · **off:** 147 stale cells, 107 s |
| `partition_heal` | 4 of 10 robots cut off for 60 s | all delivered, 0 duplicates |
| `walkers` | #18: walkers stepping out of rack aisles | robot-driven contacts 0–1 per run; **notices made no measurable difference** (seeds 1–4) |
| `fleet_scale` | 50 robots, radio, neighbour-only Status | see below |

## Findings so far

These came out of building this, and each one is worth a decision.

1. **A pick inside the lease window still sends a second robot**
   (`test_pickup_inside_the_lease_window_still_sends_a_second_robot`). An
   owner cut off from the majority must stop within `task_lease_s`, unless it
   picks the item up first, because a committed task never yields. The
   majority never hears the commit and reclaims the task. commsim's DES
   reports the same residual (0–2 duplicate episodes per partition run). A
   possible fix: don't start a pick without a quorum.
2. **A completion lost in a split is never repeated**
   (`test_a_completion_lost_in_a_split_is_never_repeated`). The cut-off owner
   delivers. Its `Complete` is lost, nothing resends it, and after the heal
   the robot that knows the task is DONE ignores peers' claims for it
   (`protocol.py` `_assert_owner` returns early for DONE). Robot after robot
   is sent to the empty shelf. The DES can't see this: its robots never find
   a shelf empty. A possible fix: answer a Claim or LeaseBeacon for a task you
   know is DONE with a Complete.
3. **Fleet-wide Status does not scale.** At 50 robots, all-to-all Status
   at 5 Hz is 12,500 unicast copies a second, and the radio model puts the
   channel at ~330 % load. That matches the DES's ~20-robot ceiling. Scale needs
   neighbour-only Status, and then the quorum needs zenoh-session liveness
   (`status.session_liveness`), which **`nodes/fleet_tasks.py` doesn't do
   today**. With Status neighbour-only, a robot never hears a majority's
   Status and can't claim anything.
4. **Parked robots wall off docks.** At 1.2 m spacing, an idle fleet on the
   apron blocked the route to the docks. Homes are now 1.6 m apart.
   Decentralised parking is itself a protocol question.
5. **Dead-end aisles deadlock** when many robots share one pickup behind a
   blockage. There's no traffic coordination beyond "lower priority replans
   first". Intents and yielding (`fleet_radio.should_yield` exists and is
   unused here) are the next nav-protocol step.

## Known issues

- **CPython 3.14.4 float fault.** With float costs, A* intermittently raised
  `'<' not supported between instances of 'Costmap' and 'float'` on
  `ng < gscore[j]`. Every local was a float at the moment it happened. It
  showed up about once in 5–10 full runs, at the same simulation step,
  depending on the process. It was not reproducible standalone, not
  hash-seed dependent, unaffected by `gc.disable()`, and the debug allocator
  (`PYTHONMALLOC=debug`) found no corruption. A* now runs on integer costs
  (`Costmap.wint`): 15/15 runs identical, no crash. Other float-heavy Python
  code could in principle be affected. CI runs 3.12. If you see odd
  TypeErrors on this machine, suspect the interpreter first.
- **Walkers never yield** (simtest's rule), so most walker contacts are a
  walker walking into a robot that stopped for it. `contacts.driven` counts
  only the ones where the robot moved into something ahead of it.
- **Notice extrapolation** is capped at 1 s. A full 3 s along a stale
  velocity planted phantom obstacles where a pacing walker never went.

## Not modelled yet

- Pose drift / localisation error, and with it the peer-masking radius.
- Intents and yielding (`/fleet/intent`, `fleet_radio.should_yield`), and
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
- Viewing a run in the operator view. Events are recorded in simtest's
  format, so a replay can be built without changing the format.
