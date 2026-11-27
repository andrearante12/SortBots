# fleetsim — benchmark groundwork plan (2026-11-26)

The working plan for getting fleetsim to the point where the fleet's task
allocation, traffic and comms can be compared against standard central and
decentralised methods (#50) and then optimised. Each item is sized to be one
session's (or one agent's) work; this file is what a new session reads instead
of the conversation that produced it. Update the status column as items land.

## The goal

Measure how close the decentralised fleet, on a realistic radio, gets to
standard central and decentralised methods, and improve it. "Close" is task
performance — throughput first (deliveries per hour), delivery time second,
both measured from the operator's request — at the radio cost each method
needs. Only 2 physical robots will exist; larger fleets are shown in fleetsim.

Two gaps get measured apart:
- **information gap** — the same algorithm with perfect, instant information
  (item A2) vs on the radio;
- **coordination gap** — a central planner that decides for the whole fleet at
  once (item A1, rolling Hungarian first) vs per-robot decisions.
Plus fixed references that don't depend on effort: a per-scenario lower bound
(A4) and, later, exact optima on tiny cases.

Throughput only separates methods when the fleet can't keep up; when it can,
every method delivers at the arrival rate and only delivery time differs. So the
benchmark has both overloaded and underloaded scenarios (C12).

Assumptions kept out of scope (calibrated later, separately): perception gives a
roughly correct 2D map, the e-stop and moving-obstacle detection roughly work.
What fleetsim must match is Nav2-like path behaviour and how a robot fills in
its map — checked against Isaac with a 2-robot task list after this groundwork.

## Status

| # | Item | Status |
|---|---|---|
| A1 | Swappable task-assignment layer (dispatcher), central greedy + rolling Hungarian | done |
| A2 | Perfect-information mode | done |
| A3 | Benchmark command (suites, seeds, sizes 2/5/10/20, one table) | done (`fleetsim bench`, `fleetsim/suites/smoke.yaml`; the real suite waits for C12) |
| A4 | Metrics: throughput, delivery time, time lost by cause, utilisation, lower bound | done |
| A5 | Crash-tolerant parallel runs + local compute logging | done (`fleetsim/sysmon.py`; CPU of death from /proc, since faulthandler hides the kernel's segfault line) |
| C12 | Scenario set (for review) + scenario editor in the operator view | done (scenarios drafted for review: `fleetsim/scenarios/bench_*.yaml`; editor in the operator view) |
| B8 | Map-tile subscriptions | done (`map_share.recipients: subscribers`, `fleetsim/mapshare.py`; measured in docs/fleetsim.md "Map sharing: subscriptions and event-driven updates": same deliveries, fewer bytes where the refresh fits its budget) |
| B9 | Event-driven map/intent updates | done (`map_share.events`; no measurable effect with exact sensing, off by default) |
| B10 | Keep right in aisles | done (`traffic.keep_right`, off by default; `fleetsim/traffic.py`, `fleetsim/suites/traffic.yaml`) |
| B11 | Single-lane passage reservation | done (`traffic.passages`, off by default; claims ride on Status) |
| C13 | Comms traffic monitor in the replay | done (replay's Traffic panel, Load on the floor, `record.net_nodes`; bench runs listed in the replay; docs/fleetsim.md "Comms traffic monitor") |
| A6 | Per-robot service times and capabilities | done (`robots.types`, tasks' `requires`; `fleetsim/scenarios/bench_mixed.yaml`, `fleetsim/suites/mixed.yaml`; docs/fleetsim.md "Robot types and capabilities") |
| A7 | Swappable local-avoidance layer (for ORCA) | done (`nav.local`, `fleetsim/local/`: `stop_replan` default, `orca` in pure python; `fleetsim/suites/local.yaml`; docs/fleetsim.md "Local avoidance") |


Later, separate tracks: Isaac calibration (2-robot task list), grid export for
PIBT/RHCR anchor runs, Ack/TaskQuery in the DES and emulation stubs, map
sharing on the real robots (Isaac side of #30), the comparison methods
themselves (#50) and idle behaviour (#51).

## Open decisions for the owner (from building the groundwork, 2026-11-27)

Each was raised while building an item and is described in that item's
section of `docs/fleetsim.md`. None blocks the benchmark.

Comparisons and metrics
- **Hungarian starvation (A1, A6).** When tasks outnumber robots, minimising
  total drive time can leave a far task waiting (one waited 286 s in
  `delivery_smoke`; p95 up to 500 s when a capability is scarce). The age
  term now exists (`dispatch.age_weight`, default 0). In `bench_rush` it cost
  11-13 % of deliveries and raised p95, and in `bench_burst` it changed
  nothing beyond 3 %. Keep it at 0, or measure p95 with undone tasks
  counted (today's p95 ignores the tasks that starve)?
- **Central dispatch decides every pick (A1, 2026-11-26).** Without that, a
  split or a lost Status sent two robots to one shelf (`bench_partition`,
  `bench_dead_gateway`). Now a robot waits at the shelf for the dispatcher's
  word and gives up at `task_lease_s`. That costs `bench_dead_gateway`
  2-5 % of per_hour and 33-54 s of mean delivery, and at 2 robots on the
  radio shelves out of range can't be picked at all (the auction has the
  same limit). `fleet_tasks.protocol.confirmed_picks: false` restores the
  old behaviour for comparison. Agree with it as the reference?
- **Central dispatch on a lossy radio (A1).** At 50 robots a lost Status makes
  the dispatcher declare a robot lost and re-plan its task, and both robots
  drive to it until the Cancel arrives (pick grants now stop the second pick).
  A longer loss timeout would cut the wasted driving.
- **Why central methods cause about twice the robot-caused stops (A1)** —
  not yet explained.
- **`per_hour` window (A4, done):** first request to last delivery, not the
  whole run. Revert is one line if unwanted.

Scenarios (C12)
- Which radio assumption the radio scenarios use: global contention, spatial
  contention (the drafts), or neighbour-only Status (`fleet_scale`).
- **The 20-robot radio cliff:** with fleet-wide Status the channel sits near
  0.7 load, and one disturbance tips it into a collapse it never recovers from
  (`/fleet/tasks` delivery to 0 %). A prime target for the comms scheme.
- Fixed relays/access points for 2-robot radio runs (a far robot is out of
  range of both the gateway and the other robot).
- Seeds or run length at 2 robots (2-6 tasks per run is too few to read).
- Spreading homes for small fleets (#51, idle behaviour).
- Requests during a gateway outage: wait (the drafts) or be refused.
- Two found gaps to fix, then make scenarios of: a rebooted robot that was
  carrying an item comes back idle and robots are sent to the empty shelf;
  a robot that never returns is never mapped as an obstacle.

Robot types (A6)
- Bids become "time until delivered" when types are set; on the real robots
  `service_s` must then be set fleet-wide. Agree?
- Stratify the capability draw per seed (the heavy share varied 11-29 %).
- 2-robot `bench_mixed` has no standard robot (every type gets at least one).

Map sharing (B8, B9)
- Redesign the refresh: it spends whatever patch traffic leaves of the rate
  cap and stalls when one tile outgrows the token bucket, so map bytes at 20+
  robots compare budgets, not policies.
- Phantom pallets (a gone pallet stays in maps until someone looks again):
  expire occupied cells that differ from the prior, or send a robot to look.
- Add a scenario where map news decides deliveries; no current one does.

Traffic (B10, B11)
- The camera stand-in reports the nearest point on a peer, not its centre,
  which still costs one short stop per meeting with keep right on; fixing it
  changes every run.
- Lanes and passages only see corridors near the grid axes; saved RTAB-Map
  maps can be rotated.
- Direction-aware passage claims, or a temporary cost on held passages.
- On the robots, claims would ride in `/fleet/status` and need clocks that
  agree well under a second.
- Keeping right costs ~3 % of each leg's drive time where robots rarely meet
  (small fleets: +8-14 s per delivery at 2 robots): lane changes are 45-degree
  steps the follower stops at, left turns cross back, corners are stepped.
  Easing lane changes into slants (`traffic.keep_right.lane_change_m`) takes a
  third off but costs 20 robots 18-26 % more stops, so it is off. Lanes on
  for every fleet size, or only past some density? (2026-11-27)
- Parked robots wall off docks 0-6 at 20 robots, and that, not the lanes,
  decides `bench_burst` (its throughput is the clearing rate, so one stuck
  task decides a run). The local layer marks a blocker for its replan with
  the body radius only, from the camera's nearest-point detection, so the
  plan goes back through the same 1.1 m gap. A wider mark ends the give-ups
  but one robot still livelocked behind the row. For A7 (the local layer)
  and #51 (where idle robots park).

Machine
- All 10 python segfaults in the kernel log (2026-10-21 to 11-17) were on one
  physical core (CPUs 10-11), and those CPUs were offline on 2026-11-26 by
  03:32. Running the tests on another machine settles it.

## Design notes per item

### A1 — dispatcher layer
Today every robot runs `FleetTasks` (the real auction) and the gateway runs one
more; `Sim._announce` calls `gw.request`. Make that one implementation of a
`fleetsim/dispatch/` layer chosen by `dispatch.method`:
- `auction` (default) — exactly today's code path. A run with it must stay
  bit-identical to before (compare event streams, as for the protocol switches).
- `central_greedy` and `central_hungarian` — a dispatcher at the gateway node.
  It assigns, robots execute: a dispatch or cancel travels gateway -> robot as a
  reliable `tasks` message over the network model, robot status comes back on
  `/fleet/status` (so a central dispatcher on the radio pays for its traffic and
  loses the fleet when the gateway is cut off — part of what's measured). The
  robots still run the same task FSM. `AWAIT_PICK` was skipped at first (a
  central planner as its own single source of truth). Since 2026-11-26 the
  dispatcher grants every pick, because a planner that can't reach a robot
  is not the source of truth for what that robot does.
- Rolling Hungarian: re-solve on events (task announced, robot finished, robot
  lost) over robots that are idle or not yet holding an item x tasks not yet
  picked; a robot's cost is the drive time from where it will be free (after its
  current carried task) to the pickup. Reassign an unpicked task only if the gain
  beats `reassign_gain_s` (hysteresis, or it thrashes). One queued task per robot
  per solve. No scipy on the system python3: a small O(n^3) Hungarian, tested
  against brute force on random matrices.
- Path costs for central planners come from the true static map (a warehouse's
  layout is known); blockages are known only through robots' reports unless A2.
- Keep the seam narrow so SSI, CBBA and the distributed Hungarian method (#50)
  can come in as per-robot agents later: the robot side talks to a small
  interface (`set_pose`, `tick`, `on_task_status`, `battery_pct`, `drive`,
  `owned()` for Status), which `FleetTasks` already provides.

### A2 — perfect information
`info:` knobs, each independent, plus a preset: `info.map: shared | truth` (every
belief follows the true grid each step, blockages included), `info.peers:
heard | truth` (peer poses from ground truth, not Status), `info.dispatch:
network | truth` (a central dispatcher reads true robot state and its messages
arrive instantly and always). With all three plus `network.model: ideal`, delay 0,
the run is the perfect-information reference.

### A3 — benchmark command
`python3 -m fleetsim bench SUITE` (`fleetsim/suites/*.yaml`): scenarios x robot
counts x seeds x variants (a variant is a named set of `--set` overrides, e.g.
`auction`, `auction_perfect`, `central_hungarian`, `central_hungarian_perfect`).
Each run is a subprocess (crash isolation), N in parallel (default: cores - 2).
Output `data/fleetsim/bench/<UTC>_<suite>/` with each run's dir, `results.csv`
(one row per run, every metric value), `summary.json` and a printed table:
rows scenario x robots, columns variant, cells mean ± sd over seeds of the
headline metric (throughput; delivery time beside it). Re-score a suite without
re-running (`bench --rescore DIR`), as runs already can be.

### A4 — metrics
- `fleet_tasks`: keep `per_hour` and `mean_s`/`p95_s` (from the announce, i.e.
  the operator's request); add `wait_assign_s` (announce -> dispatch),
  `wait_pickup_s` (dispatch -> item picked).
- new `utilisation`: share of robot-time idle / to pickup / at pickup (incl.
  AWAIT_PICK) / carrying / placing, and time lost by cause (blocked by robot,
  actor, notice; yielding; no path; awaiting the pick ack).
- new `bounds`: the sim emits one event per announced task with its lower bound
  (shortest true-map drive pickup -> dropoff at max speed + both dwells), and the
  metric reports `lb_mean_s`, `stretch` (mean of each delivery / its lower bound),
  `ub_per_hour` (fleet capacity if every robot did only lower-bound cycles) and
  `offered_load` (arrival rate / capacity: > 1 means overloaded).

### A5 — crash-tolerant runs, local logging
Each run writes `system.jsonl` into its own (gitignored) result dir from a
sampler thread, flushed per line every 2 s: CPU temperature (hwmon coretemp /
thermal zones), the frequency and id of the core the process is on, load
average, RSS, plus the python/numpy versions once. `faulthandler` writes a
stack dump to `crash.txt`. When a run dies by signal the bench runner records it,
pulls the kernel's lines for that pid (`journalctl -k`, if readable: a segfault
line names the CPU core — crashes clustering on cores point at the CPU) and
re-runs it (runs are deterministic, so a re-run is the same run). Report how
many runs crashed and on which cores.

### C12 — scenarios and editor
A benchmark suite of scenarios at 2, 5, 10, 20 robots, each underloaded and
overloaded, plus edge cases: blockages that appear and move, narrow
(single-lane) aisles, partitions, a dead robot, a dead gateway, walkers. The
synthetic warehouse gets options for narrow aisles. The scenarios are drafted
for review (the user has the context for the edge cases). Editor: a panel in the
operator view's Tests tab (operator view conventions, theme, operator_test.mjs):
the floor drawn from the scenario's world, tasks on a timeline, disturbances and
movers placed on the map, saved through a serve.py route that validates with
`config.resolve` and only writes `fleetsim/scenarios/<name>.yaml`, under the
same `--control` gate as launching runs.

### B8 — map-tile subscriptions
Receivers choose: each robot subscribes to the tiles near it and along its path
(`map_share.recipients: subscribers`), announces changes to its subscription
fleet-wide (small, reliable, rate-limited), and on a new subscription every
robot that has observed that tile sends it its snapshot (budgeted). Patches then
go to a tile's subscribers. Measure against `recipients: radius` and `all`.

### B9 — event-driven updates
When a robot's own sensing changes a cell on its own or a subscriber's path (a
blockage appears, a moved obstacle clears), publish that tile now instead of at
the next `publish_s` tick (min gap and the rate cap still apply), and with
intents on, re-announce the intent on the same Status.

### B10 — keep right
A* step cost gets a direction-dependent term in corridors: a move on the left
half of a corridor, relative to its own direction (cross(move, gradient of
distance-to-obstacle) < 0), costs extra, weighted by how corridor-like the cell
is (narrow free width, strong gradient).
The path smoother must not cut back into the left half. Expected effect: two
robots meeting head-on in a 2.4 m aisle are already on opposite sides and pass
without the local layer stopping either. Measure robot-caused stops.

### B11 — single-lane reservation
Cells whose free width can't fit two robots, grouped into passages. Before its
path enters a passage, a robot claims it from the robots near it (claim carries
claim time, priority, robot id; repeated while waiting/holding; expires if not
refreshed, so a dead robot frees it). The earliest claim holds the passage; the
others hold at the last point before the entry (the yield-hold mechanism) until
the holder's release. Needs narrow-aisle scenarios (C12).

### C13 — comms traffic monitor
In the replay: channel load over time (per node, recorded per net sample),
messages per second by kind, delivered vs lost, and one-hop links coloured by
how busy they are. Record per-node load in the `net` stream (one more field).

### A6 — per-robot service times and capabilities
Per-robot overrides (speed, dwell, battery) and capabilities; a task may require
one. Robots without it don't bid (auction) or aren't assigned (central). Needs
the capability carried with the task announce.
Built as robot types (`robots.types`: a share or count each, so they scale with
`--robots`). TaskAnnounce carries `requires` only when a task has one; FleetTasks'
cost function refuses it, protocol.py is unchanged. With types, bids and central
costs price the whole delivery at each robot's own speed and service times.

### A7 — swappable local avoidance
Move the local layer (stop for what's ahead, replan around after
`block_replan_s`, give up after `block_fail_s`) behind an interface so ORCA (RVO2,
Apache-2.0) can be tried in its place.
Built as `robot.local.decide(dt, now)` -> drive (along the path, or at a
velocity), stop (and replan around discs), or give up; the robot keeps the
books. ORCA is RVO2's formulation ported to python (RVO2 is C++), with peers'
velocities from the twist Status already carries, a wall guard instead of
obstacle polygons, and stop_replan's replan-around and give-up as its
backstop when it stalls.

## Ground rules for every item
- fleetsim runs on the system python3 (`/usr/bin/python3`), numpy + PyYAML only.
- Every new behaviour behind a config switch whose default reproduces today's
  runs exactly; prove it by comparing event streams before/after.
- Tests in `fleetsim/tests/` (and `tests/`, `webui/tests/operator_test.mjs` when
  touched). This laptop's CPU intermittently crashes long python runs: a failure
  only counts if it reproduces.
- Measure with several seeds before concluding anything (path intents looked
  good on one scenario and bad on another).
- Results and logs go under `data/` (gitignored), never into the repo.
