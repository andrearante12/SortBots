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
| C12 | Scenario set (for review) + scenario editor in the operator view | todo |
| B8 | Map-tile subscriptions | todo |
| B9 | Event-driven map/intent updates | todo |
| B10 | Keep right in aisles | todo |
| B11 | Single-lane passage reservation | todo |
| C13 | Comms traffic monitor in the replay | todo |
| A6 | Per-robot service times and capabilities | todo |
| A7 | Swappable local-avoidance layer (for ORCA) | todo |

Later, separate tracks: Isaac calibration (2-robot task list), grid export for
PIBT/RHCR anchor runs, Ack/TaskQuery in the DES and emulation stubs, map
sharing on the real robots (Isaac side of #30), the comparison methods
themselves (#50) and idle behaviour (#51).

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
  robots still run the same task FSM (`AWAIT_PICK` skipped: a central planner is
  its own single source of truth).
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

### A7 — swappable local avoidance
Move the local layer (stop for what's ahead, replan around after
`block_replan_s`, give up after `block_fail_s`) behind an interface so ORCA (RVO2,
Apache-2.0) can be tried in its place.

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
