# Scored sim tests: worlds, scenarios, test cases, metrics

How to put things in the warehouse, watch the robots deal with them live, and
turn that into a repeatable, scored test. For launching the sim at all, see
[`running.md`](running.md) and `scripts/sim_ctl.sh`; this builds on both.

```bash
scripts/simtest.py list                     # the test cases and what they run
scripts/simtest.py show crossing_yield      # the resolved plan, criteria, streams
scripts/simtest.py run crossing_yield       # launch, drive, score, stop (minutes)
scripts/simtest.py run crossing_yield --repeats 3 --set overrides.world=aisle_head_on
scripts/simtest.py rescore data/simtests/<dir>   # re-score recorded runs, no sim
scripts/simtest.py report DIR_A DIR_B       # compare two result dirs side by side
scripts/simtest.py metrics                  # the metric catalog
```

Or from the dashboard: the **tests** tab lists the test cases, runs one, and
shows verdicts and reports; switch to **live** while it runs to watch it.

## Three layers

| layer | file | answers | used by |
|---|---|---|---|
| **world** | `configs/worlds/NAME.yaml` | what's in the warehouse: props + scripted movers | scenarios, tests |
| **scenario** | `configs/scenarios/NAME.yaml` | how the stack runs: robots, map mode, explore, world, render knobs | the dashboard's launch form, `sim_ctl.sh`, tests |
| **test case** | `configs/tests/NAME.yaml` | what's asked (goals, tasks, stop), how it's scored (metrics + pass criteria), how often (repeats, variants) | `scripts/simtest.py`, the tests tab |

They stay separate on purpose. A world on its own is a **live demo**: pick it
in the launch form (or `moving_obstacles`) and click goals. A test only adds
the asking and the scoring on top, and can only change what its scenario
already lets the dashboard change (`overrides:`), so nothing in a test widens
what can reach a shell. `webui/session.py`'s `RUN_FLAGS` allowlist stays the
single gate.

The code is `simtest/` (pure python + PyYAML/numpy, importable from Isaac's
venv, the dashboard and the offline tests) plus `scripts/simtest.py`.

## Worlds

```yaml
name: aisle_crossing
scenes: [nvidia]            # which warehouse the coordinates were laid out for
autostart: true             # actors walk from sim start (live demos); tests reset them
props:                      # static colliders: a robot that drives into one stops
  - {id: crate_0, shape: box, size: [0.8, 0.6, 0.9], pose: [-9.0, 5.2, 0.0]}
  - {id: barrel_0, shape: cylinder, radius: 0.3, height: 0.9, pose: [-10.0, 1.2]}
  - {id: pallet, shape: usd, ref: Isaac/Props/Pallet/pallet.usd, pose: [-7, 0, 0],
     footprint: [1.2, 1.0]}  # usd props: the metrics need the floor footprint
actors:                     # scripted movers: visual only, fixed paths
  - id: walker_0
    shape: cylinder         # or box with size: [x, y, z]
    radius: 0.25
    height: 1.75
    path:
      waypoints: [[-11.0, 6.5], [-6.0, 6.5]]
      speed_mps: 1.0
      mode: pingpong        # once | loop | pingpong
      start_at_s: 0.0       # world-clock seconds
      dwell_s: 1.0          # pause at every waypoint
```

Full schema and validation: `simtest/world.py`. Coordinates are the **map
frame, which is Isaac's world frame** (every robot's `map -> <rid>/map` anchor
is its `configs/robots.yaml` spawn), so a point in a world file, on the
dashboard map and in `/sim/ground_truth` is the same point. The committed
worlds were laid out against a saved fleet grid of the NVIDIA warehouse;
`tests/simtest_world_test.py` checks that no actor walks through its own
world's props.

**Actors are not colliders**, deliberately: a kinematic body shoving the
robot's articulation around would make every run's trajectory depend on the
contact solver. They still render in RGB and depth, which is all perception
sees. Contact is scored from ground truth instead, by the `clearance` metric.

**Actors never react.** That is what makes a run repeatable, and it means a
robot that stops correctly can still be walked into. Lay paths out so a robot
that yields has somewhere to be; the clearance metric counts contact whoever
moved.

### The world clock and ground truth

`spawn_warehouse.py --world NAME` authors the props and moves the actors every
physics step. Always (with or without a world) it publishes:

| topic | type | what |
|---|---|---|
| `/sim/ground_truth` | `std_msgs/String` JSON, 10 Hz sim | `{t, wt, running, world, robots: {rid: {x, y, yaw}}, actors: {id: {x, y, yaw, vx, vy, moving}}}` — TRUE poses |
| `/sim/world` | `std_msgs/String` JSON, latched | the layout: props with footprints, actors with shapes and paths |
| `/sim/world_cmd` (sub) | `std_msgs/String` JSON | `{"cmd": "restart" \| "reset" \| "start" \| "pause"}` |

Actors run on a **world clock** (`wt`), not raw sim time: Isaac is publishing
for tens of sim-seconds before the ROS stack is up, and that lead varies with
the real-time factor, so anchoring actors to sim t=0 would give a different
robot/actor phase every run. A test restarts the world clock at its own t0;
the dashboard's map tracker has **restart / pause** buttons that do the same
by hand.

```bash
# from a ROS shell (scripts/ros_bash.sh -c "source /opt/ros/jazzy/setup.bash; ..." on 26.04)
ros2 topic echo /sim/ground_truth --once
ros2 topic pub --once /sim/world_cmd std_msgs/String '{data: "{\"cmd\": \"restart\"}"}'
```

### Watching a world live

1. Scenarios tab → Quick start **Moving obstacles — single robot** (or pick a
   **World** in the form), Start.
2. Live → map. Actors and props are drawn at their true positions over the
   costmap (toggle: **world**). The tracker shows the actor clock.
3. Drag a goal on the far side of a walker's path; watch the costmap pick the
   walker up (or not), and the plan bend around it.

## Test cases

```yaml
name: crossing_yield
scenario: moving_obstacles_mapped    # localizes on a saved map (see below)
overrides: {world: aisle_crossing}   # only keys the scenario lists in overrides:
warmup_s: 5                          # sim-s after the stack is ready, before t0
goals:                               # per robot, in order
  - robot: robot_0
    pose: [-8.0, 3.0, -1.5708]       # or waypoint: NAME (the saved map's own waypoints)
    at_s: 0                          # NO EARLIER than this; default: when the robot's previous goal ends
    timeout_s: 120                   # cancelled and scored `timeout` after this
tasks:                               # optional task_manager deliveries
  - {robot: robot_0, pickup: shelf_a, dropoff: zone_a, at_s: 10}
stop:
  sim_s: 150                         # hard cap after t0
  until: [goals_done]                # goals_done | tasks_done | explorer_done
  settle_s: 3                        # keep scoring this long after `until` holds
metrics:
  - metric: clearance
    pass: {contacts: {eq: 0}, min_m: {gt: 0.05}}
  - metric: clearance                # the same metric twice needs an id
    id: clearance_props
    params: {actors: [], props: true}
  - metric: run_stats                # no `pass:` = reported, not judged
repeats: 3
pass_rate: 1.0                       # fraction of repeats per variant that must pass
variants:                            # the same question under other conditions
  - {label: head_on, set: {overrides.world: aisle_head_on}}
record: [dyn]                        # extra event streams to keep for rescoring
```

Full schema: `simtest/spec.py`. Validation is complete before anything
launches — scenario and overrides through `session.py`'s allowlist, every
variant resolved, every metric's params and criteria checked, and each metric
asked whether it can be computed at all (a clearance metric on a world with no
actors fails here, not after a 5-minute run).

### Render cost

Once the render knobs (`render_every`, `lite_render` — on `local/integration`)
are merged, add them to the moving_obstacles scenarios' `overrides:` and pass
`--set overrides.render_every=2 --set overrides.lite_render=true` on a
GPU-bound laptop: the live runs on 2026-10-21 went at ~0.9–1.1x real time
that way, so `crossing_yield` takes ~2 minutes wall and `cluttered_route` ~4.

### Why avoidance tests run on a saved map

Nav2's global costmap is only as large as `/map`. On a fresh map, a goal
across floor the robot hasn't seen yet is `"Goal Coordinates ... outside
bounds"` and aborts before the robot ever meets a walker — that was
crossing_yield's first live run. So the avoidance tests use
`moving_obstacles_mapped`: localize read-only on a library map (copied
before use, never written), which also gives every run the same starting
picture. It defaults to `fleet_warehouse_test` (git-lfs, on
`origin/sim/infrastructure`); validation fails up front with
`map 'fleet_warehouse_test': no map named ...` or `... run git lfs pull`
where it isn't present, and the offline tests skip those cases rather than
fail. Any library map recorded from the same spawn poses works
(`--set overrides.map=NAME`).

### Shared networks: run on your own ROS domain

On a shared network, another machine's SortBots stack on domain 0 is the same
`/robot_0/...` graph as yours. On 2026-10-21 a teammate's full stack was
visible from this laptop's Wi-Fi: discovery traffic froze Isaac at sim
t=1 s (the main thread blocked in DDS sends), and a probe on domain 0 would
have scored — and sent Nav2 goals to — their robot. `scripts/simtest.py`
probes on the caller's `ROS_DOMAIN_ID`; give the whole pipeline a private
one:

```bash
export ROS_DOMAIN_ID=73          # anything 1-232 nobody else on the LAN uses
scripts/sim_ctl.sh console stop; scripts/sim_ctl.sh console start
scripts/simtest.py run crossing_yield
```

(`run_demo.sh`, `run_console.sh` and `session.py` must pass the domain through
instead of hard-coding 0 for this to reach Isaac and the bringup — see the
`dds-localhost-wip` branch for the related discovery-range work.)

### What a run does

`scripts/simtest.py run NAME`, per variant × repeat:

1. `sim_ctl.sh start SCENARIO k=v...` and `wait running` — an ordinary
   dashboard session, so **live** shows it.
2. Starts the probe (`simtest/probe.py`) in a ROS subshell
   (`scripts/ros_bash.sh` when present). It waits until `/clock`,
   `/sim/ground_truth`, the first `/map` and every needed `navigate_to_pose`
   server are up, waits `warmup_s` more, then **restarts the world clock**.
   t0 is read back from ground truth (`t - wt`), so actors and goals share t=0
   to within one ground-truth tick.
3. Sends goals/tasks on schedule, turns topics into events, feeds the metrics
   and records every event to `events.jsonl.gz`.
4. Stops at `sim_s`, or `settle_s` after `until` holds; scores; `sim_ctl.sh stop`.

Exit codes: 0 every variant passed, 1 a verdict failed, 2 invalid test, 3 the
sim/probe never produced a score, 130 interrupted (Ctrl-C or the tests tab's
Cancel stops the sim and still reports the finished runs).

### Results

`data/simtests/<UTC>_<name>/` (gitignored):

```
test.json         the test exactly as it ran, every variant resolved
results.jsonl     one line per run: variant, repeat, ok, score | error, meta
report.md         verdicts, every criterion per run, every value (mean [min–max])
runs/<v>_<r>/     events.jsonl.gz, score.json, probe.log, sim.log, bringup.log
```

Verdicts: a **run** passes when every criterion holds; a **variant** when
passed runs / runs ≥ `pass_rate` (a run that never got scored counts against
it — "couldn't tell" is not a pass); the **test** when every variant does. A
criterion with no data FAILS: a detection metric that never had the actor in
view didn't test what it says.

## Metrics

| metric | reads | scores |
|---|---|---|
| `clearance` | gt | min footprint-to-footprint gap robot↔actor (or prop), contacts, near-miss time |
| `goals` | goal, gt | Nav2 outcome, time to goal, TRUE final position error, distance driven |
| `map_contamination` | map | occupied `/map` cells on floor only actors walk on — movers leaking into SLAM |
| `detection` | dyn, gt | `dynamic_obstacles` recall vs ground truth, false-positive frames, latency to first detection |
| `coverage` | map, end | free / known area, mapping rate |
| `tasks` | dispatch, task | task_manager delivery outcomes and durations |
| `run_stats` | start, end, gt | sim/wall time, real-time factor, stop reason, ground-truth rate |

`scripts/simtest.py metrics` prints every param and value with its meaning.

### Changing how a metric is calculated

Metrics never touch ROS. The probe turns topics into plain events
(`simtest/events.py`), and the same events are recorded with every run. So a
metric is a pure function of recorded data, and its definition can change
after the fact:

1. Edit the metric in `simtest/metrics/<name>.py` and **bump its `version`**.
2. `scripts/simtest.py rescore data/simtests/<dir>` — replays the recorded
   events through the new code (keeps the old `results.jsonl` alongside).
   `--current-spec` also re-reads the test's YAML for the metric list and
   thresholds.
3. Every result records the version it was scored with; a report that mixes
   versions says so above the table.

A new metric: subclass `Metric` in a new module under `simtest/metrics/`, set
`name`, `version`, `streams`, `params`, `values` (every value documented),
implement `on_event` and `result`, and add the module to the import line at
the bottom of `simtest/metrics/__init__.py`. If it needs a stream the probe
doesn't produce yet, add it to `simtest/events.py`'s `STREAMS` and to the
probe — QoS for every topic is chosen there, centrally, never per metric (no
image topic, ever: CLAUDE.md). Metrics must be deterministic functions of
their events, or rescoring stops meaning anything.

Streams always recorded: `start end gt goal dispatch task explore map`
(`map` every 5 sim-s plus the last). `dyn` (dynamic obstacle points in the map
frame, ≤5 Hz, 0.1 m voxels) only when a metric needs it or `record: [dyn]`.

## Testing the test layer

All offline, seconds:

```bash
/usr/bin/python3 -m pytest tests/simtest_world_test.py tests/simtest_metrics_test.py tests/simtest_spec_test.py
node webui/tests/operator_test.mjs        # operator view: world overlay, tests tab, launch form
scripts/simtest.py validate               # every committed test case resolves
```

`pipeline_smoke` is the cheapest live end-to-end check of the harness itself
(one short goal, ground truth rate); run it after touching the probe, the
world runtime or the runner, before trusting a long test's verdict.

## Known limits

- `detection` treats an actor as visible by range and field of view only —
  no occlusion — and assumes the head looks straight ahead. Recall is a lower
  bound when paths pass behind racks.
- Every repeat restarts Isaac (robots back at spawn, fresh map). That is the
  price of identical starting conditions; a run is minutes.
- Actors are cylinders and boxes. An animated character
  (omni.anim.people) would matter for an RGB person detector, not for the
  depth-based dynamic obstacle filter this was built to test.
