# Testing

## Offline (seconds; no GPU, ROS or display). This is what CI runs.

```bash
make ci                                   # pytest + scenario smoke, system python3
/usr/bin/python3 -m pytest tests/         # node logic (NOT conda's python: no pytest there)
node webui/tests/operator_test.mjs        # operator view
python3 webui/session.py --list           # scenario validation
scripts/simtest.py validate               # scored test cases resolve
/usr/bin/python3 -m pytest commsim/tests
```

## Scored live tests (`simtest/`)

There are three layers, kept separate on purpose:

| Layer | File | Answers |
|---|---|---|
| world | `configs/worlds/NAME.yaml` | what's on the floor: static props, scripted walkers (fixed paths, never react, not colliders), and packages the magnet can pick |
| scenario | `configs/scenarios/NAME.yaml` | how the stack runs |
| test case | `configs/tests/NAME.yaml` | goals/tasks, stop condition, metrics + pass criteria, repeats, variants |

```bash
scripts/simtest.py list | show NAME | metrics
scripts/simtest.py run crossing_yield [--repeats 3] [--set overrides.world=aisle_head_on]
scripts/simtest.py rescore data/simtests/<dir>     # re-score recorded events, no sim
scripts/simtest.py report DIR_A DIR_B
```

Test cases: `pipeline_smoke` (the cheapest harness check), `crossing_yield`,
`head_on_pass`, `cluttered_route`, `fleet_traffic`, `fleet_scale_smoke`, and the pick and
place tests in [Pick and place](manipulation.md#pick-and-place-sim). A test's `section:` is its heading on the
operator view's Tests tab. `parked: true` keeps a test off the tab, but `simtest.py` can
still run it. The operator test serves on port 8201. If another session's console is
already using that port, pass `--port N --cdp-port M`.

**Other world keys:**

- `packages:`: up to 16 boxes, each with `size`, `pose` [x, y, yaw], `z` (the height of
  its bottom: 0 for the floor, or exactly a prop's top), `magnet_face` (`+x -x +y -y top`)
  and `color`. A package that floats or sits inside a prop is rejected at load. Packages
  are visual only, and the magnet snaps them.
- `marker: N` (0–9): an ArUco id (DICT_4X4_50) on a package's plate. A task on that
  package then needs no location.
- `style: table`: makes a box prop look like a table. It still collides as a solid box.
- `marks:`: painted target rectangles.
- `head: {pan, tilt}`: the head's starting angle.
- `view: {eye, target}`: aims the Sim view's camera.

In a world with packages, ground truth also carries `packages` (pose, `held_by`),
`magnet` (energized, held, tcp, nearest plate) and `head`.

How a run works:

1. A probe waits for the stack to be ready, then restarts the **world clock**, so
   walkers and goals share t=0.
2. Goals are sent on schedule.
3. Topics are recorded as events to `events.jsonl.gz`.
4. The run is scored when it stops.

Ground truth is on `/sim/ground_truth` (10 Hz JSON). The world layout is
`/sim/world`, and `/sim/world_cmd` restarts or pauses it. World coordinates are the
map frame.

- **Metrics** (`simtest/metrics/`): `clearance`, `goals`, `map_contamination`,
  `detection`, `coverage`, `tasks`, `pick_place`, `run_stats`. They are pure functions of recorded
  events. To change one, edit it, bump its `version`, then `rescore`.
- **Verdicts:** a criterion with no data FAILS.
- **Exit codes:** 0 pass, 1 fail, 2 invalid, 3 no score.
- **Results:** `data/simtests/<UTC>_<name>/`.

Avoidance tests run on a saved map (`moving_obstacles_mapped`): Nav2 rejects goals
outside the known map. On a shared network, set a private `ROS_DOMAIN_ID` and
restart the console. A teammate's stack on domain 0 is the same `/robot_0` graph.
