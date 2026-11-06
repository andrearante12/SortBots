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
scripts/simtest.py run crossing_yield [--variant forklift] [--repeats 3]
scripts/simtest.py assets                          # every usd asset the worlds name is on the asset root
scripts/simtest.py rescore data/simtests/<dir>     # re-score recorded events, no sim
scripts/simtest.py report DIR_A DIR_B
```

Test cases: the obstacle avoidance suite below, and the pick and place
tests in [Pick and place](manipulation.md#pick-and-place-sim). `pick_block` is the cheapest
harness check: no saved map, no Nav2, about two minutes. A test's `section:` is its heading on the
operator view's Tests tab. `parked: true` keeps a test off the tab, but `simtest.py` can
still run it. The operator test serves on port 8201. If another session's console is
already using that port, pass `--port N --cdp-port M`.

**What a row on the Tests tab shows:**

- `description:` is the overview: one plain sentence. Keep engineering detail in the
  file's comments.
- "Passes when" comes from the thresholds, through `simtest/plain.py`. Change a threshold
  and a sentence that no longer holds falls back to the raw check.
- The GIF is `webui/test_media/NAME.gif`. `scripts/simtest.py run NAME --clip` records the
  Sim view and writes it only if the run passes. Re-record it when the test changes, and
  commit it.

## Obstacle avoidance suite

`crossing_yield` (a person, or a forklift, across the route), `head_on_pass` (a person,
or a warehouse AMR, down the aisle), `step_out_stop` (a person steps out from behind
crates and stops: the braking case), `mixed_route` (real props, two people and a
forklift, two legs) and `peer_crossing` (two real Nav2 robots on crossing goals, plus
the person). All run on the saved map (`moving_obstacles_mapped`).

**Actors are real assets.** A world names an obstacle by `preset:` — `person`, `worker`,
`forklift`, `amr` (an iw_hub), `amr_tall` (a Nova Carter) — and `simtest/world.py`
PRESETS supplies the NVIDIA asset reference plus the floor footprint and height the
metrics score against (measured with `scripts/measure_assets.py`). Any key can be
overridden; `shape: usd` with `ref` + `footprint` or `radius` names an asset directly.
Assets are spawned with their physics stripped, so actors stay non-colliders; without the
asset server an item degrades to a primitive of the same footprint. Characters stand in
their bind pose (a T, arms out at 1.4 m, over the robot) because `omni.anim.people`
isn't in the pip install: depth sees a person-tall body ~1.8 m wide, which the tracker's
large-mover rule still calls moving. `yaw_offset_deg` turns a model to face +x,
`origin_offset` puts its body centre on the pose, `z` its feet on the floor.

**Objective scores, not only pass/fail.** `reaction` (ground truth only) opens an
*encounter* when an actor is within 4 m, ahead, and on a constant-velocity collision
course, and scores time to halve speed, time to stop, braking distance, hardest
deceleration, minimum time-to-collision, yield time and closest gap. `goals` v2 adds the
detour ratio and mean speed; `clearance` v2 takes `peers: true` for robot-to-robot gaps.
`prediction` scores the Kalman tracker's `trk` stream: association and moving recall,
position/velocity RMSE, look-ahead error at each horizon (ADE/FDE) against the analytic
actor path — exact, since paths are analytic — with `cv_fde_m`, what constant-velocity
extrapolation of the TRUE state scores, as the floor. Judge the values that are the
point of a test and leave the rest reported: a clean head-on pass need not slow at all.

**Prediction into Nav2.** `nodes/obstacle_detector.py` publishes
`/<id>/predicted_obstacles`, each moving track's footprint swept 1.5 s along its
velocity, which the LOCAL costmap reads as `predicted_scan`. Flip `predict_cloud` in
`configs/obstacle_detector.yaml` and `scripts/simtest.py report A B` to compare runs with
and without it.

**First live results (2026-11-06, `crossing_yield` person, RTX 4070 laptop).** Prediction
off: 5 contacts, goal in 57 s, 7 encounters. On: 4 contacts, goal in 34.5 s, detour 1.04,
2 encounters. Both fail on contact. The person was tracked at 5.1 m and called moving at
4.2 m, 5 s before the robot braked onto their line (1.26 m of braking from 0.43 m/s), so
the next work is Nav2's response, not perception. The other tests have not run live yet.

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

- **Metrics** (`simtest/metrics/`): `clearance`, `reaction`, `goals`, `map_contamination`,
  `detection`, `prediction`, `coverage`, `tasks`, `pick_place`, `run_stats`. They are pure functions of recorded
  events. To change one, edit it, bump its `version`, then `rescore`.
- **Verdicts:** a criterion with no data FAILS.
- **Exit codes:** 0 pass, 1 fail, 2 invalid, 3 no score.
- **Results:** `data/simtests/<UTC>_<name>/`.

Avoidance tests run on a saved map (`moving_obstacles_mapped`): Nav2 rejects goals
outside the known map. On a shared network, set a private `ROS_DOMAIN_ID` and
restart the console. A teammate's stack on domain 0 is the same `/robot_0` graph.
