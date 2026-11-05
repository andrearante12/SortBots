# Picking small packages with the magnet effector, in the Isaac sim

Can the sim pick a light box with a magnet on one side, reliably, from any position or
orientation, and can that be tested the way the avoidance tests are? This is the answer,
what was built to get it, and what still has to be checked in a live sim.

**Verdict.** Feasible in Isaac, and the right way to do it is a classical pipeline (camera
-> plate pose -> dock pose -> inverse kinematics), not a learned policy. The magnet turns
grasping into a reach problem the arm's own geometry mostly solves. Everything that can be
checked without a GPU is checked (below); the Isaac-side wiring is written but has **not**
been run in a live sim yet, and the list of what to look at first is at the bottom.

```bash
scripts/simtest.py list | grep pick        # the pick tests
scripts/simtest.py run pick_place_smoke    # one package, camera AND oracle variants
scripts/pick_layout.py check pick_orientations   # a world's docks, reach and transits
/usr/bin/python3 -m pytest tests/magnet_test.py tests/arm_kin_test.py tests/pick_planner_test.py \
    tests/package_vision_test.py tests/magnet_runtime_test.py tests/simtest_pick_test.py
```

## What the magnet changes

The stock gripper needs finger placement, closure timing and friction. The magnet needs
none of it: the grasp is "put a 25 mm disc within a few centimetres of a steel plate,
roughly facing it". `simtest/magnet.py` states the rule the sim uses: energized and the
face within **3 cm** of the plate's centre, pointing within **30 degrees** of its normal,
and the package is snapped to the magnet; de-energized, it drops onto whatever is under it.
No contact, compliance, slip or mass. That is enough to write and score pick tests and it
is **not** a grasp model to train on (see "What this does and does not prove").

What is left is three problems, none of them learning problems:

1. **Where is the plate?** A 6-DoF pose of a flat thing.
2. **Where must the base stand?** The arm is five joints, and the last (wrist roll) turns
   about the approach axis itself, so it changes neither the magnet's position nor its
   direction. The other four place the magnet in a single vertical plane through the
   shoulder. The approach azimuth is therefore **always radial from the shoulder**: a plate
   facing away from the robot can only be picked by moving the *base* round to its side.
   "From any orientation" is a base-placement problem (`simtest/magnet.dock_pose`).
3. **Reach it.** Four-joint IK for the magnet's position plus the approach's elevation
   (`nodes/arm_kin.py`).

## The arm's envelope, measured

From the URDF's joint limits (the same limits `spawn_warehouse.py` clamps commands to, so in
the sim they are the truth, whatever the real SO-101 can do). Forward kinematics over the
joint space, `tests/arm_kin_test.py::test_envelope_constants_match_the_joint_space`
recomputes the constants:

- Shoulder 0.83 m above the floor, in `base_link` at (-0.090, -0.133, 0.776), on a mast.
- Magnet height 0.65 to 1.29 m above the floor. **The floor and any deck under ~0.66 m are
  out of reach by construction**: pitch cannot go below horizontal and the elbow folds the
  same way. `pick_low` and `pick_floor` are the negative cases.
- Horizontal reach from the shoulder at most ~0.39 m; ~0.33 m for a level approach at deck
  height.
- The chassis stops short of a full-height deck: `base_link`'s collision is a thin frame
  whose front wall sits 0.153 m ahead of the base origin, so a plate on a deck's edge is
  ~0.18 m ahead of the base at best. `REACH_NOM_M` = 0.27 m from the shoulder is that dock,
  and worlds overhang each package 2 cm past the deck edge so the frame stops ~4 cm short.

Which approach elevation works, by plate height and distance from the shoulder (smallest
tilt, degrees; negative points the magnet down onto the plate; `--` none within 25):

| radial \ plate z (base frame) | 0.67 | 0.72 | 0.77 | 0.82 | 0.87 | 0.91 | 0.96 |
|---|---|---|---|---|---|---|---|
| 0.19 | -25 | -20 | -20 | -10 | 0 | 0 | 0 |
| 0.22 | -20 | -10 | 0 | 0 | 0 | 0 | 0 |
| 0.25 | -20 | 0 | 0 | 0 | 0 | 0 | 0 |
| **0.27 (dock)** | **-20** | 0 | 0 | 0 | 0 | 0 | 0 |
| 0.30 | -- | 0 | 0 | 0 | 0 | 0 | 0 |
| 0.33 | -- | -10 | 0 | 0 | 0 | 0 | 0 |

(Base-frame z is world z minus 0.05.) The capture rule tolerates 30 degrees, so a tilted
approach is fine, and `plan_reach` takes the first of 0, -8, +8, -14, +14, -20, +20 that
solves both the contact pose and a pre-approach 8 cm back along the same line. Hence the
worlds' decks: 0.70, 0.78 and 0.90 m in `pick_heights`.

## The pipeline

```
head camera rgb + depth ──> nodes/package_detector.py ──> /<id>/packages (plate centre + normal, map frame)
                                                                │
task_manager ── pick_request ──> nodes/magnet_pick.py ──────────┤  (nodes/pick_planner.py decides)
   (package task)                  │                             │
                                   ├─ tuck the arm ──────────────┘
                                   ├─ acquire: face the work order's location, scan, orbit
                                   ├─ dock: stage 0.35 m out, then a straight push in
                                   ├─ plan_reach: pre-approach + contact pose (IK)
                                   ├─ energize, wait for the magnet to hold it
                                   └─ lift, tuck, back out ──> pick_result "picked"
                       place: the same against a VIRTUAL plate at the asked resting pose
```

**Perception is markerless**: the sim authors the box in cardboard and the plate near-black,
`nodes/package_vision.py` thresholds both in HSV, keeps a dark blob only if it touches
cardboard, back-projects its depth pixels and fits a plane. It reports the plate's centre
and outward normal, nothing about identity: the controller tells packages apart by where
the work order says they are (`at`, roughly the shelf slot). It cannot see a plate that
faces away from the camera, so the controller turns on the spot and, failing that, walks a
circle round the package. On synthetic frames it finds the plate centre within 1.5 cm and
the normal within 5 degrees from 1 to 2.5 m at yaw up to 45 degrees.

**What the camera can actually see.** The head camera is at (-0.092, 0.02, 1.191) in
`base_link`, 1.24 m up and 9 cm behind the base origin, looking level at head tilt 0. A world
can start the head tilted (`head:`, tilt > 0 looks down) and the operator view's Head sliders
move it live; the sim's camera TF follows either way (simtest/head.py). Figures below are
for a level head. The D435
intrinsics (fx = fy = 605, 848 x 480) give half-angles of 35 degrees across and **21.6
degrees vertically**, not the 43 and 29 of a wider lens. A plate centred 0.4 to 0.5 m below
the lens is fully in view only from about 1.1 to 1.5 m away, and one on a 0.30 m deck from
2.6 m, the floor from beyond the 4 m range. So the controller looks from `view_m` = 2.6 m,
the robot is never closer than 0.27 m to the plate when docked (where the plate is far below
the view), and the dock runs on the last good fix rather than on live detections.

**`--perception oracle`** (scenario run key `perception`, sim only) aims at ground truth
instead of the camera. `pick_place_smoke` runs both: both pass = the chain works; only
`camera` fails = perception (then `pick_detect_sweep`); both fail = the dock, the arm or the
magnet. Both modes take "is the magnet holding something" from ground truth; on the real arm
that is the microswitch behind the plunger.

**The arm is tucked first.** The sim boots with the arm straight out at deck height, which
would put its tip inside the box at the dock. `tuck_q` is the pose found by sampling the
joint space for the one whose every link stays behind the shoulder; driving and carrying
happen tucked. Every request ends by backing out to the staging point, because the next
straight-line move from the dock can clip the deck it is standing against.

**Timing is sim time**, ticked on the steady clock (CLAUDE.md on timers under `use_sim_time`).
`task_manager`'s own timeout (300 s wall) is only a safety net: the controller answers
`failed` itself in seconds when a plate is out of reach.

## Why not a VLA (SmolVLA and friends)

- The magnet makes the task a four-degree-of-freedom reach to a known pose. Demonstrations
  would encode what the IK computes, at a cost of dozens to hundreds of teleoperated episodes
  per variant.
- The sim's only grasp signal is a teleport. A policy trained on it learns nothing about
  contact that the rule in `simtest/magnet.py` does not already say.
- Sim-to-real on pixels (Isaac's renderer to a D435) is the hardest gap in the project, and an
  end-to-end policy puts it in the critical path. The classical pipeline puts it in one small
  place, the detector's thresholds.
- A scored test needs to say *why* it failed. This pipeline has named stages and an oracle
  switch; a policy gives pass or fail.
- Inference on the Jetson Orin Nano is a budget this does not need to spend.

Where a learned policy would earn its place: clutter and heavy occlusion (packages in a bin),
where the plate is not cleanly visible. Even then, an ACT-class policy for the last 10 cm
with this pipeline doing the gross motion beats an end-to-end model, and the ManiSkill track
(`sim/envs/sortbots_sort.py`) is where to train it, against the same capture rule. Not
started; revisit only if this pipeline's reliability stalls on real hardware.

## What the sim fakes

| | in the sim | on the robot |
|---|---|---|
| grasp | snap within 3 cm / 30 degrees | 5 kg-holding electromagnet on a 3 mm spring plunger |
| "is it held" | ground truth | microswitch (`hardware/magnet_effector`) |
| plate colour | authored exactly, Isaac's lighting | real cardboard and steel, auto-exposed camera |
| the arm's position | the simulated joints, no `/joint_states` topic yet | servo feedback |
| the package | no collider, no mass, never tilts | a box |
| TCP | `Fixed_Jaw` + 47.1 mm along the roll axis | `Params.z_magnet_face` in `hardware/magnet_effector/params.py` |

`tests/magnet_test.py` recomputes the 47.1 mm from `params.py`, so the CAD and the sim
cannot drift apart silently. The URDF still carries the stock gripper (the magnet's link is
one of the README's "still to do" items); the sim finds the magnet by forward kinematics
from the joint positions, not from a link in the USD.

## Frames and conventions

Everything is the map frame (Isaac's world). A package's pose is `{x, y, z, yaw}`: x, y its
centre, **z the height of its bottom** (a package on a 0.80 m deck has z = 0.80). Its plate
is on one of `+x -x +y -y top` in the package's own frame; with face `+x`, yaw 0 faces +x,
1.5708 faces +y, 3.1416 faces -x, -1.5708 faces -y. The dock puts the plate on the base's
radial line with the base facing it, 0.18 m ahead and 0.133 m to the arm's side.

## Writing a new pick test

1. Put props and packages in `configs/worlds/NAME.yaml` (`packages:` in `docs/testing.md`).
   `scripts/pick_layout.py package --deck X Y SX SY H --normal DEG` prints a package's pose
   on a deck and where the robot docks; `scripts/pick_layout.py check NAME [--map M]` checks a
   world's docks, reach and transits (the same assertions as `tests/simtest_world_test.py`,
   with the numbers).
2. A test case in `configs/tests/` with `tasks:` that name packages and say where each
   comes to rest. `pickup`/`dropoff` default to `dock` (the controller drives itself); give a
   pose or a waypoint name to have Nav2 bring it close first (`pick_route`).
3. Lay packages out in the order the test asks for them, with the plate facing where the
   robot will be coming from: the controller does search all round the package, but a test
   that depends on that is testing the search.

## Not verified live (check these first)

Everything below is code and tests that pass offline, against stand-ins. The first live run
of `pick_place_smoke --variant oracle` (isolates the dock, the arm and the magnet from the
camera) settles most of it:

1. **`/World/robot_0/Fixed_Jaw` exists.** The sim log says `arm tcp link OK` or lists the
   robot's children. The magnet does not need it (forward kinematics), but the next item does.
2. **The FK magnet and the USD link agree.** `magnet tcp: FK (...) vs USD (...)` should say
   under 2 cm apart, at the first step and every ~5 sim-seconds after. If it says `DISAGREE`:
   the TCP offset's axis or sign, the base-pose convention, or USD not following the physics.
3. **Nothing fights `cmd_vel`.** The dock drives `/robot_0/cmd_vel` directly. Nav2's collision
   monitor is believed to publish a stop only once after its input ends; if the base judders
   while docking, that is the first suspect.
4. **The chassis really stops 4 cm from the deck**, and the tucked arm clears the head camera's
   view. The deck's collider and the robot's frame were reasoned from the URDF, not run.
5. **The detector's colour thresholds** against Isaac's lighting (`configs/package_detector.yaml`,
   `sim:`). The plate is a very dark diffuse colour; if no plate is ever found, look at the
   segmentation first. The `real:` block is unmeasured.
6. **The arm tracks its commands** closely enough for the 3 cm capture, and `settle_s` (0.4 s)
   is long enough. There is no `/joint_states` to poll.
7. **`pick_route`** (Nav2 to within 0.7 m of each plate, on the saved map) and
   **`pick_detect_sweep`** (Nav2 goals on a fresh map, in a world with decks the map does not
   know about).
8. **Real-time factor**: no camera render product was added, but the detector adds a 10 Hz
   OpenCV pass; measure with `scripts/bench_sim.sh` if the sim slows.

## What this does and does not prove

It proves the chain closes: world layout, work order, search, dock, reach, magnet, carry,
place and scoring, repeatably, with a rule tolerant enough (3 cm, 30 degrees) that a
four-joint IK is enough. It makes pick and place **testable**, which is the point.

It does not prove a grasp works on hardware. The snap has no contact or slip, "held" is an
oracle, the plate is easier to see than a real one, and the arm's envelope is the XLeRobot
URDF's, not the real SO-101's calibrated one (`origin/physical_hardware/robotic_arm` has
`configs/so101_new_calib.urdf` and a numpy kinematics script; `nodes/arm_kin.py` parses any
URDF with the same joint names, so pointing it there is the first step to the real arm).

## Open items

- A package task cannot be dispatched from the operator view yet; dispatch one from a test or
  with `ros2 topic pub /robot_0/dispatch_task std_msgs/String ...` (the JSON in
  `tests/task_manager_test.py::package`).
- A `top`-face package is in the world schema and the magnet logic; the planner will try it
  but no world or test uses it, and a vertical approach is the least-tested path.
- Only `robot_0` has an arm in the sim.
- The real robot: the URDF magnet link, the microswitch as the "held" signal, and measured
  detector thresholds.
