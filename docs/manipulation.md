# Manipulation

## Pick and place (sim)

The sim can pick light boxes with the magnet effector, using a classical pipeline
instead of a learned policy: camera → plate pose → dock pose → IK. All of it is numpy,
with no torch anywhere in the stack.

- **The grasp:** a magnet turns grasping into a reach problem, so the sim uses a simple
  capture rule (`simtest/magnet.py`). If the magnet is energized and its face is within
  **3 cm** of the plate's centre and **30°** of its normal, the package snaps on.
  De-energized, it drops. This makes pick and place testable, but it is not a contact
  model to train on.
- **The arm:** it has five joints, and wrist roll doesn't move the magnet. The approach
  is therefore always radial from the shoulder, and "pick from any orientation" becomes
  a base-placement problem (`simtest/magnet.dock_pose`).
- **The envelope:** the shoulder is 0.83 m above the floor and the magnet reaches
  0.65–1.29 m. **Decks under about 0.66 m, and the floor, are out of reach by
  construction.** Horizontal reach is about 0.39 m from the shoulder. The dock puts
  the plate 0.27 m from the shoulder. `tests/arm_kin_test.py` recomputes these numbers
  from the URDF.

Smallest approach tilt that works, by plate height (base frame, which is world z −
0.05) and radial distance from the shoulder. Negative tilts point the magnet down;
`--` means nothing within 25°:

| radial \ plate z | 0.67 | 0.72 | 0.77 | 0.82+ |
|---|---|---|---|---|
| 0.19 | −25 | −20 | −20 | −10…0 |
| 0.22 | −20 | −10 | 0 | 0 |
| **0.27 (dock)** | **−20** | 0 | 0 | 0 |
| 0.33 | -- | −10 | 0 | 0 |

**Pipeline:**

1. `package_detector.py` (`package_vision.py`) finds the plate. The plate is near-black
   on cardboard, thresholded in HSV, and its depth gives a plane fit for the plate's
   centre and normal in the map frame. A plate with an ArUco marker is identified by
   its id.
2. `task_manager` sends a `pick_request` to `magnet_pick.py`, and `pick_planner.py`
   decides what happens next.
3. The robot tucks the arm.
4. It acquires the plate: face the work order's location, scan, and orbit the package
   if needed.
5. It docks: stage 0.35 m out, push straight in, then re-measure the plate (`confirm`).
   A fix taken while moving can be 8–15 cm off.
6. `plan_reach` computes the IK, the magnet energizes, then the robot lifts, tucks and
   backs out.
7. A place runs the same steps against a virtual plate at the target pose.

During a pick, `/<id>/pick_status` reports `mode: manipulation`. `box_geometry.py`
also fits boxes on the table from depth alone, which is what `find: geometry` tasks use.
Timing is sim time, ticked on the steady clock. Tuning lives in `configs/magnet_pick.yaml`
and `configs/package_detector.yaml`.

**Camera limits:** the head camera is 1.24 m up, with a vertical half-angle of only 21.6°.
A plate on a deck is fully in view only from about 1.1–1.5 m away, so the controller
looks from 2.6 m and docks on the last good fix.

**Splitting failures:** `perception=oracle` (sim only) aims at ground truth instead of
the camera. In `pick_place_smoke`:
- both the `camera` and `oracle` variants pass: the whole chain works.
- only `camera` fails: perception is the problem (then run `pick_detect_sweep`).
- both fail: the dock, the arm or the magnet is the problem.

| Test | Passes when |
|---|---|
| `pick_place_smoke` | a box on a bench is set down 12 cm over, within 15 cm |
| `pick_block` / `place_block` | the magnet holds the block / the block is placed within 5 cm of the mark (both on the Tests tab) |
| `pick_orientations` | four plates facing N/E/W/S are all placed |
| `pick_heights` | decks at 0.70, 0.78 and 0.90 m are all placed |
| `pick_unreachable` | a 0.30 m deck or the floor: the task must FAIL inside the timeout |
| `pick_detect_sweep` | the detector alone finds four plates within 5 cm and 15° |
| `pick_route` | Nav2 to a deck, pick, Nav2 across the floor, place (saved map) |

**Writing one:**

1. Lay out props and packages in `configs/worlds/`. `scripts/pick_layout.py package
   --deck X Y SX SY H --normal DEG` prints a package's pose and its dock.
   `scripts/pick_layout.py check NAME` checks a world's docks, reach and transits.
2. Write a test whose `tasks:` name a `package:`. `pickup` and `dropoff` default to
   `dock`; give a pose or waypoint to have Nav2 drive there first. Set
   `place: {shift: [dx, dy]}`, `hold`, or a pose.

**Not yet verified in a live sim:**

- that the FK magnet and the USD `Fixed_Jaw` agree (the sim log prints `magnet tcp: FK
  ... vs USD`).
- that nothing fights `cmd_vel` while docking.
- the chassis-to-deck clearance.
- the detector thresholds under Isaac's lighting.
- that the arm tracks its commands within 3 cm.

**Open items:**

- Package tasks can't be dispatched from the operator view yet.
- No test uses `top`-face plates.
- Only `robot_0` has an arm.
- The real robot needs a magnet link in the URDF, the microswitch as the "held" signal,
  and measured detector thresholds.

`tests/magnet_test.py` keeps the sim's TCP offset (47.1 mm) matched to
`hardware/magnet_effector/params.py`.

## Hardware: magnet end effector

`hardware/magnet_effector/` replaces the SO-101 gripper with a 5 V, 5 kg Adafruit
3873 electromagnet on a spring-compliant plunger, with a microswitch to confirm
seating. The coil needs a flyback diode and a MOSFET driver. The CAD is parametric
CadQuery in its own venv:

```bash
hardware/setup_cad.sh && sudo snap install freecad   # once
make cad          # rebuild from params.py (~4 s); prints TCP offset, mass, BOM
make cad-open     # fitcheck.step in FreeCAD (stock wrist overlaid in red)
make cad-arm      # whole-arm context + 360° wrist-roll collision sweep (exit 3 = collision)
make cad-test
```

Every dimension lives in `params.py`. Edit there, not in FreeCAD. Values marked
**VERIFY** are datasheet guesses (magnet thread, lead stub). Measure them before
printing. The upstream reference STEPs and their licence are listed in
`hardware/magnet_effector/reference/SOURCE.md`.

## ManiSkill arm track

A separate conda env for arm manipulation. Never source ROS in this shell.

```bash
# Miniconda installed and its ToS accepted, then:
bash scripts/install.sh          # env `lerobot` + mani-skill + ReplicaCAD + XLeRobot overlay
conda activate lerobot
bash scripts/verify_sim.sh       # imports, headless rollout, then SAPIEN windows for Fetch + XLeRobot
python scripts/run_xle_demo.py --help                     # list XLeRobot example demos
python scripts/run_xle_demo.py demo_ctrl_action_ee_keyboard \
    -e ReplicaCAD_SceneManipulation-v1 -r xlerobot --render-mode=human \
    --shader=default -c pd_joint_delta_pos_dual_arm
```

- **Other demos:** `demo_ctrl_action_ee_xbox`, `_cam_rerun`,
  `demo_ctrl_ee_keyboard_record_dataset`, `_VR`. For single-arm, use `-r
  xlerobot_single -c pd_joint_delta_pos`.
- **Headless:** `--render-mode=rgb_array` or `sensors`.
- **Bumping XLeRobot:** `git submodule update --remote third_party/XLeRobot && python
  scripts/overlay_xlerobot.py`.
