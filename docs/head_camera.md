# Calibration: arm, head and head camera

Picking a package means turning "the D435 on the head sees marker 10 here" into "drive the
magnet there". That needs four things right, calibrated in this order:

| Stage | What it sets | Tool | Result file (under `~/.cache/huggingface/lerobot/calibration/robots/`) |
|---|---|---|---|
| 1. Arm servos | each joint's encoder range and its 0 deg | `lerobot-calibrate` | `so_follower/sortbots_arm.json` |
| 2. Head servos | ids 7/8, pan/tilt range and 0 deg | `arm.sh head setup/calibrate` | `sortbots_head/sortbots_head.json` |
| 3. Camera + arm offsets | where the camera is for any pan/tilt; the arm's joint-zero offsets | `arm.sh head handeye capture` then `solve` | `sortbots_head/sortbots_head_camera.json`, `so_follower/sortbots_arm_kin_offsets.json` |
| 4. Verify | nothing; reads the result | `arm.sh head handeye check` | |

Everything runs on the laptop in the `lerobot` conda env, from `~/SortBots` on the
`physical_hardware/robotic_arm` branch, in a shell **without ROS sourced**. One process owns
the servo port at a time: quit `arm.sh pad` before any of this, and vice versa.

**Redo stage 3 after redoing stage 1 or 2**: recalibrating a servo moves its zero, and the
camera fit and the joint offsets are relative to those zeros.

## The frames

- **`base_link`** (the arm's URDF frame) is what every answer is in: x forward along the arm's
  reach at pan 0, y left, z up, origin on the arm's mounting surface, 38.8 mm behind the
  shoulder_pan axis. "Height" in this doc means z in `base_link`, not above the table: our arm's
  base sits slightly above the table.
- **The camera's optical frame** (x right, y down, z out of the lens). The head's two servos move
  it; `scripts/head_cam.py` models `base_link <- camera` as a function of pan and tilt.
- **The magnet face** is 47.1 mm out from the wrist-roll flange along the roll axis
  (`head_cam.MAGNET_FACE_M`, from the CAD in `hardware/magnet_effector/params.py`).
- **The package target**: the magnet circle's centre is 35.68 mm left of and 47.93 mm below the
  midpoint of marker 10's top black edge, in the marker's own axes as printed
  (`head_cam.TARGET_FROM_TOP_EDGE_M`). Markers are `DICT_4X4_50`, 20 mm black square.

## Stage 1: arm servos

```bash
~/miniconda3/envs/lerobot/bin/lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=sortbots_arm
```

- At the first prompt **type `c`, ENTER**. A bare ENTER writes the old file onto the servos
  instead of calibrating.
- Hold the arm in the middle of its range, ENTER, then sweep shoulder_pan, shoulder_lift,
  elbow_flex and wrist_flex through their full ranges, ENTER. wrist_roll is a full-turn joint
  and isn't swept.
- The head can stay chained on the bus: this only touches ids 1-5.

lerobot puts each joint's 0 deg at **the middle of the range you swept**, and the arm model
(`scripts/so101_kin.py`, the URDF) has its own zero pose: upper arm nearly vertical, forearm
horizontal forward, wrist in line. Those disagree on joints whose sweep the table or base cuts
short. On our arm the magnet held flat on the table read z = -11 cm. Stage 3 measures that
disagreement as joint offsets; nothing in stage 1 fixes it.

## Stage 2: head servos

Covered in `docs/arm.md` ("Pan/tilt head"). In short: each new servo **alone** on the board,
`arm.sh head setup pan` (id 7) and `setup tilt` (id 8); chain everything back; `arm.sh head
scan` must list ids 1-5, 7, 8 at ~12 V; `arm.sh head calibrate` with the head looking straight
ahead and level, then sweep both.

## Stage 3: camera and arm offsets (touch calibration)

### How it works

Each **sample** is one known point seen two ways:

1. **LOOK**: the camera reads marker 10 on the package, takes the package's top-face plane from
   depth, and works out the circle centre in camera coordinates (median of 10 frames, refused if
   it jitters more than 4 mm). It also records the head's **measured** pan and tilt.
2. **TOUCH**: you put the magnet face flat on that circle centre; the arm's five joint angles
   say where the arm model thinks the magnet face is.

`solve` then finds the camera model and the arm's joint offsets that make the two agree over
all samples. A marker taped on the magnet would avoid the touch, but it gets in the way of the
magnet, so the package's own marker is used instead (`--touch`). The old magnet-marker mode
still exists without `--touch`.

### Before you start

- 12 V on, servo board on USB (`/dev/ttyACM0`), D435 on a **USB 3** port, the Flydigi pad
  plugged in. Short of ports: unplug the magnet's ESP32, calibration doesn't use it.
- The pad needs `sudo chmod 666 /dev/bus/usb/BUS/DEV` after every plug; capture prints the
  exact path if it is missing.
- One package with marker 10 on its top face, and a book or box to raise it on.

### Capture

```bash
scripts/arm.sh head handeye capture --touch --pad            # resumes the saved samples
scripts/arm.sh head handeye capture --touch --pad --fresh    # starts over
```

Open the guide **in your own browser**, not the Claude app's built-in pane (a live page there
was the likely trigger of an app crash, 2026-11-11):

```bash
xdg-open /tmp/handeye_preview/handeye_view.html
```

(If that browser won't live-reload `file://`, serve the folder:
`python3 -m http.server 8765 --bind 127.0.0.1 --directory /tmp/handeye_preview` and open
`http://127.0.0.1:8765/handeye_view.html`.) The page shows the camera with detected markers, the
step you're on, what to do, live ticks, "Ready: press B", and progress per head angle.

**Controls.** Pad: hold **RT** + sticks to drive the hand (half speed by default; d-pad changes
speed, hold **L3** for slow), **Y/A** wrist up/down, hold **RB** + right stick for the head,
**B** = LOOK or TOUCH, **BACK** = cancel a LOOK or drop the last sample, **START** = save and
quit (support the arm first: quitting turns torque off). Keyboard, in the capture terminal:
**`f`** frees the arm so you can **move it by hand** and holds it again, space = B, `u` = BACK,
`x` = START.

**One sample:**

1. Put the package somewhere the arm can reach, on the table (or raised, when asked). Move the
   arm clear of it so the camera sees marker 10. Press **B** (LOOK).
2. **Don't move the package or the head.** Get the magnet **face flat on the circle centre**,
   touching lightly. Easiest by hand: `f`, guide it, `f` to hold. Press **B** (TOUCH).
3. Lift off, move the package somewhere new.

**The plan** (the page tracks it): 6 samples at a HOME head angle (wherever the first sample is
taken: aim the head so the arm's reach fills the image), then 4 each at pan +20, pan -20,
tilt +10 and tilt -10 from home. **22 in total.** The page sends the head to each angle (RB +
right stick until both ticks are green).

**What makes a good set:**

- **Spread**: near the arm, at full reach, left and right of it, all over the image.
- **Heights**: from the fourth sample, every third package goes **up on a book or box**. All on
  one surface leaves height loosely determined (our first full set: all 22 at one height).
- **Flat touches**: tilt shifts the face centre off the circle. Our first set had the tool
  10-22 deg off vertical and fitted no better than ~11 mm.
- **Package still between LOOK and TOUCH**: a moved package is a bad sample (two of ours were
  39 mm and 115 mm off).

**The green box / reach map.** For the first three home samples there is no box: make them a
triangle on the table. With arm offsets already measured (a previous good `solve`), capture
then fits a rough camera pose and shades the spots the magnet can touch straight down; each box
is the reachable spot farthest from your earlier samples. Without offsets the map is off (an arm
model 10 cm wrong boxed unreachable spots) and the page guides by text. A box out of reach:
put the package at the nearest reachable spot instead; stages finish on sample count, not boxes.

**What capture handles for you:**

- A D435 that starts but sends no frames, or whose colour stream stalls mid-run (the preview
  froze and a LOOK read a stale frame): it resets the camera (~6 s) and carries on.
- Samples are saved after every TOUCH, to `sortbots_head/handeye_samples.json`; quit and resume
  any time. `--fresh` starts over (copy the file first if you want the old set).

### Solve

```bash
scripts/arm.sh head handeye solve
scripts/arm.sh head handeye solve --drop 3 1           # leave samples out by number
scripts/arm.sh head handeye solve --touch-z 0.046      # touched surface's height in base_link, m
```

It fits **18 numbers**: the camera model (pan frame 6, tilt axis offset and roll 2, camera on
the tilt frame 6, magnet length 1, held at 47.1 mm for touch-only sets) and, with 15+ samples,
**joint offsets for shoulder_lift, elbow_flex and wrist_flex**. It prints:

- every sample's error, flagging outliers (more than 3x the median, or over 8 mm),
- the RMS and worst error,
- where the lens is, next to the rig's measurements (lens 457 mm up, mast 127 mm from the arm),
  as a sanity check,
- the joint offsets.

It **always** saves the camera model (`sortbots_head_camera.json`). It saves the **joint
offsets only if the RMS is 8 mm or less** (or with `--save-offsets`), backing up the previous
file, because `SO101Kin` applies them everywhere: teleop, IK, `check`, picking. A 28.6 mm fit
once rewrote them; that's why the gate exists.

**Reading the result:** RMS of a few mm with no flagged samples is good. One or two flagged
samples: drop them and re-solve. RMS over 1 cm everywhere: the touches are noisy (tilted face,
package moved) or the set lacks spread or heights; re-capture rather than force it.

`--touch-z`: the touched surface's height in `base_link`. Measure the package top above the
table (camera depth gave 50.6 mm for ours) and subtract how far the arm's mounting surface sits
above the table. It helps only when all touches share one surface; raised samples help more.

### Why these choices

- **Joint offsets come from the camera, not a table touch.** Lift +d, elbow -2d, wrist +d keeps
  the magnet face flat and at the same height, so touching a flat table can't separate those.
  The camera sees the change in reach. A pan offset is a rigid turn of the whole arm, which the
  camera pose absorbs; roll doesn't move the face.
- **The magnet length can't be fitted from touches.** With the tool always pointing down, its
  length trades exactly with the camera's height. Picks approach the same way with the same
  length, so the error cancels there.
- **The face's tilt comes from depth, not the marker.** A 20 mm marker's own orientation is
  degrees off; the target is 52 mm from it, so that would be millimetres. Its in-plane rotation
  is reliable and is used. The plane is fitted between the marker and the circle, because the
  marker sits near a corner of the face.
- **Synthetic checks** (`tests/head_cam_test.py`): 22 touches with 1.5 mm noise fit at about
  2 mm RMS; offsets of 20 / -15 / 10 deg come back within half a degree.

## Stage 4: check

```bash
scripts/arm.sh head handeye check
```

Torque off. For every visible package it prints the magnet target in `base_link` cm, `up` (the
face normal's z, about +1.00 for a top face) and `d`, the magnet face's distance from the
target. Put the magnet on the circle by hand: `d` should read under about 1 cm.

## Pick: rest pose and taught magnet pitch

`handeye pick` starts by moving the arm to the **rest pose**. It goes back there after
each release, and the **Go to rest [h]** button sends it there too. To change the rest
pose, put the arm where you want it and press `H` (Shift+h) on the page. The file is
`pick_rest_pose.json`.

The model's idea of "straight down" is not the real one: a magnet held flat on a
package reads a few degrees off vertical in the model (-3.6 deg on 2026-11-13). In
that case a planned pick seats the magnet on one edge. To correct it:

1. Set the magnet flat on a package by hand (**Free arm**, then seat it).
2. Press `r` on the page. The pose goes to `pick_flat_poses.json`.
3. Repeat at a few reaches.

`pick` aims for the mean of the recorded pitches. Use `--pitch-deg` to override it.

### Touch samples from the pick page

The pick page can collect touch samples itself, with the head at whatever angle it is held.
Calibrating a new view does not need the capture tool:

1. With the arm clear of the marker, press **LOOK [l]**.
2. Put the magnet flat on the circle centre. Either use **Free arm** and place it by hand, or
   drive it there.
3. Press **TOUCH [t]**.

Each sample is added to `handeye_samples.json` as soon as it is taken. **Cancel look [c]**
discards a look, and **Drop last sample [z]** removes the last sample taken in this session.
Then run `scripts/arm.sh head handeye solve` and restart `pick` to load the new model.

## Diagnostics

| Command | Use |
|---|---|
| `scripts/arm.sh fk` | torque off, move the arm by hand, live magnet face and tip position from the arm model. Rest the magnet flat on the table: `z` should read the table's height. |
| `scripts/arm.sh head scan` | which servo ids answer, with voltage and temperature |
| `scripts/arm.sh head read` | torque off, live head angles |

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| capture or `arm.sh` traceback, all servo ids missing | 12 V off, or still coming up | power on, wait a few seconds, retry |
| `Frame didn't arrive within 5000` | D435 left mid-stream by a crash | capture now resets it once; else replug |
| preview stops changing | D435 colour stream stalled | capture resets it automatically (~6 s) |
| "no GUI in this OpenCV" | lerobot ships headless OpenCV | use the browser page; don't install opencv-python beside it |
| pad: permission denied | USB node is root-only after a plug | `sudo chmod 666 /dev/bus/usb/BUS/DEV` |
| TOUCH refused: "head moved X deg" | the head shifted over 1.5 deg since the LOOK | BACK, LOOK again |
| green box out of reach | no arm offsets yet, or a bad sample in the first three | place it at the nearest reachable spot |
| `solve` RMS over 1 cm | tilted touches, moved package, all at one height | drop flagged samples; re-capture flat, spread, some raised |
| hand-space teleop feels off after a `solve` | offsets saved from a doubtful fit | move `sortbots_arm_kin_offsets.json` aside (backups sit next to it) |

## Status (2026-11-11)

- Stages 1 and 2 done; the head is ids 7/8 on the arm's bus.
- Stage 3: a first full touch set (22 samples) fitted to 10.9 mm RMS after dropping two bad
  touches, with implausible offsets (wrist ~26-43 deg depending on `--touch-z`); everything was
  at one height and the touches were 10-22 deg off flat. **No joint offsets are in use** (that
  file is set aside as `sortbots_arm_kin_offsets.rejected-*.json`). The camera model from that
  fit is saved but not trustworthy.
- Next: re-capture with flat touches (hand-guided), spread, and every third package raised,
  then `solve`, then `check` on a package.
