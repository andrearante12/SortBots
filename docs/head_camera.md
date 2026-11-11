# Head camera ↔ arm calibration (D435 on the pan/tilt head)

Turns a point the head's D435 sees into the SO-101's `base_link`, at any head pan/tilt, so a
package's ArUco marker becomes a pick target. Model, fit and readout: `scripts/head_cam.py`
(numpy only), `scripts/handeye.py` (D435 + bus + keys). Runs in the `lerobot` env, no ROS;
the arm and the head must both be calibrated first (`docs/arm.md`).

`base_link` is the URDF's: x forward along the reach at pan 0, y left, z up, origin on the
mounting surface 38.8 mm behind the shoulder_pan axis.

## Measure first

- **Marker size**: the black square's outer edge, calipers. Ours are 20 mm (`--marker-m 0.020`,
  the default). A 0.5 mm error is a 2.5 % range error on the PnP fallback.
- **Dictionary**: `DICT_4X4_50`.
- **The rig** (2026-11-11): the head sits on a mast beside the arm, both on one base. Lens
  457.2 mm above the mounting surface, mast centre 127 mm horizontally from the arm base's
  centre, arm 3.6 mm in front of it. `solve` prints the fitted lens position against these as
  a sanity check; it does not need them.
- **The package** (top face, same layout on every package, marker printed upright): the magnet
  circle's centre is 35.68 mm left of and 47.93 mm below the midpoint of the marker's top black
  edge. `head_cam.TARGET_FROM_TOP_EDGE_M`.
- **Optional seeds** for `solve` (ruler, ±1 cm is plenty; the fit overrides them): pan axis
  from the shoulder_pan axis at the mounting surface (`--pan-axis X Y Z`, m), tilt axis
  ahead of the pan axis (`--tilt-fwd`), flange to marker surface (`--magnet-m`, 0.047 + tape).

At 1280×720 a 20 mm marker is ~25 px at 0.75 m: the reliable decode limit. Calibrate and pick
inside that.

## Calibrate

```bash
scripts/arm.sh head handeye capture --marker-id N   # N = the marker taped on the magnet
scripts/arm.sh head handeye solve
scripts/arm.sh head handeye check
```

1. Tape a marker **flat and centred** on the magnet face.
2. `capture --pad` drives the arm and head with the XInput pad instead of by hand (hold RT +
   sticks for the hand, RB + right stick for the head, **B** takes a sample, BACK undoes, START
   quits; plug in the pad instead of the magnet's ESP32 if ports are short). Without `--pad`:
   `f` frees the arm (move it by hand) and holds it again; `space` takes a sample
   (10 frames, refused if the point jitters > 4 mm); `a/d w/s` move the head; `u` undoes;
   `x` saves and quits. Samples append to
   `~/.cache/huggingface/lerobot/calibration/robots/sortbots_head/handeye_samples.json`
   (`--fresh` starts over).
3. Collect 6+ poses at ONE head pose that sees the workspace (on our rig head 0/0 looks level
   across the room, over the arm: tilt down first), then 4+ at each of pan ±20 and tilt ±10-15
   from there: magnet spread
   across the image and over 25–60 cm from the lens, tool tilted different ways, marker facing
   the camera.
4. `solve` prints every sample's error and saves `sortbots_head_camera.json` next to the head
   calibration. Expect an RMS of a few mm. Leave flagged outliers out with `--drop 7 12`.
   Over 1 cm: wrong marker size, a marker not flat on the face, or a stale arm calibration.
5. `check` (torque off) prints, for every visible package, its magnet target in base_link cm,
   `up` (the face normal's z: ~+1.00 for a top face) and `d`, the magnet face's distance from
   the target. Put the magnet on a package's circle by hand: `d` should read under ~1 cm.

The target comes from the marker's centre (depth), its in-plane rotation (the marker), and the
face's tilt (a plane fit to depth over three marker-widths round it): a 2 cm marker's own tilt
is degrees off, and at 52 mm from the target that would be millimetres.

## Touch calibration (no marker on the magnet)

A marker on the magnet face gets in the way of picking, so `capture --touch` uses the package
instead: its marker (id 10 by default) gives the circle centre through the measured offset, and
the magnet touched onto that circle gives the arm's side. Each sample is two presses of B: LOOK
(arm clear of the package, camera reads the circle) then TOUCH (magnet on the circle centre,
lightly). The first three touches at home just need to be reachable and form a triangle;
from them capture fits a rough camera pose and draws the REACH MAP (shaded dots: package-top
spots the magnet can touch pointing down, from a sweep of the arm's straight-down poses). After
that each green box sits on the reachable spot farthest from the samples already taken, and
from the fourth sample every third package goes up on a book or box.

```bash
scripts/arm.sh head handeye capture --touch --pad
```

The tool always points down for a touch, so the magnet's length and the camera's height can't
be told apart: `solve` holds the length at `--magnet-m` (47.1 mm) for touch-only sets. Picks
approach the same way with the same length, so it cancels there. Synthetic check: 22 touch
samples, 1.5 mm noise, 2.0 mm RMS, 3.4 mm worst at head angles never fitted.

## The arm's joint offsets (solved here too)

lerobot puts each joint's 0 deg at the middle of the range you swept when calibrating, and
`scripts/so101_kin.py` assumed that is the URDF's zero. On our arm it wasn't: the magnet held flat
on the table read z = -11 cm (2026-11-11). `solve` therefore also fits shoulder_lift, elbow_flex
and wrist_flex offsets (15+ samples; `--no-offsets` to skip) and writes them to
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/sortbots_arm_kin_offsets.json`,
backing up any older one. `SO101Kin` applies them inside its FK, so teleop, IK, `check` and picks
all use the corrected arm without other changes. Recalibrating the arm (`lerobot-calibrate`)
moves its zeros: run capture and solve again after one.

Why not a table touch: lift +d, elbow -2d, wrist +d keeps the magnet face flat and at the same
height, so a flat surface can't tell those apart. The camera sees the reach that combination
changes. Pan's offset is a rigid turn of the whole arm, which the camera pose absorbs. Synthetic
check: 22 touches with offsets of 20 / -15 / 10 deg come back as 19.6 / -14.6 / 9.2 at 2.0 mm RMS.

Until offsets exist, capture's reach map is off (a map built on a 10 cm-wrong arm boxed
unreachable spots) and the guide spreads the touches by text.

## Why it is fitted, not measured

The camera's pose follows two servos, and the arm's own FK carries the hand calibration's
"URDF zero = middle of the range" error. Ruler numbers are 5–10 mm off; a fit against the
arm's FK absorbs most of both and reports what is left. The 15 numbers: pan frame (6), tilt
axis offset and roll (2), camera on the tilt frame (6), magnet length (1).

Each sample stores the depth point (D435 depth at the marker centre, deprojected) and the
marker's PnP point; the fit uses depth and falls back to PnP where depth has a hole. A 20 mm
marker's PnP *orientation* is noisy, so plate normals should come from depth, not the marker.

`scripts/so101_kin.py` still steers teleop by the stock gripper's tip (98 mm out); the magnet
face is `SO101Kin.tool_point(q, length)` with the fitted length.
