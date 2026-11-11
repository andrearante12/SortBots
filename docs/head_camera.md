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
2. `capture`: `f` frees the arm (move it by hand) and holds it again; `space` takes a sample
   (10 frames, refused if the point jitters > 4 mm); `a/d w/s` move the head; `u` undoes;
   `x` saves and quits. Samples append to
   `~/.cache/huggingface/lerobot/calibration/robots/sortbots_head/handeye_samples.json`
   (`--fresh` starts over).
3. Collect 6+ poses at head 0/0, then 4+ at each of pan ±20 and tilt ±15: magnet spread
   across the image and over 25–60 cm from the lens, tool tilted different ways, marker facing
   the camera.
4. `solve` prints every sample's error and saves `sortbots_head_camera.json` next to the head
   calibration. Expect an RMS of a few mm. Leave flagged outliers out with `--drop 7 12`.
   Over 1 cm: wrong marker size, a marker not flat on the face, or a stale arm calibration.
5. `check` (torque off) prints each visible marker in base_link cm and its distance from the
   magnet face. Put the magnet on a package's marker: `d` should read under ~1 cm.

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
