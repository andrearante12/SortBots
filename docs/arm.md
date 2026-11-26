# Physical arm (SO-101, 5-DOF) — gamepad teleop

Hardware: 5 STS3215 servos (12 V version) on a Waveshare-style bus board, no gripper.
Switch Pro Controller over bluetooth. Runs in the `lerobot` conda env (python 3.12), never
in a ROS shell.

```bash
scripts/arm.sh            # cartesian teleop: sticks move the hand
scripts/arm.sh fk         # torque OFF, move by hand, watch hand xyz (check the model)
scripts/arm.sh dry        # print IK targets, send nothing
scripts/arm.sh joint      # joint-space teleop
scripts/arm.sh each       # one joint at a time: hold Y/X/A/B/R = pan/lift/elbow/wrist_flex/roll, stick moves it
```

Hold **ZR** (deadman) for everything. Left stick = hand radial/swing, right stick Y = up/down,
right stick X = wrist roll, L/ZL = tool tilt, X/B = wrist_flex joint, d-pad up/down = speed
level (1-4), A = slow (x0.25), + = quit. Details: header of `scripts/arm_cartesian.py`.

## XInput gamepad (Flydigi Vader 5 Pro) + magnet

`scripts/arm.sh pad` drives the arm with an XInput pad and the electromagnet (`docs/magnet.md`).
Default is hand-space (same IK as `arm_cartesian.py`): left stick = hand on a horizontal plane
(forward/back, swing), right stick Y = up/down, hold **RT** to move, **B** toggles the magnet,
**Y/A** move the `wrist_flex` joint directly (LB/LT tilt the tool in hand space), d-pad up/down = speed level (1-4, starts at 4 = the fastest the 3 A supply handled; `--max-level 6` unlocks faster ones once the supply is 5 A or more), hold **L3** (left stick click) = slow, START quits. One status line rewrites in place: hand x/y/z,
magnet state, servo bus voltage and hottest servo.
`pad --joint` is joint space; `pad --no-arm` tests the controller and magnet with a virtual arm.
`wrist_roll` is locked by default (`--lock`).

The Jetson kernel has no `xpad`, so `scripts/xinput_pad.py` reads the pad over raw USB (pyusb).
On a desktop kernel that does have `xpad` (the thinkpad), the reader detaches it from the pad's
interface for the session and reattaches it on exit. `pyusb` must be in the `lerobot` env
(`~/miniconda3/envs/lerobot/bin/pip install pyusb`).
After every plug the USB node is `root:root`: `sudo chmod 666 /dev/bus/usb/BUS/DEV` (the script
prints the exact path when it gets permission denied).

## Pan/tilt head (same bus)

Two STS3215 servos daisy-chained onto the arm's bus board: `head_pan` = id 7, `head_tilt` = id 8
(XLeRobot's head ids; 6 stays free for a gripper). Calibration is its own file
(`~/.cache/huggingface/lerobot/calibration/robots/sortbots_head/sortbots_head.json`), so
`lerobot-calibrate` on the arm never touches it. Driven by `scripts/head.py` via `arm.sh head`:

```bash
scripts/arm.sh head setup pan    # pan servo ALONE on the board -> id 7
scripts/arm.sh head setup tilt   # tilt servo ALONE on the board -> id 8
scripts/arm.sh head scan         # chain everything back: expect ids 1-5, 7, 8
scripts/arm.sh head calibrate    # center (straight ahead, camera level), then sweep each by hand
scripts/arm.sh head read         # torque off: check 0/0 is centered and the signs
scripts/arm.sh head keys         # a/d pan, w/s tilt, c center
```

- **Setup needs each servo alone**: new STS3215s ship as id 1, the same as `shoulder_pan`.
  `head.py setup` counts servos across all baud rates and refuses unless exactly one answers
  (lerobot's own `setup_motor` would renumber whichever it found first).
- **Voltage**: the arm's board runs 12 V. Only chain 12 V STS3215s (the head servos must be
  the same variant as the arm's); a 7.4 V one on this bus is cooked. `head scan` prints volts.
- One process owns `/dev/ttyACM0`: stop the arm teleop before running `head.py`, and vice versa.
  Driving both from one session (e.g. the pad) means one `SerialMotorsBus` with all 7 motors.
- Wrong direction: `head calibrate --invert head_pan` (sign flipped in software).
- **On the pad** (`arm.sh pad`): hold **RB** and the right stick pans/tilts the head (no RT
  needed; the right stick leaves the arm alone while RB is held), **X** re-centers it. One axis
  moves at a time: whichever way the stick leans more. The head
  joins the follower's own bus, so it is on whenever its calibration file exists; `--no-head`
  for the arm alone, `--head-invert pan|tilt` to flip a stick, `--head-dps` for speed.

- **wrist_flex froze near magnet-straight-down** (fixed 2026-11-10): Y/A picked "up" from the
  tool pitch at the current pose, and pitch peaks there (~+/-80 deg wrist_flex), so the direction
  flipped every tick. Y/A now take the direction from the zero pose. LB/LT (hand-space tilt)
  still cannot tilt the tool past vertical: past it, use Y/A.
- **A joint that sometimes stops responding** for any other reason is usually servo overload protection: past
  `Overload_Torque` (80%) for `Protection_Time` (200 ms) a STS3215 drops to `Protective_Torque`
  (20%), too weak to lift the magnet. The pad's status line shows `FAULT <joint> overload` (any
  nonzero Status bits) for 10 s when it happens.

## One-time setup

```bash
~/miniconda3/bin/conda create -y -n lerobot python=3.12 ffmpeg -c conda-forge
git clone https://github.com/huggingface/lerobot.git ~/lerobot && cd ~/lerobot
git checkout -b sortbots-5dof
# 5-DOF patch: delete the `"gripper": Motor(6, ...)` line in BOTH
#   src/lerobot/robots/so_follower/so_follower.py
#   src/lerobot/teleoperators/so_leader/so_leader.py
~/miniconda3/envs/lerobot/bin/pip install -e ".[feetech]" evdev
sudo usermod -aG dialout $USER      # then log out/in

lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/ttyACM0   # IDs 5..1, one servo at a time
lerobot-calibrate    --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=sortbots_arm
```

Stock lerobot hard-codes six motors and fails its handshake without the gripper, which is
why the patch exists. Pull lerobot by rebasing `sortbots-5dof`, not `git pull`.

## Things that cost time

- **The board drops off USB** under load and returns as `root:dialout 660`: `sudo chmod 666
  /dev/ttyACM0` until the login session has `dialout`. Servos timing out then the port
  vanishing = power, not software. A 3 A / 12 V supply was marginal at full speed (level 4);
  use 5 A+ before raising `LEVELS` in `arm_cartesian.py`.
- **Pro Controller buttons**: hid-nintendo reports physical B as `BTN_SOUTH`, A as
  `BTN_EAST`, X as `BTN_NORTH`. Don't trust the label in docs for other pads.
- **Kinematics** (`scripts/so101_kin.py`) use `configs/so101_new_calib.urdf`; URDF zero is
  the middle of each calibrated range. If a stick moves the hand the wrong way, run
  `arm.sh fk`, then `arm.sh -- --flip JOINT`.
- A running teleop session grabs the controller (evdev `grab`); nothing else can read it.
