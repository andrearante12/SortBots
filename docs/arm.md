# Physical arm (SO-101, 5-DOF) — gamepad teleop

Hardware: 5 STS3215 servos (12 V version) on a Waveshare-style bus board, no gripper.
Switch Pro Controller over bluetooth. Runs in the `lerobot` conda env (python 3.12), never
in a ROS shell.

```bash
scripts/arm.sh            # cartesian teleop: sticks move the hand
scripts/arm.sh fk         # torque OFF, move by hand, watch hand xyz (check the model)
scripts/arm.sh dry        # print IK targets, send nothing
scripts/arm.sh joint      # joint-space teleop
```

Hold **ZR** (deadman) for everything. Left stick = hand radial/swing, right stick Y = up/down,
right stick X = wrist roll, L/ZL = tool tilt, X/B = wrist_flex joint, d-pad up/down = speed
level (1-4), A = slow (x0.25), + = quit. Details: header of `scripts/arm_cartesian.py`.

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
