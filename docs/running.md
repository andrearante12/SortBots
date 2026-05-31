# Running the sim

All commands below assume `conda activate lerobot` and `cd ~/sortbots_ws`.

## The verify script

```bash
bash scripts/verify_sim.sh
```

Runs three things in order:

1. **Sanity imports + headless rollout** — imports `mani_skill`, `torch`, `Xlerobot`, builds a `ReplicaCAD_SceneManipulation-v1` env with the xlerobot agent, resets, and steps 5 zero-actions. Fails fast (before any window opens) if the overlay is broken or the conda env is misconfigured.
2. **Demo 1** — `demo_random_action` on Fetch in ReplicaCAD, `rt-fast` shader.
3. **Demo 2** — XLeRobot dual-arm in ReplicaCAD via `scripts/run_xle_demo.py`, `default` shader.

Close each SAPIEN viewer window to advance to the next step.

## The XLeRobot demo launcher

ManiSkill's bundled `demo_random_action` works with `-r xlerobot` but only steps random actions. For richer control (keyboard, gamepad, Rerun visualization), use XLeRobot's own example scripts under `third_party/XLeRobot/simulation/Maniskill/examples/`. They don't import `mani_skill.envs` themselves, so a thin shim is required to populate gym's env registry before they run. That shim is `scripts/run_xle_demo.py`:

```bash
python scripts/run_xle_demo.py <demo_name> [demo args...]
# List all available demos:
python scripts/run_xle_demo.py --help
```

`<demo_name>` is the file stem of any `.py` under the XLeRobot examples directory.

## Demo catalog

All examples below use the ReplicaCAD apartment scene. Substitute another mani-skill env id (e.g. `PushCube-v1`) for tabletop tasks.

### Preset action stepper (what verify_sim.sh runs as Demo 2)

```bash
python scripts/run_xle_demo.py demo_ctrl_action \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" \
    --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"
```

### Keyboard end-effector teleop, dual-arm

```bash
python scripts/run_xle_demo.py demo_ctrl_action_ee_keyboard \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" \
    --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"
```

Keymap prints on launch (WASD for the mobile base, plus per-arm IK targets).

### Keyboard end-effector teleop, single-arm

```bash
python scripts/run_xle_demo.py demo_ctrl_action_ee_keyboard_single \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot_single" \
    --render-mode="human" \
    --shader="default" \
    -c "pd_joint_delta_pos"
```

### Xbox / Switch / Bluetooth controller teleop

```bash
python scripts/run_xle_demo.py demo_ctrl_action_ee_xbox \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"
```

Sanity-check the controller bindings first with `python scripts/run_xle_demo.py test_xbox`.

### Camera streams visualized with Rerun

```bash
python scripts/run_xle_demo.py demo_ctrl_action_ee_cam_rerun \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"
```

Opens a Rerun viewer alongside the SAPIEN window with per-camera streams.

### Record a teleop dataset

```bash
python scripts/run_xle_demo.py demo_ctrl_ee_keyboard_record_dataset \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"
```

Writes episodes to the path the script prints. Useful as input to LeRobot imitation-learning pipelines.

### VR teleop (Quest 3)

```bash
python scripts/run_xle_demo.py demo_ctrl_action_ee_VR \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"
```

Requires a Quest 3 reachable from the host; see the XLeRobot docs for the pairing flow.

## Supported control modes for xlerobot

Pass with `-c`. From `Xlerobot.supported_control_modes`:

- Dual-arm: `pd_joint_pos_dual_arm`, `pd_joint_delta_pos_dual_arm`
- Single primary arm: `pd_joint_pos`, `pd_joint_delta_pos`, `pd_joint_target_delta_pos`, `pd_joint_vel`, `pd_joint_pos_vel`, `pd_joint_delta_pos_vel`, `pd_joint_delta_pos_stiff_body`
- Second arm (suffixed `_arm2`): same set as above with `_arm2` appended

Use `xlerobot_single` as `-r` when running the single-arm variant of XLeRobot.

## Headless usage

For automation / CI / RL rollouts, set `--render-mode="rgb_array"` (returns batched images via `env.render()`) or `--render-mode="sensors"` (only the agent's onboard cameras). The headless smoke test inside `verify_sim.sh` shows the pattern.

## Bumping the XLeRobot submodule

```bash
git submodule update --remote third_party/XLeRobot
python scripts/overlay_xlerobot.py    # re-apply overlay against the new submodule
git add third_party/XLeRobot && git commit -m "bump XLeRobot"
```

See [`overlay.md`](overlay.md) for what the overlay step does and when it might need adjustment.
