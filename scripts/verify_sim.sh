#!/usr/bin/env bash
# Hello-world verification for the XLeRobot sim. Opens two SAPIEN windows.
set -euo pipefail

# --- Guard: ROS 2 must not be sourced ---------------------------------------
if [[ -n "${AMENT_PREFIX_PATH:-}" || -n "${ROS_DISTRO:-}" ]]; then
    echo "ERROR: ROS 2 is sourced; open a fresh shell and re-run." >&2
    exit 1
fi

# --- Conda activate ---------------------------------------------------------
if ! command -v conda >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
else
    # shellcheck disable=SC1091
    source "$(conda info --base)/etc/profile.d/conda.sh"
fi
conda activate lerobot

echo "==> Sanity imports + headless rollout (fail-fast before opening windows)"
python - <<'PY'
import mani_skill, torch
print(f"mani_skill {mani_skill.__version__}, torch {torch.__version__}, cuda={torch.cuda.is_available()}")
from mani_skill.agents.robots import Xlerobot
print(f"Xlerobot.uid = {Xlerobot.uid!r}")

import gymnasium as gym
import mani_skill.envs  # registers envs

env = gym.make(
    "ReplicaCAD_SceneManipulation-v1",
    robot_uids="xlerobot",
    control_mode="pd_joint_delta_pos_dual_arm",
    render_mode=None,
    num_envs=1,
)
obs, _ = env.reset(seed=0)
action = env.action_space.sample() * 0.0
for _ in range(5):
    env.step(action)
env.close()
print(f"Headless OK; action space dim = {env.action_space.shape[0]}")
PY

echo
echo "==> Demo 1: random actions in ReplicaCAD (rt-fast shader; close window to continue)"
python -m mani_skill.examples.demo_random_action \
    -e "ReplicaCAD_SceneManipulation-v1" \
    --render-mode="human" \
    --shader="rt-fast"

echo
echo "==> Demo 2: keyboard-controlled XLeRobot in ReplicaCAD (default shader)"
echo "    (uses XLeRobot's own demo_ctrl_action.py from the submodule via the run_xle_demo shim)"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python "$REPO_ROOT/scripts/run_xle_demo.py" demo_ctrl_action \
    -e "ReplicaCAD_SceneManipulation-v1" \
    -r "xlerobot" \
    --render-mode="human" \
    --shader="default" \
    -c "pd_joint_delta_pos_dual_arm"

echo
echo "Verification complete."
