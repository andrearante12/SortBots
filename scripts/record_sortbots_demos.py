#!/usr/bin/env python3
"""Keyboard-teleop demo recorder for SortBotsSort-v0.

Records native ManiSkill trajectories (HDF5 + json) via `RecordEpisode`,
plus a sidecar `episode_instructions.json` mapping episode_id -> the
per-episode language instruction (`env.unwrapped.instruction`). That
sidecar exists because `RecordEpisode`'s own per-episode metadata was
checked against the real installed wrapper and does NOT include arbitrary
env info — it only captures episode_id/seed/control_mode/elapsed_steps/
success (see `mani_skill.utils.wrappers.RecordEpisode`). Since
`mani_skill.trajectory.convert_to_lerobot`'s `--task-name` is one string
for the whole file (not per-episode), converting THIS recording into a
LeRobotDataset with correct per-episode language labels needs a small
postprocessing pass that reads the sidecar and patches the converted
dataset's `task` column — not written yet; that's the next piece of Track 2
after some demos exist to convert.

Controls (arm deltas on the active arm's 5 joints, matching
`pd_joint_delta_pos`'s `arm` slice — Rotation/Pitch/Elbow/Wrist_Pitch/
Wrist_Roll — plus gripper and, in "free" base_mobility, the base):

    Q/A  Rotation +/-        W/S  Pitch +/-         E/D  Elbow +/-
    R/F  Wrist_Pitch +/-     T/G  Wrist_Roll +/-
    SPACE  toggle gripper open/closed
    ARROWS  base forward/back/turn (only does anything if --base-mobility free)
    N  finish episode now (counts as recorded, whatever state it's in)
    ESC  quit (episodes recorded so far are already saved)

Run (system ROS 2 must NOT be sourced in this shell — conda only):

    conda activate lerobot
    python scripts/record_sortbots_demos.py --episodes 70 --out demos/sortbots_sort_frozen

UNVERIFIED: the keyboard control loop needs a live display + human at the
keyboard to actually test — nobody has run this interactively yet. The
env wiring (RecordEpisode, action-space slicing, instruction sidecar) IS
verified — see the non-interactive smoke test in `tests/` (or run this
file with `--smoke-test` for a scripted, no-keyboard sanity pass).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import gymnasium as gym
import numpy as np

SCRIPTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPTS_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "sim" / "envs"))

import sortbots_sort  # noqa: E402,F401  registers SortBotsSort-v0
from mani_skill.utils.wrappers import RecordEpisode  # noqa: E402

ARM_DELTA = 0.15  # fraction of the controller's [-1,1] delta range per keypress
BASE_DELTA = 0.3
# Index within the arm's 5-dim slice for each named joint (Rotation, Pitch,
# Elbow, Wrist_Pitch, Wrist_Roll — see agents/xlerobot/xlerobot.py's
# arm_joint_names, and the controller composition confirmed live in
# sim/envs/sortbots_sort.py's `_zero_base_action`).
ARM_KEYS = {
    "q": (0, +1), "a": (0, -1),
    "w": (1, +1), "s": (1, -1),
    "e": (2, +1), "d": (2, -1),
    "r": (3, +1), "f": (3, -1),
    "t": (4, +1), "g": (4, -1),
}


def _action_slices(env):
    """Offsets of each named sub-controller in the flat action vector.

    Same technique as `SortBotsSortEnv._zero_base_action` — walks
    `agent.controller.controllers` rather than hardcoding indices, so this
    keeps working if the control_mode changes.
    """
    slices = {}
    offset = 0
    for name, sub in env.unwrapped.agent.controller.controllers.items():
        dim = sub.action_space.shape[0]
        slices[name] = (offset, offset + dim)
        offset += dim
    return slices


def run_smoke_test(out_dir: Path) -> None:
    """Non-interactive sanity pass: record a couple of scripted episodes.

    Doesn't exercise the keyboard loop (needs a real display + human), but
    proves RecordEpisode + the instruction sidecar + the action-space
    bookkeeping all work together end-to-end.
    """
    env = gym.make(
        "SortBotsSort-v0", robot_uids="xlerobot", obs_mode="rgbd",
        render_mode=None, base_mobility="frozen",
    )
    env = RecordEpisode(
        env, output_dir=str(out_dir), trajectory_name="smoke_test",
        save_video=False, record_env_state=True,
    )
    instructions = {}
    for ep in range(2):
        obs, info = env.reset(seed=ep)
        instructions[ep] = env.unwrapped.instruction
        for _ in range(5):
            env.step(env.action_space.sample())
    env.close()
    with open(out_dir / "episode_instructions.json", "w") as f:
        json.dump(instructions, f, indent=2)
    print(f"smoke test OK: wrote {out_dir}/smoke_test.h5 + episode_instructions.json")


def run_teleop(out_dir: Path, num_episodes: int, base_mobility: str) -> None:
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((500, 300))
    pygame.display.set_caption("SortBots demo recorder — keyboard focus here")
    font = pygame.font.SysFont(None, 22)

    env = gym.make(
        "SortBotsSort-v0", robot_uids="xlerobot", obs_mode="rgbd",
        render_mode="human", base_mobility=base_mobility,
    )
    env = RecordEpisode(
        env, output_dir=str(out_dir), trajectory_name="demos",
        save_video=False, record_env_state=True,
    )
    slices = _action_slices(env)
    arm_lo, arm_hi = slices["arm"]
    gripper_lo, _ = slices["gripper"]
    base_lo, base_hi = slices.get("base", (None, None))

    instructions = {}
    gripper_closed = False
    episode = 0
    obs, info = env.reset(seed=episode)
    instructions[episode] = env.unwrapped.instruction
    print(f"episode {episode}: {env.unwrapped.instruction!r}")

    running = True
    while running and episode < num_episodes:
        action = np.zeros(env.action_space.shape, dtype=np.float32)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    gripper_closed = not gripper_closed
                elif event.key == pygame.K_n:
                    episode += 1
                    if episode < num_episodes:
                        obs, info = env.reset(seed=episode)
                        instructions[episode] = env.unwrapped.instruction
                        print(f"episode {episode}: {env.unwrapped.instruction!r}")

        keys = pygame.key.get_pressed()
        for key_name, (joint_idx, sign) in ARM_KEYS.items():
            if keys[getattr(pygame, f"K_{key_name}")]:
                action[arm_lo + joint_idx] = sign * ARM_DELTA
        action[gripper_lo] = 1.0 if gripper_closed else -1.0
        if base_lo is not None:
            if keys[pygame.K_UP]:
                action[base_lo] = BASE_DELTA
            if keys[pygame.K_DOWN]:
                action[base_lo] = -BASE_DELTA
            if keys[pygame.K_LEFT]:
                action[base_hi - 1] = BASE_DELTA
            if keys[pygame.K_RIGHT]:
                action[base_hi - 1] = -BASE_DELTA

        env.step(action)
        env.render()

        screen.fill((20, 20, 20))
        lines = [
            f"episode {episode}/{num_episodes}",
            f"instruction: {env.unwrapped.instruction}",
            f"gripper: {'closed' if gripper_closed else 'open'}",
            "Q/A W/S E/D R/F T/G = arm joints, SPACE = gripper, N = next ep, ESC = quit",
        ]
        for i, line in enumerate(lines):
            screen.blit(font.render(line, True, (230, 230, 230)), (10, 10 + 24 * i))
        pygame.display.flip()

    env.close()
    pygame.quit()
    with open(out_dir / "episode_instructions.json", "w") as f:
        json.dump(instructions, f, indent=2)
    print(f"recorded {episode} episodes -> {out_dir}/demos.h5 + episode_instructions.json")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=70)
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "demos" / "sortbots_sort")
    parser.add_argument("--base-mobility", choices=("frozen", "free"), default="frozen")
    parser.add_argument("--smoke-test", action="store_true")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    if args.smoke_test:
        run_smoke_test(args.out)
    else:
        run_teleop(args.out, args.episodes, args.base_mobility)


if __name__ == "__main__":
    main()
