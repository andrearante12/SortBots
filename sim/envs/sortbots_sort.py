"""SortBotsSort-v0: pick a colored cube, place it in the commanded bin.

The VLA research task env (see the approved plan in
`~/.claude/plans/i-want-to-reseaerch-gleaming-summit.md`, Track 2). Modeled
directly on ManiSkill's own `PickCube-v1`
(`mani_skill/envs/tasks/tabletop/pick_cube.py`) — same TableSceneBuilder,
same tcp/is_grasping/is_static evaluate() pattern — swapped to the
XLeRobot agent (dual arm + holonomic base) with two colored target bins
instead of one floating goal sphere, and a per-episode language
instruction naming which bin.

Two configs, one env, selected via `base_mobility`:
  - "frozen" (default, the plan's primary hybrid-architecture config):
    the holonomic base's action component is zeroed every step. Matches
    "VLA does manipulation only" — the base stays docked, like Nav2 having
    already driven the robot to a station in the real pipeline.
  - "free": base action passes through untouched, for the plan's optional
    whole-body VLA comparison experiment.

Requires the XLeRobot ManiSkill overlay (`scripts/overlay_xlerobot.py`)
already applied — this module doesn't import Xlerobot directly, it relies
on `mani_skill.agents.robots` having been monkey-patched with it, exactly
like every other script in this repo's ManiSkill track.
"""
from __future__ import annotations

import random
from typing import Any

import numpy as np
import sapien
import torch
from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.sensors.camera import CameraConfig
from mani_skill.utils import sapien_utils
from mani_skill.utils.building import actors
from mani_skill.utils.registration import register_env
from mani_skill.utils.scene_builder.table import TableSceneBuilder
from mani_skill.utils.structs.pose import Pose

CUBE_HALF_SIZE = 0.02
CUBE_COLORS = {
    "red": [1, 0, 0, 1],
    "blue": [0, 0, 1, 1],
}
BIN_RADIUS = 0.05
# (name, color, xy position on the table)
BINS = [
    ("left", [0.2, 0.6, 0.2, 1], (0.15, -0.15)),
    ("right", [0.9, 0.6, 0.1, 1], (0.15, 0.15)),
]
CUBE_SPAWN_CENTER = (0.0, 0.0)
CUBE_SPAWN_HALF_SIZE = 0.05


@register_env("SortBotsSort-v0", max_episode_steps=100)
class SortBotsSortEnv(BaseEnv):
    """**Task:** grasp the {cube_color} cube and place it in the {bin_name} bin.

    Language instruction is regenerated every episode (`self.instruction`,
    also returned in `evaluate()`'s info dict as `instruction`) by randomly
    picking the cube color and target bin.
    """

    SUPPORTED_ROBOTS = ["xlerobot"]
    _sample_video_link = None

    def __init__(
        self,
        *args,
        robot_uids="xlerobot",
        base_mobility="frozen",
        robot_init_qpos_noise=0.02,
        **kwargs,
    ):
        assert base_mobility in ("frozen", "free"), base_mobility
        self.base_mobility = base_mobility
        self.robot_init_qpos_noise = robot_init_qpos_noise
        self.instruction = ""
        self._target_bin_idx = 0
        self._cube_color_name = "red"
        super().__init__(*args, robot_uids=robot_uids, **kwargs)

    # No env-level _default_sensor_configs: the XLeRobot agent already
    # mounts the 3 cameras that matter for the VLA (fetch_head,
    # fetch_left_arm_camera, fetch_right_arm_camera — see
    # agents/xlerobot/xlerobot.py:_sensor_configs), same topology as the
    # real robot / Isaac Sim track. Verified they appear automatically in
    # `obs_mode="rgbd"` observations without any env-level camera config.

    @property
    def _default_human_render_camera_configs(self):
        pose = sapien_utils.look_at(eye=[0.8, 0.6, 0.8], target=[0.0, 0.0, 0.1])
        return CameraConfig("render_camera", pose, 512, 512, 1, 0.01, 100)

    def _load_agent(self, options: dict):
        # PLACEHOLDER offset — picked to put the table in front of the
        # XLeRobot's arm workspace; not tuned against a live render yet
        # (see sim/README or the plan's Phase C for the tuning pass).
        super()._load_agent(options, sapien.Pose(p=[-0.7, 0, 0]))

    def _load_scene(self, options: dict):
        self.table_scene = TableSceneBuilder(
            self, robot_init_qpos_noise=self.robot_init_qpos_noise
        )
        self.table_scene.build()

        self.cube = actors.build_cube(
            self.scene,
            half_size=CUBE_HALF_SIZE,
            color=CUBE_COLORS["red"],
            name="cube",
            initial_pose=sapien.Pose(p=[0, 0, CUBE_HALF_SIZE]),
        )

        self.bins = []
        for name, color, (bx, by) in BINS:
            bin_actor = actors.build_cylinder(
                self.scene,
                radius=BIN_RADIUS,
                half_length=0.002,
                color=color,
                name=f"bin_{name}",
                body_type="kinematic",
                add_collision=False,
                initial_pose=sapien.Pose(p=[bx, by, 0.001]),
            )
            self._hidden_objects.append(bin_actor)
            self.bins.append(bin_actor)

    def _initialize_episode(self, env_idx: torch.Tensor, options: dict):
        with torch.device(self.device):
            b = len(env_idx)
            self.table_scene.initialize(env_idx)

            xyz = torch.zeros((b, 3))
            xyz[:, 0] = CUBE_SPAWN_CENTER[0] + (
                torch.rand(b) * 2 - 1
            ) * CUBE_SPAWN_HALF_SIZE
            xyz[:, 1] = CUBE_SPAWN_CENTER[1] + (
                torch.rand(b) * 2 - 1
            ) * CUBE_SPAWN_HALF_SIZE
            xyz[:, 2] = CUBE_HALF_SIZE
            qs = torch.zeros((b, 4))
            qs[:, 0] = 1.0  # identity quaternion; cube is symmetric enough not to need z-rot randomization
            self.cube.set_pose(Pose.create_from_pq(xyz, qs))

            # Single source of truth for the episode's language instruction.
            # Only meaningful for b==1 (teleop/eval); batched GPU envs would
            # need a per-env instruction list, not in scope for this env yet.
            color_name = random.choice(list(CUBE_COLORS))
            bin_idx = random.randrange(len(BINS))
            self._cube_color_name = color_name
            self._target_bin_idx = bin_idx
            self.instruction = f"put the {color_name} cube in the {BINS[bin_idx][0]} bin"

    def _get_obs_extra(self, info: dict):
        obs = dict(
            is_grasped=info["is_grasped"],
            tcp_pose=self.agent.tcp_pose.raw_pose,
        )
        if "state" in self.obs_mode:
            obs.update(obj_pose=self.cube.pose.raw_pose)
        return obs

    def evaluate(self):
        target_bin_pos = self.bins[self._target_bin_idx].pose.p
        xy_dist = torch.linalg.norm(
            (target_bin_pos - self.cube.pose.p)[:, :2], axis=1
        )
        is_obj_in_bin = xy_dist <= BIN_RADIUS
        is_obj_resting = torch.abs(self.cube.pose.p[:, 2] - CUBE_HALF_SIZE) < 0.01
        is_grasped = self.agent.is_grasping(self.cube, arm_id=1)
        is_robot_static = self.agent.is_static(0.2)
        success = is_obj_in_bin & is_obj_resting & (~is_grasped) & is_robot_static
        return {
            "success": success,
            "is_obj_in_bin": is_obj_in_bin,
            "is_grasped": is_grasped,
            "is_robot_static": is_robot_static,
            "instruction": self.instruction,
        }

    def compute_dense_reward(self, obs: Any, action: torch.Tensor, info: dict):
        tcp_to_obj_dist = torch.linalg.norm(
            self.cube.pose.p - self.agent.tcp_pose.p, axis=1
        )
        reward = 1 - torch.tanh(5 * tcp_to_obj_dist)
        is_grasped = info["is_grasped"]
        reward += is_grasped

        target_bin_pos = self.bins[self._target_bin_idx].pose.p
        obj_to_bin_dist = torch.linalg.norm(
            (target_bin_pos - self.cube.pose.p)[:, :2], axis=1
        )
        reward += (1 - torch.tanh(5 * obj_to_bin_dist)) * is_grasped
        reward[info["success"]] = 5
        return reward

    def compute_normalized_dense_reward(self, obs: Any, action: torch.Tensor, info: dict):
        return self.compute_dense_reward(obs=obs, action=action, info=info) / 5

    def step(self, action):
        if self.base_mobility == "frozen":
            action = self._zero_base_action(action)
        return super().step(action)

    def _zero_base_action(self, action):
        """Zero the flat action vector's "base" slice for the frozen config.

        XLeRobot's `CombinedController` concatenates named sub-controllers
        (arm/gripper/body/base for the default `pd_joint_delta_pos`) into
        ONE flat `Box` action space, not a `Dict` — verified against the
        real installed agent (`agent.controller.controllers` is an
        OrderedDict of sub-controllers, each with its own `action_space`).
        This walks that dict to find the base's slice dynamically so it
        keeps working if the active controller mode changes, instead of
        hardcoding indices for one specific mode.
        """
        controllers = self.agent.controller.controllers
        if "base" not in controllers:
            return action
        offset = 0
        for name, sub in controllers.items():
            dim = sub.action_space.shape[0]
            if name == "base":
                action = action.clone() if isinstance(action, torch.Tensor) else np.array(action, copy=True)
                action[..., offset : offset + dim] = 0
                break
            offset += dim
        return action
