#!/usr/bin/env python3
"""Scripted IK pick/place for the SortBots warehouse demo (Milestone 2).

Replaces Milestone 1's instant attach/detach with a visible arm motion:
STOW -> APPROACH -> GRASP -> (close/open jaw, weld-assist via Isaac's mock
pick) -> LIFT -> STOW. Still no real grasp physics — Isaac's
`_sync_package_to_gripper` (see spawn_warehouse.py) keeps teleporting the
package onto the gripper tip while attached, same as Milestone 1. What's
new here is that the arm actually reaches for it instead of teleporting the
whole box.

Protocol (see docstrings on the classes below for detail):

    task_manager.py --[[/<robot>/pick_request: "attach"|"detach"]]--> this node
    this node        --[[/<robot>/arm_joint_cmd: JointState]]--------> Isaac (spawn_warehouse.py)
    this node        --[[/<robot>/pick_cmd: "attach"|"detach"]]------> Isaac (mock weld, unchanged from M1)
    this node        --[[/<robot>/pick_result: "picked"|"placed"|"failed"]]--> task_manager.py

IK: `pytorch_kinematics.PseudoInverseIK` over the arm chain from
`third_party/XLeRobot/.../xlerobot.urdf` (`base_link` -> `Fixed_Jaw_tip`,
the first/right arm). Full derivation notes:

- The chain rooted at `base_link` unexpectedly still includes
  `root_z_rotation_joint` as its first DOF (URDF: root -> root_x ->
  root_y -> root_z -> base_link -> ...arm...; pytorch_kinematics'
  `root_link_name` doesn't trim joints strictly upstream of it). Pinning
  that DOF's limits to `(0, 0)` makes every solve arm-relative regardless
  of the robot's world yaw — verified: rotating that pinned DOF's value
  rotates the resulting end-effector position purely about Z with
  constant radius, confirming it behaves exactly like the base's own yaw.
- `PseudoInverseIK.joint_limits` is NOT a hard constraint on the returned
  solution — verified against the real installed library (pytorch-kinematics
  0.7.6): a solve returned angles outside the URDF's stated limits (e.g.
  Pitch's [-0.1, 3.45] range got a 4.33 rad answer) despite passing those
  same limits in. `_solve_ik` below re-checks every retry's solution
  against the true limits and only accepts one that's actually valid,
  returning None (caller treats as failure) if none of the retries qualify.
- Gripper orientation is NOT solved for independently. All reach targets
  reuse the neutral/stow configuration's own (proven-reachable)
  orientation and only vary position — sidesteps guessing which way
  `Fixed_Jaw_tip`'s local frame points without a live session to check.

UNVERIFIED / needs tuning against a live Isaac Sim session, all flagged
inline: the APPROACH/GRASP/LIFT position offsets (placeholders picked to
land near the FK-sampled reachable point ~(0.3, 0, 0.95) in base_link
frame), the Jaw open/closed values and their open/closed sign convention,
and per-waypoint settle time (no `/joint_states` feedback loop yet — see
module TODO below).

TODO: once `/joint_states` publishing exists on the Isaac side, replace
the fixed `SETTLE_SEC` sleep with an actual convergence poll.

Environment: needs BOTH `rclpy` (system ROS 2 Jazzy) AND
`pytorch-kinematics` (currently only installed in the `lerobot` conda env
used for ManiSkill — see environment.yml). Per this repo's hard rule
("never source ROS 2 in a conda shell"), that means installing
pytorch-kinematics into a plain system-python3 environment alongside
rclpy, e.g.:

    source /opt/ros/jazzy/setup.bash
    python3 -m pip install --user pytorch-kinematics torch --index-url https://download.pytorch.org/whl/cpu
    python3 nodes/scripted_pick.py --robot-id robot_0

CPU-only torch is enough here — this is a handful of tiny IK solves, not
training. `ArmIKSolver` (the class that owns all the pytorch_kinematics
calls) has no rclpy dependency and was verified standalone against the
`lerobot` conda env's installed pytorch-kinematics 0.7.6; `ScriptedPickNode`
(the rclpy.Node) has NOT been integration-tested end-to-end against a live
Isaac session in this environment, for the same "never mix stacks" reason.
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import rclpy
import torch
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

REPO_ROOT = Path(__file__).resolve().parent.parent
URDF_PATH = (
    REPO_ROOT
    / "third_party"
    / "XLeRobot"
    / "simulation"
    / "Maniskill"
    / "assets"
    / "xlerobot"
    / "xlerobot.urdf"
)

# Must match spawn_warehouse.py's ARM_JOINT_NAMES exactly (order + names) —
# that's the contract `_read_arm_cmd` enforces on the Isaac side.
ARM_JOINT_NAMES = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]
IK_JOINT_NAMES = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"]  # no Jaw

# PLACEHOLDER — guessed from a URDF limits midpoint FK sample, not a real
# session. 0 = open is a guess too; URDF limit is [0, 1.7] with no stated
# convention. Flip if a live session shows the gripper starts closed.
JAW_OPEN = 0.0
JAW_CLOSED = 1.2

# PLACEHOLDER offsets (m) in base_link frame from the neutral pose's
# position. Tune once Milestone 1's mock-pick geometry (package height vs.
# base_link height) is confirmed against a live session.
APPROACH_OFFSET = (0.0, 0.0, 0.10)
GRASP_OFFSET = (0.0, 0.0, 0.0)
LIFT_OFFSET = (0.0, 0.0, 0.12)

SETTLE_SEC = 1.5  # fixed dwell per waypoint — see module TODO on /joint_states feedback.
IK_NUM_RETRIES = 32


class ArmIKSolver:
    """Owns the pytorch_kinematics chain + IK solver. No rclpy dependency."""

    def __init__(self, urdf_path: Path):
        import pytorch_kinematics as pk

        with open(urdf_path, "rb") as f:
            urdf_bytes = f.read()
        self.chain = pk.build_serial_chain_from_urdf(
            urdf_bytes, end_link_name="Fixed_Jaw_tip", root_link_name="base_link"
        )
        self.joint_names = self.chain.get_joint_parameter_names()
        assert self.joint_names[0] == "root_z_rotation_joint", (
            f"expected chain[0] == root_z_rotation_joint (pin-to-zero trick "
            f"depends on it), got {self.joint_names[0]!r} — URDF structure changed?"
        )
        assert self.joint_names[1:] == IK_JOINT_NAMES, (
            f"expected {IK_JOINT_NAMES} after the pinned root, got {self.joint_names[1:]}"
        )

        lo, hi = self.chain.get_joint_limits()
        self.true_lo = torch.as_tensor(lo, dtype=torch.float32).clone()
        self.true_hi = torch.as_tensor(hi, dtype=torch.float32).clone()
        # Pin root_z_rotation_joint to 0 so every solve is base_link-relative
        # regardless of the robot's world yaw (see module docstring).
        pinned_lo = self.true_lo.clone()
        pinned_hi = self.true_hi.clone()
        pinned_lo[0] = 0.0
        pinned_hi[0] = 0.0

        self.neutral_q = (
            torch.clamp(pinned_lo, -3.0, 3.0) + torch.clamp(pinned_hi, -3.0, 3.0)
        ) / 2.0
        self.neutral_q[0] = 0.0
        self.neutral_pose = self.chain.forward_kinematics(self.neutral_q)
        self.neutral_pos = self.neutral_pose.get_matrix()[0, :3, 3]

        self.ik_solver = pk.PseudoInverseIK(
            self.chain,
            joint_limits=torch.stack([pinned_lo, pinned_hi], dim=1),
            num_retries=IK_NUM_RETRIES,
        )
        self._pk = pk

    def neutral_arm_positions(self) -> list[float]:
        """5 joint angles (no Jaw) for the stow pose."""
        return [float(v) for v in self.neutral_q[1:]]

    def solve(self, offset_xyz: tuple[float, float, float]) -> list[float] | None:
        """IK for `neutral_pos + offset_xyz`, holding the neutral orientation.

        Returns 5 joint angles (no Jaw), or None if no retry's solution
        actually satisfies the true (unpinned) joint limits — see the
        `PseudoInverseIK.joint_limits` caveat in the module docstring.
        """
        target_pos = self.neutral_pos + torch.tensor(offset_xyz, dtype=torch.float32)
        target = self._pk.Transform3d(
            rot=self.neutral_pose.get_matrix()[0, :3, :3].unsqueeze(0),
            pos=target_pos.unsqueeze(0),
        )
        sol = self.ik_solver.solve(target)
        if not bool(sol.converged_any[0]):
            return None
        for retry_idx in range(sol.solutions.shape[1]):
            if not bool(sol.converged[0, retry_idx]):
                continue
            q = sol.solutions[0, retry_idx]
            # q[0] (pinned root) is excluded from the true-limits check by
            # design — it's allowed to drift during optimization even though
            # we don't use its value (see module docstring).
            arm_q = q[1:]
            if bool(torch.all(arm_q >= self.true_lo[1:]) and torch.all(arm_q <= self.true_hi[1:])):
                return [float(v) for v in arm_q]
        return None


class ScriptedPickNode(Node):
    def __init__(self, robot_id: str, urdf_path: Path):
        super().__init__(f"{robot_id}_scripted_pick")
        self.robot_id = robot_id
        self.get_logger().info(f"loading arm chain from {urdf_path}")
        self.ik = ArmIKSolver(urdf_path)
        self.get_logger().info(f"neutral ee pos (base_link frame): {self.ik.neutral_pos.tolist()}")

        self.arm_cmd_pub = self.create_publisher(JointState, f"/{robot_id}/arm_joint_cmd", 10)
        self.pick_cmd_pub = self.create_publisher(String, f"/{robot_id}/pick_cmd", 10)
        self.result_pub = self.create_publisher(String, f"/{robot_id}/pick_result", 10)
        self.create_subscription(
            String, f"/{robot_id}/pick_request", self._on_pick_request, 10
        )
        self._busy = False

    def _publish_arm(self, arm5: list[float], jaw: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINT_NAMES
        msg.position = list(arm5) + [jaw]
        self.arm_cmd_pub.publish(msg)

    def _publish_pick_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.pick_cmd_pub.publish(msg)

    def _publish_result(self, result: str) -> None:
        msg = String()
        msg.data = result
        self.result_pub.publish(msg)

    def _settle(self, seconds: float = SETTLE_SEC) -> None:
        # Blocking sleep, not a timer callback: this node handles one pick
        # at a time (see `_busy`), and rclpy.spin() on another thread isn't
        # needed for a single-subscriber, single-in-flight-request node.
        time.sleep(seconds)

    def _on_pick_request(self, msg: String) -> None:
        if self._busy:
            self.get_logger().warn(f"pick_request {msg.data!r} dropped — already busy")
            return
        self._busy = True
        try:
            if msg.data == "attach":
                self._run_pick()
            elif msg.data == "detach":
                self._run_place()
            else:
                self.get_logger().error(f"unknown pick_request: {msg.data!r}")
        finally:
            self._busy = False

    def _reach_or_fail(self, offset) -> list[float] | None:
        q = self.ik.solve(offset)
        if q is None:
            self.get_logger().error(f"IK failed for offset {offset}")
        return q

    def _run_pick(self) -> None:
        neutral5 = self.ik.neutral_arm_positions()
        self._publish_arm(neutral5, JAW_OPEN)
        self._settle()

        approach = self._reach_or_fail(APPROACH_OFFSET)
        grasp = self._reach_or_fail(GRASP_OFFSET) if approach is not None else None
        if approach is None or grasp is None:
            self._publish_arm(neutral5, JAW_OPEN)
            self._publish_result("failed")
            return

        self._publish_arm(approach, JAW_OPEN)
        self._settle()
        self._publish_arm(grasp, JAW_OPEN)
        self._settle()
        self._publish_arm(grasp, JAW_CLOSED)
        self._settle()
        self._publish_pick_cmd("attach")  # weld-assist, same as Milestone 1
        self._settle(0.5)

        lift = self._reach_or_fail(LIFT_OFFSET)
        self._publish_arm(lift if lift is not None else grasp, JAW_CLOSED)
        self._settle()
        self._publish_arm(neutral5, JAW_CLOSED)
        self._settle()
        self._publish_result("picked")

    def _run_place(self) -> None:
        neutral5 = self.ik.neutral_arm_positions()
        # Reuses the grasp/approach geometry for placing too — scripted_pick
        # has no notion of "which dropoff station"; task_manager already
        # navigated the base there before sending "detach". See module
        # docstring's placeholder-offsets caveat.
        approach = self._reach_or_fail(APPROACH_OFFSET)
        grasp = self._reach_or_fail(GRASP_OFFSET) if approach is not None else None
        if approach is None or grasp is None:
            self._publish_arm(neutral5, JAW_CLOSED)
            self._publish_result("failed")
            return

        self._publish_arm(approach, JAW_CLOSED)
        self._settle()
        self._publish_arm(grasp, JAW_CLOSED)
        self._settle()
        self._publish_pick_cmd("detach")
        self._publish_arm(grasp, JAW_OPEN)
        self._settle()

        lift = self._reach_or_fail(LIFT_OFFSET)
        self._publish_arm(lift if lift is not None else grasp, JAW_OPEN)
        self._settle()
        self._publish_arm(neutral5, JAW_OPEN)
        self._settle()
        self._publish_result("placed")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument("--urdf", type=Path, default=URDF_PATH)
    args = parser.parse_args()

    rclpy.init()
    node = ScriptedPickNode(args.robot_id, args.urdf)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
