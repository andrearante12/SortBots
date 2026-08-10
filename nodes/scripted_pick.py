#!/usr/bin/env python3
"""Scripted pick/place for the SortBots warehouse demo.

Plays back recorded joint-space poses from `configs/arm_poses.yaml`:

    STOW -> APPROACH -> DESCEND -> CLOSE -> (weld) -> LIFT -> CARRY

There is no IK on this path. The poses were captured by jogging the arm with
the dashboard's arm pad (`webui/arm.js`) and pasting the emitted YAML into the
config — so the numbers are measured, not solved, and the node needs neither
torch nor pytorch-kinematics. That is what makes it launchable: the previous
version carried a module-scope `import torch`, so no launch file could start it
and every dispatched task timed out in PICKING with nothing listening. IK still
exists in `nodes/arm_ik.py` behind `--mode ik` for anything that wants it.

Protocol — unchanged, and `nodes/task_manager.py` depends on every line of it:

    task_manager.py --[[/<robot>/pick_request: "attach"|"detach"]]--> this node
    this node       --[[/<robot>/arm_joint_cmd: JointState]]-------> Isaac (spawn_warehouse.py)
    this node       --[[/<robot>/pick_cmd: "attach"|"detach"]]-----> Isaac (mock weld)
    this node       --[[/<robot>/pick_result: "picked"|"placed"|"failed"]]--> task_manager.py

`arm_joint_cmd` must always carry ALL SIX of `ARM_JOINT_NAMES` in order:
`spawn_warehouse._read_arm_cmd` returns None and silently drops the command if
any name is missing. The result string must reach task_manager within its
`PICK_TIMEOUT_SEC` (30 s) — `main()` logs each sequence's budget at startup and
warns if a capture has grown past it.

Still no real grasp physics: Isaac's `_sync_package_to_gripper` teleports the
package onto the gripper tip while attached. What the arm does here is reach
for it convincingly first.

The dashboard's arm pad publishes the same topic, unarbitrated — last writer
wins. Don't jog while a task is picking.

Environment: system python3 + ROS 2 Jazzy. Nothing else.

    source /opt/ros/jazzy/setup.bash
    python3 nodes/scripted_pick.py --robot-id robot_0

`--play pick` runs one sequence and exits, which is how you iterate on
`arm_poses.yaml` against a live sim without Nav2, RTAB-Map or the task FSM.

TODO: once `/joint_states` publishing exists on the Isaac side, replace the
fixed per-step `settle_sec` dwell with an actual convergence poll.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))

from arm_poses import (  # noqa: E402
    ARM_JOINT_NAMES,
    ArmPoseError,
    PoseBook,
    Step,
    interpolate,
    load_pose_book,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_POSES = REPO_ROOT / "configs" / "arm_poses.yaml"

# Rate the interpolated ramp is published at. Fast enough that the position
# drives track it smoothly, slow enough not to flood rosbridge.
ARM_STREAM_HZ = 20.0

# task_manager.PICK_TIMEOUT_SEC is 30 s. Warn well below it so a long capture
# is caught at startup rather than as a mysterious task failure.
SEQUENCE_BUDGET_WARN_SEC = 25.0

# The pose every sequence starts from, and what the sim boots into:
# configs/physics_overrides/xlerobot.json sets target_value 0.0 for all arm
# joints, so this is the true initial state, not an assumption.
HOME_Q = [0.0] * len(ARM_JOINT_NAMES)


class ScriptedPickNode(Node):
    def __init__(self, robot_id: str, book: PoseBook, ik=None):
        super().__init__(f"{robot_id}_scripted_pick")
        self.robot_id = robot_id
        self.book = book
        self.ik = ik
        self._last_q = list(HOME_Q)

        self.arm_cmd_pub = self.create_publisher(JointState, f"/{robot_id}/arm_joint_cmd", 10)
        self.pick_cmd_pub = self.create_publisher(String, f"/{robot_id}/pick_cmd", 10)
        self.result_pub = self.create_publisher(String, f"/{robot_id}/pick_result", 10)
        self.create_subscription(
            String, f"/{robot_id}/pick_request", self._on_pick_request, 10
        )
        self._busy = False

    # -- publishing --------------------------------------------------------

    def _publish_arm(self, q: list[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = [float(v) for v in q]
        self.arm_cmd_pub.publish(msg)
        self._last_q = list(q)

    def _publish_pick_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.pick_cmd_pub.publish(msg)

    def _publish_result(self, result: str) -> None:
        msg = String()
        msg.data = result
        self.result_pub.publish(msg)

    # -- playback ----------------------------------------------------------

    def _run_step(self, step: Step) -> None:
        """Ramp to `step.q`, then grip, then dwell.

        Ramping rather than writing the target in one go: the arm joints are
        position drives at stiffness 1000 / max_force 1000, so a step change
        commands a near-instant slew. See `arm_poses.interpolate`.
        """
        if step.q is not None:
            period = 1.0 / ARM_STREAM_HZ
            for q in interpolate(self._last_q, step.q, step.move_sec, ARM_STREAM_HZ):
                self._publish_arm(q)
                time.sleep(period)
        if step.grip is not None:
            self._publish_pick_cmd(step.grip)
        if step.settle_sec > 0:
            # Blocking sleep, not a timer callback: this node handles one
            # request at a time (see `_busy`), so there is nothing else for
            # the executor to do while the arm moves.
            time.sleep(step.settle_sec)

    def _play(self, sequence_name: str) -> bool:
        try:
            steps = self.book.sequence(sequence_name)
        except ArmPoseError as exc:
            self.get_logger().error(str(exc))
            return False
        started = time.monotonic()
        for i, step in enumerate(steps):
            self.get_logger().info(
                f"{sequence_name}[{i}] {step.name}"
                + (f" grip={step.grip}" if step.grip else "")
            )
            self._run_step(step)
        self.get_logger().info(
            f"{sequence_name} done in {time.monotonic() - started:.1f}s"
        )
        return True

    def _on_pick_request(self, msg: String) -> None:
        if self._busy:
            self.get_logger().warn(f"pick_request {msg.data!r} dropped — already busy")
            return
        self._busy = True
        try:
            if msg.data == "attach":
                self._publish_result("picked" if self._play("pick") else "failed")
            elif msg.data == "detach":
                self._publish_result("placed" if self._play("place") else "failed")
            else:
                self.get_logger().error(f"unknown pick_request: {msg.data!r}")
        finally:
            self._busy = False


def _log_budgets(node: ScriptedPickNode) -> None:
    for name in sorted(node.book.sequences):
        steps = node.book.sequence(name)
        total = node.book.duration(name)
        node.get_logger().info(f"sequence {name!r}: {len(steps)} steps, {total:.1f}s")
        if total > SEQUENCE_BUDGET_WARN_SEC:
            node.get_logger().warn(
                f"sequence {name!r} takes {total:.1f}s — task_manager gives up at "
                f"PICK_TIMEOUT_SEC (30s). Trim steps or shorten move_sec/settle_sec."
            )


def main() -> int:
    parser = argparse.ArgumentParser(description="Scripted pick/place via joint-space playback.")
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument("--poses", type=Path, default=DEFAULT_POSES,
                        help="pose book to play back (default: configs/arm_poses.yaml)")
    parser.add_argument("--mode", choices=("playback", "ik"), default="playback",
                        help="playback replays --poses; ik loads nodes/arm_ik.py "
                             "(needs torch + pytorch-kinematics, see that module)")
    parser.add_argument("--urdf", type=Path, default=None,
                        help="URDF for --mode ik; ignored otherwise")
    parser.add_argument("--play", choices=("pick", "place"), default=None,
                        help="run one sequence immediately and exit — iterate on "
                             "arm_poses.yaml against a live sim without Nav2 or the task FSM")
    args = parser.parse_args()

    try:
        book = load_pose_book(args.poses)
    except ArmPoseError as exc:
        print(f"scripted_pick: {exc}", file=sys.stderr)
        return 2

    ik = None
    if args.mode == "ik":
        # Imported only here: pulling arm_ik in unconditionally would drag
        # torch back into the launch path and re-break the node.
        from arm_ik import URDF_PATH, ArmIKSolver

        ik = ArmIKSolver(args.urdf or URDF_PATH)

    rclpy.init()
    node = ScriptedPickNode(args.robot_id, book, ik=ik)
    _log_budgets(node)
    try:
        if args.play:
            node.get_logger().info(f"--play {args.play}: running once, then exiting")
            ok = node._play(args.play)
            return 0 if ok else 1
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is how rclpy reports the SIGTERM that
        # `run_demo.sh stop` sends; it's a normal exit here, not a fault.
        pass
    finally:
        node.destroy_node()
        # rclpy's own signal handler may already have torn the context down
        # (SIGTERM from `run_demo.sh stop` does exactly that); calling shutdown
        # a second time raises RCLError and turns a clean exit into a traceback.
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
