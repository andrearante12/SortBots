#!/usr/bin/env python3
"""Per-robot task dispatch FSM for the SortBots warehouse demo.

Owns one robot's task queue and drives it through:

    IDLE -> NAV_TO_PICKUP -> PICKING -> NAV_TO_DROPOFF -> PLACING -> DONE
                 \\-> FAILED (nav abort/reject) <-/

Talks to three things:
  - Nav2 (`launch/sortbots_nav2.launch.py`) via the `navigate_to_pose` action.
  - `nodes/scripted_pick.py` via plain String topics (not a ROS 2 service —
    matches the subscribe-and-poll style the Isaac side already uses for
    cmd_vel/pick_cmd): this node publishes `<robot>/pick_request`
    ("attach"/"detach") on entering PICKING/PLACING, and scripted_pick.py
    reports back on `<robot>/pick_result` ("picked"/"placed"/"failed")
    once its arm motion (and Isaac's mock-weld) finishes. A fixed timeout
    (`PICK_TIMEOUT_SEC`) is a safety net only, in case scripted_pick.py
    isn't running — the real completion signal is the result message.
  - `nodes/explorer.py`, if it's running (optional; nothing below breaks if
    it isn't). Both nodes drive the same single-goal-at-a-time
    navigate_to_pose action server, so they arbitrate: this node subscribes
    `<robot>/explore_status` and refuses to pop the task queue while the
    explorer reports state "exploring", and it publishes "stop" on
    `<robot>/explore_cmd` the moment a task is dispatched during that state
    (see _on_explore_status / _on_dispatch). Nav2 itself would silently let
    a new goal preempt the old one either way — this gate exists so the
    explorer's own replan timer doesn't immediately fight back with another
    frontier goal. Once the queue drains it sends "start" again, but only if
    a task was what stopped it (nodes/explorer_handoff.py has the why).

No custom .msg/.srv package: the repo has no ament package scaffolding yet
(everything runs as loose scripts — see docs/quickstart.md), so task dispatch and
status both use std_msgs/String carrying JSON. Good enough for one robot;
would become a real message package if/when multi-robot dispatch lands.

Run (system ROS 2 Jazzy sourced, NOT the Isaac venv or conda):

    source /opt/ros/jazzy/setup.bash
    python3 nodes/task_manager.py --robot-id robot_0

Tasks arrive on `<robot>/dispatch_task`, either straight from a person (CLI
below, dashboard) or from nodes/fleet_tasks.py after this robot wins a fleet
auction. `<robot>/cancel_task` (a task_id) withdraws one: fleet_tasks sends it
when the robot loses ownership (lease lost while isolated, out-bid on a
partition heal, cancelled by the gateway) so it stops working a task some
other robot now owns.

Dispatch a task (Milestone 1 CLI path, no web UI needed):

    ros2 topic pub /robot_0/dispatch_task std_msgs/String \\
        '{data: "{\\"pickup\\": \\"shelf_a\\", \\"dropoff\\": \\"zone_b\\"}"}' --once

Watch it work:

    ros2 topic echo /robot_0/task_status
"""
from __future__ import annotations

import argparse
import json
import math
import time
import uuid
from collections import deque
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from explorer_handoff import ExplorerHandoff
from stations import Station, load_stations  # noqa: F401  (Station re-exported)

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_WAYPOINTS = REPO_ROOT / "configs" / "waypoints.yaml"

# Safety-net timeout for PICKING/PLACING: scripted_pick.py's own motion
# sequence takes several seconds (multiple SETTLE_SEC waypoints — see its
# module docstring), so this must be comfortably longer than that, not a
# tight bound. Only fires if scripted_pick.py never responds at all.
PICK_TIMEOUT_SEC = 30.0


class Task:
    def __init__(self, pickup: str, dropoff: str, task_id: str | None = None,
                 carrying: bool = False):
        self.task_id = task_id or uuid.uuid4().hex[:8]
        self.pickup = pickup
        self.dropoff = dropoff
        # The item is already on the robot (a retry after a failed dropoff
        # leg, fleet_tasks.py): skip the pickup, drive straight to dropoff.
        self.carrying = carrying


class TaskManager(Node):
    def __init__(self, robot_id: str, waypoints_path: Path):
        super().__init__(f"{robot_id}_task_manager")
        self.robot_id = robot_id
        self.stations = load_stations(waypoints_path)
        self.get_logger().info(f"loaded {len(self.stations)} stations from {waypoints_path}")

        self.queue: deque[Task] = deque()
        self.state = "IDLE"
        self.current_task: Task | None = None
        self._pick_timeout_deadline: float | None = None
        self.explorer = ExplorerHandoff()

        self.nav_client = ActionClient(
            self, NavigateToPose, f"/{robot_id}/navigate_to_pose"
        )
        self.pick_request_pub = self.create_publisher(
            String, f"/{robot_id}/pick_request", 10
        )
        # Latched: fleet_tasks.py reads the last status on start-up to learn
        # what this node is still doing after a restart of its own (a task
        # with the item on board is adopted, anything else is cancelled).
        self.status_pub = self.create_publisher(
            String, f"/{robot_id}/task_status",
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.explore_cmd_pub = self.create_publisher(String, f"/{robot_id}/explore_cmd", 10)

        self.create_subscription(
            String, f"/{robot_id}/dispatch_task", self._on_dispatch, 10
        )
        self.create_subscription(
            String, f"/{robot_id}/cancel_task", self._on_cancel, 10
        )
        self.create_subscription(
            String, f"/{robot_id}/pick_result", self._on_pick_result, 10
        )
        self.create_subscription(
            String, f"/{robot_id}/explore_status", self._on_explore_status, 10
        )

        self._nav_goal_handle = None
        self._nav_result_future = None

        # Picks up the next queued task when IDLE, and enforces
        # PICK_TIMEOUT_SEC. NavigateToPose/pick_result transitions are
        # event-driven (callbacks), not polled by this timer.
        self.create_timer(0.2, self._tick)

    # -- dispatch -----------------------------------------------------

    def _on_dispatch(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            task = Task(
                pickup=payload["pickup"],
                dropoff=payload["dropoff"],
                task_id=payload.get("task_id"),
                carrying=bool(payload.get("carrying", False)),
            )
        except (json.JSONDecodeError, KeyError) as exc:
            self.get_logger().error(f"bad dispatch payload: {msg.data!r} ({exc})")
            return
        if task.pickup not in self.stations or task.dropoff not in self.stations:
            self.get_logger().error(
                f"unknown station in task {task.task_id}: "
                f"pickup={task.pickup!r} dropoff={task.dropoff!r}"
            )
            return
        self.queue.append(task)
        self.get_logger().info(
            f"queued task {task.task_id}: {task.pickup} -> {task.dropoff} "
            f"(queue depth {len(self.queue)})"
        )
        if self.explorer.on_task_queued():
            self._explore_cmd("stop")
            self.get_logger().info(
                "dispatch received while explorer was active — sent stop so it "
                "releases the nav goal server for this task"
            )

    def _on_cancel(self, msg: String) -> None:
        task_id = msg.data.strip()
        before = len(self.queue)
        self.queue = deque(t for t in self.queue if t.task_id != task_id)
        if len(self.queue) != before:
            self.get_logger().info(f"cancelled queued task {task_id}")
            return
        if self.current_task is None or self.current_task.task_id != task_id:
            return
        self.get_logger().warn(f"cancelling active task {task_id} in {self.state}")
        if self.state in ("NAV_TO_DROPOFF", "PLACING"):
            # Only a gateway cancel gets here (fleet_tasks never yields a
            # task once the item is picked): the item stays on the robot.
            self.get_logger().error(
                f"task {task_id} cancelled with its item still on the robot — "
                "unload it by hand before the next pick")
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
        # The cancelled goal still reports a result later; current_task is
        # None by then, so _on_nav_result ignores it instead of failing
        # whatever task is active at that point.
        self._nav_goal_handle = None
        self._pick_timeout_deadline = None
        self._set_state("CANCELLED")
        self.current_task = None
        self._set_state("IDLE")

    def _on_explore_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.explorer.on_status(status.get("state"))

    def _explore_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.explore_cmd_pub.publish(msg)

    # -- FSM ------------------------------------------------------------

    def _tick(self) -> None:
        if self.state == "IDLE" and self.queue and self.explorer.may_start_task():
            self.current_task = self.queue.popleft()
            if self.current_task.carrying:
                self._start_nav(self.current_task.dropoff, next_state="NAV_TO_DROPOFF")
            else:
                self._start_nav(self.current_task.pickup, next_state="NAV_TO_PICKUP")
        elif self.state == "IDLE" and self.explorer.on_idle(not self.queue):
            self._explore_cmd("start")
            self.get_logger().info("task queue empty — resumed the exploration a task interrupted")
        elif self.state in ("PICKING", "PLACING") and self._pick_timeout_deadline is not None:
            if time.monotonic() >= self._pick_timeout_deadline:
                self.get_logger().error(
                    f"pick_result timed out in {self.state} for task "
                    f"{self.current_task.task_id} — is scripted_pick.py running?"
                )
                self._fail_task()

    def _start_nav(self, station_name: str, next_state: str) -> None:
        station = self.stations[station_name]
        x, y, yaw = station.dock_pose()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # Yaw-only rotation (planar base) -> quaternion, no tf_transformations dep.
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("navigate_to_pose server not available")
            self._fail_task()
            return

        self._set_state(next_state)
        send_future = self.nav_client.send_goal_async(goal)
        task = self.current_task
        send_future.add_done_callback(lambda f: self._on_goal_response(f, task))

    def _on_goal_response(self, future, task: Task) -> None:
        goal_handle = future.result()
        # Bound to the task the goal was sent for: after a cancel, a NEW task
        # may be active by the time this stale response lands.
        if task is not self.current_task:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().error(f"nav goal rejected for task {self.current_task.task_id}")
            self._fail_task()
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_nav_result(f, task))

    def _on_nav_result(self, future, task: Task) -> None:
        if task is not self.current_task:  # result of a goal we cancelled
            return
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"nav failed for task {self.current_task.task_id}: status={status}"
            )
            self._fail_task()
            return

        if self.state == "NAV_TO_PICKUP":
            self._send_pick_request("attach")
            self._set_state("PICKING")
        elif self.state == "NAV_TO_DROPOFF":
            self._send_pick_request("detach")
            self._set_state("PLACING")

    def _send_pick_request(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.pick_request_pub.publish(msg)
        self._pick_timeout_deadline = time.monotonic() + PICK_TIMEOUT_SEC

    def _on_pick_result(self, msg: String) -> None:
        if self.state == "PICKING" and msg.data == "picked":
            self._pick_timeout_deadline = None
            self._start_nav(self.current_task.dropoff, next_state="NAV_TO_DROPOFF")
        elif self.state == "PLACING" and msg.data == "placed":
            self._pick_timeout_deadline = None
            self._set_state("DONE")
            self.current_task = None
            self._set_state("IDLE")
        elif self.state in ("PICKING", "PLACING") and msg.data == "failed":
            self.get_logger().error(
                f"scripted_pick reported failure in {self.state} for task "
                f"{self.current_task.task_id}"
            )
            self._fail_task()
        # else: stray/late result (e.g. after a timeout already failed the
        # task) — ignore rather than corrupt whatever task is now active.

    def _fail_task(self) -> None:
        self._set_state("FAILED")
        self.current_task = None
        self._pick_timeout_deadline = None
        self._set_state("IDLE")

    def _set_state(self, state: str) -> None:
        self.state = state
        cur = self.current_task
        status = {
            "task_id": cur.task_id if cur else None,
            "state": state,
            "pickup": cur.pickup if cur else None,
            "dropoff": cur.dropoff if cur else None,
            "queue_depth": len(self.queue),
            "queue": [t.task_id for t in self.queue],
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        self.get_logger().info(f"state -> {state} ({status})")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument("--waypoints", type=Path, default=DEFAULT_WAYPOINTS)
    args = parser.parse_args()

    rclpy.init()
    node = TaskManager(args.robot_id, args.waypoints)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
