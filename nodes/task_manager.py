#!/usr/bin/env python3
"""Per-robot task dispatch FSM for the SortBots warehouse demo.

Owns one robot's task queue and drives it through (nav-only: a task is a drive
to the pickup waypoint, a short pause, a drive to the dropoff waypoint, another
pause — no arm, nothing physically picked up):

    IDLE -> NAV_TO_PICKUP -> PICKING -> NAV_TO_DROPOFF -> PLACING -> DONE
                 \\-> FAILED (nav abort/reject) <-/

A "goto" task is the single-leg version, used by the operator view's move
queue: IDLE -> NAV_TO_GOAL -> DONE. Any task, queued or running, can be
dropped with `<robot>/cancel_task` (its id, or "all"); a running one ends in
CANCELLED.

PICKING and PLACING are fixed DWELL_SEC pauses standing in for loading and
unloading; the state names are kept so the dashboard's labels and status
stream don't change. Pickup and dropoff are plain named waypoints — the same
working set the operator edits on the map (data/nav_waypoints.json). Merged
2026-10-15 from the old `stations` concept, which carried dock + prop geometry
for a scripted arm pick this track no longer drives.

A PACKAGE task (`{"package": id, "place": {x, y, z, yaw}, ...}`) is the exception:
PICKING and PLACING are then a real exchange with the pick controller
(nodes/magnet_pick.py) instead of a dwell, so the magnet effector actually picks
and places it:

    task_manager --[[/<robot>/pick_request: JSON]]--> magnet_pick
    magnet_pick  --[[/<robot>/pick_result: "picked" | "placed" | "failed"]]--> task_manager

A package task's legs are each a name, a pose dict {name, x, y, yaw}, or null,
and null means "no Nav2 leg": the controller finds the package and docks on its
own (the robot starts wherever it is, which is the point of the pick tests). The
exchange has PICK_TIMEOUT_SEC as a safety net only. The controller runs its own
deadlines on SIM time and answers `failed` itself when it can't do the job, so a
plate out of reach ends the task in seconds, not minutes. A nav-only dispatch
never touches any of this.

Talks to two things:
  - Nav2 (`launch/sortbots_nav2.launch.py`) via the `navigate_to_pose` action.
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

Dispatch a task (Milestone 1 CLI path, no web UI needed):

    ros2 topic pub /robot_0/dispatch_task std_msgs/String \\
        '{data: "{\\"pickup\\": \\"A\\", \\"dropoff\\": \\"B\\"}"}' --once
    ros2 topic pub /robot_0/dispatch_task std_msgs/String \\
        '{data: "{\\"goto\\": \\"A\\"}"}' --once
    ros2 topic pub /robot_0/cancel_task std_msgs/String '{data: all}' --once

Watch it work:

    ros2 topic echo /robot_0/task_status
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
import uuid
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from explorer_handoff import ExplorerHandoff
from waypoints import Waypoint, load_waypoints, nav_waypoints_file

# Named destinations: the working waypoint set the operator edits live on the
# map (data/nav_waypoints.json) — nodes/waypoints.py, shared with fleet_tasks.

# Nav-only tasks (no arm): on reaching the pickup, and again at the dropoff,
# pause this long to stand in for loading/unloading, then drive on. Steady
# clock, NOT sim time — it's a fixed dwell, not a distance deadline, so it must
# not stretch with the real-time factor (CLAUDE.md, on timers vs sim time).
DWELL_SEC = 2.0

# Nav2's action server exists (server_is_ready() is true) from configure, but it
# REJECTS goals until bt_navigator is active, which on a fresh 2-robot session can
# be a minute after the robot is visible ("Action server is inactive. Rejecting the
# goal.", 2026-10-21: the whole first demo phase failed on it). So a task whose
# first leg is rejected within this window goes back to the head of the queue and
# is retried every NAV_RETRY_SEC; after it, a rejection is a real failure.
NAV_STARTUP_GRACE_SEC = 90.0
NAV_RETRY_SEC = 3.0

# Safety net for a package task's pick_request, wall seconds on the steady clock.
# Generous because the arm moves in SIM time (0.3-0.8x real time here) and a pick is
# a scan, a dock and two arm sequences; the controller's own sim-time deadlines are
# what normally end a stuck pick, long before this does.
PICK_TIMEOUT_SEC = 300.0


class Task:
    """One queued job, its waypoints resolved to poses AT DISPATCH.

    Snapshotting matters since the dashboard can rename or move a waypoint
    while a task waits in the queue: looking the name up again when the task
    starts would KeyError inside the timer callback (and take the node down)
    for a rename, or drive somewhere the operator didn't queue for a move.

    Two kinds:
      - "pickup": pickup -> dwell -> dropoff -> dwell (the original task)
      - "goto":   drive to one waypoint and stop. The operator view's move
                  queue sends a route as a run of these, so each stop is its
                  own task the operator can see and cancel.
    """

    def __init__(self, kind: str, stops: list[Waypoint | None], task_id: str | None = None,
                 carrying: bool = False, package: str | None = None,
                 place: dict | None = None, cargo: dict | None = None):
        self.task_id = task_id or uuid.uuid4().hex[:8]
        self.kind = kind
        self.stops = stops
        # The load is already on board (nodes/fleet_tasks.py re-dispatches a
        # fleet task whose dropoff leg failed): skip the pickup leg and its
        # dwell, drive straight to the dropoff.
        self.carrying = carrying and kind == "pickup"
        # A package task: PICKING/PLACING are the magnet exchange, not a dwell, and a
        # leg that is None is not driven (the controller docks on its own).
        self.package = package if kind == "pickup" else None
        self.place = place       # where the package comes to rest: {x, y, z, yaw}
        # What a work order says about the package, so the controller can tell it from
        # the other plates in view: {size: [x, y, z], face: "+x", at: {x, y}} (roughly
        # where it is, the shelf slot; not its pose or which way it faces).
        self.cargo = cargo or {}
        self.first_try: float | None = None   # monotonic; see NAV_STARTUP_GRACE_SEC

    def summary(self) -> dict:
        if self.kind == "goto":
            return {"task_id": self.task_id, "kind": "goto", "goto": self.stops[0].name}
        name = lambda s: s.name if s is not None else "here"  # noqa: E731
        return {"task_id": self.task_id, "kind": "pickup",
                "pickup": name(self.stops[0]), "dropoff": name(self.stops[1]),
                **({"carrying": True} if self.carrying else {}),
                **({"package": self.package} if self.package else {})}

    def first_leg_needs_nav(self) -> bool:
        """Does popping this task need Nav2? Not when it starts with a pick that has no
        leg to drive (the controller docks itself), so a package task runs without it."""
        if self.carrying:
            return self.stops[1] is not None
        return self.stops[0] is not None


class TaskManager(Node):
    def __init__(self, robot_id: str, platform: str):
        super().__init__(f"{robot_id}_task_manager")
        self.robot_id = robot_id
        self.platform = platform
        self.waypoints = load_waypoints(platform)
        self.get_logger().info(
            f"loaded {len(self.waypoints)} waypoints for platform {platform!r} "
            f"from {nav_waypoints_file()}"
        )

        self.queue: deque[Task] = deque()
        self.state = "IDLE"
        self.current_task: Task | None = None
        self._dwell_deadline: float | None = None
        self._hold_until = 0.0   # monotonic; set while Nav2 is still rejecting goals
        self.explorer = ExplorerHandoff()

        self.nav_client = ActionClient(
            self, NavigateToPose, f"/{robot_id}/navigate_to_pose"
        )
        # Latched: nodes/fleet_tasks.py reads the last status on start-up to
        # learn what this node is still doing after a restart of its own (a
        # fleet task with the load on board is adopted, anything else of the
        # fleet's is cancelled so it gets re-auctioned).
        self.status_pub = self.create_publisher(
            String, f"/{robot_id}/task_status",
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.explore_cmd_pub = self.create_publisher(String, f"/{robot_id}/explore_cmd", 10)
        # The pick controller's pair (package tasks only; nodes/magnet_pick.py).
        self.pick_request_pub = self.create_publisher(String, f"/{robot_id}/pick_request", 10)
        self.create_subscription(
            String, f"/{robot_id}/pick_result", self._on_pick_result, 10
        )
        self._pick_deadline: float | None = None   # monotonic; see PICK_TIMEOUT_SEC

        self.create_subscription(
            String, f"/{robot_id}/dispatch_task", self._on_dispatch, 10
        )
        # Plain task_id, or "all" (current task + everything queued).
        self.create_subscription(
            String, f"/{robot_id}/cancel_task", self._on_cancel, 10
        )
        self.create_subscription(
            String, f"/{robot_id}/explore_status", self._on_explore_status, 10
        )

        self._nav_goal_handle = None
        self._nav_result_future = None

        # Picks up the next queued task when IDLE, and expires the PICKING /
        # PLACING dwells. NavigateToPose transitions are event-driven
        # (callbacks), not polled by this timer.
        self.create_timer(0.2, self._tick)

    # -- dispatch -----------------------------------------------------

    def _reject(self, task_id, reason: str) -> None:
        self.get_logger().error(f"rejected task {task_id}: {reason}")
        # Say so on the status stream: without it the dashboard row for a
        # rejected task sits at "waiting" forever.
        self._publish_status(rejected={"task_id": task_id, "reason": reason})

    def _resolve_legs(self, legs: list, task_id) -> list[Waypoint | None] | None:
        """A package task's legs: a name from the working set, a pose dict, or None
        (no Nav2 leg). None back (after rejecting) when one can't be resolved."""
        out: list[Waypoint | None] = []
        for leg in legs:
            if leg is None:
                out.append(None)
            elif isinstance(leg, dict):
                try:
                    out.append(Waypoint({"name": leg.get("name", "pose"), "x": leg["x"],
                                         "y": leg["y"], "yaw": leg.get("yaw", 0.0)}))
                except (KeyError, TypeError, ValueError):
                    self._reject(task_id, f"bad pose leg {leg!r}: want {{x, y, yaw}}")
                    return None
            elif isinstance(leg, str) and leg in self.waypoints:
                out.append(self.waypoints[leg])
            else:
                self._reject(task_id, f"unknown waypoint {leg!r} "
                                      f"(known: {', '.join(sorted(self.waypoints)) or 'none'})")
                return None
        return out

    def _on_dispatch(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            task_id = payload.get("task_id")
            package = payload.get("package")
            if "goto" in payload:
                kind, names = "goto", [payload["goto"]]
            elif package:
                kind, names = "pickup", [payload.get("pickup"), payload.get("dropoff")]
            else:
                kind, names = "pickup", [payload["pickup"], payload["dropoff"]]
        except (json.JSONDecodeError, KeyError, AttributeError) as exc:
            self.get_logger().error(f"bad dispatch payload: {msg.data!r} ({exc})")
            return
        # Re-read the working set so a waypoint placed on the map since this
        # node started still resolves (the old stations file was load-once).
        self.waypoints = load_waypoints(self.platform)
        place = cargo = None
        if package:
            place = payload.get("place")
            at, size, face = payload.get("at"), payload.get("size"), payload.get("face")
            nums = lambda d, keys: isinstance(d, dict) and all(  # noqa: E731
                isinstance(d.get(k), (int, float)) for k in keys)
            # place null is a pick-only task: done once the package is on the magnet
            if not (isinstance(package, str) and "place" in payload
                    and (place is None or nums(place, ("x", "y", "z", "yaw")))
                    and "at" in payload and (at is None or nums(at, ("x", "y"))) and isinstance(face, str)
                    and isinstance(size, list) and len(size) == 3):
                self._reject(task_id, "a package task needs package (id), size [x, y, z], "
                                      "face, at {x, y} or null and place {x, y, z, yaw} or null")
                return
            # at null: the work order doesn't say where it is, the robot finds it
            cargo = {"size": size, "face": face, "at": at and {"x": at["x"], "y": at["y"]}}
            stops = self._resolve_legs(names, task_id)
            if stops is None:
                return
        else:
            missing = [n for n in names if n not in self.waypoints]
            if missing:
                self._reject(task_id, f"unknown waypoint {', '.join(map(repr, missing))} "
                                      f"(known: {', '.join(sorted(self.waypoints)) or 'none'})")
                return
            stops = [self.waypoints[n] for n in names]
        task = Task(kind, stops, task_id=task_id, carrying=bool(payload.get("carrying", False)),
                    package=package or None, place=place, cargo=cargo)
        self.queue.append(task)
        self.get_logger().info(
            f"queued {kind} task {task.task_id}: "
            f"{' -> '.join(s.name if s is not None else 'here' for s in stops)}"
            f"{' (package ' + package + ')' if package else ''} "
            f"(queue depth {len(self.queue)})"
        )
        self._publish_status()
        if self.explorer.on_task_queued():
            self._explore_cmd("stop")
            self.get_logger().info(
                "dispatch received while explorer was active — sent stop so it "
                "releases the nav goal server for this task"
            )

    def _on_cancel(self, msg: String) -> None:
        target = msg.data.strip()
        before = len(self.queue)
        self.queue = deque(t for t in self.queue if target not in ("all", t.task_id))
        dropped = before - len(self.queue)
        if self.current_task and target in ("all", self.current_task.task_id):
            self.get_logger().info(f"cancelling running task {self.current_task.task_id}")
            if self._nav_goal_handle is not None:
                self._nav_goal_handle.cancel_goal_async()
            if self._pick_deadline is not None:        # mid pick/place: stop the arm, magnet off
                self._send_pick_request({"cmd": "cancel"})
            self._end_task("CANCELLED")
        elif dropped:
            self.get_logger().info(f"dropped {dropped} queued task(s) for {target!r}")
            self._publish_status()

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
        # server_is_ready() (non-blocking) gates the pop: Nav2 routinely comes up
        # after the robot is visible, and a task popped before then used to fail
        # outright in _start_nav's wait — which made the first demo phase fail
        # on a fresh session. Holding the queue lets it start the moment Nav2 does.
        # A package task that starts with a pick has no Nav2 leg to wait for (the
        # controller docks on its own), so it runs even when Nav2 is not up at all.
        if (self.state == "IDLE" and self.queue and self.explorer.may_start_task()
                and (self.nav_client.server_is_ready() or not self.queue[0].first_leg_needs_nav())
                and time.monotonic() >= self._hold_until):
            self.current_task = self.queue.popleft()
            if self.current_task.first_try is None:
                self.current_task.first_try = time.monotonic()
            if self.current_task.kind == "goto":
                self._start_nav(self.current_task.stops[0], next_state="NAV_TO_GOAL")
            elif self.current_task.carrying:
                if self.current_task.stops[1] is None:
                    self._begin_place()
                else:
                    self._start_nav(self.current_task.stops[1], next_state="NAV_TO_DROPOFF")
            elif self.current_task.stops[0] is None:
                self._begin_pick()
            else:
                self._start_nav(self.current_task.stops[0], next_state="NAV_TO_PICKUP")
        elif self.state == "IDLE" and self.explorer.on_idle(not self.queue):
            self._explore_cmd("start")
            self.get_logger().info("task queue empty — resumed the exploration a task interrupted")
        elif self._dwell_deadline is not None and time.monotonic() >= self._dwell_deadline:
            self._dwell_deadline = None
            if self.state == "PICKING":
                self._start_nav(self.current_task.stops[1], next_state="NAV_TO_DROPOFF")
            elif self.state == "PLACING":
                self._end_task("DONE")
        elif self._pick_deadline is not None and time.monotonic() >= self._pick_deadline:
            self.get_logger().error(
                f"no pick_result in {PICK_TIMEOUT_SEC:.0f}s ({self.state}) for task "
                f"{self.current_task.task_id if self.current_task else None} — giving up")
            self._send_pick_request({"cmd": "cancel"})   # arm stops, magnet off
            self._end_task("FAILED")

    def _start_nav(self, wp: Waypoint, next_state: str) -> None:
        x, y, yaw = wp.x, wp.y, wp.yaw

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
            self._end_task("FAILED")
            return

        self._set_state(next_state)
        self._nav_goal_handle = None
        task = self.current_task
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: self._on_goal_response(f, task))

    # Both nav callbacks carry the task they were sent for: once a task can be
    # cancelled mid-drive, a late response or result belongs to a task that is
    # no longer current and must not advance (or fail) whatever runs now.

    def _on_goal_response(self, future, task: Task) -> None:
        goal_handle = future.result()
        if task is not self.current_task:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()   # cancelled before Nav2 answered
            return
        if not goal_handle.accepted:
            first_leg = self.state in ("NAV_TO_PICKUP", "NAV_TO_GOAL")
            if first_leg and time.monotonic() - task.first_try < NAV_STARTUP_GRACE_SEC:
                self.get_logger().warn(
                    f"nav goal rejected for task {task.task_id} — Nav2 not active yet? "
                    f"requeued, retrying in {NAV_RETRY_SEC:.0f}s",
                    throttle_duration_sec=15.0,
                )
                self.queue.appendleft(task)
                self.current_task = None
                self._hold_until = time.monotonic() + NAV_RETRY_SEC
                self._set_state("IDLE")
                return
            self.get_logger().error(f"nav goal rejected for task {task.task_id}")
            self._end_task("FAILED")
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_nav_result(f, task))

    def _on_nav_result(self, future, task: Task) -> None:
        if task is not self.current_task:
            return
        self._nav_goal_handle = None
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"nav failed for task {task.task_id}: status={status}"
            )
            self._end_task("FAILED")
            return

        if self.state == "NAV_TO_PICKUP":
            self._begin_pick() if task.package else self._begin_dwell("PICKING")
        elif self.state == "NAV_TO_DROPOFF":
            self._begin_place() if task.package else self._begin_dwell("PLACING")
        elif self.state == "NAV_TO_GOAL":
            self._end_task("DONE")

    # -- the magnet exchange (package tasks) ---------------------------------------

    def _send_pick_request(self, body: dict) -> None:
        msg = String()
        msg.data = json.dumps(body)
        self.pick_request_pub.publish(msg)

    def _begin_pick(self) -> None:
        task = self.current_task
        self._set_state("PICKING")
        self._pick_deadline = time.monotonic() + PICK_TIMEOUT_SEC
        self._send_pick_request({"cmd": "attach", "package": task.package,
                                 "task_id": task.task_id, **task.cargo})

    def _begin_place(self) -> None:
        task = self.current_task
        self._set_state("PLACING")
        self._pick_deadline = time.monotonic() + PICK_TIMEOUT_SEC
        self._send_pick_request({"cmd": "detach", "package": task.package,
                                 "task_id": task.task_id, "place": task.place, **task.cargo})

    def _on_pick_result(self, msg: String) -> None:
        """The controller's answer. Only meaningful while this task is in the matching
        state: a late `picked` from a cancelled task must not advance the next one."""
        result = msg.data.strip()
        task = self.current_task
        if task is None or not task.package or self._pick_deadline is None:
            return
        if result == "failed":
            self.get_logger().error(f"pick controller failed in {self.state} for task {task.task_id}")
            self._end_task("FAILED")
        elif result == "picked" and self.state == "PICKING":
            self._pick_deadline = None
            if task.place is None:
                self._end_task("DONE")
            elif task.stops[1] is None:
                self._begin_place()
            else:
                self._start_nav(task.stops[1], next_state="NAV_TO_DROPOFF")
        elif result == "placed" and self.state == "PLACING":
            self._end_task("DONE")

    def _begin_dwell(self, state: str) -> None:
        # Arrived at the pickup (PICKING) or dropoff (PLACING): pause in place
        # for DWELL_SEC, then _tick drives on. _tick's own cadence follows sim
        # time, so a dwell can be checked a little coarsely under a low RTF —
        # the deadline itself is steady-clock, so its length doesn't stretch.
        self._set_state(state)
        self._dwell_deadline = time.monotonic() + DWELL_SEC

    def _end_task(self, final: str) -> None:
        # DONE / FAILED / CANCELLED go out with the task id, then IDLE without.
        self._set_state(final)
        self.current_task = None
        self._dwell_deadline = None
        self._pick_deadline = None
        self._nav_goal_handle = None
        self._set_state("IDLE")

    def _set_state(self, state: str) -> None:
        self.state = state
        status = self._publish_status()
        self.get_logger().info(f"state -> {state} ({status})")

    def _publish_status(self, **extra) -> dict:
        # `queue` lists what is waiting, in run order, so the dashboard can
        # show (and cancel) queued tasks it didn't dispatch itself.
        status = {
            "task_id": self.current_task.task_id if self.current_task else None,
            "state": self.state,
            "queue_depth": len(self.queue),
            "queue": [t.summary() for t in self.queue],
            **({"task": self.current_task.summary()} if self.current_task else {}),
            **extra,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        return status


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument(
        "--platform", default=os.environ.get("SORTBOTS_PLATFORM", "sim"),
        help="which platform's waypoint set to read from data/nav_waypoints.json "
             "(sim or real); default: $SORTBOTS_PLATFORM, else sim",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TaskManager(args.robot_id, args.platform)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
