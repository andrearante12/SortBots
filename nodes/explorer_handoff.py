"""Who owns the navigate_to_pose server: the explorer or task_manager.

Pure logic, no rclpy, so tests/explorer_handoff_test.py runs without ROS.
task_manager.py feeds it the explorer's `<robot>/explore_status` state and
asks it whether to publish "stop"/"start" on `<robot>/explore_cmd`.

The resume half exists because task_manager used to send "stop" and never
"start": the first task of a run ended exploration for good. 2026-10-09 in
Isaac that deadlocked a fresh map — the pickup dock sat just past the mapped
edge, nav failed, fleet_tasks re-dispatched it after its backoff, and with the
explorer stopped the map could never grow to reach it. Now an exploration that
a task interrupted resumes as soon as the queue is empty, whatever the task's
outcome (DONE, FAILED, CANCELLED).
"""
from __future__ import annotations


class ExplorerHandoff:
    def __init__(self) -> None:
        self.active = False  # explorer last reported "exploring"
        # True only while exploration is stopped because a task took the nav
        # server. An explorer that finished, or that was never running, is
        # never restarted by a task ending.
        self.paused_for_task = False

    def on_status(self, state: str | None) -> None:
        self.active = state == "exploring"
        if self.active:
            # Someone (dashboard, CLI) restarted it already; nothing to resume.
            self.paused_for_task = False

    def on_task_queued(self) -> bool:
        """True = publish "stop" so the task can have the nav server."""
        if not self.active:
            return False
        self.paused_for_task = True
        return True

    def may_start_task(self) -> bool:
        # Wait for the explorer to confirm it stopped: until then its replan
        # timer could still send a frontier goal that preempts ours.
        return not self.active

    def on_idle(self, queue_empty: bool) -> bool:
        """True = publish "start": the last queued task is over."""
        if not (queue_empty and self.paused_for_task):
            return False
        self.paused_for_task = False
        return True
