"""Offline tests for nodes/explorer_handoff.py: when task_manager stops and
resumes the explorer. No ROS — the system python3 is enough."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))

from explorer_handoff import ExplorerHandoff  # noqa: E402


def exploring():
    h = ExplorerHandoff()
    h.on_status("exploring")
    return h


def test_task_stops_explorer_and_waits_for_it_to_confirm():
    h = exploring()
    assert h.on_task_queued()
    assert not h.may_start_task()  # explorer hasn't reported "stopped" yet
    h.on_status("stopped")
    assert h.may_start_task()


def test_exploration_resumes_once_the_queue_is_empty():
    h = exploring()
    h.on_task_queued()
    h.on_status("stopped")
    assert not h.on_idle(queue_empty=False)  # more tasks still queued
    assert h.on_idle(queue_empty=True)
    assert not h.on_idle(queue_empty=True)  # "start" goes out once


def test_failed_task_retry_cycle_lets_the_map_grow_in_between():
    # 2026-10-09: nav fails -> backoff -> re-dispatch. The explorer must run
    # during every backoff, not stay stopped after the first dispatch.
    h = exploring()
    for _ in range(3):
        assert h.on_task_queued()
        h.on_status("stopped")
        assert h.on_idle(queue_empty=True)  # task FAILED, queue drained
        h.on_status("exploring")


def test_explorer_that_was_not_running_is_never_started():
    for state in (None, "idle", "done", "stopped"):
        h = ExplorerHandoff()
        h.on_status(state)
        assert not h.on_task_queued()
        assert h.may_start_task()
        assert not h.on_idle(queue_empty=True)


def test_manual_restart_during_a_task_is_not_doubled():
    h = exploring()
    h.on_task_queued()
    h.on_status("stopped")
    h.on_status("exploring")  # operator pressed start on the dashboard
    h.on_status("stopped")    # ...and stop again
    assert not h.on_idle(queue_empty=True)
