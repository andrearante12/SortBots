"""tasks — pickup/dropoff deliveries through nodes/task_manager.py."""
from __future__ import annotations

import statistics

from . import Metric, MetricError, register

TERMINAL = {"DONE", "FAILED"}


@register
class Tasks(Metric):
    name = "tasks"
    version = 1
    streams = {"dispatch", "task"}
    summary = "Delivery outcomes and durations (task_manager FSM)."
    params = {}
    values = {
        "dispatched": "tasks the probe dispatched",
        "done": "tasks that reached DONE",
        "failed": "tasks that reached FAILED",
        "unfinished": "tasks not terminal when the run stopped",
        "success_rate": "done / tasks in the test",
        "mean_time_s": "mean sim seconds from dispatch to DONE",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.planned = len(ctx.tasks)
        self.tasks: dict[str, dict] = {}

    def check_context(self):
        if not self.ctx.tasks:
            raise MetricError("tasks: the test dispatches no tasks")

    def on_event(self, ev):
        if ev["s"] == "dispatch":
            self.tasks[ev["task_id"]] = {"rid": ev["rid"], "sent_at": ev["t"],
                                         "state": "SENT", "ended_at": None}
            return
        rec = self.tasks.get(ev.get("task_id"))
        if rec is None or rec["rid"] != ev["rid"]:
            return  # someone else's task (dashboard dispatch during the run)
        rec["state"] = ev.get("state")
        if rec["state"] in TERMINAL and rec["ended_at"] is None:
            rec["ended_at"] = ev["t"]

    def result(self):
        done = [t for t in self.tasks.values() if t["state"] == "DONE"]
        failed = sum(t["state"] == "FAILED" for t in self.tasks.values())
        total = max(self.planned, len(self.tasks))
        times = [t["ended_at"] - t["sent_at"] for t in done]
        return {
            "dispatched": len(self.tasks),
            "done": len(done),
            "failed": failed,
            "unfinished": total - len(done) - failed,
            "success_rate": round(len(done) / total, 3) if total else None,
            "mean_time_s": round(statistics.fmean(times), 2) if times else None,
        }
