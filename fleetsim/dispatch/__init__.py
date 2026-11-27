"""Task assignment, swappable: who decides which robot does which task.

`dispatch.method` picks one (fleetsim/configs/base.yaml):

  auction            every robot runs the real fleet auction (FleetTasks from
                     nodes/fleet_tasks.py) and the gateway only announces and
                     keeps the board. The robots as they are; the default, and
                     bit-identical to fleetsim before this layer existed.
  central_greedy     one dispatcher at the gateway (network node 0): the oldest
                     open task goes to the idle robot that reaches its pickup
                     soonest, repeated while both remain (central.py).
  central_hungarian  the same dispatcher re-solving the whole assignment with
                     Kuhn-Munkres on every event and every resolve_period_s,
                     with hysteresis (central.py, hungarian.py).

The seam
--------
Everything task-related goes through two small interfaces, so a method is
one module and the rest of fleetsim doesn't know which one runs.

A robot's task side, `RobotAgent` (one per robot, `Method.robot_agent`).
fleetsim/robot.py and sim.py call exactly this and nothing else:

    battery_pct                      read by Status; None = no battery model
    set_pose(xy, now)                every step
    tick(now)                        every step
    on_task_status(raw, now)         the task FSM's status JSON, on every change
                                     (task_manager's /<robot>/task_status shape)
    on_message(sender, m, now)       a /fleet/tasks message (already decoded)
    on_peer_status(rid, now, battery_pct=, mode=)   a peer's /fleet/status
    heard_network(now, peer)         a live session to `peer` (status.session_liveness)
    owned() -> [(task_id, cost)]     what this robot's /fleet/status carries
    drive(dt, moving)                battery drain
    drain() -> [actions]             what it wants done, after every call above

Actions are FleetTasks' tuples: ("publish", raw) puts a /fleet/tasks
message on the network (a "to" field addresses it, else fleet-wide),
("dispatch", payload) / ("cancel", task_id) / ("pick", task_id) drive the
robot's task FSM, ("log", text) is recorded with record.fleet_log.
FleetTasks provides all of it already (auction.AuctionAgent only adds
`owned` and `heard_network`, which it keeps on its allocator); a central
method gives robots a thin client instead (central.CentralClient).

What runs at the gateway, `Method.gateway` (Sim.gw): request(pickup,
dropoff, now) -> task id, on_message, tick, drain and board(), FleetTasks'
gateway API. A robot's Status reaching the gateway goes to
`Method.gateway_status` (a central planner needs the pose, the auction only
the liveness), and the run's stop condition asks `Method.task_done`.

Later decentralised methods (sequential single-item auctions, CBBA, the
distributed Hungarian method; #50) are per-robot agents exchanging
messages: a RobotAgent whose on_message/tick implement the protocol and
whose drain publishes, with a gateway that only announces. They plug in
here without touching robot.py or sim.py.
"""
from __future__ import annotations

from ..config import DISPATCH_METHODS as METHODS  # noqa: E402,F401 (config.py validates it)


class Method:
    """One task-assignment method for a whole run."""

    name = "?"
    gateway = None

    def robot_agent(self, robot):
        raise NotImplementedError

    def gateway_status(self, k: int, now: float, battery_pct, mode: str, xy) -> None:
        """robot_k's /fleet/status reached the gateway (polled at most every
        status.liveness_poll_s per robot, as FleetTasks hears it)."""
        raise NotImplementedError

    def task_done(self, task_id: str) -> bool:
        """The gateway counts this announced task (fleet-<n>) as finished."""
        raise NotImplementedError

    def summary(self) -> dict | None:
        """Extra run-summary numbers (None: nothing to add)."""
        return None


def build(sim) -> Method:
    method = sim.cfg["dispatch"]["method"]
    if method == "auction":
        from .auction import Auction
        return Auction(sim)
    if method in ("central_greedy", "central_hungarian"):
        from .central import Central
        return Central(sim, method)
    raise ValueError(f"dispatch.method: expected one of {METHODS}, got {method!r}")
