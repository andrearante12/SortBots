"""dispatch.method: auction — the real fleet auction, as on the robots.

Every robot runs FleetTasks (nodes/fleet_tasks.py, imported, not copied)
and the gateway runs one more that announces and keeps the board. This is
exactly the code path fleetsim had before the dispatch layer: the same
objects, built in the same order with the same settings, so a run stays
bit-identical (docs/fleetsim.md, "Dispatchers and perfect information").

With robot types (`robots.types`), each robot's FleetTasks gets its own
type's speed, capabilities and service times (so it bids the whole delivery
at its own pace and skips what it can't do), and its own battery drain.
"""
from __future__ import annotations

import dataclasses
import json
import sys
from pathlib import Path

from . import Method
from ..robot import PathOracle

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "nodes"))
import fleet_tasks as ft  # noqa: E402


class AuctionAgent(ft.FleetTasks):
    """FleetTasks as a RobotAgent. It already is one, except that it keeps
    what Status carries and the session liveness on its allocator."""

    def owned(self) -> list:
        return self.alloc.owned()

    def heard_network(self, now: float, peer: int) -> None:
        self.alloc.heard_network(now, peer=peer)


class Auction(Method):
    name = "auction"

    def __init__(self, sim):
        self.sim = sim
        gw = ft.FleetTasks(None, sim.params, sim.waypoints, gateway=True, cfg=sim.ft_cfg, now=0.0)
        # Task IDs are seeded from the wall clock on the robots; a fixed
        # clock hands out the same IDs every run, so a seed reproduces one.
        gw._wall = lambda: 0.0
        gw._next_id = 0
        self.gateway = gw

    def robot_agent(self, robot):
        sim = self.sim
        prof = robot.profile
        cfg, params = sim.ft_cfg, sim.params
        if sim.typed:
            cfg = {**sim.ft_cfg, "speed_mps": prof["max_speed_mps"],
                   "capabilities": list(prof["capabilities"]),
                   "service_s": {"pick": prof["pick_s"], "place": prof["place_s"]}}
        if prof["battery_drain_pct_per_s"] is not None:
            # The allocator's battery gate and FleetTasks.drive both read it here.
            params = dataclasses.replace(params,
                                         battery_drain_pct_per_s=prof["battery_drain_pct_per_s"])
        core = AuctionAgent(robot.rid, params, sim.waypoints, cfg=cfg, now=sim.now)
        core._wall = lambda: 0.0
        core._next_id = 0
        core.grid = PathOracle(robot, float(sim.params.bid_radius_m),
                               sim.cfg["tasks"]["bid_cost"] == "path")
        core.set_pose((robot.x, robot.y), sim.now)
        bat = sim.ft_cfg["battery"].get("sim_start_pct")
        if bat is not None:
            core.battery_pct = float(bat)
        # task_manager's latched task_status reaches a starting fleet_tasks
        # before any dispatch can; FleetTasks reads that first status for
        # restart adoption, and would otherwise "adopt" our first real task
        # by cancelling it.
        core.on_task_status(json.dumps({"task_id": None, "state": "IDLE",
                                        "queue_depth": 0, "queue": []}), sim.now)
        core.drain()
        return core

    def gateway_status(self, k, now, battery_pct, mode, xy) -> None:
        self.gateway.on_peer_status(f"robot_{k}", now, battery_pct=battery_pct, mode=mode)

    def task_done(self, task_id: str) -> bool:
        # A task the allocator has already pruned finished long ago.
        t = self.gateway.alloc.tasks.get(int(task_id[len(ft.TASK_PREFIX):]))
        return t is None or t.state == "done"
