"""Offline tests for nodes/fleet_tasks.py: the auction over the real JSON wire
format, with a fake task_manager. No ROS — the system python3 is enough.

Each robot's pose and battery are set directly; the bus delivers every
publish to every node (including the sender, as DDS does).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))

import fleet_radio as fr  # noqa: E402
import fleet_tasks as ft  # noqa: E402
from commsim.core.protocol import Params  # noqa: E402
from stations import load_stations  # noqa: E402

STATIONS = load_stations(REPO / "configs" / "waypoints.yaml")
CFG = ft.load_config()


class Bus:
    def __init__(self, robots: dict, no_bid=(), status_hz=5.0, **cfg):
        cfg = {**CFG, **cfg}
        self.params = Params.from_config(cfg)
        self.t = 0.0
        self.gw = ft.FleetTasks(None, self.params, STATIONS, gateway=True, cfg=cfg)
        self.nodes = {"gateway": self.gw}
        for rid, pose in robots.items():
            n = ft.FleetTasks(rid, self.params, STATIONS, cfg=cfg, can_bid=rid not in no_bid)
            n.pose = pose
            self.nodes[rid] = n
        self.status_dt = 1.0 / status_hz
        self._next_status = 0.0
        self.dispatched: dict[str, list] = {rid: [] for rid in robots}
        self.cancelled: dict[str, list] = {rid: [] for rid in robots}
        self.log: list = []
        self.down: set = set()

    def add(self, rid, pose):
        n = ft.FleetTasks(rid, self.params, STATIONS, cfg=CFG, now=self.t)
        n.pose = pose
        self.nodes[rid] = n
        self.dispatched[rid], self.cancelled[rid] = [], []
        return n

    def _pump(self):
        moved = True
        while moved:
            moved = False
            for name, n in list(self.nodes.items()):
                for kind, val in n.drain():
                    moved = True
                    if kind == "publish":
                        for other_name, other in self.nodes.items():
                            if name in self.down or other_name in self.down:
                                continue
                            other.on_wire(val, self.t)
                    elif kind == "dispatch":
                        self.dispatched[name].append(val)
                    elif kind == "cancel":
                        self.cancelled[name].append(val)
                    elif kind == "log":
                        self.log.append((name, val))

    def run(self, seconds, dt=0.05):
        end = self.t + seconds
        while self.t < end:
            self.t += dt
            if self.t >= self._next_status:
                # fleet_radio's /fleet/status heartbeat from every live robot
                self._next_status += self.status_dt
                for rid in self.nodes:
                    if rid == "gateway" or rid in self.down:
                        continue
                    for other_name, other in self.nodes.items():
                        if other_name not in self.down:
                            other.on_peer_status(rid, self.t)
            for n in self.nodes.values():
                n.tick(self.t)  # a cut-off robot's clock keeps running
            self._pump()

    def request(self, pickup="shelf_a", dropoff="zone_a"):
        tid = self.gw.request(pickup, dropoff, self.t)
        self._pump()
        return tid

    def report(self, rid, tid, state):
        self.nodes[rid].on_task_status(
            json.dumps({"task_id": f"{ft.TASK_PREFIX}{tid}", "state": state}), self.t)
        self._pump()

    def board(self, tid):
        return next(r for r in self.gw.board() if r["task_id"] == tid)


def shelf_a():
    x, y, _ = STATIONS["shelf_a"].dock_pose()
    return (x, y)


def near(dx):
    x, y = shelf_a()
    return (x + dx, y)


def test_robot_ids_map_to_wire_ids_with_gateway_zero():
    assert ft.robot_num("robot_0") == 1 and ft.robot_num("robot_3") == 4
    assert ft.robot_name(1) == "robot_0" and ft.robot_name(0) == "gateway"


def test_closest_robot_wins_and_only_it_gets_dispatched():
    bus = Bus({"robot_0": near(6.0), "robot_1": near(1.0)})
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    assert bus.board(tid)["owner"] == "robot_1"
    assert bus.dispatched["robot_1"] == [
        {"task_id": f"fleet-{tid}", "pickup": "shelf_a", "dropoff": "zone_a"}]
    assert bus.dispatched["robot_0"] == []


def test_done_completes_fleet_wide():
    bus = Bus({"robot_0": near(1.0), "robot_1": near(5.0)})
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    bus.report("robot_0", tid, "DONE")
    bus.run(1.0)
    assert {n.alloc.tasks[tid].state for n in bus.nodes.values()} == {"done"}


def test_failed_task_goes_to_another_robot_not_back_to_the_same_one():
    bus = Bus({"robot_0": near(1.0), "robot_1": near(5.0)})
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    assert bus.board(tid)["owner"] == "robot_0"
    bus.report("robot_0", tid, "FAILED")
    bus.run(5.0)
    assert bus.board(tid)["owner"] == "robot_1"
    assert [d["task_id"] for d in bus.dispatched["robot_0"]] == [f"fleet-{tid}"]


def test_only_robot_retries_its_failed_task_after_the_backoff():
    bus = Bus({"robot_0": near(1.0)}, failed_backoff_s=10.0)
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    bus.report("robot_0", tid, "FAILED")
    bus.run(5.0)
    assert bus.board(tid)["owner"] is None  # still backing off
    bus.run(10.0)
    assert bus.board(tid)["owner"] == "robot_0"
    assert len(bus.dispatched["robot_0"]) == 2


def test_no_bid_robot_follows_but_never_wins():
    bus = Bus({"robot_0": near(8.0), "robot_1": near(0.5)}, no_bid={"robot_1"})
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    assert bus.board(tid)["owner"] == "robot_0"
    assert bus.nodes["robot_1"].alloc.tasks[tid].owner == ft.robot_num("robot_0")


def test_low_battery_robot_loses_to_slightly_farther_full_one():
    bus = Bus({"robot_0": near(1.0), "robot_1": near(3.0)})
    bus.nodes["robot_0"].battery_pct, bus.nodes["robot_1"].battery_pct = 25.0, 100.0
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    assert bus.board(tid)["owner"] == "robot_1"


def test_simulated_battery_drains_only_while_moving():
    n = ft.FleetTasks("robot_0", Params(), STATIONS, cfg=CFG)
    n.battery_pct = 50.0
    n.drive(10.0, moving=False)
    assert n.battery_pct == 50.0
    n.drive(10.0, moving=True)
    assert n.battery_pct == 50.0 - 10.0 * Params().battery_drain_pct_per_s


def test_robot_without_localization_does_not_bid():
    bus = Bus({"robot_0": None, "robot_1": near(9.0)})
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    assert bus.board(tid)["owner"] == "robot_1"


def test_gateway_cancel_stops_the_owner():
    bus = Bus({"robot_0": near(1.0)})
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    bus.gw.cancel(tid, bus.t)
    bus._pump()
    assert bus.cancelled["robot_0"] == [f"fleet-{tid}"]
    assert bus.board(tid)["state"] == "cancelled"


def test_isolated_owner_stops_and_task_is_reclaimed():
    bus = Bus({"robot_0": near(1.0), "robot_1": near(4.0)})
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    assert bus.board(tid)["owner"] == "robot_0"
    bus.down.add("robot_0")
    bus.run(20.0)
    assert bus.cancelled["robot_0"] == [f"fleet-{tid}"]  # stopped working it
    assert bus.board(tid)["owner"] == "robot_1"
    assert any(n == "robot_0" and "lease_lost" in t for n, t in bus.log)


def test_late_joiner_learns_open_tasks_from_reannounce():
    bus = Bus({"robot_0": near(2.0)}, failed_backoff_s=1000.0)
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    bus.report("robot_0", tid, "FAILED")  # robot_0 now refuses it
    bus.run(2.0)
    late = bus.add("robot_1", near(4.0))
    bus.run(8.0)
    assert tid in late.alloc.tasks
    assert bus.board(tid)["owner"] == "robot_1"


def test_own_and_malformed_messages_are_ignored():
    n = ft.FleetTasks("robot_0", Params(), STATIONS, cfg=CFG)
    n.on_wire(ft.encode(1, {"type": "Claim", "task_id": 5, "cost": 1.0}), 0.0)
    assert 5 not in n.alloc.tasks
    n.on_wire("not json", 0.0)
    n.on_wire(json.dumps({"type": "Bid"}), 0.0)  # no sender
    assert any(k == "log" and "malformed" in v for k, v in n.drain())


def test_unknown_station_request_is_rejected():
    bus = Bus({"robot_0": near(1.0)})
    assert bus.request("nowhere", "zone_a") is None


def test_alloc_state_reaches_fleet_radio_status_schema():
    bus = Bus({"robot_0": near(1.0)})
    bus.nodes["robot_0"].battery_pct = 80.0
    bus.run(1.0)
    tid = bus.request()
    bus.run(2.0)
    n = bus.nodes["robot_0"]
    n._last_alloc = None
    n._flush()
    alloc = [v for k, v in n.drain() if k == "alloc"][-1]
    raw = fr.encode_status("robot_0", 0, 0, 0, owned=alloc["owned"],
                           battery_pct=alloc["battery_pct"])
    st = fr.parse_status(raw)
    assert [t for t, _ in st["owned"]] == [tid] and st["battery_pct"] == 80.0
