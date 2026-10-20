"""Offline tests for nodes/fleet_tasks.py: the auction over the real JSON wire
format, with a fake task_manager. No ROS — the system python3 is enough.

Each robot's pose and battery are set directly; the bus delivers every
publish to every node (including the sender, as DDS does).
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))

import fleet_radio as fr  # noqa: E402
import fleet_tasks as ft  # noqa: E402
from commsim.core import fleet as F  # noqa: E402
from commsim.core import timers  # noqa: E402
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
            n.set_pose(pose, 0.0)
            self.nodes[rid] = n
        self.poses_live = True  # the TF timer keeps every pose fresh (False: it stopped)
        self.status_dt = 1.0 / status_hz
        self._next_status = 0.0
        self.dispatched: dict[str, list] = {rid: [] for rid in robots}
        self.cancelled: dict[str, list] = {rid: [] for rid in robots}
        self.log: list = []
        self.down: set = set()

    def add(self, rid, pose):
        n = ft.FleetTasks(rid, self.params, STATIONS, cfg=CFG, now=self.t)
        n.set_pose(pose, self.t)
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
                            other.on_peer_status(rid, self.t,
                                                 battery_pct=self.nodes[rid].battery_pct)
            for n in self.nodes.values():
                if self.poses_live and n.pose is not None:
                    n.set_pose(n.pose, self.t)
                n.tick(self.t)  # a cut-off robot's clock keeps running
            self._pump()

    def request(self, pickup="shelf_a", dropoff="zone_a"):
        tid = self.gw.request(pickup, dropoff, self.t)
        self._pump()
        return tid

    def report(self, rid, tid, state, **extra):
        self.nodes[rid].on_task_status(
            json.dumps({"task_id": f"{ft.TASK_PREFIX}{tid}", "state": state, **extra}), self.t)
        self._pump()

    def board(self, tid):
        return next(r for r in self.gw.board()["tasks"] if r["task_id"] == tid)

    def robot(self, rid):
        return next(r for r in self.gw.board()["robots"] if r["id"] == rid)

    def alarms(self, kind=None, active=None):
        return [a for a in self.gw.board()["alarms"]
                if (kind is None or a["kind"] == kind) and (active is None or a["active"] == active)]


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


def test_loaded_robot_bids_with_its_backlog_so_the_idle_one_wins():
    # Up to 3 tasks per robot (team decision 2026-10-19), worked in order. A
    # robot already holding a long task must not win a second one just
    # because it stands next to the pickup: the idle one across the aisle
    # would finish it sooner.
    zone_b = STATIONS["zone_b"].dock_pose()[:2]
    assert math.dist(shelf_a(), zone_b) > 1.25  # else robot_0 should win below
    cfg = {"protocol": {**CFG["protocol"], "max_owned_tasks": 3}}
    bus = Bus({"robot_0": near(0.5), "robot_1": near(3.0)}, **cfg)
    bus.run(1.0)
    t1 = bus.request("shelf_a", "zone_b")
    bus.run(3.0)
    assert bus.board(t1)["owner"] == "robot_0"
    t2 = bus.request("shelf_a", "zone_a")
    bus.run(3.0)
    assert bus.board(t2)["owner"] == "robot_1"
    assert [d["task_id"] for d in bus.dispatched["robot_1"]] == [f"fleet-{t2}"]


def test_robot_queues_up_to_three_tasks_then_takes_the_next_when_one_is_done():
    cfg = {"protocol": {**CFG["protocol"], "max_owned_tasks": 3}}
    bus = Bus({"robot_0": near(0.5)}, **cfg)
    bus.run(1.0)
    tids = [bus.request() for _ in range(4)]
    bus.run(4.0)
    assert [d["task_id"] for d in bus.dispatched["robot_0"]] == [f"fleet-{t}" for t in tids[:3]]
    assert bus.board(tids[3])["owner"] is None
    bus.report("robot_0", tids[0], "DONE")
    bus.run(3.0)
    assert bus.board(tids[3])["owner"] == "robot_0"
    assert bus.dispatched["robot_0"][-1]["task_id"] == f"fleet-{tids[3]}"


# ---- the item is on the robot (commsim protocol.py "Committed ownership")

def test_picked_up_task_survives_a_failed_dropoff_and_is_retried_carrying():
    bus = Bus({"robot_0": near(0.5), "robot_1": near(3.0)}, failed_backoff_s=2.0)
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    assert bus.board(tid)["owner"] == "robot_0"
    bus.report("robot_0", tid, "NAV_TO_DROPOFF")
    assert bus.nodes["robot_0"].alloc.committed(tid)
    bus.report("robot_0", tid, "FAILED")  # nav to the dropoff failed, item on board
    bus.run(4.0)
    assert bus.board(tid)["owner"] == "robot_0"  # not released, not re-auctioned
    assert bus.dispatched["robot_1"] == []
    assert bus.dispatched["robot_0"][-1] == {
        "task_id": f"fleet-{tid}", "pickup": "shelf_a", "dropoff": "zone_a", "carrying": True}
    bus.report("robot_0", tid, "DONE")
    bus.run(1.0)
    assert {n.alloc.tasks[tid].state for n in bus.nodes.values()} == {"done"}


def test_cut_off_robot_with_the_item_keeps_the_task_and_nobody_else_is_sent():
    bus = Bus({"robot_0": near(1.0), "robot_1": near(5.0)})
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    bus.report("robot_0", tid, "NAV_TO_DROPOFF")
    bus.run(1.0)
    bus.down = {"robot_0"}
    bus.run(30.0)
    assert bus.dispatched["robot_1"] == []
    assert bus.cancelled["robot_0"] == []
    assert bus.board(tid)["owner"] == "robot_0"
    assert any(n == "gateway" and "owner_missing" in m for n, m in bus.log)
    bus.down = set()
    bus.run(2.0)
    bus.report("robot_0", tid, "DONE")
    bus.run(1.0)
    assert {n.alloc.tasks[tid].state for n in bus.nodes.values()} == {"done"}


def test_gateway_cancel_of_a_carried_task_stops_the_robot_and_says_so():
    bus = Bus({"robot_0": near(1.0)})
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    bus.report("robot_0", tid, "NAV_TO_DROPOFF")
    bus.gw.cancel(tid, bus.t)
    bus.run(1.0)
    assert bus.cancelled["robot_0"] == [f"fleet-{tid}"]
    assert any(n == "robot_0" and "ITEM ON BOARD" in m for n, m in bus.log)


def test_restarted_fleet_tasks_adopts_the_carried_task_and_cancels_the_rest():
    bus = Bus({"robot_0": near(0.5)})
    bus.run(1.0)
    t1, t2 = bus.request(), bus.request()
    bus.run(4.0)
    assert [d["task_id"] for d in bus.dispatched["robot_0"]] == [f"fleet-{t1}", f"fleet-{t2}"]
    # fleet_tasks restarts; task_manager kept driving t1 with the item on
    # board and still has t2 queued. The latched status replays first.
    bus.nodes["robot_0"] = fresh = ft.FleetTasks("robot_0", bus.params, STATIONS, cfg=CFG,
                                                 now=bus.t)
    fresh.set_pose(near(0.5), bus.t)
    bus.report("robot_0", t1, "NAV_TO_DROPOFF", pickup="shelf_a", dropoff="zone_a",
               queue=[f"fleet-{t2}"])
    assert fresh.alloc.committed(t1)
    assert bus.cancelled["robot_0"] == [f"fleet-{t2}"]
    bus.add("robot_1", near(3.0))
    bus.run(20.0)  # past lease + margin: the fleet re-auctions t2, never t1
    assert bus.board(t1)["owner"] == "robot_0"
    assert bus.board(t2)["owner"] == "robot_1"
    assert not any("owner_missing" in m or "lease_expired" in m
                   for n, m in bus.log if n == "gateway" and str(t1) in m)
    bus.report("robot_0", t1, "DONE")
    bus.run(1.0)
    assert bus.board(t1)["state"] == "done"


def test_stale_pose_stops_bidding_until_tf_is_back():
    bus = Bus({"robot_0": near(1.0)})
    bus.poses_live = False
    bus.run(CFG["pose_stale_s"] + 1.0)
    tid = bus.request()
    bus.run(3.0)
    assert bus.dispatched["robot_0"] == []
    assert any("pose stale" in m for _, m in bus.log)
    bus.poses_live = True
    bus.nodes["robot_0"].set_pose(near(1.0), bus.t)
    bus.run(6.0)  # the gateway re-announces; a rebid round picks it up
    assert bus.board(tid)["owner"] == "robot_0"


# ---- the operator's board (requirement 2026-10-20: battery, heartbeat and
# task ownership reach the GUI; alarms latch until acknowledged)

def test_board_shows_every_robot_with_battery_liveness_and_tasks():
    bus = Bus({"robot_0": near(0.5), "robot_1": near(5.0)})
    bus.nodes["robot_0"].battery_pct = 73.0
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    bus.report("robot_0", tid, "NAV_TO_DROPOFF")
    bus.run(1.0)
    b = bus.gw.board()
    assert {r["id"] for r in b["robots"]} == {"robot_0", "robot_1"}
    r0, r1 = bus.robot("robot_0"), bus.robot("robot_1")
    assert r0["alive"] and r1["alive"] and r0["last_heard_s"] < 0.5
    assert r0["battery_pct"] == 73.0 and r1["battery_pct"] is None
    assert r0["owned"] == [tid] and r0["carrying"] == [tid] and r1["owned"] == []
    assert bus.board(tid)["committed"] and bus.board(tid)["age_s"] > 3
    assert b["alarms"] == []


def test_silent_robot_raises_a_latched_alarm_that_clears_on_return():
    bus = Bus({"robot_0": near(1.0), "robot_1": near(4.0)})
    bus.run(1.0)
    bus.down.add("robot_1")
    bus.run(CFG["protocol"]["status_timeout_s"] + 1.0)
    assert not bus.robot("robot_1")["alive"] and bus.robot("robot_0")["alive"]
    (al,) = bus.alarms("robot_silent")
    assert al["robot"] == "robot_1" and al["active"] and not al["acked"]
    bus.down.clear()
    bus.run(1.0)
    assert bus.robot("robot_1")["alive"]
    (al,) = bus.alarms("robot_silent")
    assert not al["active"] and not al["acked"]  # returned to normal, still unacked
    bus.gw.ack(al["id"])
    assert bus.alarms() == []  # cleared AND acked: gone
    bus.down.add("robot_1")
    bus.run(CFG["protocol"]["status_timeout_s"] + 1.0)
    assert bus.alarms("robot_silent", active=True)[0]["id"] != al["id"]  # a new episode


def test_carried_task_with_a_silent_owner_alarms_until_it_is_finished():
    bus = Bus({"robot_0": near(1.0), "robot_1": near(5.0)})
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    bus.report("robot_0", tid, "NAV_TO_DROPOFF")
    bus.down = {"robot_0"}
    bus.run(30.0)
    (al,) = bus.alarms("owner_missing")
    assert al["task_id"] == tid and al["robot"] == "robot_0" and al["active"]
    bus.down = set()
    bus.run(2.0)
    bus.report("robot_0", tid, "DONE")
    bus.run(2.0)
    assert bus.alarms("owner_missing", active=True) == []


def test_unclaimed_task_alarms_as_stalled_until_someone_takes_it():
    bus = Bus({"robot_0": near(1.0)}, stalled_after_s=5.0, failed_backoff_s=20.0)
    bus.run(1.0)
    tid = bus.request()
    bus.run(3.0)
    bus.report("robot_0", tid, "FAILED")  # the only robot refuses it for 20 s
    bus.run(8.0)
    (al,) = bus.alarms("task_stalled", active=True)
    assert al["task_id"] == tid
    bus.run(15.0)  # backoff over: robot_0 takes it again
    assert bus.board(tid)["owner"] == "robot_0"
    assert bus.alarms("task_stalled", active=True) == []


def test_robot_timers_match_the_measured_commsim_config():
    # configs/fleet_tasks.yaml mirrors commsim/configs/base.yaml by hand.
    # The timer ordering in commsim/core/timers.py was measured against
    # base.yaml; the robots must not drift from it unnoticed.
    base = F.load_config(REPO / "commsim" / "configs" / "base.yaml")
    for k in ("status_timeout_s", "task_lease_s", "reclaim_margin_s", "bid_window_s",
              "lease_beacon_hz", "max_open_bids"):
        assert CFG["protocol"][k] == base["protocol"][k], k
    merged = {**base, "protocol": {**base["protocol"], **CFG["protocol"]}}
    assert timers.check(merged) == []
