#!/usr/bin/env python3
"""Fleet task auction on the robots: commsim's Allocator over ROS 2 topics.

There is no central dispatcher. Every robot runs the same allocator
(commsim/core/protocol.py — the code the DES and the loopback harness
validated, imported, not copied), so all robots reach the same winner on
their own: lowest travel-time-plus-battery cost, ties to the lower ID. The
gateway only injects tasks and keeps a board of what it has heard.

    gateway:  /fleet/task_request {"pickup": "shelf_a", "dropoff": "zone_b"}
                -> TaskAnnounce on /fleet/tasks
    robots:   Bid -> (1 s window) -> Claim by the winner
    winner:   /<robot>/dispatch_task -> task_manager.py works it
              /<robot>/task_status NAV_TO_DROPOFF -> commit (item on board:
                                     the task can no longer move, protocol.py)
                                   DONE   -> Complete (fleet-wide)
                                   FAILED -> Release  (re-auctioned), or if
                                     committed: re-dispatched here after the
                                     backoff with "carrying" (dropoff only)
    loser of ownership (lease lost while isolated, out-bid on a partition
    heal, cancelled by the gateway) -> /<robot>/cancel_task

After a restart this node reads task_manager's latched task_status: a task
with the item on board is adopted (claimed back as committed), anything
else task_manager still holds is cancelled so the fleet re-auctions it.

Bid cost is the path length on the fused /map when one has arrived
(nodes/grid_dist.py), else the straight line.

Every task-protocol message travels on ONE reliable topic, `/fleet/tasks`.
Leases are renewed by LeaseBeacons on that same topic, not by
fleet_radio's `/fleet/status`: a single publisher's messages arrive in
order, so a peer can never see a Release and THEN an older status that
still lists the task, which would put the releaser back as owner until its
lease ran out. `/fleet/status` still counts as liveness (who is alive,
whether we are isolated), and fleet_radio copies our owned tasks and battery
into it for anyone watching.

Robot IDs: robot_N is N+1 on the wire, the gateway is 0 — the allocator's
tie-break is the integer, so robot_0 still wins exact ties, matching
fleet_radio's priority order.

The protocol clock is the steady clock, never sim time: leases and the bid
window bound how long a *message* may take, not how far a robot drives.

    python3 nodes/fleet_tasks.py --robot-id robot_0
    python3 nodes/fleet_tasks.py --gateway

Request a task:

    ros2 topic pub --once /fleet/task_request std_msgs/String \\
        '{data: "{\\"pickup\\": \\"shelf_a\\", \\"dropoff\\": \\"zone_a\\"}"}'
    ros2 topic echo /fleet/task_board
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from commsim.core.protocol import (  # noqa: E402
    BIDDING, CLAIMED, COMMITTED_COST, OPEN, Allocator, Params, TaskView, deliver,
)
from grid_dist import GridDistance  # noqa: E402
from stations import load_stations  # noqa: E402

DEFAULT_CFG = REPO_ROOT / "configs" / "fleet_tasks.yaml"
DEFAULT_WAYPOINTS = REPO_ROOT / "configs" / "waypoints.yaml"

TASKS_TOPIC = "/fleet/tasks"
REQUEST_TOPIC = "/fleet/task_request"
CANCEL_TOPIC = "/fleet/task_cancel"
BOARD_TOPIC = "/fleet/task_board"

GATEWAY = 0
# task_manager's task_id for an auctioned task; the prefix tells its
# task_status apart from tasks a person dispatched by hand.
TASK_PREFIX = "fleet-"

DEFAULTS = {
    "speed_mps": 0.5,           # Nav2 max_velocity x (configs/nav2_params.yaml)
    "failed_backoff_s": 60.0,
    "pose_stale_s": 5.0,        # no TF this long -> we don't know where we are: no bids
    "reannounce_s": 5.0,
    "gateway_status_hz": 1.0,
    "battery": {"sim_start_pct": None},
    "protocol": {},
}


def load_config(path: Path | None = None) -> dict:
    cfg = json.loads(json.dumps(DEFAULTS))
    p = path or DEFAULT_CFG
    if p.exists():
        with open(p) as f:
            raw = yaml.safe_load(f) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"{p}: expected a mapping")
        for k, v in raw.items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
    return cfg


def robot_num(robot_id: str) -> int:
    """robot_0 -> 1, robot_1 -> 2, …; 0 is the gateway."""
    if robot_id.startswith("robot_"):
        return int(robot_id.split("_", 1)[1]) + 1
    raise ValueError(f"not a robot id: {robot_id!r}")


def robot_name(num: int | None) -> str | None:
    if num is None:
        return None
    return "gateway" if num == GATEWAY else f"robot_{num - 1}"


def encode(sender: int, m: dict) -> str:
    return json.dumps({**m, "from": sender})


def decode(raw: str) -> tuple[int, dict] | None:
    try:
        d = json.loads(raw)
        sender = int(d.pop("from"))
        if not isinstance(d.get("type"), str):
            return None
        return sender, d
    except (json.JSONDecodeError, KeyError, TypeError, ValueError, AttributeError):
        return None


class FleetTasks:
    """The protocol side of the node, without ROS: feed it messages and the
    clock, read actions back from `drain()`. Actions are tuples:
        ("publish", raw)         -> /fleet/tasks
        ("dispatch", payload)    -> /<robot>/dispatch_task (JSON dict;
                                    "carrying": true = item already on board)
        ("cancel", task_id_str)  -> /<robot>/cancel_task
        ("alloc", payload)       -> /<robot>/task_alloc, on change
        ("log", text)
    """

    def __init__(self, robot_id: str | None, params: Params, stations: dict, *,
                 gateway: bool = False, can_bid: bool = True, cfg: dict | None = None,
                 now: float = 0.0):
        cfg = cfg or load_config()
        self.cfg = cfg
        self.gateway = gateway
        self.num = GATEWAY if gateway else robot_num(robot_id)
        self.stations = stations
        self.speed = float(cfg["speed_mps"])
        self.pose: tuple[float, float] | None = None
        self._pose_at: float | None = None
        self.grid: GridDistance | None = None
        self.battery_pct: float | None = None
        # Committed tasks whose dropoff leg failed: re-dispatch (carrying) at
        # this time. Releasing them would send another robot to an empty shelf.
        self._retry: dict[int, float] = {}
        self._synced = False  # task_manager's latched status seen (restart adoption)
        self.names: dict[int, tuple[str, str]] = {}
        # (pickup, dropoff) -> steady time until which we do not bid on it
        # again. Without it a robot that cannot do a task (no arm, a pick it
        # keeps fumbling) is still the closest bidder, wins it straight back
        # after every Release, and the task never reaches anyone else.
        self.failed: dict[tuple, float] = {}
        self.now = now
        self._next_id = int(time.time() * 10)
        self._next_reannounce = now
        self._next_heartbeat = now
        self._last_alloc: str | None = None
        self._actions: list[tuple] = []
        bids = not gateway and can_bid
        self.alloc = Allocator(self.num, params, self._cost if bids else None,
                               is_gateway=gateway, battery_fn=lambda: self.battery_pct)

    # ---- inputs -----------------------------------------------------------

    def set_pose(self, xy: tuple[float, float], now: float) -> None:
        self.pose, self._pose_at = xy, now

    def _path_m(self, a, b) -> float:
        """Drive distance a -> b: along the fused map's free cells when we
        have one, else straight line. Not the Nav2 path: asking
        planner_server here would preempt the path request of whatever Nav2
        is executing (ComputePathToPose takes one goal at a time)."""
        if self.grid is not None:
            d = self.grid.path_length(a, b)
            if d is not None:
                return d
        return math.dist(a, b)

    def _cost(self, pickup, dropoff=None, start=None):
        if self.pose is None:
            return None  # no localization yet: we cannot say how far it is
        until = self.failed.get((tuple(pickup), tuple(dropoff or ())))
        if until is not None and self.now < until:
            return None
        # `start` is the last dropoff of what we already own (the allocator
        # bids with the backlog): task_manager works its queue in order.
        origin = start if start is not None else self.pose
        d = self._path_m(origin, pickup)
        leg = self._path_m(pickup, dropoff) if dropoff is not None else 0.0
        return d / self.speed, math.dist(origin, pickup), (d + leg) / self.speed

    def on_wire(self, raw: str, now: float) -> None:
        self.now = now
        got = decode(raw)
        if got is None:
            self._log(f"dropped malformed message: {raw[:120]!r}")
            return
        sender, m = got
        if sender == self.num:
            return  # DDS delivers our own publications back to us
        if m["type"] == "TaskAnnounce" and "pickup_station" in m:
            self.names[int(m["task_id"])] = (m["pickup_station"], m["dropoff_station"])
        try:
            deliver(self.alloc, sender, m, now)
        except (KeyError, TypeError, ValueError) as exc:
            self._log(f"dropped {m.get('type')} from {robot_name(sender)}: {exc}")
        self._flush()

    def on_peer_status(self, robot_id: str, now: float) -> None:
        """A /fleet/status heartbeat: liveness only (see module docstring)."""
        self.now = now
        try:
            num = robot_num(robot_id)
        except ValueError:
            return
        if num != self.num:
            self.alloc.on_status(num, [], now)
            self._flush()

    @staticmethod
    def _fleet_tid(tid_s) -> int | None:
        if not isinstance(tid_s, str) or not tid_s.startswith(TASK_PREFIX):
            return None
        try:
            return int(tid_s[len(TASK_PREFIX):])
        except ValueError:
            return None

    def on_task_status(self, raw: str, now: float) -> None:
        """task_manager's /<robot>/task_status for an auctioned task."""
        self.now = now
        try:
            st = json.loads(raw)
            tid_s, state = st.get("task_id"), st.get("state")
        except (json.JSONDecodeError, AttributeError):
            return
        if not self._synced:
            self._synced = True
            self._adopt(st, now)
        tid = self._fleet_tid(tid_s)
        if tid is None:
            return
        t = self.alloc.tasks.get(tid)
        if t is None or t.state != CLAIMED or t.owner != self.num:
            return  # we no longer own it (cancelled, out-bid): not ours to report
        if state in ("NAV_TO_DROPOFF", "PLACING") and not self.alloc.committed(tid):
            self.alloc.commit(tid, now)
            self._log(f"task {tid}: item picked up, committed")
        elif state == "DONE":
            self._retry.pop(tid, None)
            self.alloc.complete(tid, now)
            self._log(f"task {tid} done")
        elif state == "FAILED":
            until = now + float(self.cfg["failed_backoff_s"])
            if self.alloc.committed(tid):
                # The item is on us: nobody else can finish this. Try the
                # dropoff leg again after the backoff, keeping ownership.
                self._retry[tid] = until
                self._log(f"task {tid} failed with the item on board; "
                          f"retrying the dropoff in {until - now:.0f} s")
            else:
                self.failed[(t.pickup, t.dropoff)] = until
                self.alloc.release(tid, now)
                # Our own view is now OPEN with no rebid round pending.
                # Schedule one for when the backoff ends, or a task only we
                # can do would stay open forever once the other robots have
                # stopped bidding.
                t.rebid_at = until
                self._log(f"task {tid} failed here; released for another robot")
        self._flush()

    def _adopt(self, st: dict, now: float) -> None:
        """First task_status after OUR start: task_manager may have kept
        working through a restart of this node. A task with the item on
        board is ours again (committed: peers accept the claim whatever they
        think). Anything else it holds is cancelled; the fleet re-auctions
        it and we bid fresh."""
        tid = self._fleet_tid(st.get("task_id"))
        state = st.get("state")
        if tid is not None and state in ("NAV_TO_DROPOFF", "PLACING"):
            pick, drop = st.get("pickup"), st.get("dropoff")
            t = TaskView(tid, state=CLAIMED, owner=self.num, owner_cost=COMMITTED_COST,
                         last_renew=now, claimed_at=now)
            if pick in self.stations and drop in self.stations:
                self.names[tid] = (pick, drop)
                t.pickup, t.dropoff = self._xy(pick), self._xy(drop)
            self.alloc.tasks[tid] = t
            self.alloc.out.append({"type": "Claim", "task_id": tid, "cost": COMMITTED_COST,
                                   "lease_s": self.alloc.p.task_lease_s})
            self._log(f"adopted task {tid} after restart: item on board")
        elif tid is not None and state not in ("DONE", "FAILED", "CANCELLED", "IDLE"):
            self._actions.append(("cancel", f"{TASK_PREFIX}{tid}"))
            self._log(f"cancelled task {tid} after restart; the fleet re-auctions it")
        for q in st.get("queue") or []:
            qt = self._fleet_tid(q)
            if qt is not None:
                self._actions.append(("cancel", f"{TASK_PREFIX}{qt}"))
                self._log(f"cancelled queued task {qt} after restart")

    def drive(self, dt: float, moving: bool) -> None:
        """Simulated battery (configs/fleet_tasks.yaml battery.sim_start_pct)
        for the Isaac track, which has none: drains only while driving, with
        the same rate the allocator's gate assumes."""
        if self.battery_pct is not None and moving:
            self.battery_pct = max(
                0.0, self.battery_pct - dt * self.alloc.p.battery_drain_pct_per_s)

    # ---- gateway ------------------------------------------------------------

    def request(self, pickup: str, dropoff: str, now: float) -> int | None:
        self.now = now
        if pickup not in self.stations or dropoff not in self.stations:
            self._log(f"rejected task: unknown station in {pickup!r} -> {dropoff!r}")
            self._flush()
            return None
        # Seeded from the wall clock so a restarted gateway never reuses an ID:
        # robots ignore a re-announce of a task they already know, so a reused
        # ID would make the new task invisible to them.
        tid = self._next_id = max(self._next_id + 1, int(time.time() * 10))
        self.names[tid] = (pickup, dropoff)
        self.alloc.announce(tid, self._xy(pickup), self._xy(dropoff), now)
        self._log(f"announced task {tid}: {pickup} -> {dropoff}")
        self._flush()
        return tid

    def cancel(self, tid: int, now: float) -> None:
        self.now = now
        if tid in self.alloc.tasks:
            self.alloc.cancel(tid, now)
            self._log(f"cancelled task {tid}")
        self._flush()

    def board(self) -> list[dict]:
        rows = []
        for tid, t in sorted(self.alloc.tasks.items()):
            pick, drop = self.names.get(tid, (None, None))
            rows.append({"task_id": tid, "pickup": pick, "dropoff": drop,
                         "state": t.state, "owner": robot_name(t.owner)})
        return rows

    def _xy(self, name: str) -> tuple[float, float]:
        x, y, _ = self.stations[name].dock_pose()
        return (round(x, 3), round(y, 3))

    # ---- clock --------------------------------------------------------------

    def tick(self, now: float) -> None:
        self.now = now
        if self.pose is not None and self._pose_at is not None \
                and now - self._pose_at > float(self.cfg["pose_stale_s"]):
            self.pose = None  # lost localization: a bid from the old pose would be a lie
            self._log("pose stale; not bidding until TF is back")
        for tid, due in list(self._retry.items()):
            if now >= due and self.alloc.committed(tid) and tid in self.names:
                del self._retry[tid]
                pick, drop = self.names[tid]
                self._actions.append(("dispatch", {"task_id": f"{TASK_PREFIX}{tid}",
                                                   "pickup": pick, "dropoff": drop,
                                                   "carrying": True}))
                self._log(f"retrying dropoff of task {tid}")
        self.alloc.tick(now)
        if len(self.names) > len(self.alloc.tasks):  # the allocator pruned finished tasks
            self.names = {k: v for k, v in self.names.items() if k in self.alloc.tasks}
        if self.gateway:
            if now >= self._next_heartbeat:
                # Robots with no peer in range hear nobody but us; without a
                # heartbeat they would call themselves isolated and stop bidding.
                self._next_heartbeat = now + 1.0 / float(self.cfg["gateway_status_hz"])
                self.alloc.out.append({"type": "Status", "owned": []})
            if now >= self._next_reannounce:
                # The topic is volatile: a robot that starts after a task was
                # announced never hears of it. Unowned tasks are repeated;
                # robots that already know one ignore the repeat.
                self._next_reannounce = now + float(self.cfg["reannounce_s"])
                for t in self.alloc.tasks.values():
                    if t.state in (OPEN, BIDDING) and t.pickup is not None:
                        self.alloc.out.append({"type": "TaskAnnounce", "task_id": t.task_id,
                                               "pickup": t.pickup, "dropoff": t.dropoff})
        self._flush()

    # ---- outputs ------------------------------------------------------------

    def _log(self, text: str) -> None:
        self._actions.append(("log", text))

    def _flush(self) -> None:
        out, events = self.alloc.drain()
        for m in out:
            if m["type"] == "TaskAnnounce" and m["task_id"] in self.names:
                pick, drop = self.names[m["task_id"]]
                m = {**m, "pickup_station": pick, "dropoff_station": drop}
            self._actions.append(("publish", encode(self.num, m)))
        for e in events:
            self._on_event(e)
        if not self.gateway:
            state = {"owned": [[tid, c] for tid, c in self.alloc.owned()],
                     "battery_pct": self.battery_pct, "isolated": self.alloc.isolated}
            raw = json.dumps(state)
            if raw != self._last_alloc:
                self._last_alloc = raw
                self._actions.append(("alloc", state))

    def _on_event(self, e: dict) -> None:
        kind, tid = e["event"], e.get("task_id")
        if kind == "claimed":
            if tid not in self.names:
                # Won a task announced without station names (e.g. by a
                # commsim stub). task_manager drives to named stations only.
                self.alloc.release(tid, self.now)
                self._log(f"released task {tid}: no station names")
                return
            pick, drop = self.names[tid]
            self._actions.append(("dispatch", {"task_id": f"{TASK_PREFIX}{tid}",
                                               "pickup": pick, "dropoff": drop}))
            self._log(f"won task {tid} ({pick} -> {drop}), cost {e.get('cost'):.1f}")
        elif kind in ("task_yielded", "stop_task"):
            self._actions.append(("cancel", f"{TASK_PREFIX}{tid}"))
            self._retry.pop(tid, None)
            why = e.get("reason") or f"out-bid by {robot_name(e.get('to'))}"
            if e.get("committed"):
                why += " WITH THE ITEM ON BOARD: unload it by hand"
            self._log(f"stopped task {tid}: {why}")
        elif kind in ("isolated", "rejoined", "hold_safe", "lease_expired",
                      "robot_silent", "robot_back", "battery_gated", "declined",
                      "committed", "owner_missing"):
            detail = {k: v for k, v in e.items() if k not in ("event", "robot_id", "t")}
            self._log(f"{kind} {detail}" if detail else kind)

    def drain(self) -> list[tuple]:
        actions, self._actions = self._actions, []
        return actions


# -- ROS 2 node -------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument("--gateway", action="store_true",
                        help="inject tasks and publish the task board; never bids")
    parser.add_argument("--no-bid", action="store_true",
                        help="follow the protocol but never bid (robot cannot pick)")
    parser.add_argument("--config", type=Path, default=DEFAULT_CFG)
    parser.add_argument("--waypoints", type=Path, default=DEFAULT_WAYPOINTS)
    args = parser.parse_args(argv)

    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
    )
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    import tf2_ros

    import fleet_radio

    cfg = load_config(args.config)
    params = Params.from_config(cfg)
    stations = load_stations(args.waypoints)
    rid = args.robot_id

    # Reliable and deep: a lost Claim or Release costs a lease timeout, and a
    # burst of announces plus every robot's bids arrives at once.
    TASKS_QOS = QoSProfile(depth=200, reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.VOLATILE,
                           history=QoSHistoryPolicy.KEEP_LAST)
    STATUS_QOS = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            durability=QoSDurabilityPolicy.VOLATILE,
                            history=QoSHistoryPolicy.KEEP_LAST)
    LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)

    class FleetTasksNode(Node):
        def __init__(self):
            name = "fleet_gateway" if args.gateway else f"{rid}_fleet_tasks"
            super().__init__(name)
            self.core = FleetTasks(None if args.gateway else rid, params, stations,
                                   gateway=args.gateway, can_bid=not args.no_bid,
                                   cfg=cfg, now=time.monotonic())
            self.sim_battery = (not args.gateway
                                and cfg["battery"].get("sim_start_pct") is not None)
            if self.sim_battery:
                self.core.battery_pct = float(cfg["battery"]["sim_start_pct"])
            self._moving = False
            self._last_tick = time.monotonic()

            self.tasks_pub = self.create_publisher(String, TASKS_TOPIC, TASKS_QOS)
            self.create_subscription(String, TASKS_TOPIC, self._on_tasks, TASKS_QOS)
            self.create_subscription(String, fleet_radio.STATUS_TOPIC, self._on_status,
                                     STATUS_QOS)
            if args.gateway:
                self.board_pub = self.create_publisher(String, BOARD_TOPIC, LATCHED)
                self.create_subscription(String, REQUEST_TOPIC, self._on_request, 10)
                self.create_subscription(String, CANCEL_TOPIC, self._on_cancel, 10)
            else:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
                self.dispatch_pub = self.create_publisher(
                    String, f"/{rid}/dispatch_task", 10)
                self.cancel_pub = self.create_publisher(String, f"/{rid}/cancel_task", 10)
                self.alloc_pub = self.create_publisher(String, f"/{rid}/task_alloc", LATCHED)
                self.create_subscription(String, f"/{rid}/task_status",
                                         self._on_task_status, 10)
                self.create_subscription(BatteryState, f"/{rid}/battery_state",
                                         self._on_battery, 10)
                if self.sim_battery:
                    self.create_subscription(Odometry, f"/{rid}/odom", self._on_odom, 10)
                # The fused world map (nodes/map_merge.py), latched; bids
                # measure drive distance along it instead of a straight line.
                self.create_subscription(OccupancyGrid, "/map", self._on_map, LATCHED)
            # Steady clock: see the module docstring. Under use_sim_time a
            # node-clock timer would stretch the 1 s bid window by the
            # real-time factor (CLAUDE.md, "A ROS timer follows use_sim_time").
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(0.05, self._tick, clock=steady)
            if args.gateway:
                self.create_timer(1.0, self._publish_board, clock=steady)
            else:
                self.create_timer(0.5, self._update_pose, clock=steady)
            role = "gateway" if args.gateway else ("follower" if args.no_bid else "bidder")
            self.get_logger().info(
                f"fleet_tasks {role} on {TASKS_TOPIC} (wire id {self.core.num}, "
                f"{len(stations)} stations, sim battery {self.sim_battery})")

        # -- inputs --
        def _on_tasks(self, msg):
            self.core.on_wire(msg.data, time.monotonic())
            self._act()

        def _on_status(self, msg):
            st = fleet_radio.parse_status(msg.data)
            if st is not None:
                self.core.on_peer_status(st["robot_id"], time.monotonic())
                self._act()

        def _on_task_status(self, msg):
            self.core.on_task_status(msg.data, time.monotonic())
            self._act()

        def _on_battery(self, msg):
            if not math.isnan(msg.percentage):
                self.sim_battery = False  # a real battery overrides the simulated one
                self.core.battery_pct = 100.0 * float(msg.percentage)

        def _on_map(self, msg):
            info = msg.info
            self.core.grid = GridDistance(
                info.width, info.height, info.resolution,
                (info.origin.position.x, info.origin.position.y), msg.data)

        def _on_odom(self, msg):
            tw = msg.twist.twist
            self._moving = abs(tw.linear.x) > 0.02 or abs(tw.angular.z) > 0.05

        def _on_request(self, msg):
            try:
                d = json.loads(msg.data)
                self.core.request(str(d["pickup"]), str(d["dropoff"]), time.monotonic())
            except (json.JSONDecodeError, KeyError, TypeError) as exc:
                self.get_logger().error(f"bad task request {msg.data!r}: {exc}")
            self._act()

        def _on_cancel(self, msg):
            try:
                self.core.cancel(int(json.loads(msg.data)["task_id"]), time.monotonic())
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                self.get_logger().error(f"bad cancel {msg.data!r}: {exc}")
            self._act()

        def _update_pose(self):
            try:
                tr = self.tf_buffer.lookup_transform(
                    "map", f"{rid}/base_link", rclpy.time.Time())
            except Exception:
                return  # keep the last pose; None until localized -> no bids
            t = tr.transform.translation
            self.core.set_pose((t.x, t.y), time.monotonic())

        def _tick(self):
            now = time.monotonic()
            if self.sim_battery:
                self.core.drive(now - self._last_tick, self._moving)
            self._last_tick = now
            self.core.tick(now)
            self._act()

        def _publish_board(self):
            self.board_pub.publish(String(data=json.dumps(self.core.board())))

        # -- outputs --
        def _act(self):
            log = self.get_logger()
            for kind, val in self.core.drain():
                if kind == "publish":
                    self.tasks_pub.publish(String(data=val))
                elif kind == "dispatch":
                    self.dispatch_pub.publish(String(data=json.dumps(val)))
                elif kind == "cancel":
                    self.cancel_pub.publish(String(data=val))
                elif kind == "alloc":
                    self.alloc_pub.publish(String(data=json.dumps(val)))
                elif kind == "log":
                    log.info(val)

    rclpy.init(args=argv)
    node = FleetTasksNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # SIGTERM (launch stopping us) already shut the context down; a
        # second rclpy.shutdown() raises and buries the real log in a trace.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
