#!/usr/bin/env python3
"""M1 stub: publish commsim_msgs/Status at --hz, record every peer Status heard.

Runs inside one emulated robot's namespace, against that namespace's own
rmw_zenohd on tcp/localhost:7447.

--transport udp (PROTOTYPE): the same CDR-serialized Status goes best-effort
over UDP unicast to each peer's mesh address instead of through zenoh/TCP —
batman-adv still routes it multi-hop, but there is no TCP ACK, retransmission
or exponential backoff to stretch an outage past the reroute itself. All
namespaces share the VM's clock, so recv - header.stamp IS the one-way latency
(no clock-sync error in emulation).

--alloc (UDP only): also run the REAL task allocator (commsim/core/protocol.py,
the code on the robots) over the same prototype wire: TaskAnnounce, Bid,
Claim, LeaseBeacon and TaskEvent as commsim_msgs CDR, one UDP unicast per
peer, topic ids 2-6. The gateway (--robot-id 0) announces a task every
--task-every seconds at random points on a --floor W,H; robots bid with the
straight-line drive time from --pos; a winner "works" the task in time only
(picks up 2 s after the claim, completes after its bid, capped) — there is no
motion in the emulation. Every allocator event, send and receive is logged to
--alloc-log as JSON lines: the run's own record of every auction decision and
silence event.

    python3 status_stub.py --robot-id 1 --hz 3 --duration 120 --out /run/commsim/r1.csv
"""
import argparse
import csv
import json
import math
import random
import socket
import struct
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message, serialize_message

from commsim_msgs.msg import Bid, Claim, LeaseBeacon, OwnedTask, Point2, Status, TaskAnnounce, TaskEvent

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # the repo root, for commsim.core
from commsim.core.protocol import Allocator, Params, deliver  # noqa: E402

MAGIC = 0x5342
# prototype wire: topic id -> message class. 1 is the Status of the first
# sessions; the task protocol got the next ids (2026-10-20).
TOPICS = {1: Status, 2: TaskAnnounce, 3: Bid, 4: Claim, 5: LeaseBeacon, 6: TaskEvent}
TOPIC_OF = {cls: tid for tid, cls in TOPICS.items()}
TYPE_NAME = {Status: "Status", TaskAnnounce: "TaskAnnounce", Bid: "Bid", Claim: "Claim",
             LeaseBeacon: "LeaseBeacon", TaskEvent: "TaskEvent"}
SPEED_MPS = 0.5


class Stub(Node):
    def __init__(self, a):
        super().__init__(f"status_stub_{a.robot_id}")
        self.a, self.seq = a, 0
        # Status contract: best-effort, newest wins.
        qos = QoSProfile(depth=a.depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Status, a.topic, qos)
        self.create_subscription(Status, a.topic, self.on_status, qos)
        self.f = open(a.out, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["src", "dst", "seq", "send_ns", "recv_ns"])
        # every Status we send, so delivery can be computed against what was
        # SENT (a receive-only log hides statuses nobody got)
        self.sf = open(a.sent_out, "w", newline="") if a.sent_out else None
        self.sw = csv.writer(self.sf) if self.sf else None
        if self.sw:
            self.sw.writerow(["src", "seq", "send_ns"])
        self.create_timer(1.0 / a.hz, self.tick)
        self.udp = None
        if a.transport == "udp":
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # mesh IP only: 0.0.0.0 would also listen on campus Wi-Fi
            self.udp.bind((a.bind, a.udp_port))
            self.udp.setblocking(False)
            self.peers = [ip for ip in a.peers.split(",") if ip]
            self.create_timer(0.002, self.poll_udp)
        # ---- the task allocator (optional) ----
        self.alloc = None
        if a.alloc:
            if self.udp is None:
                raise SystemExit("--alloc needs --transport udp")
            self.t0 = time.monotonic()
            self.pos = tuple(float(x) for x in a.pos.split(",")) if a.pos else (0.0, 0.0)
            params = Params()
            if a.params:
                params = Params.from_config(json.loads(Path(a.params).read_text()))
            gw = a.robot_id == 0
            self.alloc = Allocator(a.robot_id, params, None if gw else self._cost, is_gateway=gw)
            self.alog = open(a.alloc_log, "w") if a.alloc_log else None
            self.work = {}          # task_id -> (commit_at, complete_at) monotonic
            self.next_task = 0
            self.rng = random.Random(a.robot_id + 11)
            self.floor = tuple(float(x) for x in a.floor.split(",")) if a.floor else (50.0, 32.0)
            self.create_timer(0.05, self.alloc_tick)
            self.log_ev({"ev": "start", "pos": list(self.pos), "gateway": gw})

    # ---- wire -----------------------------------------------------------------

    def send_msg(self, cls, m) -> int:
        """One UDP unicast per peer of a commsim_msgs message with the prototype
        header. Returns the FleetHeader seq used (one counter for every type:
        a capture's hops join by (topic, robot, seq))."""
        m.header.version, m.header.robot_id, m.header.seq = 1, self.a.robot_id, self.seq
        ns = time.time_ns()
        m.header.stamp.sec, m.header.stamp.nanosec = ns // 10**9, ns % 10**9
        seq, self.seq = self.seq, self.seq + 1
        data = serialize_message(m)
        pkt = struct.pack("!HHI", MAGIC, TOPIC_OF[cls], len(data)) + data
        for ip in self.peers:
            try:
                self.udp.sendto(pkt, (ip, self.a.udp_port))
            except OSError:
                pass  # no route right now (mid-reroute): best effort, drop
        return seq

    def tick(self):
        m = Status()
        m.pose.x, m.pose.y, m.battery_pct = float(self.a.robot_id), 0.0, 90
        if self.alloc is not None:
            m.pose.x, m.pose.y = self.pos
            m.owned = [OwnedTask(task_id=t, cost=float(c)) for t, c in self.alloc.owned()]
        else:
            m.owned = [OwnedTask(task_id=self.a.robot_id, cost=12.5)]
        if self.udp is None:
            m.header.version, m.header.robot_id, m.header.seq = 1, self.a.robot_id, self.seq
            ns = time.time_ns()
            m.header.stamp.sec, m.header.stamp.nanosec = ns // 10**9, ns % 10**9
            if self.sw:
                self.sw.writerow([self.a.robot_id, self.seq, ns])
            self.seq += 1
            self.pub.publish(m)
            return
        ns = time.time_ns()
        seq = self.send_msg(Status, m)
        if self.sw:
            self.sw.writerow([self.a.robot_id, seq, ns])

    def poll_udp(self):
        while True:
            try:
                pkt, _ = self.udp.recvfrom(4096)
            except BlockingIOError:
                return
            magic, topic, n = struct.unpack("!HHI", pkt[:8])
            cls = TOPICS.get(topic)
            if magic != MAGIC or cls is None:
                continue
            m = deserialize_message(pkt[8:8 + n], cls)
            if cls is Status:
                self.on_status(m)
            elif self.alloc is not None:
                self.on_proto(cls, m)

    def on_status(self, m):
        if m.header.robot_id == self.a.robot_id:
            return
        recv = time.time_ns()
        send = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        self.w.writerow([m.header.robot_id, self.a.robot_id, m.header.seq, send, recv])
        if self.alloc is not None:
            self.alloc.on_status(m.header.robot_id, [(o.task_id, o.cost) for o in m.owned], self.now())
            self.flush()

    # ---- allocator ----------------------------------------------------------------

    def now(self) -> float:
        return time.monotonic() - self.t0

    def _cost(self, pickup, dropoff=None, start=None):
        here = start if start is not None else self.pos
        d = math.dist(here, pickup)
        leg = math.dist(pickup, dropoff) if dropoff is not None else 0.0
        return d / SPEED_MPS, d, (d + leg) / SPEED_MPS

    def log_ev(self, d: dict) -> None:
        if self.alog is not None:
            self.alog.write(json.dumps({"t_ns": time.time_ns(), "robot": self.a.robot_id, **d}) + "\n")
            self.alog.flush()

    def on_proto(self, cls, m):
        src = m.header.robot_id
        if src == self.a.robot_id:
            return
        if cls is TaskAnnounce:
            d = {"type": "TaskAnnounce", "task_id": m.task_id, "pickup": (m.pickup.x, m.pickup.y),
                 "dropoff": (m.dropoff.x, m.dropoff.y)}
        elif cls is Bid:
            d = {"type": "Bid", "task_id": m.task_id, "cost": m.cost}
        elif cls is Claim:
            d = {"type": "Claim", "task_id": m.task_id, "cost": m.cost}
        elif cls is LeaseBeacon:
            d = {"type": "LeaseBeacon", "owned": [(o.task_id, o.cost) for o in m.owned]}
        elif cls is TaskEvent:
            d = {"type": "TaskEvent", "task_id": m.task_id, "kind": m.kind}
        else:
            return
        self.log_ev({"ev": "rx", "type": d["type"], "src": src, "seq": m.header.seq,
                     "task_id": d.get("task_id"),
                     "lat_us": (time.time_ns() - (m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)) // 1000})
        deliver(self.alloc, src, d, self.now())
        self.flush()

    def flush(self):
        out, events = self.alloc.drain()
        for m in out:
            t = m["type"]
            if t == "TaskAnnounce":
                msg = TaskAnnounce(task_id=m["task_id"], pickup=Point2(x=float(m["pickup"][0]), y=float(m["pickup"][1])),
                                   dropoff=Point2(x=float(m["dropoff"][0]), y=float(m["dropoff"][1])))
            elif t == "Bid":
                msg = Bid(task_id=m["task_id"], cost=float(m["cost"]))
            elif t == "Claim":
                msg = Claim(task_id=m["task_id"], cost=float(m["cost"]), lease_s=float(m["lease_s"]))
            elif t == "LeaseBeacon":
                msg = LeaseBeacon(owned=[OwnedTask(task_id=tid, cost=float(c)) for tid, c in m["owned"]])
            elif t == "TaskEvent":
                msg = TaskEvent(task_id=m["task_id"], kind=int(m["kind"]))
            else:
                continue
            seq = self.send_msg(type(msg), msg)
            self.log_ev({"ev": "tx", "type": t, "seq": seq, "task_id": m.get("task_id"),
                         **({"pickup": list(m["pickup"]), "dropoff": list(m["dropoff"])} if t == "TaskAnnounce" else {}),
                         **({"cost": m["cost"]} if "cost" in m else {})})
        for e in events:
            self.log_ev({"ev": e["event"], **{k: v for k, v in e.items() if k not in ("event", "robot_id", "t")}})
            if e["event"] == "claimed":
                # no motion here: the item is "picked up" 2 s after the claim and
                # the task completes after the bid's drive time, capped
                n = self.now()
                self.work[e["task_id"]] = (n + 2.0, n + min(2.0 + float(e.get("cost", 10.0)), 40.0))
            elif e["event"] in ("stop_task", "task_yielded"):
                self.work.pop(e["task_id"], None)

    def alloc_tick(self):
        n = self.now()
        a = self.alloc
        if self.a.robot_id == 0 and self.a.task_every > 0 and n >= self.next_task:
            self.next_task = n + self.a.task_every
            tid = int(time.time() * 10) % 1_000_000
            pick = (self.rng.uniform(2.0, self.floor[0] - 2.0), self.rng.uniform(1.0, self.floor[1] - 1.0))
            drop = (self.rng.uniform(2.0, self.floor[0] - 2.0), self.rng.uniform(1.0, self.floor[1] - 1.0))
            a.announce(tid, pick, drop, n)
            self.log_ev({"ev": "announce", "task_id": tid, "pickup": list(pick), "dropoff": list(drop)})
        for tid, (commit_at, done_at) in list(self.work.items()):
            if n >= commit_at and not a.committed(tid):
                a.commit(tid, n)
            if n >= done_at:
                del self.work[tid]
                if tid in a.tasks:
                    a.complete(tid, n)
                    self.log_ev({"ev": "complete", "task_id": tid})
        a.tick(n)
        self.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", type=int, required=True)
    ap.add_argument("--hz", type=float, default=3.0)
    ap.add_argument("--topic", default="fleet/status/c0")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--sent-out", default=None, help="CSV of every Status this robot sent")
    ap.add_argument("--transport", choices=["zenoh", "udp"], default="zenoh")
    ap.add_argument("--peers", default="", help="comma-separated mesh IPs (udp transport)")
    ap.add_argument("--udp-port", type=int, default=7450)
    ap.add_argument("--bind", default=None, help="this robot's mesh IP (required for udp)")
    ap.add_argument("--depth", type=int, default=32,
                    help="KEEP_LAST depth; the queue is per subscription, not per publisher")
    ap.add_argument("--alloc", action="store_true", help="run the task allocator too (udp only)")
    ap.add_argument("--pos", default=None, help="this robot's x,y (metres) for bid costs")
    ap.add_argument("--floor", default=None, help="W,H of the floor tasks are drawn on")
    ap.add_argument("--task-every", type=float, default=0.0, help="gateway: announce a task this often")
    ap.add_argument("--params", default=None, help="path to JSON {protocol: {...}} for the allocator's Params")
    ap.add_argument("--alloc-log", default=None, help="JSON lines of allocator events/sends/receives")
    a = ap.parse_args()
    if a.transport == "udp" and not a.bind:
        ap.error("--bind <mesh IP> is required with --transport udp")
    rclpy.init()
    n = Stub(a)
    end = time.time() + a.duration
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.f.close()
    if n.sf:
        n.sf.close()
    if getattr(n, "alog", None):
        n.alog.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
