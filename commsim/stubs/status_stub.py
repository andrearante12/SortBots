#!/usr/bin/env python3
"""M1 stub: publish commsim_msgs/Status at --hz, record every peer Status heard.

Runs inside one emulated robot's namespace, against that namespace's own
rmw_zenohd on tcp/localhost:7447.

--transport udp (PROTOTYPE): the same CDR-serialized Status goes best-effort
over UDP unicast to each peer's mesh address instead of through zenoh/TCP —
batman-adv still routes it multi-hop, but there is no TCP ACK, retransmission
or exponential backoff to stretch an outage past the reroute itself. All namespaces share the VM's clock, so
recv - header.stamp IS the one-way latency (no clock-sync error in emulation).

    python3 status_stub.py --robot-id 1 --hz 3 --duration 120 --out /run/commsim/r1.csv
"""
import argparse
import csv
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message, serialize_message

from commsim_msgs.msg import Status, OwnedTask


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

    def tick(self):
        m = Status()
        m.header.version, m.header.robot_id, m.header.seq = 1, self.a.robot_id, self.seq
        ns = time.time_ns()
        m.header.stamp.sec, m.header.stamp.nanosec = ns // 10**9, ns % 10**9
        m.pose.x, m.pose.y, m.battery_pct = float(self.a.robot_id), 0.0, 90
        m.owned = [OwnedTask(task_id=self.a.robot_id, cost=12.5)]
        if self.sw:
            self.sw.writerow([self.a.robot_id, self.seq, ns])
        self.seq += 1
        if self.udp is None:
            self.pub.publish(m)
            return
        # 8 B prototype header (magic, topic id, length) + the CDR bytes
        data = serialize_message(m)
        pkt = struct.pack("!HHI", 0x5342, 1, len(data)) + data
        for ip in self.peers:
            try:
                self.udp.sendto(pkt, (ip, self.a.udp_port))
            except OSError:
                pass  # no route right now (mid-reroute): best effort, drop

    def poll_udp(self):
        while True:
            try:
                pkt, _ = self.udp.recvfrom(4096)
            except BlockingIOError:
                return
            magic, topic, n = struct.unpack("!HHI", pkt[:8])
            if magic == 0x5342 and topic == 1:
                self.on_status(deserialize_message(pkt[8:8 + n], Status))

    def on_status(self, m):
        if m.header.robot_id == self.a.robot_id:
            return
        recv = time.time_ns()
        send = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        self.w.writerow([m.header.robot_id, self.a.robot_id, m.header.seq, send, recv])


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
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
