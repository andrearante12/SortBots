#!/usr/bin/env python3
"""Restart rosbridge when it is up but forwards nothing: the page then sits on "Waiting for the
robots" while every topic flows inside the robot.

    python3 nodes/rosbridge_watchdog.py     (started by launch/sortbots_webui.launch.py)

Seen twice on the Jetson (2026-11-16). Once rosbridge had started before a boot-time NTP jump
(the Orin has no RTC battery; Fast DDS times liveness on the wall clock), which
`jetson.sh console` now waits out. The other had no clock jump and no cause found. So this
checks the symptom, not a cause: while the robot pipeline is running, subscribe to /tf through
rosbridge exactly as the page does; nothing in PROBE_S, STRIKES times running, and rosbridge is
killed. The launch respawns it, and the page reconnects by itself (operator.js retries every 2 s).

Stdlib only, and no ROS of its own: a DDS participant here could be stranded the same way and
see nothing. "Is the pipeline up" is a process check instead (sortbots_bringup.launch.py).
"""
from __future__ import annotations

import base64
import json
import os
import socket
import struct
import subprocess
import sys
import time

PORT = int(os.environ.get("ROSBRIDGE_PORT", "9090"))
EVERY_S = 15.0
PROBE_S = 5.0
STRIKES = 2
WARMUP_S = 45.0          # a pipeline this young may not publish /tf yet
MIN_GAP_S = 60.0         # never restart rosbridge more often than this


def log(msg):
    print(f"[rosbridge_watchdog] {msg}", flush=True)


def pipeline_age_s():
    """Seconds the robot pipeline has been running, or None if it isn't."""
    try:
        out = subprocess.run(["pgrep", "-f", "sortbots_bringup.launch.py"], capture_output=True,
                             text=True, timeout=5).stdout.split()
    except (OSError, subprocess.TimeoutExpired):
        return None
    ages = []
    for pid in out:
        try:
            with open(f"/proc/{pid}/stat") as f:
                start_ticks = int(f.read().rsplit(")", 1)[1].split()[19])
            with open("/proc/uptime") as f:
                up = float(f.read().split()[0])
            ages.append(up - start_ticks / os.sysconf("SC_CLK_TCK"))
        except (OSError, ValueError, IndexError):
            continue
    return max(ages) if ages else None


def tf_through_rosbridge(timeout_s=PROBE_S):
    """How many /tf messages rosbridge forwards in timeout_s, or None if it won't connect."""
    try:
        s = socket.create_connection(("127.0.0.1", PORT), timeout=3)
    except OSError:
        return None
    try:
        key = base64.b64encode(os.urandom(16)).decode()
        s.sendall(f"GET / HTTP/1.1\r\nHost: localhost:{PORT}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                  f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n".encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                return None
            buf += chunk
        head, buf = buf.split(b"\r\n\r\n", 1)
        if b" 101 " not in head.split(b"\r\n")[0]:
            return None
        data = json.dumps({"op": "subscribe", "id": "watchdog", "topic": "/tf",
                           "type": "tf2_msgs/TFMessage", "throttle_rate": 200}).encode()
        mask = os.urandom(4)
        hdr = bytes([0x81]) + (bytes([0x80 | len(data)]) if len(data) < 126
                               else bytes([0x80 | 126]) + struct.pack(">H", len(data)))
        s.sendall(hdr + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))
        n, deadline = 0, time.monotonic() + timeout_s
        s.settimeout(0.5)
        while time.monotonic() < deadline:
            try:
                chunk = s.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                break
            buf += chunk
            # Only /tf is subscribed, so every publish op is one. (rosbridge writes the topic as
            # "\/tf", slash escaped: matching on the topic name counted nothing.)
            n += buf.count(b'"op":"publish"') + buf.count(b'"op": "publish"')
            buf = buf[-16:]   # keep a tail so a marker split across reads still counts once
        return n
    except OSError:
        return None
    finally:
        s.close()


def main():
    strikes, last_restart = 0, -1e9
    log(f"watching rosbridge on :{PORT} every {EVERY_S:.0f} s while the robot pipeline runs")
    while True:
        time.sleep(EVERY_S)
        age = pipeline_age_s()
        if age is None or age < WARMUP_S:
            strikes = 0
            continue
        n = tf_through_rosbridge()
        if n is None or n > 0:
            strikes = 0            # forwarding, or not up at all (the launch handles that)
            continue
        strikes += 1
        log(f"robot pipeline up {age:.0f} s but rosbridge forwarded no /tf in {PROBE_S:.0f} s "
            f"({strikes}/{STRIKES})")
        if strikes >= STRIKES and time.monotonic() - last_restart >= MIN_GAP_S:
            log("restarting rosbridge (the launch respawns it; pages reconnect on their own)")
            subprocess.run(["pkill", "-f", "rosbridge_websocket"], timeout=5)
            last_restart, strikes = time.monotonic(), 0


if __name__ == "__main__":
    sys.exit(main())
