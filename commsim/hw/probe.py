"""UDP round-trip probe for the hardware testbed: latency, loss and the
longest outage (= reroute time when a link is pulled mid-run).

Standard library only, no ROS: runs on a Jetson, the operator laptop, or
anything else on the mesh subnet. Round trips, so no clock sync is needed.

    # far end (echoes every probe back to its sender)
    python3 -m commsim.hw.probe echo --bind 10.42.0.3
    # near end: 100 probes/s for 120 s
    python3 -m commsim.hw.probe ping --bind 10.42.0.2 --to 10.42.0.3 --hz 100 \\
        --duration 120 --out /tmp/probe.csv
    python3 -m commsim.hw.probe summary /tmp/probe.csv

Same idea as the emulation's reroute measurement (emu/reroute.py, a 10 ms
probe stream): at 100 Hz the outage is resolved to ~10 ms. Bound to the
mesh address, never 0.0.0.0 (would also answer on campus Wi-Fi).
"""
from __future__ import annotations

import argparse
import csv
import select
import socket
import struct
import sys
import time

PORT = 7460
PKT = struct.Struct("!IQ")  # seq, send time (ns, sender's clock)


def echo(bind: str, port: int) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((bind, port))
    print(f"echoing on {bind}:{port}", flush=True)
    while True:
        data, addr = s.recvfrom(2048)
        s.sendto(data, addr)


def ping(bind: str, to: str, port: int, hz: float, duration: float, size: int, out: str) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((bind, 0))
    s.setblocking(False)
    pad = b"\0" * max(0, size - PKT.size)
    sent: dict[int, int] = {}
    rows = []
    period = 1.0 / hz
    t0 = time.monotonic()
    nxt, seq = t0, 0
    while time.monotonic() - t0 < duration + 1.0:  # +1 s to collect stragglers
        now = time.monotonic()
        if now >= nxt and now - t0 < duration:
            ns = time.monotonic_ns()
            s.sendto(PKT.pack(seq, ns) + pad, (to, port))
            sent[seq] = ns
            seq += 1
            nxt += period
        # select, not sleep: a sleep-poll loop would add up to its tick to every RTT
        if select.select([s], [], [], max(0.0, min(0.05, nxt - time.monotonic())))[0]:
            data, _ = s.recvfrom(2048)
            r_seq, r_ns = PKT.unpack_from(data)
            rows.append((r_seq, r_ns, time.monotonic_ns()))
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "send_ns", "recv_ns"])  # recv_ns empty = lost
        got = {r[0]: r for r in rows}
        for q, ns in sent.items():
            w.writerow([q, ns, got[q][2] if q in got else ""])
    print(summary(out))


def summary(path: str) -> str:
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return "no probes"
    ok = [(int(r["send_ns"]), int(r["recv_ns"])) for r in rows if r["recv_ns"]]
    rtt = sorted((b - a) / 1e6 for a, b in ok)
    # Longest run of consecutive lost probes, as time between the last probe
    # answered before it and the first answered after it.
    gap, last_ok = 0.0, None
    for r in rows:
        if r["recv_ns"]:
            t = int(r["send_ns"])
            if last_ok is not None:
                gap = max(gap, (t - last_ok) / 1e9)
            last_ok = t
    def pct(p):
        return rtt[min(len(rtt) - 1, int(p * len(rtt)))] if rtt else float("nan")
    return (f"sent {len(rows)}  delivered {len(ok) / len(rows):.4f}  "
            f"rtt p50 {pct(0.5):.1f} ms  p95 {pct(0.95):.1f} ms  p99 {pct(0.99):.1f} ms  "
            f"longest outage {gap:.2f} s")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("echo")
    e.add_argument("--bind", required=True, help="this host's mesh IP")
    e.add_argument("--port", type=int, default=PORT)
    p = sub.add_parser("ping")
    p.add_argument("--bind", required=True, help="this host's mesh IP")
    p.add_argument("--to", required=True)
    p.add_argument("--port", type=int, default=PORT)
    p.add_argument("--hz", type=float, default=100.0)
    p.add_argument("--duration", type=float, default=60.0)
    p.add_argument("--size", type=int, default=150, help="bytes; 150 = one Status")
    p.add_argument("--out", required=True)
    s = sub.add_parser("summary")
    s.add_argument("csv")
    a = ap.parse_args(argv)
    if a.cmd == "echo":
        echo(a.bind, a.port)
    elif a.cmd == "ping":
        ping(a.bind, a.to, a.port, a.hz, a.duration, a.size, a.out)
    else:
        print(summary(a.csv))
    return 0


if __name__ == "__main__":
    sys.exit(main())
