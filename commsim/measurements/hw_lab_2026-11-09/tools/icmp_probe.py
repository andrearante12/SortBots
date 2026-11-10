"""ICMP echo probe for Windows without admin (IcmpSendEcho), writing the same
CSV as commsim/hw/probe.py (seq, send_ns, recv_ns; recv empty = lost) so
`python -m commsim.hw.probe summary` can read it.

    python icmp_probe.py --to 10.42.0.253 --hz 100 --duration 60 --size 150 --out p.csv
"""
import argparse
import csv
import ctypes
import socket
import struct
import time
from ctypes import wintypes

iphlp = ctypes.WinDLL("iphlpapi")
iphlp.IcmpCreateFile.restype = wintypes.HANDLE
iphlp.IcmpSendEcho.argtypes = [wintypes.HANDLE, wintypes.DWORD, ctypes.c_void_p, wintypes.WORD,
                               ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD, wintypes.DWORD]
iphlp.IcmpSendEcho.restype = wintypes.DWORD
iphlp.IcmpCloseHandle.argtypes = [wintypes.HANDLE]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--to", required=True)
    ap.add_argument("--hz", type=float, default=100)
    ap.add_argument("--duration", type=float, default=60)
    ap.add_argument("--size", type=int, default=150)
    ap.add_argument("--timeout-ms", type=int, default=1000)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    h = iphlp.IcmpCreateFile()
    dst = struct.unpack("<I", socket.inet_aton(a.to))[0]
    payload = ctypes.create_string_buffer(b"s" * a.size, a.size)
    reply = ctypes.create_string_buffer(64 + a.size + 8)
    period = 1e9 / a.hz
    t0 = time.perf_counter_ns()
    n = int(a.duration * a.hz)
    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "send_ns", "recv_ns"])
        for seq in range(n):
            target = t0 + seq * period
            while time.perf_counter_ns() < target:
                pass
            s = time.perf_counter_ns()
            ok = iphlp.IcmpSendEcho(h, dst, payload, a.size, None, reply, len(reply), a.timeout_ms)
            r = time.perf_counter_ns()
            # ICMP_ECHO_REPLY.Status (offset 4) == 0 is IP_SUCCESS
            good = ok and struct.unpack_from("<I", reply.raw, 4)[0] == 0
            w.writerow([seq, s - t0, (r - t0) if good else ""])
    iphlp.IcmpCloseHandle(h)


if __name__ == "__main__":
    main()
