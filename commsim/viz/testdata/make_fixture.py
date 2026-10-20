"""Byte builders for the frames the emulation puts on hwsim0, and a synthetic
run directory for the converter's tests and the committed page fixture.

The builders mirror commsim/viz/pcap.py's decoders field for field. They are
kept OUT of pcap.py so the decoders stay single-purpose — a decoder that is
tested only against its own encoder proves nothing, which is why
tests/viz_test.py also pins a frame captured from the real stack.

    /usr/bin/python3 -m commsim.viz.testdata.make_fixture        # rewrite trace_small.json
"""
from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
BCAST = "ff:ff:ff:ff:ff:ff"
T0_S = 1_700_000_000  # fixed epoch for the fixture: timestamps are relative anyway


def mac_bytes(m: str) -> bytes:
    return bytes(int(x, 16) for x in m.split(":"))


def radio_mac(i: int) -> str:
    return f"02:00:00:00:{i:02x}:00"


def ip_of(i: int) -> str:
    return f"10.42.0.{i + 1}"


def ip_bytes(ip: str) -> bytes:
    return bytes(int(x) for x in ip.split("."))


# ---- radiotap + 802.11 ---------------------------------------------------------

def radiotap_data(tsft_us: int = 0, rate_mbps: float = 12.0, freq: int = 2412) -> bytes:
    # present = TSFT | Flags | Rate | Channel, 22 bytes (as hwsim0 emits them)
    return struct.pack("<BBHI", 0, 0, 22, 0x0F) + struct.pack("<QBBHH", tsft_us, 0,
                                                              int(rate_mbps * 2), freq, 0x00C0)


def radiotap_ack(freq: int = 2412) -> bytes:
    # present = Flags | Channel, 14 bytes; no rate on ACK records
    return struct.pack("<BBHI", 0, 0, 14, 0x0A) + struct.pack("<BBHH", 0, 0, freq, 0x0080)


def ack(ra: str) -> bytes:
    return radiotap_ack() + struct.pack("<HH", 0x00D4, 0) + mac_bytes(ra)


def _llc(ethertype: int) -> bytes:
    return b"\xaa\xaa\x03\x00\x00\x00" + struct.pack(">H", ethertype)


def dot11_unicast(ra: str, ta: str, da: str, sa: str, seq: int, ttl: int, mseq: int,
                  ethertype: int, payload: bytes, rate_mbps: float = 12.0, tsft: int = 0) -> bytes:
    """4-address QoS data frame with 802.11s mesh control (FC 0x8803)."""
    hdr = (struct.pack("<HH", 0x0388, 0) + mac_bytes(ra) + mac_bytes(ta) + mac_bytes(da)
           + struct.pack("<H", seq << 4) + mac_bytes(sa) + b"\x01\x00"
           + bytes([0, ttl]) + struct.pack("<I", mseq))
    return radiotap_data(tsft, rate_mbps) + hdr + _llc(ethertype) + payload


def dot11_bcast(ta: str, seq: int, ttl: int, mseq: int, ethertype: int, payload: bytes,
                rate_mbps: float = 12.0, tsft: int = 0) -> bytes:
    """3-address broadcast QoS data frame (FC 0x8802), as OGMs are sent."""
    hdr = (struct.pack("<HH", 0x0288, 0) + mac_bytes(BCAST) + mac_bytes(ta) + mac_bytes(ta)
           + struct.pack("<H", seq << 4) + b"\x20\x01" + bytes([0, ttl]) + struct.pack("<I", mseq))
    return radiotap_data(tsft, rate_mbps) + hdr + _llc(ethertype) + payload


def beacon(ta: str, seq: int = 0, tsft: int = 0) -> bytes:
    hdr = (struct.pack("<HH", 0x0080, 0) + mac_bytes(BCAST) + mac_bytes(ta) + mac_bytes(ta)
           + struct.pack("<H", seq << 4))
    body = struct.pack("<QHH", tsft, 1000, 0x0401) + b"\x00\x00"  # timestamp, interval, cap, SSID(empty)
    return radiotap_data(tsft, 6.0) + hdr + body


def mgmt_action(ra: str, ta: str, seq: int = 0, tsft: int = 0) -> bytes:
    hdr = (struct.pack("<HH", 0x00D0, 0) + mac_bytes(ra) + mac_bytes(ta) + mac_bytes(ta)
           + struct.pack("<H", seq << 4))
    return radiotap_data(tsft, 12.0) + hdr + bytes([13, 1]) + b"\x00" * 20  # self-protected / peering


# ---- batman-adv IV --------------------------------------------------------------

def batman_ogm(orig: str, seqno: int, ttl: int = 50, tq: int = 255, count: int = 1) -> bytes:
    one = (bytes([0x00, 0x0F, ttl, 0]) + struct.pack(">I", seqno) + mac_bytes(orig)
           + mac_bytes(orig) + bytes([0, tq]) + struct.pack(">H", 0))
    return one * count


def batman_unicast(dest: str, ttl: int) -> bytes:
    return bytes([0x40, 0x0F, ttl, 0]) + mac_bytes(dest)


def batman_bcast(orig: str, seqno: int, ttl: int = 50) -> bytes:
    return bytes([0x01, 0x0F, ttl, 0]) + struct.pack(">I", seqno) + mac_bytes(orig)


# ---- Ethernet / IPv4 / UDP / ICMP ---------------------------------------------

def ether(dst: str, src: str, ethertype: int, payload: bytes) -> bytes:
    return mac_bytes(dst) + mac_bytes(src) + struct.pack(">H", ethertype) + payload


def _ipv4(src: str, dst: str, proto: int, payload: bytes) -> bytes:
    total = 20 + len(payload)
    hdr = struct.pack(">BBHHHBBH4s4s", 0x45, 0, total, 0, 0x4000, 64, proto, 0,
                      ip_bytes(src), ip_bytes(dst))
    return hdr + payload


def ipv4_udp(src: str, dst: str, sport: int, dport: int, payload: bytes) -> bytes:
    return _ipv4(src, dst, 17, struct.pack(">HHHH", sport, dport, 8 + len(payload), 0) + payload)


def ipv4_icmp_echo(src: str, dst: str, ident: int, seq: int) -> bytes:
    return _ipv4(src, dst, 1, struct.pack(">BBHHH", 8, 0, 0, ident, seq) + b"\x00" * 56)


def ipv4_tcp(src: str, dst: str, sport: int, dport: int, flags: int, payload: bytes) -> bytes:
    return _ipv4(src, dst, 6, struct.pack(">HHIIBBHHH", sport, dport, 1, 1, 5 << 4, flags, 1024,
                                          0, 0) + payload)


def arp_request(sender_mac: str, sender_ip: str, target_ip: str) -> bytes:
    return (struct.pack(">HHBBH", 1, 0x0800, 6, 4, 1) + mac_bytes(sender_mac) + ip_bytes(sender_ip)
            + b"\x00" * 6 + ip_bytes(target_ip))


# ---- UDP Status prototype ------------------------------------------------------

def fleet_payload(topic: int, robot_id: int, seq: int, stamp_ns: int, task_id: int = 0,
                  cost: float = 0.0) -> bytes:
    """A task-protocol message on the prototype wire: topic 2 TaskAnnounce,
    3 Bid, 4 Claim, 6 TaskEvent — FleetHeader then `uint32 task_id` at CDR
    offset 20 (Bid/Claim: `float32 cost` after it)."""
    cdr = (b"\x00\x01\x00\x00"
           + struct.pack("<BBxxIiI", 1, robot_id, seq, stamp_ns // 10**9, stamp_ns % 10**9)
           + struct.pack("<I", task_id))
    if topic in (3, 4):
        cdr += struct.pack("<f", cost)
    if topic == 4:
        cdr += struct.pack("<f", 10.0)
    if topic == 6:
        cdr += b"\x00\x00\x00\x00"
    return struct.pack("!HHI", 0x5342, topic, len(cdr)) + cdr


def fleet_hop(topic: int, src: int, dst: int, ta: int, ra: int, seq: int, ttl: int, stamp_ns: int,
              mseq: int, task_id: int, cost: float = 0.0) -> bytes:
    inner = ether("fe:ed:00:00:00:%02x" % dst, "fe:ed:00:00:00:%02x" % src, 0x0800,
                  ipv4_udp(ip_of(src), ip_of(dst), 7450, 7450,
                           fleet_payload(topic, src, seq, stamp_ns, task_id, cost)))
    return dot11_unicast(radio_mac(ra), radio_mac(ta), radio_mac(ra), radio_mac(ta), mseq, 31, mseq,
                         0x4305, batman_unicast(radio_mac(dst), ttl) + inner)


def status_payload(robot_id: int, seq: int, stamp_ns: int, battery: int = 90) -> bytes:
    """`struct.pack("!HHI", 0x5342, 1, len)` + the 64-byte CDR_LE Status the
    stub sends (one owned task, no path, no obstacles) — same field layout
    as the frame pinned in tests/viz_test.py."""
    cdr = (b"\x00\x01\x00\x00"
           + struct.pack("<BBxxIiI", 1, robot_id, seq, stamp_ns // 10**9, stamp_ns % 10**9)
           + struct.pack("<fffff", float(robot_id), 0.0, 0.0, 0.0, 0.0)
           + struct.pack("<BxxxIIfII", battery, 1, robot_id, 12.5, 0, 0))
    assert len(cdr) == 64, len(cdr)
    return struct.pack("!HHI", 0x5342, 1, len(cdr)) + cdr


# ---- pcap ---------------------------------------------------------------------

def pcap(frames: list[tuple[int, bytes]], linktype: int = 127) -> bytes:
    out = [struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 262144, linktype)]
    for t_ns, b in frames:
        out.append(struct.pack("<IIII", t_ns // 10**9, (t_ns % 10**9) // 1000, len(b), len(b)) + b)
    return b"".join(out)


# ---- the synthetic run ----------------------------------------------------------

def status_hop_payload(src: int, dst: int, seq: int, ttl: int, stamp_ns: int) -> bytes:
    """batman unicast header naming dst + the inner Ethernet/IPv4/UDP Status."""
    inner = ether("fe:ed:00:00:00:%02x" % dst, "fe:ed:00:00:00:%02x" % src, 0x0800,
                  ipv4_udp(ip_of(src), ip_of(dst), 7450, 7450, status_payload(src, seq, stamp_ns)))
    return batman_unicast(radio_mac(dst), ttl) + inner


def status_hop(src: int, dst: int, ta: int, ra: int, seq: int, ttl: int, stamp_ns: int,
               mseq: int, tsft: int = 0) -> bytes:
    """One hop of a Status (src -> dst) as hwsim0 sees it: ta -> ra, the
    batman header naming dst, the inner IP naming src/dst."""
    return dot11_unicast(radio_mac(ra), radio_mac(ta), radio_mac(ra), radio_mac(ta), mseq, 31, mseq,
                         0x4305, status_hop_payload(src, dst, seq, ttl, stamp_ns), tsft=tsft)


def status_hop_rx(src: int, dst: int, ta: int, ra: int, seq: int, ttl: int, stamp_ns: int) -> bytes:
    """The same hop as the RECEIVER's mesh interface logs it: the kernel's
    802.11 -> Ethernet conversion, src = transmitter, dst = this node."""
    return ether(radio_mac(ra), radio_mac(ta), 0x4305, status_hop_payload(src, dst, seq, ttl, stamp_ns))


def poll_block(t_ns: int, routes: list[tuple[str, float, int, str]],
               stations: list[dict]) -> str:
    """One poller sample in the exact text the VM produces (batctl 2024.0)."""
    lines = [f"--- {t_ns}",
             "[B.A.T.M.A.N. adv 2024.0, MainIF/MAC: wlan0/02:00:00:00:00:00 (bat0/c6:80:da:f2:37:39 BATMAN_IV)]",
             "   Originator        last-seen (#/255) Nexthop           [outgoingIF]"]
    for orig, seen, tq, nh in routes:
        lines.append(f" * {orig}    {seen:.3f}s   ({tq}) {nh} [     wlan0]")
    lines.append("+++")
    for s in stations:
        lines.append(f"Station {s['mac']} (on wlan0)")
        lines.append(f"\tmesh plink:\t{s.get('plink', 'ESTAB')}")
        lines.append(f"\ttx packets:\t{s.get('tx', 0)}")
        lines.append(f"\ttx retries:\t{s.get('retries', 0)}")
        lines.append(f"\ttx failed:\t{s.get('failed', 0)}")
        lines.append(f"\tsignal:  \t{s.get('signal', -66)} dBm")
    return "\n".join(lines) + "\n"


def write_synthetic_dir(out: Path) -> dict:
    """A 3-node line n0 - n1 - n2 (gateway n0), 20 s, UDP Status. What it
    contains, so the tests can name their expectations:

      t=1.000  OGM broadcast from n1
      t=2.000  Status (2, seq 7) -> n0: hop n2->n1 (ttl 50), hop n1->n0
               (ttl 49); each in the receiver's capture 400 µs after TX and
               ACKed on hwsim0 500 µs after; app_tx at 2.000, app_rx at n0
               2.003. No frames toward n1 at all -> 'unroutable' (2, 7, 1).
      t=2.333  Status (2, seq 8) -> n0: first hop n2->n1 in no capture and
               never ACKed -> 'lost'; app_tx 2.333, never received.
      t=3.000  ICMP echo n0 -> n2, second hop n1 -> n2 (ACKed). n2 has no
               rx capture, so this hop is judged by the ACK: via "ack"
      t=4.000  beacon from n0 (mgmt, no ack expected)
      t=9.000  link n0-n1 blocked (links.jsonl), t=12.000 unblocked
      t=15.000 Status (2, seq 9) -> n0 in two acked hops; app_rx 15.003
               => n0 heard n2 at 2.003 then 15.003: robot_silent (0, 2) at
                  8.003, robot_back at 15.003
      polls    n0 at 0.5 s: 1 via 1 (255), 2 via 1 (191); at 10.5 s: 2 via 2
    """
    out.mkdir(parents=True, exist_ok=True)
    # integer ns: (T0_S + s) * 1e9 is ~1.7e18, past float precision at the µs
    T = lambda s: T0_S * 10**9 + int(round(s * 1e6)) * 1000  # noqa: E731
    n = 3
    nodes = [{"ns": f"n{i}", "phy": f"phy{i}", "dev": f"wlan{i}", "ip": ip_of(i),
              "mac": radio_mac(i), "bat0_mac": "c6:80:da:f2:37:%02x" % i, "pos": [5.0 * i, 0.0]}
             for i in range(n)]
    snr = [[(25 if abs(i - j) == 1 else -10) if i != j else 0.0 for j in range(n)] for i in range(n)]
    (out / "mesh_state.json").write_text(json.dumps({
        "n": n, "interference": True, "wmediumd_pid": 1, "nodes": nodes, "snr": snr,
        "band": "g24", "topology": {"line": True, "links": None, "layout": "warehouse", "seed": 1,
                                    "orig_ms": 250}}))
    (out / "result.json").write_text(json.dumps({"n": n, "hz": 3.0, "transport": "udp",
                                                 "cap_seconds": 20.0, "depth": 32}))
    frames = [
        (T(1.000), dot11_bcast(radio_mac(1), 100, 31, 100, 0x4305, batman_ogm(radio_mac(1), 7))),
        (T(2.000000), status_hop(2, 0, 2, 1, 7, 50, T(2.0), 200)),
        (T(2.000500), ack(radio_mac(2))),
        (T(2.001000), status_hop(2, 0, 1, 0, 7, 49, T(2.0), 300)),
        (T(2.001500), ack(radio_mac(1))),
        (T(2.333000), status_hop(2, 0, 2, 1, 8, 50, T(2.333), 201)),
        (T(3.000000), dot11_unicast(radio_mac(2), radio_mac(1), radio_mac(2), radio_mac(1), 400, 31,
                                    400, 0x4305, batman_unicast(radio_mac(2), 49)
                                    + ether("fe:ed:00:00:00:02", "fe:ed:00:00:00:00", 0x0800,
                                            ipv4_icmp_echo(ip_of(0), ip_of(2), 7, 1)))),
        (T(3.000400), ack(radio_mac(1))),
        (T(4.000000), beacon(radio_mac(0), 5)),
        (T(15.000000), status_hop(2, 0, 2, 1, 9, 50, T(15.0), 202)),
        (T(15.000500), ack(radio_mac(2))),
        (T(15.001000), status_hop(2, 0, 1, 0, 9, 49, T(15.0), 301)),
        (T(15.001500), ack(radio_mac(1))),
    ]
    (out / "hwsim0.pcap").write_bytes(pcap(frames, 127))
    (out / "gw_bat0.pcap").write_bytes(pcap([], 1))
    # Receiver captures: every delivered hop, 400 µs after its TX (the ACK
    # record above is 500 µs after, so the tests can tell which oracle won).
    # n2 has none, so hops INTO n2 fall back to ACK pairing (the ICMP echo).
    (out / "rx_1.pcap").write_bytes(pcap([
        (T(2.000400), status_hop_rx(2, 0, 2, 1, 7, 50, T(2.0))),
        (T(15.000400), status_hop_rx(2, 0, 2, 1, 9, 50, T(15.0))),
    ], 1))
    (out / "rx_0.pcap").write_bytes(pcap([
        (T(2.001400), status_hop_rx(2, 0, 1, 0, 7, 49, T(2.0))),
        (T(15.001400), status_hop_rx(2, 0, 1, 0, 9, 49, T(15.0))),
    ], 1))
    for i in range(n):
        (out / f"sent_{i}.csv").write_text("src,seq,send_ns\n" + (
            f"2,7,{T(2.0)}\n2,8,{T(2.333)}\n2,9,{T(15.0)}\n" if i == 2 else ""))
        (out / f"stub_{i}.csv").write_text("src,dst,seq,send_ns,recv_ns\n" + (
            f"2,0,7,{T(2.0)},{T(2.003)}\n2,0,9,{T(15.0)},{T(15.003)}\n" if i == 0 else ""))
    (out / "poll_0.txt").write_text(
        poll_block(T(0.5), [(radio_mac(1), 0.059, 255, radio_mac(1)),
                            (radio_mac(2), 0.203, 191, radio_mac(1))],
                   [{"mac": radio_mac(1), "tx": 8}])
        + poll_block(T(10.5), [(radio_mac(1), 0.1, 255, radio_mac(1)),
                               (radio_mac(2), 0.1, 200, radio_mac(2))],
                     [{"mac": radio_mac(1), "tx": 20, "retries": 3, "failed": 1, "plink": "BLOCKED"},
                      {"mac": radio_mac(2), "tx": 4}]))
    for i in (1, 2):
        (out / f"poll_{i}.txt").write_text(poll_block(T(0.5), [], []))
    (out / "links.jsonl").write_text(
        json.dumps({"t_ns": T(9.0), "a": 0, "b": 1, "action": "block"}) + "\n"
        + json.dumps({"t_ns": T(12.0), "a": 0, "b": 1, "action": "unblock"}) + "\n")
    (out / "marks.jsonl").write_text(json.dumps({"t_ns": T(0.0), "text": "stubs_started"}) + "\n")
    return {"n": n, "t0_ns": T(1.0)}


def main(argv=None) -> int:
    import tempfile
    from commsim.viz.build_trace import build
    with tempfile.TemporaryDirectory() as d:
        src = Path(d) / "run"
        write_synthetic_dir(src)
        trace = build(src, name="trace_small")
    trace["header"]["built_at"] = "fixture"
    (HERE / "trace_small.json").write_text(json.dumps(trace, separators=(",", ":"), sort_keys=True))
    print(f"wrote {HERE / 'trace_small.json'}: {len(trace['events'])} events")
    return 0


if __name__ == "__main__":
    sys.exit(main())
