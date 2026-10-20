"""Turn one emulation run directory into a replay trace for commsim/viz.

    /usr/bin/python3 -m commsim.viz.build_trace commsim/results/<run> -o trace.json [--gz]
        [--window T0 T1] [--kinds status,zenoh,...] [--max-inflight-ms 200]
        [--pos "0:0,0 1:5,3 2:5,-3 3:10,0"] [--name NAME]

Inputs (written by commsim/emu/m1.py in the VM): mesh_state.json, hwsim0.pcap
(every over-the-air frame, full snaplen), sent_*.csv / stub_*.csv (the stubs'
own send/receive logs), poll_*.txt (batctl + station dump samples),
links.jsonl / block.json (link blocks), marks.jsonl, result.json.

Output: ONE JSON document {format, header, events}, events sorted by `t`
(integer µs since header.t0_ns). One document rather than JSONL because the
page needs the header before it can draw anything, scrubs rather than
streams, and one URL / one dropped file keeps `?trace=` and drag-drop the
same code path. The format is documented in commsim/viz/README.md.

What is inferred, and how (every one of these is labelled best effort):

- Delivery of a hop (`ack`, `dur`, `via`): the receiver's own capture is the
  oracle when the run has one (rx_<i>.pcap, m1.py: tcpdump on the mesh
  interface inside each namespace, `ether dst <its radio MAC>`): a unicast
  frame that reached node i is in it, Ethernet-framed with the transmitter
  as source, so (ta, ra, batman dest, inner IP, seq) names the hop exactly
  and `dur` = receiver time - hwsim0 TX time (`via: rx`). Without it the
  fallback is hwsim0's ACK records (RA = the data frame's TA, nothing else),
  paired per-transmitter FIFO (`via: ack`) — best effort: the n-1 copies of
  one Status are queued within ~20 µs and one lost copy shifts the pairing
  until a gap longer than --max-inflight-ms drains the queue (on the first
  n=10 smoke run that put the ACK-TX median at 10 ms instead of ~0.5 ms).
  Either way a frame no receiver saw and no ACK closed is `ack: false`.
- Hop chains: UDP Status carries (robot_id, seq) in clear in every hop, and
  the inner IPv4 names the destination, so (src, seq, dst) groups the hops;
  batman's ttl (50 at the first hop) gives the hop index.
- Outcomes: app_rx present -> ok; last hop unacked -> lost; acked but never
  received -> lost_after; no frame at all for a destination the stub sent to
  -> unroutable (its sendto() failed, or batman dropped it mid-reroute).
- Silence: a gap > status_timeout_s between receptions at an observer is
  what the Allocator would call robot_silent; the stubs do not run the
  Allocator, so it is derived here (events named as in core/protocol.py).
"""
from __future__ import annotations

import argparse
import bisect
import collections
import csv
import datetime as dt
import gzip
import json
import re
import struct
import sys
from pathlib import Path
from typing import Optional

from commsim.core.fleet import load_config
from commsim.des import radio as R
from commsim.des.layout import LAYOUTS
from commsim.viz import pcap as P

REPO = Path(__file__).resolve().parents[2]
BASE_CFG = REPO / "commsim/configs/base.yaml"
FORMAT = "commsim-viz/1"
GROUP = -1          # `ra` of a broadcast / multicast frame
KINDS = ("status", "zenoh", "tcp_ack", "icmp", "ogm", "bcast", "arp", "mgmt", "frag", "other")
_MAC_RE = re.compile(r"([0-9a-f]{2}(?::[0-9a-f]{2}){5})")


# ---- inputs ---------------------------------------------------------------------

def load_state(d: Path) -> dict:
    st = json.loads((d / "mesh_state.json").read_text())
    for i, nd in enumerate(st["nodes"]):
        nd.setdefault("id", i)
    return st


def layout_for(st: dict, pos_override: Optional[dict]) -> tuple[dict, list]:
    """Floor geometry for the header and the node positions to draw. A
    --line / --links run has no floor: a synthetic strip (or --pos)."""
    topo = st.get("topology") or {}
    pos = [list(nd["pos"]) for nd in st["nodes"]]
    if pos_override:
        for i, p in pos_override.items():
            if i < len(pos):
                pos[i] = list(p)
    synthetic = bool(topo.get("line") or topo.get("links") or pos_override)
    name = topo.get("layout", "warehouse")
    if not synthetic and name in LAYOUTS:
        lay = LAYOUTS[name]()
        geo = {"name": name, "width": lay.width, "height": lay.height, "racks": lay.racks,
               "shelves": lay.shelves, "aisles_y": lay.aisles_y, "cross_x": list(lay.cross_x),
               "gateway_pos": list(lay.gateway_pos)}
    else:
        xs = [p[0] for p in pos] or [0.0]
        ys = [p[1] for p in pos] or [0.0]
        geo = {"name": "synthetic", "width": max(xs) - min(xs) + 10.0,
               "height": max(ys) - min(ys) + 10.0, "racks": [], "shelves": [], "aisles_y": [],
               "cross_x": [], "gateway_pos": pos[0] if pos else [0.0, 0.0],
               "origin": [min(xs) - 5.0, min(ys) - 5.0]}
    return geo, pos


def parse_pos(s: Optional[str]) -> Optional[dict]:
    if not s:
        return None
    out = {}
    for tok in s.split():
        i, xy = tok.split(":")
        x, y = xy.split(",")
        out[int(i)] = (float(x), float(y))
    return out


def read_csvs(d: Path, t0: int) -> tuple[list, list]:
    tx, rx = [], []
    for f in sorted(d.glob("sent_*.csv")):
        for r in csv.DictReader(open(f)):
            tx.append({"t": (int(r["send_ns"]) - t0) // 1000, "k": "app_tx",
                       "src": int(r["src"]), "seq": int(r["seq"])})
    for f in sorted(d.glob("stub_*.csv")):
        for r in csv.DictReader(open(f)):
            rx.append({"t": (int(r["recv_ns"]) - t0) // 1000, "k": "app_rx",
                       "src": int(r["src"]), "dst": int(r["dst"]), "seq": int(r["seq"]),
                       "lat_us": (int(r["recv_ns"]) - int(r["send_ns"])) // 1000})
    return tx, rx


def read_polls(d: Path, t0: int, by_mac: dict) -> list:
    """poll_<i>.txt: blocks of `--- <ns>` / batctl o / `+++` / station dump.
    Emits route and linkstat DELTAS (the first sample as deltas from
    nothing), so a 0.5 s poller over 120 s is a few hundred events, not
    thousands of identical rows."""
    ev = []
    for f in sorted(d.glob("poll_*.txt")):
        node = int(f.stem.split("_")[1])
        routes, stats = {}, {}
        for block in f.read_text().split("--- ")[1:]:
            head, _, body = block.partition("\n")
            try:
                t = (int(head.strip()) - t0) // 1000
            except ValueError:
                continue
            otab, _, sdump = body.partition("+++")
            seen = {}
            for line in otab.splitlines():
                if not line.startswith(" *"):
                    continue  # header lines, or non-best alternatives
                macs = _MAC_RE.findall(line)
                tq = re.search(r"\((\s*\d+)\)", line)
                if len(macs) >= 2 and macs[0] in by_mac and macs[1] in by_mac:
                    seen[by_mac[macs[0]]] = (by_mac[macs[1]], int(tq.group(1)) if tq else 0)
            for dst, (nh, tq) in seen.items():
                if routes.get(dst) != (nh, tq):
                    ev.append({"t": t, "k": "route", "node": node, "dst": dst, "nh": nh, "tq": tq})
            for dst in [x for x in routes if x not in seen]:
                ev.append({"t": t, "k": "route", "node": node, "dst": dst, "nh": -1, "tq": 0})
            routes = seen
            cur = None
            now_stats = {}
            for line in sdump.splitlines():
                m = re.match(r"Station (\S+)", line)
                if m:
                    cur = by_mac.get(m.group(1))
                    if cur is not None:
                        now_stats[cur] = {}
                    continue
                if cur is None or ":" not in line:
                    continue
                key, _, val = line.strip().partition(":")
                val = val.strip()
                key = key.strip()
                if key == "mesh plink":
                    now_stats[cur]["plink"] = val
                elif key in ("tx packets", "tx retries", "tx failed"):
                    now_stats[cur][key.split()[1]] = int(val or 0)
                elif key == "signal":
                    try:
                        now_stats[cur]["signal"] = int(val.split()[0])
                    except (ValueError, IndexError):
                        pass
            for peer, s in now_stats.items():
                prev = stats.get(peer, {})
                if s != prev:
                    e = {"t": t, "k": "linkstat", "a": node, "b": peer,
                         "plink": s.get("plink"), "signal": s.get("signal"),
                         "tx": s.get("packets", 0) - prev.get("packets", 0),
                         "retries": s.get("retries", 0) - prev.get("retries", 0),
                         "failed": s.get("failed", 0) - prev.get("failed", 0)}
                    ev.append(e)
            stats = now_stats
    return ev


def read_links(d: Path, t0: int, st: dict) -> list:
    ev = []
    if (d / "links.jsonl").exists():
        for line in (d / "links.jsonl").read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                ev.append({"t": (int(r["t_ns"]) - t0) // 1000, "k": "link", "a": int(r["a"]),
                           "b": int(r["b"]), "action": r["action"]})
    elif (d / "block.json").exists():  # runs before links.jsonl: one block, n0 to its relay
        r = json.loads((d / "block.json").read_text())
        relay = next((nd["id"] for nd in st["nodes"] if nd["ns"] == r.get("relay")), None)
        if relay is not None:
            ev.append({"t": (int(r["t_ns"]) - t0) // 1000, "k": "link", "a": 0, "b": relay,
                       "action": "block"})
    if (d / "marks.jsonl").exists():
        for line in (d / "marks.jsonl").read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                ev.append({"t": (int(r["t_ns"]) - t0) // 1000, "k": "mark", "text": r["text"]})
    return ev


# ---- air frames -------------------------------------------------------------------

def classify(d: P.Dot11, f: bytes) -> tuple[str, dict]:
    """Kind and the decoded fields of one non-ACK frame."""
    x: dict = {}
    if d.type == P.TYPE_MGMT:
        return "mgmt", {"sub": d.subtype}
    if d.type != P.TYPE_DATA or d.ethertype is None:
        return "other", x
    if d.ethertype != P.ETHERTYPE_BATMAN:
        return "other", {"ethertype": d.ethertype}
    bm = P.batman(f, d.payload)
    if bm.ptype in (P.BAT_IV_OGM, P.BAT_OGM2):
        return "ogm", {"n_ogm": bm.ogm_count}
    if bm.truncated:
        return "other", {"bat": bm.name, "trunc": True}
    if bm.ptype == P.BAT_BCAST:
        inner = P.ethernet(f, bm.inner)
        return ("arp" if inner.ethertype == P.ETHERTYPE_ARP else "bcast"), {"orig_mac": bm.orig}
    if bm.ptype == P.BAT_UNICAST_FRAG:
        return "frag", {"dest_mac": bm.dest, "ttl": bm.ttl}
    if bm.ptype == P.BAT_ICMP:
        return "icmp", {"dest_mac": bm.dest, "ttl": bm.ttl, "seq": bm.seqno}
    if bm.ptype not in (P.BAT_UNICAST, P.BAT_UNICAST_4ADDR):
        return "other", {"bat": bm.name}
    x = {"dest_mac": bm.dest, "ttl": bm.ttl}
    inner = P.ethernet(f, bm.inner)
    if inner.ethertype != P.ETHERTYPE_IPV4 or inner.truncated:
        return ("arp" if inner.ethertype == P.ETHERTYPE_ARP else "other"), x
    x["ip_src"], x["ip_dst"] = inner.ip_src, inner.ip_dst
    if inner.proto == 17 and inner.dport == P.STATUS_UDP_PORT:
        st = P.status_cdr(f, inner.l4_payload, inner.l4_len)
        if st is not None:
            x["src"], x["seq"] = st.robot_id, st.seq
        return "status", x
    if inner.proto == 6 and P.ZENOH_TCP_PORT in (inner.sport, inner.dport):
        x["l4"] = (inner.sport, inner.dport, inner.tcp_seq, inner.l4_len)
        return ("zenoh" if inner.l4_len > 0 else "tcp_ack"), x
    if inner.proto == 1:
        x["seq"] = inner.icmp_seq
        return "icmp", x
    return "other", {**x, "proto": inner.proto}


def hop_key(kind: str, ta_mac: str, ra_mac: str, x: dict) -> Optional[tuple]:
    """What identifies one hop of one packet in both the air and a receiver
    capture. None for kinds without a usable identity (OGMs, mgmt, frags)."""
    if kind == "status" and "seq" in x:
        return ("s", ta_mac, ra_mac, x.get("src"), x["seq"], x.get("ttl"))
    if kind == "icmp" and "ip_src" in x:
        return ("i", ta_mac, ra_mac, x["ip_src"], x["ip_dst"], x.get("seq"), x.get("ttl"))
    if kind in ("zenoh", "tcp_ack") and "l4" in x:
        return ("t", ta_mac, ra_mac, x["ip_src"], x["ip_dst"], x["l4"], x.get("ttl"))
    return None


def read_rx(d: Path, t0: int, by_ip: dict) -> tuple[dict, list]:
    """rx_<i>.pcap -> {hop key: t_rx_us}. Ethernet-framed (the kernel's
    802.11 -> 802.3 conversion on the mesh interface): src = transmitter,
    dst = this node, then batman + inner exactly as on air."""
    seen: dict = {}
    nodes = []
    for f in sorted(d.glob("rx_*.pcap")):
        try:
            node = int(f.stem.split("_")[1])
            if P.pcap_linktype(f) != P.LINKTYPE_ETHERNET:
                continue
        except (ValueError, IndexError):
            continue
        nodes.append(node)
        for rec in P.records(f):
            b = rec.data
            if len(b) < 16 or struct.unpack(">H", b[12:14])[0] != P.ETHERTYPE_BATMAN:
                continue
            ta_mac, ra_mac = P.mac(b[6:12]), P.mac(b[0:6])
            bm = P.batman(b, 14)
            if bm.truncated or bm.ptype not in (P.BAT_UNICAST, P.BAT_UNICAST_4ADDR, P.BAT_ICMP):
                continue
            x: dict = {"ttl": bm.ttl}
            if bm.ptype == P.BAT_ICMP:
                kind = "icmp"
                x.update({"ip_src": bm.orig, "ip_dst": bm.dest, "seq": bm.seqno})
            else:
                inner = P.ethernet(b, bm.inner)
                if inner.ethertype != P.ETHERTYPE_IPV4 or inner.truncated:
                    continue
                x["ip_src"], x["ip_dst"] = inner.ip_src, inner.ip_dst
                if inner.proto == 17 and inner.dport == P.STATUS_UDP_PORT:
                    st = P.status_cdr(b, inner.l4_payload, inner.l4_len)
                    if st is None:
                        continue
                    kind = "status"
                    x["src"], x["seq"] = st.robot_id, st.seq
                elif inner.proto == 6:
                    kind = "zenoh" if inner.l4_len > 0 else "tcp_ack"
                    x["l4"] = (inner.sport, inner.dport, inner.tcp_seq, inner.l4_len)
                elif inner.proto == 1:
                    kind = "icmp"
                    x["seq"] = inner.icmp_seq
                else:
                    continue
            key = hop_key(kind, ta_mac, ra_mac, x)
            if key is not None and key not in seen:
                seen[key] = (rec.t_ns - t0) // 1000
    return seen, sorted(nodes)


def read_air(path: Path, t0: int, by_mac: dict, by_ip: dict, max_inflight_us: int,
             report: dict, rx_seen: Optional[dict] = None, rx_nodes: Optional[list] = None) -> list:
    ev: list = []
    rx_seen = rx_seen or {}
    rx_nodes = set(rx_nodes or [])
    joined = 0
    open_q: dict = collections.defaultdict(collections.deque)  # ta -> deque of air events
    per_node = collections.defaultdict(lambda: [0, 0, 0])      # ta -> [unicast tx, acks, orphan acks]

    def expire(now: int) -> None:
        for q in open_q.values():
            while q and now - q[0]["t"] > max_inflight_us:
                e = q.popleft()
                e["ack"] = False
                e["dur"] = model_dur(e, unicast=True)

    def model_dur(e: dict, unicast: bool) -> int:
        return int(R.frame_airtime_us(e["size"], e["rate"] or 12.0, "g24", unicast=unicast,
                                      legacy=True))

    for rec in P.records(path):
        rt = P.radiotap(rec.data)
        f = rec.data[rt.length:]
        if len(f) < 10:
            continue
        d = P.dot11(f)
        t = (rec.t_ns - t0) // 1000
        if d.is_ack:
            # Expire BEFORE matching: a lost frame left at the head of the
            # queue would otherwise take every later ACK and shift the
            # pairing for the rest of the run (seen on the first smoke run:
            # ACK-TX p50 149 ms instead of ~0.5 ms).
            expire(t)
            ta = by_mac.get(d.addr1)
            q = open_q.get(d.addr1)
            if q and q[0]["t"] < t:
                e = q.popleft()
                e["ack"] = True
                e["dur"] = max(1, t - e["t"])
                if ta is not None:
                    per_node[ta][1] += 1
            elif ta is not None and not rx_nodes:
                # only meaningful without receiver captures: with them, hops
                # into covered nodes never enter the queue, so every ACK for
                # them would count here and say nothing
                per_node[ta][2] += 1
            continue
        expire(t)
        kind, x = classify(d, f)
        e = {"t": t, "k": "air", "ta": by_mac.get(d.addr2, -2), "ra": GROUP if d.broadcast
             else by_mac.get(d.addr1, -2), "kind": kind, "size": rec.orig - rt.length,
             "rate": rt.rate_mbps, "mseq": d.seq}
        if d.mesh_ttl is not None:
            e["mttl"] = d.mesh_ttl
        if "ttl" in x:
            e["ttl"] = x["ttl"]
            e["hop"] = P.BATADV_TTL - x["ttl"]
        if "dest_mac" in x:
            e["dest"] = by_mac.get(x["dest_mac"], -2)
        if "ip_src" in x:
            e["src"] = by_ip.get(x["ip_src"], x.get("src", -2))
            e["dst"] = by_ip.get(x["ip_dst"], -2)
        if "src" in x and "src" not in e:
            e["src"] = x["src"]
        if "seq" in x:
            e["seq"] = x["seq"]
        if x.get("trunc"):
            report["truncated"] = report.get("truncated", 0) + 1
        ev.append(e)
        if e["ra"] == GROUP:
            e["ack"] = None
            e["dur"] = model_dur(e, unicast=False)
            continue
        key = hop_key(kind, d.addr2, d.addr1, {**x, "ttl": x.get("ttl")})
        if e["ra"] in rx_nodes and key is not None:
            # the receiver's capture decides; a frame it never logged was lost
            e["via"] = "rx"
            t_rx = rx_seen.get(key)
            if t_rx is not None and t_rx >= t:
                e["ack"], e["dur"] = True, max(1, t_rx - t)
                joined += 1
            else:
                e["ack"], e["dur"] = False, model_dur(e, unicast=True)
            continue
        e["via"] = "ack"
        open_q[d.addr2].append(e)
        if e["ta"] >= 0:
            per_node[e["ta"]][0] += 1
    expire(ev[-1]["t"] + max_inflight_us + 1 if ev else 0)
    report["ack_match"] = {n: {"unicast": c[0], "acked": c[1], "orphan_acks": c[2]}
                          for n, c in sorted(per_node.items())}
    report["rx_joined"] = joined
    return ev


# ---- derivations ---------------------------------------------------------------

def join_delivery(air: list, tx: list, rx: list, n: int, report: dict) -> list:
    """Outcome of every (src, seq, dst) Status, written onto its last hop's
    `ok`; destinations with no frame at all become `drop` events."""
    chains: dict = collections.defaultdict(list)
    for e in air:
        if e["kind"] == "status" and "seq" in e and e.get("dst", -2) >= 0 and e.get("src", -2) >= 0:
            chains[(e["src"], e["seq"], e["dst"])].append(e)
    got = {(r["src"], r["seq"], r["dst"]) for r in rx}
    ttl_bad = 0
    out = {"ok": 0, "lost": 0, "lost_after": 0, "unroutable": 0}
    drops = []
    for key, hops in chains.items():
        hops.sort(key=lambda e: e["t"])
        ttls = [h.get("ttl", 0) for h in hops]
        if any(b >= a for a, b in zip(ttls, ttls[1:])):
            ttl_bad += 1
        last = hops[-1]
        if key in got:
            last["ok"] = "ok"
        elif last.get("ack") is False:
            last["ok"] = "lost"
        else:
            last["ok"] = "lost_after"
        out[last["ok"]] += 1
    if tx:
        for e in tx:
            for dst in range(n):
                if dst == e["src"]:
                    continue
                key = (e["src"], e["seq"], dst)
                if key not in chains and key not in got:
                    drops.append({"t": e["t"], "k": "drop", "src": e["src"], "dst": dst,
                                  "seq": e["seq"], "why": "unroutable"})
                    out["unroutable"] += 1
    report["outcomes"] = out
    report["ttl_chain_violations"] = ttl_bad
    report["chains"] = len(chains)
    report["app_rx_matched"] = sum(1 for k in got if k in chains)
    return drops


def derive_silence(tx: list, rx: list, n: int, timeout_us: int) -> list:
    ev = []
    heard: dict = collections.defaultdict(list)
    for r in rx:
        heard[(r["dst"], r["src"])].append(r["t"])
    first_tx: dict = {}
    for e in tx:
        first_tx[e["src"]] = min(first_tx.get(e["src"], e["t"]), e["t"])
    for obs in range(n):
        for peer in range(n):
            if obs == peer or peer not in first_tx:
                continue
            ts = sorted(heard.get((obs, peer), []))
            if not ts:
                ev.append({"t": first_tx[peer] + timeout_us, "k": "robot_silent", "obs": obs,
                           "peer": peer, "gap_us": None})
                continue
            if ts[0] - first_tx[peer] > timeout_us:
                ev.append({"t": first_tx[peer] + timeout_us, "k": "robot_silent", "obs": obs,
                           "peer": peer, "gap_us": ts[0] - first_tx[peer]})
                ev.append({"t": ts[0], "k": "robot_back", "obs": obs, "peer": peer})
            for a, b in zip(ts, ts[1:]):
                if b - a > timeout_us:
                    ev.append({"t": a + timeout_us, "k": "robot_silent", "obs": obs, "peer": peer,
                               "gap_us": b - a})
                    ev.append({"t": b, "k": "robot_back", "obs": obs, "peer": peer})
    return ev


# ---- build ------------------------------------------------------------------------

def build(d: Path, *, name: Optional[str] = None, max_inflight_ms: float = 200.0,
          pos: Optional[dict] = None, window: Optional[tuple] = None,
          kinds: Optional[set] = None, report: Optional[dict] = None) -> dict:
    report = report if report is not None else {}
    st = load_state(d)
    n = int(st["n"])
    by_mac = {nd["mac"]: nd["id"] for nd in st["nodes"]}
    by_mac.update({nd["bat0_mac"]: nd["id"] for nd in st["nodes"] if nd.get("bat0_mac")})
    by_ip = {nd["ip"]: nd["id"] for nd in st["nodes"]}
    cfg = load_config(BASE_CFG)
    res = json.loads((d / "result.json").read_text()) if (d / "result.json").exists() else {}
    tmo_s = float(cfg["protocol"]["status_timeout_s"])
    air_path = d / "hwsim0.pcap"
    t0 = next(iter(P.records(air_path))).t_ns if air_path.exists() else None
    tx, rx = read_csvs(d, t0 or 0)
    if t0 is None:
        t0 = min([e["t"] for e in tx + rx] or [0])
        tx, rx = read_csvs(d, t0)
    rx_seen, rx_nodes = read_rx(d, t0, by_ip)
    report["rx_capture_nodes"] = rx_nodes
    air = read_air(air_path, t0, by_mac, by_ip, int(max_inflight_ms * 1000), report,
                   rx_seen, rx_nodes) if air_path.exists() else []
    drops = join_delivery(air, tx, rx, n, report)
    silence = derive_silence(tx, rx, n, int(tmo_s * 1e6))
    polls = read_polls(d, t0, by_mac)
    links = read_links(d, t0, st)
    events = air + tx + rx + drops + silence + polls + links
    if kinds:
        events = [e for e in events if e["k"] != "air" or e["kind"] in kinds]
    if window:
        lo, hi = int(window[0] * 1e6), int(window[1] * 1e6)
        events = [e for e in events if lo <= e["t"] <= hi]
    events.sort(key=lambda e: (e["t"], e["k"] != "air"))
    geo, positions = layout_for(st, pos)
    topo = st.get("topology") or {}
    counts = collections.Counter(e["kind"] for e in events if e["k"] == "air")
    n_tx, n_rx = len(tx), len(rx)
    header = {
        "name": name or d.name, "n": n, "transport": res.get("transport", "udp"),
        "hz": res.get("hz"), "orig_ms": topo.get("orig_ms"),
        "duration_s": res.get("cap_seconds"), "t0_ns": t0,
        "built_at": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
        "source_dir": str(d),
        "layout": geo,
        "nodes": [{"id": nd["id"], "ns": nd["ns"], "ip": nd["ip"], "mac": nd["mac"],
                   "bat0_mac": nd.get("bat0_mac"), "pos": positions[nd["id"]],
                   "gateway": nd["id"] == 0} for nd in st["nodes"]],
        "snr": st["snr"],
        "knobs": {"status_timeout_s": tmo_s, "status_hz": res.get("hz", cfg["protocol"]["status_hz"]),
                  "orig_interval_ms": topo.get("orig_ms", cfg["batman"]["orig_interval_ms"]),
                  "zenoh_lease_ms": cfg["zenoh"]["transport_lease_ms"],
                  "batadv_ttl": P.BATADV_TTL, "snr_link_min_db": 4.0,
                  "max_inflight_ms": max_inflight_ms},
        "counts": {"air_by_kind": dict(counts), "app_tx": n_tx, "app_rx": n_rx,
                   "unacked": sum(1 for e in air if e.get("ack") is False),
                   "outcomes": report.get("outcomes", {}),
                   "delivery": (n_rx / (n_tx * (n - 1))) if n_tx and n > 1 else None},
        "limits": [
            "only Status stubs ran: no TaskAnnounce/Bid/Claim/LeaseBeacon on the wire",
            ("hop delivery from the receivers' own captures (rx_<i>.pcap) on nodes "
             f"{rx_nodes}; other hops from hwsim0 ACK pairing, best effort: no ACK within "
             f"{max_inflight_ms:g} ms = lost") if rx_nodes else
            ("no receiver captures in this run: hop delivery is hwsim0 ACK pairing "
             f"(per-transmitter FIFO, best effort; no ACK within {max_inflight_ms:g} ms = lost)"),
            "retries happen inside wmediumd and are invisible",
            "`rate` is the kernel's requested rate at handoff, not what wmediumd used; 0 = unknown",
            "zenoh (TCP) payloads are opaque: hops carry IPs only, no (src, seq) join",
            "`dur` of an acked frame includes wmediumd queueing and backoff",
            "robot_silent/robot_back are derived from the stubs' receive logs with "
            f"status_timeout_s = {tmo_s:g}, not observed from an Allocator",
        ],
    }
    if (d / "hwsim0.pcap").exists() and report.get("truncated"):
        header["limits"].append(f"{report['truncated']} frames were truncated by the capture "
                                "snaplen: their payloads could not be decoded")
    return {"format": FORMAT, "header": header, "events": events}


def print_report(trace: dict, report: dict) -> None:
    h = trace["header"]
    print(f"{h['name']}: n={h['n']} {h['transport']} events={len(trace['events'])}", file=sys.stderr)
    print(f"  air by kind: {h['counts']['air_by_kind']}", file=sys.stderr)
    print(f"  app_tx={h['counts']['app_tx']} app_rx={h['counts']['app_rx']} "
          f"delivery={h['counts']['delivery']}", file=sys.stderr)
    print(f"  status chains={report.get('chains')} outcomes={report.get('outcomes')} "
          f"ttl violations={report.get('ttl_chain_violations')} "
          f"app_rx matched to a chain={report.get('app_rx_matched')}", file=sys.stderr)
    print(f"  receiver captures on nodes {report.get('rx_capture_nodes')}: "
          f"{report.get('rx_joined', 0)} hops joined", file=sys.stderr)
    for node, c in (report.get("ack_match") or {}).items():
        pct = 100.0 * c["acked"] / c["unicast"] if c["unicast"] else 0.0
        print(f"  node {node}: unicast {c['unicast']} acked {c['acked']} ({pct:.1f} %) "
              f"orphan acks {c['orphan_acks']}", file=sys.stderr)
    if report.get("truncated"):
        print(f"  truncated frames: {report['truncated']} (snaplen)", file=sys.stderr)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=None, help="default: <run_dir>/trace.json")
    ap.add_argument("--gz", action="store_true", help="gzip the output (the page inflates it)")
    ap.add_argument("--name", default=None)
    ap.add_argument("--window", nargs=2, type=float, metavar=("T0", "T1"),
                    help="keep events between T0 and T1 seconds into the capture")
    ap.add_argument("--kinds", default=None, help=f"comma list of air kinds to keep: {','.join(KINDS)}")
    ap.add_argument("--max-inflight-ms", type=float, default=200.0,
                    help="a unicast frame with no ACK by then is lost (observed max 16 ms)")
    ap.add_argument("--pos", default=None, help='node positions for --links runs, e.g. "0:0,0 1:5,3"')
    a = ap.parse_args(argv)
    report: dict = {}
    trace = build(a.run_dir, name=a.name, max_inflight_ms=a.max_inflight_ms, pos=parse_pos(a.pos),
                  window=tuple(a.window) if a.window else None,
                  kinds=set(a.kinds.split(",")) if a.kinds else None, report=report)
    out = a.out or (a.run_dir / ("trace.json.gz" if a.gz else "trace.json"))
    data = json.dumps(trace, separators=(",", ":")).encode()
    if a.gz or str(out).endswith(".gz"):
        data = gzip.compress(data)
    out.write_bytes(data)
    print_report(trace, report)
    print(f"  wrote {out} ({len(data) / 1e6:.1f} MB)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
