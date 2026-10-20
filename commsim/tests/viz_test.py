"""commsim/viz: capture decoders and the trace builder, offline.

The decoders are checked two ways: against the byte builders in
viz/testdata/make_fixture.py (round trip) AND against a frame captured from
the real stack (so a shared misunderstanding of the wire format cannot pass).
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from commsim.viz import pcap as P
from commsim.viz.build_trace import build
from commsim.viz.testdata import make_fixture as M

HERE = Path(__file__).resolve().parent
FIXTURE = HERE.parent / "viz" / "testdata" / "trace_small.json"

# The UDP payload of a Status captured on the gateway's bat0, 2026-10-20
# (commsim/results/viz_smoke/gw_bat0.pcap, first `udp port 7450` frame):
# robot 0, seq 20, stamp 1792470384.715228208. Pins the CDR offsets to what
# rmw's serializer really emits, not to our own encoder.
REAL_STATUS_PAYLOAD = bytes.fromhex(
    "534200010000004000010000010000001400000070edd66a3084a12a0000000000000000"
    "0000000000000000000000005a71b3000100000000000000000048410000000000000000")


def test_real_status_payload_decodes():
    assert len(REAL_STATUS_PAYLOAD) == 72
    st = P.status_cdr(REAL_STATUS_PAYLOAD, 0, len(REAL_STATUS_PAYLOAD))
    assert st is not None
    assert (st.version, st.robot_id, st.seq) == (1, 0, 20)
    assert st.stamp_ns == 1792470384 * 10**9 + 715228208
    # our builder must agree on every header field it sets
    ours = M.status_payload(0, 20, st.stamp_ns)
    assert ours[:28] == REAL_STATUS_PAYLOAD[:28]
    assert P.status_cdr(b"\x00" * 72, 0, 72) is None


def test_builders_round_trip_through_the_decoders():
    rt = P.radiotap(M.radiotap_data(tsft_us=1234, rate_mbps=24.0))
    assert (rt.length, rt.tsft, rt.rate_mbps, rt.freq_mhz) == (22, 1234, 24.0, 2412)
    a = M.ack("02:00:00:00:05:00")
    rt = P.radiotap(a)
    assert rt.length == 14 and rt.rate_mbps == 0.0
    d = P.dot11(a[rt.length:])
    assert d.is_ack and d.addr1 == "02:00:00:00:05:00"

    frame = M.status_hop(2, 0, 2, 1, 7, 50, 1_700_000_002 * 10**9, 200)
    rt = P.radiotap(frame)
    f = frame[rt.length:]
    d = P.dot11(f)
    assert d.type == P.TYPE_DATA and not d.broadcast and not d.is_ack
    assert (d.addr1, d.addr2) == (M.radio_mac(1), M.radio_mac(2))
    assert d.mesh_ttl == 31 and d.ethertype == P.ETHERTYPE_BATMAN and d.payload == 46
    bm = P.batman(f, d.payload)
    assert (bm.name, bm.dest, bm.ttl) == ("unicast", M.radio_mac(0), 50)
    inner = P.ethernet(f, bm.inner)
    assert (inner.ethertype, inner.proto, inner.ip_src, inner.ip_dst, inner.dport) == (
        0x0800, 17, "10.42.0.3", "10.42.0.1", 7450)
    st = P.status_cdr(f, inner.l4_payload, inner.l4_len)
    assert (st.robot_id, st.seq) == (2, 7)

    ogm = M.dot11_bcast(M.radio_mac(1), 9, 31, 9, 0x4305, M.batman_ogm(M.radio_mac(1), 42, count=3))
    rt = P.radiotap(ogm)
    d = P.dot11(ogm[rt.length:])
    assert d.broadcast and d.payload == 40
    bm = P.batman(ogm[rt.length:], d.payload)
    assert (bm.name, bm.orig, bm.seqno, bm.ogm_count) == ("ogm", M.radio_mac(1), 42, 3)

    # a 64-byte snaplen leaves two bytes of batman header: still an OGM
    cut = ogm[:64]
    d = P.dot11(cut[22:])
    bm = P.batman(cut[22:], d.payload)
    assert bm.name == "ogm" and bm.truncated

    b = P.dot11(M.beacon(M.radio_mac(0))[22:])
    assert b.type == P.TYPE_MGMT and b.subtype == 8 and b.broadcast


def test_pcap_reader_rejects_pcapng_and_reads_records(tmp_path):
    p = tmp_path / "x.pcap"
    p.write_bytes(M.pcap([(1_700_000_000 * 10**9 + 5000, b"\x00" * 30)], 127))
    recs = list(P.records(p))
    assert len(recs) == 1 and recs[0].t_ns == 1_700_000_000 * 10**9 + 5000 and recs[0].incl == 30
    assert P.pcap_linktype(p) == 127
    bad = tmp_path / "x.pcapng"
    bad.write_bytes(P.PCAPNG_MAGIC + b"\x00" * 40)
    with pytest.raises(ValueError, match="pcapng"):
        list(P.records(bad))


def test_build_trace_from_the_synthetic_run(tmp_path):
    M.write_synthetic_dir(tmp_path)
    report = {}
    tr = build(tmp_path, name="syn", report=report)
    h, ev = tr["header"], tr["events"]
    assert tr["format"] == "commsim-viz/1"
    assert h["n"] == 3 and [nd["id"] for nd in h["nodes"]] == [0, 1, 2] and h["nodes"][0]["gateway"]
    assert h["layout"]["name"] == "synthetic" and h["nodes"][2]["pos"] == [10.0, 0.0]
    assert h["t0_ns"] == (M.T0_S + 1) * 10**9  # first hwsim0 record
    assert [e["t"] for e in ev] == sorted(e["t"] for e in ev)

    air = [e for e in ev if e["k"] == "air"]
    kinds = h["counts"]["air_by_kind"]
    assert kinds == {"ogm": 1, "status": 5, "icmp": 1, "mgmt": 1}
    ogm = next(e for e in air if e["kind"] == "ogm")
    assert ogm["ta"] == 1 and ogm["ra"] == -1 and ogm["ack"] is None and ogm["dur"] > 0

    # Status (2, 7) -> n0: two delivered hops. n1 and n0 have receiver
    # captures, so dur = receiver capture time - TX = 400 µs (the ACK record
    # 500 µs after TX is only the fallback for runs without rx captures).
    chain = [e for e in air if e["kind"] == "status" and e["seq"] == 7]
    assert [(e["ta"], e["ra"], e["hop"], e["dest"], e["src"], e["dst"]) for e in chain] == [
        (2, 1, 0, 0, 2, 0), (1, 0, 1, 0, 2, 0)]
    assert all(e["ack"] is True and e["dur"] == 400 and e["via"] == "rx" for e in chain)
    assert "ok" not in chain[0] and chain[1]["ok"] == "ok"
    # Status (2, 8): first hop never reached n1 -> lost, modelled duration
    lost = next(e for e in air if e["kind"] == "status" and e["seq"] == 8)
    assert lost["ack"] is False and lost["ok"] == "lost" and 100 < lost["dur"] < 1000
    assert report["rx_capture_nodes"] == [0, 1] and report["rx_joined"] == 4
    # the stub also sent every Status to n1 and nothing went on air: unroutable
    drops = [e for e in ev if e["k"] == "drop"]
    assert [(e["src"], e["seq"], e["dst"], e["why"]) for e in drops] == [
        (2, 7, 1, "unroutable"), (2, 8, 1, "unroutable"), (2, 9, 1, "unroutable")]
    assert report["outcomes"] == {"ok": 2, "lost": 1, "lost_after": 0, "unroutable": 3}
    assert report["ttl_chain_violations"] == 0

    icmp = next(e for e in air if e["kind"] == "icmp")
    assert (icmp["ta"], icmp["ra"], icmp["dest"], icmp["ack"], icmp["via"]) == (1, 2, 2, True, "ack")
    assert icmp["dur"] == 400  # ACK pairing: n2 has no rx capture
    # orphan ACKs are not counted when receiver captures exist (they would all be)
    assert report["ack_match"][1] == {"unicast": 1, "acked": 1, "orphan_acks": 0}

    tx = [e for e in ev if e["k"] == "app_tx"]
    rx = [e for e in ev if e["k"] == "app_rx"]
    assert [e["seq"] for e in tx] == [7, 8, 9] and [e["seq"] for e in rx] == [7, 9]
    assert rx[0]["lat_us"] == 3000 and h["counts"]["delivery"] == pytest.approx(2 / 6)

    routes = [e for e in ev if e["k"] == "route"]
    assert [(e["t"], e["node"], e["dst"], e["nh"], e["tq"]) for e in routes] == [
        (-500_000, 0, 1, 1, 255), (-500_000, 0, 2, 1, 191), (9_500_000, 0, 2, 2, 200)]
    ls = [e for e in ev if e["k"] == "linkstat"]
    assert [(e["t"], e["a"], e["b"], e["plink"], e["retries"], e["failed"]) for e in ls] == [
        (-500_000, 0, 1, "ESTAB", 0, 0), (9_500_000, 0, 1, "BLOCKED", 3, 1),
        (9_500_000, 0, 2, "ESTAB", 0, 0)]
    links = [e for e in ev if e["k"] == "link"]
    assert [(e["t"], e["a"], e["b"], e["action"]) for e in links] == [
        (8_000_000, 0, 1, "block"), (11_000_000, 0, 1, "unblock")]
    assert [e["text"] for e in ev if e["k"] == "mark"] == ["stubs_started"]

    # n0 heard n2 at 2.003 and 15.003: silent 6 s after the first, back at the second
    sil = [e for e in ev if e["k"] in ("robot_silent", "robot_back") and e["obs"] == 0]
    assert [(e["k"], e["t"]) for e in sil] == [("robot_silent", 7_003_000), ("robot_back", 14_003_000)]
    # n1 never heard n2 at all
    assert [(e["k"], e["t"], e["gap_us"]) for e in ev if e["k"] == "robot_silent" and e["obs"] == 1] == [
        ("robot_silent", 7_000_000, None)]

    # the guided tour: first delivery, the pull, the route that answers it, the restore, silence
    titles = [s["title"] for s in h["tour"]]
    assert titles[0] == "first Status delivered"
    assert "link the gateway–robot 1 pulled" in titles and "link the gateway–robot 1 restored" in titles
    assert "route moved" in titles and "the gateway declares robot 2 silent" in titles
    assert [s["t"] for s in h["tour"]] == sorted(s["t"] for s in h["tour"])
    moved = next(s for s in h["tour"] if s["title"] == "route moved")
    assert moved["t"] == 9_500_000 and "via robot 2" in moved["text"]


def test_ack_pairing_fallback_when_no_receiver_captures(tmp_path):
    M.write_synthetic_dir(tmp_path)
    for p in tmp_path.glob("rx_*.pcap"):
        p.unlink()
    report = {}
    tr = build(tmp_path, report=report)
    chain = [e for e in tr["events"] if e["k"] == "air" and e["kind"] == "status" and e["seq"] == 7]
    assert all(e["ack"] is True and e["dur"] == 500 and e["via"] == "ack" for e in chain)
    assert report["rx_capture_nodes"] == [] and report["rx_joined"] == 0
    assert report["ack_match"][2] == {"unicast": 3, "acked": 2, "orphan_acks": 0}


def test_build_trace_window_and_kinds_filter(tmp_path):
    M.write_synthetic_dir(tmp_path)
    tr = build(tmp_path, kinds={"status"}, window=(0.5, 3.0))
    air = [e for e in tr["events"] if e["k"] == "air"]
    assert {e["kind"] for e in air} == {"status"} and all(500_000 <= e["t"] <= 3_000_000 for e in air)


def test_committed_fixture_is_current(tmp_path):
    """`python3 -m commsim.viz.testdata.make_fixture` after changing the
    builders or the trace format, or the page's test replays stale data."""
    M.write_synthetic_dir(tmp_path)
    tr = build(tmp_path, name="trace_small")
    tr["header"]["built_at"] = "fixture"
    tr["header"]["source_dir"] = json.loads(FIXTURE.read_text())["header"]["source_dir"]
    assert json.loads(json.dumps(tr, sort_keys=True)) == json.loads(FIXTURE.read_text())
