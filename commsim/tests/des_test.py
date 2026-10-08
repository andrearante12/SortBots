"""Sanity checks of the DES medium against textbook 802.11 behavior."""
import math

import pytest

from commsim.core.fleet import load_config
from commsim.des import radio as R
from commsim.des.sim import Sim, Frame, BG_AIRTIME_S
from commsim.tests.fleet_test import BASE


def test_airtime_arithmetic():
    # 100 B at HT MCS0 (6.5 Mbps): 36 µs preamble + 32 symbols + 6 µs signal ext,
    # then SIFS (10) + legacy ACK at 24 Mbps (20 + 2 symbols + 6) = 214 µs.
    assert R.frame_airtime_us(100, 6.5, "g24", unicast=True) == pytest.approx(214.0)
    assert R.frame_airtime_us(100, 6.5, "g5", unicast=False) == pytest.approx(164.0)


def saturate(k, size=1500, seconds=2.0):
    """k saturated senders at the gateway position, all sending to the gateway."""
    cfg = load_config(BASE)
    sim = Sim(cfg, "V2", n=k, warmup_s=0.0, steady_s=seconds, layout="open_floor")
    for r in sim.nodes.values():
        r.pos = sim.layout.gateway_pos  # everyone at max SNR
    sim.topo()
    ch = sim.ch["g24"]
    delivered = [0]
    sim.on_frame_rx = lambda node, f: delivered.__setitem__(0, delivered[0] + 1)

    def refill():
        for i in range(1, k + 1):
            st = sim.st[(i, "g24")]
            while len(st.q) < 5:
                f = Frame("seg", size, i, final=0)
                f.nh, f.band = 0, "g24"
                ch.enqueue(st, f)
        sim.at(sim.now + 1e-3, refill)
    sim.at(0.0, refill)
    while sim._q:
        t, _, fn, a = __import__("heapq").heappop(sim._q)
        if t > seconds:
            break
        sim.now = t
        fn(*a)
    goodput = delivered[0] * size * 8 / seconds / 1e6
    return goodput, ch.collisions / max(ch.tx, 1)


def test_single_link_saturation_efficiency():
    # One sender, 1500 B at 65 Mbps, no A-MPDU: 230 µs data + 44 µs SIFS/ACK
    # + 28 µs DIFS + 67.5 µs mean backoff = 370 µs/frame -> 32.4 Mbps.
    g, coll = saturate(1)
    assert g == pytest.approx(32.4, rel=0.03) and coll == 0


def test_collisions_grow_with_contenders():
    _, c2 = saturate(2)
    _, c10 = saturate(10)
    assert c2 < c10
    assert 0.05 < c10 < 0.35  # Bianchi-order collision rate at CWmin 15


def test_background_share_is_realized_on_an_idle_channel():
    cfg = load_config(BASE)
    sim = Sim(cfg, "V2", n=1, bg_share=0.3, warmup_s=0.0, steady_s=20.0)
    sim.at(0.0, sim.background, "g24")
    while sim._q:
        t, _, fn, a = __import__("heapq").heappop(sim._q)
        if t > 20.0:
            break
        sim.now = t
        fn(*a)
    assert sim.ch["g24"].busy_acc / 20.0 == pytest.approx(0.3, abs=0.04)


def test_tiny_fleet_run_is_clean():
    cfg = load_config(BASE)
    # open floor: a 3-robot fleet in the racked warehouse is coverage-limited
    # by design (see docs/sim_log.md), which is not what this test is about.
    r = Sim(cfg, "V3", n=3, warmup_s=5, steady_s=20, layout="open_floor",
            lease_beacon_hz=1.0).run()
    assert r["failed"] == [] and r["ctl_samples"] > 100


def test_same_seed_same_result_across_processes():
    # Station objects in a set iterated in memory order -> results changed
    # between processes for one seed. Run in two fresh interpreters.
    import subprocess, sys
    code = ("from commsim.core.fleet import load_config; from commsim.des.sim import Sim; "
            "from commsim.tests.fleet_test import BASE; "
            "r = Sim(load_config(BASE), 'V3', n=8, seed=5, warmup_s=5, steady_s=15).run(); "
            "print(r['ctl_p95_ms'], r['busy_g24'], r['mac_drops'])")
    root = str(BASE.parents[2])
    outs = {subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                           cwd=root).stdout for _ in range(2)}
    assert len(outs) == 1 and outs != {""}


def test_per_has_no_long_tail_below_threshold():
    # 5.7 dB under the 12 Mb/s threshold a 100 B frame must essentially never
    # get through (the old length scaling let 32% through -> routing over dead links)
    assert R.per(1.3, R.bcast_threshold(12.0), 100) > 0.999
    assert R.per(30.0, 7.0, 1500) < 1e-6
    assert R.per(7.0, 7.0, 1000) == pytest.approx(0.5)
    assert R.per(7.0, 7.0, 100) < R.per(7.0, 7.0, 1500)  # short frames slightly easier



def test_legacy_rate_table_exists_and_runs():
    # the per() rewrite once deleted LEGACY_RATES -> NameError for any ht=0 run
    assert R.pick_unicast_rate(20.0, ht=False) == (36.0, 16.0)
    assert R.pick_unicast_rate(20.0, ht=False, cap=12.0) == (12.0, 7.0)
    cfg = load_config(BASE)
    cfg.setdefault("radio_model", {}).update(ht=0.0, max_rate_mbps=12.0)
    assert Sim(cfg, "V3", n=3, warmup_s=2, steady_s=5, layout="open_floor").run()["ctl_samples"] > 0


def test_tcp_session_delivers_to_app_in_order():
    sim = Sim(load_config(BASE), "V2", n=1, warmup_s=0, steady_s=1, layout="open_floor")
    times = []
    sim.at = lambda t, fn, *a: times.append(t) if fn == sim.deliver else None
    fl = sim.flows[(1, 0)]
    from commsim.des.sim import Segment
    msgs = [{"type": "Status", "src": 1, "t0": 0.0, "data": {"owned": []}, "reliable": False,
             "wire": 100} for _ in range(50)]
    fl._deliver(Segment(fl, 0, msgs, 1000))
    fl._deliver(Segment(fl, 1, msgs[:5], 100))
    assert times == sorted(times) and len(set(times)) == len(times)


def test_obstacle_burst_actually_bursts():
    sim = Sim(load_config(BASE), "V3", n=10, warmup_s=0, steady_s=1)
    n = []
    sim.at = lambda t, fn, *a: n.append(t) if fn == sim._publish_obstacle else None
    sim.obstacle_burst(10 * sim.obstacle_rate, 10.0)  # 0.2/s x 10 robots x 10 s ~ 20
    assert 8 <= len(n) <= 40 and all(0 <= t < 10.0 for t in n)
