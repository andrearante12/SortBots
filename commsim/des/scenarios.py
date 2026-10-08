"""Failure-scenario runs (plan §8) on the DES, with pass/fail per scenario.

    python3 -m commsim.des.scenarios --variants V4 V3 --n 10 15 --seeds 1 2 3 \
        --out commsim/results/S_scenarios

Each run: warm-up 20 s, injection at t=40 s, measured until t=100 s. The
batman reroute delay is the emulation-measured 1.95 s. Clock drift is covered
by tests/protocol_test.py (leases run on each robot's own clock) and is not
repeated here.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from commsim.core.fleet import load_config
from commsim.core.protocol import CLAIMED
from .run import BASE
from .sim import Sim, GW, pct

T_INJ, T_END = 40.0, 100.0


def owner_of_any(sim):
    for r in sim.nodes.values():
        if not r.is_gw and r.alive and r.alloc.owned():
            return r.id, r.alloc.owned()[0][0]
    return None, None


def scen_link_loss(sim, ctx, bands=None):
    _, links = sim.transit_load()
    (i, j) = tuple(max(links, key=links.get))
    ctx["victim"] = f"{i}-{j}"
    sim.block_link(i, j, bands)


def scen_degraded(sim, ctx):
    _, links = sim.transit_load()
    key = max(links, key=links.get)
    ctx["victim"] = "-".join(map(str, sorted(key)))
    for k in range(30):  # -1 dB/s for 30 s
        sim.at(sim.now + k, lambda k=k: (sim.snr_off.__setitem__(key, -float(k + 1)), sim.topo()))


def scen_flapping(sim, ctx, period=2.0):
    _, links = sim.transit_load()
    i, j = tuple(max(links, key=links.get))
    ctx["victim"] = f"{i}-{j} every {period}s"
    for k in range(int(30 / period)):
        t = sim.now + k * period
        sim.at(t, (sim.block_link if k % 2 == 0 else sim.unblock_link), i, j)
    sim.at(sim.now + 30, sim.unblock_link, i, j)


def scen_relay_dies(sim, ctx):
    relay, _ = sim.transit_load()
    cands = {k: v for k, v in relay.items() if k != GW}
    rid = max(cands, key=cands.get) if cands else 1
    ctx["victim"] = rid
    ctx["victim_tasks"] = [t for t, _ in sim.nodes[rid].alloc.owned()]
    ctx["legit_silent"] = {rid}
    sim.kill(rid)


def scen_isolated(sim, ctx):
    rid, tid = owner_of_any(sim)
    rid = rid or 1
    ctx["victim"] = rid
    ctx["victim_tasks"] = [t for t, _ in sim.nodes[rid].alloc.owned()]
    ctx["legit_silent"] = {rid}
    for o in sim.nodes:
        if o != rid:
            sim.block_link(rid, o)


def scen_band_loss(sim, ctx):
    ctx["victim"] = "all g24 links"
    for i in sim.nodes:
        for j in sim.nodes:
            if i < j:
                sim.block_link(i, j, ["g24"])


def scen_partition(sim, ctx):
    xs = sorted(r.pos[0] for r in sim.nodes.values())
    mid = xs[len(xs) // 2]
    a = {r.id for r in sim.nodes.values() if r.pos[0] < mid}
    b = set(sim.nodes) - a
    ctx["victim"] = f"{sorted(a)} | {sorted(b)}"
    ctx["legit_silent_pairs"] = {(x, y) for x in a for y in b} | {(y, x) for x in a for y in b}
    for x in a:
        for y in b:
            sim.block_link(x, y)
    def heal():
        ctx["t_heal"] = sim.now
        for x in a:
            for y in b:
                sim.unblock_link(x, y)
    sim.at(sim.now + 30, heal)


def scen_gateway_loss(sim, ctx):
    ctx["victim"] = GW
    ctx["done_before"] = sum(1 for t in sim.tasks.values() if t["done"])
    sim.kill(GW)
    def back():
        ctx["t_back"] = sim.now
        ctx["done_during"] = sum(1 for t in sim.tasks.values() if t["done"]) - ctx["done_before"]
        sim.revive(GW, fresh=True)
    sim.at(sim.now + 30, back)


def scen_reboot(sim, ctx):
    rid = 1
    ctx["victim"] = rid
    ctx["legit_silent"] = {rid}
    sim.kill(rid)
    sim.at(sim.now + 10, lambda: (ctx.__setitem__("t_back", sim.now), sim.revive(rid, fresh=True)))


def scen_burst(sim, ctx):
    ctx["victim"] = "30 tasks + 10x obstacles for 10 s"
    for _ in range(30):
        sim.task_arrival_once()
    sim.obstacle_burst(10 * sim.obstacle_rate, 10.0)


def scen_timer_misorder(sim, ctx):
    # negative control: status timeout below the measured reroute time
    for r in sim.nodes.values():
        r.alloc.p.status_timeout_s = 1.5
    scen_link_loss(sim, ctx)


SCENARIOS = {
    "link_loss": scen_link_loss,
    "link_loss_g24": lambda s, c: scen_link_loss(s, c, ["g24"]),
    "degraded": scen_degraded,
    "flapping": scen_flapping,
    "relay_dies": scen_relay_dies,
    "isolated": scen_isolated,
    "band_loss": scen_band_loss,
    "partition": scen_partition,
    "gateway_loss": scen_gateway_loss,
    "reboot": scen_reboot,
    "burst": scen_burst,
    "timer_misorder": scen_timer_misorder,
}


def run_one(spec):
    cfg = load_config(BASE, *spec.get("configs", []))
    if spec.get("status_timeout"):
        cfg["protocol"]["status_timeout_s"] = spec["status_timeout"]
    if spec.get("ogm_ms"):
        cfg["batman"]["orig_interval_ms"] = spec["ogm_ms"]
    if spec.get("reroute_s"):
        cfg.setdefault("commsim_des", {})["reroute_delay_s"] = spec["reroute_s"]
    if spec.get("max_open_bids") is not None:
        cfg["protocol"]["max_open_bids"] = spec["max_open_bids"]
    if spec.get("lease_ms"):
        cfg["zenoh"]["transport_lease_ms"] = spec["lease_ms"]
    sim = Sim(cfg, spec["variant"], spec["n"], seed=spec["seed"], warmup_s=20.0,
              steady_s=T_END - 20.0, status_transport=spec.get("status_transport"),
              track_gaps=True)
    if spec["scenario"] == "timer_misorder":
        # Allocators share one Params object; copy so only this run is affected
        import copy
        sim.params = copy.copy(sim.params)
        for r in sim.nodes.values():
            r.alloc.p = sim.params
    ctx = {"legit_silent": set(), "legit_silent_pairs": set()}
    events = []
    orig_on_event = sim.on_event

    def on_event(r, e):
        ev = dict(e, robot_id=r.id)
        if e["event"] == "robot_silent":
            # silence is correct when the failure left no route between the two
            ev["no_route"] = sim.hop(r.id, e["peer"]) is None or sim.hop(e["peer"], r.id) is None
        if e["event"] == "lease_expired" and e.get("owner") is not None:
            # expiry of an unreachable owner's lease is the accepted partition behavior
            ev["no_route"] = sim.hop(r.id, e["owner"]) is None or sim.hop(e["owner"], r.id) is None
        events.append(ev)
        orig_on_event(r, e)
    sim.on_event = on_event
    sim.at(T_INJ, lambda: SCENARIOS[spec["scenario"]](sim, ctx))
    res = sim.run()

    after = [e for e in events if e["t"] >= T_INJ]
    def legit(e):
        p = e.get("peer")
        return (p in ctx["legit_silent"] or e["robot_id"] in ctx["legit_silent"]
                or (e["robot_id"], p) in ctx["legit_silent_pairs"] or e.get("no_route"))
    false_silent = [e for e in after if e["event"] == "robot_silent" and not legit(e)]
    # recovery: longest Status gap after injection among pairs that were flowing before
    gaps = []
    dead = ctx["legit_silent"] | ({ctx["victim"]} if isinstance(ctx.get("victim"), int) else set())
    # Only pairs that stayed subscribed for the whole window (neighbor-only status
    # stops when robots move apart — that is not an outage), plus robot -> gateway.
    span = [hs for t, hs in sim.sub_hist if T_INJ - 10 <= t <= T_END]
    steady = frozenset.intersection(*span) if span and sim.v["neighbor"] else None
    for (src, dst), ts in sim.status_rx.items():
        if src in dead or dst in dead or (src, dst) in ctx["legit_silent_pairs"]:
            continue
        if steady is not None and dst != GW and (src, dst) not in steady:
            continue
        if sim.hop(src, dst) is None:  # failure partitioned them: an outage, not a fault
            continue
        before = [t for t in ts if T_INJ - 10 <= t < T_INJ]
        if len(before) < 5:
            continue
        win = [t for t in ts if t >= T_INJ - 1]
        spans = list(zip(win, win[1:]))
        if win and win[-1] < T_END - 2:
            spans.append((win[-1], T_END))
        cut = sim.unreach.get((src, dst), [])
        # time with no route at all is an outage the failure caused (e.g. the
        # isolated robot was this pair's only relay), not a transport fault
        g = max((b - a - sum(1.0 for t in cut if a < t < b) for a, b in spans), default=0.0)
        gaps.append(max(g, 0.0))
    out = {
        **{k: spec[k] for k in ("scenario", "variant", "n", "seed")},
        "tag": spec.get("tag", ""), "status_transport": sim.status_transport,
        "zenoh_lease_s": sim.lease_s, "orig_interval_ms": int(sim.cfg["batman"]["orig_interval_ms"]),
        "max_open_bids": sim.params.max_open_bids, "reroute_delay_s": sim.reroute_delay_s,
        "lease_beacon_hz": sim.lease_beacon_hz, "single_connect": sim.single_connect,
        "status_timeout_s": 1.5 if spec["scenario"] == "timer_misorder" else sim.params.status_timeout_s,
        "victim": str(ctx.get("victim")),
        "max_status_gap_s": max(gaps) if gaps else None,
        "p95_status_gap_s": pct(gaps, 95) if gaps else None,
        "false_silent": len(false_silent),
        "lease_expired_after": sum(1 for e in after if e["event"] == "lease_expired"),
        "lease_expired_reachable": sum(1 for e in after if e["event"] == "lease_expired"
                                       and not e.get("no_route") and not legit(e)),
        "dup_episodes": res["dup_episodes"], "abandoned": res["abandoned"],
        # duplicates between owners that could still reach each other = a fault;
        # between mutually unreachable owners = the accepted partition behavior
        "dup_connected": dup_persisting_connected(sim.dup_log),
        "dup_partitioned": len({tid for t, tid, o, ok in sim.dup_log if not ok and t >= T_INJ}),
        "ctl_p95_ms": res["ctl_p95_ms"], "ctl_p99_ms": res["ctl_p99_ms"],
        "auction_p95_s": res["auction_p95_s"], "auction_max_s": res["auction_max_s"],
        "session_drops": res["session_drops"],
    }
    vt = ctx.get("victim_tasks") or []
    if vt:
        claims = [e for e in after if e["event"] == "claimed" and e["task_id"] in vt]
        out["reclaim_after_s"] = (min(e["t"] for e in claims) - T_INJ) if claims else None
        stops = [e for e in after if e["event"] == "stop_task" and e["robot_id"] == ctx["victim"]]
        out["victim_stop_after_s"] = (min(e["t"] for e in stops) - T_INJ) if stops else None
    if "t_heal" in ctx:
        dups_after = [e for e in after if e["event"] == "task_yielded" and e["t"] >= ctx["t_heal"]]
        out["yields_after_heal"] = len(dups_after)
        out["last_yield_after_heal_s"] = max((e["t"] - ctx["t_heal"] for e in dups_after), default=None)
    if "done_during" in ctx:
        out["tasks_done_during_gw_outage"] = ctx["done_during"]
        out["stops_during_gw_outage"] = sum(
            1 for e in after if e["event"] in ("stop_task", "task_yielded") and e["t"] < ctx["t_back"])
        gw_heard = [t for (src, dst), ts in sim.status_rx.items() if dst == GW
                    for t in ts if t >= ctx["t_back"]]
        firsts = {}
        for (src, dst), ts in sim.status_rx.items():
            if dst == GW:
                later = [t for t in ts if t >= ctx["t_back"]]
                if later:
                    firsts[src] = later[0]
        out["gw_view_rebuilt_after_s"] = (max(firsts.values()) - ctx["t_back"]) \
            if len(firsts) >= spec["n"] else None
    if "t_back" in ctx and spec["scenario"] == "reboot":
        r = sim.nodes[ctx["victim"]]
        out["rebooted_knows_tasks"] = len(r.alloc.tasks)
        heard = [ts for (src, dst), ts in sim.status_rx.items() if src == ctx["victim"] and dst == GW]
        out["rejoin_first_status_s"] = min((t for ts in heard for t in ts if t >= ctx["t_back"]),
                                           default=math.nan) - ctx["t_back"]
    out["pass"], out["why"] = judge(spec["scenario"], out)
    return out


def dup_persisting_connected(log, hold_s=3.0):
    """Tasks whose duplicate owners could reach each other AND stayed
    duplicated for >= hold_s (1 Hz samples). A tie-break resolving duplicates
    within ~2 s of a heal is the accepted behavior, not a fault."""
    runs, bad = {}, set()
    for t, tid, owners, ok in sorted(log):
        if t < T_INJ:
            continue
        if not ok:
            runs.pop(tid, None)
            continue
        start, last = runs.get(tid, (t, t))
        if t - last > 1.5:
            start = t
        runs[tid] = (start, t)
        if t - start >= hold_s:
            bad.add(tid)
    return len(bad)


def judge(sc, o):
    gap = o["max_status_gap_s"] or 0
    if sc in ("link_loss", "link_loss_g24", "flapping", "band_loss"):
        ok = o["false_silent"] == 0 and o["lease_expired_reachable"] == 0
        return ok, (f"false_silent={o['false_silent']} lease_expired_reachable={o['lease_expired_reachable']} "
                    f"(all={o['lease_expired_after']}) max_gap={gap:.2f}s")
    if sc == "degraded":
        ok = o["false_silent"] == 0 and o["ctl_p95_ms"] <= 150
        return ok, f"false_silent={o['false_silent']} p95={o['ctl_p95_ms']:.0f}ms max_gap={gap:.2f}s"
    if sc in ("relay_dies", "isolated"):
        rc = o.get("reclaim_after_s")
        st = o.get("victim_stop_after_s")
        ok = o["dup_connected"] == 0 and (rc is None or rc >= 12.0) and \
            (sc != "isolated" or st is None or st <= 10.2) and o["false_silent"] == 0
        return ok, (f"reclaim={rc} stop={st} dup_conn={o['dup_connected']} dup_part={o['dup_partitioned']} "
                    f"false_silent={o['false_silent']} max_gap={gap:.2f}s")
    if sc == "partition":
        ok = (o.get("last_yield_after_heal_s") or 0) <= 3.0 and o["false_silent"] == 0
        return ok, (f"dup_during={o['dup_episodes']} yields_after_heal={o.get('yields_after_heal')} "
                    f"resolved_in={o.get('last_yield_after_heal_s')} false_silent={o['false_silent']}")
    if sc == "gateway_loss":
        ok = o.get("stops_during_gw_outage") == 0 and o["false_silent"] == 0 and \
            o.get("gw_view_rebuilt_after_s") is not None
        return ok, (f"stops_during_outage={o.get('stops_during_gw_outage')} "
                    f"done_during_outage={o.get('tasks_done_during_gw_outage')} "
                    f"gui_rebuilt_in={o.get('gw_view_rebuilt_after_s')} false_silent={o['false_silent']}")
    if sc == "reboot":
        ok = (o.get("rejoin_first_status_s") or 99) < 5 and o["false_silent"] == 0
        return ok, (f"rejoin={o.get('rejoin_first_status_s'):.2f}s knows_tasks={o.get('rebooted_knows_tasks')} "
                    f"(transient-local replay NOT modeled)")
    if sc == "burst":
        # Spec pass condition: "auctions complete; control traffic not starved".
        # In a 30-task burst some winners hit max_owned and step aside, so a
        # second 1 s window is fleet capacity — not judged against the 1.5 s
        # steady-state auction threshold (that judging was used until 2026-10-03).
        amax = o["auction_max_s"]
        ok = (amax == amax and amax <= 10.0) and o["ctl_p95_ms"] <= 150 and \
            o["false_silent"] == 0 and o["dup_connected"] == 0
        return ok, (f"auction_p95={o['auction_p95_s']:.2f}s max={amax:.2f}s ctl_p95={o['ctl_p95_ms']:.0f}ms "
                    f"false_silent={o['false_silent']} dup_conn={o['dup_connected']}")
    if sc == "timer_misorder":
        ok = o["false_silent"] > 0  # negative control: SHOULD produce false silents
        return ok, f"(negative control) false_silent={o['false_silent']}"
    return False, "unknown"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", nargs="+", default=list(SCENARIOS))
    ap.add_argument("--variants", nargs="+", default=["V4"])
    ap.add_argument("--n", nargs="+", type=int, default=[10])
    ap.add_argument("--seeds", nargs="+", type=int, default=[1])
    ap.add_argument("--status-timeout", type=float, default=None)
    ap.add_argument("--lease-ms", type=int, default=None, help="zenoh transport lease")
    ap.add_argument("--status-transport", choices=["tcp", "udp"], default=None)
    ap.add_argument("--configs", nargs="*", default=[], help="extra YAML layers over base.yaml")
    ap.add_argument("--tag", default="", help="label stored with each row")
    ap.add_argument("--max-open-bids", type=int, default=None)
    ap.add_argument("--ogm-ms", type=int, default=None, help="batman originator interval")
    ap.add_argument("--reroute-s", type=float, default=None,
                    help="DES reroute delay (measured worst case: 5.67 s @500 ms, 3.0 s @250 ms)")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 4))
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    specs = [{"scenario": sc, "variant": v, "n": n, "seed": s, "status_timeout": a.status_timeout,
              "lease_ms": a.lease_ms, "status_transport": a.status_transport,
              "configs": a.configs, "tag": a.tag, "max_open_bids": a.max_open_bids,
              "ogm_ms": a.ogm_ms, "reroute_s": a.reroute_s}
             for sc in a.scenarios for v in a.variants for n in a.n for s in a.seeds
             if not (sc in ("band_loss", "link_loss_g24") and v in ("V3", "V2", "V1", "phase0"))]
    a.out.mkdir(parents=True, exist_ok=True)
    with ProcessPoolExecutor(a.jobs) as ex, open(a.out / "runs.jsonl", "a") as f:
        for o in ex.map(run_one, specs):
            f.write(json.dumps(o, default=str) + "\n")
            f.flush()
            print(f"{o['scenario']:<15}{o['variant']:<4} n={o['n']:<3} s={o['seed']} T={o['status_timeout_s']} "
                  f"{'PASS' if o['pass'] else 'FAIL'}  {o['why']}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
