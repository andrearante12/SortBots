"""DES vs emulation at the M1 point: 3 nodes in a line (0-1-2, 25 dB links,
0 and 2 out of range), Status only, 3 Hz all-to-all, two sessions per pair.

    python3 -m commsim.des.crosscheck
"""
import json
import sys

from commsim.core.fleet import load_config
from commsim.des.sim import Sim, pct
from commsim.des.run import BASE

# The emulation runs being matched used the spec settings (both ends dial, 2 s lease)
SPEC = BASE.parent / "spec_original.yaml"

# Emulation reference (M1 depth 10, 90 s, 2026-10-03) — see docs/sim_log.md
EMU = {"p50_1hop_ms": 3.89, "p95_1hop_ms": 5.67, "p50_2hop_ms": 4.89, "p95_2hop_ms": 7.21,
       "delivery": 1.0, "pure_ack_per_data_seg": 0.91}


def main():
    cfg = load_config(BASE, SPEC)
    sim = Sim(cfg, "V2", n=2, seed=1, warmup_s=15, steady_s=75, layout="open_floor",
              status_only=True, fixed_snr={(0, 1): 25.0, (1, 2): 25.0},
              positions=[(0, 0), (5, 0), (10, 0)])
    hop_lat = {1: [], 2: []}
    orig = sim.deliver

    def deliver(dst, msg, hops, via_bcast=False):
        if msg["type"] == "Status" and sim.in_window(msg["t0"]):
            hop_lat.setdefault(hops, []).append((sim.now - msg["t0"]) * 1e3)
        orig(dst, msg, hops, via_bcast)
    sim.deliver = deliver
    acks = {"ack": 0, "seg": 0}
    send = sim.net_send

    def net_send(node, f):
        if node == 0 and f.kind in acks and f.final in (1, 2) and sim.in_window(sim.now):
            acks[f.kind] += 1
        send(node, f)
    sim.net_send = net_send
    r = sim.run()
    des = {"p50_1hop_ms": pct(hop_lat[1], 50), "p95_1hop_ms": pct(hop_lat[1], 95),
           "p50_2hop_ms": pct(hop_lat[2], 50), "p95_2hop_ms": pct(hop_lat[2], 95),
           "delivery": r["status_delivery"],
           "pure_ack_per_data_seg": acks["ack"] / max(1, acks["seg"])}
    print(f"{'metric':<24}{'emulation':>12}{'DES':>12}")
    for k in EMU:
        print(f"{k:<24}{EMU[k]:>12.2f}{des[k]:>12.2f}")
    return 0


def n10(path="commsim/measurements/spec_config/m1_n10_interf_result.json",
        state="commsim/measurements/spec_config/m1_n10_mesh_state.json"):
    """DES on the emulation's own n=10 warehouse SNR matrix, legacy rates."""
    emu = json.load(open(path))
    st = json.load(open(state))
    m = st["snr"]
    snr = {(i, j): float(m[i][j]) for i in range(len(m)) for j in range(i + 1, len(m))}
    print(f"{'metric':<16}{'emulation':>11}{'DES legacy':>12}{'DES HT':>9}{'DES leg<=24':>13}")
    rows = {}
    for label, ht, cap in (("legacy", 0.0, 1e9), ("ht", 1.0, 1e9), ("cap24", 0.0, 24.0)):
        cfg = load_config(BASE, SPEC)
        cfg.setdefault("radio_model", {}).update(ht=ht, max_rate_mbps=cap)
        sim = Sim(cfg, "V2", n=len(m) - 1, seed=1, warmup_s=25, steady_s=95,
                  status_only=True, fixed_snr=snr, positions=[tuple(p) for p in [n["pos"] for n in st["nodes"]]])
        r = sim.run()
        ctl = sim.lat["Status"]
        rows[label] = {"busy": r["busy_g24"], "p50": pct(ctl, 50) * 1e3, "p95": pct(ctl, 95) * 1e3,
                       "p99": pct(ctl, 99) * 1e3, "delivery": r["status_delivery"]}
    ref = {"busy": emu["air_busy_frac"], "p50": emu["p50_ms"], "p95": emu["p95_ms"],
           "p99": emu["p99_ms"], "delivery": emu["delivery_lower_bound"]}
    for k in ref:
        print(f"{k:<16}{ref[k]:>11.3f}{rows['legacy'][k]:>12.3f}{rows['ht'][k]:>9.3f}{rows['cap24'][k]:>13.3f}")


if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "n10":
    n10()
    sys.exit(0)

if __name__ == "__main__":
    sys.exit(main())
