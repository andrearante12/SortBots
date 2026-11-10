"""DES at the lab setup: 2 nodes (1 robot + gateway), one hop, 2.4 GHz (V3),
measured signal, with and without the measured background busy share.
Run from the repo root:
    python commsim/measurements/hw_lab_2026-11-09/tools/des_labmatch.py"""
import json
import math
import sys

sys.path.insert(0, ".")
from commsim.core.fleet import load_config
from commsim.des.run import BASE
from commsim.des.sim import Sim

cfg = load_config(BASE, BASE.parent / "hw_lab.yaml")
# measured -18..-23 dBm with noise -95 -> ~72 dB; the model only needs "far above MCS7"
SNR = 72.0
for bg in (0.0, 0.25, 0.30, 0.36):
    for status_only in (True, False):
        rows = []
        for seed in (1, 2, 3):
            sim = Sim(cfg, "V3", n=1, seed=seed, bg_share=bg, layout="open_floor",
                      warmup_s=15, steady_s=60, status_only=status_only,
                      fixed_snr={(0, 1): SNR}, positions=[(0, 0), (2, 0)])
            rows.append(sim.run())
        avg = lambda k: sum(r[k] for r in rows) / len(rows)
        print(json.dumps({"bg": bg, "traffic": "status only" if status_only else "full protocol",
                          "ctl_p50_ms": round(avg("ctl_p50_ms"), 2),
                          "ctl_p95_ms": round(avg("ctl_p95_ms"), 2),
                          "ctl_p99_ms": round(avg("ctl_p99_ms"), 2),
                          "status_delivery": round(avg("status_delivery"), 4),
                          "busy_g24": round(avg("busy_g24"), 4),
                          "fleet_busy_g24": round(avg("busy_g24") - bg, 4)}))
