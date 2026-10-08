"""Run DES points / sweeps in parallel; append results as JSON lines + CSV.

    python3 -m commsim.des.run --variants V2 V3 --n 5 10 20 --seeds 1 2 3 \
        --bg 0 0.15 0.30 --out commsim/results/sweep1 [--until-fail]

--until-fail grows n per (variant, bg) group in --n order and stops a group
after the first n where a majority of seeds fail — the sweep then names the
failure point and the criterion that tripped first.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from commsim.core.fleet import load_config
from .sim import Sim

REPO = Path(__file__).resolve().parents[2]
BASE = REPO / "commsim" / "configs" / "base.yaml"


def git_sha() -> str:
    try:
        return subprocess.check_output(["git", "-C", str(REPO), "rev-parse", "--short", "HEAD"],
                                       text=True).strip()
    except Exception:
        return "unknown"


def run_point(spec: dict) -> dict:
    cfg = load_config(*spec["configs"])
    if spec.get("radio"):
        cfg.setdefault("radio_model", {}).update(spec["radio"])
    sim = Sim(cfg, spec["variant"], spec["n"], seed=spec["seed"], bg_share=spec["bg"],
              layout=spec["layout"], warmup_s=spec["warmup"], steady_s=spec["steady"],
              lease_beacon_hz=spec["lease_beacon_hz"],
              status_transport=spec.get("status_transport"),
              transport_liveness=spec["transport_liveness"],
              single_connect=spec.get("single_connect"),
              overrides=spec.get("overrides"))
    res = sim.run()
    res.update(layout=spec["layout"], warmup_s=spec["warmup"], steady_s=spec["steady"],
               label=spec["label"], transport_liveness=spec["transport_liveness"])
    blob = json.dumps({"cfg": cfg, "spec": spec}, sort_keys=True, default=str)
    res["config_hash"] = hashlib.sha1(blob.encode()).hexdigest()[:10]
    res["run_id"] = f"{res['config_hash']}-s{spec['seed']}"
    return res


def write(out: Path, rows: list[dict]) -> None:
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "runs.jsonl", "a") as f:
        for r in rows:
            f.write(json.dumps(r, default=str) + "\n")
    allrows = [json.loads(l) for l in open(out / "runs.jsonl")]
    keys = sorted({k for r in allrows for k in r})
    with open(out / "runs.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in allrows:
            w.writerow({k: (";".join(v) if isinstance(v, list) else v) for k, v in r.items()})


def fmt(r: dict) -> str:
    def g(k, d=1):
        v = r.get(k)
        return "nan" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.{d}f}"
    return (f"{r['label']:<10} n={r['n']:<3} s={r['seed']} bg={r['bg_share']:.2f} "
            f"busy24={g('busy_g24', 3)} busy5={g('busy_g5', 3)} "
            f"p95={g('ctl_p95_ms')}ms p99={g('ctl_p99_ms')}ms (N={r['ctl_samples']}) "
            f"deliv={g('status_delivery', 3)} silent={r['false_silent']} "
            f"dup={r['dup_episodes']} aband={r['abandoned']} lease_exp={r['lease_expired']} "
            f"auc95={g('auction_p95_s', 2)}s maplag={g('map_lag_p95', 0)} "
            f"conn={g('conn_frac', 3)} FAIL={','.join(r['failed']) or '-'} [{r['wall_s']}s]")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", nargs="+", default=["V2"])
    ap.add_argument("--n", nargs="+", type=int, default=[5])
    ap.add_argument("--seeds", nargs="+", type=int, default=[1])
    ap.add_argument("--bg", nargs="+", type=float, default=[0.0])
    ap.add_argument("--layout", default="warehouse")
    ap.add_argument("--warmup", type=float, default=20.0)
    ap.add_argument("--steady", type=float, default=60.0)
    ap.add_argument("--lease-beacon-hz", type=float, default=None,
                    help="default: protocol.lease_beacon_hz from the config (1.0; 0 = spec)")
    ap.add_argument("--status-transport", choices=["tcp", "udp"], default=None)
    ap.add_argument("--no-transport-liveness", action="store_true")
    ap.add_argument("--single-connect", action="store_true",
                    help="one TCP session per pair (lower id dials) instead of both dialing")
    ap.add_argument("--label", default=None, help="defaults to the variant name")
    ap.add_argument("--suffix", default="", help="appended to each variant's label, e.g. +LB")
    ap.add_argument("--configs", nargs="*", default=[], help="extra YAML layers over base.yaml")
    ap.add_argument("--set", nargs="*", default=[], help="variant overrides key=json")
    ap.add_argument("--radio", nargs="*", default=[], help="radio_model overrides key=value")
    ap.add_argument("--until-fail", action="store_true")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 4))
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    overrides = {k: json.loads(v) for k, v in (s.split("=", 1) for s in a.set)}
    radio = {k: float(v) for k, v in (s.split("=", 1) for s in a.radio)}
    sha = git_sha()

    def spec(v, n, s, bg):
        return {"variant": v, "n": n, "seed": s, "bg": bg, "layout": a.layout,
                "warmup": a.warmup, "steady": a.steady, "lease_beacon_hz": a.lease_beacon_hz,
                "transport_liveness": not a.no_transport_liveness,
                "single_connect": True if a.single_connect else None,
                "status_transport": a.status_transport,
                "label": (a.label or v) + a.suffix, "configs": [str(BASE)] + a.configs,
                "overrides": overrides or None, "radio": radio or None, "git": sha}

    groups = [(v, bg) for v in a.variants for bg in a.bg]
    with ProcessPoolExecutor(max_workers=a.jobs) as ex:
        if not a.until_fail:
            specs = [spec(v, n, s, bg) for v, bg in groups for n in a.n for s in a.seeds]
            rows = list(ex.map(run_point, specs))
            for r in rows:
                print(fmt(r), flush=True)
            write(a.out, rows)
            return 0
        # grow n per group; all groups advance in lockstep for parallelism
        live = {g: True for g in groups}
        for n in a.n:
            batch = [(g, spec(g[0], n, s, g[1])) for g in groups if live[g] for s in a.seeds]
            if not batch:
                break
            rows = list(ex.map(run_point, [sp for _, sp in batch]))
            write(a.out, rows)
            for (g, _), r in zip(batch, rows):
                print(fmt(r), flush=True)
            for g in groups:
                if not live[g]:
                    continue
                rs = [r for (gg, _), r in zip(batch, rows) if gg == g]
                if sum(1 for r in rs if r["failed"]) * 2 > len(rs):
                    live[g] = False
                    crit = {}
                    for r in rs:
                        for c in r["failed"]:
                            crit[c] = crit.get(c, 0) + 1
                    print(f"== {(a.label or g[0]) + a.suffix} bg={g[1]:.2f}: FAILS at n={n} "
                          f"({', '.join(f'{c} x{k}' for c, k in sorted(crit.items()))})", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
