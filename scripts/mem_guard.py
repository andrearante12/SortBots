#!/usr/bin/env python3
"""Stop the sim cleanly before it runs out of memory.

    scripts/mem_guard.py            # run_demo.sh starts this; it exits with the pipeline
    scripts/mem_guard.py --check    # one reading against the thresholds, no action

run_demo.sh already fences the pipeline into a systemd scope (MemoryHigh /
MemoryMax), so a runaway can't take the desktop down with it. But hitting
the fence isn't a clean end either. Above MemoryHigh the kernel throttles and
reclaims every process in the scope, so the sim crawls; at MemoryMax it
OOM-kills one of them, and by OOM score that's Isaac or the biggest
RTAB-Map. Either way the run carries on half dead, and the dashboard says
"stopped outside the dashboard?". RTAB-Map is the process that grows without
bound: there's no Rtabmap/MemoryThr, and two exploring robots filled the box
in about 30 min on 2026-10-21.

This trips first, stops the whole pipeline with `run_demo.sh stop
--keep-console`, and writes the reason to REPORT. webui/session.py shows that
reason as the session's error.

Trips, each sustained for `--sustain` consecutive samples so one spike
(a map checkpoint) doesn't end a run:
  - fence anon memory above `--fence-frac` of the scope's MemoryHigh
    (anon, not memory.current: page cache is reclaimable and would trip on I/O)
  - system MemAvailable below `--min-avail-gb` (default max(3 GB, 10 % of
    RAM)): the fence only bounds the sim, and the rest of the machine can
    still eat what's left. systemd-oomd watches user@<uid>.service at 50 %
    pressure, and a desktop app is as likely a victim as the sim.
  - memory PSI "full" avg10 above `--max-psi-full` %, in the fence or
    system-wide: every task stalled on reclaim at once, the swap storm that
    froze the desktop on 2026-10-15.
Warnings are logged once per episode and take no action: fence anon above 85
% of the budget; GPU memory above 97 % (VRAM can't be reclaimed or OOM-killed,
so the only relief is fewer render products).
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mem_probe  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
REPORT = Path("/tmp/sortbots_mem_guard.json")
LOG = Path("/tmp/sortbots_mem_guard.log")
GB = 2**30


def default_min_avail_gb() -> float:
    total_gb = mem_probe.meminfo()["MemTotal"] / 1048576
    return max(3.0, round(0.10 * total_gb, 1))


def gpu_total_mib() -> int | None:
    try:
        r = subprocess.run(["nvidia-smi", "--query-gpu=memory.total",
                            "--format=csv,noheader,nounits"],
                           capture_output=True, text=True, timeout=5)
        return int(r.stdout.split()[0])
    except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
        return None


def top_groups(s: dict, n: int = 6) -> list[list]:
    g = sorted(s["groups_kb"].items(), key=lambda kv: -kv[1])[:n]
    return [[k, round(v / 1024)] for k, v in g]


def fence_budget(fence: dict) -> int | None:
    """MemoryHigh if set (where throttling starts), else MemoryMax."""
    return fence.get("high") or fence.get("max")


def verdict(s: dict, args) -> str | None:
    fence = s["cgroups"].get("fence") or {}
    budget = fence_budget(fence)
    if budget and fence.get("anon", 0) > args.fence_frac * budget:
        return (f"sim memory {fence['anon'] / GB:.1f} GB is over "
                f"{args.fence_frac:.0%} of its {budget / GB:.0f} GB budget (MemoryHigh)")
    avail_gb = s["mem_avail_kb"] / 1048576
    if avail_gb < args.min_avail_gb:
        return f"machine MemAvailable {avail_gb:.1f} GB < {args.min_avail_gb:.1f} GB"
    full = max(s["psi"].get("full", 0.0), fence.get("psi_full", 0.0))
    if full > args.max_psi_full:
        return f"memory pressure (PSI full avg10) {full:.0f}% > {args.max_psi_full:.0f}%"
    return None


def log(msg: str) -> None:
    line = time.strftime("%H:%M:%S ") + msg
    print(line, flush=True)
    with open(LOG, "a") as f:
        f.write(line + "\n")


def _largest(s: dict, n: int = 4) -> str:
    return ", ".join(f"{k} {mb} MB" for k, mb in top_groups(s, n))


def stop_pipeline(reason: str, s: dict) -> None:
    fence = s["cgroups"].get("fence") or {}
    report = {
        "stopped_at": time.time(),
        "reason": reason,
        "sim_t": s["sim_t"],
        "fence_anon_gb": round(fence.get("anon", 0) / GB, 2),
        "mem_avail_gb": round(s["mem_avail_kb"] / 1048576, 2),
        "swap_used_gb": round(s["swap_used_kb"] / 1048576, 2),
        "psi": s["psi"],
        "gpu_mib": s["gpu_mib"],
        "top": top_groups(s),
    }
    REPORT.write_text(json.dumps(report, indent=1))
    log(f"STOPPING the sim: {reason}; largest: {_largest(s, 6)}")
    # --keep-console: the dashboard is what tells the operator why.
    subprocess.run([str(REPO_ROOT / "scripts" / "run_demo.sh"), "stop", "--keep-console"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--interval", type=float, default=5.0)
    ap.add_argument("--sustain", type=int, default=3)
    ap.add_argument("--fence-frac", type=float,
                    default=float(os.environ.get("SORTBOTS_MEM_GUARD_FRAC", 0.97)))
    ap.add_argument("--min-avail-gb", type=float,
                    default=float(os.environ.get("SORTBOTS_MEM_GUARD_MIN_GB", 0)) or None)
    ap.add_argument("--max-psi-full", type=float, default=30.0)
    ap.add_argument("--startup-grace", type=float, default=300.0,
                    help="seconds to wait for the pipeline to appear before giving up")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)
    if args.min_avail_gb is None:
        args.min_avail_gb = default_min_avail_gb()
    return args


def main() -> int:
    args = parse_args()
    if args.check:
        s = mem_probe.sample()
        v = verdict(s, args)
        fence = s["cgroups"].get("fence") or {}
        print(json.dumps({
            "verdict": v or "ok",
            "fence_anon_gb": round(fence.get("anon", 0) / GB, 2) if fence else None,
            "fence_budget_gb": round(fence_budget(fence) / GB, 1) if fence_budget(fence) else None,
            "mem_avail_gb": round(s["mem_avail_kb"] / 1048576, 2),
            "min_avail_gb": args.min_avail_gb,
            "psi": s["psi"], "gpu_mib": s["gpu_mib"], "top": top_groups(s)}, indent=1))
        return 1 if v else 0

    gpu_total = gpu_total_mib()
    REPORT.unlink(missing_ok=True)  # a report from an earlier run must not explain this one
    log(f"guarding (pid {os.getpid()}): trip at fence anon > {args.fence_frac:.0%} of MemoryHigh, "
        f"MemAvailable < {args.min_avail_gb:.1f} GB or PSI full > {args.max_psi_full:.0f}%, "
        f"sustained {args.sustain}×{args.interval:.0f}s")
    started = time.time()
    seen = False
    strikes = 0
    warned = {"budget": False, "gpu": False}
    while True:
        s = mem_probe.sample()
        alive = any(k.endswith("/isaac") for k in s["groups_kb"])
        if alive:
            seen = True
        elif seen or time.time() - started > args.startup_grace:
            log("pipeline gone; guard exiting")
            return 0
        v = verdict(s, args) if alive else None
        strikes = strikes + 1 if v else 0
        if strikes >= args.sustain:
            stop_pipeline(v, s)
            return 3
        fence = s["cgroups"].get("fence") or {}
        budget = fence_budget(fence)
        near = bool(budget and fence.get("anon", 0) > 0.85 * budget)
        if near and not warned["budget"]:
            log(f"warning: sim memory {fence['anon'] / GB:.1f} GB of a {budget / GB:.0f} GB "
                f"budget; largest: {_largest(s)}")
        warned["budget"] = near
        hot = bool(gpu_total and s["gpu_mib"] and s["gpu_mib"] > 0.97 * gpu_total)
        if hot and not warned["gpu"]:
            log(f"warning: GPU memory {s['gpu_mib']}/{gpu_total} MiB; "
                "fewer render products (chase cams, robots) is the only relief")
        warned["gpu"] = hot
        time.sleep(args.interval)


if __name__ == "__main__":
    sys.exit(main())
