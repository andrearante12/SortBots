#!/usr/bin/env python3
"""Detect VRAM/RAM/CPU headroom and size a run to fit within a budget of it.

Stdlib-only at import time (no isaacsim/omni, no yaml, no third-party deps) so
it can run before Isaac's venv is even activated — scripts/run_demo.sh shells
out to this in a clean shell, same as it already does for --robot-ids
derivation. Kept alongside scripts/_isaac_utils.py but separate from it:
_isaac_utils.gpu_name() only works AFTER Kit has booted (it reads Kit's own
carb.settings), so it's useless for a pre-launch sizing decision.

Scope, per the 2026-08-26 hardware-accommodation plan: this only sizes
--robots / --chase-cam-robots. scripts/bench_sim.sh's own measured findings
say render-PRODUCT count is what scales GPU cost, not render rate or camera
resolution — those stay untouched here.

    python3 scripts/_hw_budget.py --dump
    python3 scripts/_hw_budget.py --pick --max-robots 2
    python3 scripts/_hw_budget.py --pick --max-robots 2 --requested-robots 2
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess

# TODO-calibrate (2026-08-26): no per-robot/per-chase-cam VRAM sweep has been
# run yet (scripts/bench_sim.sh could not run it either — it was passing a
# dead --render-every flag that spawn_warehouse.py never accepted, fixed in
# the same commit as this file). These are conservative placeholders
# extrapolated only from docs/setup.md's informal troubleshooting note ("Kit
# alone takes ~6 GB [VRAM]" on an 8GB card). Replace with dated real numbers
# from the four-corner sweep documented in bench_sim.sh's header, then delete
# this comment.
KIT_BASELINE_MIB = 6144
PER_ROBOT_MIB = 400
PER_CHASE_CAM_MIB = 250
RAM_BASELINE_MIB = 4096
PER_ROBOT_RAM_MIB = 300

DEFAULT_BUDGET_PCT = 90.0


def detect_vram_mib() -> int | None:
    """Total VRAM of the first GPU, via nvidia-smi. None if unavailable.

    Same query shape as scripts/install_isaac_sim.sh:60, so numbers here agree
    with what the install-time gate already reported for this machine.
    """
    override = os.environ.get("SORTBOTS_HW_VRAM_MIB")
    if override:
        return int(override)
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.total", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=10, check=True,
        )
        return int(out.stdout.strip().splitlines()[0].strip())
    except Exception:
        return None


def detect_ram_mib() -> int | None:
    """Total system RAM, via /proc/meminfo. None if unavailable."""
    override = os.environ.get("SORTBOTS_HW_RAM_MIB")
    if override:
        return int(override)
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    kib = int(line.split()[1])
                    return kib // 1024
    except Exception:
        pass
    return None


def detect_cpu_count() -> int:
    override = os.environ.get("SORTBOTS_HW_CPUS")
    if override:
        return int(override)
    return os.cpu_count() or 1


def budget_pct() -> float:
    raw = os.environ.get("SORTBOTS_HW_BUDGET_PCT")
    pct = float(raw) if raw else DEFAULT_BUDGET_PCT
    if not 0 < pct <= 100:
        raise ValueError(f"SORTBOTS_HW_BUDGET_PCT must be in (0, 100], got {pct}")
    return pct


def pick(
    *,
    max_robots: int,
    vram_mib: int | None,
    ram_mib: int | None,
    pct: float,
    requested_robots: int | None = None,
    requested_chase_cams: int | None = None,
) -> tuple[int, int, list[str]]:
    """Return (robots, chase_cam_robots, notes).

    Pure function, no I/O — callers pass in already-detected values, which
    keeps this trivially unit-testable without real hardware or env-var
    plumbing (see tests/hw_budget_test.py).

    Only computes whichever of the two values wasn't explicitly requested;
    a `requested_*` value always passes through unchanged. Degrades to the
    minimum viable config (1 robot, no chase cam) rather than refusing to
    start when the budget is too tight even for that — refusing outright is
    scripts/install_isaac_sim.sh's job (the install-time hardware gate), not
    this one's.
    """
    notes: list[str] = []

    if vram_mib is None or ram_mib is None:
        notes.append(
            "could not detect VRAM/RAM (no nvidia-smi / /proc/meminfo) — "
            "falling back to the hardcoded default (1 robot)"
        )
        robots = requested_robots if requested_robots is not None else 1
        chase_cams = requested_chase_cams if requested_chase_cams is not None else min(robots, 1)
        return robots, chase_cams, notes

    vram_budget = vram_mib * pct / 100
    ram_budget = ram_mib * pct / 100
    notes.append(
        f"detected {vram_mib} MiB VRAM, {ram_mib} MiB RAM — "
        f"budget {pct:.0f}%: {vram_budget:.0f} MiB VRAM, {ram_budget:.0f} MiB RAM"
    )

    if requested_robots is not None:
        robots = requested_robots
    else:
        vram_robots = 1 + max(0, int((vram_budget - KIT_BASELINE_MIB) // PER_ROBOT_MIB))
        ram_robots = 1 + max(0, int((ram_budget - RAM_BASELINE_MIB) // PER_ROBOT_RAM_MIB))
        robots = max(1, min(max_robots, vram_robots, ram_robots))

    if requested_chase_cams is not None:
        chase_cams = requested_chase_cams
    else:
        vram_after_robots = vram_budget - KIT_BASELINE_MIB - robots * PER_ROBOT_MIB
        chase_cams = max(0, min(robots, int(vram_after_robots // PER_CHASE_CAM_MIB)))

    if robots <= 1 and chase_cams == 0 and requested_robots is None:
        estimated = KIT_BASELINE_MIB + PER_ROBOT_MIB
        if estimated > vram_budget:
            # Not an "ERROR:" line — webui/session.py's PHASE_PATTERNS matches
            # ^ERROR: and would flip the dashboard's phase to "failed" over
            # what is only a tight-budget warning, not a launch failure.
            notes.append(
                "budget too tight even for 1 robot with no chase cam — "
                "running anyway at the minimum; consider closing other GPU "
                "apps or raising SORTBOTS_HW_BUDGET_PCT"
            )

    notes.append(f"picked robots={robots} chase_cam_robots={chase_cams}")
    return robots, chase_cams, notes


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--dump", action="store_true", help="Print detected values + a pick as JSON.")
    g.add_argument("--pick", action="store_true", help="Print 'robots chase_cam_robots note...' as one line.")
    p.add_argument("--max-robots", type=int, default=2)
    p.add_argument("--requested-robots", type=int, default=None)
    p.add_argument("--requested-chase-cams", type=int, default=None)
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    vram = detect_vram_mib()
    ram = detect_ram_mib()
    cpus = detect_cpu_count()
    pct = budget_pct()
    robots, chase_cams, notes = pick(
        max_robots=args.max_robots,
        vram_mib=vram,
        ram_mib=ram,
        pct=pct,
        requested_robots=args.requested_robots,
        requested_chase_cams=args.requested_chase_cams,
    )
    if args.dump:
        print(json.dumps({
            "vram_mib": vram,
            "ram_mib": ram,
            "cpus": cpus,
            "budget_pct": pct,
            "robots": robots,
            "chase_cam_robots": chase_cams,
            "notes": notes,
        }))
    else:
        # One line: robots, chase_cam_robots, then the note(s) joined for a
        # human to read in run_demo.sh's echo. Callers `read -r a b rest`.
        print(f"{robots} {chase_cams} {'; '.join(notes)}")


if __name__ == "__main__":
    main()
