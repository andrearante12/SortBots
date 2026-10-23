#!/usr/bin/env python3
"""Markdown table of bench runs (each RUN_DIR/score.json from run.sh).

    python3 scripts/slam_bench/summarize.py /tmp/bench/b1/*/
"""
import json
import sys
from pathlib import Path

COLS = [
    ("odom_hz", "odom Hz"),
    ("odom_coverage", "frames tracked"),
    ("odom_lost_msgs", "lost"),
    ("odom_ate_m", "odom ATE m"),
    ("odom_drift_pct", "odom drift %"),
    ("slam_ate_m", "SLAM ATE m"),
    ("obstacle_latency_p50_ms", "obst p50 ms"),
    ("obstacle_latency_p99_ms", "obst p99 ms"),
    ("link_mbit_s", "link Mb/s"),
]


def main(argv):
    rows = []
    for d in argv:
        f = Path(d) / "score.json"
        if f.exists():
            rows.append((Path(d).name, json.loads(f.read_text())))
    print("| run | " + " | ".join(h for _, h in COLS) + " |")
    print("|---|" + "---|" * len(COLS))
    for name, s in sorted(rows):
        print(f"| {name} | " + " | ".join(
            "" if s.get(k) is None else str(s[k]) for k, _ in COLS) + " |")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
