"""Summarize DES sweep results: per (label, bg, n), seed means + failures.

    python3 -m commsim.des.summarize commsim/results/A_spec [more dirs...]
"""
import json
import math
import sys
from collections import defaultdict


def load(dirs):
    rows = []
    for d in dirs:
        rows += [json.loads(l) for l in open(f"{d}/runs.jsonl")]
    return rows


def mean(xs):
    xs = [x for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return sum(xs) / len(xs) if xs else math.nan


def table(rows):
    g = defaultdict(list)
    for r in rows:
        g[(r["label"], r["bg_share"], r["n"])].append(r)
    out = []
    hdr = (f"{'label':<11}{'bg':>5}{'n':>4}{'busy24':>8}{'busy5':>7}{'p95ms':>9}{'p99ms':>9}"
           f"{'deliv':>7}{'conn':>6}{'leaseX':>7}{'dup':>5}  failed (seeds)")
    out.append(hdr)
    for (lab, bg, n), rs in sorted(g.items()):
        fails = defaultdict(int)
        for r in rs:
            for c in r["failed"]:
                fails[c] += 1
        nf = sum(1 for r in rs if r["failed"])
        out.append(f"{lab:<11}{bg:>5.2f}{n:>4}{mean([r['busy_g24'] for r in rs]):>8.3f}"
                   f"{mean([r['busy_g5'] for r in rs]):>7.3f}{mean([r['ctl_p95_ms'] for r in rs]):>9.1f}"
                   f"{mean([r['ctl_p99_ms'] for r in rs]):>9.1f}{mean([r['status_delivery'] for r in rs]):>7.3f}"
                   f"{mean([r.get('conn_frac') for r in rs]):>6.3f}{mean([r['lease_expired'] for r in rs]):>7.1f}"
                   f"{mean([r['dup_episodes'] for r in rs]):>5.1f}  {nf}/{len(rs)} "
                   + ",".join(f"{c}x{k}" for c, k in sorted(fails.items())))
    return "\n".join(out)


if __name__ == "__main__":
    print(table(load(sys.argv[1:])))
