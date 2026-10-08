"""Pass table for scenario runs, one column per --tag.

    python3 -m commsim.des.scen_table commsim/results/FIX/scen
"""
import collections
import json
import sys


def main(d):
    res = collections.defaultdict(lambda: collections.defaultdict(list))
    tags = []
    for l in open(f"{d}/runs.jsonl"):
        r = json.loads(l)
        res[(r["scenario"], r["variant"], r["n"])][r["tag"]].append(r)
        if r["tag"] not in tags:
            tags.append(r["tag"])
    print(f"{'scenario':<15}{'var':<4}{'n':>3}  " + "".join(f"{t:>11}" for t in tags))
    tot = {t: [0, 0] for t in tags}
    for k in sorted(res):
        cells = []
        for t in tags:
            rs = res[k][t]
            p = sum(r["pass"] for r in rs)
            cells.append(f"{p}/{len(rs)}" if rs else "-")
            if k[0] != "timer_misorder":
                tot[t][0] += p
                tot[t][1] += len(rs)
        print(f"{k[0]:<15}{k[1]:<4}{k[2]:>3}  " + "".join(f"{c:>11}" for c in cells))
    print("total (excl. negative control)  " + "".join(f"{f'{a}/{b}':>11}" for a, b in tot.values()))


if __name__ == "__main__":
    main(sys.argv[1])
