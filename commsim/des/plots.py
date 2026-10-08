"""Headline charts from DES sweep results (needs matplotlib; not a repo dep).

    python3 -m commsim.des.plots commsim/results/FINAL --out commsim/docs/figures

Palette: reference categorical slots 1-5 (validated: CVD ΔE ≥ 9.1, normal
ΔE ≥ 19.6; three hues below 3:1 on the surface → every line is direct-labeled
and every number is also in the sim_log tables). Ordinal blue ramp for load.
Filled marker = all seeds pass; open marker = majority of seeds fail.
"""
import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
ORDINAL = ["#86b6ef", "#2a78d6", "#104281"]
DESIGNS = ["V1", "V2", "V2+LB+SC", "V3+LB+SC", "V4+LB+SC"]  # fixed order = fixed color
BG_DESIGNS = ["V2+LB+SC", "V3+LB+SC", "V4+LB+SC"]


def load(d):
    return [json.loads(l) for l in open(Path(d) / "runs.jsonl")]


def agg(rows, label, bg=0.0, key="busy_g24"):
    g = defaultdict(list)
    for r in rows:
        if r["label"] == label and abs(r["bg_share"] - bg) < 1e-9:
            g[r["n"]].append(r)
    out = []
    for n in sorted(g):
        rs = g[n]
        vals = [r[key] for r in rs if r[key] == r[key]]
        fail = sum(1 for r in rs if r["failed"]) * 2 > len(rs)
        out.append((n, sum(vals) / len(vals) if vals else math.nan, fail))
    return out


def style(ax, title, ylabel):
    fig = ax.figure
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", color=INK, fontsize=12, pad=12)
    ax.set_xlabel("robots in fleet (n)", color=INK2)
    ax.set_ylabel(ylabel, color=INK2)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK2)


def lines(rows, key, ylabel, title, ref, ref_label, out, log=False, scale=1.0):
    fig, ax = plt.subplots(figsize=(8, 4.6), dpi=150)
    style(ax, title, ylabel)
    ends = []
    for i, lab in enumerate(DESIGNS):
        pts = [(n, v * scale, f) for n, v, f in agg(rows, lab, key=key) if v == v]
        if not pts:
            continue
        c = SERIES[i]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=c, linewidth=2, label=lab)
        for n, v, f in pts:
            ax.plot(n, v, "o", markersize=8, color=c,
                    markerfacecolor=SURFACE if f else c, markeredgewidth=2,
                    markeredgecolor=SURFACE if not f else c, zorder=3)
        ends.append((pts[-1][0], pts[-1][1], lab))
    # end labels, nudged apart vertically when two lines end close together
    placed = []
    for n, v, lab in sorted(ends, key=lambda e: e[1]):
        dy = 0
        for pn, pv, pdy in placed:
            if pn == n and abs((math.log10(v) if log else v) - (math.log10(pv) if log else pv)) < (0.12 if log else 3):
                dy = pdy + 10
        placed.append((n, v, dy))
        ax.annotate(lab, (n, v), xytext=(6, dy), textcoords="offset points",
                    va="center", fontsize=8, color=INK2)
    ax.axhline(ref, color=INK2, linewidth=1, linestyle=(0, (4, 3)))
    ax.text(ax.get_xlim()[0], ref, f" {ref_label}", va="bottom", fontsize=8, color=INK2)
    if log:
        ax.set_yscale("log")
    ax.legend(frameon=False, fontsize=8, labelcolor=INK2, loc="upper left")
    fig.text(0.01, 0.01, "DES model, warehouse placeholder layout, no background load, 3 seeds. "
             "Open marker = majority of seeds fail; below n=10 the floor is coverage-limited.",
             fontsize=7, color=INK2)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(out, facecolor=SURFACE)
    plt.close(fig)


def supported(rows, lab, bg, n_min=10):
    """Largest n before the first n (>= n_min) where a majority of seeds fail —
    the same rule the sweeps use. Below n_min the floor is coverage-limited."""
    pts = [(n, f) for n, _, f in agg(rows, lab, bg=bg) if n >= n_min]
    best = n_min - 1
    for n, f in pts:
        if f:
            return best
        best = n
    return best


def bg_bars(rows, out):
    designs = BG_DESIGNS
    bgs = [0.0, 0.15, 0.30]
    fig, ax = plt.subplots(figsize=(7, 4.2), dpi=150)
    style(ax, "Largest fleet supported, by campus background load", "robots supported (n)")
    ax.set_xlabel("design", color=INK2)
    w = 0.26
    top = 0
    for j, bg in enumerate(bgs):
        for i, lab in enumerate(designs):
            best = supported(rows, lab, bg)
            top = max(top, best)
            x = i + (j - 1) * w
            ax.bar(x, best, width=w - 0.03, color=ORDINAL[j],
                   label=f"{int(bg * 100)}% background" if i == 0 else None)
            ax.text(x, best + 0.3, "<10" if best < 10 else str(best), ha="center",
                    fontsize=8, color=INK2)
    ax.set_xticks(range(len(designs)), designs)
    ax.set_ylim(0, top + 5)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK2, loc="upper left", ncols=3)
    fig.text(0.01, 0.01, "DES, warehouse placeholder, 3 seeds; supported = last n before a "
             "majority of seeds fail (n >= 10).", fontsize=7, color=INK2)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(out, facecolor=SURFACE)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--out", type=Path, default=Path("commsim/docs/figures"))
    ap.add_argument("--designs", nargs="*", default=None, help="labels for the line charts (<= 5)")
    ap.add_argument("--bg-designs", nargs="*", default=None, help="labels for the background chart")
    ap.add_argument("--suffix", default="", help="filename suffix, e.g. _fixed")
    a = ap.parse_args()
    global DESIGNS, BG_DESIGNS
    DESIGNS = a.designs or DESIGNS
    BG_DESIGNS = a.bg_designs or BG_DESIGNS
    rows = [r for d in a.dirs for r in load(d)]
    a.out.mkdir(parents=True, exist_ok=True)
    lines(rows, "busy_g24", "2.4 GHz channel busy time (%)",
          "Channel busy time vs fleet size", 50, "50% failure threshold",
          a.out / f"busy_vs_n{a.suffix}.png", scale=100)
    lines(rows, "ctl_p95_ms", "control-message latency p95 (ms, log)",
          "Control latency p95 vs fleet size", 150, "150 ms requirement",
          a.out / f"p95_vs_n{a.suffix}.png", log=True)
    bg_bars(rows, a.out / f"bg_failure{a.suffix}.png")
    print("wrote", *sorted(p.name for p in a.out.glob("*.png")))


if __name__ == "__main__":
    main()
