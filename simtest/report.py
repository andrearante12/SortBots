"""Verdicts and the markdown report for a test run directory.

A run directory (data/simtests/<UTC>_<test>/) holds:

    test.json          the test exactly as it ran (after --set), every variant resolved
    results.jsonl      one line per run: {variant, repeat, ok, score | error, meta}
    report.md          rendered by render() below
    runs/<v>_<r>/      events.jsonl.gz, score.json, probe.log, sim/bringup log tails

Verdicts:
  run      every criterion of every metric held (simtest.metrics.MetricSet.score)
  variant  passed runs / runs >= the test's pass_rate. A run that never got
           scored (the sim didn't come up, the probe crashed) counts against
           it — "we couldn't tell" is not a pass — and is listed as an error.
  test     every variant passed

Values are aggregated per variant as mean [min–max] across repeats. When the
runs being compared were scored by different VERSIONS of a metric (an old run
dir next to a new one, or a partial rescore), the report says so above the
table instead of silently averaging two definitions.
"""
from __future__ import annotations

import json
import statistics
from pathlib import Path

from .metrics import OP_TEXT


def load_results(run_dir: Path) -> list[dict]:
    p = Path(run_dir) / "results.jsonl"
    if not p.exists():
        return []
    out = []
    for line in p.read_text().splitlines():
        if line.strip():
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def verdicts(results: list[dict], pass_rate: float, variants: list[str] | None = None) -> dict:
    """{variant: {runs, passed, errors, ok}} plus "_all": overall ok."""
    order = variants or list(dict.fromkeys(r["variant"] for r in results))
    out = {}
    for v in order:
        rs = [r for r in results if r["variant"] == v]
        passed = sum(1 for r in rs if r.get("ok"))
        errors = sum(1 for r in rs if "error" in r)
        out[v] = {"runs": len(rs), "passed": passed, "errors": errors,
                  "ok": bool(rs) and passed / len(rs) >= pass_rate - 1e-9}
    out["_all"] = bool(order) and all(out[v]["ok"] for v in order)
    return out


def _fmt(v) -> str:
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.3g}" if abs(v) < 1000 else f"{v:.0f}"
    return str(v)


def _agg(vals: list) -> str:
    nums = [v for v in vals if isinstance(v, (int, float)) and not isinstance(v, bool)]
    if not vals:
        return "—"
    if len(nums) != len(vals) or not nums:
        uniq = list(dict.fromkeys(_fmt(v) for v in vals))
        return ", ".join(uniq)
    if len(nums) == 1 or min(nums) == max(nums):
        return _fmt(nums[0])
    return f"{_fmt(statistics.fmean(nums))} [{_fmt(min(nums))}–{_fmt(max(nums))}]"


def version_notes(results: list[dict]) -> list[str]:
    seen: dict[str, set] = {}
    for r in results:
        for m in (r.get("score") or {}).get("metrics", []):
            seen.setdefault(m["id"], set()).add(m["version"])
    return [f"`{mid}` was scored with versions {sorted(vs)} across these runs — "
            f"values below mix definitions; `scripts/simtest.py rescore` the older runs"
            for mid, vs in sorted(seen.items()) if len(vs) > 1]


def render(test: dict, results: list[dict], *, wall_s: float | None = None,
           title_suffix: str = "") -> str:
    variants = [v["label"] for v in test["variants"]]
    vd = verdicts(results, test["pass_rate"], variants)
    head = "PASS" if vd["_all"] else "FAIL"
    n_runs = len(results)
    n_pass = sum(1 for r in results if r.get("ok"))
    lines = [f"# {test['name']} — {head}{title_suffix}", "",
             f"{test['title']}", "",
             f"{n_pass}/{n_runs} run(s) passed · pass_rate {test['pass_rate']:g} per variant"
             + (f" · {wall_s / 60:.1f} min wall" if wall_s else ""), ""]

    lines += ["| variant | runs | passed | errors | verdict |", "|---|---|---|---|---|"]
    for v in variants:
        d = vd[v]
        lines.append(f"| {v} | {d['runs']} | {d['passed']} | {d['errors']} | "
                     f"{'PASS' if d['ok'] else 'FAIL'} |")

    notes = version_notes(results)
    if notes:
        lines += ["", "> **Metric versions differ:**"] + [f"> - {n}" for n in notes]

    # criteria: one row per (variant, metric id, criterion), value per repeat
    lines += ["", "## Criteria", "",
              "| variant | metric | criterion | per run | held |", "|---|---|---|---|---|"]
    for v in variants:
        rs = [r for r in results if r["variant"] == v and "score" in r]
        if not rs:
            continue
        for m0 in rs[0]["score"]["metrics"]:
            for i, c0 in enumerate(m0["criteria"]):
                per, held = [], 0
                for r in rs:
                    m = next((x for x in r["score"]["metrics"] if x["id"] == m0["id"]), None)
                    c = m["criteria"][i] if m and i < len(m["criteria"]) else None
                    per.append("no data" if c is None or c["no_data"] else _fmt(c["actual"]))
                    held += bool(c and c["ok"])
                lines.append(f"| {v} | {m0['id']} | {c0['value']} {OP_TEXT[c0['op']]} "
                             f"{_fmt(c0['threshold'])} | {', '.join(per)} | "
                             f"{held}/{len(rs)} |")

    # every value, aggregated
    lines += ["", "## Values", "", "mean [min–max] across repeats.", ""]
    for v in variants:
        rs = [r for r in results if r["variant"] == v and "score" in r]
        if not rs:
            continue
        if len(variants) > 1:
            lines += [f"### {v}", ""]
        for m0 in rs[0]["score"]["metrics"]:
            keys = list(m0["values"])
            lines += [f"**{m0['id']}** (`{m0['metric']}` v{m0['version']})", "",
                      "| " + " | ".join(keys) + " |", "|" + "---|" * len(keys)]
            cols = []
            for k in keys:
                vals = []
                for r in rs:
                    m = next((x for x in r["score"]["metrics"] if x["id"] == m0["id"]), None)
                    if m is not None:
                        vals.append(m["values"].get(k))
                cols.append(_agg(vals))
            lines += ["| " + " | ".join(cols) + " |", ""]

    errs = [r for r in results if "error" in r]
    if errs:
        lines += ["## Errors", ""]
        lines += [f"- [{r['variant']} #{r['repeat']}] {r['error']}" for r in errs]
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def summary(run_dir: Path) -> dict | None:
    """Compact JSON for the dashboard's results list."""
    run_dir = Path(run_dir)
    try:
        test = json.loads((run_dir / "test.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    results = load_results(run_dir)
    variants = [v["label"] for v in test["variants"]]
    planned = len(variants) * test["repeats"]
    vd = verdicts(results, test["pass_rate"], variants)
    rows = []
    for r in results:
        crit = []
        for m in (r.get("score") or {}).get("metrics", []):
            for c in m["criteria"]:
                crit.append({"label": f"{m['id']}.{c['value']} {OP_TEXT[c['op']]} "
                                      f"{_fmt(c['threshold'])}",
                             "actual": None if c["no_data"] else c["actual"], "ok": c["ok"]})
        rows.append({"variant": r["variant"], "repeat": r["repeat"], "ok": bool(r.get("ok")),
                     "error": r.get("error"), "criteria": crit,
                     "sim_s": (r.get("meta") or {}).get("sim_s"),
                     "stop_reason": (r.get("meta") or {}).get("stop_reason")})
    state = (json.loads((run_dir / "state.json").read_text())
             if (run_dir / "state.json").exists() else {})
    return {"dir": run_dir.name, "name": test["name"], "title": test["title"],
            "planned": planned, "done": len(results),
            "complete": (run_dir / "report.md").exists(),
            "verdict": ("PASS" if vd["_all"] else "FAIL") if results else None,
            "variants": {v: vd[v] for v in variants}, "runs": rows,
            "state": state.get("state"), "started": state.get("started")}


def recent(results_dir: Path, limit: int = 20) -> list[dict]:
    results_dir = Path(results_dir)
    if not results_dir.is_dir():
        return []
    dirs = sorted((d for d in results_dir.iterdir() if (d / "test.json").exists()),
                  key=lambda d: d.name, reverse=True)[:limit]
    return [s for s in (summary(d) for d in dirs) if s]
