"""fleetsim command line. System python3 only — no ROS, no GPU, no Isaac.

    /usr/bin/python3 -m fleetsim list
    /usr/bin/python3 -m fleetsim validate [NAME ...]        # exit 2 if any is invalid
    /usr/bin/python3 -m fleetsim show NAME [--set k=v ...]  # the resolved config
    /usr/bin/python3 -m fleetsim run NAME [--robots N] [--seed S] [--duration S]
                                         [--set k=v ...] [--out DIR] [--quiet]
    /usr/bin/python3 -m fleetsim rescore DIR                # re-score recorded events

A run writes data/fleetsim/<UTC>_<name>/ (gitignored):

    scenario.json      the scenario exactly as it ran, fully resolved
    events.jsonl.gz    every event (simtest/events.py format)
    summary.json       run summary: sim/wall seconds, speed-up, counters
    score.json         metric values and pass/fail per criterion

Exit codes: 0 every criterion passed (or none given), 1 a criterion failed,
2 invalid scenario or arguments.
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from fleetsim import config  # noqa: E402
from simtest import events as ev_lib  # noqa: E402
from simtest import metrics as metrics_lib  # noqa: E402
from simtest import robots as robots_lib  # noqa: E402
from simtest import world as world_lib  # noqa: E402

RESULTS_DIR = config.RESULTS_DIR


def _sets(items) -> list[tuple[str, object]]:
    out = []
    for it in items or []:
        if "=" not in it:
            raise config.ScenarioError(f"--set {it!r}: expected key=value")
        k, v = it.split("=", 1)
        out.append((k.strip(), yaml.safe_load(v)))
    return out


def _resolve(args) -> dict:
    return config.load(args.name, sets=_sets(args.set), robots=args.robots, seed=args.seed,
                       duration=args.duration)


def score(scn: dict, events_path: Path) -> dict | None:
    entries = scn.get("metrics_resolved") or (metrics_lib.validate_list(scn["metrics"])
                                              if scn.get("metrics") else None)
    if not entries:
        return None
    announced = [ev for ev in ev_lib.read(events_path) if ev["s"] == "announce"]
    world = {"actors": scn.get("actors") or [], "props": []}
    ctx = metrics_lib.Context(robots=[f"robot_{k}" for k in range(scn["config"]["robots"]["count"])],
                              world=world, footprint=robots_lib.footprint_points(),
                              tasks=announced)
    ms = metrics_lib.MetricSet(entries, ctx)
    for ev in ev_lib.read(events_path):
        ms.feed(ev)
    return ms.score()


def print_score(sc: dict | None) -> None:
    if sc is None:
        print("no metrics in this scenario")
        return
    for m in sc["metrics"]:
        mark = "PASS" if m["ok"] else "FAIL"
        print(f"  [{mark}] {m['id']} (v{m['version']})")
        crit = {c["value"]: c for c in m["criteria"]}
        for k, v in m["values"].items():
            c = crit.get(k)
            tag = ""
            if c is not None:
                tag = f"   {'ok ' if c['ok'] else 'BAD'} {c['op']} {c['threshold']}"
            print(f"      {k:<22} {v!s:<12}{tag}")
    print(f"verdict: {'PASS' if sc['ok'] else 'FAIL'}")


def cmd_run(args) -> int:
    from fleetsim import sim as sim_lib

    scn = _resolve(args)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = Path(args.out) if args.out else RESULTS_DIR / f"{stamp}_{scn['name']}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "scenario.json").write_text(json.dumps(scn, indent=2, default=str))
    rec = ev_lib.Recorder(out / "events.jsonl.gz")
    n, net = scn["config"]["robots"]["count"], scn["config"]["network"]["model"]
    if not args.quiet:
        print(f"fleetsim {scn['name']}: {n} robots, {net} network, "
              f"{scn['duration_s']:.0f} sim-s, seed {scn['seed']} -> {out}")
    try:
        s = sim_lib.Sim(scn, recorder=rec)
        summary = s.run()
    finally:
        rec.close()
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    for note in summary.get("notes", []):
        print(f"note: {note}")
    if not args.quiet:
        print(f"ran {summary['sim_s']} sim-s in {summary['wall_s']} s "
              f"({summary['speedup']}x real time), stopped: {summary['reason']}")
    sc = score(scn, out / "events.jsonl.gz")
    if sc is not None:
        (out / "score.json").write_text(json.dumps(sc, indent=2))
    print_score(sc)
    return 0 if sc is None or sc["ok"] else 1


def cmd_rescore(args) -> int:
    d = Path(args.dir)
    scn = json.loads((d / "scenario.json").read_text())
    scn.pop("metrics_resolved", None)  # re-validate against the current metric code
    sc = score(scn, d / "events.jsonl.gz")
    if sc is not None:
        (d / "score.json").write_text(json.dumps(sc, indent=2))
    print_score(sc)
    return 0 if sc is None or sc["ok"] else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="fleetsim", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    v = sub.add_parser("validate")
    v.add_argument("names", nargs="*")
    for name in ("show", "run"):
        p = sub.add_parser(name)
        p.add_argument("name")
        p.add_argument("--set", action="append", metavar="KEY=VALUE")
        p.add_argument("--robots", type=int)
        p.add_argument("--seed", type=int)
        p.add_argument("--duration", type=float)
        if name == "run":
            p.add_argument("--out")
            p.add_argument("--quiet", action="store_true")
    r = sub.add_parser("rescore")
    r.add_argument("dir")
    args = ap.parse_args(argv)
    try:
        if args.cmd == "list":
            for s in config.list_scenarios():
                if s["status"] == "ready":
                    print(f"{s['name']:<24} {s['robots']:>4} robots  {s['network']:<6} {s['title']}")
                else:
                    print(f"{s['name']:<24} INVALID: {s['error']}")
            return 0
        if args.cmd == "validate":
            names = args.names or [p.stem for p in sorted(config.SCENARIOS_DIR.glob("*.yaml"))]
            bad = 0
            for nm in names:
                try:
                    config.load(nm)
                    print(f"ok       {nm}")
                except config.ScenarioError as e:
                    bad += 1
                    print(f"INVALID  {nm}: {e}")
            return 2 if bad else 0
        if args.cmd == "show":
            scn = _resolve(args)
            scn.pop("metrics_resolved", None)
            print(yaml.safe_dump(scn, sort_keys=False))
            return 0
        if args.cmd == "run":
            return cmd_run(args)
        if args.cmd == "rescore":
            return cmd_rescore(args)
    except (config.ScenarioError, metrics_lib.MetricError, world_lib.WorldError,
            FileNotFoundError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":
    sys.exit(main())
