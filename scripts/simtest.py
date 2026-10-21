#!/usr/bin/env python3
"""Run scored sim test cases, and re-score or compare their results.

    scripts/simtest.py list                       # configs/tests/*.yaml, with status
    scripts/simtest.py validate [NAME ...]        # exit 2 if any is invalid
    scripts/simtest.py show NAME                  # the resolved plan, per variant
    scripts/simtest.py metrics                    # the metric catalog (simtest/metrics)
    scripts/simtest.py run NAME [--set k=v ...] [--repeats N] [--variant LABEL]
                                [--dry-run] [--force] [--keep]
    scripts/simtest.py rescore DIR [--current-spec]   # re-score recorded events
    scripts/simtest.py report DIR [DIR ...]       # re-render / compare result dirs

Results land in data/simtests/<UTC>_<name>/ (gitignored):

    test.json          the test exactly as it ran, every variant resolved
    results.jsonl      one line per run: variant, repeat, ok, score | error, meta
    report.md          verdicts, criteria per run, every value (simtest/report.py)
    state.json         progress, for the dashboard's Tests panel
    runs/<v>_<r>/      events.jsonl.gz (the full scored stream), score.json,
                       probe.log, sim.log / bringup.log tails

A live run is an ordinary dashboard session — it starts through
scripts/sim_ctl.sh, so you can watch it on the dashboard while it runs: the
map shows the actors' true positions, the session pane the log.

`rescore` is why metrics never touch ROS: change how a metric is calculated,
bump its version, and re-score every recorded run without a sim. By default it
keeps the criteria the run was judged by; --current-spec re-reads the test's
YAML for the metric list and thresholds instead.

Exit codes: 0 every variant passed, 1 a verdict failed, 2 invalid test /
bad arguments, 3 infrastructure (the sim or probe never produced a score
for any run), 130 interrupted.

Run from a CLEAN shell like everything else that launches the sim — the
probe sources ROS in its own subshell (scripts/ros_bash.sh when present).
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from simtest import events as ev_lib  # noqa: E402
from simtest import metrics as metrics_lib  # noqa: E402
from simtest import report as report_lib  # noqa: E402
from simtest import spec  # noqa: E402

SIM_CTL = REPO_ROOT / "scripts" / "sim_ctl.sh"
# Stand-in for `bash` wherever ROS is sourced: the distrobox fallback on hosts
# with no /opt/ros (Ubuntu 26.04). Plain bash where it doesn't exist yet.
ROS_BASH = REPO_ROOT / "scripts" / "ros_bash.sh"
ROS_SETUP = "/opt/ros/jazzy/setup.bash"
SYSTEM_PATH = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
LOG_TAILS = {"sim.log": Path("/tmp/sortbots_demo_sim.log"),
             "bringup.log": Path("/tmp/sortbots_demo_bringup.log")}
START_TIMEOUT_S = 900   # first-ever run streams the NVIDIA warehouse


def log(msg: str) -> None:
    print(msg, flush=True)


def _parse_sets(items: list[str]) -> list[tuple[str, object]]:
    out = []
    for kv in items:
        if "=" not in kv:
            raise spec.SpecError(f"--set {kv!r}: expected key=value")
        k, v = kv.split("=", 1)
        out.append((k, yaml.safe_load(v)))
    return out


def _jsonable(obj):
    return json.loads(json.dumps(obj, default=list))


# ---------------------------------------------------------------- listing

def cmd_list(_a) -> int:
    rows = spec.list_tests()
    if not rows:
        log(f"(no tests in {spec.TESTS_DIR})")
    for r in rows:
        if r["status"] != "ready":
            log(f"! {r['name']:<22} INVALID  {r['error']}")
            continue
        log(f"• {r['name']:<22} {r['runs']:>2} run(s)  {r['scenario']}"
            f"{'/' + r['world'] if r.get('world') else ''}  {r['title']}")
    return 0


def cmd_validate(a) -> int:
    names = a.names or [p.stem for p in sorted(spec.TESTS_DIR.glob("*.yaml"))]
    bad = 0
    for n in names:
        try:
            t = spec.load(n)
            log(f"ok       {n}  ({len(t['variants'])} variant(s) x {t['repeats']})")
        except spec.SpecError as e:
            bad += 1
            log(f"INVALID  {n}: {e}")
    return 2 if bad else 0


def _overrides_args(resolved: dict) -> list[str]:
    return [f"{k}={str(v).lower() if isinstance(v, bool) else v}"
            for k, v in resolved["overrides"].items()]


def cmd_show(a) -> int:
    try:
        t = spec.load(a.name, _parse_sets(a.set))
    except spec.SpecError as e:
        log(f"INVALID: {e}")
        return 2
    log(f"{t['name']}: {t['title']}")
    log(f"  {len(t['variants'])} variant(s) x {t['repeats']} repeat(s), "
        f"pass_rate {t['pass_rate']:g}")
    for label, r in t["variants"]:
        log(f"\n[{label}]  scripts/sim_ctl.sh start {r['scenario']} "
            + " ".join(_overrides_args(r)))
        w = r["world"]
        log(f"  world: {w['name'] if w else '-'}"
            + (f" ({len(w['props'])} props, {len(w['actors'])} actors)" if w else ""))
        log(f"  robots: {', '.join(r['robots'])}   warmup {r['warmup_s']:g}s   "
            f"stop: sim_s {r['stop']['sim_s']:g}"
            + (f", until {r['stop']['until']} (+{r['stop']['settle_s']:g}s)"
               if r["stop"]["until"] else ""))
        for g in r["goals"]:
            log(f"  goal {g['id']}: {g['robot']} -> {g['station'] or g['pose']}  "
                f"(at_s>={g['at_s']:g}, timeout {g['timeout_s']:g}s)")
        for tk in r["tasks"]:
            log(f"  task {tk['id']}: {tk['robot']} {tk['pickup']} -> {tk['dropoff']} "
                f"(at_s>={tk['at_s']:g})")
        for e in r["metrics"]:
            crit = ", ".join(f"{v} {metrics_lib.OP_TEXT[op]} {th}" for v, op, th in e["pass"])
            log(f"  metric {e['id']} ({e['metric']} v{metrics_lib.get(e['metric']).version})"
                + (f": {crit}" if crit else ": (reported, not judged)"))
        log(f"  streams: {sorted(spec.streams_for(t, r))}")
    return 0


def cmd_metrics(_a) -> int:
    for c in metrics_lib.catalog():
        log(f"\n{c['name']} v{c['version']} — {c['summary']}")
        log(f"  reads: {', '.join(c['streams'])}")
        for k, p in c["params"].items():
            log(f"  param {k} ({p['type']}, default {p['default']!r}): {p['doc']}")
        for k, doc in c["values"].items():
            log(f"  value {k}: {doc}")
    return 0


# ---------------------------------------------------------------- running

def _sim_ctl(*args, timeout=None) -> subprocess.CompletedProcess:
    return subprocess.run([str(SIM_CTL), *args], cwd=REPO_ROOT, capture_output=True,
                          text=True, timeout=timeout)


def _pipeline_alive() -> bool:
    return subprocess.run(["pgrep", "-f", r"spawn_warehouse\.py"],
                          capture_output=True).returncode == 0


def _domain() -> int:
    """ROS_DOMAIN_ID to probe on: the caller's (default 0, like every launcher).

    Inherited, never assumed: on a shared network another machine's SortBots
    stack on domain 0 is the same /robot_0 namespace, and a probe on the wrong
    domain would score — and send goals to — someone else's robot (seen
    2026-10-21: a teammate's full stack visible from this laptop's Wi-Fi).
    """
    try:
        return max(0, min(232, int(os.environ.get("ROS_DOMAIN_ID", "0"))))
    except ValueError:
        return 0


def _ros_shell(cmd: str) -> list[str]:
    shell = str(ROS_BASH) if ROS_BASH.exists() else "bash"
    body = (f"source {shlex.quote(ROS_SETUP)}; "
            f"export PATH={shlex.quote(SYSTEM_PATH)}:\"$PATH\"; "
            f"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID={_domain()}; "
            f"cd {shlex.quote(str(REPO_ROOT))}; {cmd}")
    return [shell, "-c", body]


class State:
    """data/simtests/<dir>/state.json — what the dashboard's Tests panel polls."""

    def __init__(self, path: Path, planned: int):
        self.path = path
        self.data = {"state": "running", "pid": os.getpid(), "planned": planned,
                     "done": 0, "current": None,
                     "started": datetime.now(timezone.utc).isoformat(timespec="seconds")}
        self.write()

    def update(self, **kw):
        self.data.update(kw)
        self.write()

    def write(self):
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.data, indent=2))
        tmp.replace(self.path)


def run_once(label: str, resolved: dict, streams: set[str], run_dir: Path, a,
             state: State) -> dict:
    """One sim run: start -> wait running -> probe -> stop. Always stops."""
    if _sim_ctl("status").returncode == 3:
        log("  console not up — starting it")
        if _sim_ctl("console", "start", timeout=240).returncode != 0:
            return {"error": "console start failed (scripts/sim_ctl.sh console start)"}
    started = time.monotonic()
    probe_rc = None
    try:
        args = ["start", resolved["scenario"], *_overrides_args(resolved)]
        log(f"  sim_ctl {' '.join(args)}")
        state.update(current={**state.data["current"], "phase": "starting sim"})
        r = _sim_ctl(*args, timeout=180)
        if r.returncode != 0:
            return {"error": f"start failed: {(r.stdout + r.stderr).strip()[-400:]}"}
        r = _sim_ctl("wait", "running", "--timeout", str(START_TIMEOUT_S),
                     timeout=START_TIMEOUT_S + 60)
        if r.returncode != 0:
            return {"error": f"sim never reached running (sim_ctl exit {r.returncode}): "
                             f"{r.stdout.strip()[-400:]}"}
        log(f"  running after {time.monotonic() - started:.0f}s wall; probing")
        state.update(current={**state.data["current"], "phase": "probing"})

        vjson = run_dir / "variant.json"
        vjson.write_text(json.dumps({"label": label, "resolved": _jsonable(resolved),
                                     "streams": sorted(streams)}, indent=2))
        cmd = (f"exec python3 -m simtest.probe --variant {shlex.quote(str(vjson))} "
               f"--out {shlex.quote(str(run_dir))}")
        with open(run_dir / "probe.log", "w") as fh:
            p = subprocess.Popen(_ros_shell(cmd), cwd=REPO_ROOT, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True)
            try:
                for line in p.stdout:
                    fh.write(line)
                    fh.flush()
                    if line.startswith("[probe]"):
                        log("  " + line.rstrip())
                        if "phase " in line:
                            state.update(current={**state.data["current"],
                                                  "phase": line.split("phase ", 1)[1].strip()})
                probe_rc = p.wait()
            except KeyboardInterrupt:
                # Forward it explicitly: a terminal Ctrl-C reaches the whole
                # foreground group, but a SIGINT to this process alone (pkill,
                # a supervisor) would leave the probe running — and inside the
                # jazzy distrobox it is in another session entirely, reachable
                # only through ros_bash.sh's relay (found 2026-10-21: an
                # orphaned probe outlived its sim). Then give it time to write
                # score.json for what it saw.
                try:
                    p.send_signal(signal.SIGINT)
                    probe_rc = p.wait(timeout=30)
                except (subprocess.TimeoutExpired, ProcessLookupError):
                    p.kill()
                raise
        out = run_dir / "score.json"
        if not out.exists():
            return {"error": f"probe wrote no score (exit {probe_rc}); see probe.log"}
        body = json.loads(out.read_text())
        if probe_rc == 2:
            return {"error": f"probe: {body['meta']['stop_reason']} — see probe.log",
                    "meta": body["meta"]}
        return body
    finally:
        if not (a.keep and state.data.get("last")):
            log("  stopping the sim")
            _sim_ctl("stop", timeout=240)
        for name, src in LOG_TAILS.items():
            try:
                lines = src.read_text(errors="replace").splitlines()[-3000:]
                (run_dir / name).write_text("\n".join(lines) + "\n")
            except OSError:
                pass


def cmd_run(a) -> int:
    try:
        sets = _parse_sets(a.set)
        if a.repeats is not None:
            sets.append(("repeats", a.repeats))
        t = spec.load(a.name, sets)
    except spec.SpecError as e:
        log(f"INVALID: {e}")
        return 2
    variants = t["variants"]
    if a.variant:
        variants = [(l, r) for l, r in variants if l in a.variant]
        if not variants:
            log(f"no variant(s) {a.variant}; have {[l for l, _ in t['variants']]}")
            return 2
    total = len(variants) * t["repeats"]
    log(f"[simtest] {t['name']}: {len(variants)} variant(s) x {t['repeats']} repeat(s) "
        f"= {total} run(s)")
    for label, r in variants:
        log(f"  - {label}: {r['scenario']} {' '.join(_overrides_args(r))}")
    if a.dry_run:
        return 0
    if _pipeline_alive() and not a.force:
        log("[simtest] a sim is already running — refusing to tear it down. Stop it "
            "(scripts/sim_ctl.sh stop) or pass --force.")
        return 1

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = Path(a.out) if a.out else spec.RESULTS_DIR / f"{stamp}_{t['name']}"
    (out_dir / "runs").mkdir(parents=True, exist_ok=True)
    test_doc = {k: v for k, v in t.items() if k not in ("variants", "raw")}
    test_doc["variants"] = [{"label": l, "resolved": _jsonable(r)} for l, r in variants]
    test_doc["raw"] = t["raw"]
    test_doc["metric_versions"] = {c["name"]: c["version"] for c in metrics_lib.catalog()}
    (out_dir / "test.json").write_text(json.dumps(test_doc, indent=2, default=list))
    state = State(out_dir / "state.json", total)
    results_path = out_dir / "results.jsonl"
    log(f"[simtest] results: {out_dir}")

    t_all = time.monotonic()
    interrupted = False
    n = 0
    try:
        for vi, (label, resolved) in enumerate(variants):
            streams = spec.streams_for(t, resolved)
            for rep in range(t["repeats"]):
                n += 1
                run_dir = out_dir / "runs" / f"{vi}_{rep}"
                run_dir.mkdir(parents=True, exist_ok=True)
                log(f"\n[simtest] run {n}/{total}: [{label}] repeat {rep}")
                state.update(current={"variant": label, "repeat": rep, "n": n,
                                      "phase": "queued"}, last=n == total)
                try:
                    body = run_once(label, resolved, streams, run_dir, a, state)
                except KeyboardInterrupt:
                    # Keep the in-flight run in the record: a cancelled run
                    # that silently vanished would make the report read as
                    # fewer runs, not as a cancellation.
                    with open(results_path, "a") as fh:
                        fh.write(json.dumps({"variant": label, "repeat": rep,
                                             "run_dir": run_dir.name, "ok": False,
                                             "error": "interrupted"}) + "\n")
                    raise
                line = {"variant": label, "repeat": rep, "run_dir": run_dir.name}
                if "score" in body and "error" not in body:
                    line.update(ok=body["score"]["ok"], score=body["score"], meta=body["meta"])
                    verdict = "PASS" if body["score"]["ok"] else "FAIL"
                    log(f"[simtest] [{label}] repeat {rep}: {verdict}")
                    for m in body["score"]["metrics"]:
                        for c in m["criteria"]:
                            if not c["ok"]:
                                log(f"    ✗ {m['id']}.{c['value']} "
                                    f"{metrics_lib.OP_TEXT[c['op']]} {c['threshold']}: "
                                    f"{'no data' if c['no_data'] else c['actual']}")
                else:
                    line.update(ok=False, error=body.get("error", "unknown"),
                                meta=body.get("meta"))
                    log(f"[simtest] [{label}] repeat {rep}: ERROR {line['error']}")
                with open(results_path, "a") as fh:
                    fh.write(json.dumps(line, default=str) + "\n")
                state.update(done=n)
    except KeyboardInterrupt:
        interrupted = True
        log("[simtest] interrupted — writing a report for the runs that finished")

    results = report_lib.load_results(out_dir)
    md = report_lib.render(test_doc, results, wall_s=time.monotonic() - t_all,
                           title_suffix=" (interrupted)" if interrupted else "")
    (out_dir / "report.md").write_text(md)
    vd = report_lib.verdicts(results, t["pass_rate"], [l for l, _ in variants])
    state.update(state="cancelled" if interrupted else "done", current=None,
                 verdict="PASS" if vd["_all"] else "FAIL")
    log("\n" + md)
    log(f"[simtest] results: {out_dir}")
    if interrupted:
        return 130
    if results and all("error" in r for r in results):
        return 3
    return 0 if vd["_all"] else 1


# ---------------------------------------------------------------- after the fact

def cmd_rescore(a) -> int:
    d = Path(a.dir)
    test_doc = json.loads((d / "test.json").read_text())
    if a.current_spec:
        try:
            current = spec.load(test_doc["name"])
        except spec.SpecError as e:
            log(f"INVALID current spec: {e}")
            return 2
        cur = dict(current["variants"])
    results = report_lib.load_results(d)
    by_label = {v["label"]: v["resolved"] for v in test_doc["variants"]}
    out = []
    for r in results:
        ev_path = d / "runs" / r.get("run_dir", "") / "events.jsonl.gz"
        if "error" in r or not ev_path.exists():
            out.append(r)
            continue
        resolved = by_label[r["variant"]]
        if a.current_spec:
            if r["variant"] not in cur:
                log(f"  {r['variant']}: not a variant of the current spec — kept as was")
                out.append(r)
                continue
            resolved = {**resolved, "metrics": _jsonable(cur[r["variant"]]["metrics"])}
        need = metrics_lib.MetricSet(resolved["metrics"], spec.context(resolved)).streams()
        have = ev_lib.recorded_streams(ev_path)
        missing = need - have
        if missing:
            log(f"  {r['run_dir']}: can't rescore — stream(s) {sorted(missing)} weren't "
                f"recorded (add them to the test's `record:` for future runs)")
            out.append(r)
            continue
        ms = metrics_lib.MetricSet(resolved["metrics"], spec.context(resolved))
        for ev in ev_lib.read(ev_path):
            ms.feed(ev)
        score = ms.score()
        changed = score["ok"] != r.get("ok")
        log(f"  {r['run_dir']} [{r['variant']} #{r['repeat']}]: "
            f"{'PASS' if score['ok'] else 'FAIL'}"
            + (f" (was {'PASS' if r.get('ok') else 'FAIL'})" if changed else ""))
        out.append({**r, "ok": score["ok"], "score": score,
                    "rescored": datetime.now(timezone.utc).isoformat(timespec="seconds")})
    if a.dry_run:
        return 0
    backup = d / f"results.{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}.jsonl"
    (d / "results.jsonl").replace(backup)
    with open(d / "results.jsonl", "w") as fh:
        for r in out:
            fh.write(json.dumps(r, default=str) + "\n")
    if a.current_spec:
        test_doc["variants"] = [{"label": l, "resolved": {**res, "metrics":
                                 _jsonable(cur[l]["metrics"]) if l in cur else res["metrics"]}}
                                for l, res in ((v["label"], v["resolved"])
                                               for v in test_doc["variants"])]
    md = report_lib.render(test_doc, out, title_suffix=" (rescored)")
    (d / "report.md").write_text(md)
    log(f"\n{md}\n[simtest] previous results kept as {backup.name}")
    return 0


def cmd_report(a) -> int:
    if len(a.dirs) == 1:
        d = Path(a.dirs[0])
        test_doc = json.loads((d / "test.json").read_text())
        log(report_lib.render(test_doc, report_lib.load_results(d)))
        return 0
    # Comparison: every dir's variants become rows labelled <dir>:<variant>.
    merged, variants, name = [], [], None
    pass_rate = 1.0
    for d in map(Path, a.dirs):
        test_doc = json.loads((d / "test.json").read_text())
        name = name or test_doc["name"]
        pass_rate = test_doc["pass_rate"]
        for r in report_lib.load_results(d):
            merged.append({**r, "variant": f"{d.name}:{r['variant']}"})
        variants += [{"label": f"{d.name}:{v['label']}"} for v in test_doc["variants"]]
    log(report_lib.render({"name": name, "title": f"comparison of {len(a.dirs)} result dirs",
                           "pass_rate": pass_rate, "variants": variants}, merged))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="SortBots scored sim tests",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list").set_defaults(fn=cmd_list)
    sub.add_parser("metrics").set_defaults(fn=cmd_metrics)
    v = sub.add_parser("validate")
    v.add_argument("names", nargs="*")
    v.set_defaults(fn=cmd_validate)
    s = sub.add_parser("show")
    s.add_argument("name")
    s.add_argument("--set", action="append", default=[], metavar="K=V")
    s.set_defaults(fn=cmd_show)
    r = sub.add_parser("run")
    r.add_argument("name")
    r.add_argument("--set", action="append", default=[], metavar="K=V",
                   help="change a test field before validation, e.g. overrides.world=aisle_head_on "
                        "or stop.sim_s=60")
    r.add_argument("--repeats", type=int)
    r.add_argument("--variant", action="append", help="run only this variant (repeatable)")
    r.add_argument("--dry-run", action="store_true")
    r.add_argument("--force", action="store_true",
                   help="stop an already-running sim instead of refusing")
    r.add_argument("--keep", action="store_true",
                   help="leave the sim up after the LAST run, to look around")
    r.add_argument("--out", help="results dir (default data/simtests/<UTC>_<name>)")
    r.set_defaults(fn=cmd_run)
    rs = sub.add_parser("rescore")
    rs.add_argument("dir")
    rs.add_argument("--current-spec", action="store_true",
                    help="judge by the test YAML's CURRENT metrics/criteria, not the recorded ones")
    rs.add_argument("--dry-run", action="store_true", help="print verdicts, write nothing")
    rs.set_defaults(fn=cmd_rescore)
    rp = sub.add_parser("report")
    rp.add_argument("dirs", nargs="+")
    rp.set_defaults(fn=cmd_report)
    a = ap.parse_args(argv)
    # SIGTERM (dashboard cancel fallback) behaves like Ctrl-C: stop the sim cleanly.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
