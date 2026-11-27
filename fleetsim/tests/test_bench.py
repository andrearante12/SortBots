"""fleetsim bench (suites, retries, tables) and the compute log (sysmon.py).

Crash handling runs against a fake launcher, since real crashes can't be
ordered up; one tiny real suite checks the whole path end to end."""
from __future__ import annotations

import csv
import faulthandler
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from fleetsim import bench, sysmon  # noqa: E402


def raw_suite(**over):
    raw = {"name": "t", "scenarios": [{"scenario": "delivery_smoke", "robots": [2, 3]}],
           "seeds": [1, 2], "variants": {"default": {}, "intents": {"intent.enabled": True}},
           "headline": "fleet_tasks.per_hour", "also": ["nav.blocked_robot_s"]}
    raw.update(over)
    return raw


# ---- suites -------------------------------------------------------------------------

def test_a_suite_expands_to_every_combination():
    s = bench.resolve_suite(raw_suite())
    jobs = bench.plan(s)
    assert len(jobs) == 2 * 2 * 2
    assert {j["name"] for j in jobs} >= {"delivery_smoke_r2_s1_default", "delivery_smoke_r3_s2_intents"}
    argv = bench.run_argv(next(j for j in jobs if j["variant"] == "intents"), Path("/x"))
    assert argv[argv.index("--set") + 1] == "intent.enabled=true"
    assert argv[-3:] == ["--out", "/x", "--quiet"]
    assert [j["name"] for j in bench.plan(s, only=["*_r3_*"])] == [
        j["name"] for j in jobs if j["robots"] == 3]


def test_suite_shorthands():
    s = bench.resolve_suite(raw_suite(seeds=3, variants={"radio": ["network.model=radio"]},
                                      scenarios=[{"scenario": "walkers", "duration_s": 60}]))
    assert s["seeds"] == [1, 2, 3]
    assert s["variants"] == {"radio": [["network.model", "radio"]]}
    assert s["scenarios"][0]["robots"] == [12]  # the scenario's own count
    argv = bench.run_argv(bench.plan(s)[0], Path("/x"))
    assert argv[argv.index("--duration") + 1] == "60.0"


@pytest.mark.parametrize("over,match", [
    ({"bogus": 1}, "unknown key"),
    ({"name": "Bad Name"}, "name: must match"),
    ({"scenarios": []}, "at least one"),
    ({"scenarios": [{"scenario": "no_such"}]}, "no scenario 'no_such'"),
    ({"scenarios": [{"scenario": "delivery_smoke", "robot": 2}]}, "unknown key"),
    ({"scenarios": [{"scenario": "delivery_smoke", "robots": [0]}]}, "positive"),
    ({"scenarios": [{"scenario": "delivery_smoke"}, {"scenario": "delivery_smoke"}]}, "give one an id"),
    ({"scenarios": [{"scenario": "delivery_smoke", "set": {"world.cell_m": 5}}]}, "cell_m"),
    ({"variants": {"x": {"intent.nope": 1}}}, r"variant x: config\.intent\.nope: no such setting"),
    ({"variants": {"Bad Name": {}}}, "must match"),
    ({"variants": {"x": ["intent.enabled"]}}, "key=value"),
    ({"seeds": [1, 1]}, "distinct"),
    ({"seeds": []}, "seeds"),
    ({"headline": "per_hour"}, "<metric id>.<value>"),
    ({"headline": "fleet_tasks.per_hr"}, "has no value 'per_hr'"),
    ({"headline": "clearance.min_clearance_m"}, "has no metric 'clearance'"),
    ({"also": ["nosuch.x"]}, "no scenario in the suite has a metric 'nosuch'"),
    ({"also": "fleet_tasks.mean_s"}, "also: expected a list"),
])
def test_bad_suites_say_what_is_wrong(over, match):
    with pytest.raises(bench.SuiteError, match=match):
        bench.resolve_suite(raw_suite(**over))


def test_shipped_suites_are_valid():
    suites = bench.list_suites()
    assert "smoke" in {s["name"] for s in suites}
    assert all(s["status"] == "ready" for s in suites), suites


def test_inline_yaml_round_trips():
    for v in (True, 0.5, 3, "radius", "true", None, ["robot_0", "robot_1"], {"a": 1}):
        assert yaml.safe_load(bench.inline_yaml(v)) == v


# ---- tables -------------------------------------------------------------------------

def rec(var, seed, value, status="pass"):
    return {"name": f"a_r2_s{seed}_{var}", "scenario": "a", "robots": 2, "seed": seed,
            "variant": var, "status": status, "attempts": [{"kind": status}],
            "values": {} if value is None else {"m.v": value}}


TABLE_SUITE = {"name": "t", "scenarios": [{"id": "a", "robots": [2]}], "seeds": [1, 2, 3],
               "variants": {"x": [], "y": [], "z": []}, "headline": "m.v", "also": []}


def test_mean_and_sd_over_seeds():
    records = [rec("x", 1, 1.0), rec("x", 2, 2.0), rec("x", 3, 3.0),
               rec("y", 1, 10.0), rec("y", 2, None, status="crashed"), rec("y", 3, 14.0),
               rec("z", 1, None), rec("z", 2, None), rec("z", 3, None)]
    [row] = bench.aggregate(records, "m.v", TABLE_SUITE)
    x, y, z = (row["cells"][v] for v in "xyz")
    assert (x["mean"], x["sd"], x["n"], x["of"]) == (2.0, 1.0, 3, 3)
    assert y["mean"] == 12.0 and y["sd"] == pytest.approx(2.828, abs=1e-3) and y["n"] == 2
    assert bench.fmt_cell(x) == "2.00 ± 1.00"
    assert bench.fmt_cell(y) == "12.0 ± 2.8 (2/3)"
    assert bench.fmt_cell(z) == "-"
    assert bench.fmt_cell({"mean": 0.0, "sd": 0.0, "n": 3, "of": 3}) == "0.0 ± 0.0"
    assert bench.fmt_cell({"mean": 405.2, "sd": None, "n": 1, "of": 1}) == "405"
    table = bench.render_table([row], list("xyz"), "m.v")
    lines = table.splitlines()
    assert lines[0] == "### m.v" and "| a " in lines[-1] and "12.0 ± 2.8 (2/3)" in lines[-1]
    assert len({len(ln) for ln in lines[2:]}) == 1  # aligned columns


def test_flatten_reads_any_metric():
    sc = {"ok": True, "metrics": [{"id": "fleet_tasks", "values": {"per_hour": 40.0, "mean_s": None}},
                                  {"id": "x", "values": {"nested": {"a": 1}, "listy": [1, 2]}}]}
    assert bench.flatten(sc) == {"fleet_tasks.per_hour": 40.0, "fleet_tasks.mean_s": None,
                                 "x.nested.a": 1, "x.listy": "[1, 2]"}


@pytest.mark.parametrize("code,score,kind", [
    (0, {"ok": True}, "pass"), (1, {"ok": False}, "fail"), (1, None, "crash"), (0, None, "crash"),
    (2, None, "invalid"), (130, None, "stopped"), (-11, None, "crash"), (-9, {"ok": True}, "crash"),
    (bench.TIMEOUT, None, "crash"), (3, None, "crash"),
])
def test_classify(tmp_path, code, score, kind):
    if score is not None:
        (tmp_path / "score.json").write_text(json.dumps(score))
    assert bench.classify(code, tmp_path)[0] == kind
    if code == -11:
        assert bench.classify(code, tmp_path)[1] == "SIGSEGV"


def test_kernel_lines_match_the_process_only():
    lines = ["kernel: python3[1001]: segfault at 8 ip 0 sp 0 error 4 in python3.14[1,2+3] "
             "likely on CPU 5 (core 8, socket 0)",
             "kernel: python3[11001]: segfault at 8 likely on CPU 9 (core 18, socket 0)",
             "kernel: Out of memory: Killed process 2002 (python3)",
             "kernel: mce: [Hardware Error]: Machine check events logged",
             "kernel: usb 1-1: new device"]
    got = bench.kernel_lines_for(lines, [1001, 2002])
    assert got == [lines[0], lines[2], lines[3]]


# ---- retries and crash accounting, with a fake launcher ---------------------------

def job(name, seed=1, variant="x"):
    return {"name": name, "scenario": "a", "scenario_file": "a", "robots": 2, "seed": seed,
            "variant": variant, "duration_s": None, "sets": [], "cost": 1}


def fake_launcher(script):
    """script: run name -> one outcome per attempt (pass, fail, segv, segv_exit, noscore,
    invalid). The pid is 1000 * attempt + the job's place in the script."""
    calls: dict[str, int] = {}

    def launch(j, run_dir, runner):
        k = calls[j["name"]] = calls.get(j["name"], 0) + 1
        what = script[j["name"]][k - 1]
        pid = 1000 * k + list(script).index(j["name"]) + 1
        with open(run_dir / "system.jsonl", "w") as f:
            f.write(json.dumps({"type": "info", "pid": pid, "sampler_tid": pid + 1}) + "\n")
            f.write(json.dumps({"type": "sample", "cpu": 7, "core": 14, "sim_s": 12.5,
                                "phase": "sim"}) + "\n")
        (run_dir / "log.txt").write_text(f"attempt {k}\n")
        if what in ("pass", "fail"):
            (run_dir / "score.json").write_text(json.dumps(
                {"ok": what == "pass",
                 "metrics": [{"id": "m", "values": {"v": 10.0 * j["seed"]}}]}))
            (run_dir / "summary.json").write_text(json.dumps({"sim_s": 100.0, "reason": "duration"}))
        if what.startswith("segv"):
            (run_dir / "crash.txt").write_text("Fatal Python error: Segmentation fault\n")
        code = {"pass": 0, "fail": 1, "segv": -11, "segv_exit": -11, "noscore": 1, "invalid": 2}[what]
        return {"exit": code, "pid": pid, "t0": time.time() - 1.0, "t1": time.time(),
                "exit_cpu": 3 if what == "segv_exit" else None}

    return launch, calls


def segv_kernel(t0, t1):
    return (["kernel: python3[1001]: segfault at 8 likely on CPU 5 (core 8, socket 0)",
             "kernel: chrome[4242]: segfault at 0 likely on CPU 9 (core 18, socket 0)"], "fake")


def test_crashes_are_retried_and_counted(tmp_path):
    script = {"recovers": ["segv", "pass"], "never": ["noscore", "noscore", "noscore"],
              "invalid": ["invalid"], "fails": ["fail"], "exitcpu": ["segv_exit", "pass"]}
    launch, calls = fake_launcher(script)
    said = []
    r = bench.Runner(tmp_path, retries=2, launch=launch, kernel=segv_kernel, say=said.append,
                     kernel_wait_s=0)
    r.cores = {3: 6}
    jobs = [job("recovers"), job("never", seed=2), job("invalid", seed=3), job("fails", seed=4),
            job("exitcpu", seed=5)]
    records = {x["name"]: x for x in r.run(jobs, n_jobs=2)}
    assert calls == {"recovers": 2, "never": 3, "invalid": 1, "fails": 1, "exitcpu": 2}
    assert [records[n]["status"] for n in script] == ["pass", "crashed", "invalid", "fail", "pass"]
    # No kernel line for it: the CPU the dead process last ran on stands in.
    assert {k: records["exitcpu"]["attempts"][0][k] for k in ("cpu", "core", "cpu_src")} == {
        "cpu": 3, "core": 6, "cpu_src": "exit"}

    ok = records["recovers"]
    assert ok["values"] == {"m.v": 10.0} and ok["summary"]["sim_s"] == 100.0
    a1 = ok["attempts"][0]
    assert (a1["kind"], a1["reason"], a1["cpu"], a1["core"], a1["cpu_src"]) == (
        "crash", "SIGSEGV", 5, 8, "kernel")
    d = tmp_path / "recovers"
    assert (d / "score.json").is_file() and not (d / "crash.txt").exists()
    kern = (d / "attempt_1" / "kernel.txt").read_text()
    assert "python3[1001]" in kern and "chrome[4242]" not in kern
    assert (d / "attempt_1" / "crash.txt").is_file() and (d / "attempt_1" / "system.jsonl").is_file()

    never = records["never"]
    assert len(never["attempts"]) == 3 and all(a["cpu_src"] == "sample" for a in never["attempts"])
    assert sorted(p.name for p in (tmp_path / "never").iterdir()) == ["attempt_1", "attempt_2",
                                                                     "attempt_3"]
    assert not (tmp_path / "never" / "attempt_1" / "kernel.txt").exists()  # not a signal
    assert any("giving up" in s for s in said)

    cs = bench.crash_stats(list(records.values()))
    assert cs["attempts"] == 5 and cs["runs"] == 3
    assert cs["recovered"] == 2 and cs["recovered_after_attempts"] == {"2": 2}
    assert cs["unrecovered"] == 1
    assert cs["by_cpu"] == {"3": 1, "5": 1} and cs["by_core"] == {"6": 1, "8": 1}
    assert cs["by_cpu_sampled"] == {"7": 3}

    suite = {**TABLE_SUITE, "seeds": [1, 2, 3, 4, 5], "variants": {"x": []}}
    text = bench.write_outputs(tmp_path, {"started": "now"}, suite, list(records.values()))
    assert "2 recovered (2 on attempt 2), 1 never completed" in text
    rows = {row["run"]: row for row in csv.DictReader((tmp_path / "results.csv").open())}
    assert rows["recovers"]["crash_cpus"] == "5" and rows["recovers"]["attempts"] == "2"
    assert rows["never"]["crash_cpus"] == "7?;7?;7?" and rows["never"]["m.v"] == ""
    assert rows["fails"]["m.v"] == "40.0"
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["complete"] and summary["crashes"]["by_core"] == {"6": 1, "8": 1}


def test_unreadable_kernel_log_falls_back_to_the_last_sample(tmp_path):
    launch, _ = fake_launcher({"r": ["segv", "pass"]})
    r = bench.Runner(tmp_path, retries=1, launch=launch, say=lambda s: None, kernel_wait_s=0,
                     kernel=lambda t0, t1: (None, "journalctl: permission denied"))
    [x] = r.run([job("r")], n_jobs=1)
    assert x["status"] == "pass" and x["attempts"][0]["cpu_src"] == "sample"
    assert "not readable: journalctl: permission denied" in (
        tmp_path / "r" / "attempt_1" / "kernel.txt").read_text()


def test_a_stopped_bench_starts_nothing(tmp_path):
    launch, calls = fake_launcher({"r": ["pass"]})
    r = bench.Runner(tmp_path, launch=launch, say=lambda s: None)
    r.stop()
    [x] = r.run([job("r")], n_jobs=1)
    assert x["status"] == "skipped" and calls == {}


# ---- the compute log ----------------------------------------------------------------

def test_monitor_writes_info_samples_and_end(tmp_path):
    was = faulthandler.is_enabled()
    m = sysmon.Monitor(tmp_path, interval_s=0.05)
    m.clock = lambda: 42.0
    time.sleep(0.25)
    m.stop()
    m.stop()  # idempotent
    rows = sysmon.read(tmp_path / "system.jsonl")
    assert rows[0]["type"] == "info" and rows[0]["pid"] == os.getpid()
    assert rows[0]["python"] and "sampler_tid" in rows[0]
    samples = [r for r in rows if r["type"] == "sample"]
    assert len(samples) >= 2 and samples[-1]["sim_s"] == 42.0
    assert set(samples[0]) >= {"t", "cpu", "core", "mhz", "pkg_c", "core_c", "load", "rss_mb", "cpu_s"}
    assert rows[-1]["type"] == "end" and rows[-1]["outcome"] == "ok"
    assert not (tmp_path / "crash.txt").exists()
    assert faulthandler.is_enabled() == was


def test_monitor_keeps_the_traceback_of_an_exception(tmp_path):
    m = sysmon.Monitor(tmp_path, interval_s=10)
    try:
        raise TypeError("'<' not supported between instances of 'Costmap' and 'float'")
    except TypeError as e:
        m.stop(e)
    assert "TypeError: '<' not supported" in (tmp_path / "crash.txt").read_text()
    assert sysmon.read(tmp_path / "system.jsonl")[-1]["outcome"] == "error"


def test_a_torn_last_line_is_skipped(tmp_path):
    (tmp_path / "s.jsonl").write_text('{"type": "info"}\n{"type": "sam')
    assert sysmon.read(tmp_path / "s.jsonl") == [{"type": "info"}]


# ---- end to end: two real runs --------------------------------------------------------

def test_a_tiny_suite_end_to_end(tmp_path):
    (tmp_path / "tiny.yaml").write_text(
        "name: tiny\n"
        "scenarios: [{scenario: delivery_smoke, robots: [2], duration_s: 20}]\n"
        "seeds: [1]\n"
        "variants: {default: {}, nosharing: {map_share.enabled: false}}\n"
        "headline: fleet_tasks.per_hour\n"
        "also: [fleet_tasks.done]\n")
    out = tmp_path / "b"
    p = subprocess.run([sys.executable, "-m", "fleetsim", "bench", str(tmp_path / "tiny.yaml"),
                        "--out", str(out), "--jobs", "2"], cwd=REPO, capture_output=True,
                       text=True, timeout=120)
    assert p.returncode == 0, p.stdout + p.stderr
    assert "### fleet_tasks.per_hour" in p.stdout and "| delivery_smoke |" in p.stdout
    rows = list(csv.DictReader((out / "results.csv").open()))
    assert sorted(r["variant"] for r in rows) == ["default", "nosharing"]
    assert all(r["status"] in ("pass", "fail") and r["fleet_tasks.announced"] for r in rows)
    assert "fleet_tasks.done" in (out / "table.md").read_text()
    for r in rows:
        d = out / r["run"]
        sysrows = sysmon.read(d / "system.jsonl")
        assert sysrows[0]["type"] == "info" and sysrows[0]["executable"]
        assert any(x["type"] == "sample" for x in sysrows)
        assert sysrows[-1]["type"] == "end" and sysrows[-1]["outcome"] == "ok"
        assert not (d / "crash.txt").exists()
    summary = json.loads((out / "summary.json").read_text())
    assert summary["complete"] and summary["host"]["python"]

    p = subprocess.run([sys.executable, "-m", "fleetsim", "bench", "--rescore", str(out)],
                       cwd=REPO, capture_output=True, text=True, timeout=120)
    assert p.returncode == 0, p.stdout + p.stderr
    assert "rescored" in json.loads((out / "summary.json").read_text())
