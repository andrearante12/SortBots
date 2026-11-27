"""fleetsim benchmarks: scenarios x robot counts x seeds x variants, one table.

    /usr/bin/python3 -m fleetsim bench --list
    /usr/bin/python3 -m fleetsim bench SUITE [--jobs N] [--out DIR] [--retries K]
                                     [--only PATTERN] [--timeout S] [--dry-run]
                                     [--no-faulthandler]
    /usr/bin/python3 -m fleetsim bench --rescore DIR

A suite is fleetsim/suites/<name>.yaml (or a path to a .yaml file):

    name: smoke                      # = the file stem
    description: ...
    scenarios:                       # each entry runs at each of its robot counts
      - {scenario: delivery_smoke, robots: [2, 5]}
      - {scenario: walkers, robots: [5], duration_s: 120}
      - {scenario: walkers, id: walkers_radio, set: {network.model: radio}}
    seeds: [1, 2, 3]                 # or a count: 3 = [1, 2, 3]
    variants:                        # name -> overrides, the same as --set
      default: {}
      intents: {intent.enabled: true}
      radio: [network.model=radio]   # a list of key=value strings works too
    headline: fleet_tasks.per_hour   # <metric id>.<value>: the main table
    also: [fleet_tasks.mean_s, nav.blocked_robot_s]   # one more table each

Entry keys: `scenario` (required), `robots` (a count or a list; default the
scenario's own), `duration_s`, `set` (overrides for this entry, applied before
the variant's), `id` (the row label and run-name prefix, default the scenario
name; give one when a scenario appears twice). Every scenario x count x
variant is resolved with config.load before anything runs, so a bad knob is an
error up front, not 300 failed runs. The headline's metric must be in every
scenario's `metrics:`; values are read generically from each run's score.json,
so a new metric or knob needs no change here.

Each run is `<python> -u -m fleetsim run SCENARIO --robots N --seed S
[--duration D] [--set k=v ...] --out RUN_DIR --quiet`, its own process (a crash
takes down one run, not the bench), N at a time (default: CPUs - 2). Output,
gitignored:

    data/fleetsim/bench/<UTC>_<suite>/
      suite.yaml             the suite file as it ran
      summary.json           suite, host, every run with its attempts and metric
                             values, the tables, crash counts by kind and core
      results.csv            one row per run: plan, status, attempts, crash
                             cores, wall_s, then every metric value as <id>.<value>
      table.md               the printed tables
      <id>_r<N>_s<seed>_<variant>/     one per run: the files `fleetsim run`
                             writes (system.jsonl, crash.txt included), log.txt
        attempt_<k>/         a crashed attempt, moved aside before the re-run:
                             its log.txt, system.jsonl, crash.txt, and kernel.txt
                             when it died by a signal

A run is complete when it exits 0 (criteria held) or 1 (one failed) and wrote
score.json. Exit 2 is an invalid scenario: reported, not retried. Anything else
(a signal, exit 1 with no score.json, a --timeout) is a crash, retried up to
--retries times: runs are seeded and deterministic, so a re-run is the same
run. For a death by signal the kernel's log lines for the run's pid are saved
(journalctl -k, else dmesg): on recent kernels a segfault line names the core.

The CPU a crashed run died on comes from the kernel's segfault line when there
is one, else from the dead process's /proc/<pid>/stat (read before it is
reaped; it matched the kernel's line 4 of 4 times, 2026-11-26), else from its
last system.jsonl sample (up to 2 s stale, marked "?" in results.csv). With
faulthandler on (the default) the kernel logs no segfault line at all
(sysmon.py); --no-faulthandler trades crash.txt's Python stack for it.

Exit codes: 0 every run completed, 1 some run never did (crashed out or
invalid), 2 invalid suite or arguments, 130 interrupted (what finished is
still written).
"""
from __future__ import annotations

import csv
import fnmatch
import json
import math
import os
import re
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path

import yaml

from fleetsim import config, sysmon
from simtest import metrics as metrics_lib

REPO_ROOT = config.FLEETSIM_DIR.parent
SUITES_DIR = Path(os.environ.get("FLEETSIM_SUITES_DIR") or config.FLEETSIM_DIR / "suites")
BENCH_DIR = config.RESULTS_DIR / "bench"
SUITE_KEYS = {"name", "description", "scenarios", "seeds", "variants", "headline", "also"}
ENTRY_KEYS = {"scenario", "id", "robots", "duration_s", "set"}
LABEL_RX = re.compile(r"^[a-z0-9][a-z0-9_-]{0,47}$")
COMPLETED = ("pass", "fail")
TIMEOUT = "timeout"
NO_SCORE = "exited without writing a new score.json"
SUMMARY_KEYS = ("sim_s", "wall_s", "speedup", "reason")
PLAN_KEYS = ("name", "scenario", "scenario_file", "robots", "seed", "variant", "duration_s", "sets")
# The kernel names the core of a segfault or trap on recent kernels:
# "python3[1234]: segfault at 8 ip ... in python3.14[...] likely on CPU 10 (core 20, socket 0)"
CORE_RX = re.compile(r"likely on CPU (\d+) \(core (\d+), socket (\d+)\)")
# Machine-check lines name no pid but are the most direct evidence against a CPU.
HW_RX = re.compile(r"mce:|machine check|hardware error", re.I)
# Where a CPU number came from: the kernel's fault line and the dead process's
# stat both name the CPU it died on; the last sample is up to 2 s older.
AT_DEATH = ("kernel", "exit")
FAULT_SIGNALS = {signal.SIGSEGV, signal.SIGBUS, signal.SIGILL, signal.SIGFPE, signal.SIGABRT}


class SuiteError(config.ScenarioError):
    pass


# ---- suites -------------------------------------------------------------------------

def inline_yaml(v) -> str:
    """A value as the text after `--set key=`, so the run's yaml.safe_load gets it back."""
    s = yaml.safe_dump(v, default_flow_style=True, width=1 << 30).strip()
    if s.endswith("\n..."):
        s = s[:-4].strip()
    elif s.endswith("..."):
        s = s[:-3].strip()
    return s


def _overrides(raw, where: str) -> list[list]:
    """A mapping {dotted.key: value} or a list of "key=value" -> [[key, inline text]]."""
    if raw is None:
        return []
    pairs = []
    if isinstance(raw, dict):
        pairs = list(raw.items())
    elif isinstance(raw, list):
        for it in raw:
            if not isinstance(it, str) or "=" not in it:
                raise SuiteError(f"{where}: expected key=value strings, got {it!r}")
            k, v = it.split("=", 1)
            pairs.append((k.strip(), yaml.safe_load(v)))
    else:
        raise SuiteError(f"{where}: expected a mapping {{key: value}} or a list of key=value")
    out = []
    for k, v in pairs:
        if not isinstance(k, str) or not re.match(r"^[A-Za-z0-9_]+(\.[A-Za-z0-9_]+)*$", k):
            raise SuiteError(f"{where}: bad key {k!r} (expected dotted.config.key)")
        text = inline_yaml(v)
        if "\n" in text or yaml.safe_load(text) != v:
            raise SuiteError(f"{where}.{k}: {v!r} can't be passed as --set {k}=...")
        out.append([k, text])
    return out


def _metric_ref(ref, where: str) -> tuple[str, str]:
    if not isinstance(ref, str) or "." not in ref:
        raise SuiteError(f"{where}: expected <metric id>.<value>, e.g. fleet_tasks.per_hour")
    mid, value = ref.split(".", 1)
    return mid, value


def _check_value(ref: str, metric_name: str, where: str) -> None:
    cls = metrics_lib.REGISTRY.get(metric_name)
    first = ref.split(".", 1)[1].split(".", 1)[0]
    if cls is not None and first not in cls.values:
        raise SuiteError(f"{where}: {metric_name} has no value {first!r} (has: {sorted(cls.values)})")


def resolve_suite(raw, stem: str | None = None) -> dict:
    """A suite mapping -> a validated suite. Loads every scenario x robots x variant."""
    if not isinstance(raw, dict):
        raise SuiteError("a suite must be a YAML mapping")
    unknown = set(raw) - SUITE_KEYS
    if unknown:
        raise SuiteError(f"unknown key(s) {sorted(unknown)} (have: {sorted(SUITE_KEYS)})")
    name = raw.get("name")
    if not isinstance(name, str) or not config.NAME_RX.match(name):
        raise SuiteError(f"name: must match {config.NAME_RX.pattern}")
    if stem is not None and name != stem:
        raise SuiteError(f"name: must equal the file stem {stem!r}")

    seeds = raw.get("seeds", [1])
    if isinstance(seeds, int) and not isinstance(seeds, bool):
        seeds = list(range(1, seeds + 1))
    if (not isinstance(seeds, list) or not seeds or len(set(seeds)) != len(seeds)
            or any(isinstance(s, bool) or not isinstance(s, int) or s < 0 for s in seeds)):
        raise SuiteError("seeds: a list of distinct non-negative integers, or a count")

    rawv = raw.get("variants") or {"default": {}}
    if not isinstance(rawv, dict):
        raise SuiteError("variants: expected a mapping {name: overrides}")
    variants = {}
    for vname, over in rawv.items():
        if not isinstance(vname, str) or not LABEL_RX.match(vname):
            raise SuiteError(f"variants: name {vname!r} must match {LABEL_RX.pattern}")
        variants[vname] = _overrides(over, f"variants.{vname}")

    headline = raw.get("headline")
    _metric_ref(headline, "headline")
    also = raw.get("also") or []
    if not isinstance(also, list):
        raise SuiteError("also: expected a list of <metric id>.<value>")
    for i, ref in enumerate(also):
        _metric_ref(ref, f"also[{i}]")

    entries = raw.get("scenarios")
    if not isinstance(entries, list) or not entries:
        raise SuiteError("scenarios: need at least one {scenario: NAME, robots: [...]}")
    out_entries, ids = [], set()
    metric_ids: dict[str, str] = {}  # every metric id any scenario has -> its metric
    for i, e in enumerate(entries):
        w = f"scenarios[{i}]"
        if not isinstance(e, dict) or "scenario" not in e:
            raise SuiteError(f"{w}: expected a mapping with scenario: NAME")
        bad = set(e) - ENTRY_KEYS
        if bad:
            raise SuiteError(f"{w}: unknown key(s) {sorted(bad)} (have: {sorted(ENTRY_KEYS)})")
        scn = e["scenario"]
        eid = e.get("id", scn)
        if not isinstance(eid, str) or not LABEL_RX.match(eid):
            raise SuiteError(f"{w}.id: {eid!r} must match {LABEL_RX.pattern}")
        if eid in ids:
            raise SuiteError(f"{w}: {eid!r} is in the suite twice; give one an id:")
        ids.add(eid)
        robots = e.get("robots")
        robots = [robots] if robots is None or isinstance(robots, int) else robots
        if (not isinstance(robots, list) or not robots or len(set(robots)) != len(robots)
                or any(r is not None and (isinstance(r, bool) or not isinstance(r, int) or r < 1)
                       for r in robots)):
            raise SuiteError(f"{w}.robots: a positive count or a list of distinct counts")
        dur = e.get("duration_s")
        if dur is not None and (isinstance(dur, bool) or not isinstance(dur, (int, float))):
            raise SuiteError(f"{w}.duration_s: expected a number of seconds")
        sets = _overrides(e.get("set"), f"{w}.set")
        counts = []
        for n in robots:
            for vname, vsets in variants.items():
                try:
                    s = config.load(scn, robots=n, duration=dur,
                                    sets=[(k, yaml.safe_load(v)) for k, v in sets + vsets])
                except config.ScenarioError as ex:
                    raise SuiteError(f"{w} ({eid}), {n or 'default'} robots, variant {vname}: "
                                     f"{ex}") from None
            counts.append(s["config"]["robots"]["count"])
            here = {m["id"]: m["metric"] for m in s.get("metrics_resolved") or []}
        if len(set(counts)) != len(counts):
            raise SuiteError(f"{w}.robots: {robots} resolve to the same count twice")
        hid = headline.split(".", 1)[0]
        if hid not in here:
            raise SuiteError(f"headline: scenario {scn} has no metric {hid!r} "
                             f"(its metrics: {sorted(here) or 'none'})")
        metric_ids.update(here)
        out_entries.append({"id": eid, "scenario": scn, "robots": counts,
                            "duration_s": None if dur is None else float(dur), "set": sets,
                            "duration_eff": s["duration_s"]})
    for where, ref in [("headline", headline)] + [(f"also[{i}]", r) for i, r in enumerate(also)]:
        mid = ref.split(".", 1)[0]
        if mid not in metric_ids:
            raise SuiteError(f"{where}: no scenario in the suite has a metric {mid!r} "
                             f"(they have: {sorted(metric_ids)})")
        _check_value(ref, metric_ids[mid], where)
    return {"name": name, "description": str(raw.get("description") or "").strip(),
            "scenarios": out_entries, "seeds": seeds, "variants": variants,
            "headline": headline, "also": list(also)}


def suite_path(ref: str) -> Path:
    if ref.endswith((".yaml", ".yml")) or "/" in ref:
        return Path(ref)
    if not config.NAME_RX.match(ref or ""):
        raise SuiteError(f"suite name {ref!r} must match {config.NAME_RX.pattern}")
    return SUITES_DIR / f"{ref}.yaml"


def load_suite(ref: str) -> tuple[dict, str]:
    """-> (resolved suite, the file's text)."""
    p = suite_path(ref)
    try:
        text = p.read_text()
        raw = yaml.safe_load(text)
    except FileNotFoundError:
        raise SuiteError(f"no suite {ref!r} (looked for {p})") from None
    except yaml.YAMLError as e:
        raise SuiteError(f"{p.name}: {e}") from None
    try:
        return resolve_suite(raw, stem=p.stem), text
    except SuiteError as e:
        raise SuiteError(f"{p.name}: {e}") from None


def list_suites() -> list[dict]:
    out = []
    for p in sorted(SUITES_DIR.glob("*.yaml")):
        try:
            s, _ = load_suite(str(p))
            out.append({"name": s["name"], "runs": len(plan(s)), "description": s["description"],
                        "status": "ready"})
        except SuiteError as e:
            out.append({"name": p.stem, "status": "invalid", "error": str(e)})
    return out


def plan(suite: dict, only: list[str] | None = None) -> list[dict]:
    jobs = []
    for e in suite["scenarios"]:
        for n in e["robots"]:
            for vname, vsets in suite["variants"].items():
                for seed in suite["seeds"]:
                    jobs.append({"name": f"{e['id']}_r{n}_s{seed}_{vname}", "scenario": e["id"],
                                 "scenario_file": e["scenario"], "robots": n, "seed": seed,
                                 "variant": vname, "duration_s": e["duration_s"],
                                 "sets": e["set"] + vsets,
                                 "cost": n * e["duration_eff"]})
    if only:
        jobs = [j for j in jobs if any(fnmatch.fnmatchcase(j["name"], p) for p in only)]
    return jobs


def run_argv(job: dict, run_dir: Path) -> list[str]:
    # The bench's own interpreter: on CI that's setup-python's, which has numpy;
    # /usr/bin/python3 there may not.
    argv = [sys.executable, "-u", "-m", "fleetsim", "run", job["scenario_file"],
            "--robots", str(job["robots"]), "--seed", str(job["seed"])]
    if job["duration_s"] is not None:
        argv += ["--duration", repr(job["duration_s"])]
    for k, v in job["sets"]:
        argv += ["--set", f"{k}={v}"]
    return argv + ["--out", str(run_dir), "--quiet"]


def child_env(faulthandler: bool = True) -> dict:
    env = dict(os.environ)
    if not faulthandler:
        env["FLEETSIM_FAULTHANDLER"] = "0"
    # One BLAS thread per run: N runs at once with a CPU-sized pool each would
    # oversubscribe the machine, and it keeps a run to two threads (main and
    # sampler) whose tids the kernel's segfault line can be matched against.
    for k in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        env.setdefault(k, "1")
    return env


def default_jobs() -> int:
    try:
        n = len(os.sched_getaffinity(0))
    except AttributeError:
        n = os.cpu_count() or 1
    return max(1, n - 2)


# ---- one run ------------------------------------------------------------------------

def _iso(t: float) -> str:
    return datetime.fromtimestamp(t, timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")[:-4] + "Z"


def _json(path: Path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def signal_name(code: int) -> str:
    try:
        return signal.Signals(-code).name
    except ValueError:
        return f"signal {-code}"


def classify(code, run_dir: Path) -> tuple[str, str]:
    """An attempt's exit -> (kind, reason); kind: pass | fail | invalid | stopped | crash.
    A str code is a crash its launcher named (TIMEOUT, NO_SCORE)."""
    if isinstance(code, str):
        return "crash", code
    if code in (0, 1):
        sc = _json(run_dir / "score.json")
        if isinstance(sc, dict) and "ok" in sc:
            return ("pass" if sc["ok"] else "fail"), f"exit {code}"
        return "crash", f"exit {code} without score.json"
    if code == 2:
        return "invalid", "exit 2: invalid scenario or arguments"
    if code == 130:
        return "stopped", "exit 130: SIGINT"
    if isinstance(code, int) and code < 0:
        return "crash", signal_name(code)
    return "crash", f"exit {code}"


def flatten(score: dict | None) -> dict:
    """score.json -> {<metric id>.<value>: v}; nested values get more dots, lists JSON."""
    out: dict = {}

    def put(key, v):
        if isinstance(v, dict):
            for k2, v2 in v.items():
                put(f"{key}.{k2}", v2)
        elif isinstance(v, (list, tuple)):
            out[key] = json.dumps(v)
        else:
            out[key] = v

    for m in (score or {}).get("metrics", []):
        for k, v in (m.get("values") or {}).items():
            put(f"{m['id']}.{k}", v)
    return out


def kernel_log(t0: float, t1: float) -> tuple[list[str] | None, str]:
    """The kernel's log lines from t0 - 1 s to t1 + 2 s -> (lines or None, source or why not)."""
    since, until = int(t0) - 1, int(math.ceil(t1)) + 2
    errs = []
    try:
        p = subprocess.run(["journalctl", "-k", "--no-pager", "-q", "-o", "short-iso-precise",
                            "--since", f"@{since}", "--until", f"@{until}"],
                           capture_output=True, text=True, timeout=20)
        if p.returncode == 0:
            return p.stdout.splitlines(), "journalctl -k"
        errs.append(f"journalctl: {(p.stderr or '').strip() or f'exit {p.returncode}'}")
    except (OSError, subprocess.TimeoutExpired) as e:
        errs.append(f"journalctl: {e}")
    try:
        # No time filter in dmesg; the pid match below does the narrowing.
        p = subprocess.run(["dmesg", "--time-format", "iso"], capture_output=True, text=True,
                           timeout=20)
        if p.returncode == 0:
            return p.stdout.splitlines(), "dmesg"
        errs.append(f"dmesg: {(p.stderr or '').strip() or f'exit {p.returncode}'}")
    except (OSError, subprocess.TimeoutExpired) as e:
        errs.append(f"dmesg: {e}")
    return None, "; ".join(errs)


def kernel_lines_for(lines: list[str], ids: list[int]) -> list[str]:
    """Lines naming one of the process's ids ("comm[pid]:", "Killed process pid"), plus
    machine-check lines, which name no process."""
    rx = re.compile(r"\b(" + "|".join(str(i) for i in ids) + r")\b") if ids else None
    return [ln for ln in lines if (rx is not None and rx.search(ln)) or HW_RX.search(ln)]


def proc_cpu(pid) -> int | None:
    """Field 39 of /proc/<pid>/stat: the CPU the process last ran on."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            return int(f.read().rsplit(")", 1)[-1].split()[39 - 3])
    except (OSError, IndexError, ValueError):
        return None


def wait_exit(p, timeout: float | None) -> tuple:
    """-> (returncode or TIMEOUT, the CPU it died on). Waits with WNOWAIT so the
    child stays a zombie while its /proc stat is read, then reaps it."""
    if not hasattr(os, "waitid"):
        try:
            return p.wait(timeout=timeout), None
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
            return TIMEOUT, None
    deadline = None if timeout is None else time.monotonic() + timeout
    while True:
        try:
            if os.waitid(os.P_PID, p.pid, os.WEXITED | os.WNOWAIT | os.WNOHANG) is not None:
                break
        except ChildProcessError:  # reaped already
            return p.wait(), None
        if deadline is not None and time.monotonic() > deadline:
            p.kill()
            p.wait()
            return TIMEOUT, None
        time.sleep(0.1)
    cpu = proc_cpu(p.pid)
    return p.wait(), cpu


def launch_subprocess(job: dict, run_dir: Path, runner: "Runner") -> dict:
    argv = run_argv(job, run_dir)
    with open(run_dir / "log.txt", "w") as log:
        log.write("$ " + " ".join(argv) + "\n")
        log.flush()
        t0 = time.time()
        p = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                             env=child_env(runner.faulthandler), cwd=REPO_ROOT)
        runner.track(p)
        try:
            code, cpu = wait_exit(p, runner.timeout)
        finally:
            runner.untrack(p)
    return {"exit": code, "pid": p.pid, "t0": t0, "t1": time.time(), "exit_cpu": cpu}


class Runner:
    """Runs a plan, each job with up to `retries` re-runs after a crash.

    launch(job, run_dir, runner) -> {exit, pid, t0, t1} runs one attempt (the
    real one starts `fleetsim run`; tests inject a fake), kernel(t0, t1) ->
    (lines, source) reads the kernel log."""

    def __init__(self, bench_dir: Path, *, retries: int = 2, timeout: float | None = None,
                 launch=launch_subprocess, kernel=kernel_log, say=None, kernel_wait_s: float = 3.0,
                 rescoring: bool = False, faulthandler: bool = True):
        self.bench_dir = Path(bench_dir)
        self.faulthandler = faulthandler
        self.cores = sysmon.core_ids()
        # A rescore re-reads a finished run: a crash in it must neither move the
        # run's files aside nor be blamed on the run's own compute log.
        self.rescoring = rescoring
        self.retries = int(retries)
        self.timeout = timeout
        self.launch = launch
        self.kernel = kernel
        self.kernel_wait_s = kernel_wait_s
        self._say = say or (lambda s: print(s, flush=True))
        self._say_lock = threading.Lock()
        self.stopping = threading.Event()
        self._procs: set = set()
        self._procs_lock = threading.Lock()

    def say(self, s: str) -> None:
        with self._say_lock:
            self._say(s)

    def track(self, p) -> None:
        with self._procs_lock:
            self._procs.add(p)
        if self.stopping.is_set():  # started just as the stop came in
            self._interrupt(p)

    def untrack(self, p) -> None:
        with self._procs_lock:
            self._procs.discard(p)

    @staticmethod
    def _interrupt(p) -> None:
        try:
            p.send_signal(signal.SIGINT)  # a run closes its recording on SIGINT
        except OSError:
            pass

    def stop(self) -> None:
        self.stopping.set()
        with self._procs_lock:
            procs = list(self._procs)
        for p in procs:
            self._interrupt(p)

    def run_job(self, job: dict) -> dict:
        rec = {k: job[k] for k in PLAN_KEYS}
        rec.update(status="skipped", attempts=[], values={}, summary=None)
        try:
            self._run_job(job, rec)
        except Exception as e:  # noqa: BLE001 - a bug here must not take the bench down
            rec["status"] = "crashed"
            rec["attempts"].append({"attempt": len(rec["attempts"]) + 1, "kind": "crash",
                                    "reason": f"bench runner error: {e!r}"})
        return rec

    def _run_job(self, job: dict, rec: dict) -> None:
        run_dir = self.bench_dir / job["name"]
        run_dir.mkdir(parents=True, exist_ok=True)
        for k in range(1, self.retries + 2):
            if self.stopping.is_set():
                rec["status"] = "stopped" if rec["attempts"] else "skipped"
                return
            res = self.launch(job, run_dir, self)
            att = self._attempt(k, res, run_dir)
            rec["attempts"].append(att)
            if att["kind"] in COMPLETED:
                rec["status"] = att["kind"]
                rec["values"] = flatten(_json(run_dir / "score.json"))
                summ = _json(run_dir / "summary.json") or {}
                rec["summary"] = {key: summ.get(key) for key in SUMMARY_KEYS}
                return
            if att["kind"] in ("invalid", "stopped") or self.stopping.is_set():
                # A crash while stopping is most likely our own SIGINT landing
                # before the run's handler was in place.
                rec["status"] = "stopped" if self.stopping.is_set() else att["kind"]
                return
            if not self.rescoring:
                self._stash(run_dir, k)
            where = (f" on CPU {att['cpu']} (core {att['core']}, {att['cpu_src']})"
                     if att.get("cpu") is not None else "")
            last = k > self.retries
            rec["status"] = "crashed"
            self.say(f"          crash {job['name']} attempt {k}: {att['reason']}{where}"
                     + (f" at sim {att['sim_s']} s" if att.get("sim_s") is not None else "")
                     + (" - giving up" if last else " - re-running"))

    def _attempt(self, k: int, res: dict, run_dir: Path) -> dict:
        code = res["exit"]
        kind, reason = classify(code, run_dir)
        att = {"attempt": k, "exit": code, "kind": kind, "reason": reason, "pid": res.get("pid"),
               "start": _iso(res["t0"]), "wall_s": round(res["t1"] - res["t0"], 2)}
        if kind != "crash" or self.rescoring:
            return att
        rows = sysmon.read(run_dir / "system.jsonl")
        info = next((r for r in rows if r.get("type") == "info"), {})
        samples = [r for r in rows if r.get("type") == "sample"]
        last = samples[-1] if samples else {}
        att.update(sim_s=last.get("sim_s"), phase=last.get("phase"), cpu=None, core=None,
                   cpu_src=None, exit_cpu=res.get("exit_cpu"),
                   crash_txt=(run_dir / "crash.txt").is_file())
        if isinstance(code, int) and code < 0:
            self._kernel(att, res, info, run_dir, code)
        if att["cpu"] is None and res.get("exit_cpu") is not None:
            att.update(cpu=res["exit_cpu"], core=self.cores.get(res["exit_cpu"]), cpu_src="exit")
        if att["cpu"] is None and last.get("cpu") is not None:
            # Where the main thread was at the last sample (up to 2 s before the
            # end): a weaker hint than the kernel's line, so labelled apart.
            att.update(cpu=last["cpu"], core=last.get("core"), cpu_src="sample")
        return att

    def _kernel(self, att: dict, res: dict, info: dict, run_dir: Path, code: int) -> None:
        pid = res.get("pid")
        ids = [i for i in (pid, info.get("sampler_tid")) if isinstance(i, int)]
        lines, src, mine = None, "", []
        deadline = time.monotonic() + self.kernel_wait_s
        while True:
            lines, src = self.kernel(res["t0"], res["t1"])
            mine = kernel_lines_for(lines or [], ids)
            # journald can lag the kernel by a moment; an uncaught fault should
            # have left a line, so give it a few seconds. A caught one (the run's
            # faulthandler) never does.
            if (mine or lines is None or -code not in FAULT_SIGNALS or info.get("faulthandler")
                    or time.monotonic() >= deadline):
                break
            time.sleep(0.5)
        head = [f"# kernel log for pid {pid} (thread ids {ids}), {signal_name(code)}, "
                f"{_iso(res['t0'] - 1)} .. {_iso(res['t1'] + 2)}",
                f"# source: {src}" if lines is not None else f"# not readable: {src}"]
        if lines is not None and not mine:
            head.append("# no line names this process"
                        + (": its faulthandler handled the signal, and the kernel logs only "
                           "unhandled faults (exit_cpu in summary.json has the CPU)"
                           if info.get("faulthandler") and -code in FAULT_SIGNALS else ""))
        (run_dir / "kernel.txt").write_text("\n".join(head + mine) + "\n")
        for ln in mine:
            m = CORE_RX.search(ln)
            if m:
                att.update(cpu=int(m.group(1)), core=int(m.group(2)), cpu_src="kernel")
                break
        att["kernel_lines"] = len(mine)

    @staticmethod
    def _stash(run_dir: Path, k: int) -> None:
        dest = run_dir / f"attempt_{k}"
        dest.mkdir(exist_ok=True)
        for p in run_dir.iterdir():
            if p.name.startswith("attempt_"):
                continue
            shutil.move(str(p), str(dest / p.name))

    def run(self, jobs: list[dict], n_jobs: int, on_done=None) -> list[dict]:
        """-> one record per job (plan order). on_done(rec, i, n) after each finishes."""
        records: dict[str, dict] = {}
        # Longest first, so the slowest run doesn't start last and set the wall time.
        order = sorted(jobs, key=lambda j: -j["cost"])
        ex = ThreadPoolExecutor(max_workers=max(1, n_jobs))
        futs = {ex.submit(self.run_job, j): j for j in order}
        try:
            for i, f in enumerate(as_completed(futs), 1):
                rec = f.result()
                records[rec["name"]] = rec
                if on_done:
                    on_done(rec, i, len(jobs))
        except KeyboardInterrupt:
            self.stop()
            for f in futs:
                f.cancel()
            self.say("stopping: waiting for the runs in flight to close their recordings")
            for f, j in futs.items():
                if not f.cancelled():
                    rec = f.result()
                    if rec["name"] not in records:
                        records[rec["name"]] = rec
                        if on_done:
                            on_done(rec, len(records), len(jobs))
        finally:
            ex.shutdown(wait=True, cancel_futures=True)
        for j in jobs:
            if j["name"] not in records:
                records[j["name"]] = {**{k: j[k] for k in PLAN_KEYS}, "status": "skipped",
                                      "attempts": [], "values": {}, "summary": None}
        return [records[j["name"]] for j in jobs]


# ---- tables -------------------------------------------------------------------------

def _number(v):
    if isinstance(v, bool):
        return float(v)
    if isinstance(v, (int, float)) and math.isfinite(v):
        return float(v)
    return None


def aggregate(records: list[dict], key: str, suite: dict) -> list[dict]:
    """-> rows [{scenario, robots, cells: {variant: {mean, sd, n, of}}}] in suite order."""
    rows = []
    for e in suite["scenarios"]:
        for n in e["robots"]:
            cells = {}
            for v in suite["variants"]:
                mine = [r for r in records if r["scenario"] == e["id"] and r["robots"] == n
                        and r["variant"] == v]
                xs = [x for x in (_number(r["values"].get(key)) for r in mine
                                  if r["status"] in COMPLETED) if x is not None]
                cells[v] = {"mean": statistics.fmean(xs) if xs else None,
                            "sd": statistics.stdev(xs) if len(xs) > 1 else None,
                            "n": len(xs), "of": len(mine)}
            if any(c["of"] for c in cells.values()):
                rows.append({"scenario": e["id"], "robots": n, "cells": cells})
    return rows


def _decimals(mean: float, sd: float | None) -> int:
    a = max(abs(mean), sd or 0.0)
    return 1 if a == 0 else 0 if a >= 100 else 1 if a >= 10 else 2 if a >= 1 else 3


def fmt_cell(c: dict) -> str:
    if not c["n"]:
        return "-" if c["of"] else ""
    d = _decimals(c["mean"], c["sd"])
    s = f"{c['mean']:.{d}f}"
    if c["sd"] is not None:
        s += f" ± {c['sd']:.{d}f}"
    if c["n"] < c["of"]:
        s += f" ({c['n']}/{c['of']})"
    return s


def render_table(rows: list[dict], variants: list[str], title: str) -> str:
    head = ["scenario", "robots"] + variants
    body = [[r["scenario"], str(r["robots"])] + [fmt_cell(r["cells"][v]) for v in variants]
            for r in rows]
    w = [max(len(x) for x in col) for col in zip(head, *body)]
    line = lambda cells, right: "| " + " | ".join(   # noqa: E731
        c.rjust(w[i]) if right[i] else c.ljust(w[i]) for i, c in enumerate(cells)) + " |"
    right = [False] + [True] * (len(head) - 1)
    sep = "|" + "|".join(("-" * (w[i] + 1) + ":") if right[i] else ("-" * (w[i] + 2))
                         for i in range(len(head))) + "|"
    return "\n".join([f"### {title}", "", line(head, [False] * len(head)), sep]
                     + [line(b, right) for b in body])


def _tally(values) -> dict:
    """Most common first."""
    out: dict = {}
    for v in values:
        if v is not None:
            out[v] = out.get(v, 0) + 1
    return dict(sorted(out.items(), key=lambda kv: (-kv[1], str(kv[0]))))


def crash_stats(records: list[dict]) -> dict:
    atts = [a for r in records for a in r["attempts"] if a.get("kind") == "crash"]
    runs = {r["name"] for r in records if any(a.get("kind") == "crash" for a in r["attempts"])}
    after: dict[str, int] = {}
    for r in records:
        if r["name"] in runs and r["status"] in COMPLETED:
            k = str(len(r["attempts"]))
            after[k] = after.get(k, 0) + 1
    by = lambda f: _tally(f(a) for a in atts)  # noqa: E731
    return {
        "attempts": len(atts), "runs": len(runs),
        "recovered": sum(after.values()), "recovered_after_attempts": after,
        "unrecovered": sum(1 for r in records if r["status"] == "crashed"),
        "by_kind": by(lambda a: a.get("reason")),
        "by_cpu": by(lambda a: str(a["cpu"]) if a.get("cpu_src") in AT_DEATH else None),
        "by_core": by(lambda a: str(a["core"]) if a.get("cpu_src") in AT_DEATH else None),
        "by_cpu_sampled": by(lambda a: str(a["cpu"]) if a.get("cpu_src") == "sample" else None),
        "no_cpu": sum(1 for a in atts if a.get("cpu") is None),
    }


def crash_text(cs: dict) -> str:
    if not cs["attempts"]:
        return "crashes: none"
    rec = ", ".join(f"{n} on attempt {k}" for k, n in sorted(cs["recovered_after_attempts"].items()))
    lines = [f"crashes: {cs['attempts']} attempt(s) in {cs['runs']} run(s); "
             f"{cs['recovered']} recovered" + (f" ({rec})" if rec else "")
             + f", {cs['unrecovered']} never completed",
             "  by kind: " + ", ".join(f"{k} x{n}" for k, n in cs["by_kind"].items())]
    if cs["by_cpu"]:
        lines.append("  died on CPU: " + ", ".join(f"{k} x{n}" for k, n in cs["by_cpu"].items())
                     + "; core: " + ", ".join(f"{k} x{n}" for k, n in cs["by_core"].items()))
    if cs["by_cpu_sampled"]:
        lines.append("  last sampled CPU (weaker): "
                     + ", ".join(f"{k} x{n}" for k, n in cs["by_cpu_sampled"].items()))
    if cs["no_cpu"]:
        lines.append(f"  no CPU known for {cs['no_cpu']}")
    return "\n".join(lines)


def status_counts(records: list[dict]) -> dict:
    out: dict[str, int] = {}
    for r in records:
        out[r["status"]] = out.get(r["status"], 0) + 1
    return out


def report(suite: dict, records: list[dict], tables: bool = True) -> tuple[str, dict]:
    """-> (the tables and crash summary as markdown, the same as data)."""
    variants = list(suite["variants"])
    out, parts = {}, []
    for i, key in enumerate([suite["headline"]] + suite["also"] if tables else []):
        rows = aggregate(records, key, suite)
        out[key] = rows
        title = f"{key} - mean ± sd over seeds {suite['seeds']}" if i == 0 else key
        parts.append(render_table(rows, variants, title))
    counts = status_counts(records)
    cs = crash_stats(records)
    parts.append("runs: " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items()))
                 + f" (of {len(records)})\n" + crash_text(cs))
    return "\n\n".join(parts) + "\n", {"tables": out, "counts": counts, "crashes": cs}


FIXED_COLS = ["run", "scenario", "robots", "seed", "variant", "status", "attempts", "crashes",
              "crash_kinds", "crash_cpus", "crash_cores", "wall_s", "sim_s", "speedup", "reason"]


def csv_rows(records: list[dict]) -> tuple[list[str], list[dict]]:
    metric_cols: list[str] = []
    seen = set()
    rows = []
    for r in records:
        crashes = [a for a in r["attempts"] if a.get("kind") == "crash"]
        last = r["attempts"][-1] if r["attempts"] else {}
        summ = r.get("summary") or {}
        row = {"run": r["name"], "scenario": r["scenario"], "robots": r["robots"], "seed": r["seed"],
               "variant": r["variant"], "status": r["status"], "attempts": len(r["attempts"]),
               "crashes": len(crashes),
               "crash_kinds": ";".join(a.get("reason", "") for a in crashes),
               "crash_cpus": ";".join("" if a.get("cpu") is None else
                                      f"{a['cpu']}{'' if a.get('cpu_src') in AT_DEATH else '?'}"
                                      for a in crashes),
               "crash_cores": ";".join("" if a.get("core") is None else
                                       f"{a['core']}{'' if a.get('cpu_src') in AT_DEATH else '?'}"
                                       for a in crashes),
               "wall_s": last.get("wall_s"), "sim_s": summ.get("sim_s"),
               "speedup": summ.get("speedup"), "reason": summ.get("reason")}
        for k, v in r["values"].items():
            if k not in seen:
                seen.add(k)
                metric_cols.append(k)
            row[k] = v
        rows.append(row)
    return FIXED_COLS + metric_cols, rows


def _write_atomic(path: Path, text: str) -> None:
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, path)


def write_outputs(bench_dir: Path, meta: dict, suite: dict, records: list[dict],
                  final: bool = True) -> str:
    """summary.json always; the tables, results.csv and table.md when final. -> the report."""
    text, data = report(suite, records, tables=final)
    summary = {**meta, "complete": final and all(r["status"] != "skipped" for r in records),
               "suite": suite, **data, "runs": records}
    _write_atomic(bench_dir / "summary.json", json.dumps(summary, indent=1, default=str))
    if final:
        cols, rows = csv_rows(records)
        with open(bench_dir / "results.csv.tmp", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)
        os.replace(bench_dir / "results.csv.tmp", bench_dir / "results.csv")
        _write_atomic(bench_dir / "table.md",
                      f"# fleetsim bench {suite['name']} ({meta.get('started', '')})\n\n" + text)
    return text


# ---- commands -----------------------------------------------------------------------

def _new_bench_dir(name: str, out: str | None) -> Path:
    if out:
        d = Path(out)
        if (d / "summary.json").exists():
            raise SuiteError(f"--out {d}: already holds a bench (summary.json); pick a new dir")
        d.mkdir(parents=True, exist_ok=True)
        return d
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    BENCH_DIR.mkdir(parents=True, exist_ok=True)
    for k in range(1, 1000):  # mkdir without exist_ok claims the name atomically
        d = BENCH_DIR / (f"{stamp}_{name}" + (f"_{k}" if k > 1 else ""))
        try:
            d.mkdir()
            return d
        except FileExistsError:
            continue
    raise SuiteError("no free bench dir name")


def _progress(runner: Runner, suite: dict, jobs: list[dict]):
    w = max(len(j["name"]) for j in jobs) if jobs else 0

    def on_done(rec, i, n):
        last = rec["attempts"][-1] if rec["attempts"] else {}
        v = rec["values"].get(suite["headline"])
        more = f"  {suite['headline']}={v}" if v is not None else ""
        if len(rec["attempts"]) > 1:
            more += f"  (attempt {len(rec['attempts'])})"
        if rec["status"] not in COMPLETED and last.get("reason"):
            more += f"  {last['reason']}"
        wall = f"{last['wall_s']:6.1f} s" if last.get("wall_s") is not None else " " * 8
        runner.say(f"[{i:>{len(str(n))}}/{n}] {rec['status']:<8} {rec['name']:<{w}} {wall}{more}")
    return on_done


def cmd_bench(args) -> int:
    suite, text = load_suite(args.suite)
    only = [p.strip() for p in (args.only or "").split(",") if p.strip()]
    jobs = plan(suite, only)
    if not jobs:
        raise SuiteError(f"--only {args.only!r} matches no run")
    n_jobs = min(args.jobs or default_jobs(), len(jobs))
    if args.dry_run:
        for j in jobs:
            print(f"{j['name']}\n    " + " ".join(run_argv(j, Path("<bench dir>") / j["name"])))
        print(f"{len(jobs)} runs, {n_jobs} at a time, up to {args.retries} re-runs each after a crash")
        return 0
    bench_dir = _new_bench_dir(suite["name"], args.out)
    (bench_dir / "suite.yaml").write_text(text)
    meta = {"started": _iso(time.time()), "argv": sys.argv, "jobs": n_jobs,
            "retries": args.retries, "timeout_s": args.timeout, "only": only,
            "faulthandler": not args.no_faulthandler,
            "host": sysmon.static_info(), "bench_dir": str(bench_dir)}
    print(f"fleetsim bench {suite['name']}: {len(jobs)} runs, {n_jobs} at a time -> {bench_dir}",
          flush=True)
    runner = Runner(bench_dir, retries=args.retries, timeout=args.timeout,
                    faulthandler=not args.no_faulthandler)
    done: list[dict] = []
    show = _progress(runner, suite, jobs)
    last_save = [0.0]

    def on_done(rec, i, n):
        show(rec, i, n)
        done.append(rec)
        # Kept current, so a bench cut short (or a laptop that locks up) still
        # leaves the runs it finished on record; at most every 5 s, since it
        # rewrites every record so far.
        if time.monotonic() - last_save[0] > 5.0 or rec["status"] not in COMPLETED:
            last_save[0] = time.monotonic()
            write_outputs(bench_dir, meta, suite, done, final=False)

    t0 = time.monotonic()
    records = runner.run(jobs, n_jobs, on_done)
    meta.update(finished=_iso(time.time()), wall_s=round(time.monotonic() - t0, 1))
    print()
    print(write_outputs(bench_dir, meta, suite, records), end="")
    print(f"\n-> {bench_dir}/table.md, results.csv, summary.json")
    if runner.stopping.is_set():
        return 130
    return 0 if all(r["status"] in COMPLETED for r in records) else 1


def cmd_rescore(args) -> int:
    """Re-score every completed run with the current metric code, then rebuild the tables."""
    bench_dir = Path(args.rescore)
    old = _json(bench_dir / "summary.json")
    if not isinstance(old, dict) or "runs" not in old:
        raise SuiteError(f"{bench_dir}: no bench summary.json here")
    suite, records = old["suite"], old["runs"]
    todo = [r for r in records if r["status"] in COMPLETED]
    print(f"rescoring {len(todo)} runs in {bench_dir}", flush=True)

    def launch(job, run_dir, runner):
        argv = [sys.executable, "-u", "-m", "fleetsim", "rescore", str(run_dir)]
        t0 = time.time()
        p = subprocess.run(argv, capture_output=True, text=True, env=child_env(), cwd=REPO_ROOT)
        sc = run_dir / "score.json"
        fresh = sc.is_file() and sc.stat().st_mtime >= t0 - 1
        # The old score.json is still there, so exit 0/1 alone doesn't prove a new one.
        code = NO_SCORE if p.returncode in (0, 1) and not fresh else p.returncode
        (run_dir / "rescore.log").write_text(p.stdout + p.stderr)
        return {"exit": code, "pid": None, "t0": t0, "t1": time.time()}

    runner = Runner(bench_dir, retries=args.retries, launch=launch, rescoring=True)
    jobs = [{**r, "cost": 0} for r in todo]
    n_jobs = min(args.jobs or default_jobs(), max(1, len(jobs)))
    fresh = {r["name"]: r for r in runner.run(jobs, n_jobs, _progress(runner, suite, jobs))}
    bad = 0
    for r in records:
        f = fresh.get(r["name"])
        if f is None:
            continue
        if f["status"] in COMPLETED:
            r.update(status=f["status"], values=f["values"])
        else:
            bad += 1
            print(f"rescore of {r['name']}: {f['status']}; its old score is kept", file=sys.stderr)
    meta = {k: v for k, v in old.items()
            if k not in ("complete", "suite", "tables", "counts", "crashes", "runs")}
    meta["rescored"] = _iso(time.time())
    print()
    print(write_outputs(bench_dir, meta, suite, records), end="")
    if runner.stopping.is_set():
        return 130
    return 1 if bad else 0


def cmd_list() -> int:
    for s in list_suites():
        if s["status"] == "ready":
            print(f"{s['name']:<20} {s['runs']:>5} runs  {s['description'][:70]}")
        else:
            print(f"{s['name']:<20} INVALID: {s['error']}")
    return 0


def add_parser(sub) -> None:
    b = sub.add_parser("bench", help="run a benchmark suite (fleetsim/bench.py)")
    b.add_argument("suite", nargs="?", help="fleetsim/suites/<name>.yaml, or a path")
    b.add_argument("--list", action="store_true", help="list the suites")
    b.add_argument("--rescore", metavar="DIR", help="re-score a bench dir, rebuild its tables")
    b.add_argument("--jobs", type=int, help=f"runs at once (default CPUs - 2 = {default_jobs()})")
    b.add_argument("--out", help="bench dir (default data/fleetsim/bench/<UTC>_<suite>/)")
    b.add_argument("--retries", type=int, default=2, help="re-runs after a crash (default 2)")
    b.add_argument("--only", metavar="PATTERN",
                   help="run names to run, fnmatch, comma-separated (e.g. 'walkers_*,*_intents')")
    b.add_argument("--timeout", type=float, help="kill and count as a crash after S wall seconds")
    b.add_argument("--dry-run", action="store_true", help="print the runs, run nothing")
    b.add_argument("--no-faulthandler", action="store_true",
                   help="no Python stack in crash.txt, so the kernel logs each segfault "
                        "with its instruction address")


def main(args) -> int:
    picked = sum(bool(x) for x in (args.suite, args.list, args.rescore))
    if picked != 1:
        raise SuiteError("bench: give one of SUITE, --list, --rescore DIR")
    if args.jobs is not None and args.jobs < 1:
        raise SuiteError("--jobs: at least 1")
    if args.retries < 0:
        raise SuiteError("--retries: 0 or more")
    if args.list:
        return cmd_list()
    if args.rescore:
        return cmd_rescore(args)
    return cmd_bench(args)
