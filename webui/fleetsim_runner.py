"""Backend for the "Run a fleet sim" form in the operator view's Tests tab.

Starts `python3 -m fleetsim run` as a subprocess, one at a time. SimtestRunner
already knows how to launch a process, read its log and stop it, so this class
only adds input checks and the verdict.

The console can be reached from the internet, so everything from the page is
checked before it goes on the command line. That's also why the page can't send
arbitrary `--set` values: world.map takes a path, for example. It can only pick
from SETTABLE.
"""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import yaml

from simtest_runner import SimtestRunner  # must come first, it adds the repo root to sys.path
from fleetsim import config
from fleetsim.config import ScenarioError

# fleetsim needs numpy and serve.py doesn't, so a console started from conda
# would otherwise launch a run that dies on import.
PYTHON = "/usr/bin/python3"

MAX_ROBOTS = 200
MAX_DURATION_S = 3600.0
MAX_SEED = 2 ** 31 - 1

# Overrides the form can offer. Each option is the text after `--set key=`.
SETTABLE = {
    "map_share.prior": {"label": "Starting map", "options": ["none", "truth"]},
    "map_share.enabled": {"label": "Map sharing", "options": ["true", "false"]},
    "network.model": {"label": "Network", "options": ["ideal", "radio"]},
    "network.radio.contention": {"label": "Radio contention", "options": ["global", "spatial"]},
    "status.recipients": {"label": "Status sent to", "options": ["all", "radius"]},
    "status.session_liveness": {"label": "Session liveness", "options": ["true", "false"]},
    "record.belief_maps": {"label": "Own maps recorded", "options": ["all"]},
}


def get_setting(cfg, key):
    for part in key.split("."):
        cfg = cfg[part]
    return cfg


def catalog():
    """The scenarios the form lists, with their default values."""
    scenarios = []
    for entry in config.list_scenarios():
        if entry["status"] != "ready":
            continue
        s = config.load(entry["name"])
        scenarios.append({
            "name": s["name"],
            "title": s["title"],
            "robots": s["config"]["robots"]["count"],
            "seed": s["seed"],
            "duration_s": s["duration_s"],
            "settings": {key: get_setting(s["config"], key) for key in SETTABLE},
        })
    return {"scenarios": scenarios, "settable": SETTABLE}


def check_number(value, name, low, high, whole=True):
    allowed = int if whole else (int, float)
    # bool counts as int in Python. NaN fails the range check.
    if isinstance(value, bool) or not isinstance(value, allowed) or not low <= value <= high:
        raise ScenarioError(f"{name} must be in {low}..{high}")


def check_sets(sets):
    if not isinstance(sets, dict):
        raise ScenarioError("sets must be an object")
    for key, value in sets.items():
        if key not in SETTABLE:
            raise ScenarioError(f"{key!r} can't be set from here (run it from a terminal)")
        if value not in SETTABLE[key]["options"]:
            raise ScenarioError(f"{key} must be one of {SETTABLE[key]['options']}")


class FleetsimRunner(SimtestRunner):
    def __init__(self, env_fn, results_dir=config.RESULTS_DIR):
        self.results_dir = Path(results_dir)
        # The log is a file so replay.list_runs, which only lists folders, skips it.
        super().__init__(env_fn, log_path=self.results_dir / "_runner.log")
        self.run_dir = None  # folder name of the current or last run

    def start(self, name, robots, seed, duration, sets):
        if not isinstance(name, str):
            raise ScenarioError("bad scenario name")
        check_number(robots, "robots", 1, MAX_ROBOTS)
        check_number(seed, "seed", 0, MAX_SEED)
        check_number(duration, "duration", 1, MAX_DURATION_S, whole=False)
        check_sets(sets)

        # Load the config the run will use, so a bad one is an error on the form
        # and not a run that crashes straight away.
        parsed_sets = [(key, yaml.safe_load(value)) for key, value in sets.items()]
        config.load(name, robots=robots, seed=seed, duration=duration, sets=parsed_sets)

        # We pick the folder name so the page knows which run to replay.
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        run_dir = f"{timestamp}_{name}"

        # -u because stdout is a file: buffered output wouldn't show until exit.
        argv = [PYTHON, "-u", "-m", "fleetsim", "run", name,
                "--out", str(self.results_dir / run_dir),
                "--robots", str(robots),
                "--seed", str(seed),
                "--duration", str(duration)]
        for key, value in sets.items():
            argv += ["--set", f"{key}={value}"]

        self._launch(name, argv)
        self.run_dir = run_dir
        return self.status()

    def status(self, offset=None):
        state = super().status(offset)
        state["dir"] = self.run_dir
        state["verdict"] = None if state["exit"] is None else self.verdict(state["exit"])
        return state

    def verdict(self, exit_code):
        if exit_code == 130:
            return "STOPPED"
        # Exit 1 can mean a failed criterion or a Python crash (see Known issues
        # in fleetsim/README.md). Only a run that wrote a score actually failed.
        try:
            score = json.loads((self.results_dir / self.run_dir / "score.json").read_text())
        except (OSError, ValueError):
            # Exit 0 with no score is a scenario that has no metrics.
            return "DONE" if exit_code == 0 else "CRASHED"
        return "PASS" if score["ok"] else "FAIL"
