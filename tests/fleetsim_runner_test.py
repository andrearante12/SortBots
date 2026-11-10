"""Tests for webui/fleetsim_runner.py. No ROS, no sim, no server.

    /usr/bin/python3 -m pytest tests/fleetsim_runner_test.py
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "webui"))

import fleetsim_runner as fr  # noqa: E402

GOOD = {"name": "delivery_smoke", "robots": 8, "seed": 1, "duration": 60, "sets": {}}


class FakePopen:
    last = None
    pid = 1
    returncode = 0

    def __init__(self, argv, **kw):
        FakePopen.last = argv

    def poll(self):
        return 0


@pytest.fixture
def runner(tmp_path):
    return fr.FleetsimRunner(lambda: dict(os.environ), results_dir=tmp_path)


@pytest.fixture
def fake(monkeypatch):
    FakePopen.last = None
    monkeypatch.setattr(sys.modules["simtest_runner"].subprocess, "Popen", FakePopen)


def test_argv_matches_terminal_command(runner, fake, tmp_path):
    state = runner.start("fleet_scale", 20, 1, 600,
                         {"map_share.prior": "none", "record.belief_maps": "all"})
    argv = FakePopen.last
    assert argv[:7] == [fr.PYTHON, "-u", "-m", "fleetsim", "run", "fleet_scale", "--out"]
    assert Path(argv[7]).parent == tmp_path and Path(argv[7]).name == state["dir"]
    assert argv[8:] == ["--robots", "20", "--seed", "1", "--duration", "600",
                        "--set", "map_share.prior=none", "--set", "record.belief_maps=all"]


@pytest.mark.parametrize("bad", [
    {"name": "../etc/passwd"},
    {"name": "no_such_scenario"},
    {"name": 7},
    {"robots": None},  # a field missing from the request arrives as None
    {"robots": 0},
    {"robots": fr.MAX_ROBOTS + 1},
    {"robots": True},
    {"robots": "20; rm -rf /"},
    {"seed": -1},
    {"duration": float("nan")},
    {"duration": float("inf")},
    {"sets": None},
    {"sets": ["map_share.prior=none"]},
    {"sets": {"world.map": "/etc"}},  # takes a path, so it's terminal only
    {"sets": {"map_share.prior": "none\n--out=/tmp/x"}},
    {"sets": {"map_share.prior": ["none"]}},
])
def test_bad_input_is_rejected_before_popen(runner, fake, bad):
    with pytest.raises(fr.ScenarioError):
        runner.start(**{**GOOD, **bad})
    assert FakePopen.last is None


def test_catalog_lists_all_scenarios_and_overrides():
    listed = fr.catalog()["scenarios"]
    assert [s["name"] for s in listed] == [s["name"] for s in fr.config.list_scenarios()]
    for scenario in listed:
        assert set(scenario["settings"]) == set(fr.SETTABLE)


@pytest.mark.parametrize("exit_code,score,expected", [
    (0, {"ok": True}, "PASS"),
    (1, {"ok": False}, "FAIL"),
    (0, None, "DONE"),
    (130, None, "STOPPED"),
    (1, None, "CRASHED"),   # Python traceback
    (-11, None, "CRASHED"),  # segfault
])
def test_verdict(runner, tmp_path, exit_code, score, expected):
    runner.run_dir = "run"
    (tmp_path / "run").mkdir()
    if score is not None:
        (tmp_path / "run" / "score.json").write_text(json.dumps(score))
    assert runner.verdict(exit_code) == expected


def test_stop_ends_run_and_keeps_recording(tmp_path):
    # Runs in a child with SIGINT ignored, like a console started in the
    # background of a non-interactive shell. Stop still has to work there.
    code = f"""
import os, signal, sys, time
sys.path[:0] = [{str(REPO_ROOT)!r}, {str(REPO_ROOT / 'webui')!r}]
signal.signal(signal.SIGINT, signal.SIG_IGN)
import fleetsim_runner as fr
from pathlib import Path
r = fr.FleetsimRunner(lambda: dict(os.environ), results_dir=Path({str(tmp_path)!r}))
r.start("delivery_smoke", 8, 1, 3600, {{}})
end = time.monotonic() + 30
while "sim-s, seed" not in r.status(0)["log"] and time.monotonic() < end: time.sleep(0.05)
time.sleep(1.0)
r.cancel()
end = time.monotonic() + 30
while r.running() and time.monotonic() < end: time.sleep(0.05)
s = r.status(0)
print(s["verdict"], s["exit"], s["dir"])
"""
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=90)
    assert out.returncode == 0, out.stderr
    verdict, exit_code, run_dir = out.stdout.split()
    assert (verdict, exit_code) == ("STOPPED", "130")
    assert (tmp_path / run_dir / "events.jsonl.gz").stat().st_size > 0
