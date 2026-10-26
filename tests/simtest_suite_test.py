"""Test tiers: `suite:` in a test case, selection, and `simtest.py suite`'s exit codes.

Pure python, no sim: cmd_run is replaced by a stub, so what is under test is
the tier plumbing (which tests run, in what order, and what the scheduled job's
exit code says), not the runner.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from simtest import spec  # noqa: E402

# scripts/simtest.py shares the package's name, so it can't be imported by name.
_s = importlib.util.spec_from_file_location("simtest_cli", REPO_ROOT / "scripts" / "simtest.py")
cli = importlib.util.module_from_spec(_s)
_s.loader.exec_module(cli)


@pytest.fixture()
def tests_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(spec, "TESTS_DIR", tmp_path)

    def write(name, suite=None):
        body = f"name: {name}\n" + (f"suite: {suite}\n" if suite else "")
        (tmp_path / f"{name}.yaml").write_text(body)
    return write


def test_selection_follows_tier_order_then_name(tests_dir):
    tests_dir("b_reg", "regression")
    tests_dir("a_reg", "regression")
    tests_dir("z_smoke", "smoke")
    tests_dir("m_long", "long")
    assert spec.select_suites(["smoke", "regression"]) == ["z_smoke", "a_reg", "b_reg"]
    assert spec.select_suites(["regression", "smoke"]) == ["a_reg", "b_reg", "z_smoke"]
    assert spec.select_suites(["long"]) == ["m_long"]


def test_no_suite_key_means_regression(tests_dir):
    tests_dir("plain")
    assert spec.select_suites(["regression"]) == ["plain"]
    assert spec.select_suites(["smoke"]) == []


def test_unknown_suite_is_an_error_not_an_empty_tier(tests_dir):
    tests_dir("t", "nightly")          # a typo'd tier must not silently drop out of every run
    with pytest.raises(spec.SpecError, match="suite"):
        spec.select_suites(["regression"])
    with pytest.raises(spec.SpecError, match="unknown suite"):
        spec.select_suites(["nightly"])


def test_a_test_with_a_missing_map_stays_in_its_tier(tests_dir):
    # select reads the raw file: it never resolves maps, so it can't drop a test
    # whose saved map is missing — that test must fail when run, not vanish.
    (spec.TESTS_DIR / "needs_map.yaml").write_text(
        "name: needs_map\nsuite: smoke\nscenario: moving_obstacles_mapped\n"
        "overrides: {map: nonexistent_map}\n")
    assert spec.select_suites(["smoke"]) == ["needs_map"]


def test_validate_rejects_a_bad_suite_value():
    raw = spec.load_raw("pipeline_smoke")
    raw["suite"] = "sometimes"
    with pytest.raises(spec.SpecError, match="suite"):
        spec.validate(raw)


def test_committed_tests_all_have_a_valid_tier():
    for p in spec.TESTS_DIR.glob("*.yaml"):
        names = [n for s in spec.SUITES for n in spec.select_suites([s])]
        assert p.stem in names
    assert "pipeline_smoke" in spec.select_suites(["smoke"])   # the tier everything else waits on


@pytest.mark.parametrize("codes, expected", [
    ([], 0), ([0, 0], 0), ([0, 1], 1), ([1, 2], 2), ([2, 3], 3), ([3, 1, 0], 3),
    ([0, 130, 3], 130),
])
def test_suite_exit_code_ranks_by_what_the_operator_does_next(codes, expected):
    assert cli.suite_exit_code(codes) == expected


def _run_suite(monkeypatch, codes, *, suites=("regression",)):
    ran = []

    def fake_run(ns):
        ran.append(ns.name)
        return codes[ns.name]
    monkeypatch.setattr(cli, "cmd_run", fake_run)
    ns = cli.argparse.Namespace(suites=list(suites), list=False, repeats=None,
                                dry_run=False, force=False)
    return cli.cmd_suite(ns), ran


def test_suite_runs_every_test_and_reports_the_worst_code(tests_dir, monkeypatch):
    for n in ("a", "b", "c"):
        tests_dir(n, "regression")
    rc, ran = _run_suite(monkeypatch, {"a": 0, "b": 1, "c": 0})
    assert ran == ["a", "b", "c"]          # a failing verdict does not stop the tier
    assert rc == 1


def test_suite_stops_at_an_infrastructure_failure(tests_dir, monkeypatch):
    for n in ("a", "b", "c"):
        tests_dir(n, "regression")
    rc, ran = _run_suite(monkeypatch, {"a": 0, "b": 3, "c": 0})
    assert ran == ["a", "b"]               # c would only hit the same dead sim
    assert rc == 3


def test_suite_passes_only_when_every_test_ran_and_passed(tests_dir, monkeypatch):
    tests_dir("a", "regression")
    rc, _ = _run_suite(monkeypatch, {"a": 0})
    assert rc == 0


def test_an_empty_tier_is_a_failure_not_a_pass(tests_dir, monkeypatch):
    tests_dir("a", "regression")
    rc, ran = _run_suite(monkeypatch, {}, suites=("long",))
    assert rc == 2 and ran == []
