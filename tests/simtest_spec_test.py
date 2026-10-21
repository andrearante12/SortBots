"""simtest.spec + simtest.probe.ProbeCore + simtest.report: test-case schema,
the probe's scheduling decisions with a fake clock, and verdict/report
rendering. Offline: no ROS, no sim.

    /usr/bin/python3 -m pytest tests/simtest_spec_test.py
"""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from simtest import report, spec, world  # noqa: E402
from simtest.probe import ProbeCore  # noqa: E402

BASE = {
    "name": "unit",
    "scenario": "moving_obstacles",
    "overrides": {"world": "aisle_crossing"},
    "warmup_s": 2,
    "goals": [{"robot": "robot_0", "pose": [-8.0, 3.0, 0.0], "timeout_s": 30},
              {"robot": "robot_0", "station": "shelf_a", "at_s": 50}],
    "stop": {"sim_s": 100, "until": ["goals_done"], "settle_s": 2},
    "metrics": [{"metric": "clearance", "pass": {"contacts": {"eq": 0}}},
                {"metric": "goals"}],
}


def raw(**kw):
    r = copy.deepcopy(BASE)
    r.update(kw)
    return r


# -- committed test cases --------------------------------------------------------

@pytest.mark.parametrize("name", sorted(p.stem for p in spec.TESTS_DIR.glob("*.yaml")))
def test_committed_tests_validate(name):
    t = spec.load(name)
    assert t["variants"] and all(label for label, _ in t["variants"])


def test_list_tests_reports_every_file():
    names = {t["name"]: t for t in spec.list_tests()}
    assert set(names) == {p.stem for p in spec.TESTS_DIR.glob("*.yaml")}
    assert all(t["status"] == "ready" for t in names.values()), \
        [t.get("error") for t in names.values()]


# -- validation -------------------------------------------------------------------

def test_resolves_station_goal_and_robots():
    t = spec.validate(raw())
    label, r = t["variants"][0]
    assert label == "base" and r["robots"] == ["robot_0"]
    assert r["world"]["name"] == "aisle_crossing"
    assert r["goals"][1]["pose"][:2] == [-4.5, 5.5] and r["goals"][1]["at_s"] == 50


@pytest.mark.parametrize("mutate,err", [
    (lambda r: r.update(bogus=1), "unknown key"),
    (lambda r: r.update(scenario="nope"), "scenario"),
    (lambda r: r.update(overrides={"robots": 2}), "not offered"),
    (lambda r: r.update(overrides={"world": "nope"}), "world"),
    (lambda r: r["goals"].append({"robot": "robot_1", "pose": [0, 0]}), "not in this run"),
    (lambda r: r["goals"].append({"robot": "robot_0", "station": "atlantis"}), "station"),
    (lambda r: r["goals"].append({"robot": "robot_0", "pose": [0, 0], "station": "shelf_a"}),
     "exactly one"),
    (lambda r: r.update(stop={"sim_s": 0}), "sim_s"),
    (lambda r: r.update(stop={"sim_s": 10, "until": ["tasks_done"]}), "tasks_done"),
    (lambda r: r.update(stop={"sim_s": 10, "until": ["explorer_done"]}), "explores"),
    (lambda r: r.update(metrics=[]), "at least one"),
    (lambda r: r.update(metrics=[{"metric": "tasks"}]), "no tasks"),
    (lambda r: r.update(repeats=0), "repeats"),
    (lambda r: r.update(record=["video"]), "record"),
    (lambda r: r.update(variants=[{"label": "x", "set": {"metrics": []}}]), "variants may set"),
    (lambda r: r.update(variants=[{"label": "x", "set": {"overrides.world": "nope"}}]),
     "variant 'x'"),
])
def test_validation_rejects(mutate, err):
    r = raw()
    mutate(r)
    with pytest.raises(spec.SpecError, match=err):
        spec.validate(r)


def test_variants_resolve_independently():
    t = spec.validate(raw(variants=[
        {"label": "cross", "set": {"overrides.world": "aisle_crossing"}},
        {"label": "head_on", "set": {"overrides.world": "aisle_head_on", "stop.sim_s": 40}}]))
    (l1, a), (l2, b) = t["variants"]
    assert (l1, l2) == ("cross", "head_on")
    assert a["world"]["name"] == "aisle_crossing" and b["world"]["name"] == "aisle_head_on"
    assert b["stop"]["sim_s"] == 40 and a["stop"]["sim_s"] == 100


def test_streams_include_metric_needs():
    t = spec.validate(raw(metrics=[{"metric": "detection"}]))
    assert "dyn" in spec.streams_for(t, t["variants"][0][1])
    t = spec.validate(raw())
    assert "dyn" not in spec.streams_for(t, t["variants"][0][1])


# -- ProbeCore with a fake clock ---------------------------------------------------

class FakeRecorder:
    def __init__(self):
        self.events = []

    def write(self, ev):
        self.events.append(ev)


def _gt(sim, wt, running=True, x=-7.15, y=11.62):
    return {"t": sim, "wt": wt, "running": running, "world": "aisle_crossing",
            "robots": {"robot_0": {"x": x, "y": y, "yaw": 0.0}},
            "actors": {"walker_0": world.pose_at(
                world.load("aisle_crossing")["actors"][0]["path"], wt)}}


def _resolved_json(r):
    """What the probe actually receives: the resolved variant, through JSON."""
    t = spec.validate(r)
    res = t["variants"][0][1]
    return json.loads(json.dumps(res, default=list)), spec.streams_for(t, res)


def test_probe_waits_for_ready_and_warmup_then_restarts_world():
    res, streams = _resolved_json(raw())
    core = ProbeCore(res, streams, recorder=FakeRecorder())
    core.on_clock(40.0)
    core.on_gt(_gt(40.0, 40.0))
    assert not core.want_restart()            # no map, no nav server yet
    core.on_map(object())
    core.nav_ready.add("robot_0")
    assert not core.want_restart()            # ready now, warmup starts at 40
    assert core.phase == "warmup"
    core.on_clock(42.5)
    assert core.want_restart() and core.phase == "t0"
    assert not core.want_restart()            # exactly once
    core.on_gt(_gt(42.6, 42.6))               # restart not landed: clock still ~whole run
    assert core.t0 is None
    core.on_gt(_gt(42.7, 0.1))                # landed
    assert core.t0 == pytest.approx(42.6) and core.phase == "running"


def _running_core(r=None):
    res, streams = _resolved_json(r or raw())
    rec = FakeRecorder()
    core = ProbeCore(res, streams, recorder=rec)
    core.on_clock(10.0)
    core.on_gt(_gt(10.0, 10.0))
    core.on_map(object())
    core.nav_ready.add("robot_0")
    core.want_restart()
    core.on_clock(12.0)
    core.want_restart()
    core.on_gt(_gt(12.1, 0.0))
    return core, rec


def test_goals_sequence_per_robot_and_at_s_is_no_earlier_than():
    core, rec = _running_core()
    first = core.due_goals()
    assert [g["id"] for g in first] == ["g0"]
    core.on_goal_state("g0", "sent")
    assert core.due_goals() == []             # g1 waits for g0 (and for at_s 50)
    core.on_clock(12.1 + 20)
    core.on_goal_state("g0", "succeeded")
    assert core.due_goals() == []             # g0 done but at_s=50 not reached
    core.on_clock(12.1 + 50)
    assert [g["id"] for g in core.due_goals()] == ["g1"]
    states = [e["state"] for e in rec.events if e["s"] == "goal"]
    assert states == ["sent", "succeeded"]


def test_goal_timeout_and_late_result_ignored():
    core, rec = _running_core()
    core.due_goals()
    core.on_goal_state("g0", "sent")
    core.on_clock(12.1 + 31)
    assert [g["id"] for g in core.timed_out_goals()] == ["g0"]
    core.on_goal_state("g0", "timeout")
    core.on_goal_state("g0", "canceled")      # the cancel's own result: ignored
    assert core.goals["g0"]["state"] == "timeout"


def test_stop_until_goals_done_with_settle():
    core, _ = _running_core()
    for g in ("g0", "g1"):
        core.goals[g]["state"] = "succeeded"
    core.on_clock(12.1 + 60)
    assert core.stop_reason() is None and core.phase == "settle"
    core.on_clock(12.1 + 62.5)
    assert core.stop_reason() == "goals_done"


def test_stop_at_sim_cap_and_score():
    core, rec = _running_core()
    core.on_clock(12.1 + 100)
    assert core.stop_reason() == "sim_s"
    score = core.finish("sim_s", wall_s=250.0)
    assert rec.events[0]["s"] == "start" and rec.events[-1]["s"] == "end"
    assert rec.events[-1]["sim_s"] == pytest.approx(100.0)
    assert {m["id"] for m in score["metrics"]} == {"clearance", "goals"}
    goals = next(m for m in score["metrics"] if m["id"] == "goals")
    assert goals["values"]["unfinished"] == 2 and goals["values"]["success_rate"] == 0.0


def test_events_before_t0_are_dropped():
    res, streams = _resolved_json(raw())
    rec = FakeRecorder()
    core = ProbeCore(res, streams, recorder=rec)
    core.on_clock(5.0)
    core.on_gt(_gt(5.0, 5.0))
    core.on_task("robot_0", {"task_id": "x", "state": "IDLE"})
    assert rec.events == []


# -- report ---------------------------------------------------------------------------

def _result(variant, repeat, ok, contacts):
    return {"variant": variant, "repeat": repeat, "ok": ok,
            "meta": {"sim_s": 40.0, "stop_reason": "goals_done"},
            "score": {"ok": ok, "metrics": [{
                "id": "clearance", "metric": "clearance", "version": 1, "params": {},
                "values": {"min_m": 0.4, "contacts": contacts},
                "detail": {}, "ok": ok,
                "criteria": [{"value": "contacts", "op": "eq", "threshold": 0,
                              "actual": contacts, "ok": ok, "no_data": False}]}]}}


def test_verdicts_and_report():
    test = {"name": "unit", "title": "Unit", "pass_rate": 0.5, "repeats": 2,
            "variants": [{"label": "a"}, {"label": "b"}]}
    results = [_result("a", 0, True, 0), _result("a", 1, False, 2),
               _result("b", 0, False, 1),
               {"variant": "b", "repeat": 1, "ok": False, "error": "sim never came up"}]
    vd = report.verdicts(results, 0.5, ["a", "b"])
    assert vd["a"]["ok"] and not vd["b"]["ok"] and not vd["_all"]
    assert vd["b"]["errors"] == 1
    md = report.render(test, results)
    assert md.startswith("# unit — FAIL")
    assert "| a | clearance | contacts == 0 | 0, 2 | 1/2 |" in md
    assert "sim never came up" in md


def test_report_flags_mixed_metric_versions():
    a, b = _result("a", 0, True, 0), _result("a", 1, True, 0)
    b["score"]["metrics"][0]["version"] = 2
    assert report.version_notes([a, b])
