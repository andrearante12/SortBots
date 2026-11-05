"""Pick and place in the test layer: package tasks in a test case, what the probe
records for them, and the pick_place metric. Offline: no ROS, no sim.

    /usr/bin/python3 -m pytest tests/simtest_pick_test.py
"""
from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from simtest import events, magnet, metrics, spec, world  # noqa: E402
from simtest.probe import ProbeCore, dispatch_body  # noqa: E402

# The saved map's named waypoints, without needing a saved map: the pick tests run on
# a fresh map and only a few of these cases name one.
WAYPOINTS = {"shelf_a": {"name": "shelf_a", "x": -4.5, "y": 5.5, "yaw": 0.0}}


@pytest.fixture(autouse=True)
def _waypoints(monkeypatch):
    monkeypatch.setattr(spec, "_named_waypoints", lambda run: WAYPOINTS)


PICK = {
    "name": "unit_pick",
    "scenario": "pick_bench",
    "warmup_s": 2,
    "tasks": [{"robot": "robot_0", "package": "pkg_0", "place": {"shift": [-0.12, 0]}}],
    "stop": {"sim_s": 120, "until": ["tasks_done"], "settle_s": 2},
    "metrics": [{"metric": "pick_place"}, {"metric": "tasks"}],
}


def pick(**kw):
    r = copy.deepcopy(PICK)
    r.update(kw)
    return r


def resolved(**kw):
    return spec.validate(pick(**kw))["variants"][0][1]


# -- the committed pick tests ----------------------------------------------------

PICK_TESTS = sorted(p.stem for p in spec.TESTS_DIR.glob("pick_*.yaml"))


def test_the_pick_tests_exist():
    assert {"pick_place_smoke", "pick_orientations", "pick_heights", "pick_route",
            "pick_unreachable", "pick_detect_sweep"} <= set(PICK_TESTS)


@pytest.mark.parametrize("name", PICK_TESTS)
def test_committed_pick_tests_validate(name):
    try:
        t = spec.load(name)
    except spec.SpecError as e:
        if "map '" in str(e) and ("no map named" in str(e) or "pose graph is" in str(e)):
            pytest.skip(f"saved map not present here: {e}")
        raise
    for _, r in t["variants"]:
        assert all(m["metric"] in {x["metric"] for x in r["metrics"]} for m in r["metrics"])


# pick_detect_sweep has no tasks (perception only) and pick_unreachable is a one-package
# negative case by design; every other pick test must use all of its world's packages.
@pytest.mark.parametrize("name", [n for n in PICK_TESTS
                                  if n not in ("pick_detect_sweep", "pick_unreachable")])
def test_pick_tests_ask_for_every_package_in_their_world(name):
    """A test that leaves a package out is testing less than its world offers."""
    try:
        t = spec.load(name)
    except spec.SpecError:
        pytest.skip("saved map not present here")
    label, r = t["variants"][0]
    asked = {task["package"] for task in r["tasks"]}
    assert asked == {p["id"] for p in r["world"]["packages"]}, (name, label)


def test_smoke_test_runs_camera_and_oracle_variants():
    t = spec.load("pick_place_smoke")
    assert [label for label, _ in t["variants"]] == ["camera", "oracle"]
    sm = spec._session()
    argv = {label: sm.build_argv(r["run"], "sim") for label, r in t["variants"]}
    assert "--perception" not in argv["camera"]                    # the stock value emits nothing
    i = argv["oracle"].index("--perception")
    assert argv["oracle"][i + 1] == "oracle"
    assert argv["oracle"][:i] + argv["oracle"][i + 2:] == argv["camera"]   # and nothing else differs


def test_perception_is_an_allowlisted_run_key_with_two_values():
    sm = spec._session()
    assert sm.build_argv({"perception": "oracle"}, "sim") == ["--perception", "oracle"]
    assert sm.build_argv({"perception": "camera"}, "sim") == []
    with pytest.raises(sm.ScenarioError, match="expected one of"):
        sm.build_argv({"perception": "telepathy"}, "sim")
    # sim-only: ground truth does not exist on the real robot, so a real scenario naming it
    # is rejected when it loads (the same rule as every other sim-only key)
    assert "perception" in sm.RUN_FLAGS and "perception" not in sm.REAL_RUN_FLAGS
    assert sm.RUN_DEFAULTS["perception"] == "camera"


def test_a_test_may_only_override_perception_where_the_scenario_allows_it():
    # moving_obstacles has no `perception` in its overrides: the pick controller isn't its business
    with pytest.raises(spec.SpecError, match="not offered"):
        spec.validate(pick(scenario="moving_obstacles", overrides={"perception": "oracle"},
                           tasks=[], stop={"sim_s": 10}, metrics=[{"metric": "run_stats"}]))


# -- spec: package tasks ---------------------------------------------------------

def test_package_task_resolves_legs_and_place():
    task = resolved()["tasks"][0]
    assert task["package"] == "pkg_0"
    assert task["pickup"] is None and task["dropoff"] is None       # dock: no Nav2 leg
    # shifted from where the box started (-7.9, 9.9), on the bench that holds it up
    assert task["place"] == {"x": -8.02, "y": 9.9, "z": 0.8, "yaw": 1.5708, "support": "bench_a"}


def test_package_task_legs_may_be_poses_or_waypoints():
    t = resolved(tasks=[{"robot": "robot_0", "package": "pkg_0", "pickup": [-7.9, 11.0, -1.5],
                         "dropoff": "shelf_a", "place": [-7.8, 9.9]}])["tasks"][0]
    assert t["pickup"] == {"name": "pose", "x": -7.9, "y": 11.0, "yaw": -1.5}
    assert t["dropoff"] == {"name": "shelf_a", "x": -4.5, "y": 5.5, "yaw": 0.0}
    assert t["place"]["yaw"] == 1.5708                              # keeps the box's own yaw


def test_place_on_the_floor_is_a_place_too():
    t = resolved(tasks=[{"robot": "robot_0", "package": "pkg_0", "place": [-9.5, 9.9]}])["tasks"][0]
    assert t["place"]["support"] == "floor" and t["place"]["z"] == 0.0


@pytest.mark.parametrize("task,err", [
    ({"package": "pkg_9", "place": [-7.9, 9.9]}, "not in this run's world"),
    ({"package": "pkg_0"}, "says where the box comes to rest"),
    ({"package": "pkg_0", "place": {"shift": [-0.33, 0]}}, "floating"),      # hangs off the bench
    ({"package": "pkg_0", "place": {"shift": [1]}}, "shift"),
    ({"package": "pkg_0", "place": "table"}, "expected"),
    ({"package": "pkg_0", "place": [-7.9, 9.9], "pickup": 3}, "expected dock"),
    ({"package": "pkg_0", "place": [-7.9, 9.9], "pickup": [1, 2]}, "pose is"),
    ({"package": "pkg_0", "place": [-7.9, 9.9], "dropoff": "atlantis"}, "not one of the map's"),
    ({"pickup": "shelf_a", "dropoff": "shelf_a", "place": [0, 0]}, "only a package task"),
])
def test_package_task_rejections(task, err):
    with pytest.raises(spec.SpecError, match=err):
        spec.validate(pick(tasks=[{"robot": "robot_0", **task}]))


def test_package_task_needs_a_world_with_that_package():
    with pytest.raises(spec.SpecError, match="has no packages"):
        spec.validate(pick(overrides={"world": "aisle_crossing"}))


def test_pick_place_metric_needs_a_world_with_packages():
    with pytest.raises(spec.SpecError, match="has no packages"):
        spec.validate(pick(overrides={"world": "aisle_crossing"}, tasks=[],
                           stop={"sim_s": 10}))


def test_pick_place_without_package_tasks_is_a_perception_test():
    r = resolved(tasks=[], stop={"sim_s": 10}, metrics=[{"metric": "pick_place"}])
    assert r["tasks"] == [] and len(r["world"]["packages"]) == 1


def test_pick_place_streams_include_detect_and_gt():
    t = spec.validate(pick())
    assert {"gt", "dispatch", "task", "detect"} <= spec.streams_for(t, t["variants"][0][1])


# -- the probe -------------------------------------------------------------------

class FakeRecorder:
    def __init__(self):
        self.events = []

    def write(self, ev):
        self.events.append(ev)


def _gt(sim, wt, packages=None, magnet_state=None):
    out = {"t": sim, "wt": wt, "running": True, "world": "pick_bench",
           "robots": {"robot_0": {"x": -7.15, "y": 11.62, "yaw": 0.0}}, "actors": {}}
    if packages is not None:
        out["packages"] = packages
    if magnet_state is not None:
        out["magnet"] = magnet_state
    return out


def _resolved_json(r):
    t = spec.validate(r)
    res = t["variants"][0][1]
    return json.loads(json.dumps(res, default=list)), spec.streams_for(t, res)


def _core():
    res, streams = _resolved_json(pick())
    rec = FakeRecorder()
    core = ProbeCore(res, streams, recorder=rec)
    core.on_clock(10.0)
    core.on_gt(_gt(10.0, 10.0))
    core.on_map(object())
    core.want_restart()
    core.on_clock(12.0)
    core.want_restart()
    core.on_gt(_gt(12.1, 0.0, {"pkg_0": {"x": -7.9, "y": 9.9, "z": 0.8, "yaw": 1.5708,
                                          "held_by": None}},
                   {"robot_0": {"energized": False, "held": None}}))
    assert core.t0 is not None
    return core, rec


def test_dispatch_body_is_unchanged_for_plain_deliveries():
    plain = {"task_id": "simtest_t0", "pickup": "a", "dropoff": "b", "package": None, "place": None}
    assert dispatch_body(plain) == {"pickup": "a", "dropoff": "b", "task_id": "simtest_t0"}
    task = dict(resolved()["tasks"][0], task_id="simtest_t0")
    body = dispatch_body(task)
    assert body["package"] == "pkg_0" and body["pickup"] is None
    assert body["place"] == {"x": -8.02, "y": 9.9, "z": 0.8, "yaw": 1.5708}
    # the work order: what the box is and roughly where, not its pose or which way it faces
    assert body["size"] == [0.24, 0.18, 0.12] and body["face"] == "+x"
    assert body["at"] == {"x": -7.9, "y": 9.9}
    json.dumps(body)


def test_probe_passes_packages_and_magnet_through_ground_truth():
    _, rec = _core()
    first = next(e for e in rec.events if e["s"] == "gt")
    assert first["packages"]["pkg_0"]["held_by"] is None and "magnet" in first


def test_a_world_without_packages_leaves_the_gt_event_as_it_was():
    res, streams = _resolved_json(pick(overrides={"world": "aisle_crossing"},
                                       tasks=[], stop={"sim_s": 10},
                                       metrics=[{"metric": "run_stats"}]))
    rec = FakeRecorder()
    core = ProbeCore(res, streams, recorder=rec)
    core.on_clock(10.0)
    core.on_gt(_gt(10.0, 10.0))
    core.on_map(object())
    core.want_restart()
    core.on_clock(12.0)
    core.want_restart()
    core.on_gt(_gt(12.1, 0.0))
    ev = next(e for e in rec.events if e["s"] == "gt")
    assert "packages" not in ev and "magnet" not in ev


def test_probe_records_detections_throttled_and_trimmed():
    core, rec = _core()
    plates = {"plates": [{"x": 1, "y": 2, "z": 3, "nx": 0, "ny": 1, "nz": 0, "conf": 0.9,
                          "px": [1, 2, 3, 4]}]}
    core.on_packages("robot_0", plates)
    core.on_packages("robot_0", plates)               # inside the 0.5 s throttle
    core.on_clock(12.1 + 1.0)
    core.on_packages("robot_0", plates)
    det = [e for e in rec.events if e["s"] == "detect"]
    assert len(det) == 2
    assert det[0]["plates"] == [{"x": 1, "y": 2, "z": 3, "nx": 0, "ny": 1, "nz": 0, "conf": 0.9}]


def test_a_rejected_task_ends_as_failed_instead_of_hanging():
    core, rec = _core()
    (task,) = core.due_tasks()
    assert task["task_id"] == "simtest_t0"
    core.on_task("robot_0", {"task_id": None, "state": "IDLE",
                             "rejected": {"task_id": "simtest_t0", "reason": "bad leg"}})
    assert core.tasks["t0"]["state"] == "FAILED"
    ev = [e for e in rec.events if e["s"] == "task"][-1]
    assert ev["task_id"] == "simtest_t0" and ev["state"] == "FAILED" and ev["reason"] == "bad leg"
    core.on_clock(12.1 + 5)
    assert core.stop_reason() is None and core.phase == "settle"
    core.on_clock(12.1 + 8)
    assert core.stop_reason() == "tasks_done"


# -- the pick_place metric -------------------------------------------------------

WORLD = world.load("pick_bench")
SIZE = WORLD["packages"][0]["size"]
START = {"x": -7.9, "y": 9.9, "z": 0.8, "yaw": 1.5708}
PLACE = {"x": -8.02, "y": 9.9, "z": 0.8, "yaw": 1.5708}


def mctx():
    return metrics.Context(
        robots=["robot_0"], world=WORLD,
        tasks=[{"id": "t0", "robot": "robot_0", "package": "pkg_0", "pickup": None,
                "dropoff": None, "place": dict(PLACE, support="bench_a"), "at_s": 0.0}])


def pgt(t, pose, held_by=None):
    return {"s": "gt", "t": t, "wt": t, "robots": {"robot_0": {"x": 0, "y": 0, "yaw": 0}},
            "actors": {}, "packages": {"pkg_0": dict(pose, held_by=held_by)}}


def dispatch(t=0.5, package="pkg_0"):
    return {"s": "dispatch", "t": t, "rid": "robot_0", "task_id": "simtest_t0", "pickup": None,
            "dropoff": None, "package": package, "place": PLACE}


def run_metric(stream, **params):
    m = metrics.get("pick_place")(mctx(), **params)
    for ev in stream:
        m.on_event(ev)
    return m


def good_stream():
    s = [dispatch(), pgt(0.0, START)]
    s += [pgt(1.0 + i, START) for i in range(3)]                       # waiting
    s += [pgt(10.0, dict(START, z=1.0, x=-7.9), "robot_0"),            # picked at t=10
          pgt(14.0, dict(PLACE, z=1.0), "robot_0")]                    # carried
    s += [pgt(15.0, PLACE), pgt(16.0, PLACE),                          # let go, at rest
          {"s": "task", "t": 16.0, "rid": "robot_0", "task_id": "simtest_t0", "state": "DONE"}]
    return s


def test_a_clean_pick_and_place_scores_as_placed():
    v = run_metric(good_stream()).result()
    assert (v["planned"], v["picked"], v["placed"], v["success_rate"]) == (1, 1, 1, 1.0)
    assert v["drops"] == 0 and v["false_picks"] == 0
    assert v["mean_time_to_pick_s"] == pytest.approx(9.5)
    assert v["max_place_error_m"] == pytest.approx(0.0) and v["max_place_z_error_m"] == 0.0


def test_never_picked_is_no_data_not_a_pass():
    s = [dispatch()] + [pgt(float(i), START) for i in range(20)]
    v = run_metric(s).result()
    assert v["picked"] == 0 and v["placed"] == 0 and v["success_rate"] == 0.0
    assert v["max_place_error_m"] is None            # so a criterion on it FAILS (no data)


def test_letting_go_in_the_wrong_place_is_a_drop_and_not_a_place():
    wrong = {"x": -7.0, "y": 9.0, "z": 0.0, "yaw": 0.0}
    s = [dispatch(), pgt(0.0, START), pgt(5.0, dict(START, z=1.0), "robot_0"),
         pgt(8.0, wrong), pgt(9.0, wrong)]
    v = run_metric(s).result()
    assert v["picked"] == 1 and v["placed"] == 0 and v["drops"] == 1
    assert v["max_place_error_m"] > 1.0


def test_right_spot_wrong_height_is_not_placed():
    sunk = dict(PLACE, z=0.0)                          # fell through the bench: a bad release
    s = [dispatch(), pgt(0.0, START), pgt(5.0, dict(START, z=1.0), "robot_0"),
         pgt(8.0, sunk), pgt(9.0, sunk)]
    v = run_metric(s).result()
    assert v["picked"] == 1 and v["placed"] == 0 and v["max_place_z_error_m"] == pytest.approx(0.8)


def test_still_holding_it_at_the_end_is_not_placed():
    s = [dispatch(), pgt(0.0, START), pgt(5.0, dict(PLACE, z=1.0), "robot_0"),
         pgt(6.0, dict(PLACE, z=1.0), "robot_0")]
    v = run_metric(s).result()
    assert v["picked"] == 1 and v["placed"] == 0


def _hold_dispatch():
    return dict(dispatch(), place=None)


def test_a_pick_only_task_succeeds_by_still_holding_the_package():
    s = [_hold_dispatch(), pgt(0.0, START), pgt(5.0, dict(START, z=1.0), "robot_0"),
         pgt(9.0, dict(START, z=1.1), "robot_0")]
    v = run_metric(s).result()
    assert (v["picked"], v["held_at_end"], v["placed"], v["success_rate"]) == (1, 1, 0, 1.0)
    assert v["drops"] == 0 and v["max_place_error_m"] is None


def test_a_pick_only_task_that_lets_go_dropped_it():
    s = [_hold_dispatch(), pgt(0.0, START), pgt(5.0, dict(START, z=1.0), "robot_0"), pgt(6.0, START)]
    v = run_metric(s).result()
    assert (v["picked"], v["held_at_end"], v["drops"], v["success_rate"]) == (1, 0, 1, 0.0)


def test_pick_block_resolves_to_a_task_with_no_place_all_the_way_to_the_wire():
    (task,) = spec.load("pick_block")["variants"][0][1]["tasks"]
    assert task["place"] is None
    assert dispatch_body(dict(task, task_id="t"))["place"] is None


def test_place_block_sets_the_block_down_inside_the_marked_area():
    r = spec.load("place_block")["variants"][0][1]
    (task,), (mark,) = r["tasks"], r["world"]["marks"]
    assert abs(task["place"]["x"] - mark["pose"][0]) < mark["size"][0] / 2
    assert abs(task["place"]["y"] - mark["pose"][1]) < mark["size"][1] / 2
    assert task["place"]["z"] == mark["z"]


def test_the_tab_tests_give_the_robot_no_location_for_the_block_only_its_marker():
    for name in ("pick_block", "place_block"):
        (task,) = spec.load(name)["variants"][0][1]["tasks"]
        assert task["at"] is None and task["marker"] == 0
        body = dispatch_body(dict(task, task_id="t"))
        assert body["at"] is None and body["marker"] == 0


def test_the_tests_tab_offers_two_pick_tests_and_parked_ones_still_validate():
    listed = {t["name"]: t for t in spec.listed_tests()}
    assert {n for n, t in listed.items() if t["section"] == "Pick and place"} == {"pick_block", "place_block"}
    parked = {t["name"] for t in spec.list_tests() if t["parked"]}
    assert "pick_orientations" in parked and not parked & set(listed)
    assert spec.load("pick_orientations")["parked"] is True


def test_picking_a_package_nobody_asked_for_is_a_false_pick():
    s = [pgt(0.0, START), pgt(2.0, dict(START, z=1.0), "robot_0")]      # no dispatch at all
    v = run_metric(s).result()
    assert v["false_picks"] == 1 and v["picked"] == 0


def test_a_detection_on_the_true_plate_is_matched_and_measured():
    centre, normal = magnet.magnet_face_pose(START, SIZE, "+x")
    near = {"s": "detect", "t": 3.0, "rid": "robot_0", "plates": [
        {"x": centre[0] + 0.02, "y": centre[1], "z": centre[2], "nx": 0.0, "ny": 1.0, "nz": 0.0},
        {"x": 5.0, "y": 5.0, "z": 0.9, "nx": 1.0, "ny": 0.0, "nz": 0.0}]}            # a ghost
    tilted = dict(near, t=4.0, plates=[
        {"x": centre[0], "y": centre[1], "z": centre[2],
         "nx": 0.0, "ny": math.cos(math.radians(10)), "nz": math.sin(math.radians(10))}])
    v = run_metric([dispatch(), pgt(0.0, START), near, tilted]).result()
    assert v["detected"] == 1 and v["false_detections"] == 1
    assert v["mean_detect_err_m"] == pytest.approx(0.01)
    assert v["mean_normal_err_deg"] == pytest.approx(5.0, abs=1e-3)       # (0 + 10) / 2
    assert v["first_detect_s"] == 3.0


def test_a_plate_seen_while_the_package_is_held_is_not_the_true_one():
    centre, _ = magnet.magnet_face_pose(START, SIZE, "+x")
    held = pgt(5.0, dict(START, z=1.0), "robot_0")
    ghost = {"s": "detect", "t": 6.0, "rid": "robot_0", "plates": [
        {"x": centre[0], "y": centre[1], "z": centre[2], "nx": 0, "ny": 1, "nz": 0}]}
    v = run_metric([dispatch(), pgt(0.0, START), held, ghost]).result()
    assert v["detected"] == 0 and v["false_detections"] == 1


def test_perception_only_counts_every_package_in_the_world():
    centre, _ = magnet.magnet_face_pose(START, SIZE, "+x")
    seen = {"s": "detect", "t": 3.0, "rid": "robot_0", "plates": [
        {"x": centre[0], "y": centre[1], "z": centre[2], "nx": 0, "ny": 1, "nz": 0}]}
    m = metrics.get("pick_place")(metrics.Context(robots=["robot_0"], world=WORLD))
    for ev in (pgt(0.0, START), seen):
        m.on_event(ev)
    v = m.result()
    assert v["detected"] == 1 and v["planned"] == 0 and v["success_rate"] is None


def test_metric_context_checks():
    entries = metrics.validate_list([{"metric": "pick_place"}])
    with pytest.raises(metrics.MetricError, match="has no packages"):
        metrics.MetricSet(entries, metrics.Context(robots=["robot_0"],
                                                   world=world.load("aisle_crossing")))
    bad = metrics.Context(robots=["robot_0"], world=world.load("pick_low"),
                          tasks=[{"id": "t0", "package": "pkg_9", "place": PLACE}])
    with pytest.raises(metrics.MetricError, match="not in the test's world"):
        metrics.MetricSet(entries, bad)


def test_criteria_judge_and_missing_data_fails():
    entries = metrics.validate_list([{"metric": "pick_place", "pass": {
        "placed": {"gte": 1}, "max_place_error_m": {"lte": 0.15}, "drops": {"eq": 0}}}])
    good = metrics.MetricSet(entries, mctx())
    for ev in good_stream():
        good.feed(ev)
    assert good.score()["ok"]
    idle = metrics.MetricSet(entries, mctx())
    idle.feed(dispatch())
    idle.feed(pgt(0.0, START))
    score = idle.score()
    assert not score["ok"]
    nodata = [c for c in score["metrics"][0]["criteria"] if c["value"] == "max_place_error_m"][0]
    assert nodata["no_data"]


def test_recorded_events_rescore_identically(tmp_path):
    entries = metrics.validate_list([{"metric": "pick_place"}, {"metric": "tasks"}])
    live = metrics.MetricSet(entries, mctx())
    rec = events.Recorder(tmp_path / "events.jsonl.gz")
    centre, _ = magnet.magnet_face_pose(START, SIZE, "+x")
    stream = good_stream() + [{"s": "detect", "t": 2.0, "rid": "robot_0", "plates": [
        {"x": centre[0], "y": centre[1], "z": centre[2], "nx": 0, "ny": 1, "nz": 0}]}]
    stream.sort(key=lambda e: e["t"])
    for ev in stream:
        live.feed(ev)
        rec.write(ev)
    rec.close()
    replay = metrics.MetricSet(entries, mctx())
    for ev in events.read(tmp_path / "events.jsonl.gz"):
        replay.feed(ev)
    assert replay.score() == live.score()
    assert events.recorded_streams(tmp_path / "events.jsonl.gz") == {"dispatch", "gt", "task",
                                                                      "detect"}
