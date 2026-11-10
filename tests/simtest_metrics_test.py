"""simtest.metrics: every registered metric against synthetic event streams,
criteria judging, and the record -> replay round trip rescoring depends on.
Offline: no ROS, no sim.

    /usr/bin/python3 -m pytest tests/simtest_metrics_test.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from simtest import events, metrics, world  # noqa: E402
from simtest.events import MapGrid  # noqa: E402

FOOTPRINT = ((0.25, 0.2), (0.25, -0.2), (-0.25, -0.2), (-0.25, 0.2))

# One walker crossing x = 0 along y = 0, from x=-4 to x=4 and back at 1 m/s.
WORLD = world.validate({
    "name": "unit",
    "props": [{"id": "crate", "shape": "box", "size": [1, 1, 1], "pose": [0, 5]}],
    "actors": [{"id": "w", "shape": "cylinder", "radius": 0.25, "height": 1.7,
                "path": {"waypoints": [[-4, 0], [4, 0]], "speed_mps": 1.0,
                         "mode": "pingpong"}}],
})


def ctx(**kw):
    base = dict(robots=["robot_0"], world=WORLD, footprint=FOOTPRINT,
                camera_hfov_rad=math.radians(87))
    base.update(kw)
    return metrics.Context(**base)


def gt(t, robot=(0.0, -3.0, math.pi / 2), wt=None):
    p = world.pose_at(WORLD["actors"][0]["path"], t if wt is None else wt)
    return {"s": "gt", "t": t, "wt": t, "robots": {"robot_0": dict(zip("xy", robot[:2]),
                                                                     yaw=robot[2])},
            "actors": {"w": p}}


# -- registry / validation -----------------------------------------------------

def test_every_metric_documents_itself():
    for c in metrics.catalog():
        assert c["summary"] and c["values"] and c["streams"], c["name"]
        assert set(c["streams"]) <= set(events.STREAMS), c["name"]


def test_validate_entry_resolves_defaults_and_criteria():
    e = metrics.validate_entry({"metric": "clearance", "pass": {"min_m": {"gt": 0.1}}}, 0)
    assert e["id"] == "clearance" and e["params"]["near_m"] == 0.5
    assert e["pass"] == [("min_m", "gt", 0.1)]


@pytest.mark.parametrize("entry,err", [
    ({"metric": "nope"}, "no metric"),
    ({"metric": "clearance", "params": {"bogus": 1}}, "unknown param"),
    ({"metric": "clearance", "params": {"near_m": "far"}}, "expected float"),
    ({"metric": "clearance", "pass": {"nothing": {"gt": 0}}}, "has no value"),
    ({"metric": "clearance", "pass": {"min_m": {"about": 0}}}, "unknown op"),
    ({"metric": "clearance", "pass": {"min_m": {"gt": "x"}}}, "number"),
    ({"metric": "clearance", "extra": 1}, "unknown key"),
])
def test_validate_entry_rejects(entry, err):
    with pytest.raises(metrics.MetricError, match=err):
        metrics.validate_entry(entry, 0)


def test_duplicate_ids_need_an_alias():
    with pytest.raises(metrics.MetricError, match="duplicate"):
        metrics.validate_list([{"metric": "clearance"}, {"metric": "clearance"}])
    ok = metrics.validate_list([{"metric": "clearance"},
                                {"metric": "clearance", "id": "clearance_props",
                                 "params": {"props": True}}])
    assert [e["id"] for e in ok] == ["clearance", "clearance_props"]


def test_check_context_fails_early():
    nothing = world.validate({"name": "empty"})
    with pytest.raises(metrics.MetricError, match="no actors"):
        metrics.MetricSet(metrics.validate_list([{"metric": "clearance"}]), ctx(world=nothing))
    with pytest.raises(metrics.MetricError, match="no goals"):
        metrics.MetricSet(metrics.validate_list([{"metric": "goals"}]), ctx())


def test_judge_no_data_fails():
    ms = metrics.MetricSet(metrics.validate_list(
        [{"metric": "clearance", "pass": {"min_m": {"gt": 0}}}]), ctx())
    s = ms.score()
    assert s["ok"] is False and s["metrics"][0]["criteria"][0]["no_data"] is True


# -- clearance -------------------------------------------------------------------

def test_clearance_robot_in_walkers_way():
    """Robot parked across the walker's line (centre at (0,0)): the walker
    walks straight through it — one contact per pass."""
    ms = metrics.MetricSet(metrics.validate_list(
        [{"metric": "clearance", "pass": {"contacts": {"eq": 0}}}]), ctx())
    for i in range(0, 161):          # 16 s = two passes at 10 Hz
        ms.feed(gt(i / 10, robot=(0.0, 0.0, 0.0)))
    s = ms.score()
    m = s["metrics"][0]["values"]
    assert m["min_m"] == 0.0 and m["contacts"] == 2
    # contact while |x| <= 0.25 + 0.25 (footprint half-length + radius): 1 s/pass
    assert m["contact_s"] == pytest.approx(2.0, abs=0.25)
    assert s["ok"] is False


def test_clearance_robot_clear_of_walker():
    ms = metrics.MetricSet(metrics.validate_list([{"metric": "clearance"}]), ctx())
    for i in range(0, 81):
        ms.feed(gt(i / 10, robot=(0.0, -3.0, math.pi / 2)))
    v = ms.score()["metrics"][0]["values"]
    # footprint rotated 90deg: half-length 0.25 along y -> gap = 3 - 0.25 - 0.25
    assert v["min_m"] == pytest.approx(2.5, abs=1e-3)
    assert v["contacts"] == 0 and v["near_s"] == 0.0


def test_clearance_props_option():
    ms = metrics.MetricSet(metrics.validate_list(
        [{"metric": "clearance", "params": {"props": True}}]), ctx())
    ms.feed(gt(0.0, robot=(0.0, 4.0, 0.0)))  # crate spans y 4.5..5.5
    v = ms.score()["metrics"][0]
    assert v["values"]["min_m"] == pytest.approx(0.3, abs=1e-3)
    assert "robot_0~crate" in v["detail"]


# -- goals -----------------------------------------------------------------------

def test_goals_success_time_and_error():
    goals = [{"robot": "robot_0", "pose": [0.0, 3.0, 0.0]},
             {"robot": "robot_0", "pose": [5.0, 5.0, 0.0]}]
    ms = metrics.MetricSet(metrics.validate_list(
        [{"metric": "goals", "pass": {"success_rate": {"gte": 1.0}}}]), ctx(goals=goals))
    ms.feed({"s": "goal", "t": 1.0, "rid": "robot_0", "goal_id": "g0", "state": "sent",
             "pose": [0.0, 3.0, 0.0]})
    for i in range(11):
        ms.feed(gt(1.0 + i, robot=(0.0, -3.0 + 0.59 * i, 0.0)))
    ms.feed({"s": "goal", "t": 11.0, "rid": "robot_0", "goal_id": "g0",
             "state": "succeeded"})
    ms.feed({"s": "goal", "t": 11.0, "rid": "robot_0", "goal_id": "g1", "state": "sent",
             "pose": [5.0, 5.0, 0.0]})
    ms.feed({"s": "goal", "t": 20.0, "rid": "robot_0", "goal_id": "g1", "state": "timeout"})
    s = ms.score()
    v = s["metrics"][0]["values"]
    assert (v["sent"], v["succeeded"], v["failed"], v["unfinished"]) == (2, 1, 1, 0)
    assert v["success_rate"] == 0.5 and v["mean_time_s"] == 10.0
    assert v["max_final_error_m"] == pytest.approx(0.1, abs=1e-6)
    assert v["path_m"] == pytest.approx(5.9, abs=1e-6)
    assert s["ok"] is False


def test_goals_never_sent_count_as_unfinished():
    goals = [{"robot": "robot_0", "pose": [0, 0, 0]}] * 3
    ms = metrics.MetricSet(metrics.validate_list([{"metric": "goals"}]), ctx(goals=goals))
    v = ms.score()["metrics"][0]["values"]
    assert v["unfinished"] == 3 and v["success_rate"] == 0.0


# -- map contamination -------------------------------------------------------------

def _grid(fill=0, size=200, res=0.05, origin=(-5.0, -5.0)):
    return MapGrid(size, size, res, origin[0], origin[1],
                   np.full(size * size, fill, dtype=np.int8))


def _paint(grid, x, y, value=100):
    c = int((x - grid.ox) / grid.resolution)
    r = int((y - grid.oy) / grid.resolution)
    grid.data[r, c] = value


def test_contamination_clean_and_dirty():
    ms = metrics.MetricSet(metrics.validate_list([{"metric": "map_contamination"}]), ctx())
    g = _grid()
    ms.feed({"s": "map", "t": 1.0, "grid": g})
    dirty = _grid()
    for x in np.arange(-1.0, 1.0, 0.05):
        _paint(dirty, x, 0.0)          # a smear along the walker's line
    _paint(dirty, 0.0, 3.0)            # far from the swept band: ignored
    ms.feed({"s": "map", "t": 2.0, "grid": dirty})
    ms.feed({"s": "map", "t": 3.0, "grid": g})
    v = ms.score()["metrics"][0]["values"]
    assert v["occupied_cells"] == 0 and v["occupied_frac"] == 0.0   # final map is clean
    assert v["peak_occupied_frac"] > 0.0                            # but it wasn't, mid-run
    assert v["observed_frac"] == 1.0 and v["maps"] == 3


def test_contamination_counts_only_the_swept_band():
    ms = metrics.MetricSet(metrics.validate_list([{"metric": "map_contamination"}]), ctx())
    g = _grid()
    _paint(g, 0.0, 0.0)
    _paint(g, 0.0, 2.0)
    ms.feed({"s": "map", "t": 1.0, "grid": g})
    v = ms.score()["metrics"][0]["values"]
    assert v["occupied_cells"] == 1


def test_contamination_unknown_floor_is_not_observed():
    ms = metrics.MetricSet(metrics.validate_list([{"metric": "map_contamination"}]), ctx())
    ms.feed({"s": "map", "t": 1.0, "grid": _grid(fill=-1)})
    v = ms.score()["metrics"][0]["values"]
    assert v["observed_frac"] == 0.0 and v["occupied_frac"] is None


# -- detection -------------------------------------------------------------------

def _dyn(t, pts):
    return {"s": "dyn", "t": t, "rid": "robot_0", "pts": pts}


def test_detection_recall_latency_and_false_positives():
    ms = metrics.MetricSet(metrics.validate_list([{"metric": "detection"}]), ctx())
    robot = (0.0, -3.0, math.pi / 2)   # looking up the y axis at the walker's line
    for i in range(0, 81):
        t = i / 10
        ev = gt(t, robot=robot)
        ms.feed(ev)
        a = ev["actors"]["w"]
        visible = abs(math.atan2(a["x"], 3.0)) < math.radians(87 / 2 - 5)
        # perception: blind for the first ~0.6 s of the sighting (the walker
        # enters view at t~1.6), then sees it; a stray blob every 20th frame
        pts = [[a["x"] + 0.1, a["y"]]] * 4 if visible and t > 2.2 else []
        if i % 20 == 0:
            pts = pts + [[3.0, 3.0]] * 5
        ms.feed(_dyn(t, pts))
    v = ms.score()["metrics"][0]["values"]
    assert v["visible_frames"] > 0 and 0.5 < v["recall"] < 1.0
    assert v["fp_frames"] == 5 and v["fp_rate"] == pytest.approx(5 / 81, abs=1e-3)
    assert v["episodes"] == 1 and v["mean_latency_s"] == pytest.approx(0.6, abs=0.15)


def test_detection_ignores_actor_out_of_view():
    ms = metrics.MetricSet(metrics.validate_list([{"metric": "detection"}]), ctx())
    ms.feed(gt(0.0, robot=(0.0, -3.0, -math.pi / 2)))  # facing away
    ms.feed(_dyn(0.0, []))
    v = ms.score()["metrics"][0]["values"]
    assert v["visible_frames"] == 0 and v["recall"] is None and v["frames"] == 1


# -- run_stats / coverage / tasks ----------------------------------------------------

def test_run_stats():
    ms = metrics.MetricSet(metrics.validate_list([{"metric": "run_stats"}]), ctx())
    ms.feed({"s": "start", "t": 0.0, "wall": 100.0, "sim": 30.0})
    for i in range(101):
        ms.feed(gt(i / 10))
    ms.feed({"s": "end", "t": 10.0, "reason": "goals_done", "sim_s": 10.0, "wall_s": 20.0})
    v = ms.score()["metrics"][0]["values"]
    assert v == {"sim_s": 10.0, "wall_s": 20.0, "rtf": 0.5, "stop_reason": "goals_done",
                 "gt_hz": 10.1}


def test_stop_reason_criterion_is_string_eq():
    e = metrics.validate_entry({"metric": "run_stats",
                                "pass": {"stop_reason": {"eq": "goals_done"}}}, 0)
    ms = metrics.MetricSet([e], ctx())
    ms.feed({"s": "end", "t": 1.0, "reason": "sim_s", "sim_s": 1.0, "wall_s": 2.0})
    assert ms.score()["ok"] is False


def test_coverage():
    ms = metrics.MetricSet(metrics.validate_list([{"metric": "coverage"}]), ctx())
    g = _grid(fill=-1)
    g.data[:20, :20] = 0
    g.data[:20, 20:24] = 100
    ms.feed({"s": "map", "t": 60.0, "grid": g})
    ms.feed({"s": "end", "t": 120.0, "reason": "sim_s", "sim_s": 120.0, "wall_s": 240.0})
    v = ms.score()["metrics"][0]["values"]
    assert v["free_m2"] == 1.0 and v["known_m2"] == 1.2 and v["free_m2_per_min"] == 0.5


def test_tasks():
    ms = metrics.MetricSet(metrics.validate_list([{"metric": "tasks"}]),
                           ctx(tasks=[{"robot": "robot_0"}, {"robot": "robot_0"}]))
    ms.feed({"s": "dispatch", "t": 1.0, "rid": "robot_0", "task_id": "t0",
             "pickup": "shelf_a", "dropoff": "zone_a"})
    ms.feed({"s": "task", "t": 5.0, "rid": "robot_0", "task_id": "t0", "state": "PICKING"})
    ms.feed({"s": "task", "t": 9.0, "rid": "robot_0", "task_id": "t0", "state": "DONE"})
    ms.feed({"s": "task", "t": 9.0, "rid": "robot_0", "task_id": "other", "state": "DONE"})
    v = ms.score()["metrics"][0]["values"]
    assert (v["dispatched"], v["done"], v["unfinished"]) == (1, 1, 1)
    assert v["mean_time_s"] == 8.0 and v["success_rate"] == 0.5


# -- record -> replay ----------------------------------------------------------------

def test_recorded_events_rescore_identically(tmp_path):
    entries = metrics.validate_list([{"metric": "clearance"}, {"metric": "map_contamination"},
                                     {"metric": "run_stats"}])
    live = metrics.MetricSet(entries, ctx())
    rec = events.Recorder(tmp_path / "events.jsonl.gz")
    g = _grid()
    _paint(g, 1.0, 0.0)
    stream = [{"s": "start", "t": 0.0, "wall": 0.0, "sim": 12.0}]
    stream += [gt(i / 10, robot=(0.5, -0.6, 0.3)) for i in range(50)]
    stream += [{"s": "map", "t": 5.0, "grid": g},
               {"s": "end", "t": 5.0, "reason": "sim_s", "sim_s": 5.0, "wall_s": 9.0}]
    for ev in stream:
        live.feed(ev)
        rec.write(ev)
    rec.close()

    replay = metrics.MetricSet(entries, ctx())
    for ev in events.read(tmp_path / "events.jsonl.gz"):
        replay.feed(ev)
    assert replay.score() == live.score()
    assert events.recorded_streams(tmp_path / "events.jsonl.gz") == {"start", "gt", "map",
                                                                      "end"}
