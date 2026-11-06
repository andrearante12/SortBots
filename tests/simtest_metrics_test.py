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



# -- reaction --------------------------------------------------------------------

def _braking_robot(t, brake_from=2.0, brake_s=1.0, v=0.6):
    """North along x=0 from y=-3 at v m/s, then a linear stop between brake_from
    and brake_from + brake_s (None = never brakes)."""
    if brake_from is None or t <= brake_from:
        return (0.0, -3.0 + v * t, math.pi / 2)
    tb = min(t - brake_from, brake_s)
    y = -3.0 + v * brake_from + v * tb - 0.5 * (v / brake_s) * tb * tb
    return (0.0, y, math.pi / 2)


def _reaction(robot_fn, seconds=8.0, **params):
    ms = metrics.MetricSet(metrics.validate_list(
        [{"metric": "reaction", "params": params}]), ctx())
    for i in range(int(seconds * 10) + 1):
        ms.feed(gt(i / 10, robot=robot_fn(i / 10)))
    m = ms.score()["metrics"][0]
    return m["values"], m["detail"]


def test_reaction_times_a_robot_that_brakes_for_the_walker():
    """Walker reaches x=0 at t=4; the robot (0.6 m/s north) is on a collision
    course from t~0.9 (range <= 4 m) and brakes 2 -> 3 s."""
    v, d = _reaction(_braking_robot)
    assert v["encounters"] == 1 and v["timed"] == 1 and v["reacted"] == 1
    e = next(iter(d.values()))
    assert e["onset_s"] == pytest.approx(0.9, abs=0.15)
    assert e["v0"] == pytest.approx(0.6, abs=0.05)
    # half speed at t=2.5, stopped at t~2.95: smoothing (0.5 s window) blurs each by < 0.3 s
    assert v["max_time_to_slow_s"] == pytest.approx(1.6, abs=0.3)
    assert v["max_time_to_stop_s"] == pytest.approx(2.05, abs=0.35)
    assert v["stopped"] == 1
    # driven from onset to standstill: 0.6 * (2 - 0.9) + 0.3 m of braking
    assert v["max_brake_m"] == pytest.approx(0.96, abs=0.2)
    assert 0.4 < v["max_decel_mps2"] < 0.75
    assert v["min_ttc_s"] is not None and 0 < v["min_ttc_s"] < 4
    assert v["yield_s"] > 0.2            # stopped ~2.95 s, released 1 s after the threat ends
    assert v["min_gap_m"] > 0.5 and v["samples"] == 81


def test_reaction_counts_a_robot_that_never_slows_as_unreacted():
    v, _ = _reaction(lambda t: _braking_robot(t, brake_from=None))
    assert v["encounters"] == 1 and v["reacted"] == 0 and v["unreacted"] == 1
    assert v["max_time_to_slow_s"] is None and v["max_time_to_stop_s"] is None
    assert v["min_ttc_s"] < 1.0


def test_reaction_no_encounter_when_the_walker_is_behind_or_parallel():
    # facing south (away from the walker's line): never a threat in front
    v, _ = _reaction(lambda t: (0.0, -3.0 - 0.3 * t, -math.pi / 2))
    assert v["encounters"] == 0 and v["max_time_to_slow_s"] is None
    # parked 3 m off the line, facing it: the walker passes 2.75 m clear -> no collision course
    v, _ = _reaction(lambda t: (0.0, -3.0, math.pi / 2))
    assert v["encounters"] == 0


def test_reaction_slow_robot_is_counted_but_not_timed():
    v, _ = _reaction(lambda t: (0.0, -1.2 + 0.1 * t, math.pi / 2))
    assert v["encounters"] >= 1 and v["timed"] == 0 and v["reacted"] == 0


def test_reaction_check_context_needs_actors():
    with pytest.raises(metrics.MetricError, match="no actors"):
        metrics.MetricSet(metrics.validate_list([{"metric": "reaction"}]),
                          ctx(world=world.validate({"name": "empty"})))
    with pytest.raises(metrics.MetricError, match="no actor"):
        metrics.MetricSet(metrics.validate_list(
            [{"metric": "reaction", "params": {"actors": ["ghost"]}}]), ctx())


# -- goals v2: detour and speed ----------------------------------------------------

def test_goals_detour_ratio_and_mean_speed():
    goals = [{"robot": "robot_0", "pose": [3.0, 0.0, 0.0]}]
    ms = metrics.MetricSet(metrics.validate_list([{"metric": "goals"}]), ctx(goals=goals))
    # L-shaped drive (0,0)->(0,2)->(3,2)->(3,0): 7 m for a 3 m leg, at 0.5 m/s
    path = [(0.0, i * 0.05) for i in range(41)] + [(i * 0.05, 2.0) for i in range(1, 61)] \
        + [(3.0, 2.0 - i * 0.05) for i in range(1, 41)]
    ms.feed(gt(0.0, robot=(0.0, 0.0, 0.0)))
    ms.feed({"s": "goal", "t": 0.0, "rid": "robot_0", "goal_id": "g0", "state": "sent",
             "pose": [3.0, 0.0, 0.0]})
    for i, (x, y) in enumerate(path):
        ms.feed(gt((i + 1) / 10, robot=(x, y, 0.0)))
    ms.feed({"s": "goal", "t": 14.0, "rid": "robot_0", "goal_id": "g0", "state": "succeeded",
             "pose": [3.0, 0.0, 0.0]})
    m = ms.score()["metrics"][0]
    assert m["values"]["max_detour_ratio"] == pytest.approx(7.0 / 3.0, abs=0.02)
    assert m["values"]["mean_speed_mps"] == pytest.approx(0.5, abs=0.01)
    assert m["detail"]["g0"]["detour_ratio"] == m["values"]["max_detour_ratio"]
    assert m["version"] == 2


# -- clearance v2: peers -----------------------------------------------------------

def _gt2(t, r0, r1):
    ev = gt(t, robot=r0)
    ev["robots"]["robot_1"] = dict(zip("xy", r1[:2]), yaw=r1[2])
    return ev


def test_clearance_peers_scores_robot_pairs():
    ms = metrics.MetricSet(metrics.validate_list(
        [{"metric": "clearance", "params": {"actors": [], "peers": True}}]),
        ctx(robots=["robot_0", "robot_1"]))
    ms.feed(_gt2(0.0, (0.0, -3.0, 0.0), (1.0, -3.0, 0.0)))   # 0.5 m apart nose to tail
    ms.feed(_gt2(0.1, (0.0, -3.0, 0.0), (0.4, -3.0, 0.0)))   # overlapping
    m = ms.score()["metrics"][0]
    assert m["values"]["contacts"] == 1 and m["values"]["min_m"] == 0.0
    assert list(m["detail"]) == ["robot_0~robot_1"]
    assert m["version"] == 2


def test_clearance_peers_needs_two_robots():
    with pytest.raises(metrics.MetricError, match="two robots"):
        metrics.MetricSet(metrics.validate_list(
            [{"metric": "clearance", "params": {"peers": True}}]), ctx())


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



# -- prediction --------------------------------------------------------------------

HORIZONS = (0.5, 1.0, 1.5)


def _trk(t, tracks, rid="robot_0"):
    return {"s": "trk", "t": t, "rid": rid, "stamp": t, "tracks": tracks}


def _true_track(t, *, tid=7, off=(0.0, 0.0), pred_off=(0.0, 0.0), moving=True):
    """A track that follows the walker exactly, with perfect constant-velocity
    look-ahead (the walker is on a straight leg for the whole window)."""
    a = world.pose_at(WORLD["actors"][0]["path"], t)
    return {"id": tid, "moving": moving, "hits": 10,
            "pos": [a["x"] + off[0], a["y"] + off[1]], "vel": [a["vx"], a["vy"]],
            "cov": [0.01, 0.0, 0.01],
            "pred": [[h, a["x"] + a["vx"] * h + pred_off[0], a["y"] + a["vy"] * h + pred_off[1]]
                     for h in HORIZONS]}


def _run_prediction(track_fn, t0=1.7, t1=3.0, robots=("robot_0",), peer=None, **params):
    ms = metrics.MetricSet(metrics.validate_list(
        [{"metric": "prediction", "params": params}]), ctx(robots=list(robots)))
    n = 0
    t = t0
    while t <= t1 + 1e-9:
        ev = gt(round(t, 2))
        if peer is not None:
            ev["robots"]["robot_1"] = {"x": peer[0], "y": peer[1], "yaw": 0.0}
        ms.feed(ev)
        ms.feed(_trk(round(t, 2), track_fn(round(t, 2))))
        n += 1
        t += 0.2
    m = ms.score()["metrics"][0]
    return m["values"], n


def test_prediction_perfect_tracker_scores_zero_error():
    """Walker at x = -4 + t on y = 0; the robot at (0, -3) faces north and has it in
    view from t ~ 1.6 (x = -2.4, 38.5 deg off axis) on."""
    v, n = _run_prediction(lambda t: [_true_track(t, off=(0.1, 0.0))])
    assert v["frames"] == n and v["visible_frames"] == n and v["assoc_recall"] == 1.0
    assert v["moving_recall"] == 1.0 and v["moving_visible_frames"] == n
    assert v["pos_rmse_m"] == pytest.approx(0.1, abs=1e-3)
    assert v["vel_rmse_mps"] == 0.0
    assert v["ade_m"] == 0.0 and v["fde_m"] == 0.0 and v["horizon_s"] == 1.5
    assert v["cv_fde_m"] == 0.0            # straight leg: constant velocity IS the truth
    assert v["fragmentation"] == 0.0 and v["episodes"] == 1
    assert v["false_moving_frames"] == 0 and v["false_moving_rate"] == 0.0


def test_prediction_offset_lookahead_reads_as_ade_and_fde():
    v, _ = _run_prediction(lambda t: [_true_track(t, pred_off=(0.0, 0.5))])
    assert v["ade_m"] == pytest.approx(0.5, abs=1e-3) and v["fde_m"] == pytest.approx(0.5, abs=1e-3)
    assert v["pos_rmse_m"] == 0.0


def test_prediction_counts_missed_moving_and_false_moving():
    # the tracker sees the walker but never calls it moving
    v, n = _run_prediction(lambda t: [_true_track(t, moving=False)])
    assert v["assoc_recall"] == 1.0 and v["moving_recall"] == 0.0
    # a moving track off in empty floor every frame: all false, unless it sits on a peer robot
    stray = {"id": 9, "moving": True, "hits": 10, "pos": [2.0, 2.0], "vel": [0.3, 0.0],
             "cov": None, "pred": []}
    v, n = _run_prediction(lambda t: [_true_track(t), stray])
    assert v["false_moving_frames"] == n
    v, n = _run_prediction(lambda t: [_true_track(t), stray], robots=("robot_0", "robot_1"),
                           peer=(2.1, 2.0))
    assert v["false_moving_frames"] == 0
    # no tracks at all: visible but never associated, and the criterion has data
    v, n = _run_prediction(lambda t: [])
    assert v["visible_frames"] == n and v["assoc_recall"] == 0.0 and v["ade_m"] is None


def test_prediction_fragmentation_counts_extra_ids_per_episode():
    v, n = _run_prediction(lambda t: [_true_track(t, tid=1 + int(round(t * 5)) % 3)])
    assert v["episodes"] == 1 and v["fragmentation"] == 2.0


def test_prediction_ignores_actor_out_of_view():
    # robot facing south: the walker is behind it the whole time
    ms = metrics.MetricSet(metrics.validate_list([{"metric": "prediction"}]), ctx())
    for i in range(10):
        t = 2.0 + i * 0.2
        ms.feed(gt(t, robot=(0.0, -3.0, -math.pi / 2)))
        ms.feed(_trk(t, [_true_track(t)]))
    v = ms.score()["metrics"][0]["values"]
    assert v["frames"] == 10 and v["visible_frames"] == 0 and v["assoc_recall"] is None


def test_prediction_check_context_needs_actors():
    with pytest.raises(metrics.MetricError, match="no actors"):
        metrics.MetricSet(metrics.validate_list([{"metric": "prediction"}]),
                          ctx(world=world.validate({"name": "empty"})))


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
