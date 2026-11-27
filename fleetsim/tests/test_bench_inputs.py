"""The benchmark's inputs (C12): floor options, arrivals and disturbances
that scale with the fleet. One scenario must run unchanged at 2, 5, 10 and
20 robots, so every input that names robots or a rate is checked here at
several fleet sizes."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from fleetsim import config, sim  # noqa: E402
from fleetsim import grid as G  # noqa: E402
from fleetsim.planner import FREE, OCC, Costmap  # noqa: E402
from test_sim import run, scenario, score  # noqa: E402

NAV = {"inflation_m": 0.65, "cost_scaling": 2.5, "cost_weight": 3.0, "unknown_cost": 0.0}
BODY_R = 0.25


def costmap(occ, g):
    return Costmap(g, np.where(occ, OCC, FREE).astype(np.int8), NAV, BODY_R)


def lanes(occ, g, x, y):
    """Drivable cell centres across the aisle at (x, y): their y values."""
    cm = costmap(occ, g)
    cx, cy = g.cell(x, y)
    ys = []
    for dy in range(-10, 11):
        if 0 <= cy + dy < g.height and not cm.lethal(int((cy + dy) * g.width + cx)):
            ys.append(g.centre(cx, cy + dy)[1])
    return ys


# ---- floor options ------------------------------------------------------------

@pytest.mark.parametrize("cell", [0.2, 0.1])
def test_a_narrow_aisle_is_drivable_but_one_robot_wide(cell):
    occ, g, wps, homes, _ = G.synthetic_warehouse({"narrow_aisles": [2]}, cell)
    ay = wps["shelf_2_1"][1]
    # Beside a rack segment: 1.0 m free, so no two drivable centres are far
    # enough apart for two 0.25 m bodies plus the local layer's 0.1 m margin.
    free = ~occ[:, g.cell(9.0, ay)[0]]
    cy = g.cell(9.0, ay)[1]
    run_ = [k for k in range(cy - 10, cy + 11) if free[k]]
    assert len(run_) * cell == pytest.approx(1.0)
    ys = lanes(occ, g, 9.0, ay)
    assert ys and max(ys) - min(ys) < 2 * BODY_R + 0.1
    # At the cross-aisle the aisle keeps its full width.
    assert len(lanes(occ, g, 16.1, ay)) > len(ys)
    # And a robot can still drive in to the shelf in the middle of it.
    assert costmap(occ, g).plan(homes[0], wps["shelf_2_1"]) is not None
    # The aisles beside it are untouched.
    plain, *_ = G.synthetic_warehouse({}, cell)
    row = g.cell(9.0, wps["shelf_4_1"][1])[1]
    assert (occ[row] == plain[row]).all()


def test_a_dead_end_segment_is_entered_from_one_end_only():
    occ, g, wps, homes, _ = G.synthetic_warehouse({"dead_ends": [[3, 1, "east"]]}, 0.2)
    cm = costmap(occ, g)
    ay = wps["shelf_3_2"][1]
    # Just east of segment 1 (x 17.3-28.8), in the cross-aisle, to just inside
    # the aisle: 1 m apart, but the wall sends the path round the far end.
    p = cm.plan((30.0, ay), (28.0, ay))
    assert p is not None
    assert sum(math.dist(p[i], p[i + 1]) for i in range(len(p) - 1)) > 20.0
    plain = costmap(G.synthetic_warehouse({}, 0.2)[0], g).plan((30.0, ay), (28.0, ay))
    assert sum(math.dist(plain[i], plain[i + 1]) for i in range(len(plain) - 1)) < 3.0


def test_options_that_seal_a_shelf_fail_validation_naming_it():
    with pytest.raises(config.ScenarioError, match="shelf_3_"):
        config.resolve({"name": "t", "config": {"world": {"synthetic": {
            "narrow_aisles": [3], "narrow_m": 0.4}}}})
    # Walls across both ends of aisle 2's segment 0 and its one cross-aisle.
    with pytest.raises(config.ScenarioError, match="shelf_2_0"):
        config.resolve({"name": "t", "config": {"world": {"synthetic": {
            "dead_ends": [[2, 0, "west"], [2, 0, "east"]]}}}})
    with pytest.raises(config.ScenarioError, match="unknown key"):
        config.resolve({"name": "t", "config": {"world": {"synthetic": {"narow_aisles": [3]}}}})
    with pytest.raises(config.ScenarioError, match="aisle 0"):
        config.resolve({"name": "t", "config": {"world": {"synthetic": {"narrow_aisles": [0]}}}})


def test_the_default_floor_keeps_every_waypoint_and_home():
    occ, g, wps, homes, gw = G.synthetic_warehouse({}, 0.2)
    occ2, _, wps2, homes2, _ = G.synthetic_checked({}, 0.2, clearance_m=0.35, tol_m=0.5)
    assert (occ == occ2).all() and wps == wps2 and homes == homes2


def test_a_wall_over_part_of_the_apron_drops_those_homes():
    _, _, _, homes, _ = G.synthetic_warehouse({}, 0.2)
    _, _, _, kept, _ = G.synthetic_checked({"walls": [[3.0, 3.0, 9.0, 8.0]]}, 0.2,
                                           clearance_m=0.35, tol_m=0.5)
    assert 0 < len(homes) - len(kept) < 20
    assert all(not (3.0 <= x <= 9.0 and 3.0 <= y <= 8.0) for x, y in kept)


# ---- arrivals ---------------------------------------------------------------

def _announces(**kw) -> int:
    return sim.Sim(scenario(tasks=[], **kw))._pending_announces


def test_a_per_robot_rate_keeps_the_load_per_robot():
    arr = {"rate_per_robot_hz": 0.05, "pickups": "shelf_*", "dropoffs": "dock_*"}
    per_robot = [_announces(robots=n, duration_s=2000, task_arrival=arr) / n for n in (2, 8)]
    assert per_robot == [pytest.approx(100, rel=0.2)] * 2
    fixed = {"rate_hz": 0.05, "pickups": "shelf_*", "dropoffs": "dock_*"}
    assert _announces(robots=2, duration_s=2000, task_arrival=fixed) == \
        _announces(robots=8, duration_s=2000, task_arrival=fixed)


def test_count_per_robot_caps_a_burst_exactly():
    arr = {"rate_per_robot_hz": 1.0, "start_s": 10, "count_per_robot": 3,
           "pickups": "shelf_*", "dropoffs": "dock_*"}
    assert [_announces(robots=n, task_arrival=arr) for n in (2, 5)] == [6, 15]


def test_arrival_rates_are_one_or_the_other():
    for arr in ({"rate_hz": 0.1, "rate_per_robot_hz": 0.01}, {"pickups": "*"},
                {"rate_per_robot_hz": 0}):
        with pytest.raises(config.ScenarioError, match="exactly one of"):
            scenario(task_arrival=arr)


# ---- disturbances -------------------------------------------------------------

@pytest.mark.parametrize("n,cut", [(2, 1), (5, 2), (10, 4), (20, 8)])
def test_a_fraction_partition_splits_any_fleet(n, cut):
    s = sim.Sim(scenario(robots=n))
    groups = [{"first": 0.4}, "rest"]
    a, b = (s._group(g, groups) for g in groups)
    assert a == set(range(1, cut + 1))  # node k + 1 is robot_k
    assert b == set(range(n + 1)) - a and sim.GW in b
    half = math.floor(0.5 * n + 0.5)
    assert s._group([{"last": 0.5}, "gateway"], []) == {sim.GW} | set(range(n - half + 1, n + 1))


def test_a_named_robot_missing_from_this_fleet_fails_validation():
    for d in ({"at_s": 1, "kill": "robot_3"}, {"at_s": 1, "block_link": ["robot_2", "gateway"]},
              {"at_s": 1, "partition": [["robot_0", "robot_2"], "rest"]}):
        with pytest.raises(config.ScenarioError, match="no robot_[23] in a fleet of 2"):
            scenario(robots=2, disturbances=[d])
    with pytest.raises(config.ScenarioError, match="fresh gateway"):
        scenario(disturbances=[{"at_s": 1, "kill": "gateway", "fresh": True}])


def test_requests_wait_for_a_dead_gateway_and_are_all_delivered():
    # Before 2026-11-26 `kill: gateway` killed the last robot instead.
    tasks = [{"at_s": t, "pickup": p, "dropoff": d} for t, p, d in (
        (1, "shelf_1_0", "dock_0"), (30, "shelf_2_1", "dock_1"), (60, "shelf_1_3", "dock_3"))]
    s, summary = run(scenario(robots=3, duration_s=500, tasks=tasks,
                              disturbances=[{"at_s": 20, "duration_s": 60, "kill": "gateway"}]))
    assert all(r.alive for r in s.robots)
    ann = [e for e in s.events if e["s"] == "announce"]
    assert [e.get("requested_s") for e in ann] == [None, 30.0, 60.0]
    assert [e["t"] for e in ann[1:]] == [80.0, 80.0]
    assert summary["reason"] == "tasks_done"
    v = score(s, [{"metric": "fleet_tasks"}])["fleet_tasks"]
    assert v["done"] == 3 and v["duplicates"] == 0


def test_kill_select_takes_the_robot_carrying_and_it_finishes_after_revival():
    s, summary = run(scenario(robots=4, duration_s=500, disturbances=[
        {"at_s": 40, "duration_s": 30, "kill": {"select": "carrying"}}]))
    d = next(e for e in s.events if e["s"] == "disturb" and e["on"])
    assert d["matched"] == "carrying"
    before = [e for e in s.events if e["s"] == "task" and e["rid"] == d["kill"] and e["t"] <= 40]
    assert before[-1]["state"] == "NAV_TO_DROPOFF"
    off = next(e for e in s.events if e["s"] == "disturb" and not e["on"])
    assert off["kill"] == d["kill"]  # the revival finds the robot the kill picked
    assert _body(off) == _body(d)  # the replay pairs a span's on and off by body
    assert summary["reason"] == "tasks_done"


def _body(ev):
    return {k: v for k, v in ev.items() if k not in ("t", "on")}


def test_kill_select_within_waits_for_a_match_and_keeps_the_outage_length():
    # At t=1 nobody carries anything yet: without within_s this took robot_0
    # idle on its home spot.
    s, _ = run(scenario(robots=4, duration_s=500, disturbances=[
        {"at_s": 1, "duration_s": 30, "kill": {"select": "carrying", "within_s": 200}}]))
    on, off = [e for e in s.events if e["s"] == "disturb"]
    assert on["matched"] == "carrying" and on["t"] > 10
    assert off["t"] == pytest.approx(on["t"] + 30)
    with pytest.raises(config.ScenarioError, match="within_s"):
        scenario(disturbances=[{"at_s": 1, "kill": {"select": "busy", "within_s": -1}}])


def test_a_select_with_nobody_doing_it_falls_back_to_a_busy_then_any_robot():
    s = sim.Sim(scenario(robots=3))
    assert s._select("carrying") == (s.robots[0], "any")
    s.robots[0].alive = False
    assert s._select("any") == (s.robots[1], "any")


def _cells(s, rect):
    g = s.grid
    c0, c1 = g.cell(rect[0], rect[1]), g.cell(rect[2], rect[3])
    return s.static[c0[1]:c1[1] + 1, c0[0]:c1[0] + 1]


def test_overlapping_blockages_come_and_go_independently():
    s = sim.Sim(scenario(robots=1))
    a = {"at_s": 0, "blockage": {"rect": [8.0, 12.0, 9.0, 14.4]}}
    b = {"at_s": 0, "blockage": {"rect": [8.6, 12.0, 9.6, 14.4]}}
    s._disturb(0, a, True)
    s._disturb(1, b, True)
    s._disturb(0, a, False)
    # The old code restored the static map under all of a, deleting b's
    # overlapping half: a pallet moved by half its length left a hole.
    assert _cells(s, [8.6, 12.0, 9.6, 14.4]).all()
    assert not _cells(s, [8.0, 12.1, 8.3, 14.3]).any()  # a's own half is floor again
    s._disturb(1, b, False)
    assert (s.static == s.base_static).all()


def test_a_blockage_waits_until_no_robot_is_under_it():
    s = sim.Sim(scenario(robots=1))
    r = s.robots[0]
    d = {"at_s": 0, "blockage": {"rect": [r.x - 0.5, r.y - 0.5, r.x + 0.5, r.y + 0.5]}}
    s._disturb(0, d, True)
    assert not [e for e in s.events if e["s"] == "disturb"]
    assert any(fn == s._disturb and a[0] == 0 for _t, _q, fn, a in s._heap)  # tries again
    assert s.try_move(r, r.x + 3.0, r.y)
    s.now = 1.5
    s._disturb(0, d, True)
    ev = [e for e in s.events if e["s"] == "disturb"]
    assert len(ev) == 1 and ev[0]["waited_s"] == 1.5
    assert _cells(s, d["blockage"]["rect"]).all()
    s._disturb(0, d, False)
    ev = [e for e in s.events if e["s"] == "disturb"]
    assert _body(ev[1]) == _body(ev[0]) and (s.static == s.base_static).all()


def test_a_revived_robot_does_not_catch_up_on_missed_status():
    scn = scenario(robots=3, duration_s=200, stop={},
                   disturbances=[{"at_s": 20, "duration_s": 60, "kill": "robot_0"}])
    s = sim.Sim(scn)
    sent = []
    tick = s._status_tick
    s._status_tick = lambda r: (sent.append((s.now, r.k)), tick(r))
    s.run()
    after = [t for t, k in sent if k == 0 and 80.0 <= t < 90.0]
    assert len(after) <= 10 * float(s.cfg["status"]["hz"]) + 1  # 5 Hz, not one per step
