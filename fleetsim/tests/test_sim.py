"""fleetsim end to end, on a small floor so the whole file runs in seconds."""
from __future__ import annotations

import copy
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fleetsim import config, sim  # noqa: E402
from simtest import metrics as metrics_lib  # noqa: E402

SMALL = {"width_m": 30.0, "height_m": 22.0, "docks": 4, "cross_every_m": 12.0}


def scenario(**over):
    raw = {"name": "t", "seed": 1, "duration_s": 400, "robots": 3,
           "config": {"world": {"synthetic": SMALL}, "map_share": {"prior": "truth"}},
           "tasks": [{"at_s": 1, "pickup": "shelf_1_0", "dropoff": "dock_0"},
                     {"at_s": 2, "pickup": "shelf_2_2", "dropoff": "dock_2"},
                     {"at_s": 3, "pickup": "shelf_3_1", "dropoff": "dock_3"}],
           "stop": {"until": "tasks_done", "settle_s": 1}}
    for k, v in over.items():
        if k == "config":
            raw["config"] = sim.deep_merge(raw["config"], v)
        else:
            raw[k] = v
    return config.resolve(raw)


def run(scn):
    s = sim.Sim(scn)
    summary = s.run()
    return s, summary


def score(s, entries):
    ctx = metrics_lib.Context(robots=[r.rid for r in s.robots], world={"actors": s.actors},
                              tasks=s.announced)
    ms = metrics_lib.MetricSet(metrics_lib.validate_list(entries), ctx)
    for ev in s.events:
        ms.feed(ev)
    return {m["id"]: m["values"] for m in ms.score()["metrics"]}


def test_small_fleet_delivers_everything_without_contact():
    s, summary = run(scenario())
    assert summary["reason"] == "tasks_done"
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "contacts"}, {"metric": "tasks"}])
    assert v["fleet_tasks"]["done"] == 3 and v["fleet_tasks"]["duplicates"] == 0
    assert v["contacts"]["contacts"] == 0
    assert v["tasks"]["done"] == 3  # the existing simtest metric reads the same streams


def test_a_seed_reproduces_a_run_exactly():
    a, _ = run(scenario())
    b, _ = run(scenario())
    strip = lambda evs: [{k: v for k, v in e.items() if k not in ("wall", "wall_s", "summary")}
                         for e in evs]
    assert strip(a.events) == strip(b.events)


def test_robots_start_blind_and_learn_the_floor():
    s, _ = run(scenario(config={"map_share": {"prior": "none"}}))
    v = score(s, [{"metric": "map_divergence"}])["map_divergence"]
    assert 0.0 < v["mean_known_free"] < v["final_known_free"] <= 1.0
    assert v["final_wrong_occ"] == 0


def test_shared_blockage_reaches_robots_that_never_saw_it():
    # A pallet across aisle 1; only the robot sent past it can see it.
    base = {"disturbances": [{"at_s": 0.5, "blockage": {"rect": [9.0, 12.0, 10.0, 14.4]}}],
            "tasks": [{"at_s": 1, "pickup": "shelf_1_1", "dropoff": "dock_1"}],
            "config": {"map_share": {"recipients": "all"}}}
    on, _ = run(scenario(**copy.deepcopy(base)))
    off_cfg = copy.deepcopy(base)
    off_cfg["config"]["map_share"]["enabled"] = False
    off, _ = run(scenario(**off_cfg))
    md = [{"metric": "map_divergence"}]
    v_on, v_off = score(on, md)["map_divergence"], score(off, md)["map_divergence"]
    # Nobody sees the pallet's far face, so the truth is never fully known;
    # what sharing closes is the gap to the fleet's newest knowledge.
    assert v_off["final_stale"] > 0
    assert v_on["final_stale"] < v_off["final_stale"] / 10
    assert v_on["final_wrong_free"] < v_off["final_wrong_free"]


PARTITION = [{"at_s": 20, "duration_s": 60, "partition": [["robot_0", "robot_1"], "rest"]}]


def test_tasks_announced_during_a_split_are_never_worked_twice():
    # Announced after the split: the cut-off pair has no quorum, so it never
    # claims, and nothing it owns can be reclaimed under it.
    scn = scenario(robots=5, duration_s=500, disturbances=PARTITION,
                   tasks=[{"at_s": t, "pickup": p, "dropoff": d} for t, p, d in (
                       (25, "shelf_1_0", "dock_0"), (30, "shelf_2_1", "dock_1"),
                       (35, "shelf_3_2", "dock_2"), (40, "shelf_4_0", "dock_3"))])
    s, _ = run(scn)
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "nav"}])
    assert v["fleet_tasks"]["duplicates"] == 0
    assert v["nav"]["empty_shelf"] == 0
    assert v["fleet_tasks"]["done"] == v["fleet_tasks"]["announced"]


def _split_around_the_pick(**kw):
    """One task; cut its owner off 5 s before it reaches the shelf (inside
    its 10 s lease), for 60 s."""
    tasks = [{"at_s": 1, "pickup": "shelf_2_1", "dropoff": "dock_1"}]
    base, _ = run(scenario(robots=5, duration_s=500, tasks=tasks))
    owner, t_pick = next((e["rid"], e["t"]) for e in base.events
                         if e["s"] == "task" and e["state"] == "NAV_TO_DROPOFF")
    s, _ = run(scenario(robots=5, duration_s=500, tasks=tasks, **kw, disturbances=[
        {"at_s": max(1.0, t_pick - 5.0), "duration_s": 60, "partition": [[owner], "rest"]}]))
    return s, owner


def test_pickup_inside_the_lease_window_still_sends_a_second_robot():
    # KNOWN PROTOCOL GAP 1 (found 2026-11-02; commsim's DES reports the same
    # 0-2 duplicate episodes per partition run). An owner cut off from the
    # majority must stop within task_lease_s — unless it picks the item up
    # first: a committed task never yields. The majority never hears the
    # commit and reclaims at lease + margin, so a second robot drives to a
    # shelf the item has left. Flip when the protocol closes it (e.g. no
    # pick without a quorum).
    s, _owner = _split_around_the_pick()
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "nav"}])
    assert v["fleet_tasks"]["duplicates"] >= 1
    assert v["nav"]["empty_shelf"] >= 1
    assert v["fleet_tasks"]["done"] == v["fleet_tasks"]["announced"]


def test_a_completion_lost_in_a_split_is_never_repeated():
    # KNOWN PROTOCOL GAP 2 (found 2026-11-02). The cut-off owner delivers
    # while still split; its fleet-wide Complete is lost and nothing ever
    # repeats it. After the heal the majority still holds the task open,
    # and the robot that knows it is DONE ignores their claims for it
    # (protocol.py _assert_owner returns early for DONE tasks), so robot
    # after robot is sent to the empty shelf until each one's failed_backoff
    # runs. The DES can't see this: its robots never find a shelf empty.
    # Flip when e.g. a node that knows a task is DONE answers a claim or
    # lease beacon for it with a Complete.
    s, _owner = _split_around_the_pick()
    v = score(s, [{"metric": "nav"}])
    assert v["nav"]["empty_shelf"] > 1
    gw_state = s.gw.alloc.tasks.get(1)
    assert gw_state is None or gw_state.state != "done"


def test_radio_network_runs_and_reports_load():
    s, _ = run(scenario(config={"network": {"model": "radio"}}))
    v = score(s, [{"metric": "comms"}, {"metric": "fleet_tasks"}])
    assert v["comms"]["status_delivery"] > 0.5
    assert v["comms"]["load_max"] > 0.0
    assert v["fleet_tasks"]["done"] >= 2


def test_unknown_setting_is_rejected():
    with pytest.raises(config.ScenarioError, match="no such setting"):
        scenario(config={"nav": {"stop_distanse_m": 1.0}})


def test_every_shipped_scenario_validates():
    names = [p.stem for p in sorted(config.SCENARIOS_DIR.glob("*.yaml"))]
    assert names
    for n in names:
        config.load(n)
