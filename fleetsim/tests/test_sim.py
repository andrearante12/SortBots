"""fleetsim end to end, on a small floor so the whole file runs in seconds."""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fleetsim import __main__ as cli  # noqa: E402
from fleetsim import config, sim  # noqa: E402
from simtest import events as ev_lib  # noqa: E402
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


def _strip(evs):
    return [{k: v for k, v in e.items() if k not in ("wall", "wall_s", "summary")} for e in evs]


def test_a_seed_reproduces_a_run_exactly():
    a, _ = run(scenario())
    b, _ = run(scenario())
    assert _strip(a.events) == _strip(b.events)


def test_task_bounds_are_the_only_change_to_a_run(monkeypatch):
    # The bounds' A* reads no RNG and touches nothing the robots use, so a
    # run without it is the same run minus its task_bound events.
    a, _ = run(scenario())
    monkeypatch.setattr(sim.Sim, "_task_bound", lambda self, rec: None)
    b, _ = run(scenario())
    with_bounds = _strip(a.events)
    assert [e for e in with_bounds if e["s"] != "task_bound"] == _strip(b.events)
    after = [with_bounds[i - 1] for i, e in enumerate(with_bounds) if e["s"] == "task_bound"]
    assert [e["s"] for e in after] == ["announce"] * 3  # each right after its announce


def test_the_lower_bound_is_never_above_a_real_delivery():
    s, _ = run(scenario())
    ev = s.events
    lb = {e["task_id"]: e["lb_s"] for e in ev if e["s"] == "task_bound"}
    ann = {e["task_id"]: e["t"] for e in ev if e["s"] == "announce"}
    shelf, done = {}, {}
    for e in ev:
        if e["s"] == "task" and e["state"] in ("AWAIT_PICK", "PICKING"):
            shelf.setdefault(e["task_id"], e["t"])
        elif e["s"] == "task" and e["state"] == "DONE":
            done.setdefault(e["task_id"], e["t"])
    assert set(lb) == set(ann) == set(done)
    dwell = s.cfg["tasks"]["dwell_s"]
    for t in done:
        assert lb[t] > 2 * dwell
        assert done[t] - ann[t] >= lb[t]
        # Holds even leaving out the drive to the shelf: the bound is close
        # (seed 1 of delivery_smoke, walkers and fleet_scale: >= 1.016x, 2026-11-26).
        assert done[t] - shelf[t] >= lb[t]
    v = score(s, [{"metric": "bounds"}, {"metric": "utilisation"}, {"metric": "nav"}])
    assert v["bounds"]["stretch"] > 1.0 and v["bounds"]["offered_load"] > 0
    u = v["utilisation"]
    shares = [u[f"{k}_frac"] for k in ("idle", "to_pickup", "at_pickup", "carrying", "placing",
                                       "home", "off")]
    assert sum(shares) == pytest.approx(1.0, abs=1e-3)
    assert u["busy_frac"] > 0 and u["lost_robot_s"] <= v["nav"]["blocked_robot_s"]


def test_a_run_recorded_before_task_bound_still_rescores(tmp_path):
    # An old run: the same events without task_bound, scored by the CLI.
    scn = scenario(metrics=[{"metric": "fleet_tasks"}, {"metric": "bounds"},
                            {"metric": "utilisation"}])
    s, _ = run(scn)
    rec = ev_lib.Recorder(tmp_path / "events.jsonl.gz")
    for e in s.events:
        if e["s"] != "task_bound":
            rec.write(e)
    rec.close()
    (tmp_path / "scenario.json").write_text(json.dumps(scn, default=str))
    assert cli.main(["rescore", str(tmp_path)]) == 0
    v = {m["id"]: m["values"]
         for m in json.loads((tmp_path / "score.json").read_text())["metrics"]}
    assert v["bounds"]["lb_mean_s"] is None and v["bounds"]["offered_load"] is None
    assert v["bounds"]["arrival_per_hour"] is not None
    assert v["fleet_tasks"]["wait_assign_s"] is not None and v["fleet_tasks"]["backlog_end"] == 0
    assert v["utilisation"]["busy_frac"] > 0


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


ONE_TASK = [{"at_s": 1, "pickup": "shelf_2_1", "dropoff": "dock_1"}]
SPEC = {"fleet_tasks": {"protocol": {"confirmed_picks": False, "confirmed_completes": False}}}


def _times():
    """When the owner of ONE_TASK picks it up and delivers it, unsplit."""
    base, _ = run(scenario(robots=5, duration_s=500, tasks=ONE_TASK))
    ev = [e for e in base.events if e["s"] == "task"]
    owner, t_pick = next((e["rid"], e["t"]) for e in ev if e["state"] == "NAV_TO_DROPOFF")
    t_done = next(e["t"] for e in ev if e["state"] == "DONE")
    return owner, t_pick, t_done


def _split(at_s, duration_s, config=None):
    owner, *_ = _times()
    s, _ = run(scenario(robots=5, duration_s=500, tasks=ONE_TASK, config=config or {},
                        disturbances=[{"at_s": max(1.0, at_s), "duration_s": duration_s,
                                       "partition": [[owner], "rest"]}]))
    return s, owner


def test_no_pick_without_the_majority_knowing():
    # Owner cut off 5 s before the shelf, inside its 10 s lease. Fixed
    # 2026-11-17 (confirmed picks): it waits at the shelf for acks that can't
    # come, gives the task up at its lease, and the majority's robot is the
    # only one that ever picks.
    _, t_pick, _ = _times()
    s, owner = _split(t_pick - 5.0, 60)
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "nav"}])
    assert v["fleet_tasks"]["duplicates"] == 0
    assert v["nav"]["empty_shelf"] == 0
    assert v["fleet_tasks"]["done"] == v["fleet_tasks"]["announced"]
    assert [e for e in s.events if e["s"] == "task" and e["rid"] == owner
            and e["state"] == "AWAIT_PICK"]


def test_without_confirmed_picks_the_split_sends_a_second_robot():
    # The gap as found 2026-11-02 (commsim's DES reports the same 0-2
    # duplicate episodes per partition run): the owner picks while cut off, a
    # committed task never yields, the majority never heard the commit and
    # reclaims at lease + margin, and a second robot finds the shelf empty.
    _, t_pick, _ = _times()
    s, _ = _split(t_pick - 5.0, 60, SPEC)
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "nav"}])
    assert v["fleet_tasks"]["duplicates"] >= 1
    assert v["nav"]["empty_shelf"] >= 1


def test_a_completion_made_during_a_split_reaches_everyone():
    # Cut off after the pick, delivered while still cut off: the Complete is
    # lost. Fixed 2026-11-17 (confirmed completes): the owner repeats it until
    # acked, so after the heal the gateway's board says done and nobody is
    # ever sent to the empty shelf.
    _, t_pick, t_done = _times()
    s, owner = _split(t_pick + 1.0, t_done - t_pick + 20.0)
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "nav"}])
    assert s.gw.alloc.tasks[1].state == "done"
    assert v["nav"]["empty_shelf"] == 0 and v["fleet_tasks"]["duplicates"] == 0
    assert [e for e in s.events if e["s"] == "fleet_log" and e["rid"] == owner
            and "complete_confirmed" in e["text"]] or not s.cfg["record"]["fleet_log"]


def test_without_confirmed_completes_the_gateway_never_learns():
    # The same split with the spec's fire-and-forget Complete: the item was
    # committed before the split, so nobody reclaims it, but the gateway holds
    # it as owner_missing for ever.
    _, t_pick, t_done = _times()
    s, _ = _split(t_pick + 1.0, t_done - t_pick + 20.0, SPEC)
    gw_state = s.gw.alloc.tasks.get(1)
    assert gw_state is not None and gw_state.state != "done"


def test_path_intents_run_clean_on_both_transports():
    for transport in ("status", "message"):
        s, _ = run(scenario(robots=5, config={"intent": {"enabled": True, "transport": transport}}))
        v = score(s, [{"metric": "fleet_tasks"}, {"metric": "contacts"}, {"metric": "nav"}])
        assert v["fleet_tasks"]["done"] == 3 and v["contacts"]["robot"] == 0, transport
        assert v["nav"]["yield_timeouts"] == 0


def test_radio_network_runs_and_reports_load():
    s, _ = run(scenario(config={"network": {"model": "radio"}}))
    v = score(s, [{"metric": "comms"}, {"metric": "fleet_tasks"}])
    assert v["comms"]["status_delivery"] > 0.5
    assert v["comms"]["load_max"] > 0.0
    assert v["fleet_tasks"]["done"] >= 2


def test_two_sets_into_a_free_form_block_both_apply():
    scn = config.resolve({"name": "t"}, sets=[("fleet_tasks.protocol.confirmed_picks", False),
                                              ("fleet_tasks.protocol.confirmed_completes", False)])
    assert scn["config"]["fleet_tasks"] == {"protocol": {"confirmed_picks": False,
                                                         "confirmed_completes": False}}


def test_unknown_setting_is_rejected():
    with pytest.raises(config.ScenarioError, match="no such setting"):
        scenario(config={"nav": {"stop_distanse_m": 1.0}})


def test_every_shipped_scenario_validates():
    names = [p.stem for p in sorted(config.SCENARIOS_DIR.glob("*.yaml"))]
    assert names
    for n in names:
        config.load(n)
