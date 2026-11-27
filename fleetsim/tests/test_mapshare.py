"""Map-tile subscriptions (map_share.recipients: subscribers), event-driven map
publishes (map_share.events) and the map_learn recording. Small floor."""
from __future__ import annotations

import copy
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from fleetsim import belief as B  # noqa: E402
from fleetsim import config, mapshare, maplearn, sim  # noqa: E402
from fleetsim.planner import OCC  # noqa: E402
from test_sim import _strip, run, scenario, score  # noqa: E402

SUBS = {"map_share": {"recipients": "subscribers"}}
# A pallet across aisle 1 of the small floor, between shelf_1_0 and shelf_1_1:
# only through-traffic along the aisle runs into it.
PALLET = [{"at_s": 0.5, "blockage": {"rect": [9.0, 12.0, 10.0, 14.4]}}]
# The first robot sees the pallet on its way to shelf_1_1 (t ~28 s); the
# second task's drop-off is on the pallet's far side, so its path runs along
# aisle 1 into it.
TWO_TASKS = [{"at_s": 1, "pickup": "shelf_1_1", "dropoff": "dock_1"},
             {"at_s": 60, "pickup": "shelf_1_3", "dropoff": "shelf_1_0"}]


def built(**over):
    """A Sim, not run: for poking at the bookkeeping directly."""
    return sim.Sim(scenario(**over))


def learned(s, **match):
    return [e for e in s.events if e["s"] == "map_learn"
            and all(e.get(k) == v for k, v in match.items())]


# ---- what a robot subscribes to ------------------------------------------------

def test_a_robot_subscribes_to_the_tiles_round_it_and_along_its_path():
    s = built(config=sim.deep_merge(SUBS, {"map_share": {"sub_radius_m": 2.0,
                                                          "sub_path_m": 10.0}}))
    subs, r = s.subs, s.robots[0]
    b = r.belief
    tile = lambda x, y: int(b.tile_of(np.array([s.grid.index(x, y)]))[0])  # noqa: E731
    r.x, r.y, r.goal, r.path = 3.0, 3.0, None, []
    assert subs.wanted(r) == {tile(3.0, 3.0)}  # 2 m from (3, 3) stays in one 6.4 m tile
    r.x, r.y = 6.0, 6.0  # 0.4 m from three tile edges: all four tiles meet the disc
    assert subs.wanted(r) == {tile(x, y) for x in (3, 9) for y in (3, 9)}
    # A path east along y = 3: the next 10 m of it, not the 25 m beyond.
    r.x, r.y, r.seg = 3.0, 3.0, 0
    r.goal, r.path = (28.0, 3.0), [(3.0, 3.0), (28.0, 3.0)]
    got = subs.wanted(r)
    assert {tile(3.0, 3.0), tile(9.0, 3.0), tile(13.0, 3.0)} <= got
    assert tile(20.0, 3.0) not in got and tile(27.0, 3.0) not in got
    s.cfg["map_share"]["sub_path_m"] = None
    assert tile(27.0, 3.0) in mapshare.Subscriptions(s).wanted(r)  # null: the whole path


def test_announcements_keep_each_robots_view_of_who_subscribes():
    s = built(config=SUBS)
    subs = s.subs
    a, b = s.robots[0], s.robots[1]
    subs.deliver_announce(b.node, (a.node, 2, frozenset({4, 5})))
    assert subs.dsts(b, 4).tolist() == [a.node] and subs.dsts(b, 6).tolist() == []
    subs.deliver_announce(b.node, (a.node, 1, frozenset({6})))  # older: ignored
    assert subs.dsts(b, 4).tolist() == [a.node] and subs.dsts(b, 6).tolist() == []
    s.now = 10.0
    subs.deliver_announce(b.node, (a.node, 3, frozenset({5})))  # dropped 4
    m = subs.st[b.k]
    assert subs.dsts(b, 4).tolist() == [] and m.last_left[4] == (10.0, a.node)
    # A patch that carries a flip (the world moved) goes to every robot.
    assert subs.dsts(b, 9, flip=True).tolist() == [r.node for r in s.robots if r is not b]
    # A peer not heard for 2.5 keepalives is dropped: a dead robot stops costing patches.
    subs._expire(m, 10.0 + 2.5 * subs.keep + 1)
    assert subs.dsts(b, 5).tolist() == [] and a.node not in m.view


def test_a_new_tile_is_pulled_from_one_peer_nearest_first():
    s = built(config=SUBS)
    subs = s.subs
    me, near, far = s.robots
    m = subs.st[me.k]
    s.now = 100.0
    me.x, me.y = 0.0, 0.0
    for r, xy in ((near, (3.0, 0.0)), (far, (20.0, 0.0))):
        s.seen_t[me.k, r.k] = 99.0
        s.seen_xy[me.k, r.k] = xy
    m.subs_of[7] = {far.node: 10.0, near.node: 10.0}
    assert subs._source(me, m, 7, 100.0) == near.node
    # One that only just subscribed may not have had its own answer yet.
    m.subs_of[7] = {far.node: 10.0, near.node: 99.0}
    assert subs._source(me, m, 7, 100.0) == far.node
    # Nobody subscribes: whoever was heard leaving it last; else nobody.
    m.subs_of[7] = {}
    m.last_left[7] = (50.0, far.node)
    assert subs._source(me, m, 7, 100.0) == far.node
    assert subs._source(me, m, 8, 100.0) is None


def test_a_pull_answer_with_a_shared_prior_carries_only_what_differs_from_it():
    s = built(config=SUBS)  # test_sim's scenario starts every robot on the true map
    b = s.robots[0].belief
    tile = 7
    cells = b.tile_cells(tile)
    occ = b.s[cells] == OCC
    none = np.zeros(0, dtype=np.int64)
    b.observe(cells[~occ], cells[occ], 5.0)  # sees the whole tile as it was
    assert len(b.answer_payload(tile, -math.inf)[0]) == 0  # nothing new: nothing to send
    free = cells[~occ][:3]
    b.observe(none, free, 6.0)   # a pallet
    idx, st, ts = b.answer_payload(tile, -math.inf)
    assert sorted(idx.tolist()) == sorted(free.tolist()) and (st == OCC).all()
    b.observe(free, none, 7.0)   # gone again: still sent, as free
    idx, st, ts = b.answer_payload(tile, -math.inf)
    assert sorted(idx.tolist()) == sorted(free.tolist()) and (st != OCC).all() and (ts == 7.0).all()


def test_an_answer_hands_over_sequence_numbers_so_the_next_delta_is_no_gap():
    a, b = B.Belief(built().grid, 6.4), B.Belief(built().grid, 6.4)
    a.in_seq[(5, 3)] = 7
    a.out_seq[3] = 2
    b.adopt_seq(3, a.seq_vector(3, me=1), me=2)
    assert b.in_seq[(5, 3)] == 7 and b.in_seq[(1, 3)] == 2
    assert not b.check_seq(5, 3, 8, snapshot=False)  # the next delta from origin 5
    assert b.check_seq(5, 3, 10, snapshot=False)     # a real gap still asks


def test_news_of_a_tile_it_does_not_subscribe_to_records_no_sequence_number():
    # Recorded, it would hide the gap the robot is owed a resync for once it
    # subscribes (review, 2026-11-26).
    s = built(config=SUBS)
    a, b = s.robots[0], s.robots[1]
    tile, none = 7, (np.zeros(0, dtype=np.int64),) * 3
    s._deliver_map(b.node, (a.node, tile, 5, False, none))
    assert (a.node, tile) not in b.belief.in_seq and not b.resync_wait
    s.subs.st[b.k].tiles = frozenset({tile})
    s._deliver_map(b.node, (a.node, tile, 6, False, none))
    assert (a.node, tile) in b.resync_wait  # subscribed now: the gap asks a MapResync
    # ...but not while a pull of that tile is open.
    s.subs.st[b.k].pulling[tile + 1] = s.now + 5.0
    s.subs.st[b.k].tiles = frozenset({tile, tile + 1})
    s._deliver_map(b.node, (a.node, tile + 1, 9, False, none))
    assert (a.node, tile + 1) not in b.resync_wait


# ---- end to end ------------------------------------------------------------------

def _drive_in(**over):
    cfg = sim.deep_merge(SUBS, {"map_share": {"sub_flips_to": "subscribers",
                                              "sub_radius_m": 2.0}})
    return run(scenario(disturbances=PALLET, tasks=TWO_TASKS,
                        config=sim.deep_merge(cfg, over)))


def test_a_robot_whose_path_enters_a_tile_gets_its_state_within_a_bounded_time():
    # News stays with subscribers here (sub_flips_to: subscribers), so the
    # second robot can only learn of the pallet by subscribing to its tile.
    s, summary = _drive_in()
    assert summary["reason"] == "tasks_done"
    first = learned(s, how="sensed")[0]
    second = [e for e in s.events if e["s"] == "dispatch" and e["task_id"] == "fleet-2"][0]
    assert second["rid"] != first["rid"]
    got = learned(s, rid=second["rid"])[0]
    # From a peer, and within one announcement interval (plus the network) of
    # being given the task whose path runs there.
    bound = s.cfg["map_share"]["sub_min_interval_s"] + 0.5
    assert got["how"] == "shared" and got["t"] - second["t"] <= bound
    assert s.subs.counts["answers"] > 0


def test_without_the_pull_that_robot_waits_for_a_resync_or_a_refresh(monkeypatch):
    # The old machinery still gets there (the origin's next delta shows a gap,
    # or a refresh comes round), only later: 7 s here, against 1.4 s.
    s0, _ = _drive_in()
    rid = [e for e in s0.events if e["s"] == "dispatch" and e["task_id"] == "fleet-2"][0]["rid"]
    monkeypatch.setattr(mapshare.Subscriptions, "_pull", lambda *a: None)
    s, _ = _drive_in()
    t0 = learned(s0, rid=rid)[0]["t"]
    got = learned(s, rid=rid)[0]
    assert got["after_s"] is None or got["t"] > t0 + 3


def test_subscriptions_deliver_like_fleet_wide_sharing_with_less_map_traffic():
    base = {"disturbances": PALLET, "tasks": TWO_TASKS}
    a, sa = run(scenario(**copy.deepcopy(base), config={"map_share": {"recipients": "all"}}))
    b, sb = run(scenario(**copy.deepcopy(base), config=SUBS))
    m = [{"metric": "fleet_tasks"}, {"metric": "comms"}, {"metric": "map_spread"}]
    va, vb = score(a, m), score(b, m)
    assert sb["reason"] == "tasks_done" and vb["fleet_tasks"]["done"] == 2
    # Flips go fleet-wide, so every robot heard of the pallet as with `all`.
    assert vb["map_spread"]["all_never"] == va["map_spread"]["all_never"] == 0
    assert vb["comms"]["map_bytes_per_robot_s"] < va["comms"]["map_bytes_per_robot_s"]
    assert vb["comms"]["sub_bytes_per_robot_s"] > 0 == va["comms"]["sub_bytes_per_robot_s"]


def test_subscriptions_survive_a_partition_and_a_reboot():
    dist = [{"at_s": 20, "duration_s": 40, "partition": [["robot_0"], "rest"]},
            {"at_s": 30, "duration_s": 20, "kill": "robot_1", "fresh": True}]
    s, summary = run(scenario(config=SUBS, disturbances=dist))
    assert summary["reason"] == "tasks_done"
    # The rebooted robot announced afresh: every robot's view has it again.
    assert all(s.robots[1].node in s.subs.st[r.k].view for r in s.robots if r.k != 1)


# ---- event-driven publishes ------------------------------------------------------

def _pallet_run(tasks=None, **cfg):
    base = {"disturbances": PALLET,
            "tasks": tasks or [{"at_s": 1, "pickup": "shelf_1_1", "dropoff": "dock_1"}],
            "config": sim.deep_merge({"map_share": {"recipients": "all"}}, cfg)}
    return run(scenario(**base))


def test_an_event_publish_goes_out_when_the_camera_sees_the_blockage():
    s, _ = _pallet_run(map_share={"events": True})
    seen = learned(s, how="sensed")[0]
    pubs = [e for e in s.events if e["s"] == "map_pub"]
    assert pubs and pubs[0]["t"] == seen["t"] and pubs[0]["rid"] == seen["rid"]
    others = learned(s, how="shared")
    assert len(others) == len(s.robots) - 1
    delay = s.cfg["network"]["ideal"]["delay_s"]
    assert all(e["t"] - seen["t"] < 2 * delay + 1e-3 for e in others)
    # The same run without: the others wait for the discoverer's next tick.
    s0, _ = _pallet_run()
    t0 = learned(s0, how="sensed")[0]["t"]
    assert all(e["t"] >= t0 for e in learned(s0, how="shared"))
    assert not [e for e in s0.events if e["s"] == "map_pub"]


def test_event_publishes_keep_their_gap_and_the_rate_cap():
    s, _ = _pallet_run(map_share={"events": True, "event_gap_s": 3.0})
    by: dict = {}
    for e in s.events:
        if e["s"] == "map_pub":
            by.setdefault(e["rid"], []).append(e["t"])
    assert by and all(b - a >= 3.0 - 1e-9 for ts in by.values() for a, b in zip(ts, ts[1:]))
    # No budget, no publish: an event draws on the same tokens as the tick.
    s, _ = _pallet_run(map_share={"events": True, "rate_cap_bps": 0})
    assert not [e for e in s.events if e["s"] == "map_pub"]
    assert all(e["kinds"].get("map", {"sent": 0})["sent"] == 0
               for e in s.events if e["s"] == "net")


def test_with_intents_an_event_reannounces_the_new_path_on_an_extra_status(monkeypatch):
    calls = []
    orig = sim.Sim._status_tick
    monkeypatch.setattr(sim.Sim, "_status_tick",
                        lambda self, r: (calls.append((self.now, r.rid, r.next_status)),
                                         orig(self, r)))
    # Along aisle 1 from its far end: the robot's own path runs into the pallet.
    s, _ = _pallet_run(tasks=[{"at_s": 1, "pickup": "shelf_1_3", "dropoff": "shelf_1_0"}],
                       map_share={"events": True}, intent={"enabled": True})
    pub = [e for e in s.events if e["s"] == "map_pub"][0]
    replans = [e for e in s.events if e["s"] == "nav" and e["event"] == "replan"
               and e["rid"] == pub["rid"] and e["t"] == pub["t"]]
    assert replans  # its own path ran into what it just saw
    # A Status at the event's time, off its own schedule (next_status is later).
    extra = [c for c in calls if c[0] == pub["t"] and c[1] == pub["rid"] and c[2] > c[0]]
    assert extra
    assert s.robots[int(pub["rid"].split("_")[1])].intent_msg["built"] >= pub["t"]


# ---- defaults and the recording -----------------------------------------------------

def test_defaults_never_enter_the_new_code(monkeypatch):
    def boom(*a, **k):
        raise AssertionError("a default run used B8/B9 code")
    monkeypatch.setattr(mapshare.Subscriptions, "__init__", boom)
    monkeypatch.setattr(sim.Sim, "_map_event", boom)
    monkeypatch.setattr(B.Belief, "track_touched", boom)
    monkeypatch.setattr(B.Belief, "take_dirty_tiles", boom)
    s, summary = run(scenario(disturbances=PALLET))
    assert summary["reason"] == "tasks_done" and s.subs is None and not s.track_flips
    assert all(not r.flip_tiles for r in s.robots)


def test_map_learn_only_adds_its_events(monkeypatch):
    # It reads maps and paths, draws no random number and sends nothing, so a
    # run without it is the same run minus its events.
    scn = scenario(disturbances=PALLET + [{"at_s": 90, "blockage": {"rect": [20, 12, 21, 14.4]},
                                           "duration_s": 60}])
    a, _ = run(copy.deepcopy(scn))
    monkeypatch.setattr(maplearn, "LearnTracker", lambda s: None)
    b, _ = run(copy.deepcopy(scn))
    with_rec = _strip(a.events)
    assert [e for e in with_rec if e["s"] != "map_learn"] == _strip(b.events)
    rows = [e for e in with_rec if e["s"] == "map_learn"]
    assert {(e["blockage"], e["on"]) for e in rows} >= {(0, True), (1, True)}
    assert len([e for e in rows if e["on"]]) == 2 * len(a.robots)


def test_map_spread_times_from_when_the_path_ran_into_it():
    from simtest import metrics as metrics_lib
    ctx = metrics_lib.Context(robots=["robot_0", "robot_1", "robot_2"])
    ms = metrics_lib.MetricSet(metrics_lib.validate_list([{"metric": "map_spread"}]), ctx)
    rows = [  # robot_0 found it (its path ran in at 4 s, its camera at 20 s);
        # robot_1 planned through it at 100 s and got it 2 s later from a peer;
        # robot_2 never needed it; at the clear, robot_1 dropped it after 5 s.
        dict(rid="robot_0", blockage=0, on=True, after_s=20.0, how="sensed", path_s=4.0),
        dict(rid="robot_1", blockage=0, on=True, after_s=102.0, how="shared", path_s=100.0),
        dict(rid="robot_2", blockage=0, on=True, after_s=None, how=None, path_s=None),
        dict(rid="robot_1", blockage=0, on=False, after_s=5.0, how="sensed", ghost_s=None),
        dict(rid="robot_0", blockage=0, on=False, after_s=None, how=None, ghost_s=None),
        # robot_2 took the gone pallet in from a stale peer 30 s on, and held it 10 s.
        dict(rid="robot_2", blockage=0, on=False, after_s=40.0, how="sensed", ghost_s=30.0),
    ]
    for i, r in enumerate(rows):
        ms.feed({"s": "map_learn", "t": float(i), **r})
    v = ms.score()["metrics"][0]["values"]
    assert v["on_path"] == 2 and v["late_mean_s"] == pytest.approx(9.0)  # (16 + 2) / 2
    assert v["spread_mean_s"] == pytest.approx(2.0) and v["missed"] == 0
    assert v["from_peer_frac"] == 0.5 and v["never"] == 0 and v["all_never"] == 1
    assert v["cleared"] == 2 and v["clear_mean_s"] == 5.0 and v["clear_never"] == 1
    assert v["ghosts"] == 1 and v["ghost_held_s"] == 10.0 and v["ghost_never"] == 0


@pytest.mark.parametrize("key,val", [("recipients", "near"), ("sub_flips_to", "some"),
                                     ("events", "yes"), ("sub_keepalive_s", 0),
                                     ("sub_path_m", -1), ("sub_transport", "radio")])
def test_bad_map_share_settings_fail_validation(key, val):
    with pytest.raises(config.ScenarioError, match=f"map_share.{key}"):
        scenario(config={"map_share": {key: val}})


def test_on_status_a_subscription_rides_the_next_two_statuses_and_no_message_of_its_own():
    s, _ = run(scenario(config=SUBS, disturbances=PALLET))
    assert s.cfg["map_share"]["sub_transport"] == "status"
    kinds = set().union(*(e["kinds"] for e in s.events if e["s"] == "net"))
    assert "mapsub" not in kinds and "mappull" in kinds
    # Every robot's view of every other robot came off a Status.
    assert all(set(s.subs.st[r.k].view) == {o.node for o in s.robots if o is not r}
               for r in s.robots)
    m = s.subs.st[0]
    m.carry = 2
    assert s.subs.for_status(s.robots[0]) == (s.robots[0].node, m.ver, m.tiles)
    assert s.subs.for_status(s.robots[0]) is not None and s.subs.for_status(s.robots[0]) is None


def test_as_a_message_a_subscription_goes_to_every_robot():
    s, summary = run(scenario(config=sim.deep_merge(SUBS, {"map_share": {
        "sub_transport": "message"}}), disturbances=PALLET))
    assert summary["reason"] == "tasks_done"
    sent = sum(e["kinds"].get("mapsub", {"sent": 0})["sent"] for e in s.events if e["s"] == "net")
    assert sent == s.subs.counts["announces"] * (len(s.robots) - 1)
