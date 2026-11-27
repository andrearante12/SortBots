"""fleetsim/dispatch: the swappable task-assignment layer, and perfect
information (fleetsim/info.py). Small floor, seconds per test."""
from __future__ import annotations

import hashlib
import itertools
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from fleetsim import sim  # noqa: E402
from fleetsim.dispatch import hungarian  # noqa: E402
from fleetsim.planner import OCC  # noqa: E402
from test_sim import run, scenario, score  # noqa: E402

import fleet_tasks as ft  # noqa: E402  (sim.py put nodes/ on the path)

CENTRAL = ("central_greedy", "central_hungarian")


def central(method, **over):
    cfg = over.pop("config", {})
    return scenario(config=sim.deep_merge({"dispatch": {"method": method}}, cfg), **over)


# ---- Hungarian ----------------------------------------------------------------

def _brute(c):
    n, m = c.shape
    if n > m:
        return _brute(c.T)
    return min(sum(c[i, p[i]] for i in range(n)) for p in itertools.permutations(range(m), n))


def test_hungarian_matches_brute_force_on_random_matrices():
    rng = np.random.default_rng(7)
    for k in range(1500):
        n, m = (int(x) for x in rng.integers(1, 7, 2))
        # Integers make ties (several optima) common; floats test the arithmetic.
        c = (rng.integers(-5, 20, (n, m)).astype(float) if k % 2
             else rng.random((n, m)) * 100.0)
        cols = hungarian.solve(c)
        assert len(cols) == n
        used = [j for j in cols if j >= 0]
        assert len(used) == len(set(used)) == min(n, m), (c, cols)
        assert hungarian.total(c, cols) == pytest.approx(_brute(c), abs=1e-9), (c, cols)


def test_hungarian_edge_shapes():
    assert hungarian.solve(np.zeros((0, 3))) == []
    assert hungarian.solve(np.zeros((2, 0))) == [-1, -1]
    assert hungarian.solve([[5.0]]) == [0]
    with pytest.raises(ValueError):
        hungarian.solve([[1.0, np.inf]])


# ---- central methods, end to end ------------------------------------------------

@pytest.mark.parametrize("method", CENTRAL)
def test_central_method_delivers_every_task(method):
    s, summary = run(central(method))
    assert summary["reason"] == "tasks_done"
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "contacts"}, {"metric": "nav"}])
    assert v["fleet_tasks"]["done"] == 3 and v["fleet_tasks"]["duplicates"] == 0
    assert v["nav"]["empty_shelf"] == 0
    # Every pick waited for the dispatcher's word (confirmed picks).
    waits = [e for e in s.events if e["s"] == "task" and e["state"] == "AWAIT_PICK"]
    assert len(waits) == 3 and summary["dispatch"]["picks"] == 3
    assert summary["dispatch"]["method"] == method


def test_central_dispatch_travels_the_radio():
    s, _ = run(central("central_hungarian", config={"network": {"model": "radio"}}))
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "comms"}])
    assert v["fleet_tasks"]["done"] == 3
    assert v["comms"]["tasks_delivery"] > 0.9  # Dispatch/Report frames were sent and paid for


def test_a_seed_reproduces_a_central_run_exactly():
    strip = lambda evs: [{k: v for k, v in e.items() if k not in ("wall", "wall_s", "summary")}
                         for e in evs]
    a, _ = run(central("central_hungarian", robots=4))
    b, _ = run(central("central_hungarian", robots=4))
    assert strip(a.events) == strip(b.events)


GW_CUT = [{"at_s": 5, "duration_s": 60, "partition": [["gateway"], "rest"]}]


@pytest.mark.parametrize("method", CENTRAL)
def test_central_dispatch_stops_while_the_gateway_is_cut_off_and_resumes_after(method):
    # Task 1 is dispatched before the cut; tasks 2 and 3 are announced during
    # it. The robots keep working what they have, nobody is told anything
    # new until the heal, and afterwards every task is delivered once.
    tasks = [{"at_s": 1, "pickup": "shelf_1_0", "dropoff": "dock_0"},
             {"at_s": 20, "pickup": "shelf_2_2", "dropoff": "dock_2"},
             {"at_s": 30, "pickup": "shelf_3_1", "dropoff": "dock_3"}]
    s, summary = run(central(method, tasks=tasks, disturbances=GW_CUT, duration_s=600))
    disp = [(e["t"], e["task_id"]) for e in s.events if e["s"] == "dispatch"]
    assert [tid for t, tid in disp if t < 5] == ["fleet-1"]
    assert not [d for d in disp if 5 <= d[0] < 65]
    assert {tid for t, tid in disp if t >= 65} >= {"fleet-2", "fleet-3"}
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "nav"}])
    assert v["fleet_tasks"]["done"] == 3 and v["nav"]["empty_shelf"] == 0
    assert summary["dispatch"]["lost"] == 3  # every robot, once
    assert all(s.gw.tasks[t].state == "done" for t in (1, 2, 3))


@pytest.mark.parametrize("fresh", [False, True])
def test_central_dispatch_survives_a_robot_dying_and_coming_back(fresh):
    dist = [{"at_s": 3, "duration_s": 40, "kill": "robot_0", "fresh": fresh}]
    s, _ = run(central("central_hungarian", disturbances=dist, duration_s=600))
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "nav"}])
    assert v["fleet_tasks"]["done"] == 3 and v["nav"]["empty_shelf"] == 0


ONE = [{"at_s": 1, "pickup": "shelf_2_1", "dropoff": "dock_1"}]
NOPICK = {"fleet_tasks": {"protocol": {"confirmed_picks": False}}}
PERFECT = {"info": {"dispatch": "truth"}}


def _one(**cfg):
    """The ONE task's run: who did it, when it reached the shelf, when it
    left it with the item, when it was delivered."""
    s, _ = run(central("central_hungarian", tasks=ONE, duration_s=400, config=cfg))
    ev = [e for e in s.events if e["s"] == "task"]
    owner = ev[0]["rid"]
    at_shelf = next(e["t"] for e in ev if e["state"] in ("AWAIT_PICK", "PICKING"))
    picked = next(e["t"] for e in ev if e["state"] == "NAV_TO_DROPOFF")
    done = next(e["t"] for e in ev if e["state"] == "DONE")
    return owner, at_shelf, picked, done


@pytest.mark.parametrize("cfg", [{}, NOPICK, PERFECT], ids=["grants", "no_grants", "perfect"])
def test_a_delivery_made_while_the_gateway_is_down_is_never_redone(cfg):
    # The gateway dies just before the robot reaches the shelf and comes back
    # long after the delivery would have been made. Found 2026-11-26
    # (bench_dead_gateway): back up, the dispatcher took every robot for
    # lost, re-planned the task it still thought unpicked, asked the robot
    # only about the tasks it still counted as its, and sent a second robot
    # to the empty shelf. Now it knows its own outage, asks about every task
    # it gave, and the robot repeats the outcome until it is acked; with
    # perfect information it reads where the item is.
    owner, at_shelf, _picked, done = _one(**cfg)
    down = [{"at_s": at_shelf - 3.0, "duration_s": done - at_shelf + 60.0, "kill": "gateway"}]
    s, summary = run(central("central_hungarian", tasks=ONE, disturbances=down,
                             duration_s=600, config=cfg))
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "nav"}])
    assert v["fleet_tasks"]["done"] == 1 and v["nav"]["empty_shelf"] == 0
    assert s.gw.tasks[1].state == "done" and summary["dispatch"]["restarts"] == 1
    back = at_shelf - 3.0 + done - at_shelf + 60.0
    ev = [e for e in s.events if e["s"] == "task"]
    t_done = next(e["t"] for e in ev if e["state"] == "DONE")
    if cfg is NOPICK or cfg is PERFECT:
        assert t_done < back  # it kept working through the outage...
        assert [e["rid"] for e in s.events if e["s"] == "dispatch"] == [owner]  # ...and once
    else:
        # It asked at the shelf, nobody answered, and it gave the task back at
        # the lease rather than stand on the spot; after the outage the task
        # went out again, and was picked only on the dispatcher's word.
        waits = [e for e in ev if e["rid"] == owner and e["state"] in ("AWAIT_PICK", "CANCELLED")]
        assert [e["state"] for e in waits][:2] == ["AWAIT_PICK", "CANCELLED"]
        assert waits[1]["t"] - waits[0]["t"] == pytest.approx(s.params.task_lease_s, abs=0.2)
        assert t_done > back


@pytest.mark.parametrize("cfg", [{}, NOPICK], ids=["grants", "no_grants"])
def test_a_robot_cut_off_before_the_shelf_never_shares_it(cfg):
    # The owner is cut off 4 s before it reaches the shelf; the dispatcher
    # loses it and gives the task to another robot. Found 2026-11-26
    # (bench_partition): without pick grants the cut-off robot picks anyway
    # and the other one finds the shelf empty. With them it waits there,
    # and after the heal it is cancelled.
    owner, at_shelf, _picked, _done = _one()
    cut = [{"at_s": at_shelf - 4.0, "duration_s": 80.0, "partition": [[owner], "rest"]}]
    s, _ = run(central("central_hungarian", tasks=ONE, disturbances=cut, duration_s=500,
                       config=cfg))
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "nav"}])
    assert v["fleet_tasks"]["done"] == 1
    if cfg is NOPICK:
        assert v["nav"]["empty_shelf"] >= 1  # the gap, kept as a control
    else:
        assert v["nav"]["empty_shelf"] == 0
        assert len({e["rid"] for e in s.events if e["s"] == "dispatch"}) == 2


def test_an_outcome_is_repeated_until_the_dispatcher_acks_it():
    s = sim.Sim(central("central_hungarian", tasks=[]))
    r = s.robots[0]
    client = r.core
    sent = []
    s.handle_actions = lambda owner, acts: sent.extend(
        ft.decode(v)[1] for k, v in acts if k == "publish")
    client.on_task_status(json.dumps({"task_id": "fleet-7", "state": "DONE", "queue": []}), 0.0)
    client._flush()
    assert sent[-1]["finals"] == [[7, "DONE", "queued", 0.0]]
    for k in range(1, 40):  # 4 s of steps: repeated after 1 s, then 2 s more (backing off)
        s.now = k * 0.1
        client.tick(s.now)
        if client._dirty:
            client._flush()
    assert len(sent) == 3 and all(m["finals"] for m in sent)
    client.on_message(0, {"type": "Ack", "finals": [7]}, s.now)
    for k in range(40, 200):
        s.now = k * 0.1
        client.tick(s.now)
    assert not client._dirty and not client.unacked
    # ...but it still answers a Query about it.
    client.on_message(0, {"type": "Query", "tasks": [7, 8], "seq": 1}, s.now)
    client._flush()
    assert sent[-1]["outcomes"] == {"7": ["DONE", "queued", 0.0]} and sent[-1]["unknown"] == [8]


def test_a_central_planner_never_sees_a_blockage_the_robots_drive_around():
    # A pallet across aisle 1 between the cross-aisles: the robot sent past
    # it replans around (the planner's true map doesn't have it, and doesn't
    # need it); the task is still delivered.
    s, _ = run(central("central_greedy", robots=2,
                       tasks=[{"at_s": 1, "pickup": "shelf_1_1", "dropoff": "dock_1"}],
                       disturbances=[{"at_s": 0.5, "blockage": {"rect": [9.0, 12.0, 10.0, 14.4]}}]))
    assert score(s, [{"metric": "fleet_tasks"}])["fleet_tasks"]["done"] == 1
    assert s.gw.costs.free.reshape(s.static.shape)[s.grid.cell(9.5, 13.2)[::-1]]


# ---- rolling Hungarian: reassignment and hysteresis ---------------------------------

def _hungarian(gain):
    s = sim.Sim(scenario(robots=2, tasks=[],
                         config={"dispatch": {"method": "central_hungarian",
                                              "reassign_gain_s": gain}}))
    return s, s.gw


def _msgs(gw, acks=False):
    """What the dispatcher sent; Acks of outcomes only when asked for."""
    out = []
    for kind, raw in gw.drain():
        if kind == "publish":
            m = ft.decode(raw)[1]
            if acks or m["type"] != "Ack":
                out.append(m)
    return out


def _report(gw, k, now, seq, cur=None, state="IDLE", queue=(), finals=()):
    gw.on_message(k + 1, {"type": "Report", "seq": seq, "cur": cur, "state": state,
                          "queue": list(queue), "finals": [list(f) for f in finals]}, now)


def _setup(gain):
    """robot_0 idle at home; robot_1 far away, carrying U to dock_3. T
    (shelf_0_3, beside dock_3) goes to robot_0. Returns what the gain
    would have to beat for robot_1, once it has delivered U at dock_3, to
    take T over."""
    s, gw = _hungarian(gain)
    home = (s.robots[0].x, s.robots[0].y)
    far = s.waypoint_xy("shelf_3_0")
    gw.on_status(0, 0.0, None, "idle", home)
    gw.on_status(1, 0.0, None, "task", far)
    u = gw.request("shelf_2_0", "dock_3", 0.0)
    _report(gw, 1, 0.0, 0, cur=u, state="NAV_TO_DROPOFF")  # the item is on robot_1
    t = gw.request("shelf_0_3", "dock_3", 0.0)
    gw.tick(0.1)
    sent = _msgs(gw)
    assert gw.tasks[u].robot == 1 and gw.tasks[u].state == "picked"
    assert gw.tasks[t].robot == 0, "the busy robot_1 is the worse choice at first"
    assert [(m["type"], m["to"], m["task_id"]) for m in sent] == [("Dispatch", 1, t)]
    _report(gw, 0, 0.2, sent[0]["seq"], cur=t, state="NAV_TO_PICKUP")
    c = gw.costs
    t0 = float(c.seconds("shelf_0_3", [c.snap(home)])[0])
    t1 = c.between("dock_3", "shelf_0_3")
    return s, gw, u, t, home, t0 - t1


def _robot_1_delivers(gw, u, home, now):
    gw.on_status(0, now, None, "task", home)
    gw.on_status(1, now, None, "idle", gw.sim.waypoint_xy("dock_3"))
    _report(gw, 1, now, 0, finals=[(u, "DONE", "PLACING")])
    gw.tick(now)
    return _msgs(gw)


def test_hungarian_reassigns_when_a_much_better_robot_frees_up():
    s, gw, u, t, home, saving = _setup(10.0)
    assert saving > 20.0  # the geometry is what the test assumes
    sent = _robot_1_delivers(gw, u, home, 2.0)
    assert gw.tasks[u].state == "done"
    assert gw.tasks[t].robot == 1
    assert [(m["type"], m["to"], m["task_id"]) for m in sent] == [("Cancel", 1, t),
                                                                  ("Dispatch", 2, t)]
    assert gw.stats["reassigned"] == 1


def test_hungarian_keeps_a_task_within_the_hysteresis():
    _s, _gw, _u, _t, _home, saving = _setup(10.0)
    s, gw, u, t, home, _ = _setup(saving + 5.0)
    sent = _robot_1_delivers(gw, u, home, 2.0)
    assert gw.tasks[t].robot == 0 and not sent
    assert gw.stats["reassigned"] == 0


def test_a_cancel_never_takes_an_item_off_a_robot():
    # The Cancel crossed the pick: robot_0 already has the item. It keeps the
    # task, and the dispatcher takes it back from whoever it went to.
    s, gw, u, t, home, _ = _setup(10.0)
    _robot_1_delivers(gw, u, home, 2.0)
    assert gw.tasks[t].robot == 1
    _report(gw, 0, 2.1, 1, cur=t, state="NAV_TO_DROPOFF")
    sent = _msgs(gw)
    assert gw.tasks[t].robot == 0 and gw.tasks[t].state == "picked"
    assert [(m["type"], m["to"], m["task_id"]) for m in sent] == [("Cancel", 2, t)]


def test_an_age_weight_lets_a_waiting_far_task_win_the_robot():
    # One robot, two open tasks: an old one far away and a new one close by.
    # Minimum drive time alone takes the near one; with the age term the old
    # one wins once its wait is worth more than the extra drive.
    picks = {}
    for w in (0.0, 5.0):
        s = sim.Sim(scenario(robots=1, tasks=[],
                             config={"dispatch": {"method": "central_hungarian",
                                                  "age_weight": w}}))
        gw = s.gw
        home = (s.robots[0].x, s.robots[0].y)
        far = gw.request("shelf_3_3", "dock_3", 0.0)
        near = gw.request("shelf_0_0", "dock_0", 50.0)
        gw.on_status(0, 60.0, None, "idle", home)
        gw._last_tick = 59.9
        gw.tick(60.0)
        sent = [m for m in _msgs(gw) if m["type"] == "Dispatch"]
        picks[w] = sent[0]["task_id"]
        extra = gw.costs.seconds("shelf_3_3", [gw.costs.snap(home)])[0] \
            - gw.costs.seconds("shelf_0_0", [gw.costs.snap(home)])[0]
        assert 0 < extra < 5.0 * 50.0  # the geometry the test assumes
    assert picks == {0.0: near, 5.0: far}


# ---- perfect information ---------------------------------------------------------------

BLOCK = {"rect": [9.0, 12.0, 10.0, 14.4]}


def _blockage_known(map_info):
    s = sim.Sim(scenario(robots=3, tasks=[], duration_s=1.0,
                         disturbances=[{"at_s": 0.5, "blockage": BLOCK}],
                         config={"info": {"map": map_info}}))
    s.run()
    i = s.grid.index(9.5, 13.2)
    return [int(r.belief.s[i]) == OCC for r in s.robots], s


def test_info_map_truth_makes_a_fresh_blockage_known_to_every_robot_at_once():
    known, s = _blockage_known("truth")
    assert all(known)
    assert all((r.belief.state == np.where(s.static, OCC, 0)).all() for r in s.robots)
    shared, _ = _blockage_known("shared")
    assert not any(shared)  # the robots are on the apron; nobody has seen it yet


def test_info_peers_truth_knows_peers_whose_status_never_arrives():
    # Status only within 1 cm: nobody hears anybody. With peers: truth the
    # local layer knows every peer's pose anyway (idle robots stand still,
    # so the pose of the last step is the pose now).
    for peers in ("heard", "truth"):
        s = sim.Sim(scenario(tasks=[], duration_s=2.0,
                             config={"status": {"recipients": "radius", "radius_m": 0.01},
                                     "info": {"peers": peers}}))
        s.run()
        if peers == "heard":
            assert np.isinf(s.seen_t[0, 1:]).all()
            continue
        assert (s.seen_t[0] >= s.now - s.step_s).all()
        assert [tuple(xy) for xy in s.seen_xy[0]] == [(r.x, r.y) for r in s.robots]


@pytest.mark.parametrize("method", ("auction",) + CENTRAL)
def test_perfect_information_assigns_through_a_partition(method):
    # info.dispatch: truth — /fleet/tasks crosses the cut instantly and off
    # the radio, so tasks announced while the gateway is cut off are still
    # assigned at once, and a central dispatcher never loses anyone.
    tasks = [{"at_s": t, "pickup": p, "dropoff": d} for t, p, d in (
        (10, "shelf_1_0", "dock_0"), (20, "shelf_2_2", "dock_2"), (30, "shelf_3_1", "dock_3"))]
    scn = scenario(robots=4, tasks=tasks, disturbances=GW_CUT, duration_s=500,
                   config={"dispatch": {"method": method},
                           "info": {"map": "truth", "peers": "truth", "dispatch": "truth"},
                           "network": {"ideal": {"delay_s": 0.0}}})
    s, summary = run(scn)
    v = score(s, [{"metric": "fleet_tasks"}, {"metric": "comms"}])
    assert v["fleet_tasks"]["done"] == 3 and v["fleet_tasks"]["duplicates"] == 0
    assert v["comms"]["tasks_delivery"] is None  # never on the radio
    disp = {e["task_id"]: e["t"] for e in s.events if e["s"] == "dispatch"}
    assert all(disp[f"fleet-{k}"] < 10 * k + 3 for k in (1, 2, 3)), disp
    if method != "auction":
        assert summary["dispatch"]["lost"] == 0


# ---- defaults are the robots as they were ------------------------------------------------

# sha256 over the announce/dispatch/task/gt/contact events of test_sim's
# scenario(), recorded on the tree just before fleetsim/dispatch existed
# (b4c9249, numpy 2.3.5, 2026-11-26). Narrow on purpose: other streams may
# grow fields; these are what a run does. Different numpy versions may draw
# different random streams, so the check runs only where it was recorded.
# Regenerate (only when a change is MEANT to alter default runs) with the
# loop below on the old and new trees and say why in the commit.
GOLDEN = {"numpy": "2.3.5", "events": 600,
          "sha256": "28d35a5ee271219ebe8a0d5fad0c518f5215ba57b336fd9139ac6ed21fa85626"}
CORE = {"announce", "dispatch", "task", "gt", "contact"}


@pytest.mark.skipif(np.__version__ != GOLDEN["numpy"], reason="golden recorded with another numpy")
def test_defaults_reproduce_the_event_stream_from_before_the_dispatch_layer():
    s, _ = run(scenario())
    h, n = hashlib.sha256(), 0
    for e in s.events:
        if e["s"] in CORE:
            h.update(json.dumps(e, sort_keys=True).encode())
            n += 1
    assert (n, h.hexdigest()) == (GOLDEN["events"], GOLDEN["sha256"])
