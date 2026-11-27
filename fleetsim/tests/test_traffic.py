"""fleetsim/traffic.py: keep right (B10) and single-lane passages (B11)."""
from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fleetsim import config, sim  # noqa: E402
from fleetsim import grid as G  # noqa: E402
from fleetsim import traffic as TR  # noqa: E402
from fleetsim.planner import FREE, OCC, Costmap  # noqa: E402
from fleetsim.robot import Robot  # noqa: E402

BASE = config.load_base()
NAV = BASE["nav"]
KR = {**BASE["traffic"]["keep_right"], "enabled": True}
SINGLE = float(BASE["traffic"]["single_lane_m"])
SMALL = {"width_m": 30.0, "height_m": 22.0, "docks": 4, "cross_every_m": 12.0}
AISLE_Y = 13.2  # aisle 1 of SMALL, 12.0-14.4 between racks; segment 0 spans x 3.4-13.8


def corridor(width_m=2.4, cell=0.2):
    """A 12 m E-W corridor between walls, open rooms at both ends."""
    g = G.Grid(int(20.0 / cell), int(10.0 / cell), cell, 0.0, 0.0)
    occ = np.zeros((g.height, g.width), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    lo = int(round((5.0 - width_m / 2) / cell))  # whole cells: floor(3.8 / 0.2) is 18
    hi = lo + int(round(width_m / cell))
    x0, x1 = int(round(4.0 / cell)), int(round(16.0 / cell))
    occ[:lo, x0:x1] = True
    occ[hi:, x0:x1] = True
    return g, occ


def lanes_on(g, occ):
    cm = Costmap(g, np.where(occ, OCC, FREE).astype(np.int8), NAV, 0.25)
    return cm, TR.attach_lanes(cm, KR, SINGLE)


def edt_gradient(g, occ):
    """Brute-force Euclidean distance to the nearest occupied cell centre and
    its central-difference gradient: the field the plan states the rule in."""
    ys, xs = np.nonzero(occ)
    cy, cx = np.mgrid[0:g.height, 0:g.width]
    d = np.full(occ.shape, np.inf)
    for oy, ox in zip(ys.tolist(), xs.tolist()):
        np.minimum(d, np.hypot(cx - ox, cy - oy) * g.res, out=d)
    gy, gx = np.gradient(d)
    return gx, gy


# ---- B10: the corridor field ---------------------------------------------------

def test_corridor_field_marks_the_aisle_and_its_left_half():
    g, occ = corridor()
    cm, ln = lanes_on(g, occ)
    mid = g.cell(10.0, 5.0)
    room = g.cell(2.0, 5.0)
    assert ln.c[mid[1], mid[0]] == 1.0 and ln.ax[mid[1], mid[0]] == 1  # runs along x
    assert ln.c[room[1], room[0]] == 0.0  # open floor
    # Left of an eastbound move is north; the rule's sign agrees with
    # cross(move, gradient of the Euclidean distance) < 0 in every cell of
    # the aisle off its centre line.
    gx, gy = edt_gradient(g, occ)
    x = mid[0]
    checked = 0
    for cy in range(g.height):
        j = cy * g.width + x
        if occ[cy, x] or ln.c[cy, x] < 1.0:
            continue
        for mx, my in ((1.0, 0.0), (-1.0, 0.0)):
            s = ln.sigma(j, mx, my)
            cross = mx * gy[cy, x] - my * gx[cy, x]
            if abs(s) > 0.15:  # off the centre line, where the gradient has a side
                assert (s > 0) == (cross < 0)
                checked += 1
    assert checked >= 16
    # The step cost: east costs nothing extra on the south edge of the flat
    # band, the full weight on the north edge; west the other way round.
    south, north = g.cell(10.0, 4.5)[1] * g.width + x, g.cell(10.0, 5.5)[1] * g.width + x
    east, west = 0, 1  # NEIGHBOURS_INT order
    assert ln.wdir[east][south] == cm.wint[south] and ln.wdir[west][north] == cm.wint[north]
    assert ln.wdir[east][north] == cm.wint[north] + 1000 and ln.wdir[west][south] == cm.wint[south] + 1000
    # Crossing the aisle costs nothing extra.
    assert ln.wdir[2][north] == cm.wint[north] and ln.wdir[3][south] == cm.wint[south]


def test_single_lane_and_wide_corridors_get_no_lanes():
    for width, lanes in ((1.0, False), (2.4, True), (3.0, True), (4.0, False)):
        g, occ = corridor(width)
        _cm, ln = lanes_on(g, occ)
        c = g.cell(10.0, 5.0)
        assert (ln.c[c[1], c[0]] > 0) == lanes, width


def test_window_update_matches_a_fresh_build():
    g, occ = corridor()
    cm, ln = lanes_on(g, occ)
    c = g.cell(10.0, 4.6)
    cm.state[c[1], c[0]] = OCC  # a box in the aisle
    cm.update_cells(np.array([c[1] * g.width + c[0]]))
    ln.refresh(10.0)
    occ2 = occ.copy()
    occ2[c[1], c[0]] = True
    cm2, ln2 = lanes_on(g, occ2)
    for a, b in zip(ln.wdir, ln2.wdir):
        assert a == b


def _smoothed(cm, a, b):
    raw = cm.plan(a, b)
    fake = SimpleNamespace(cost=cm, sim=SimpleNamespace(grid=cm.grid))
    return Robot._smooth(fake, raw)


def test_a_planned_path_down_an_aisle_keeps_to_its_right_half():
    g, occ = corridor()
    cm, _ln = lanes_on(g, occ)
    # Start and goal on the centre line, as shelves are: the smoother must
    # not pull the leg taut down the middle.
    for a, b, right in (((5.0, 5.0), (15.0, 5.0), -1), ((15.0, 5.0), (5.0, 5.0), +1)):
        pts = _smoothed(cm, a, b)
        mids = [y for x, y in _sample(pts) if 7.0 <= x <= 13.0]
        assert mids and all((y - 5.0) * right >= 0.4 for y in mids), (a, b, pts)
    plain = Costmap(g, np.where(occ, OCC, FREE).astype(np.int8), NAV, 0.25)
    pts = _smoothed(plain, (5.0, 5.0), (15.0, 5.0))
    assert any(abs(y - 5.0) < 0.2 for x, y in _sample(pts) if 7.0 <= x <= 13.0)  # without: the middle


def _sample(pts, step=0.1):
    out = []
    for a, b in zip(pts[:-1], pts[1:]):
        n = max(1, int(math.dist(a, b) / step))
        out += [(a[0] + i / n * (b[0] - a[0]), a[1] + i / n * (b[1] - a[1])) for i in range(n)]
    return out


def _meet(keep_right: bool, predict_s=None):
    """Two robots sent past each other down a 2.4 m aisle, shelf line to
    shelf line. -> (stops, seconds stopped, when each arrived)."""
    a, b = (4.5, AISLE_Y), (12.8, AISLE_Y)
    raw = {"name": "t", "seed": 1, "duration_s": 45, "robots": 2,
           "config": {"world": {"synthetic": SMALL, "homes": [list(a), list(b)]},
                      "map_share": {"prior": "truth"}, "nav": {"return_home_s": None},
                      "traffic": {"keep_right": {"enabled": keep_right}}}}
    if predict_s is not None:
        raw["config"]["nav"]["predict_s"] = predict_s
    s = sim.Sim(config.resolve(raw))
    s.at(1.0, s.robots[0].set_goal, b, "home")
    s.at(1.0, s.robots[1].set_goal, a, "home")
    s.run()
    stops = [e for e in s.events if e["s"] == "nav" and e["event"] == "unblocked"]
    arrived = [e["t"] for e in s.events if e["s"] == "path" and not e["pts"]]
    assert len(arrived) == 2  # both got there
    return len(stops), sum(e["after_s"] for e in stops), arrived


def test_two_robots_meeting_head_on_pass_without_a_stop():
    # The camera reports a peer at the nearest point it saw, with a velocity
    # from consecutive such points; as the peer slides past, that point
    # swings sideways and predict_s throws it toward our lane. Without that
    # prediction the meeting costs no stop at all...
    assert _meet(True, predict_s=0.0)[0] == 0
    assert _meet(False, predict_s=0.0)[0] >= 3
    # ...and with it, one blip of a step or two, against 7-12 stops today.
    n, secs, arrived = _meet(True)
    off_n, off_secs, off_arrived = _meet(False)
    assert n <= 2 and secs <= 0.5
    assert off_n >= 5 and off_secs >= 5.0
    assert max(arrived) < max(off_arrived)


# ---- B11: passages ---------------------------------------------------------------

def _floor(**syn):
    occ, g, wps, _homes, _gw = G.synthetic_warehouse({**SMALL, **syn}, 0.2)
    return g, np.where(occ, OCC, FREE).astype(np.int8), wps


def test_passages_found_on_a_narrow_aisle_and_a_dead_end_bay():
    g, st, _ = _floor()
    pm = TR.PassageMap(g, st, SINGLE, 0.35)
    pm.refresh()
    assert pm.boxes == []  # the plain floor has none
    g, st, wps = _floor(narrow_aisles=[2], dead_ends=[[2, 0, "west"]])
    pm = TR.PassageMap(g, st, SINGLE, 0.35)
    pm.refresh()
    assert len(pm.boxes) == 2  # aisle 2 beside segments 0 and 1
    bay, through = sorted(pm.boxes)
    # From the dead-end wall's face (3.4 + 0.4) to the crossing, and through
    # the segment: as far as the racks reach in cells (fill() takes every
    # cell a rack touches, so segment 0's 13.8 m end is the cell to 14.0).
    occ = st == OCC
    row = occ[g.cell(0, 16.0)[1]]
    rack = [g.centre(x, 0)[0] for x in range(g.width) if row[x]]
    assert bay[0] == pytest.approx(3.8) and bay[2] == pytest.approx(max(x for x in rack if x < 15) + 0.1)
    assert through[0] == pytest.approx(min(x for x in rack if x > 15) - 0.1)
    assert through[2] == pytest.approx(max(x for x in rack if x < 28) + 0.1)
    for b in pm.boxes:
        assert b[3] - b[1] == pytest.approx(1.0)  # one robot wide
    # A leg into the bay ends inside it; one along the aisle runs through.
    lab = {b: k + 1 for k, b in enumerate(pm.boxes)}
    into, _ = TR.path_crossings([(15.0, 16.8), wps["shelf_2_0"]], pm.flat, g, 0.1)
    assert [(c[0], c[2]) for c in into] == [(lab[bay], None)]
    thru, _ = TR.path_crossings([(15.0, 16.8), (28.0, 16.8)], pm.flat, g, 0.1)
    assert [c[0] for c in thru] == [lab[through]]
    # in and out where the box is, to a sample (0.1 m) and a float's floor()
    assert thru[0][1] == pytest.approx(through[0] - 15.0, abs=0.15)
    assert thru[0][2] == pytest.approx(through[2] - 15.0, abs=0.15)


def test_claims_order_inside_first_then_time_then_priority_then_index():
    k = TR.claim_key
    assert k({"inside": True, "t0": 9.0, "prio": 2}, 5) < k({"inside": False, "t0": 1.0, "prio": 0}, 0)
    assert k({"inside": False, "t0": 1.0, "prio": 2}, 5) < k({"inside": False, "t0": 1.1, "prio": 0}, 0)
    assert k({"inside": False, "t0": 1.0, "prio": 0}, 5) < k({"inside": False, "t0": 1.0, "prio": 1}, 0)
    assert k({"inside": False, "t0": 1.0, "prio": 1}, 0) < k({"inside": False, "t0": 1.0, "prio": 1}, 5)


def _narrow_scenario(seed, n=6, **over):
    tasks = [{"at_s": 1 + 3 * i, "pickup": f"shelf_2_{i % 4}", "dropoff": f"dock_{i % 4}"}
             for i in range(2 * n)]
    raw = {"name": "t", "seed": seed, "duration_s": 600, "robots": n,
           "config": {"world": {"synthetic": {**SMALL, "narrow_aisles": [2]}},
                      "map_share": {"prior": "truth"},
                      "traffic": {"passages": {"enabled": True}}},
           "tasks": tasks, "stop": {"until": "tasks_done", "settle_s": 1}}
    return config.resolve(sim.deep_merge(raw, over))


def test_two_claimants_are_never_both_inside_a_passage():
    holds = 0
    for seed in (1, 2, 3):
        s = sim.Sim(_narrow_scenario(seed))
        s.run()
        inside: dict[tuple, dict] = {}
        for e in s.events:
            if e["s"] != "nav":
                continue
            if e["event"] == "pass_enter":
                box = tuple(e["box"])
                others = set(inside.get(box, {})) - {e["rid"]}
                assert not others, (seed, e, others)
                inside.setdefault(box, {})[e["rid"]] = e["t"]
            elif e["event"] == "pass_exit":
                inside[tuple(e["box"])].pop(e["rid"])
            elif e["event"] == "pass_hold":
                holds += 1
            assert e["event"] != "pass_hold_timeout"
        assert sum(1 for e in s.events if e["s"] == "task" and e["state"] == "DONE") == 12
    assert holds >= 3  # the property was exercised, not dodged


def test_a_dead_holders_claim_lapses():
    # robot_0 stands in the narrow aisle (an occupant's claim, on every
    # Status); robot_1 on the apron hears it. Kill robot_0 at t = 3.
    g, st, wps = _floor(narrow_aisles=[2])
    raw = {"name": "t", "seed": 1, "duration_s": 8, "robots": 2,
           "config": {"world": {"synthetic": {**SMALL, "narrow_aisles": [2]},
                                "homes": [list(wps["shelf_2_1"]), [8.0, 4.0]]},
                      "map_share": {"prior": "truth"}, "nav": {"return_home_s": None},
                      "traffic": {"passages": {"enabled": True}}},
           "disturbances": [{"at_s": 3.0, "kill": "robot_0"}]}
    s = sim.Sim(config.resolve(raw))
    ttl = s.cfg["traffic"]["passages"]["ttl_s"]
    box = s.robots[1].traffic.map.boxes[s.robots[1].traffic.map.label_at(*wps["shelf_2_1"]) - 1]
    seen = {}
    for t in (2.5, 3.0 + ttl - 0.3, 3.0 + ttl + 0.3):
        s.at(t, lambda t=t: seen.__setitem__(t, s.robots[1].traffic.first_claim(box, s.now)))
    s.run()
    assert seen[2.5][0] == 0 and seen[2.5][1]["inside"]
    assert seen[3.0 + ttl - 0.3][0] == 0  # not yet lapsed
    assert seen[3.0 + ttl + 0.3] is None  # lapsed: nobody holds it


def test_a_status_without_the_claim_releases_it_at_once():
    s = sim.Sim(_narrow_scenario(1, n=2))
    ag = s.robots[1].traffic
    box = ag.map.boxes[0]
    ag.on_claims(0, 10.0, [[list(box), 9.0, 1, 0]], 10.0)
    assert ag.first_claim(box, 10.1)[0] == 0
    ag.on_claims(0, 9.8, [], 10.2)  # an older Status, overtaken: ignored
    assert ag.first_claim(box, 10.3)[0] == 0
    ag.on_claims(0, 10.2, [], 10.4)
    assert ag.first_claim(box, 10.5) is None


# ---- defaults ------------------------------------------------------------------------

# The narrow-aisle run below with every default, on the tree before traffic.py
# (c7bf064, 2026-11-26): every event but wall, wall_s and summary.
GOLDEN = {"numpy": "2.3.5", "events": 2501,
          "sha256": "3107351263918571dab03290bbbc69f123703d455dc6b84730ec0a5bb1e5036b"}


@pytest.mark.skipif(np.__version__ != GOLDEN["numpy"], reason="golden recorded with another numpy")
def test_defaults_reproduce_the_event_stream_from_before_traffic():
    import hashlib
    import json
    tasks = [{"at_s": 1 + 3 * i, "pickup": f"shelf_2_{i % 4}", "dropoff": f"dock_{i % 4}"}
             for i in range(12)]
    raw = {"name": "t", "seed": 1, "duration_s": 600, "robots": 6,
           "config": {"world": {"synthetic": {**SMALL, "narrow_aisles": [2]}},
                      "map_share": {"prior": "truth"}},
           "tasks": tasks, "stop": {"until": "tasks_done", "settle_s": 1}}
    s = sim.Sim(config.resolve(raw))
    s.run()
    h = hashlib.sha256()
    for e in s.events:
        h.update(json.dumps({k: v for k, v in e.items() if k not in ("wall", "wall_s", "summary")},
                            sort_keys=True).encode())
    assert (len(s.events), h.hexdigest()) == (GOLDEN["events"], GOLDEN["sha256"])


def test_with_both_off_nothing_of_it_runs():
    scn = config.resolve({"name": "t", "seed": 1, "duration_s": 60, "robots": 3,
                          "config": {"world": {"synthetic": {**SMALL, "narrow_aisles": [2]}},
                                     "map_share": {"prior": "truth"}},
                          "tasks": [{"at_s": 1, "pickup": "shelf_2_1", "dropoff": "dock_0"}]})
    s = sim.Sim(scn)
    for r in s.robots:
        assert r.traffic is None and r.cost.lanes is None
        assert all(a is r.cost.wint for *_x, a in r.cost._nbrs)  # A* reads what it always read
    summary = s.run()
    assert not any(e["s"] == "nav" and e["event"].startswith("pass_") for e in s.events)
    assert set(summary["nav"]) == {"replans", "replan_sensed", "replan_shared", "replan_blocked",
                                   "plan_fail", "blocked_s", "stuck", "shared_changed", "yields",
                                   "yield_s"}
