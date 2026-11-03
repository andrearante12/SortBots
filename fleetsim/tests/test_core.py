"""fleetsim building blocks, offline and fast: planner, belief fusion,
sensing, network. System python3 (numpy + PyYAML), no ROS."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fleetsim import grid as G  # noqa: E402
from fleetsim import network as N  # noqa: E402
from fleetsim.belief import Belief  # noqa: E402
from fleetsim.planner import FREE, OCC, UNKNOWN, Costmap, path_length  # noqa: E402
from fleetsim.sensing import Sensor  # noqa: E402

NAV = {"inflation_m": 0.65, "cost_scaling": 2.5, "cost_weight": 3.0, "unknown_cost": 0.0}


def open_room(w_m=10.0, h_m=6.0, cell=0.2):
    g = G.Grid(int(w_m / cell), int(h_m / cell), cell, 0.0, 0.0)
    occ = np.zeros((g.height, g.width), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    return g, occ


def belief_from(occ):
    return np.where(occ, OCC, FREE).astype(np.int8)


# ---- planner ----------------------------------------------------------------

def test_open_floor_path_is_near_straight_line():
    g, occ = open_room()
    cm = Costmap(g, belief_from(occ), NAV, 0.32)
    p = cm.plan((2.0, 3.0), (8.0, 3.0))
    assert p is not None
    assert path_length(p) == pytest.approx(6.0, abs=0.3)


def test_wall_forces_detour_through_the_gap():
    g, occ = open_room()
    wall_x = g.cell(5.0, 0.0)[0]
    occ[:, wall_x] = True
    gap = slice(g.cell(0, 4.6)[1], g.cell(0, 5.8)[1])  # a 1.2 m doorway near the top
    occ[gap, wall_x] = False
    cm = Costmap(g, belief_from(occ), NAV, 0.32)
    p = cm.plan((2.0, 1.0), (8.0, 1.0))
    assert p is not None
    assert max(y for _x, y in p) > 4.5  # went through the doorway
    assert all(not cm.lethal_xy(x, y) for x, y in p[1:-1])


def test_lethal_band_keeps_the_body_off_walls():
    g, occ = open_room()
    cm = Costmap(g, belief_from(occ), NAV, 0.32)
    # the wall row is y in [0, 0.2): a centre 0.3 m off it is inside the body radius
    assert cm.lethal_xy(5.0, 0.3)
    assert not cm.lethal_xy(5.0, 1.5)


def test_unknown_is_free_when_optimistic_and_avoided_when_costly():
    g, occ = open_room()
    st = belief_from(occ)
    band = slice(g.cell(0, 2.0)[1], g.cell(0, 4.0)[1])
    st[band, 10:40] = UNKNOWN
    straight = Costmap(g, st, NAV, 0.32).plan((1.5, 3.0), (8.5, 3.0))
    cautious = Costmap(g, st, {**NAV, "unknown_cost": 5.0}, 0.32).plan((1.5, 3.0), (8.5, 3.0))
    assert path_length(straight) < path_length(cautious)


def test_no_path_returns_none():
    g, occ = open_room()
    occ[:, g.cell(5.0, 0)[0]] = True
    assert Costmap(g, belief_from(occ), NAV, 0.32).plan((2.0, 3.0), (8.0, 3.0)) is None


def test_window_update_matches_full_rebuild():
    g, occ = open_room()
    st = belief_from(occ)
    cm = Costmap(g, st, NAV, 0.32)
    c = g.cell(5.0, 3.0)
    st[c[1], c[0]] = OCC
    cm.update_cells(np.array([c[1] * g.width + c[0]]))
    fresh = Costmap(g, st.copy(), NAV, 0.32)
    assert list(cm.mult) == list(fresh.mult)


# ---- belief -----------------------------------------------------------------

def test_newest_observation_wins_and_a_removed_box_clears():
    g, _ = open_room()
    a, b = Belief(g, 3.2), Belief(g, 3.2)
    cell = np.array([g.index(5.0, 3.0)])
    a.observe(np.array([], dtype=np.int64), cell, t=10.0)   # A sees a box
    b.merge(*a.cells_payload(cell))
    assert b.s[cell[0]] == OCC
    b.observe(cell, np.array([], dtype=np.int64), t=20.0)   # B later sees it gone
    a.merge(*b.cells_payload(cell))
    assert a.s[cell[0]] == FREE
    # an older report can't bring it back
    stale = (cell, np.array([OCC], np.int8), np.array([15.0], np.float32))
    assert len(a.merge(*stale)) == 0 and a.s[cell[0]] == FREE


def test_only_own_changes_are_dirty():
    g, _ = open_room()
    a = Belief(g, 3.2)
    cell = np.array([g.index(5.0, 3.0)])
    a.merge(cell, np.array([OCC], np.int8), np.array([1.0], np.float32))
    assert a.take_dirty() == {}
    a.observe(np.array([g.index(2.0, 2.0)]), np.array([], dtype=np.int64), t=2.0)
    d = a.take_dirty()
    assert sum(len(v) for v in d.values()) == 1 and a.take_dirty() == {}


def test_sequence_gap_asks_for_resync_but_a_snapshot_does_not():
    g, _ = open_room()
    b = Belief(g, 3.2)
    assert not b.check_seq(origin=1, tile=0, seq=1, snapshot=False)
    assert b.check_seq(origin=1, tile=0, seq=3, snapshot=False)   # 2 was lost
    assert not b.check_seq(origin=1, tile=0, seq=7, snapshot=True)


def test_prior_loses_to_any_observation():
    g, occ = open_room()
    b = Belief(g, 3.2, prior=occ)
    cell = np.array([g.index(5.0, 3.0)])
    assert b.s[cell[0]] == FREE
    b.merge(cell, np.array([OCC], np.int8), np.array([0.0], np.float32))
    assert b.s[cell[0]] == OCC


# ---- sensing ----------------------------------------------------------------

def test_scan_sees_walls_ahead_not_behind_and_stops_at_them():
    g, occ = open_room()
    wall_x = g.cell(6.0, 0)[0]
    occ[:, wall_x] = True
    sensor = Sensor(g, {"range_m": 4.5, "fov_deg": 87.0})
    none_ = np.zeros(g.size, dtype=np.int32)
    free, hits, dets = sensor.scan(3.0, 3.0, 0.0, occ.reshape(-1), none_, none_.astype(np.int16), 1)
    xs = {i % g.width for i in hits.tolist()}
    assert wall_x in xs
    assert all(i % g.width <= wall_x for i in free.tolist())     # nothing seen through it
    assert all(g.centre_of(i)[0] > 2.6 for i in free.tolist())   # nothing behind the robot
    assert dets == []


def test_a_peer_is_a_detection_and_occludes_but_is_not_mapped():
    g, occ = open_room()
    owner = np.zeros(g.size, dtype=np.int32)
    peer = [g.index(4.5, 3.0 + dy) for dy in (-0.2, 0.0, 0.2)]
    owner[peer] = 2
    sensor = Sensor(g, {"range_m": 4.5, "fov_deg": 87.0})
    free, hits, dets = sensor.scan(3.0, 3.0, 0.0, occ.reshape(-1), owner,
                                   np.zeros(g.size, np.int16), 1)
    assert [d[:2] for d in dets] == [("robot", 1)]
    assert not set(peer) & set(hits.tolist())
    behind = g.index(5.5, 3.0)
    assert behind not in set(free.tolist())


# ---- network ----------------------------------------------------------------

def test_ideal_network_delivers_in_order_and_partitions_cut():
    rng = np.random.default_rng(0)
    net = N.IdealNetwork(4, {"ideal": {"delay_s": 0.003}}, rng)
    net.update(np.zeros((4, 2)), 0.0)
    d, arr, _, _ = net.send("tasks", 1, np.arange(4), 100, True, 0.0)
    assert sorted(d.tolist()) == [0, 2, 3] and np.all(arr > 0)
    d2, arr2, _, _ = net.send("tasks", 1, np.array([2]), 100, True, 0.0)
    assert arr2[0] > arr[d.tolist().index(2)]  # in order per pair
    net.block(1, 2, 0.0)
    net.update(np.zeros((4, 2)), 0.01)
    d, _, _, _ = net.send("status", 1, np.arange(4), 100, False, 0.01)
    assert 2 not in d.tolist() and {0, 3} <= set(d.tolist())


def _radio(positions, occ=None, g=None):
    if g is None:
        g = G.Grid(300, 100, 0.2, 0.0, 0.0)
        occ = np.zeros((g.height, g.width), dtype=bool)
    net = N.RadioNetwork(len(positions), {"radio": {}}, np.random.default_rng(1), occ, g)
    net.update(np.array(positions, dtype=float), 0.0)
    return net


def test_radio_close_pair_links_and_far_pair_needs_relays():
    near = _radio([(10, 10), (12, 10)])
    assert near.reach[0, 1] and near.hops[0, 1] == 1 and near.p[0, 1] > 0.99
    # a wall of racks between two robots kills the direct link; a relay
    # around the end of the wall carries it
    g = G.Grid(300, 100, 0.2, 0.0, 0.0)
    occ = np.zeros((g.height, g.width), dtype=bool)
    for x0 in range(40, 220, 30):  # six thick racks across the floor
        occ[5:90, x0:x0 + 10] = True
    net = _radio([(4, 10), (56, 10), (30, 19.5)], occ, g)
    assert net.snr[0, 1] < net.snr[0, 2]
    assert net.reach[0, 1]
    assert net.hops[0, 1] == 2


def test_radio_load_grows_with_traffic_and_slows_delivery():
    net = _radio([(10, 10), (12, 10), (14, 10)])
    _, quiet, _, _ = net.send("status", 0, np.array([1]), 200, False, 0.0)
    for _ in range(4000):
        net.send("status", 0, np.array([1, 2]), 1400, False, 0.0)
    net.update(net.pos, 1.0)
    assert net.load.max() > 0.5
    _, busy, _, _ = net.send("status", 0, np.array([1]), 200, False, 1.0)
    assert len(busy) == 0 or (busy[0] - 1.0) > quiet[0]


def test_vectorised_crossings_match_layout():
    from commsim.des import layout as L

    lay = L.warehouse()
    rects = N._rect_arrays(lay)
    rng = np.random.default_rng(3)
    a = rng.uniform(0, 50, (200, 2)) * [1, 0.64]
    b = rng.uniform(0, 50, (200, 2)) * [1, 0.64]
    racks, shelves = N._crossings(rects, a, b)
    for i in range(len(a)):
        assert (racks[i], shelves[i]) == lay.crossings(tuple(a[i]), tuple(b[i]))


def test_synthetic_warehouse_waypoints_are_reachable():
    occ, g, wps, homes, gw = G.synthetic_warehouse({}, 0.2)
    cm = Costmap(g, belief_from(occ), NAV, 0.32)
    for name in ("shelf_1_3", "shelf_8_7", "dock_0", "dock_11"):
        assert cm.plan(homes[0], wps[name]) is not None, name
    assert len(homes) >= 50
    assert all(math.dist(homes[0], h) >= 1.6 - 1e-6 for h in homes[1:])
