"""fleetsim/intent.py: the conflict maths, without a sim."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fleetsim import intent as I  # noqa: E402


def _abs(pts, now=0.0):
    p = np.array(pts, dtype=float)
    p[:, 2] += now
    return p


def test_announce_times_the_path_and_cuts_it_at_the_horizon():
    pts = I.announce([(0, 0), (4, 0), (4, 10)], v=0.5, horizon_s=10.0)
    assert pts[0] == [0.0, 0.0, 0.0]
    assert pts[1] == [4.0, 0.0, 8.0]
    assert pts[-1] == [4.0, 1.0, 10.0]  # 2 s more at 0.5 m/s past the corner


def test_crossing_peer_is_a_conflict_and_a_parallel_one_is_not():
    taus, own = I.along([(0, 0), (10, 0)], 0.5, 20.0, 0.25)
    crossing = _abs([[5, -5, 0.0], [5, 5, 20.0]])   # at (5, 0) at t = 10 s, as are we
    parallel = _abs([[0, 3, 0.0], [10, 3, 20.0]])
    c = I.first_conflict(own, taus, crossing, 0.9)
    # first within 0.9 m about 1.3 s before both reach (5, 0)
    assert c is not None and 8.0 < taus[c] < 10.0
    assert I.first_conflict(own, taus, parallel, 0.9) is None


def test_a_standing_peer_is_left_to_the_local_layer():
    taus, own = I.along([(0, 0), (10, 0)], 0.5, 20.0, 0.25)
    assert I.first_conflict(own, taus, _abs([[5, 0, 0.0]]), 0.9) is None


def test_safe_hold_is_short_of_the_peers_whole_path():
    taus, own = I.along([(0, 0), (10, 0)], 0.5, 20.0, 0.25)
    crossing = _abs([[5, -5, 0.0], [5, 5, 20.0]])
    c = I.first_conflict(own, taus, crossing, 0.9)
    i = I.safe_hold(own, c, crossing, 0.0, 1.1)
    assert i is not None and own[i][0] <= 5 - 1.1 + 1e-6
    assert own[i][0] > 3.5  # ...but as close as that allows, not back at the start


def test_no_safe_hold_when_we_stand_in_its_way():
    # head-on down the line the peer is coming up
    taus, own = I.along([(5, 0), (5, -10)], 0.5, 20.0, 0.25)
    through_us = _abs([[5, -5, 0.0], [5, 5, 20.0]])
    c = I.first_conflict(own, taus, through_us, 0.9)
    assert c is not None and I.safe_hold(own, c, through_us, 0.0, 1.1) is None


def test_clears_at_is_the_wait_a_yield_costs():
    taus, own = I.along([(0, 0), (10, 0)], 0.5, 20.0, 0.25)
    crossing = _abs([[5, -5, 0.0], [5, 5, 20.0]])
    c = I.first_conflict(own, taus, crossing, 0.9)
    assert 10.0 < I.clears_at(own, taus, crossing, 0.9, c) < 12.0


def test_priority_carrying_first_then_aging_then_index():
    assert I.outranks((0, 0, 5), (1, 0, 0))
    assert I.outranks((1, -1, 5), (1, 0, 0))
    assert I.outranks((1, 0, 0), (1, 0, 1))
