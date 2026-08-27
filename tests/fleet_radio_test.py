"""Offline tests for fleet mesh radio helpers (no ROS)."""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))

import fleet_radio as fr  # noqa: E402


def test_encode_parse_status_roundtrip():
    raw = fr.encode_status("robot_1", 1.5, -2.0, 0.3, vx=0.1, wz=-0.2, mode="explore", stamp=123.0)
    st = fr.parse_status(raw)
    assert st["robot_id"] == "robot_1"
    assert st["x"] == pytest.approx(1.5)
    assert st["y"] == pytest.approx(-2.0)
    assert st["yaw"] == pytest.approx(0.3)
    assert st["vx"] == pytest.approx(0.1)
    assert st["mode"] == "explore"
    assert st["stamp"] == pytest.approx(123.0)


def test_encode_parse_intent_with_corridor_and_release():
    raw = fr.encode_intent(
        "robot_0", 3.0, 4.0,
        corridor=[[1.0, 1.0], [2.0, 2.0]],
        priority=0,
        expires_at=9999999999.0,
    )
    it = fr.parse_intent(raw)
    assert it["x"] == pytest.approx(3.0)
    assert it["corridor"] == [[1.0, 1.0], [2.0, 2.0]]
    assert it["priority"] == 0
    assert not it["released"]

    rel = fr.parse_intent(fr.encode_intent("robot_0", 0, 0, released=True))
    assert rel["released"] is True


def test_active_peer_poses_ttl():
    now = 100.0
    statuses = {
        "robot_1": ({"x": 1.0, "y": 2.0}, 99.5),
        "robot_2": ({"x": 9.0, "y": 9.0}, 98.0),  # stale if ttl=1
    }
    poses = fr.active_peer_poses(statuses, now=now, ttl_s=1.0, self_id="robot_0")
    assert poses == [(1.0, 2.0)]


def test_active_peer_intents_respects_expiry_and_self():
    now_wall = time.time()
    intents = {
        "robot_0": ({"robot_id": "robot_0", "x": 0, "y": 0, "corridor": [],
                     "priority": 0, "expires_at": now_wall + 10, "released": False}, 0.0),
        "robot_1": ({"robot_id": "robot_1", "x": 5, "y": 5, "corridor": [[4, 4]],
                     "priority": 1, "expires_at": now_wall + 10, "released": False}, 0.0),
        "robot_2": ({"robot_id": "robot_2", "x": 1, "y": 1, "corridor": [],
                     "priority": 2, "expires_at": now_wall - 1, "released": False}, 0.0),
    }
    active = fr.active_peer_intents(
        intents, now_wall=now_wall, now_mono=1.0, ttl_s=90.0, self_id="robot_0"
    )
    assert len(active) == 1
    assert active[0]["robot_id"] == "robot_1"


def test_corridor_points_includes_goal_and_vertices():
    pts = fr.corridor_points([
        {"x": 3.0, "y": 4.0, "corridor": [[1.0, 1.0], [2.0, 2.0]]},
    ])
    assert (3.0, 4.0) in pts
    assert (1.0, 1.0) in pts


def test_segments_intersect_crossing():
    assert fr.segments_intersect((0, 0), (2, 2), (0, 2), (2, 0), pad_m=0.0)
    assert not fr.segments_intersect((0, 0), (1, 0), (0, 2), (1, 2), pad_m=0.1)


def test_should_yield_higher_priority_peer_blocks():
    # robot_0 priority 0 outranks robot_1 priority 1 — robot_1 should yield
    blocker = fr.should_yield(
        own_priority=1,
        own_start=(0.0, 0.0),
        own_goal=(10.0, 0.0),
        peer_intents=[{
            "robot_id": "robot_0",
            "x": 10.0,
            "y": 0.0,
            "corridor": [[0.0, 0.0], [10.0, 0.0]],
            "priority": 0,
            "expires_at": time.time() + 60,
            "released": False,
        }],
        pad_m=0.6,
    )
    assert blocker is not None
    assert blocker["robot_id"] == "robot_0"

    # Same paths but we outrank them — no yield
    assert fr.should_yield(
        own_priority=0,
        own_start=(0.0, 0.0),
        own_goal=(10.0, 0.0),
        peer_intents=[{
            "robot_id": "robot_1",
            "x": 10.0,
            "y": 0.0,
            "corridor": [[0.0, 0.0], [10.0, 0.0]],
            "priority": 1,
            "expires_at": time.time() + 60,
            "released": False,
        }],
        pad_m=0.6,
    ) is None


def _peer_intent(corridor, goal, priority=0, robot_id="robot_0"):
    return {
        "robot_id": robot_id,
        "x": goal[0],
        "y": goal[1],
        "corridor": corridor,
        "priority": priority,
        "expires_at": time.time() + 60,
        "released": False,
    }


def test_should_yield_horizon_ignores_distant_crossing():
    # Peer corridor crosses ours 10 m ahead. Without a horizon that's a
    # conflict; with a 3 m one it isn't, because both robots replan long
    # before either gets there. This is what stopped robot_1 from ever
    # sending a goal — see should_yield's docstring.
    far_crossing = [_peer_intent([[10.0, -5.0], [10.0, 5.0]], (10.0, 5.0))]
    assert fr.should_yield(
        own_priority=1,
        own_start=(0.0, 0.0),
        own_goal=(20.0, 0.0),
        peer_intents=far_crossing,
        pad_m=0.6,
    ) is not None
    assert fr.should_yield(
        own_priority=1,
        own_start=(0.0, 0.0),
        own_goal=(20.0, 0.0),
        peer_intents=far_crossing,
        pad_m=0.6,
        horizon_m=3.0,
    ) is None


def test_should_yield_horizon_still_blocks_imminent_crossing():
    # A crossing 2 m ahead is inside the horizon and must still yield —
    # the horizon must not disable the mechanism outright.
    near_crossing = [_peer_intent([[2.0, -5.0], [2.0, 5.0]], (2.0, 5.0))]
    blocker = fr.should_yield(
        own_priority=1,
        own_start=(0.0, 0.0),
        own_goal=(20.0, 0.0),
        peer_intents=near_crossing,
        pad_m=0.6,
        horizon_m=3.0,
    )
    assert blocker is not None
    assert blocker["robot_id"] == "robot_0"


def test_priority_for_robot_id():
    assert fr.priority_for_robot_id("robot_0") == 0
    assert fr.priority_for_robot_id("robot_3") == 3
    assert fr.priority_for_robot_id("other") == 100
