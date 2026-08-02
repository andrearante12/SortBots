#!/usr/bin/env python3
"""Unit tests for task_manager.Station's dock geometry + waypoints.yaml sanity.

Pure python — imports only the dataclass-ish `Station`/`load_stations` half of
nodes/task_manager.py, which has no rclpy at module scope.

    python3 -m pytest tests/station_dock_test.py

The reach assertion at the bottom is the one that matters: the arm is only
~0.50 m fully extended, and the config shipped a 0.5 m dock offset for months,
which parked the base a whole arm-length from the shelf.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "nodes"))

pytest.importorskip("rclpy", reason="task_manager imports rclpy at module scope")

from task_manager import Station, load_stations  # noqa: E402

# Summed URDF link origins Rotation -> Fixed_Jaw_tip. This is the fully
# extended straight-line reach from the shoulder, so it is an upper bound —
# a usable dock offset has to be comfortably inside it.
ARM_REACH_M = 0.50


def station(**over) -> Station:
    spec = {"kind": "pickup", "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}}
    spec.update(over)
    return Station("s", spec)


def test_zero_offset_docks_on_the_station():
    s = station(dock_offset_m=0.0)
    assert s.dock_pose() == pytest.approx((0.0, 0.0, 0.0))


@pytest.mark.parametrize("yaw, expect", [
    (0.0,             (-0.3,  0.0)),   # facing +x -> back off toward -x
    (math.pi / 2,     ( 0.0, -0.3)),   # facing +y -> back off toward -y
    (math.pi,         ( 0.3,  0.0)),
    (-math.pi / 2,    ( 0.0,  0.3)),
])
def test_backs_off_along_the_reverse_facing_direction(yaw, expect):
    s = station(pose={"x": 0.0, "y": 0.0, "yaw": yaw}, dock_offset_m=0.3)
    x, y, got_yaw = s.dock_pose()
    assert (x, y) == pytest.approx(expect, abs=1e-9)
    # Backing off must never change which way the robot faces.
    assert got_yaw == yaw


def test_dock_pose_keeps_the_station_at_offset_distance():
    s = station(pose={"x": 2.0, "y": -3.0, "yaw": 0.9}, dock_offset_m=0.28)
    x, y, _ = s.dock_pose()
    assert math.hypot(x - s.x, y - s.y) == pytest.approx(0.28)


def test_lateral_defaults_to_zero_preserving_old_behaviour():
    # dock_lateral_m is new; a config without it must dock exactly as before.
    assert station(dock_offset_m=0.4).dock_lateral_m == 0.0
    a = station(pose={"x": 1.0, "y": 2.0, "yaw": 0.7}, dock_offset_m=0.4).dock_pose()
    b = station(pose={"x": 1.0, "y": 2.0, "yaw": 0.7},
                dock_offset_m=0.4, dock_lateral_m=0.0).dock_pose()
    assert a == pytest.approx(b)


@pytest.mark.parametrize("yaw, expect", [
    (0.0,         (0.0,  0.2)),   # facing +x -> lateral is +y
    (math.pi / 2, (-0.2, 0.0)),   # facing +y -> lateral is -x
])
def test_lateral_shifts_ninety_degrees_from_facing(yaw, expect):
    s = station(pose={"x": 0.0, "y": 0.0, "yaw": yaw},
                dock_offset_m=0.0, dock_lateral_m=0.2)
    x, y, _ = s.dock_pose()
    assert (x, y) == pytest.approx(expect, abs=1e-9)


def test_lateral_is_perpendicular_to_the_backoff():
    # The two terms must not interfere: total displacement is the hypotenuse.
    s = station(pose={"x": 0.0, "y": 0.0, "yaw": 1.1},
                dock_offset_m=0.3, dock_lateral_m=0.2)
    x, y, _ = s.dock_pose()
    assert math.hypot(x, y) == pytest.approx(math.hypot(0.3, 0.2))


# ----------------------------------------------------------- shipped config


@pytest.fixture(scope="module")
def stations():
    return load_stations(REPO_ROOT / "configs" / "waypoints.yaml")


def test_config_has_both_kinds(stations):
    kinds = {s.kind for s in stations.values()}
    assert kinds == {"pickup", "dropoff"}


def test_every_dock_offset_is_within_arm_reach(stations):
    for name, s in stations.items():
        assert 0 < s.dock_offset_m < ARM_REACH_M, (
            f"{name}: dock_offset_m={s.dock_offset_m} is at or beyond the arm's "
            f"~{ARM_REACH_M} m fully-extended reach — the base would park a whole "
            f"arm-length from the shelf and no pick pose could ever reach it"
        )


def test_prop_decks_are_reachable_and_consistent(stations):
    import yaml

    raw = yaml.safe_load((REPO_ROOT / "configs" / "waypoints.yaml").read_text())
    for name, spec in raw["stations"].items():
        prop = spec.get("prop")
        if not prop:
            continue
        assert prop["type"] == "shelf", f"{name}: spawn_warehouse only knows 'shelf'"
        deck = float(prop["deck_height_m"])
        assert 0.0 < deck < ARM_REACH_M, (
            f"{name}: deck_height_m={deck} is not plausibly reachable"
        )
        sx, sy, sz = (float(v) for v in prop["size"])
        assert sz < deck, f"{name}: deck slab ({sz}) is thicker than its own height ({deck})"
        assert sx > 0 and sy > 0


def test_pickup_and_dropoff_shelves_do_not_overlap(stations):
    # Two shelves authored at the same spot would interpenetrate, and
    # _place_package_on_surface picks the FIRST deck whose footprint contains
    # the package — overlapping decks would make the release ambiguous.
    import itertools
    import yaml

    raw = yaml.safe_load((REPO_ROOT / "configs" / "waypoints.yaml").read_text())
    props = [(n, s) for n, s in raw["stations"].items() if s.get("prop")]
    for (na, sa), (nb, sb) in itertools.combinations(props, 2):
        ax, ay = sa["pose"]["x"], sa["pose"]["y"]
        bx, by = sb["pose"]["x"], sb["pose"]["y"]
        a_hx, a_hy = sa["prop"]["size"][0] / 2, sa["prop"]["size"][1] / 2
        b_hx, b_hy = sb["prop"]["size"][0] / 2, sb["prop"]["size"][1] / 2
        overlap = abs(ax - bx) < (a_hx + b_hx) and abs(ay - by) < (a_hy + b_hy)
        assert not overlap, f"{na} and {nb} shelf footprints overlap"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
