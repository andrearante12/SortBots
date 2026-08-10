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

# Summed URDF link origins Rotation -> Fixed_Jaw_tip: fully extended
# straight-line reach FROM THE SHOULDER. An upper bound, so anything usable
# has to sit comfortably inside it.
ARM_REACH_M = 0.45

# Shoulder height above the FLOOR: xlerobot.urdf's arm_base_joint puts the arm
# base 0.7600 above base_link, Rotation adds 0.0165, and the robot spawns at
# z=0.05. The arm is on a mast, not at chassis height.
#
# This distinction is the whole point of these assertions. An earlier version
# compared deck_height_m directly against ARM_REACH_M, which silently mixed
# two different reference frames — it "passed" a 0.30 m deck that in fact sat
# half a metre BELOW the shoulder, at the very edge of the envelope pointing
# straight down. Reach is measured from the shoulder, never from the floor.
ARM_SHOULDER_Z_M = 0.7765 + 0.05


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
        ptype = prop.get("type", "shelf")
        assert ptype in ("shelf", "usd"), f"{name}: unknown prop type {ptype!r}"
        if ptype == "usd":
            assert prop.get("ref"), f"{name}: type: usd needs a `ref`"

        deck = float(prop["deck_height_m"])
        # Vertical drop from the shoulder to the pick surface. Must leave real
        # horizontal reach behind, otherwise the arm is at full stretch just
        # getting DOWN to the deck and can't extend forward at all.
        drop = ARM_SHOULDER_Z_M - deck
        assert abs(drop) < ARM_REACH_M, (
            f"{name}: deck_height_m={deck} is {drop:+.2f} m from the shoulder "
            f"(z={ARM_SHOULDER_Z_M:.2f}), beyond the {ARM_REACH_M} m envelope"
        )
        horizontal = math.sqrt(max(ARM_REACH_M**2 - drop**2, 0.0))
        assert horizontal > 0.20, (
            f"{name}: deck_height_m={deck} leaves only {horizontal:.2f} m of "
            f"horizontal reach — the arm would be at full stretch reaching down"
        )
        assert horizontal > stations[name].dock_offset_m, (
            f"{name}: dock_offset_m={stations[name].dock_offset_m} exceeds the "
            f"{horizontal:.2f} m of horizontal reach available at deck height "
            f"{deck} — the base parks further away than the arm can reach"
        )

        sx, sy, sz = (float(v) for v in prop.get("size", [0.9, 0.4, 0.06]))
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
