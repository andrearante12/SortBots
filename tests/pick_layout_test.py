"""scripts/pick_layout.py: the pick-world authoring tool. Offline.

    /usr/bin/python3 -m pytest tests/pick_layout_test.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))
sys.path.insert(0, str(REPO_ROOT / "nodes"))
sys.path.insert(0, str(REPO_ROOT))

import pick_layout as pl  # noqa: E402
from simtest import magnet, world  # noqa: E402

needs_urdf = pytest.mark.skipif(not (REPO_ROOT / "third_party" / "XLeRobot").joinpath(
    "simulation", "Maniskill", "assets", "xlerobot", "xlerobot.urdf").exists(),
    reason="third_party/XLeRobot submodule not checked out")


@pytest.mark.parametrize("deck,normal,lateral,want", [
    ((-8.0, 10.0, 0.5, 0.5, 0.80), 90, 0.0, (-8.0, 10.15, math.pi / 2)),     # pick_orientations pkg_0
    ((-9.3, 12.4, 0.5, 0.5, 0.80), 0, 0.0, (-9.15, 12.4, 0.0)),               # ... pkg_1
    ((-7.9, 9.8, 0.7, 0.4, 0.80), 90, 0.0, (-7.9, 9.9, math.pi / 2)),         # pick_bench
])
def test_package_on_deck_reproduces_the_committed_world_files(deck, normal, lateral, want):
    p = pl.package_on_deck(deck, normal, lateral=lateral)
    assert (p["x"], p["y"], p["yaw"]) == pytest.approx(want, abs=1e-3)
    assert p["z"] == deck[4]


def test_the_package_hangs_the_asked_overhang_past_the_edge():
    deck = (0.0, 0.0, 0.6, 0.4, 0.8)
    for normal in (0, 90, 180, 270):
        p = pl.package_on_deck(deck, normal, overhang=0.02)
        centre, n = magnet.magnet_face_pose(p, pl.SIZE, "+x")
        along = centre[0] * n[0] + centre[1] * n[1]                 # plate's distance from the deck centre
        half = abs(n[0]) * 0.3 + abs(n[1]) * 0.2
        assert along == pytest.approx(half + 0.02, abs=1e-9)


def test_lateral_slides_along_the_edge_not_off_it():
    deck = (0.0, 0.0, 0.6, 0.6, 0.8)
    a, b = pl.package_on_deck(deck, 90), pl.package_on_deck(deck, 90, lateral=0.1)
    assert (b["x"] - a["x"], b["y"] - a["y"]) == pytest.approx((-0.1, 0.0), abs=1e-9)   # +y plate: along -x


def test_dock_for_puts_the_staging_point_out_along_the_normal():
    p = pl.package_on_deck((-8.0, 10.0, 0.5, 0.5, 0.8), 90)
    d = pl.dock_for(p)
    assert (d["stage"][0] - d["dock"][0], d["stage"][1] - d["dock"][1]) == pytest.approx((0.0, pl.STAGE_M))
    base_plate = magnet.to_base(d["dock"], d["plate"])
    assert base_plate[0] == pytest.approx(magnet.SHOULDER_XYZ_BASE[0] + magnet.REACH_NOM_M)


@needs_urdf
@pytest.mark.parametrize("name", ["pick_bench", "pick_orientations", "pick_heights"])
def test_the_committed_pick_worlds_check_clean(name):
    assert pl.check_world(name, None) == []


@needs_urdf
def test_a_plate_below_the_arm_is_flagged():
    problems = pl.check_world("pick_low", None)
    assert any("no arm solution" in p for p in problems)


@needs_urdf
def test_cli_prints_the_world_line_and_the_dock(capsys):
    rc = pl.main(["package", "--deck", "-8", "10", "0.5", "0.5", "0.80", "--normal", "90"])
    out = capsys.readouterr().out
    assert rc == 0
    assert 'pose: [-8.000, 10.150, 1.5708], z: 0.8, magnet_face: "+x"' in out
    assert "dock    x -7.867  y 10.450" in out and "reach   ok" in out


@needs_urdf
def test_cli_exits_nonzero_for_an_unreachable_plate(capsys):
    assert pl.main(["package", "--deck", "-8", "10", "0.5", "0.5", "0.30", "--normal", "90"]) == 1
    assert "NO arm solution" in capsys.readouterr().out


def test_check_on_a_world_without_packages_says_so():
    assert pl.check_world("aisle_crossing", None) == ["aisle_crossing: no packages"]
