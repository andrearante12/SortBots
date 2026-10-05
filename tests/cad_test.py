#!/usr/bin/env python3
"""Geometry checks for hardware/magnet_effector (SO-101 electromagnet end effector).

Needs CadQuery, which lives only in hardware/.venv — the system python3 that
CI and `make test` use doesn't have it, so this whole module skips there.

    hardware/.venv/bin/python -m pytest tests/cad_test.py      (or: make cad-test)
"""
from __future__ import annotations

import dataclasses
import math
import sys
from pathlib import Path

import pytest

cq = pytest.importorskip("cadquery")

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "hardware" / "magnet_effector"))

from OCP.BRepAdaptor import BRepAdaptor_Surface  # noqa: E402

import model  # noqa: E402
from params import DEFAULT, Params  # noqa: E402

# Overlap below this is OCC boolean noise on touching faces, not interference.
EPS_MM3 = 1e-3


def _overlap(a: cq.Workplane, b: cq.Workplane) -> float:
    return a.val().intersect(b.val()).Volume()


@pytest.fixture(scope="module")
def body():
    return model.adapter_body(DEFAULT)


def test_printed_parts_are_single_valid_solids(body):
    for wp in (body, model.plunger(DEFAULT)):
        assert wp.val().isValid()
        assert len(wp.val().Solids()) == 1


@pytest.mark.parametrize("compressed", [0.0, DEFAULT.comp.travel])
def test_no_interference_over_the_stroke(body, compressed):
    pl = model.plunger(DEFAULT, compressed)
    assert _overlap(body, pl) < EPS_MM3
    assert _overlap(model.guide_screws(DEFAULT), pl) < EPS_MM3
    # The actuator is SUPPOSED to be pushed; the switch body is not.
    assert _overlap(model.switch_body(DEFAULT, actuator=False), pl) < EPS_MM3


def test_switch_fits_its_pocket(body):
    assert _overlap(model.switch_body(DEFAULT), body) < EPS_MM3


def test_switch_survives_the_hard_stop():
    s = DEFAULT.switch
    assert s.ot < s.op, "overtravel can't exceed the actuator's own height"
    assert DEFAULT.switch_ot_margin >= 0, (
        "plunger hard-stops past the switch's overtravel: raise trip or shorten travel")


def test_standoffs_exist(body):
    # Regression: re-cutting the bore after unioning the standoffs silently
    # deleted them (2026-10-05) — the part still built and looked fine end-on.
    for a in DEFAULT.comp.pin_angles:
        r = DEFAULT.pin_circle_r + DEFAULT.standoff_d / 2 - 0.5
        probe = model._cyl(0.5, DEFAULT.floor_top + 0.5, DEFAULT.z_standoff - 0.5,
                           model._polar(r, a))
        assert _overlap(body, probe) > 0.5 * probe.val().Volume()


def _z_holes(shape, radius: float) -> set:
    out = set()
    for f in shape.Faces():
        if f.geomType() != "CYLINDER":
            continue
        # BRepAdaptor, not f._geomAdaptor(): boolean results come back as
        # trimmed surfaces, which have no .Cylinder().
        cyl = BRepAdaptor_Surface(f.wrapped).Cylinder()
        if abs(cyl.Radius() - radius) > 0.01 or abs(abs(cyl.Axis().Direction().Z()) - 1) > 1e-6:
            continue
        loc = cyl.Axis().Location()
        out.add((round(loc.X(), 2), round(loc.Y(), 2)))
    return out


def test_horn_pattern_matches_stock_part(body):
    """Measured live from the upstream STEP, so an upstream change can't drift past us."""
    r = DEFAULT.horn.hole_d / 2
    stock = _z_holes(model.reference_wrist().val(), r)
    ours = _z_holes(body.val(), r)
    assert len(stock) == 4
    assert ours == stock


def test_walls(body):
    p = DEFAULT
    assert (p.body_d - p.bore_d) / 2 >= p.pr.wall - 1e-9
    # The flat switch seat eats into the round wall at its corners.
    corner = math.hypot(p.bore_d / 2, p.switch.l / 2 + p.pr.clearance)
    assert p.body_d / 2 - corner >= 1.0
    # The magnet bolt counterbore must leave material under the head.
    assert p.plunger_t - p.magnet.bolt_head_h >= 1.5


def test_guide_screw_reaches_but_never_touches_the_horn():
    p = DEFAULT
    tip = p.body_len - model.guide_screw_len(p)
    assert tip >= 1.0          # blind tap hole bottom
    assert p.z_standoff - tip >= 5.0  # thread engagement


@pytest.mark.parametrize("d", [20.0, 34.0])
def test_regenerates_for_other_magnets(d):
    p = dataclasses.replace(DEFAULT, magnet=dataclasses.replace(DEFAULT.magnet, d=d))
    b, pl = model.adapter_body(p), model.plunger(p)
    assert b.val().isValid() and pl.val().isValid()
    assert _overlap(b, pl) < EPS_MM3
    assert p.z_magnet_face > 0 and isinstance(p, Params)
