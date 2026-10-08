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
    # Magnet incl. its side lead stub vs the cup rim, and vs its own keeper.
    mag = model.magnet(DEFAULT, compressed)
    assert _overlap(body, mag) < EPS_MM3
    assert _overlap(pl, mag) < EPS_MM3
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


def test_keeper_captures_the_lead_stub():
    # Rotate the magnet a few degrees: the stub must hit the keeper, otherwise
    # the keeper isn't actually keying the orientation.
    pl = model.plunger(DEFAULT)
    turned = model.magnet(DEFAULT).rotate((0, 0, 0), (0, 0, 1), 8.0)
    assert _overlap(pl, turned) > 0.5


def test_magnet_bolt_window_is_buyable():
    lo, hi = DEFAULT.magnet_bolt_len
    assert any(lo <= L <= hi for L in (8, 10, 12, 16))


def test_standoffs_exist(body):
    # Regression: re-cutting the bore after unioning the standoffs silently
    # deleted them (2026-10-05) — the part still built and looked fine end-on.
    for a in DEFAULT.comp.pin_angles:
        r = DEFAULT.pin_circle_r + DEFAULT.standoff_d / 2 - 0.5
        probe = model._cyl(0.5, DEFAULT.cavity_top + 0.5, DEFAULT.z_standoff - 0.5,
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
    # The magnet bolt clamps what's left between its counterbore and the
    # magnet's locating recess, so measure that, not plunger_t - head.
    floor = p.plunger_t - p.magnet.bolt_head_h - p.magnet.locate_depth
    assert floor >= 2.0


def test_guide_screw_reaches_but_never_touches_the_horn():
    p = DEFAULT
    tip = p.body_len - model.guide_screw_len(p)
    assert tip >= p.horn.neck_len + 1.0  # blind tap hole bottom
    assert p.z_standoff - tip >= 5.0  # thread engagement


@pytest.mark.parametrize("d", [20.0, 34.0])
def test_regenerates_for_other_magnets(d):
    p = dataclasses.replace(DEFAULT, magnet=dataclasses.replace(DEFAULT.magnet, d=d))
    b, pl = model.adapter_body(p), model.plunger(p)
    assert b.val().isValid() and pl.val().isValid()
    assert _overlap(b, pl) < EPS_MM3
    assert p.z_magnet_face > 0 and isinstance(p, Params)


# ---- whole-arm checks (need reference/SO101_Assembly.step, ~15 s to load) --

@pytest.fixture(scope="module")
def arm_ctx():
    import arm
    if not arm.ARM_STEP.exists():
        pytest.skip("SO101_Assembly.step not fetched; run hardware/setup_cad.sh")
    return arm.Arm()


def test_arm_registration_lands_on_motor5_horn(arm_ctx):
    # arm.register() already raises if any vertex of the stock jaw misses its
    # twin by >0.01 mm; this checks the gripper/horn/arm split on top of it.
    names = sorted(n for n, _ in arm_ctx.groups["horn"])
    assert len(names) == 5
    assert {n for n, _ in arm_ctx.removed if "Servo" in n} == {"ST3215 Servo v2:6"}


def test_clears_wrist_bracket_over_full_roll(arm_ctx):
    assert arm_ctx.roll_clearance(DEFAULT, step=30.0) == []


def test_roll_sweep_catches_a_full_width_cup(arm_ctx):
    # The sweep is only worth something if it fails on the design it was
    # written for: no neck -> the cup hits the motor-5 bracket (2026-10-05).
    p = dataclasses.replace(DEFAULT, horn=dataclasses.replace(DEFAULT.horn, neck_len=0.5))
    hits = arm_ctx.roll_clearance(p, angles=[0.0])
    assert any("Wrist_Roll_Pitch" in theirs for _, _, theirs, _ in hits)


def test_hardware_fits_the_printed_parts(body):
    # The magnet screw's shank threads INTO the magnet, so it's only checked
    # against the printed parts; springs ride in seats on both sides.
    pl = model.plunger(DEFAULT)
    for hw in (model.springs(DEFAULT), model.magnet_screw(DEFAULT), model.horn_screws(DEFAULT)):
        assert _overlap(body, hw) < EPS_MM3
        assert _overlap(pl, hw) < EPS_MM3


def test_assembly_tree_groups():
    eff = model.effector_assembly(DEFAULT)
    assert [c.name for c in eff.children] == ["printed_parts", "magnet_Adafruit_3873", "hardware"]
    assert [c.name for c in eff.children[0].children] == ["adapter_body", "plunger"]
