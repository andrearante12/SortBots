"""CadQuery model of the SO-101 electromagnet end effector.

Two printed parts:
  adapter_body(p) - inverted cup: horn interface on top, guide-pin standoffs,
                    microswitch pocket, cable exit + zip-tie lug
  plunger(p)      - disc the magnet bolts to; slides in the cup bore on the
                    guide screws, springs push it toward the magnet face

Everything is in the frame described in params.py (z=0 on the horn face, +Z
toward the magnet). Placeholders (magnet, screws, switch, servo) exist only
for the assembly and the interference tests — they are never printed.
"""
import math
from pathlib import Path

import cadquery as cq

from params import DEFAULT, Params

REF = Path(__file__).resolve().parent / "reference"


def _polar(r: float, deg: float) -> tuple:
    a = math.radians(deg)
    return (r * math.cos(a), r * math.sin(a))


def _cyl(d: float, z0: float, z1: float, xy=(0.0, 0.0)) -> cq.Workplane:
    """Solid cylinder from z0 to z1 (z1 > z0)."""
    return (cq.Workplane("XY").workplane(offset=z0)
            .center(*xy).circle(d / 2).extrude(z1 - z0))


def _box_at(angle: float, r_center: float, tang: float, rad: float,
            z0: float, z1: float) -> cq.Workplane:
    """Box with its radial axis along `angle`, centred at radius r_center."""
    b = (cq.Workplane("XY").workplane(offset=z0)
         .center(r_center, 0).rect(rad, tang).extrude(z1 - z0))
    return b.rotate((0, 0, 0), (0, 0, 1), angle)


def horn_holes(p: Params = DEFAULT) -> list:
    s = p.horn.hole_square / 2
    return [(sx * s, sy * s) for sx in (-1, 1) for sy in (-1, 1)]


def switch_slot_z(p: Params = DEFAULT) -> tuple:
    """(z of the actuator-side face, z of the mounting holes) at slot centre."""
    zf = p.z_switch_body
    return zf, zf - p.switch.hole_from_act


def adapter_body(p: Params = DEFAULT) -> cq.Workplane:
    h, c, s = p.horn, p.comp, p.switch
    r_bore = p.bore_d / 2
    r_out = p.body_d / 2

    body = (cq.Workplane("XY").circle(r_out).extrude(p.body_len)
            .edges().fillet(p.pr.fillet))
    body = body.cut(_cyl(p.bore_d, p.floor_top, p.body_len + 1))

    # Zip-tie lug beside the cable exit: wires bend back toward the servo and
    # get tied here, so the solder joints on the magnet never see the flex.
    lug_a = p.cable_angle + 20
    lug = _box_at(lug_a, r_out + 2.0, 8.0, 6.0, 0, p.z_standoff)
    lug = lug.cut(_box_at(lug_a, r_out + 2.5, p.ziptie_w + 0.4, 1.8, -1, p.z_standoff + 1))
    body = body.union(lug)

    # Horn interface (same as stock): pilot recess, 4x M3 on 9.9 square, centre access.
    body = body.cut(_cyl(h.pilot_d + p.pr.clearance, -1, h.pilot_depth))
    for xy in horn_holes(p):
        body = body.cut(_cyl(h.hole_d, -1, p.floor_top + 1, xy))
    body = body.cut(_cyl(h.center_hole_d, -1, p.floor_top + 1))

    # Guide-pin standoffs hang from the floor and merge into the wall; the
    # plunger hard-stops on their faces at full travel.
    for a in c.pin_angles:
        xy = _polar(p.pin_circle_r, a)
        body = body.union(_cyl(p.standoff_d, p.floor_top - 0.01, p.z_standoff, xy))
        body = body.cut(_cyl(c.spring_od + 0.5,
                             p.z_standoff - p.spring_seat_standoff, p.z_standoff + 1, xy))
        # Blind tap hole: stops 1 mm short of the horn face so no screw tip
        # can ever touch the servo horn.
        body = body.cut(_cyl(c.tap_d, 1.0, p.z_standoff, xy))

    # Microswitch: flat seat cut into the bore wall (the bore is round, the
    # switch isn't), stretched +-adjust in Z; M2 slots through the outer wall
    # let the trip point be tuned on the bench, nuts inside.
    zf, zh = switch_slot_z(p)
    z_top = max(p.floor_top, zf - s.h - s.adjust)
    body = body.cut(_box_at(s.angle, r_bore - s.w / 2 + 0.01, s.l + 2 * p.pr.clearance,
                            s.w + 0.02, z_top, zf + s.adjust))
    for t in (-s.hole_pitch / 2, s.hole_pitch / 2):
        slot = (cq.Workplane("YZ").workplane(offset=r_bore - 1)
                .center(t, zh).slot2D(2 * s.adjust + s.screw_d, s.screw_d, angle=90)
                .extrude(r_out - r_bore + 3))
        body = body.cut(slot.rotate((0, 0, 0), (0, 0, 1), s.angle))

    # Cable exit: above the plunger even at full compression.
    body = body.cut(_box_at(p.cable_angle, r_bore + p.pr.wall / 2, p.cable_slot_w,
                            p.pr.wall + 2, p.floor_top + 0.8, p.z_standoff))

    return body


def plunger(p: Params = DEFAULT, compressed: float = 0.0) -> cq.Workplane:
    """`compressed` (0..travel) shifts it toward the servo, for the tests/assembly."""
    c, m = p.comp, p.magnet
    z0 = p.z_plunger_top - compressed
    z1 = z0 + p.plunger_t

    pl = _cyl(p.plunger_d, z0, z1).edges("<Z or >Z").chamfer(0.4)
    for a in c.pin_angles:
        xy = _polar(p.pin_circle_r, a)
        pl = pl.cut(_cyl(c.pin_d + c.pin_slide_clear, z0 - 1, z1 + 1, xy))
        pl = pl.cut(_cyl(c.spring_od + 0.5, z0 - 1, z0 + c.spring_seat_plunger, xy))
    # Magnet bolt goes in from the top; head counterbored flush.
    pl = pl.cut(_cyl(m.bolt_clear_d, z0 - 1, z1 + 1))
    pl = pl.cut(_cyl(m.bolt_head_d, z0 - 1, z0 + m.bolt_head_h))
    pl = pl.cut(_cyl(m.lead_hole_d, z0 - 1, z1 + 1, _polar(m.lead_r, m.lead_angle)))
    pl = pl.cut(_cyl(m.d + 2 * p.pr.clearance, z1 - m.locate_depth, z1 + 1))
    return pl


# ---- placeholders (assembly / tests only) ----------------------------------

def magnet(p: Params = DEFAULT, compressed: float = 0.0) -> cq.Workplane:
    m = p.magnet
    z0 = p.body_len - m.locate_depth - compressed
    return _cyl(m.d, z0, z0 + m.h).faces(">Z").edges().chamfer(0.5)


def guide_screws(p: Params = DEFAULT, compressed: float = 0.0) -> cq.Workplane:
    """M3 screws: head under the plunger, shank up into the standoff tap hole."""
    c = p.comp
    out = cq.Workplane("XY")
    zb = p.body_len  # heads are the extend stop, so they don't move with the plunger
    for a in c.pin_angles:
        xy = _polar(p.pin_circle_r, a)
        out = out.union(_cyl(c.pin_head_d, zb, zb + 3.0, xy))
        out = out.union(_cyl(c.pin_d, zb - guide_screw_len(p), zb, xy))
    return out


def guide_screw_len(p: Params = DEFAULT) -> float:
    """Shortest stock M3 length that gives >=5 mm of thread in the standoff."""
    need = (p.body_len - p.z_standoff) + 5.0
    max_len = p.body_len - 1.0  # tap hole stops at z=1
    for L in (8, 10, 12, 14, 16, 18, 20, 25, 30):
        if need <= L <= max_len:
            return float(L)
    return need


def switch_body(p: Params = DEFAULT, actuator: bool = True) -> cq.Workplane:
    """Switch at slot centre; actuator tip drawn at the operating point."""
    s = p.switch
    zf, _ = switch_slot_z(p)
    r_bore = p.bore_d / 2
    box = _box_at(s.angle, r_bore - s.w / 2, s.l, s.w, zf - s.h, zf)
    if not actuator:
        return box
    return box.union(_cyl(1.6, zf, zf + s.op, _polar(r_bore - s.w / 2, s.angle)))


def reference_wrist() -> cq.Workplane:
    """Stock follower wrist/fixed jaw, shifted into our frame (its roll axis is
    at y=-0.22 and its horn face at z=-0.05 in the upstream file)."""
    w = cq.importers.importStep(str(REF / "Wrist_Roll_Follower_SO101.step"))
    return w.translate((0, 0.22, 0.05))


def reference_servo(p: Params = DEFAULT) -> cq.Workplane:
    """Motor 5 (STS3215): horn disc face (z=18.7 in its file, shaft at x=12.5)
    seated on the pilot-recess floor."""
    s = cq.importers.importStep(str(REF / "STS3215_03a.step"))
    return s.translate((-12.5, 0, p.horn.pilot_depth - 18.7))


def assembly(p: Params = DEFAULT, with_reference: bool = False) -> cq.Assembly:
    a = cq.Assembly(name="magnet_effector")
    a.add(adapter_body(p), name="adapter_body", color=cq.Color(0.15, 0.45, 0.85))
    a.add(plunger(p), name="plunger", color=cq.Color(0.95, 0.55, 0.1))
    a.add(magnet(p), name="magnet_placeholder", color=cq.Color(0.6, 0.6, 0.6))
    a.add(guide_screws(p), name="m3_guide_screws", color=cq.Color(0.2, 0.2, 0.2))
    a.add(switch_body(p), name="switch_placeholder", color=cq.Color(0.1, 0.1, 0.1))
    a.add(reference_servo(p), name="sts3215_motor5", color=cq.Color(0.25, 0.25, 0.25))
    if with_reference:
        a.add(reference_wrist(), name="stock_wrist_reference",
              color=cq.Color(0.9, 0.2, 0.2, 0.35))
    return a
