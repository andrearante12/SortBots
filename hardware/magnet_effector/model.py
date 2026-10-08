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

    # Ø24 neck (clears the motor-5 bracket, see Horn.neck_len), then the cup.
    neck = _cyl(h.neck_d, 0, h.neck_len + 1)
    cup = (cq.Workplane("XY").workplane(offset=h.neck_len).circle(r_out)
           .extrude(p.body_len - h.neck_len).edges().fillet(p.pr.fillet))
    body = neck.union(cup).faces("<Z").edges().chamfer(0.4)
    body = body.cut(_cyl(p.bore_d, p.cavity_top, p.body_len + 1))

    # Rim notch for the magnet's side lead stub: at full compression the stub
    # rises into the cup's bottom edge (caught by the stroke test, 2026-10-05).
    mg = p.magnet
    z_stub_top = (p.body_len - mg.locate_depth - p.comp.travel
                  + mg.lead_z_from_back - mg.lead_stub_d / 2)
    body = body.cut(_box_at(mg.lead_angle, r_bore + p.pr.wall / 2, mg.lead_stub_d + 1.5,
                            p.pr.wall + 2, z_stub_top - 1.0, p.body_len + 1))

    # Zip-tie lug beside the cable exit: wires bend back toward the servo and
    # get tied here, so the solder joints on the magnet never see the flex.
    lug_a = p.cable_angle + 20
    lug = _box_at(lug_a, r_out + 2.0, 8.0, 6.0, h.neck_len, p.z_standoff)
    lug = lug.cut(_box_at(lug_a, r_out + 2.5, p.ziptie_w + 0.4, 1.8, -1, p.z_standoff + 1))
    body = body.union(lug)

    # Horn interface (same as stock): pilot recess, 4x M3 on 9.9 square, centre access.
    body = body.cut(_cyl(h.pilot_d + p.pr.clearance, -1, h.pilot_depth))
    for xy in horn_holes(p):
        body = body.cut(_cyl(h.hole_d, -1, p.floor_top + 1, xy))
        body = body.cut(_cyl(h.head_cbore_d, p.floor_top, p.cavity_top + 1, xy))
    body = body.cut(_cyl(h.center_hole_d, -1, p.cavity_top + 1))

    # Guide-pin standoffs hang from the floor and merge into the wall; the
    # plunger hard-stops on their faces at full travel.
    for a in c.pin_angles:
        xy = _polar(p.pin_circle_r, a)
        body = body.union(_cyl(p.standoff_d, p.cavity_top - 0.01, p.z_standoff, xy))
        body = body.cut(_cyl(c.spring_od + 0.5,
                             p.z_standoff - p.spring_seat_standoff, p.z_standoff + 1, xy))
        # Blind tap hole: stops 1 mm into the roof, so it never breaks through
        # the step face beside the motor-5 bracket.
        body = body.cut(_cyl(c.tap_d, h.neck_len + 1.0, p.z_standoff, xy))

    # Microswitch: flat seat cut into the bore wall (the bore is round, the
    # switch isn't), stretched +-adjust in Z; M2 slots through the outer wall
    # let the trip point be tuned on the bench, nuts inside.
    zf, zh = switch_slot_z(p)
    z_top = max(p.cavity_top, zf - s.h - s.adjust)
    body = body.cut(_box_at(s.angle, r_bore - s.w / 2 + 0.01, s.l + 2 * p.pr.clearance,
                            s.w + 0.02, z_top, zf + s.adjust))
    for t in (-s.hole_pitch / 2, s.hole_pitch / 2):
        slot = (cq.Workplane("YZ").workplane(offset=r_bore - 1)
                .center(t, zh).slot2D(2 * s.adjust + s.screw_d, s.screw_d, angle=90)
                .extrude(r_out - r_bore + 3))
        body = body.cut(slot.rotate((0, 0, 0), (0, 0, 1), s.angle))

    # Switch-wire exit, above the plunger even at full compression. (The
    # magnet's own leads never enter the cup: they leave its side under the
    # plunger and run up the outside to the lug.)
    body = body.cut(_box_at(p.cable_angle, r_bore + p.pr.wall / 2, p.cable_slot_w,
                            p.pr.wall + 2, p.cavity_top + 0.5, p.z_standoff))

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
    pl = pl.cut(_cyl(m.d + 2 * p.pr.clearance, z1 - m.locate_depth, z1 + 1))

    # Keeper: an arc of skirt around the magnet's back, slotted for the lead
    # stub. The slot is open at the bottom, so the magnet pushes up into it.
    r_in = m.d / 2 + p.pr.clearance
    r_out = r_in + m.keeper_wall
    zk = z1 + p.keeper_h
    ring = _cyl(2 * r_out, z1 - 0.01, zk).cut(_cyl(2 * r_in, z1 - 1, zk + 1))
    span = (cq.Workplane("XY").workplane(offset=z1 - 1)
            .moveTo(0, 0).lineTo(*_polar(r_out + 5, m.lead_angle - m.keeper_span / 2))
            .threePointArc(_polar(r_out + 5, m.lead_angle),
                           _polar(r_out + 5, m.lead_angle + m.keeper_span / 2))
            .close().extrude(p.keeper_h + 2))
    keeper = ring.intersect(span)
    z_stub = z1 - m.locate_depth + m.lead_z_from_back
    slot_w = m.lead_stub_d + 0.6
    keeper = keeper.cut(_box_at(m.lead_angle, r_in + m.keeper_wall / 2, slot_w,
                                m.keeper_wall + 2, z_stub - 0.01, zk + 1))
    # Round the slot's closed end so the stub seats in a cradle, not a corner.
    keeper = keeper.cut(cq.Workplane("XY").add(cq.Solid.makeCylinder(
        slot_w / 2, m.keeper_wall + 2,
        cq.Vector(*_polar(r_in - 1, m.lead_angle), z_stub),
        cq.Vector(*_polar(1, m.lead_angle), 0))))
    return pl.union(keeper)


# ---- placeholders (assembly / tests only) ----------------------------------

def magnet_parts(p: Params = DEFAULT, compressed: float = 0.0) -> dict:
    """Adafruit 3873, split into coloured pieces so it reads like the product
    photos: chrome body with the tapped hole on the back, dark potting ring
    around the Ø12 pole on the face, black heat-shrink lead stub out of the
    SIDE, and the start of the blue leads. {name: (Workplane, Color)}."""
    m = p.magnet
    z0 = p.body_len - m.locate_depth - compressed  # back face, against the plunger
    zf = z0 + m.h                                   # holding face
    body = _cyl(m.d, z0, zf).faces(">Z").edges().chamfer(0.5)
    body = body.cut(_cyl(m.potting_od, zf - 1.0, zf + 1).cut(_cyl(m.pole_d, zf - 2, zf + 2)))
    body = body.cut(_cyl(3.3, z0 - 1, z0 + m.thread_depth))
    potting = _cyl(m.potting_od, zf - 1.0, zf - 0.3).cut(_cyl(m.pole_d, zf - 2, zf + 2))

    zs = z0 + m.lead_z_from_back
    out = cq.Vector(*_polar(1, m.lead_angle), 0)
    stub = cq.Solid.makeCylinder(m.lead_stub_d / 2, m.lead_stub_len + 1,
                                 cq.Vector(*_polar(m.d / 2 - 1, m.lead_angle), zs), out)
    tang = cq.Vector(*_polar(1, m.lead_angle + 90), 0)
    tip = cq.Vector(*_polar(m.d / 2 + m.lead_stub_len, m.lead_angle), zs)
    leads = [cq.Solid.makeCylinder(0.7, 12.0, tip + tang * s, out) for s in (-0.8, 0.8)]
    return {
        "magnet_body": (cq.Workplane("XY").add(body.val()), cq.Color(0.78, 0.78, 0.8)),
        "magnet_potting": (potting, cq.Color(0.12, 0.12, 0.12)),
        "magnet_lead_stub": (cq.Workplane("XY").add(stub), cq.Color(0.05, 0.05, 0.05)),
        "magnet_leads": (cq.Workplane("XY").add(cq.Compound.makeCompound(leads)),
                         cq.Color(0.1, 0.55, 0.9)),
    }


def magnet(p: Params = DEFAULT, compressed: float = 0.0) -> cq.Workplane:
    """Solid envelope (body + lead stub) for clearance checks and tests."""
    parts = magnet_parts(p, compressed)
    env = parts["magnet_body"][0].union(parts["magnet_potting"][0])
    return env.union(parts["magnet_lead_stub"][0])


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
    max_len = p.body_len - (p.horn.neck_len + 1.0)  # tap hole bottom
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


def springs(p: Params = DEFAULT) -> cq.Workplane:
    """3 compression springs, drawn as tubes at their installed (rest) length:
    from the bottom of the standoff seat to the bottom of the plunger seat."""
    c = p.comp
    z0 = p.z_standoff - p.spring_seat_standoff
    z1 = p.z_plunger_top + c.spring_seat_plunger
    out = cq.Workplane("XY")
    for a in c.pin_angles:
        xy = _polar(p.pin_circle_r, a)
        out = out.union(_cyl(c.spring_od, z0, z1, xy).cut(_cyl(c.spring_od - 1.2, z0 - 1, z1 + 1, xy)))
    return out


def magnet_screw(p: Params = DEFAULT) -> cq.Workplane:
    """The included M4 pan head + split + flat washer, seated in the plunger
    counterbore, shank down into the magnet's back."""
    m = p.magnet
    z_seat = p.z_plunger_top + m.bolt_head_h          # counterbore floor
    washers = _cyl(9.0, z_seat - m.washer_stack, z_seat)
    head = _cyl(8.0, z_seat - m.washer_stack - 2.7, z_seat - m.washer_stack)
    length = next(L for L in (10, 12, 14, 16) if m.washer_stack + p.plunger_floor + 5 <= L)
    shank = _cyl(4.0, z_seat - m.washer_stack, z_seat - m.washer_stack + length)
    return head.union(washers).union(shank)


def horn_screws(p: Params = DEFAULT) -> cq.Workplane:
    """The 4 stock M3x6 screws: heads on the floor, through into motor 5's horn."""
    out = cq.Workplane("XY")
    for xy in horn_holes(p):
        out = out.union(_cyl(5.6, p.floor_top, p.floor_top + 2.4, xy))
        out = out.union(_cyl(3.0, p.floor_top - 6.0, p.floor_top, xy))
    return out


PRINTED = cq.Color(0.15, 0.45, 0.85)
PLUNGER = cq.Color(0.95, 0.55, 0.1)
STEEL = cq.Color(0.25, 0.25, 0.27)


def effector_assembly(p: Params = DEFAULT, horn: cq.Shape = None) -> cq.Assembly:
    """The whole end effector in OUR frame, as a tree built for FreeCAD:

        magnet_effector            origin = centre of motor 5's horn face,
         |                         +Z (FreeCAD's W) = out along the wrist
         |- printed_parts          what you 3D print
         |    |- adapter_body
         |    |- plunger
         |- magnet_Adafruit_3873   what you buy
         |    |- body, potting, lead_stub, leads
         |- hardware               screws, springs, switch
              |- m3x18_guide_screws_x3, springs_x3, m4_magnet_screw,
                 microswitch, horn_screws

    Every group shares that origin, so selecting a group and moving it along
    W pulls it straight off the wrist (an exploded view). `horn`: the arm's
    own horn screws (already in our frame) to use instead of drawn ones.
    """
    printed = cq.Assembly(name="printed_parts")
    printed.add(adapter_body(p), name="adapter_body", color=PRINTED)
    printed.add(plunger(p), name="plunger", color=PLUNGER)

    mag = cq.Assembly(name="magnet_Adafruit_3873")
    for name, (wp, col) in magnet_parts(p).items():
        mag.add(wp, name=name.replace("magnet_", ""), color=col)

    hw = cq.Assembly(name="hardware")
    hw.add(guide_screws(p), name=f"m3x{guide_screw_len(p):.0f}_guide_screws_x3", color=STEEL)
    hw.add(springs(p), name="springs_x3", color=cq.Color(0.7, 0.7, 0.72))
    hw.add(magnet_screw(p), name="m4_magnet_screw", color=STEEL)
    hw.add(switch_body(p), name="microswitch", color=cq.Color(0.08, 0.08, 0.08))
    hw.add(cq.Workplane("XY").add(horn) if horn is not None else horn_screws(p),
           name="horn_screws", color=STEEL)

    eff = cq.Assembly(name="magnet_effector")
    eff.add(printed)
    eff.add(mag)
    eff.add(hw)
    return eff


def assembly(p: Params = DEFAULT, with_reference: bool = False) -> cq.Assembly:
    a = cq.Assembly(name="magnet_effector_on_motor5")
    a.add(effector_assembly(p))
    ref = cq.Assembly(name="reference_not_part_of_design")
    ref.add(reference_servo(p), name="sts3215_motor5", color=cq.Color(0.25, 0.25, 0.25))
    if with_reference:
        ref.add(reference_wrist(), name="stock_gripper_overlay",
                color=cq.Color(0.9, 0.2, 0.2, 0.35))
    a.add(ref)
    return a
