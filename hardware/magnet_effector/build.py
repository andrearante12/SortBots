"""Regenerate every output from params.py.

    hardware/.venv/bin/python hardware/magnet_effector/build.py   (or: make cad)

build/adapter_body.{step,stl}, build/plunger.{step,stl}  - printable parts
build/assembly.step  - parts + placeholders + motor 5, at rest
build/fitcheck.step  - same plus the stock wrist overlaid (translucent red)
"""
from pathlib import Path

import cadquery as cq

import model
from params import DEFAULT as p

OUT = Path(__file__).resolve().parent / "build"


def _mass_g(shape: cq.Workplane, infill: bool = True) -> float:
    vol = sum(s.Volume() for s in shape.val().Solids())
    f = p.pr.infill_factor if infill else 1.0
    return vol / 1000 * p.pr.density_g_cm3 * f


def main() -> None:
    OUT.mkdir(exist_ok=True)
    parts = {"adapter_body": model.adapter_body(p), "plunger": model.plunger(p)}
    for name, wp in parts.items():
        cq.exporters.export(wp, str(OUT / f"{name}.step"))
        # Exported in the assembly frame; orient for printing in the slicer (README).
        cq.exporters.export(wp, str(OUT / f"{name}.stl"), tolerance=0.01, angularTolerance=0.1)
    model.assembly(p).export(str(OUT / "assembly.step"))
    model.assembly(p, with_reference=True).export(str(OUT / "fitcheck.step"))

    added = sum(_mass_g(w) for w in parts.values()) + p.magnet.mass_g
    stock = _mass_g(model.reference_wrist()) + _mass_g(cq.importers.importStep(
        str(model.REF / "Moving_Jaw_SO101.step"))) + p.removed_servo_g

    print(f"wrote {', '.join(sorted(f.name for f in OUT.iterdir()))} -> {OUT}")
    print(f"body  {p.body_d:.1f} dia x {p.body_len:.1f} mm   plunger {p.plunger_d:.1f} x {p.plunger_t:.1f}")
    print(f"TCP (magnet face) at rest: z = {p.z_magnet_face:.1f} mm from the horn face "
          f"(-{p.comp.travel:.1f} when fully seated)  <- URDF tcp joint origin")
    print(f"mass  new ~{added:.0f} g (magnet {p.magnet.mass_g:.0f} g, PETG @{p.pr.infill_factor:.0%})"
          f"  vs stock removed ~{stock:.0f} g (wrist+jaw @same infill, servo {p.removed_servo_g:.0f} g)")
    print(f"switch trips at {p.switch.trip:.1f} of {p.comp.travel:.1f} mm travel; "
          f"overtravel margin {p.switch_ot_margin:+.2f} mm" + ("  !! NEGATIVE" if p.switch_ot_margin < 0 else ""))
    print(f"BOM: 3x M3x{model.guide_screw_len(p):.0f} SHCS (guide pins), 3x spring "
          f"{p.comp.spring_od:.1f} OD x {p.comp.spring_free_len:.0f} free, "
          f"4x M3 horn screws (reuse stock), 1x {p.magnet.tap}x{p.plunger_t - p.magnet.bolt_head_h + 6:.0f} magnet bolt, "
          f"2x M2 + nuts (switch)")


if __name__ == "__main__":
    main()
