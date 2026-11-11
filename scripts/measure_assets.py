#!/usr/bin/env python3
"""Measure the usd assets the world presets name: bounding box on the floor,
height, and the physics the asset ships — so simtest/world.py PRESETS carry
the footprint the asset really has, not a guess.

Runs headless inside Isaac's venv (scripts/activate_isaac.sh), ~1-2 min:

    python3 scripts/measure_assets.py            # every preset
    python3 scripts/measure_assets.py person amr # some

Prints one line per asset: the composed bbox in metres (x, y extents and
z range at identity transform), how many prims carry physics APIs, and the
strip-physics count spawn_warehouse would apply. Compare against PRESETS and
fix footprint / height / z / yaw_offset_deg there.
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))


def main(argv) -> int:
    from simtest import world as wl

    names = argv or sorted(wl.PRESETS)
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        import omni.usd
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.storage.native import get_assets_root_path
        from pxr import Usd, UsdGeom, UsdPhysics

        root = get_assets_root_path()
        if not root:
            print("asset root unreachable")
            return 2
        stage = omni.usd.get_context().get_stage()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_,
                                                            UsdGeom.Tokens.render])
        for name in names:
            p = wl.PRESETS[name]
            path = f"/World/measure_{name}"
            add_reference_to_stage(usd_path=f"{root.rstrip('/')}/{p['ref']}", prim_path=path)
            app.update()
            prim = stage.GetPrimAtPath(path)
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            lo, hi = box.GetMin(), box.GetMax()
            n_phys = 0
            for q in Usd.PrimRange(prim):
                if q.HasAPI(UsdPhysics.CollisionAPI) or q.HasAPI(UsdPhysics.RigidBodyAPI) \
                        or q.HasAPI(UsdPhysics.ArticulationRootAPI) or q.IsA(UsdPhysics.Joint):
                    n_phys += 1
            declared = p.get("footprint") or (2 * p["radius"], 2 * p["radius"])
            print(f"{name:9s} {p['ref']}\n"
                  f"          bbox x {lo[0]:+.2f}..{hi[0]:+.2f} ({hi[0] - lo[0]:.2f} m)  "
                  f"y {lo[1]:+.2f}..{hi[1]:+.2f} ({hi[1] - lo[1]:.2f} m)  "
                  f"z {lo[2]:+.2f}..{hi[2]:+.2f} ({hi[2] - lo[2]:.2f} m)\n"
                  f"          preset footprint {declared[0]:.2f} x {declared[1]:.2f}, "
                  f"height {p['height']:.2f}; physics prims {n_phys}", flush=True)
            stage.RemovePrim(path)
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
