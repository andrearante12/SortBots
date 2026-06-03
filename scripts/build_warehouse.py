#!/usr/bin/env python3
"""Build the Phase 3 warehouse USD from primitives.

Authored via `pxr.Usd` after a headless `SimulationApp` boots Kit (pxr
isn't importable from the pip-installed Isaac venv without it). The
scene is gitignored; `scripts/spawn_warehouse.py` and
`tests/isaac/test_ros2_topics.py` auto-rebuild it if missing.

Run after `source scripts/activate_isaac.sh`:

    python scripts/build_warehouse.py --out scenes/warehouse_v0.usd

Exit codes: 0 OK, 1 exception. Progress is mirrored to
``$ISAAC_WAREHOUSE_BUILD_RESULT`` (default
``/tmp/isaac_warehouse_build_result.txt``).

The module also exports `SPAWN_POSITIONS`, `ZONES`, and `author_warehouse()`
so spawn / test scripts can re-use the same constants without re-parsing
the USD.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

# Sibling for emit_factory. Module-level imports stay stdlib only.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _isaac_utils import emit_factory, guard_against_ros2  # noqa: E402

RESULT_FILE = os.environ.get(
    "ISAAC_WAREHOUSE_BUILD_RESULT", "/tmp/isaac_warehouse_build_result.txt"
)


# Public constants — importable by spawn + test scripts.
SPAWN_POSITIONS: dict[str, tuple[float, float, float]] = {
    "robot_0": (-3.0, -1.0, 0.05),
    "robot_1": (-3.0, 1.0, 0.05),
}

ZONES: dict[str, tuple[float, float, float]] = {
    "pickup": (-4.0, -2.5, 0.0),
    "dropoff": (4.0, 2.5, 0.0),
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Build the Phase 3 warehouse USD.")
    p.add_argument("--out", required=True, help="Destination .usd path")
    p.add_argument("--force", action="store_true", help="Overwrite if file exists")
    return p.parse_args()


def _author_box(
    stage,
    prim_path: str,
    size: tuple[float, float, float],
    translate: tuple[float, float, float],
    collider: bool = True,
):
    """Add a UsdGeom.Cube at `prim_path` with the given size + translation.

    UsdGeom.Cube has a fixed 2-unit edge, so we use `xformOp:scale` to
    achieve arbitrary box dimensions. Optionally applies CollisionAPI.
    """
    from pxr import Gf, UsdGeom, UsdPhysics

    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.CreateSizeAttr(2.0)  # explicit; UsdGeom.Cube default
    sx, sy, sz = (s / 2.0 for s in size)
    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xform.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
    if collider:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def _author_xform(stage, prim_path: str, translate: tuple[float, float, float]):
    """Add a plain UsdGeomXform (used for zones + spawn points)."""
    from pxr import Gf, UsdGeom

    xform = UsdGeom.Xform.Define(stage, prim_path)
    op = UsdGeom.Xformable(xform).AddTranslateOp()
    op.Set(Gf.Vec3d(*translate))
    return xform


def author_warehouse(stage) -> None:
    """Author the full warehouse hierarchy on the given stage.

    Reusable: callable on an existing stage (e.g. from spawn_warehouse.py
    if you ever want to inline the geometry instead of referencing the saved
    USD).
    """
    from pxr import Gf, Sdf, UsdGeom, UsdLux

    # Stage metadata: Z-up, meters.
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # Lights.
    UsdGeom.Xform.Define(stage, "/World/Lights")
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
    dome.CreateIntensityAttr(800.0)
    distant = UsdLux.DistantLight.Define(stage, "/World/Lights/DistantLight")
    distant.CreateIntensityAttr(2500.0)
    distant_xform = UsdGeom.Xformable(distant)
    distant_xform.AddRotateXYZOp().Set(Gf.Vec3f(45.0, 0.0, 30.0))

    # Floor — 12 x 8 x 0.05, centered on origin, top surface at z=0.
    _author_box(
        stage,
        "/World/Floor",
        size=(12.0, 8.0, 0.05),
        translate=(0.0, 0.0, -0.025),
    )

    # Walls — 1.5 m tall, 0.2 m thick. Box bottoms at z=0, tops at z=1.5.
    UsdGeom.Xform.Define(stage, "/World/Walls")
    walls = {
        "north": ((12.0, 0.2, 1.5), (0.0, 4.0, 0.75)),
        "south": ((12.0, 0.2, 1.5), (0.0, -4.0, 0.75)),
        "east": ((0.2, 8.0, 1.5), (6.0, 0.0, 0.75)),
        "west": ((0.2, 8.0, 1.5), (-6.0, 0.0, 0.75)),
    }
    for name, (size, translate) in walls.items():
        _author_box(stage, f"/World/Walls/{name}", size=size, translate=translate)

    # Obstacles — shelves (4) and small boxes (2). Placed to leave a clear
    # forward corridor for both spawn points so the circle-drive demo runs
    # without immediate collisions.
    UsdGeom.Xform.Define(stage, "/World/Obstacles")
    shelves = {
        "shelf_0": ((1.0, 0.4, 1.2), (1.0, 0.0, 0.6)),
        "shelf_1": ((1.0, 0.4, 1.2), (2.5, 2.0, 0.6)),
        "shelf_2": ((1.0, 0.4, 1.2), (-1.5, 2.5, 0.6)),
        "shelf_3": ((1.0, 0.4, 1.2), (3.5, -2.0, 0.6)),
    }
    boxes = {
        "box_0": ((0.4, 0.4, 0.4), (0.5, -1.8, 0.2)),
        "box_1": ((0.4, 0.4, 0.4), (-2.0, -2.8, 0.2)),
    }
    for name, (size, translate) in {**shelves, **boxes}.items():
        _author_box(stage, f"/World/Obstacles/{name}", size=size, translate=translate)

    # Named pickup + dropoff zone Xforms with a `sortbots:zone` token attr.
    for zone, pos in ZONES.items():
        prim_name = "PickupZone" if zone == "pickup" else "DropoffZone"
        xform = _author_xform(stage, f"/World/{prim_name}", pos)
        attr = xform.GetPrim().CreateAttribute(
            "sortbots:zone", Sdf.ValueTypeNames.Token, custom=True
        )
        attr.Set(zone)

    # Spawn points for each robot.
    UsdGeom.Xform.Define(stage, "/World/SpawnPoints")
    for robot, pos in SPAWN_POSITIONS.items():
        _author_xform(stage, f"/World/SpawnPoints/{robot}", pos)


def main() -> int:
    args = parse_args()
    out_path = Path(args.out).resolve()

    emit = emit_factory(RESULT_FILE)
    if out_path.exists() and not args.force:
        emit(f"build_warehouse: {out_path} already exists; use --force to overwrite")
        emit("build_warehouse: SKIPPED")
        return 0

    from pxr import Usd

    emit("build_warehouse v0")
    emit(f"  out={out_path}")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(out_path))
    author_warehouse(stage)
    stage.GetRootLayer().Save()

    emit(f"  spawn_points={list(SPAWN_POSITIONS)}")
    emit(f"  zones={list(ZONES)}")
    emit(f"  wrote {out_path}")
    emit("build_warehouse: OK")
    return 0


if __name__ == "__main__":
    guard_against_ros2()

    # SimulationApp MUST be constructed before any other omni / isaacsim / pxr
    # imports — the pip-installed Isaac venv exposes `pxr` only after Kit loads.
    from isaacsim import SimulationApp  # noqa: E402

    simulation_app = SimulationApp({"headless": True})
    try:
        rc = main()
    except Exception as e:
        print(f"ERROR: {type(e).__name__}: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        rc = 1
    finally:
        simulation_app.close()
    sys.exit(rc)
