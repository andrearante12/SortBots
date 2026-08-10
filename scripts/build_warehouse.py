#!/usr/bin/env python3
"""Build the SortBots warehouse USD from primitives + PolyHaven textures.

Authored via `pxr.Usd` after a headless `SimulationApp` boots Kit (pxr
isn't importable from the pip-installed Isaac venv without it).

Run after `source scripts/activate_isaac.sh`:

    python scripts/fetch_warehouse_textures.py   # one-time, ~3 MB
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

import yaml

# Sibling for emit_factory. Module-level imports stay stdlib-or-pyyaml only —
# no pxr/omni here, so --help and doc/test introspection work without the
# Isaac venv.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _isaac_utils import emit_factory, guard_against_ros2  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
TEXTURES_DIR = REPO_ROOT / "assets" / "textures"
ROBOTS_CONFIG = REPO_ROOT / "configs" / "robots.yaml"

RESULT_FILE = os.environ.get(
    "ISAAC_WAREHOUSE_BUILD_RESULT", "/tmp/isaac_warehouse_build_result.txt"
)


def _load_spawn_positions(scene: str) -> dict[str, tuple[float, float, float]]:
    """Fleet roster spawn poses for `scene`, in configs/robots.yaml order.

    configs/robots.yaml is the single source of truth for robot ids + spawn
    poses, shared with spawn_warehouse.py (actual robot placement) and
    launch/sortbots_bringup.launch.py (static map -> <id>/map anchors, which
    must match the spawn pose for RTAB-Map's per-robot frame to align with
    the merged /map).
    """
    with open(ROBOTS_CONFIG) as f:
        roster = yaml.safe_load(f)
    return {r["id"]: tuple(r["spawn"][scene]) for r in roster["robots"]}


# Public constant — importable by spawn + test scripts.
SPAWN_POSITIONS: dict[str, tuple[float, float, float]] = _load_spawn_positions("primitive")

ZONES: dict[str, tuple[float, float, float]] = {
    "pickup": (-4.0, -2.5, 0.0),
    "dropoff": (4.0, 2.5, 0.0),
}

# PolyHaven assets fetched by `scripts/fetch_warehouse_textures.py`.
# Roles → texture file mapping.
TEXTURE_ROLES = {
    "floor":   "concrete_floor_painted_diff_1k.jpg",
    "walls":   "red_brick_03_diff_1k.jpg",
    "shelves": "rusty_metal_03_diff_1k.jpg",
    "boxes":   "weathered_brown_planks_diff_1k.jpg",
    "pallets": "wood_planks_grey_diff_1k.jpg",
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Build the SortBots warehouse USD.")
    p.add_argument("--out", required=True, help="Destination .usd path")
    p.add_argument("--force", action="store_true", help="Overwrite if file exists")
    return p.parse_args()


# ------------------------------------------------------------------- geometry

def _author_box(
    stage,
    prim_path: str,
    size: tuple[float, float, float],
    translate: tuple[float, float, float],
    rotate_xyz: tuple[float, float, float] | None = None,
    collider: bool = True,
):
    """Add a UV-mapped box `UsdGeom.Mesh` at `translate`.

    Built from a unit cube (`±0.5`) scaled by `size` (so face UVs map to
    [0,1] per face — texture tiling stays sane regardless of dimensions).
    `rotate_xyz` is Euler degrees (XYZ order). Optionally applies
    `UsdPhysics.CollisionAPI`.

    UsdGeom.Cube has no UV primvars and Kit's RTX render delegate does
    not auto-generate them, so the previous `Cube`-based geometry showed
    up as flat gray. A Mesh with explicit face-varying `st` is the only
    portable way to texture box-shaped surfaces here.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, Vt

    mesh = UsdGeom.Mesh.Define(stage, prim_path)

    # 8 corners of a unit cube ±0.5.
    pts = [
        Gf.Vec3f(-0.5, -0.5, -0.5),  # 0
        Gf.Vec3f( 0.5, -0.5, -0.5),  # 1
        Gf.Vec3f( 0.5,  0.5, -0.5),  # 2
        Gf.Vec3f(-0.5,  0.5, -0.5),  # 3
        Gf.Vec3f(-0.5, -0.5,  0.5),  # 4
        Gf.Vec3f( 0.5, -0.5,  0.5),  # 5
        Gf.Vec3f( 0.5,  0.5,  0.5),  # 6
        Gf.Vec3f(-0.5,  0.5,  0.5),  # 7
    ]
    # 6 quad faces, CCW from outside.
    face_indices = [
        4, 5, 6, 7,  # +Z top
        0, 3, 2, 1,  # -Z bottom
        3, 7, 6, 2,  # +Y
        0, 1, 5, 4,  # -Y
        1, 2, 6, 5,  # +X
        0, 4, 7, 3,  # -X
    ]
    face_counts = [4] * 6
    # Per-face UVs — each face uses the full [0,1]^2 patch.
    uv = [
        (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0),
    ] * 6

    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(face_indices))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(face_counts))
    primvars = UsdGeom.PrimvarsAPI(mesh)
    st_primvar = primvars.CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.faceVarying,
    )
    st_primvar.Set(Vt.Vec2fArray([Gf.Vec2f(*uv_) for uv_ in uv]))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)

    sx, sy, sz = size
    xform = UsdGeom.Xformable(mesh)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if rotate_xyz is not None:
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotate_xyz))
    xform.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))

    if collider:
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return mesh.GetPrim()


def _author_cylinder(
    stage,
    prim_path: str,
    radius: float,
    height: float,
    translate: tuple[float, float, float],
    collider: bool = True,
):
    """Add a UsdGeom.Cylinder (Z axis) at `translate`."""
    from pxr import Gf, UsdGeom, UsdPhysics

    cyl = UsdGeom.Cylinder.Define(stage, prim_path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr("Z")
    xform = UsdGeom.Xformable(cyl)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if collider:
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    return cyl.GetPrim()


def _author_xform(stage, prim_path: str, translate: tuple[float, float, float]):
    """Add a plain UsdGeom.Xform (for zones + spawn-point markers)."""
    from pxr import Gf, UsdGeom

    xform = UsdGeom.Xform.Define(stage, prim_path)
    op = UsdGeom.Xformable(xform).AddTranslateOp()
    op.Set(Gf.Vec3d(*translate))
    return xform


# ------------------------------------------------------------------- materials

_OMNI_PBR_MDL = (
    "/home/andre/isaacsim/venv/lib/python3.11/site-packages/isaacsim/kit/mdl/core/Base/"
    "OmniPBR.mdl"
)


def _mdl_shader(stage, material_path: str):
    """Create a UsdShade.Material + Shader pair wired up for OmniPBR.mdl.

    Authored directly via pxr (no `omni.kit.commands.execute`) so it lands
    on the standalone stage we're saving. UsdPreviewSurface does NOT
    render textures in Kit's RTX path on Isaac Sim 5.1 — surfaces show
    up as flat gray. OmniPBR is Kit's native MDL shader and is the
    canonical path for textured surfaces here. The USD layout matches
    what `omni.kit.commands.execute("CreateMdlMaterialPrim", ...)`
    would produce against an in-Kit stage.
    """
    from pxr import Sdf, UsdShade

    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.SetSourceAsset(Sdf.AssetPath(_OMNI_PBR_MDL), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    shader.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
    # Wire shader.outputs:out (token) → material.outputs:mdl:surface.
    shader_out = shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput("mdl").ConnectToSource(shader_out)
    material.CreateDisplacementOutput("mdl").ConnectToSource(shader_out)
    material.CreateVolumeOutput("mdl").ConnectToSource(shader_out)
    return material, shader


def _make_texture_material(
    stage,
    material_path: str,
    texture_file: Path,
):
    """Create an OmniPBR MDL material with a diffuse texture parameter."""
    from pxr import Sdf

    material, shader = _mdl_shader(stage, material_path)
    shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(str(texture_file))
    )
    shader.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(False)
    shader.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set((1.0, 1.0))
    return material


def _make_color_material(stage, material_path: str, rgb: tuple[float, float, float]):
    """Create an OmniPBR MDL material with a flat (slightly emissive) color."""
    from pxr import Gf, Sdf

    material, shader = _mdl_shader(stage, material_path)
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*rgb)
    )
    shader.CreateInput("emissive_color", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(rgb[0] * 0.4, rgb[1] * 0.4, rgb[2] * 0.4)
    )
    shader.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(True)
    return material


def _bind(prim, material):
    """Bind a material to a prim via UsdShade.MaterialBindingAPI."""
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


# ------------------------------------------------------------------- authoring

def author_warehouse(stage) -> None:
    """Author the full warehouse hierarchy on the given stage.

    Reusable: callable on an existing stage if you'd rather inline the
    geometry than reference the saved USD.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdLux

    # Stage metadata: Z-up, meters.
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # Materials root.
    UsdGeom.Scope.Define(stage, "/World/Looks")

    materials: dict[str, "UsdShade.Material"] = {}
    for role, fname in TEXTURE_ROLES.items():
        texture_path = TEXTURES_DIR / fname
        if not texture_path.is_file():
            print(
                f"WARNING: texture missing — {texture_path}. "
                "Run scripts/fetch_warehouse_textures.py first.",
                file=sys.stderr,
            )
        materials[role] = _make_texture_material(
            stage, f"/World/Looks/mat_{role}", texture_path
        )

    materials["pickup_marker"] = _make_color_material(
        stage, "/World/Looks/mat_pickup", (0.1, 0.8, 0.2)
    )
    materials["dropoff_marker"] = _make_color_material(
        stage, "/World/Looks/mat_dropoff", (0.9, 0.15, 0.1)
    )

    # ---- Lights ----
    UsdGeom.Xform.Define(stage, "/World/Lights")
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
    dome.CreateIntensityAttr(600.0)
    distant = UsdLux.DistantLight.Define(stage, "/World/Lights/DistantLight")
    distant.CreateIntensityAttr(2500.0)
    UsdGeom.Xformable(distant).AddRotateXYZOp().Set(Gf.Vec3f(45.0, 0.0, 30.0))

    # Three RectLights ("ceiling fixtures") for contrast / shadow definition.
    # Pointing -Z (downward) by default for RectLight.
    ceiling_lights = [
        ("/World/Lights/RectLight_north", (-3.0,  2.5, 2.8), (3.0, 1.5)),
        ("/World/Lights/RectLight_south", (-3.0, -2.5, 2.8), (3.0, 1.5)),
        ("/World/Lights/RectLight_east",  ( 3.0,  0.0, 2.8), (3.0, 1.5)),
    ]
    for path, pos, (w, h) in ceiling_lights:
        rl = UsdLux.RectLight.Define(stage, path)
        rl.CreateIntensityAttr(8000.0)
        rl.CreateWidthAttr(w)
        rl.CreateHeightAttr(h)
        rl.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
        rxf = UsdGeom.Xformable(rl)
        rxf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        # Tilt 180° about X so it points downward (RectLight default points +Z;
        # we want -Z).
        rxf.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))

    # ---- Floor ----
    floor_prim = _author_box(
        stage,
        "/World/Floor",
        size=(12.0, 8.0, 0.05),
        translate=(0.0, 0.0, -0.025),
    )
    _bind(floor_prim, materials["floor"])

    # ---- Walls ----
    UsdGeom.Xform.Define(stage, "/World/Walls")
    walls = {
        "north": ((12.0, 0.2, 1.5), (0.0, 4.0, 0.75)),
        "south": ((12.0, 0.2, 1.5), (0.0, -4.0, 0.75)),
        "east":  ((0.2, 8.0, 1.5),  (6.0, 0.0, 0.75)),
        "west":  ((0.2, 8.0, 1.5),  (-6.0, 0.0, 0.75)),
    }
    for name, (size, translate) in walls.items():
        prim = _author_box(stage, f"/World/Walls/{name}", size=size, translate=translate)
        _bind(prim, materials["walls"])

    # ---- Shelves (metal) ----
    UsdGeom.Xform.Define(stage, "/World/Obstacles")
    shelves = {
        "shelf_0": ((1.0, 0.4, 1.2), (1.0, 0.0, 0.6)),
        "shelf_1": ((1.0, 0.4, 1.2), (2.5, 2.0, 0.6)),
        "shelf_2": ((1.0, 0.4, 1.2), (-1.5, 2.5, 0.6)),
        "shelf_3": ((1.0, 0.4, 1.2), (3.5, -2.0, 0.6)),
    }
    for name, (size, translate) in shelves.items():
        prim = _author_box(stage, f"/World/Obstacles/{name}", size=size, translate=translate)
        _bind(prim, materials["shelves"])

    # ---- Cardboard / wood boxes ----
    boxes = {
        "box_0": ((0.4, 0.4, 0.4), (0.5, -1.8, 0.2)),
        "box_1": ((0.4, 0.4, 0.4), (-2.0, -2.8, 0.2)),
        "box_2": ((0.6, 0.5, 0.5), (1.6, -2.6, 0.25)),
        "box_3": ((0.3, 0.3, 0.3), (-0.5, 1.0, 0.15)),
    }
    for name, (size, translate) in boxes.items():
        prim = _author_box(stage, f"/World/Obstacles/{name}", size=size, translate=translate)
        _bind(prim, materials["boxes"])

    # ---- Cylindrical barrels (variation in geometry → ORB-friendly) ----
    barrels = {
        "barrel_0": (0.3, 0.9, (2.0, -1.0, 0.45)),
        "barrel_1": (0.3, 0.9, (-2.5, 0.5, 0.45)),
    }
    for name, (r, h, translate) in barrels.items():
        prim = _author_cylinder(stage, f"/World/Obstacles/{name}", r, h, translate)
        _bind(prim, materials["pallets"])

    # ---- Tilted slab (a leaning panel — adds non-axis-aligned features) ----
    slab = _author_box(
        stage,
        "/World/Obstacles/slab",
        size=(0.05, 1.2, 1.6),
        translate=(4.5, 1.0, 0.8),
        rotate_xyz=(0.0, 15.0, 0.0),
    )
    _bind(slab, materials["walls"])

    # ---- Floor pallets (thin platforms) ----
    pallets = {
        "pallet_0": ((1.2, 0.8, 0.12), (-4.0, 0.5, 0.06)),
        "pallet_1": ((1.2, 0.8, 0.12), (4.5, -1.5, 0.06)),
    }
    for name, (size, translate) in pallets.items():
        prim = _author_box(stage, f"/World/Obstacles/{name}", size=size, translate=translate)
        _bind(prim, materials["pallets"])

    # ---- Named pickup + dropoff zone Xforms + visible floor markers ----
    for zone, pos in ZONES.items():
        prim_name = "PickupZone" if zone == "pickup" else "DropoffZone"
        xform = _author_xform(stage, f"/World/{prim_name}", pos)
        attr = xform.GetPrim().CreateAttribute(
            "sortbots:zone", Sdf.ValueTypeNames.Token, custom=True
        )
        attr.Set(zone)
        # Visible floor tile on top of the floor at the zone location.
        # 1.4 × 1.4 m colored slab, 0.005 m above floor surface.
        marker_path = f"/World/ZoneMarkers/{prim_name}_marker"
        UsdGeom.Xform.Define(stage, "/World/ZoneMarkers")
        marker = _author_box(
            stage,
            marker_path,
            size=(1.4, 1.4, 0.01),
            translate=(pos[0], pos[1], 0.005),
            collider=False,
        )
        _bind(marker, materials[f"{zone}_marker"])

    # ---- Spawn points for each robot ----
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

    emit("build_warehouse v1 (textured)")
    emit(f"  out={out_path}")
    emit(f"  textures_dir={TEXTURES_DIR}")

    missing = [
        TEXTURES_DIR / fname
        for fname in TEXTURE_ROLES.values()
        if not (TEXTURES_DIR / fname).is_file()
    ]
    if missing:
        emit("WARNING: missing textures; run scripts/fetch_warehouse_textures.py:")
        for p in missing:
            emit(f"  {p}")

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
