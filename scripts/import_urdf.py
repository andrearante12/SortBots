#!/usr/bin/env python3
"""URDF → USD import pipeline for Isaac Sim 5.1 (Phase 2).

Wraps `URDFParseAndImportFile` so that one CLI invocation produces a USD
asset with the BEHAVIOR conventions baked in (Fix Base = false, Self
Collision = true), optionally swaps mesh wheel colliders for cylinder
primitives, and applies a JSON-driven physics override for joint drives.

Run after `source scripts/activate_isaac.sh`:

    python scripts/import_urdf.py \\
        --urdf  path/to/robot.urdf \\
        --out   assets/generated/robot.usd \\
        [--physics-overrides configs/physics_overrides/robot.json] \\
        [--wheel-link LINK]... \\
        [--no-fix-base] [--no-self-collision] [--merge-fixed-joints] \\
        [--dry-run]

Exit codes: 0 OK, 1 exception, 2 timeout. Progress is written to
``$ISAAC_IMPORT_RESULT`` (default ``/tmp/isaac_import_result.txt``) AND
stdout, because Kit swallows late writes during shutdown.
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import os
import sys
from pathlib import Path
from typing import Any

from _isaac_utils import (
    cancel_timeout,
    emit_factory,
    gpu_name,
    guard_against_ros2,
    install_timeout,
)

guard_against_ros2()

# SimulationApp MUST be constructed before any other omni / isaacsim imports.
from isaacsim import SimulationApp  # noqa: E402

SIM_APP_CONFIG = {"headless": True, "renderer": "RayTracedLighting"}
simulation_app = SimulationApp(SIM_APP_CONFIG)

install_timeout(
    seconds=180,
    hint=(
        "URDF import + USD save took too long. Common causes: large mesh files,\n"
        "Kit running on llvmpipe instead of NVIDIA, or a hang during physics\n"
        "scene authoring. Inspect $ISAAC_IMPORT_RESULT for the last completed step."
    ),
    close_app=simulation_app.close,
)

RESULT_FILE = os.environ.get("ISAAC_IMPORT_RESULT", "/tmp/isaac_import_result.txt")
emit = emit_factory(RESULT_FILE)


# --------------------------------------------------------------------------- CLI

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="URDF → USD importer for Isaac Sim 5.1")
    p.add_argument("--urdf", required=True, help="Path to input URDF")
    p.add_argument("--out", required=True, help="Destination USD path")
    p.add_argument(
        "--physics-overrides",
        default=None,
        help="JSON file with per-joint drive / articulation overrides",
    )
    p.add_argument(
        "--wheel-link",
        action="append",
        default=[],
        metavar="LINK",
        help="URDF link name whose mesh collider should be replaced with a cylinder primitive (repeatable)",
    )
    p.add_argument("--wheel-radius", type=float, default=None, help="Override auto-derived wheel radius (m)")
    p.add_argument("--wheel-length", type=float, default=None, help="Override auto-derived wheel length (m)")
    p.add_argument("--fix-base", dest="fix_base", action="store_true")
    p.add_argument("--no-fix-base", dest="fix_base", action="store_false")
    p.set_defaults(fix_base=False)
    p.add_argument("--self-collision", dest="self_collision", action="store_true")
    p.add_argument("--no-self-collision", dest="self_collision", action="store_false")
    p.set_defaults(self_collision=True)
    p.add_argument("--merge-fixed-joints", action="store_true", default=False)
    p.add_argument("--dry-run", action="store_true", help="Parse only, don't write USD")
    return p.parse_args()


# ------------------------------------------------------------------- import config

def make_import_config(args: argparse.Namespace):
    """Build an `ImportConfig` matching BEHAVIOR conventions for mobile robots."""
    import omni.kit.commands

    status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("URDFCreateImportConfig failed")

    cfg.merge_fixed_joints = args.merge_fixed_joints
    cfg.fix_base = args.fix_base
    cfg.self_collision = args.self_collision
    cfg.make_default_prim = True
    cfg.create_physics_scene = False  # tests / Phase 3 own the physics scene
    cfg.import_inertia_tensor = True
    cfg.distance_scale = 1.0
    cfg.parse_mimic = True
    # Leave replace_cylinders_with_capsules at its default (False) — we want
    # cylinders to stay cylinders.

    return cfg


# ------------------------------------------------------------------- URDF import

def import_urdf(urdf_path: str, cfg) -> str:
    """Run `URDFParseAndImportFile` into the current stage, return robot prim path."""
    import omni.kit.commands

    # NOTE: we pass dest_path="" so the importer adds the robot to the open
    # stage. We mutate it (wheel swap, physics overrides) and Save() ourselves.
    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=cfg,
        dest_path="",
        get_articulation_root=False,
    )
    if not status or not prim_path:
        raise RuntimeError(f"URDFParseAndImportFile failed for {urdf_path}")
    return prim_path


# ------------------------------------------------------ wheel mesh → cylinder swap

def _bbox_extent(prim) -> tuple[float, float, float]:
    """Return the world-space bounding-box extent (x, y, z) of a prim."""
    from pxr import Usd, UsdGeom

    bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    bound = bbcache.ComputeWorldBound(prim).ComputeAlignedBox()
    size = bound.GetSize()
    return (size[0], size[1], size[2])


def replace_wheel_colliders(
    stage,
    robot_prim_path: str,
    wheel_links: list[str],
    radius_override: float | None,
    length_override: float | None,
) -> int:
    """Replace mesh colliders on named wheel links with cylinder primitives.

    Returns the number of links actually mutated. If a named link has no mesh
    collider (e.g. URDF already uses `<cylinder>`), it is silently skipped.
    """
    if not wheel_links:
        return 0

    from pxr import PhysxSchema, Sdf, UsdGeom, UsdPhysics

    root = stage.GetPrimAtPath(robot_prim_path)
    if not root or not root.IsValid():
        raise RuntimeError(f"Robot prim {robot_prim_path} not found on stage")

    replaced = 0
    for link_name in wheel_links:
        # Match by USD prim name (Isaac importer mangles `-` → `_` etc.).
        link_prim = None
        for prim in Usd_iter(root):
            if prim.GetName() == link_name:
                link_prim = prim
                break
        if link_prim is None:
            emit(f"  wheel-swap: link '{link_name}' not found, skipping")
            continue

        # Find any child mesh prim with a CollisionAPI.
        mesh_child = None
        for child in link_prim.GetChildren():
            if child.IsA(UsdGeom.Mesh) and child.HasAPI(UsdPhysics.CollisionAPI):
                mesh_child = child
                break
        if mesh_child is None:
            emit(f"  wheel-swap: '{link_name}' has no mesh collider, skipping")
            continue

        x, y, z = _bbox_extent(mesh_child)
        # Heuristic: cylinder length is the smallest extent, radius is half the
        # largest of the other two. Callers can override with CLI flags.
        ext = sorted([(x, "x"), (y, "y"), (z, "z")])
        length = length_override if length_override is not None else ext[0][0]
        radius = radius_override if radius_override is not None else max(ext[1][0], ext[2][0]) / 2.0
        axis = ext[0][1].upper()

        # Define a sibling cylinder under the link, then remove the mesh.
        cyl_path = link_prim.GetPath().AppendChild(f"{link_name}_cyl_collider")
        cyl = UsdGeom.Cylinder.Define(stage, cyl_path)
        cyl.CreateRadiusAttr(radius)
        cyl.CreateHeightAttr(length)
        cyl.CreateAxisAttr(axis)
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        PhysxSchema.PhysxCollisionAPI.Apply(cyl.GetPrim())

        stage.RemovePrim(mesh_child.GetPath())
        emit(f"  wheel-swap: {link_name}  r={radius:.3f}m  L={length:.3f}m  axis={axis}")
        replaced += 1

    return replaced


def Usd_iter(prim):
    """Depth-first traversal of a prim's descendants."""
    for child in prim.GetChildren():
        yield child
        yield from Usd_iter(child)


# ------------------------------------------------------------ physics overrides

def load_overrides(path: str | None) -> dict[str, Any]:
    if path is None:
        return {"schema_version": 1}
    with open(path) as f:
        data = json.load(f)
    if data.get("schema_version") != 1:
        raise ValueError(
            f"{path}: unsupported schema_version {data.get('schema_version')!r} (expected 1)"
        )
    allowed = {"schema_version", "joints", "articulation"}
    extra = set(data.keys()) - allowed
    if extra:
        raise ValueError(f"{path}: unknown top-level keys {sorted(extra)}")
    return data


def apply_physics_overrides(stage, robot_prim_path: str, overrides: dict[str, Any]) -> int:
    """Walk joints, glob-match override entries, apply drive parameters.

    Returns the number of joints touched.
    """
    from pxr import PhysxSchema, UsdPhysics

    joint_specs = overrides.get("joints", {})
    art_spec = overrides.get("articulation", {})
    if not joint_specs and not art_spec:
        return 0

    root = stage.GetPrimAtPath(robot_prim_path)
    touched = 0

    # Joint overrides.
    for prim in Usd_iter(root):
        # Joints have UsdPhysics.Joint as their schema (or one of its derivatives).
        if not prim.IsA(UsdPhysics.Joint):
            continue
        name = prim.GetName()
        for glob, spec in joint_specs.items():
            if not fnmatch.fnmatchcase(name, glob):
                continue
            axis = _drive_axis_for(prim)
            if axis is None:
                emit(f"  drive: {name} has no compatible drive axis, skipping")
                continue
            drive = UsdPhysics.DriveAPI.Get(prim, axis)
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(prim, axis)
            _set_drive_params(drive, spec)
            if "friction" in spec:
                px = PhysxSchema.PhysxJointAPI.Apply(prim)
                px.CreateJointFrictionAttr(float(spec["friction"]))
            emit(f"  drive: {name} axis={axis} <- {glob}")
            touched += 1
            break  # first matching glob wins

    # Articulation-level overrides.
    if art_spec:
        for prim in Usd_iter(root):
            if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                continue
            px = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            for key, attr in (
                ("solver_position_iteration_count", px.CreateSolverPositionIterationCountAttr),
                ("solver_velocity_iteration_count", px.CreateSolverVelocityIterationCountAttr),
                ("sleep_threshold", px.CreateSleepThresholdAttr),
                ("stabilization_threshold", px.CreateStabilizationThresholdAttr),
                ("enabled_self_collisions", px.CreateEnabledSelfCollisionsAttr),
            ):
                if key in art_spec:
                    attr(art_spec[key])
            touched += 1
            break  # only one articulation root expected

    return touched


def _drive_axis_for(joint_prim) -> str | None:
    """Return 'angular' for revolute/continuous joints, 'linear' for prismatic."""
    from pxr import UsdPhysics

    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        return "angular"
    if joint_prim.IsA(UsdPhysics.PrismaticJoint):
        return "linear"
    return None


def _set_drive_params(drive, spec: dict[str, Any]) -> None:
    """Apply stiffness / damping / max_force / target to a UsdPhysics.DriveAPI."""
    drive_type = spec.get("drive_type")
    if drive_type in ("velocity", "position"):
        # Authoring as both "force" type and a target.
        drive.CreateTypeAttr("force")
        if drive_type == "velocity":
            target = float(spec.get("target_value", 0.0))
            attr = drive.GetTargetVelocityAttr() or drive.CreateTargetVelocityAttr(target)
            attr.Set(target)
        else:
            target = float(spec.get("target_value", 0.0))
            attr = drive.GetTargetPositionAttr() or drive.CreateTargetPositionAttr(target)
            attr.Set(target)
    elif drive_type == "none":
        # Zero stiffness + zero damping makes the drive a no-op.
        spec.setdefault("stiffness", 0.0)
        spec.setdefault("damping", 0.0)

    for key, getter, creator in (
        ("stiffness", drive.GetStiffnessAttr, drive.CreateStiffnessAttr),
        ("damping", drive.GetDampingAttr, drive.CreateDampingAttr),
        ("max_force", drive.GetMaxForceAttr, drive.CreateMaxForceAttr),
    ):
        if key in spec:
            attr = getter() or creator(float(spec[key]))
            attr.Set(float(spec[key]))


# ------------------------------------------------------------------------- main

def main() -> int:
    args = parse_args()

    urdf_path = str(Path(args.urdf).resolve())
    out_path = str(Path(args.out).resolve())
    if not Path(urdf_path).is_file():
        print(f"ERROR: URDF not found: {urdf_path}", file=sys.stderr)
        return 1

    overrides = load_overrides(args.physics_overrides)

    import isaacsim

    version = getattr(isaacsim, "__version__", "unknown")
    emit(f"import_urdf v0 | isaacsim {version} | GPU: {gpu_name()}")
    emit(f"  urdf={urdf_path}")
    emit(f"  out={out_path}")
    emit(f"  fix_base={args.fix_base}  self_collision={args.self_collision}")
    emit(f"  merge_fixed_joints={args.merge_fixed_joints}")
    emit(f"  wheel_links={args.wheel_link or '(none)'}")
    emit(f"  physics_overrides={args.physics_overrides or '(none)'}")

    if args.dry_run:
        emit("  --dry-run: parsing only, no USD will be written")

    import omni.usd

    # Start from a fresh stage so each run is reproducible.
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()

    cfg = make_import_config(args)
    robot_prim_path = import_urdf(urdf_path, cfg)
    emit(f"  imported -> {robot_prim_path}")

    n_wheels = replace_wheel_colliders(
        stage, robot_prim_path, args.wheel_link, args.wheel_radius, args.wheel_length
    )
    emit(f"  wheel-swap: replaced {n_wheels} mesh collider(s)")

    n_overrides = apply_physics_overrides(stage, robot_prim_path, overrides)
    emit(f"  overrides: applied {n_overrides} entr{'y' if n_overrides == 1 else 'ies'}")

    if args.dry_run:
        emit("import_urdf: DRY-RUN OK")
        return 0

    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    stage.Export(out_path)
    emit(f"  wrote {out_path}")
    emit("import_urdf: OK")
    return 0


try:
    rc = main()
except Exception as e:
    print(f"ERROR: {type(e).__name__}: {e}", file=sys.stderr)
    import traceback

    traceback.print_exc()
    rc = 1
finally:
    cancel_timeout()
    simulation_app.close()

sys.exit(rc)
