# Isaac Sim Phase 2 — URDF → USD import pipeline

End state: `python scripts/import_urdf.py --urdf ... --out ...` produces a
USD asset that loads cleanly into Isaac Sim and (for a mobile robot) drives
forward >1 m in 5 s of headless physics. `pytest -x tests/isaac/` exercises
the pipeline on two placeholder URDFs — NVIDIA Carter (ships with Isaac Sim)
and the team's XLeRobot URDF (`third_party/XLeRobot/...`).

This page assumes Phase 1 ([`isaac_sim_setup.md`](isaac_sim_setup.md)) is
done.

## What the pipeline does

`scripts/import_urdf.py` is a thin wrapper over `URDFParseAndImportFile` from
the `isaacsim.asset.importer.urdf` extension. One invocation:

1. Opens a fresh in-memory USD stage (headless `SimulationApp`).
2. Builds an `ImportConfig` with the BEHAVIOR conventions for mobile robots
   — `Fix Base Link = false`, `Self Collision = true`, fixed joints kept,
   inertia tensors copied through, distance scale fixed at 1 m.
3. Parses the URDF and adds the robot prim to the stage.
4. Optionally swaps mesh wheel colliders for `UsdGeom.Cylinder` primitives
   (PhysX is much happier with primitives than mesh collisions for wheels).
   No-op when no `--wheel-link` is passed.
5. Applies a JSON-driven set of joint-drive and articulation-solver
   overrides.
6. `stage.Export()`s the resulting USD to disk.

Progress is mirrored to stdout AND `$ISAAC_IMPORT_RESULT`
(default `/tmp/isaac_import_result.txt`), because Kit captures stdout during
shutdown and can drop the last few prints.

## Running the importer

After `source scripts/activate_isaac.sh`:

```bash
# Carter (ships under the Isaac Sim install)
python scripts/import_urdf.py \
    --urdf  ~/isaacsim/venv/lib/python3.11/site-packages/isaacsim/exts/isaacsim.asset.importer.urdf/data/urdf/robots/carter/urdf/carter.urdf \
    --out   assets/generated/carter.usd \
    --physics-overrides configs/physics_overrides/carter.json

# XLeRobot (vendored via the submodule)
python scripts/import_urdf.py \
    --urdf  third_party/XLeRobot/simulation/Maniskill/assets/xlerobot/xlerobot.urdf \
    --out   assets/generated/xlerobot.usd \
    --physics-overrides configs/physics_overrides/xlerobot.json \
    --fix-base
```

`assets/generated/` is `.gitignore`d — it's a build artifact directory.

Important per-robot wrinkles:

- **Carter**'s URDF already uses `<cylinder>` collisions on its wheels, so
  `--wheel-link` is unnecessary. The flag stays in the CLI for the day a
  team-authored URDF needs it.
- **XLeRobot**'s URDF models the mobile base as a ManiSkill-style holonomic
  base: a `root` link plus three faux joints (`root_x_axis_joint` prismatic,
  `root_y_axis_joint` prismatic, `root_z_rotation_joint` continuous) that
  carry `base_link` around the world. The visible "raskog wheels" are
  visual-only STLs with no collision. Because `root` has nothing above it,
  the entire articulation must be imported with `--fix-base` — otherwise
  applying velocity to the prismatic joint pushes `root_arm_1_link_1` one
  way and reaction-pushes `root` the other.

## CLI flags

| Flag                    | Default | Notes                                                              |
|-------------------------|---------|--------------------------------------------------------------------|
| `--urdf`                | required | Path to input URDF.                                                |
| `--out`                 | required | Destination `.usd` path.                                           |
| `--physics-overrides`   | none    | JSON file (schema below). Missing keys = no change.               |
| `--wheel-link LINK`     | none    | Repeatable. Mesh collider on that link → cylinder primitive.       |
| `--wheel-radius`        | auto    | Override the auto-derived cylinder radius (meters).               |
| `--wheel-length`        | auto    | Override the auto-derived cylinder length (meters).               |
| `--fix-base` / `--no-fix-base` | `--no-fix-base` | BEHAVIOR convention for mobile robots; flip for holonomic bases. |
| `--self-collision` / `--no-self-collision` | `--self-collision` | BEHAVIOR convention. |
| `--merge-fixed-joints`  | off     | Off matches the BEHAVIOR convention; on shrinks the kinematic tree. |
| `--dry-run`             | off     | Parse and emit overrides, but do not write USD.                    |

## Physics-override JSON

`configs/physics_overrides/schema.json` is the authoritative schema. A
typical file looks like:

```json
{
  "schema_version": 1,
  "joints": {
    "*_wheel": {
      "drive_type": "velocity",
      "target_value": 0.0,
      "stiffness": 0.0,
      "damping": 100000.0,
      "max_force": 10000000.0
    }
  },
  "articulation": {
    "solver_position_iteration_count": 16,
    "solver_velocity_iteration_count": 1,
    "enabled_self_collisions": true
  }
}
```

- Joint matchers are `fnmatch` globs over the USD joint prim name (which is
  derived from the URDF joint name). First match wins.
- Values are in USD-native units: **degrees / deg-per-second** for angular
  drives (revolute, continuous), **meters / m-per-second** for linear
  drives (prismatic). The script automatically picks the `linear` vs
  `angular` driver axis from the joint's USD schema.
- `drive_type` of `"none"` zeroes stiffness and damping so the joint becomes
  effectively free (matches Carter's caster `rear_pivot` / `rear_axle`).
- Unknown top-level keys fail loudly so typos don't silently no-op.

Three overrides ship in `configs/physics_overrides/`:

- `default.json` — `{"schema_version": 1}`, a no-op baseline.
- `carter.json` — velocity drive on Carter's two main wheels, drives removed
  from the caster, and a tighter articulation solver.
- `xlerobot.json` — velocity drive on `root_x_axis_joint`, locking position
  drives on the other two holonomic joints, and stiff position drives on
  every arm / head joint so the arms don't flop during a drive test.

## How to add a new robot

1. Drop the URDF somewhere addressable (most likely `third_party/` or
   `assets/imported_urdfs/`).
2. Author a `configs/physics_overrides/<name>.json` modeled on the existing
   files. Use `*_wheel` (or whatever the URDF joint naming convention is)
   to set wheel drives to velocity mode, and add position drives on
   joints that should hold pose during the test.
3. Run the importer with `--physics-overrides` pointed at the new JSON.
4. Add an entry to `ROBOT_CONFIGS` in `tests/isaac/conftest.py` so the
   regression test runs against the new robot. Pick `drive_mode`:
   - `"diff_drive"` if the URDF has real `left_wheel`/`right_wheel`-style
     continuous joints with cylinder colliders (Carter pattern).
   - `"prismatic"` if the URDF models the base with a fake X-axis prismatic
     joint that translates the body (XLeRobot / ManiSkill pattern).

## Running the regression test

```bash
source scripts/activate_isaac.sh
pip install pytest                # one-time, into the Isaac venv
pytest -x tests/isaac/
```

The test session:

1. Subprocess-runs `scripts/import_urdf.py` twice (Carter, XLeRobot) and
   caches the resulting USDs under `assets/generated/`. Re-runs reuse the
   cache; set `SORTBOTS_FORCE_REIMPORT=1` to force fresh imports.
2. Starts one session-wide `SimulationApp` for the drive tests so we pay
   the ~9 s Kit startup once.
3. `test_import_urdf.py` opens each USD and asserts at least one prim
   carries `UsdPhysics.ArticulationRootAPI`.
4. `test_drive_forward.py` spawns each robot in a fresh `World`, drives
   forward for 300 sim steps (5 s at 60 Hz), and asserts the displacement
   along the drive axis exceeds 1 m without the robot flying off the
   ground.

Per-robot drive-test results land in `/tmp/isaac_drive_test_<robot>_result.txt`.

## Troubleshooting

- **`URDFParseAndImportFile` returned status False** — the URDF reference
  resolver couldn't find a mesh file. Check that `meshes/*.stl` and friends
  are next to the URDF (or that `package://` references resolve via your
  ROS package paths).
- **"Static 3D model, joints disabled"** — Isaac 5.x's known import-time
  fallback when the articulation root can't be resolved, typically because
  a URDF root link has no inertial block. Add an inertial to the root link
  in the URDF.
- **"Unresolved reference prim path" warnings on `*/visuals`** — non-fatal.
  Triggered by URDF links that declare `<collision>` and `<inertial>` but
  no `<visual>` (e.g. Carter's `com_offset` and `imu`). The geometry that
  matters is still imported.
- **Wheel cylinder is mis-oriented** — `UsdGeom.Cylinder` defaults to a Z
  axis; URDF wheel joints almost always rotate around Y. The importer
  reads the mesh bounding box to pick the axis, but a wheel mesh that's
  closer to a sphere than a disc can confuse the heuristic — pass
  `--wheel-radius` / `--wheel-length` explicitly.
- **`SimulationApp` constructor hangs** — same root causes as Phase 1.
  Re-check `OMNI_KIT_ACCEPT_EULA=YES` (Phase 1's `activate_isaac.sh` sets
  it), confirm `vulkaninfo --summary` lists the NVIDIA GPU, and make sure
  no other Kit process is holding the GPU.
- **Pytest "ROS 2 is sourced" exit** — `_isaac_utils.guard_against_ros2`
  fail-fasts when `AMENT_PREFIX_PATH` is set. Open a clean terminal and
  re-source `scripts/activate_isaac.sh`.
- **Second test in the session crashes with `'World' object has no
  attribute '_scene'`** — between tests you must call
  `World.clear_instance()`, NOT `SimulationContext.clear_instance()`. The
  `World` class keeps a class-level `_world_initialized` flag that gates
  `self._scene = Scene()` in `__init__`; clearing only `SimulationContext`
  leaves the flag set, so the next `World()` returns early without
  initializing `_scene`. The `world` fixture in
  `tests/isaac/conftest.py` already does this correctly.
