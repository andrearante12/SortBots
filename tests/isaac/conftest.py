"""Pytest fixtures for Isaac Sim Phase 2 tests.

A single session-scoped `SimulationApp` is shared across all tests so we pay
the ~9 s Kit startup once. URDF→USD imports are run as subprocesses (using
`scripts/import_urdf.py`) BEFORE the SimulationApp is constructed, so the
two processes never compete for the GPU.

Set `SORTBOTS_FORCE_REIMPORT=1` to force fresh USDs even if cached files exist.
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

# Make sibling modules (_robot_configs.py) importable from test files.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _robot_configs import REPO_ROOT, ROBOT_CONFIGS  # noqa: E402

SCRIPTS = REPO_ROOT / "scripts"


def _run_import(name: str, cfg: dict) -> Path:
    out: Path = cfg["out"]
    if out.is_file() and not os.environ.get("SORTBOTS_FORCE_REIMPORT"):
        return out

    if not cfg["urdf"].is_file():
        pytest.skip(f"{name}: URDF not found at {cfg['urdf']}")

    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(SCRIPTS / "import_urdf.py"),
        "--urdf",
        str(cfg["urdf"]),
        "--out",
        str(out),
        "--physics-overrides",
        str(cfg["override"]),
    ]
    if cfg["fix_base"]:
        cmd.append("--fix-base")
    env = os.environ.copy()
    env["ISAAC_IMPORT_RESULT"] = f"/tmp/isaac_import_{name}_result.txt"

    result = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=300)
    if result.returncode != 0 or not out.is_file():
        raise RuntimeError(
            f"import_urdf.py failed for {name}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}\n"
        )
    return out


@pytest.fixture(scope="session")
def carter_usd_path() -> Path:
    return _run_import("carter", ROBOT_CONFIGS["carter"])


@pytest.fixture(scope="session")
def xlerobot_usd_path() -> Path:
    return _run_import("xlerobot", ROBOT_CONFIGS["xlerobot"])


@pytest.fixture(scope="session")
def sim_app(carter_usd_path, xlerobot_usd_path):
    """Session-wide SimulationApp.

    Depends on the USD-generation fixtures so the subprocess imports complete
    before we open this process's own SimulationApp. Avoids two Kit processes
    fighting for the ~3 GB VRAM each one needs.
    """
    # Ensure scripts/ is importable for any test that needs internal helpers.
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})
    try:
        yield app
    finally:
        app.close()


@pytest.fixture()
def world(sim_app):
    """Fresh `World` per test, with ground plane and a distant light.

    `World.clear_instance()` (NOT `SimulationContext.clear_instance()`) is
    required between tests — World sets a class-level `_world_initialized`
    flag that gates `self._scene = Scene()` in `__init__`. Clearing only
    SimulationContext leaves the flag True, so the next `World()` returns
    without initializing `_scene`, and `world.scene` raises AttributeError.
    """
    import omni.usd
    from isaacsim.core.api import World

    World.clear_instance()
    omni.usd.get_context().new_stage()

    from isaacsim.core.api.objects.ground_plane import GroundPlane

    w = World(stage_units_in_meters=1.0)
    GroundPlane(prim_path="/World/groundPlane", z_position=0.0)

    from pxr import Sdf, UsdLux

    stage = omni.usd.get_context().get_stage()
    light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
    light.CreateIntensityAttr(500)

    try:
        yield w
    finally:
        try:
            w.stop()
        except Exception:
            pass
        try:
            w.clear()
        except Exception:
            pass
        World.clear_instance()
