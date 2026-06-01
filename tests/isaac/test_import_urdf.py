"""Phase 2 smoke test: import_urdf.py produces valid Isaac-Sim USDs.

Asserts that each robot's USD opens cleanly and has a UsdPhysics ArticulationRootAPI
somewhere under the robot's default prim. Does NOT step physics.
"""
from __future__ import annotations

from pathlib import Path

import pytest


@pytest.mark.parametrize("robot", ["carter", "xlerobot"])
def test_usd_has_articulation_root(robot, request, sim_app):
    usd_path: Path = request.getfixturevalue(f"{robot}_usd_path")
    assert usd_path.is_file(), f"{usd_path} missing"

    import omni.usd
    from pxr import UsdPhysics

    ctx = omni.usd.get_context()
    ctx.open_stage(str(usd_path))
    stage = ctx.get_stage()
    assert stage is not None, "stage failed to open"

    found = False
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            found = True
            break
    assert found, f"no UsdPhysics.ArticulationRootAPI in {usd_path}"
