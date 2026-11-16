"""Export the effector for the sim: assets/magnet_effector_{body,magnet}.stl.

    hardware/.venv/bin/python hardware/magnet_effector/sim_mesh.py

Coarser than build.py's print STLs, on purpose: the sim only has to look right, and these
are committed. Same assembly frame as model.py: +Z is the wrist-roll axis, z=0 the horn face.
Re-run after changing params.py; scripts/spawn_warehouse.py mounts these on the arm, and
tests/magnet_mesh_test.py holds the magnet's face to simtest.magnet.TCP_OFFSET_M.
"""
from pathlib import Path

import cadquery as cq

import model
from params import DEFAULT as p

ASSETS = Path(__file__).resolve().parents[2] / "assets"
TOLERANCE_MM, ANGULAR_RAD = 0.15, 0.35


def _export(shapes: list, name: str) -> None:
    solids = [s for wp in shapes for s in wp.val().Solids()]
    path = ASSETS / f"magnet_effector_{name}.stl"
    cq.Compound.makeCompound(solids).exportStl(str(path), TOLERANCE_MM, ANGULAR_RAD, ascii=False)
    print(f"wrote {path} ({path.stat().st_size // 1024} KiB)")


def main() -> None:
    _export([model.adapter_body(p), model.plunger(p)], "body")
    _export([model.magnet(p)], "magnet")


if __name__ == "__main__":
    main()
