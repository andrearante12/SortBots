"""The magnet effector's mesh, ready for USD. numpy only.

hardware/magnet_effector/sim_mesh.py exports binary STLs in the CAD frame (millimetres, +Z
along the wrist-roll axis away from the servo, z=0 the horn face). The arm's `Fixed_Jaw`
link has its roll axis along -Y (simtest/magnet.py), so a CAD point lands at (x, -z, y).
That is a proper rotation, +90 degrees about X; which way round the roll it sits is cosmetic.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
BODY_STL = REPO_ROOT / "assets" / "magnet_effector_body.stl"
MAGNET_STL = REPO_ROOT / "assets" / "magnet_effector_magnet.stl"

_TRIANGLE = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])


def read_stl(path: Path | str) -> np.ndarray:
    """A binary STL's triangles as (N, 3, 3) float64 vertices."""
    data = Path(path).read_bytes()
    n = int.from_bytes(data[80:84], "little")
    if len(data) != 84 + n * _TRIANGLE.itemsize:
        raise ValueError(f"{path}: not a binary STL of {n} triangles")
    return np.frombuffer(data, dtype=_TRIANGLE, offset=84, count=n)["v"].astype(float)


def to_link_frame(triangles: np.ndarray) -> np.ndarray:
    """CAD millimetres -> metres in the Fixed_Jaw frame."""
    x, y, z = triangles[..., 0], triangles[..., 1], triangles[..., 2]
    return np.stack([x, -z, y], axis=-1) * 0.001
