"""simtest/stl.py and the committed effector meshes. Offline, numpy.

    /usr/bin/python3 -m pytest tests/magnet_mesh_test.py
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from simtest import magnet, stl  # noqa: E402


def test_the_magnets_face_is_where_the_sim_grabs_from():
    # the mesh and the capture rule have to agree, or the box snaps to a point in mid air
    tri = stl.to_link_frame(stl.read_stl(stl.MAGNET_STL))
    assert tri[..., 1].min() == pytest.approx(magnet.TCP_OFFSET_M[1], abs=5e-4)


def test_the_effector_sits_on_the_roll_axis_and_reaches_back_to_the_horn():
    tri = stl.to_link_frame(stl.read_stl(stl.BODY_STL))
    assert abs(tri[..., 1].max()) < 0.002                 # the horn face, at the link's origin
    assert abs(tri[..., 0].mean()) < 0.005 and abs(tri[..., 2].mean()) < 0.005
    assert 0.045 < np.ptp(tri[..., 0]) < 0.055           # Ø45.4 body plus the switch boss


def test_the_meshes_stay_small_enough_to_commit():
    assert stl.BODY_STL.stat().st_size + stl.MAGNET_STL.stat().st_size < 2_000_000


def test_to_link_frame_is_a_proper_rotation():
    axes = np.eye(3).reshape(1, 3, 3) * 1000.0
    out = stl.to_link_frame(axes)[0]
    assert np.linalg.det(out) == pytest.approx(1.0)
    assert out[2] == pytest.approx((0.0, -1.0, 0.0))      # CAD +Z is the roll axis, -Y on the link


def test_a_truncated_file_is_refused(tmp_path):
    bad = tmp_path / "x.stl"
    bad.write_bytes(b"\0" * 80 + struct.pack("<I", 5) + b"\0" * 10)
    with pytest.raises(ValueError, match="not a binary STL"):
        stl.read_stl(bad)
