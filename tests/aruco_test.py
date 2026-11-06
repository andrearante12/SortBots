"""simtest/aruco.py and package_vision.detect_markers: the marker the sim builds is one OpenCV
reads, from every face. Offline, numpy + cv2.

    /usr/bin/python3 -m pytest tests/aruco_test.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "nodes"))
sys.path.insert(0, str(REPO_ROOT))

cv2 = pytest.importorskip("cv2")
import package_vision as pv  # noqa: E402
from simtest import aruco, world  # noqa: E402

W, H = 848, 480
K = (605.0, 605.0, 424.0, 240.0)
SIZE = (0.14, 0.14, 0.10)


def view(size, face, marker, dist=0.7, skew_deg=0.0):
    """RGB + depth of a marker, from `dist` in front of its face, seen `skew_deg` off its normal.
    Returns the images and the true (face centre, outward normal) in the optical frame."""
    right, up, n = (np.array(v) for v in aruco.face_axes(face))
    corners = np.array(aruco.backing(size, face))
    centre = corners.mean(axis=0)
    a = math.radians(skew_deg)
    eye = centre + dist * (math.cos(a) * n + math.sin(a) * right)
    fwd = (centre - eye) / np.linalg.norm(centre - eye)
    x = np.cross(fwd, up)
    x /= np.linalg.norm(x)
    y = np.cross(fwd, x)
    rot = np.stack([x, y, fwd])

    def px(p):
        c = rot @ (np.asarray(p) - eye)
        return [K[0] * c[0] / c[2] + K[2], K[1] * c[1] / c[2] + K[3]]

    # 8x supersampled, as a renderer's antialiasing would leave it
    ss = 4
    img = np.full((H * ss, W * ss), 150, np.uint8)
    poly = lambda q, v: cv2.fillConvexPoly(img, (np.array([px(p) for p in q]) * ss).astype(np.int32), v)  # noqa: E731
    poly(corners, 255)
    for cell in aruco.dark_cells(size, face, marker):
        poly(cell, 0)
    gray = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)
    rgb = np.stack([gray] * 3, axis=-1)

    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    ray = np.stack([(us - K[2]) / K[0], (vs - K[3]) / K[1], np.ones_like(us, dtype=float)], axis=-1)
    nc = rot @ n
    t = ((rot @ (centre - eye)) @ nc) / (ray @ nc)
    depth = (t * ray[..., 2]).astype(np.float32)
    face_c = rot @ (centre - n * (world.PLATE_THICKNESS_M + aruco.BACKING_PROUD_M) - eye)
    return rgb, depth, face_c, nc


@pytest.mark.parametrize("marker", sorted(aruco.DATA))
def test_the_table_is_opencvs(marker):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    # drawMarker was renamed generateImageMarker in OpenCV 4.7; Ubuntu 24.04's apt cv2 is 4.6.
    draw = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    m = draw(d, marker, 6)
    assert tuple("".join("1" if v > 127 else "0" for v in row) for row in m[1:5, 1:5]) == aruco.DATA[marker]


@pytest.mark.parametrize("face", ["+x", "-x", "+y", "-y", "top"])
def test_a_marker_built_on_any_face_reads_back_not_mirrored(face):
    rgb, depth, *_ = view(SIZE, face, 3)
    assert [m["marker"] for m in pv.detect_markers(rgb, depth, K)] == [3]


@pytest.mark.parametrize("marker", sorted(aruco.DATA))
def test_every_id_reads_back(marker):
    rgb, depth, *_ = view(SIZE, "+x", marker)
    assert [m["marker"] for m in pv.detect_markers(rgb, depth, K)] == [marker]


@pytest.mark.parametrize("dist,skew", [(0.45, 0.0), (0.74, 0.0), (0.74, 30.0), (1.2, 20.0)])
def test_position_and_normal_come_from_depth(dist, skew):
    rgb, depth, centre, normal = view(SIZE, "+x", 0, dist=dist, skew_deg=skew)
    (m,) = pv.detect_markers(rgb, depth, K)
    assert math.dist(m["c"], centre) < 0.01
    assert math.degrees(math.acos(min(1.0, float(np.dot(m["n"], normal))))) < 2.0
    assert m["range_m"] == pytest.approx(float(np.linalg.norm(centre)), abs=0.02)


def test_without_depth_the_corners_still_give_a_pose():
    rgb, depth, centre, _ = view(SIZE, "+x", 0)
    (m,) = pv.detect_markers(rgb, np.full_like(depth, np.nan), K)
    assert math.dist(m["c"], centre) < 0.05     # coarse: ranged from a 43-pixel square


def test_a_marker_takes_over_from_the_colour_guess_on_the_same_plate():
    found = pv.prefer_markers([{"c": (0, 0, 1), "marker": 2}],
                              [{"c": (0.01, 0, 1)}, {"c": (1, 0, 1)}])
    assert [p.get("marker") for p in found] == [2, None] and found[1]["c"] == (1, 0, 1)


def test_the_tracker_and_to_map_carry_the_id():
    m = pv.to_map({"c": (0, 0, 1), "n": (0, 0, -1), "conf": 1.0, "px": (1, 2, 3, 4), "uv": (2, 3),
                   "range_m": 1.0, "marker": 7}, np.eye(4))
    assert m["marker"] == 7
    t = pv.Tracker()
    t.update([dict(m, marker=None)], 0.0)
    (tr,) = t.update([m], 0.1)
    assert tr["marker"] == 7


def test_a_package_can_carry_a_marker_if_its_plate_is_big_enough():
    base = {"name": "t", "packages": [{"id": "p", "pose": [3, 3], "size": [0.14, 0.14, 0.10], "marker": 4}]}
    assert world.validate(base)["packages"][0]["marker"] == 4
    base["packages"][0]["size"] = [0.08, 0.08, 0.08]
    with pytest.raises(world.WorldError, match="too small"):
        world.validate(base)
    base["packages"][0].update(size=[0.14, 0.14, 0.10], marker=99)
    with pytest.raises(world.WorldError, match="marker"):
        world.validate(base)
