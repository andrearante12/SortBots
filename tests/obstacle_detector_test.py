"""Offline tests for nodes/obstacle_detector.py (no ROS / no GPU).

Scenes are rendered as synthetic depth images through a pinhole camera, so
the tests exercise the same back-projection -> TF -> floor -> cluster path
the node runs on camera/depth.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))

import obstacle_detector as od  # noqa: E402

W, H = 320, 240
FX = FY = 200.0
CX, CY = W / 2, H / 2
K = [FX, 0, CX, 0, FY, CY, 0, 0, 1]
CAM_Z = 1.0  # camera height above the floor, base_link origin on the floor
# Optical (x right, y down, z fwd) -> base_link (x fwd, y left, z up).
R_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)


def _tilt(deg: float) -> np.ndarray:
    """Camera pitched DOWN by `deg` (rotation about base_link y)."""
    a = np.radians(deg)
    ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0],
                   [-np.sin(a), 0, np.cos(a)]], dtype=np.float32)
    return ry @ R_OPT


def render(boxes=(), *, tilt_deg=30.0, floor_z=0.0, cam_z=CAM_Z):
    """Ray-cast a depth image of a floor plane plus axis-aligned boxes.

    boxes: ((xmin, ymin, zmin), (xmax, ymax, zmax)) in base_link.
    Returns (depth HxW float32, R_base_cam, t_base_cam).
    """
    R = _tilt(tilt_deg)
    t = np.array([0, 0, cam_z], dtype=np.float32)
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays_c = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u, float)], -1)
    rays = rays_c @ R.T  # base_link direction per pixel, for unit optical z
    depth = np.full((H, W), np.inf, dtype=np.float32)
    # Floor: t.z + s * ray.z = floor_z
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (floor_z - t[2]) / rays[..., 2]
    depth = np.where((s > 0) & np.isfinite(s), s, depth).astype(np.float32)
    for lo, hi in boxes:
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (lo - t) / rays
            t2 = (hi - t) / rays
        tn = np.nanmax(np.minimum(t1, t2), axis=-1)
        tf = np.nanmin(np.maximum(t1, t2), axis=-1)
        hit = (tn <= tf) & (tn > 0)
        depth = np.where(hit & (tn < depth), tn, depth).astype(np.float32)
    return depth, R, t


PERSON = ((2.0, -0.2, 0.0), (2.3, 0.2, 1.7))
CFG = {"stride": 2, "min_range_m": 0.2, "max_range_m": 6.0}


def test_empty_floor_has_no_obstacles():
    depth, R, t = render()
    obs, pts, floor = od.detect(depth, K, R, t, CFG)
    assert obs == []
    assert pts.shape == (0, 3)
    assert floor == pytest.approx(0.0, abs=0.03)


def test_person_sized_box_detected_with_geometry():
    depth, R, t = render([PERSON])
    obs, pts, _ = od.detect(depth, K, R, t, CFG)
    assert len(obs) == 1
    o = obs[0]
    # Only the near face is visible, so x is the face at 2.0 m.
    assert o["range_m"] == pytest.approx(2.0, abs=0.1)
    assert o["centroid"][1] == pytest.approx(0.0, abs=0.1)
    assert o["bearing_deg"] == pytest.approx(0.0, abs=5)
    assert o["min"][1] == pytest.approx(-0.2, abs=0.05)
    assert o["max"][1] == pytest.approx(0.2, abs=0.05)
    assert o["height_m"] > 0.5
    assert len(pts) == o["n"]
    u0, v0, u1, v1 = o["uv"]
    assert u0 < CX < u1 and 0 <= v0 < v1 < H


def test_floor_offset_is_estimated():
    # base_link origin 0.4 m above the floor (a model whose base_link sits at
    # the axle): with floor_z_m left at 0 the auto estimate still finds it.
    depth, R, t = render([((2.0, -0.2, -0.4), (2.3, 0.2, 1.3))],
                         floor_z=-0.4, cam_z=0.6)
    obs, _, floor = od.detect(depth, K, R, t, CFG)
    assert floor == pytest.approx(-0.4, abs=0.03)
    assert len(obs) == 1


def test_floor_auto_off_uses_configured_height():
    depth, R, t = render(floor_z=-0.4, cam_z=0.6)
    _, _, floor = od.detect(depth, K, R, t, {**CFG, "floor_auto": False,
                                             "floor_z_m": -0.4})
    assert floor == pytest.approx(-0.4)


def test_low_debris_below_height_band_ignored():
    depth, R, t = render([((1.5, -0.2, 0.0), (1.7, 0.2, 0.05))])
    obs, _, _ = od.detect(depth, K, R, t, CFG)
    assert obs == []


def test_two_separated_objects_are_two_clusters_nearest_first():
    a = ((1.5, 0.4, 0.0), (1.7, 0.7, 1.0))
    b = ((2.5, -0.9, 0.0), (2.7, -0.6, 1.0))
    depth, R, t = render([a, b])
    obs, _, _ = od.detect(depth, K, R, t, CFG)
    assert len(obs) == 2
    assert obs[0]["range_m"] < obs[1]["range_m"]
    assert obs[0]["centroid"][1] > 0 > obs[1]["centroid"][1]


def test_out_of_range_object_cropped():
    depth, R, t = render([((5.0, -0.2, 0.0), (5.3, 0.2, 1.7))], tilt_deg=0)
    obs, _, _ = od.detect(depth, K, R, t, {**CFG, "max_range_m": 4.0})
    assert obs == []


def test_16uc1_millimetres_decode():
    raw = np.array([[1000, 0], [2500, 65535]], dtype=np.uint16)
    d = od.depth_to_metres(raw.tobytes(), "16UC1", 2, 2)
    assert d[0, 0] == pytest.approx(1.0)
    assert d[1, 0] == pytest.approx(2.5)
    assert od.depth_to_metres(b"", "rgb8", 0, 0) is None


def test_cluster_cells_connectivity_and_speckle():
    xy = np.array([[0.05, 0.05]] * 5 + [[0.15, 0.15]] * 5   # diagonal: joins
                  + [[1.05, 1.05]] * 5                       # separate
                  + [[3.05, 3.05]], dtype=np.float32)        # lone speckle
    lab = od.cluster_cells(xy, cell_m=0.1, min_cell_points=3)
    assert lab[0] == lab[5] != lab[10]
    assert lab[15] == -1


def test_quat_identity_and_yaw():
    assert np.allclose(od.quat_to_matrix(0, 0, 0, 1), np.eye(3))
    r = od.quat_to_matrix(0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4))
    assert np.allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-6)


def test_tracker_flags_crossing_person_as_moving():
    """A person crossing the field of view left to right at 1 m/s."""
    tr = od.Tracker()
    states = []
    for k in range(10):
        t = k * 0.1
        states.append(tr.update([(2.0, 1.0 - 1.0 * t)], [0.4], t)[0])
    assert len({s["track_id"] for s in states}) == 1
    assert not states[0]["moving"]
    assert states[-1]["moving"]
    assert states[-1]["vel"][1] == pytest.approx(-1.0, abs=0.2)


def test_tracker_static_and_long_structures_never_move():
    tr = od.Tracker()
    for k in range(10):
        t = k * 0.1
        # Small static box with 1 cm jitter, and a shelf whose centroid
        # slides along it at 1 m/s as more of it comes into view.
        s = tr.update([(1.5, 0.01 * (-1) ** k), (3.0 + t, 2.0)], [0.3, 4.0], t)
    assert not s[0]["moving"]
    assert not s[1]["moving"]


def test_tracker_drops_stale_tracks_and_gates_jumps():
    tr = od.Tracker()
    a = tr.update([(1.0, 0.0)], [0.3], 0.0)[0]["track_id"]
    # A detection 2 m away is a new object, not a 20 m/s jump.
    b = tr.update([(3.0, 0.0)], [0.3], 0.1)[0]["track_id"]
    assert a != b
    tr.update([], [], 5.0)
    assert tr.tracks == {}


def test_end_to_end_person_crossing_rendered_frames():
    """Rendered frames of a person walking across: detected and moving."""
    tr = od.Tracker()
    last = None
    for k in range(8):
        t = k * 0.1
        y = 0.8 - 1.0 * t
        depth, R, tt = render([((2.0, y - 0.2, 0.0), (2.3, y + 0.2, 1.7))])
        obs, _, _ = od.detect(depth, K, R, tt, CFG)
        assert len(obs) == 1
        last = tr.update([tuple(obs[0]["centroid"][:2])], [obs[0]["extent_m"]], t)[0]
    assert last["moving"]


def test_payload_shape():
    p = od.build_payload([{"moving": True}, {"moving": False}], stamp=1.23456,
                         frame="robot_0/base_link", floor_z=0.01, width=848,
                         height=480, world_frame="robot_0/odom")
    assert p["count"] == 2 and p["moving"] == 1
    assert p["w"] == 848 and p["stamp"] == 1.235
