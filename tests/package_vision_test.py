"""nodes/package_vision.py: plate detection on synthetic frames. Offline, numpy + cv2.

Frames come from a tiny ray caster (rectangles in the camera's optical frame), so the
plate's true centre and normal are known exactly and the detector is held to them.

    /usr/bin/python3 -m pytest tests/package_vision_test.py
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

pytest.importorskip("cv2")
import package_vision as pv  # noqa: E402
from simtest import world  # noqa: E402

W, H = 848, 480
K = (605.0, 605.0, 424.0, 240.0)
CARDBOARD = tuple(int(round(c * 255)) for c in world.DEFAULT_PACKAGE_COLOR)
PLATE = tuple(int(round(c * 255)) for c in world.PLATE_COLOR)
GREY = (95, 95, 98)
FACE_W, FACE_H = 0.18, 0.12          # a +x face of the default package (sy x sz)


def render(rects, bg_depth=6.0):
    """RGB uint8 + depth float32 from rectangles {c, u, v, hu, hv, rgb} in the optical
    frame; nearest hit per pixel, grey background, depth in metres."""
    fx, fy, cx, cy = K
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    d = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us, dtype=float)], axis=-1)
    rgb = np.empty((H, W, 3), np.uint8)
    rgb[:] = GREY
    depth = np.full((H, W), bg_depth, np.float32)
    for r in rects:
        c, u, v = (np.array(r[k], float) for k in ("c", "u", "v"))
        nrm = np.cross(u, v)
        t = (c @ nrm) / (d @ nrm)
        p = d * t[..., None]
        a = (p - c) @ u
        b = (p - c) @ v
        hit = (t > 0) & (np.abs(a) <= r["hu"]) & (np.abs(b) <= r["hv"]) & (t < depth)
        rgb[hit] = r["rgb"]
        depth[hit] = t[hit].astype(np.float32)
    return rgb, depth


def box_with_plate(centre, yaw_deg, plate=True):
    """A box face (cardboard) at `centre` in the optical frame, turned `yaw_deg` about
    the camera's vertical axis, with a magnet plate on it 4 mm proud. Returns the
    rectangles and the plate's true (centre, outward normal)."""
    a = math.radians(yaw_deg)
    n = np.array([math.sin(a), 0.0, -math.cos(a)])         # outward: towards the camera
    u = np.array([math.cos(a), 0.0, math.sin(a)])
    v = np.array([0.0, 1.0, 0.0])
    c = np.array(centre, float)
    rects = [{"c": c, "u": u, "v": v, "hu": FACE_W / 2, "hv": FACE_H / 2, "rgb": CARDBOARD}]
    if plate:
        rects.append({"c": c + n * 0.004, "u": u, "v": v, "hu": 0.6 * FACE_W / 2,
                      "hv": 0.6 * FACE_H / 2, "rgb": PLATE})
    return rects, c, n


def angle(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(a, b))))))


@pytest.mark.parametrize("dist", [1.0, 1.6, 2.5])
@pytest.mark.parametrize("yaw", [0, 25, -25, 45, -45])
def test_plate_centre_and_normal_on_clean_frames(dist, yaw):
    rects, c_true, n_true = box_with_plate((0.15, 0.30, dist), yaw)
    found = pv.detect_plates(*render(rects), K)
    assert len(found) == 1, found
    p = found[0]
    assert np.linalg.norm(np.array(p["c"]) - c_true) < 0.015
    assert angle(p["n"], n_true) < 5.0
    assert p["conf"] > 0.3


def test_a_plain_box_has_no_plate():
    rects, _, _ = box_with_plate((0.0, 0.3, 1.5), 0, plate=False)
    assert pv.detect_plates(*render(rects), K) == []


def test_a_dark_thing_not_touching_cardboard_is_not_a_plate():
    # a tyre or a rack leg: dark, flat, in range, and nowhere near a box
    dark = [{"c": (0.0, 0.0, 1.5), "u": (1, 0, 0), "v": (0, 1, 0), "hu": 0.1, "hv": 0.07,
             "rgb": PLATE}]
    assert pv.detect_plates(*render(dark), K) == []


def _dark(c, hu, hv):
    return {"c": c, "u": (1, 0, 0), "v": (0, 1, 0), "hu": hu, "hv": hv, "rgb": PLATE}


def _card(c, hu, hv):
    return {"c": c, "u": (1, 0, 0), "v": (0, 1, 0), "hu": hu, "hv": hv, "rgb": CARDBOARD}


def test_a_dark_gap_beside_a_carton_is_not_a_plate():
    # what filled the full warehouse with false plates: plate-sized and flat, and it
    # touches cardboard, but only along one side
    rects = [_card((-0.2, 0, 1.2), 0.15, 0.15), _dark((0.0, 0, 1.2), 0.05, 0.04)]
    assert pv.detect_plates(*render(rects), K) == []


def test_a_dark_panel_too_big_to_be_a_plate_is_refused():
    rects = [_dark((0, 0, 1.2), 0.15, 0.12), _card((0, 0, 1.21), 0.3, 0.25)]
    assert pv.detect_plates(*render(rects), K) == []


def test_a_thin_dark_strip_on_cardboard_is_refused():
    rects = [_dark((0, 0, 1.2), 0.08, 0.006), _card((0, 0, 1.21), 0.2, 0.15)]
    assert pv.detect_plates(*render(rects), K) == []


def test_a_dark_hole_behind_the_cardboard_is_not_a_plate():
    # cardboard all round it, but 30 cm back: a gap seen through a carton's frame
    frame = [_card((0, 0.12, 1.2), 0.2, 0.08), _card((0, -0.12, 1.2), 0.2, 0.08),
             _card((-0.14, 0, 1.2), 0.06, 0.05), _card((0.14, 0, 1.2), 0.06, 0.05)]
    rects = frame + [_dark((0, 0, 1.5), 0.2, 0.2)]
    assert pv.detect_plates(*render(rects), K) == []


def test_a_detection_says_what_the_camera_saw_for_the_operator_view():
    rects, centre, _ = box_with_plate((0.0, 0.0, 1.2), 0)
    (p,) = pv.detect_plates(*render(rects), K)
    u, v, w, h = p["px"]
    assert u < p["uv"][0] < u + w and v < p["uv"][1] < v + h
    assert p["range_m"] == pytest.approx(math.dist(centre, (0, 0, 0)), abs=0.01)
    m = pv.to_map(p, np.eye(4))
    assert m["uv"] == list(p["uv"]) and m["range_m"] == p["range_m"]
    t = pv.Tracker()
    t.update([m], 0.0)
    (tr,) = t.update([dict(m, uv=[1.0, 2.0], range_m=9.0)], 0.1)
    assert tr["uv"] == [1.0, 2.0] and tr["range_m"] == 9.0


def test_a_plate_near_the_bottom_of_the_frame_still_counts():
    # pick_table's first frame: the plate's lower edge a few rows above the image's
    rects, centre, _ = box_with_plate((0.0, 0.25, 0.75), 0)
    found = pv.detect_plates(*render(rects), K)
    assert len(found) == 1
    assert math.dist(found[0]["c"], centre) < 0.03


def test_two_boxes_two_plates():
    a, ca, _ = box_with_plate((-0.5, 0.3, 1.5), 15)
    b, cb, _ = box_with_plate((0.6, 0.3, 2.0), -20)
    found = pv.detect_plates(*render(a + b), K)
    assert len(found) == 2
    got = sorted(found, key=lambda p: p["c"][0])
    assert np.linalg.norm(np.array(got[0]["c"]) - ca) < 0.015
    assert np.linalg.norm(np.array(got[1]["c"]) - cb) < 0.015


def test_a_plate_beyond_the_range_limit_is_ignored():
    rects, _, _ = box_with_plate((0.0, 0.3, 5.0), 0)
    assert pv.detect_plates(*render(rects), K) == []


def test_a_bumpy_depth_fit_is_refused():
    rects, _, _ = box_with_plate((0.0, 0.3, 1.5), 0)
    rgb, depth = render(rects)
    rng = np.random.default_rng(3)
    depth = depth + rng.normal(0.0, 0.02, depth.shape).astype(np.float32)   # 2 cm of noise
    assert pv.detect_plates(rgb, depth, K) == []


def test_depth_holes_do_not_crash_the_fit():
    rects, c_true, _ = box_with_plate((0.0, 0.3, 1.5), 0)
    rgb, depth = render(rects)
    depth[::7, ::5] = np.nan
    depth[:, 300:330] = 0.0
    (p,) = pv.detect_plates(rgb, depth, K)
    assert np.linalg.norm(np.array(p["c"]) - c_true) < 0.02


def test_segment_masks_follow_the_two_colours():
    rects, _, _ = box_with_plate((0.0, 0.3, 1.5), 0)
    box, plate = pv.segment(render(rects)[0])
    assert box.sum() > plate.sum() > 100
    assert not (box & plate).any()
    assert not box[:20].any() and not plate[:20].any()      # the grey background is neither


def test_to_map_applies_the_camera_pose():
    # camera at (2, 3) 1.2 m up looking along +y: optical z -> map +y, x -> map +x, y -> map -z
    t = np.eye(4)
    t[:3, :3] = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])  # columns: x_c, y_c, z_c in map
    t[:3, 3] = (2.0, 3.0, 1.2)
    p = pv.to_map({"c": (0.1, 0.4, 1.5), "n": (0.0, 0.0, -1.0), "conf": 0.9, "px": (1, 2, 3, 4), "uv": (2.0, 3.0), "range_m": 1.0}, t)
    assert (p["x"], p["y"], p["z"]) == pytest.approx((2.1, 4.5, 0.8))
    assert (p["nx"], p["ny"], p["nz"]) == pytest.approx((0.0, -1.0, 0.0))


def test_depth_encodings():
    f = np.full((2, 3), 1.5, np.float32)
    assert pv.depth_to_metres(f.tobytes(), "32FC1", 2, 3) == pytest.approx(f)
    u = np.full((2, 3), 1500, np.uint16)
    assert pv.depth_to_metres(u.tobytes(), "16UC1", 2, 3) == pytest.approx(f)
    assert pv.depth_to_metres(b"", "rgb8", 0, 0) is None


def test_tracker_smooths_associates_and_forgets():
    tr = pv.Tracker(assoc_m=0.15, ema=0.5, drop_after_s=2.0)
    mk = lambda x: {"x": x, "y": 1.0, "z": 0.8, "nx": 0.0, "ny": 1.0, "nz": 0.0,  # noqa: E731
                    "conf": 0.8, "px": [0, 0, 1, 1]}
    out = tr.update([mk(0.00)], 0.0)
    out = tr.update([mk(0.04)], 0.1)                   # near: the same plate, smoothed
    assert len(out) == 1 and out[0]["x"] == pytest.approx(0.02)
    out = tr.update([mk(2.0)], 0.2)                    # far: a second plate
    assert len(out) == 2
    out = tr.update([mk(2.0)], 1.0)                    # only the second is still in view
    assert len(out) == 2                               # the first is remembered, and aged
    assert sorted(o["age_s"] for o in out) == pytest.approx([0.0, 0.9])
    out = tr.update([], 2.5)                           # the first (last seen 0.1) is now stale
    assert len(out) == 1 and out[0]["x"] == pytest.approx(2.0) and out[0]["age_s"] == 1.5
    assert tr.update([], 5.0) == []


def test_the_committed_config_matches_the_vision_defaults_and_detects():
    import package_detector as pd

    sim = pd.load_config(pd.DEFAULT_CONFIG, "sim")
    merged = pv.merged(sim)
    for key in ("box", "plate", "plane_rms_m", "min_range_m", "max_range_m", "plate_thickness_m"):
        assert merged[key] == pv.DEFAULTS[key], key        # one set of numbers, two places
    assert pv.DEFAULTS["plate_thickness_m"] == world.PLATE_THICKNESS_M    # what the sim authors
    rects, c_true, _ = box_with_plate((0.1, 0.3, 1.5), 20)
    (p,) = pv.detect_plates(*render(rects), K, sim)
    assert np.linalg.norm(np.array(p["c"]) - c_true) < 0.015


def test_real_thresholds_override_sim_ones_only_on_the_real_platform():
    import package_detector as pd

    sim = pd.load_config(pd.DEFAULT_CONFIG, "sim")
    real = pd.load_config(pd.DEFAULT_CONFIG, "real")
    assert "sim" not in sim and "real" not in sim and "track" in sim
    assert real["plate"]["v_max"] != sim["plate"]["v_max"]
    assert real["rate_hz"] == sim["rate_hz"]


def test_image_encodings():
    import package_detector as pd

    rgb = np.zeros((2, 2, 3), np.uint8)
    rgb[0, 0] = (10, 20, 30)
    assert pd.rgb_from_image(rgb.tobytes(), "rgb8", 2, 2)[0, 0].tolist() == [10, 20, 30]
    assert pd.rgb_from_image(rgb.tobytes(), "bgr8", 2, 2)[0, 0].tolist() == [30, 20, 10]
    rgba = np.concatenate([rgb, np.full((2, 2, 1), 255, np.uint8)], axis=-1)
    assert pd.rgb_from_image(rgba.tobytes(), "rgba8", 2, 2).shape == (2, 2, 3)
    assert pd.rgb_from_image(b"", "mono8", 0, 0) is None


def test_tracker_never_merges_two_plates_seen_in_one_frame():
    tr = pv.Tracker(assoc_m=0.5)
    mk = lambda x: {"x": x, "y": 0.0, "z": 0.8, "nx": 0.0, "ny": 1.0, "nz": 0.0,  # noqa: E731
                    "conf": 1.0, "px": [0, 0, 1, 1]}
    assert len(tr.update([mk(0.0), mk(0.3)], 0.0)) == 2
    assert len(tr.update([mk(0.0), mk(0.3)], 0.1)) == 2
