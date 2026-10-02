"""Offline tests for the dashboard's sensor-view nodes (no ROS / no GPU)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))

import depth_colorizer as dc  # noqa: E402
import feature_overlay as fo  # noqa: E402


def _payload(keys, kps, matches, inliers, **kw):
    return fo.build_payload(keys, kps, matches, inliers, lost=False, features=len(keys),
                            n_matches=len(matches), n_inliers=len(inliers),
                            stamp=1.5, width=848, height=480, **kw)


def test_states_follow_word_ids():
    p = _payload([10, 11, 12], [(1.4, 2.6, 7.0), (3, 4, 5), (5, 6, 7)], [11, 12], [12])
    by_xy = {(pt[0], pt[1]): pt[3] for pt in p["pts"]}
    assert by_xy[(1, 3)] == fo.DETECTED
    assert by_xy[(3, 4)] == fo.MATCHED
    assert by_xy[(5, 6)] == fo.INLIER
    assert p["w"] == 848 and p["inliers"] == 1 and p["lost"] is False


def test_cap_keeps_inliers_first():
    keys = list(range(100))
    kps = [(i, i, 4.0) for i in keys]
    p = _payload(keys, kps, keys[:50], keys[:10], max_points=20)
    assert len(p["pts"]) == 20
    assert sum(pt[3] == fo.INLIER for pt in p["pts"]) == 10


def test_empty_and_json_roundtrip():
    import json
    p = _payload([], [], [], [])
    assert p["pts"] == []
    json.dumps(p)


def test_colorize_shape_and_invalid_black():
    d = np.full((48, 64), 2.0, dtype=np.float32)
    d[0, 0] = np.nan
    d[2, 2] = 0.0
    d[4, 4] = 9.0  # beyond max range
    img = dc.colorize_depth(d, decimate=1)
    assert img.shape == (48, 64, 3) and img.dtype == np.uint8
    for yx in [(0, 0), (2, 2), (4, 4)]:
        assert not img[yx].any()
    assert img[10, 10].any()


def test_near_is_redder_than_far():
    d = np.zeros((4, 8), dtype=np.float32)
    d[:, :4] = 0.5
    d[:, 4:] = 4.0
    img = dc.colorize_depth(d, decimate=1)
    near, far = img[0, 0], img[0, 7]  # BGR
    assert near[2] > near[0]
    assert far[0] > far[2]


def test_decimate():
    d = np.full((48, 64), 1.0, dtype=np.float32)
    assert dc.colorize_depth(d, decimate=2).shape == (24, 32, 3)


def test_depth_to_metres_encodings():
    mm = np.full((2, 3), 1500, dtype=np.uint16).tobytes()
    assert np.allclose(dc.depth_to_metres(mm, "16UC1", 2, 3), 1.5)
    m = np.full((2, 3), 2.5, dtype=np.float32).tobytes()
    assert np.allclose(dc.depth_to_metres(m, "32FC1", 2, 3), 2.5)
    assert dc.depth_to_metres(mm, "rgb8", 2, 3) is None
