"""Offline tests for prompt selection and tracks (no ROS / no model)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))

import seg_tracks as st  # noqa: E402

PERSON = {"bbox": [100, 50, 140, 200], "cls": "person", "score": 0.8}
BOX = {"bbox": [300, 150, 360, 210], "cls": "box", "score": 0.7}


def test_always_class_is_prompted_on_sight():
    ps = st.select_prompts([PERSON], [], [])
    assert [(p.reason, p.cls) for p in ps] == [("always", "person")]


def test_if_moving_class_needs_a_blob_or_track():
    assert st.select_prompts([BOX], [], []) == []
    blob = {"bbox": [310, 160, 330, 190], "source": "motion"}
    ps = st.select_prompts([BOX], [blob], [])
    assert [(p.reason, p.cls) for p in ps] == [("if_moving", "box")]  # the blob is covered
    t = st.Track(1, [302, 150, 362, 210], "box", "if_moving", 0.0, 0.0)
    assert [p.reason for p in st.select_prompts([BOX], [], [t])] == ["if_moving"]


def test_unclassed_blob_is_still_segmented():
    blob = {"bbox": [500, 100, 540, 180], "source": "conflict", "centroid": [520, 140]}
    ps = st.select_prompts([PERSON], [blob], [])
    assert {p.reason for p in ps} == {"always", "conflict"}
    assert next(p for p in ps if p.reason == "conflict").point == [520, 140]


def test_low_score_detection_ignored():
    assert st.select_prompts([dict(PERSON, score=0.1)], [], []) == []


def test_unmatched_track_is_reprompted_from_its_prediction():
    t = st.Track(7, [10, 10, 50, 90], None, "motion", 0.0, 0.0)
    ps = st.select_prompts([], [], [t], predicted={7: [20, 10, 60, 90]})
    assert len(ps) == 1 and ps[0].reason == "track" and ps[0].track_id == 7
    assert ps[0].bbox == [20, 10, 60, 90]


def test_prompts_dedupe_and_cap_by_priority():
    dets = [dict(PERSON, bbox=[i * 50, 0, i * 50 + 40, 100]) for i in range(10)]
    blobs = [{"bbox": [100 + i * 50, 300, 140 + i * 50, 380], "source": "motion"} for i in range(5)]
    ps = st.select_prompts(dets + [dict(PERSON, bbox=[1, 0, 41, 100])], blobs, [], cfg={"max_prompts": 8})
    assert len(ps) == 8
    assert all(p.reason == "always" for p in ps)   # people outrank blobs


def test_accept_mask_rejects_shelf_swallowing():
    m = np.zeros((480, 848), bool)
    m[50:200, 100:140] = True
    assert st.accept_mask(m, [100, 50, 140, 200])
    shelf = m.copy()
    shelf[0:300, 140:600] = True                 # most of it outside the box
    assert not st.accept_mask(shelf, [100, 50, 140, 200])
    assert not st.accept_mask(np.ones((480, 848), bool), [0, 0, 848, 480])
    assert not st.accept_mask(np.zeros((480, 848), bool), [0, 0, 10, 10])


def test_stopped_person_stays_tracked_while_detected():
    ts = st.TrackSet({"track_ttl_s": 5.0})
    for i in range(30):                          # 30 s standing still, still detected
        p = st.Prompt([100, 50, 140, 200], "always", "person")
        live = ts.update([(p, [100, 50, 140, 200], {})], float(i))
    assert len(live) == 1 and live[0].hits == 30


def test_stopped_unknown_object_expires_after_ttl():
    ts = st.TrackSet({"track_ttl_s": 5.0})
    ts.update([(st.Prompt([10, 10, 50, 90], "motion"), [10, 10, 50, 90], {})], 0.0)
    tid = ts.tracks[0].id
    for i in range(1, 10):                       # only its own re-prompts from here
        p = st.Prompt([10, 10, 50, 90], "track", track_id=tid)
        live = ts.update([(p, [10, 10, 50, 90], {})], float(i))
        if i <= 5:
            assert len(live) == 1, i
    assert live == []


def test_tracks_associate_by_overlap_and_new_ones_get_ids():
    ts = st.TrackSet()
    ts.update([(st.Prompt([0, 0, 40, 100], "always", "person"), [0, 0, 40, 100], {})], 0.0)
    ts.update([(st.Prompt([5, 0, 45, 100], "always", "person"), [5, 0, 45, 100], {}),
               (st.Prompt([300, 0, 340, 100], "motion"), [300, 0, 340, 100], {})], 1.0)
    assert sorted(t.id for t in ts.tracks) == [1, 2]
    assert ts.tracks[0].bbox == [5, 0, 45, 100]


def test_predict_box_follows_the_camera():
    # Optical frame == odom frame at first; the object 4 m ahead, centred.
    K = (400.0, 400.0, 400.0, 240.0)
    t = st.Track(1, [380, 140, 420, 340], "person", "always", 0.0, 0.0,
                 depth_m=4.0, center_odom=[0.0, 0.0, 4.0])
    same = st.predict_box(t, K, np.eye(3), np.zeros(3))
    assert np.allclose(same, t.bbox)
    # Camera stepped 2 m closer: same centre, box twice the size.
    closer = st.predict_box(t, K, np.eye(3), np.array([0.0, 0.0, -2.0]))
    assert np.allclose([(closer[0] + closer[2]) / 2, closer[2] - closer[0]], [400, 80])
    # Camera moved 1 m left: object shifts right in the image.
    left = st.predict_box(t, K, np.eye(3), np.array([1.0, 0.0, 0.0]))
    assert (left[0] + left[2]) / 2 == 500
