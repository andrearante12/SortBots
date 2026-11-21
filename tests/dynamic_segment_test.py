"""Offline tests for nodes/dynamic_segment.py's model I/O and per-frame step.

No GPU, no ROS, no real weights: the detector and SAM sessions are fakes with
the real models' input/output shapes, plus one tiny ONNX graph run through
onnxruntime so the ort backend's plumbing is exercised for real.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "nodes"))

cv2 = pytest.importorskip("cv2")
import dynamic_segment as ds  # noqa: E402
import seg_tracks as st  # noqa: E402

CLASSES = ["person", "forklift", "pallet jack", "robot", "box", "cart", "pallet", "trolley", "bin"]
H, W = 480, 848


def _yolo_out(boxes_cls_score, n=50, size=640, hw=(H, W)):
    """A (1, 4+nc, n) head output with the given original-pixel boxes."""
    s = min(size / hw[0], size / hw[1])
    px, py = (size - round(hw[1] * s)) // 2, (size - round(hw[0] * s)) // 2
    out = np.zeros((1, 4 + len(CLASSES), n), np.float32)
    for i, (box, cls, score) in enumerate(boxes_cls_score):
        u0, v0, u1, v1 = box
        out[0, :4, i] = [(u0 + u1) / 2 * s + px, (v0 + v1) / 2 * s + py, (u1 - u0) * s, (v1 - v0) * s]
        out[0, 4 + CLASSES.index(cls), i] = score
    return out


def test_yolo_postprocess_maps_back_to_image_pixels():
    out = _yolo_out([([100, 50, 140, 200], "person", 0.9), ([102, 52, 141, 199], "person", 0.6),
                     ([300, 150, 360, 210], "box", 0.4), ([10, 10, 20, 20], "cart", 0.1)])
    _, meta = ds.yolo_preprocess(np.zeros((H, W, 3), np.uint8), 640)
    dets = ds.yolo_postprocess(out, meta, CLASSES, 0.25, 0.5)
    assert sorted(d["cls"] for d in dets) == ["box", "person"]       # NMS'd duplicate, low score gone
    person = next(d for d in dets if d["cls"] == "person")
    assert np.allclose(person["bbox"], [100, 50, 140, 200], atol=0.5) and person["score"] == pytest.approx(0.9)
    # Transposed (1, N, 4+nc) layout reads the same.
    assert len(ds.yolo_postprocess(out.transpose(0, 2, 1), meta, CLASSES, 0.25, 0.5)) == 2


def test_yolo_postprocess_rejects_a_vocabulary_mismatch():
    _, meta = ds.yolo_preprocess(np.zeros((H, W, 3), np.uint8), 640)
    with pytest.raises(ValueError, match="does not match"):
        ds.yolo_postprocess(np.zeros((1, 10, 50), np.float32), meta, CLASSES, 0.25, 0.5)


def test_sam_pre_and_prompt_coordinates():
    x, s = ds.sam_preprocess(np.full((H, W, 3), 128, np.uint8), 1024)
    assert x.shape == (1, 3, 1024, 1024) and s == pytest.approx(1024 / W)
    assert np.all(x[0, :, round(H * s) + 2:, :] == 0)                 # bottom padding
    coords, labels = ds.sam_prompt_arrays(st.Prompt([100, 50, 140, 200], "motion", point=[120, 120]), s)
    assert coords.shape == (1, 3, 2) and labels.tolist() == [[1.0, 2.0, 3.0]]
    assert np.allclose(coords[0, 1], np.array([100, 50]) * s)


def test_sam_feeds_follow_the_decoder_inputs():
    names5 = ["image_embeddings", "point_coords", "point_labels", "mask_input", "has_mask_input"]
    f = ds.sam_feeds(names5, np.zeros((1, 256, 64, 64)), np.zeros((1, 2, 2)), np.zeros((1, 2)), (H, W))
    assert list(f) == names5
    f6 = ds.sam_feeds(names5 + ["orig_im_size"], None, None, None, (H, W))
    assert f6["orig_im_size"].tolist() == [H, W]


def test_sam_postprocess_low_res_to_image():
    s = 1024 / W
    low = np.full((1, 4, 256, 256), -5.0, np.float32)
    # Object at image rows 100-300, cols 200-400 → 256-grid = image * s / 4.
    r0, r1, c0, c1 = (int(round(v * s / 4)) for v in (100, 300, 200, 400))
    low[0, 2, r0:r1, c0:c1] = 5.0
    m, iou = ds.sam_postprocess({"iou_predictions": np.array([[0.1, 0.2, 0.9, 0.3]]),
                                 "low_res_masks": low}, s, (H, W))
    assert m.shape == (H, W) and iou == pytest.approx(0.9)
    assert m[120:280, 220:380].all() and not m[:80].any() and not m[:, 450:].any()


class FakeDetector:
    def __init__(self, dets):
        self.dets = dets

    def detect(self, rgb):
        return self.dets


class FakeSegmenter:
    """Returns exactly the prompt box as the mask."""
    def set_image(self, rgb):
        self.hw = rgb.shape[:2]

    def segment(self, p):
        m = np.zeros(self.hw, bool)
        u0, v0, u1, v1 = (int(round(x)) for x in p.bbox)
        m[v0:v1, u0:u1] = True
        return m, 0.9


def test_pipeline_masks_people_and_moving_boxes_only():
    rgb = np.zeros((H, W, 3), np.uint8)
    person = {"bbox": [100, 50, 140, 200], "cls": "person", "score": 0.8}
    box = {"bbox": [300, 150, 360, 210], "cls": "box", "score": 0.8}
    shelf_box = {"bbox": [600, 100, 660, 160], "cls": "box", "score": 0.8}
    blob = {"bbox": [310, 160, 340, 200], "source": "motion"}
    p = ds.Pipeline(FakeDetector([person, box, shelf_box]), FakeSegmenter(), {})
    mask, info = p.step(rgb, [blob], 0.0)
    assert mask[60:190, 110:130].all()           # person
    assert mask[160:200, 310:350].all()          # box backed by motion
    assert not mask[100:160, 600:660].any()      # box sitting on a shelf
    assert info["prompts"] == 2 and len(info["tracks"]) == 2


def test_pipeline_keeps_a_stopped_unknown_mover_for_the_ttl():
    rgb = np.zeros((H, W, 3), np.uint8)
    p = ds.Pipeline(FakeDetector([]), FakeSegmenter(), {"track_ttl_s": 3.0})
    blob = {"bbox": [400, 100, 440, 300], "source": "conflict"}
    assert p.step(rgb, [blob], 0.0)[0][150, 420]
    for t in (1.0, 2.0, 3.0):                    # it stopped: no blob, no detection
        assert p.step(rgb, [], t)[0][150, 420], t
    assert not p.step(rgb, [], 4.0)[0].any()


def test_pipeline_drops_a_mask_that_swallows_the_shelf():
    class Greedy(FakeSegmenter):
        def segment(self, p):
            m, s = super().segment(p)
            m[0:300, 140:700] = True
            return m, s
    p = ds.Pipeline(FakeDetector([{"bbox": [100, 50, 140, 200], "cls": "person", "score": 0.9}]),
                    Greedy(), {})
    mask, info = p.step(np.zeros((H, W, 3), np.uint8), [], 0.0)
    assert not mask.any() and info["rejected"] == 1


def test_oracle_mask_picks_actor_ids():
    ids = np.array([[0, 1, 5], [5, 7, 9]], np.uint32)
    labels = {"0": "BACKGROUND", "1": "UNLABELLED", "5": "/World/SimTest/actors/p1/body",
              "7": "/World/robot_1/base_link", "9": {"class": "actor:cart1"}}
    assert ds.oracle_mask(ids, labels).tolist() == [[False, False, True], [True, False, True]]
    assert not ds.oracle_mask(ids, {}).any()


def test_ort_session_runs_a_real_onnx_graph(tmp_path):
    onnx = pytest.importorskip("onnx")
    pytest.importorskip("onnxruntime")
    from onnx import TensorProto, helper
    # images (1,3,640,640) -> output0 = constant head output + 0 * mean(images)
    head = _yolo_out([([100, 50, 140, 200], "person", 0.9)])
    g = helper.make_graph(
        [helper.make_node("ReduceMean", ["images"], ["m"], keepdims=0),
         helper.make_node("Mul", ["m", "zero"], ["z"]),
         helper.make_node("Add", ["head", "z"], ["output0"])],
        "fake_yolo",
        [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, 640, 640])],
        [helper.make_tensor_value_info("output0", TensorProto.FLOAT, list(head.shape))],
        [helper.make_tensor("head", TensorProto.FLOAT, head.shape, head.ravel()),
         helper.make_tensor("zero", TensorProto.FLOAT, [], [0.0])])
    model = helper.make_model(g, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    path = tmp_path / "fake_yolo.onnx"
    onnx.save(model, path)
    det = ds.Detector(ds.open_session(str(path), "ort"), CLASSES, 640, 0.25, 0.5)
    dets = det.detect(np.zeros((H, W, 3), np.uint8))
    assert len(dets) == 1 and dets[0]["cls"] == "person"
    assert np.allclose(dets[0]["bbox"], [100, 50, 140, 200], atol=0.5)


def test_config_classes_are_in_the_export_vocabulary():
    pytest.importorskip("yaml")
    for platform in ("sim", "real"):
        cfg = ds.load_config(ds.DEFAULT_CONFIG, platform)
        vocab = set(cfg["detector"]["classes"])
        assert set(cfg["always"]) <= vocab and set(cfg["if_moving"]) <= vocab
    real = ds.load_config(ds.DEFAULT_CONFIG, "real")
    assert real["backend"] == "remote" and real["remote"]["port"]   # the host's TensorRT server


class _Echo:
    """A session that remembers what it was fed."""
    input_names = ["image"]

    def __init__(self, out):
        self.out, self.fed = out, None

    def run(self, feeds):
        self.fed = feeds
        return self.out


def test_segmenter_b1_encoder_takes_512_raw_but_prompts_stay_in_1024():
    # PPHGV2-B1: a 512 input, no mean/std, and the decoder still in SAM's 1024 frame
    enc = _Echo({"image_embeddings": np.zeros((1, 256, 64, 64), np.float32)})
    seg = ds.Segmenter(enc, None, size=1024, encoder_size=512, normalize=False)
    rgb = np.full((H, W, 3), 200, np.uint8)
    seg.set_image(rgb)
    x = enc.fed["image"]
    assert x.shape == (1, 3, 512, 512)
    assert x[0, :, 0, 0].tolist() == [200.0, 200.0, 200.0]   # raw 0-255, not normalised
    assert seg._scale == pytest.approx(1024 / W)             # image px -> the decoder's 1024 frame
    old = ds.Segmenter(enc, None, size=1024)                 # the ResNet18 default is unchanged
    old.set_image(rgb)
    assert enc.fed["image"].shape == (1, 3, 1024, 1024) and enc.fed["image"][0, 0, 0, 0] != 200.0


def test_mask_rle_round_trips_through_the_pooled_grid():
    m = np.zeros((H, W), bool)
    m[100:200, 300:310] = True          # a thin arm survives the max-pool
    m[0, 0] = True                      # runs that start with ones begin with a 0 run
    mw, mh, runs = ds.mask_rle(m, 4)
    assert (mw, mh) == (W // 4, H // 4) and sum(runs) == mw * mh and runs[0] == 0
    flat, on = [], False
    for r in runs:
        flat += [on] * r
        on = not on
    small = np.array(flat).reshape(mh, mw)
    assert small[25:50, 75:78].all() and small[0, 0] and small.sum() == 25 * 3 + 1
    p = ds.overlay_payload(m, {"masked": [{"bbox": [300, 100, 310, 200], "cls": "person", "reason": "always"}],
                               "dets": [], "tracks": [{}], "latency_ms": 41.0}, 12.5)
    assert p["stride"] == 4 and p["tracks"] == 1 and p["masked"][0]["cls"] == "person"


def test_remote_protocol_carries_arrays_both_ways():
    import socket
    a, b = socket.socketpair()
    feeds = {"images": np.arange(24, dtype=np.float32).reshape(1, 2, 3, 4),
             "point_labels": np.array([[2.0, 3.0]], np.float32)}
    ds.send_msg(a, {"op": "run", "model": "yolo_world"}, feeds)
    h, got = ds.recv_msg(b)
    assert h == {"op": "run", "model": "yolo_world"}
    assert set(got) == set(feeds) and all(np.array_equal(got[k], feeds[k]) and got[k].dtype == feeds[k].dtype
                                          for k in feeds)
    ds.send_msg(b, {"error": "nope"})
    assert ds.recv_msg(a) == ({"error": "nope"}, {})
    a.close(); b.close()


def test_remote_session_against_a_fake_server():
    # the node's side of scripts/dynamic_seg_server.py: open, then run
    import socket
    import threading
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)

    def serve():
        conn, _ = srv.accept()
        h, _a = ds.recv_msg(conn)
        assert h == {"op": "open", "model": "nanosam_encoder"}
        ds.send_msg(conn, {"input_names": ["image"], "output_names": ["image_embeddings"]})
        h, arrs = ds.recv_msg(conn)
        ds.send_msg(conn, {}, {"image_embeddings": arrs["image"][:, :1] * 2})
        conn.close()
    t = threading.Thread(target=serve, daemon=True)
    t.start()
    s = ds.open_session("models/nanosam_encoder.onnx", "remote", {"host": "127.0.0.1", "port": srv.getsockname()[1]})
    assert s.input_names == ["image"]
    out = s.run({"image": np.ones((1, 3, 2, 2), np.float32)})
    assert out["image_embeddings"].shape == (1, 1, 2, 2) and float(out["image_embeddings"].max()) == 2.0
    t.join(2)
    srv.close()


def test_remote_session_says_how_to_start_the_server():
    with pytest.raises(RuntimeError, match="jetson.sh seg start"):
        ds.RemoteSession("yolo_world", port=1, timeout_s=0.5)
