#!/usr/bin/env python3
"""Segment the movers in the head camera, so the depth filter can strip them.

    rgb ─► open-vocab detector (YOLO-World) ─┐
    dynamic_blobs (depth filter: motion,     ├─► seg_tracks.select_prompts ─► SAM ─► tracks
      map conflict) ─────────────────────────┘                                      │
                                                  /<id>/camera/dynamic_seg  ◄───────┘
                                                  (mono8, the rgb frame's stamp)

The depth filter (nodes/dynamic_obstacle_filter.py --dynamic-seg) unions this
mask into the pixels it NaNs out of depth_static, so RTAB-Map never paints a
person, cart or forklift into the map, moving or standing still. Why a
detector AND the depth cues: frame motion and map conflict are class-agnostic
but only see things while they move or right after they arrive; the detector's
text vocabulary sees people who have stood still all along. Which boxes count
is seg_tracks' job (an `if_moving` box needs motion behind it, or shelves of
stock would vanish from the map).

Backends, one interface (session.run(feeds) -> outputs):
  ort     onnxruntime, CPU or CUDA. Offline tests and the sim desktop.
  trt     TensorRT engines built on the Jetson (scripts/dynamic_seg_models.sh).
  oracle  --mode oracle: Isaac's instance segmentation of the world's actors
          instead of any model (spawn_warehouse.py --instance-seg). Separates
          the depth-filter half from model quality in the scored tests.

No torch anywhere: pre/post-processing is numpy + cv2, as in the rest of the
stack. Model I/O helpers are module-level and pure so
tests/dynamic_segment_test.py can check them against fake sessions.

    python3 nodes/dynamic_segment.py --robot-id robot_0 --platform real --mode model
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import seg_tracks  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "configs" / "dynamic_segment.yaml"

# SAM's pixel normalisation (ImageNet mean/std on 0-255).
SAM_MEAN = np.array([123.675, 116.28, 103.53], dtype=np.float32)
SAM_STD = np.array([58.395, 57.12, 57.375], dtype=np.float32)


def load_config(path: Path, platform: str) -> dict:
    """The YAML with its `sim:` or `real:` block laid over the top-level keys."""
    import yaml
    raw = yaml.safe_load(Path(path).read_text()) or {}
    cfg = {k: v for k, v in raw.items() if k not in ("sim", "real")}
    for k, v in (raw.get(platform) or {}).items():
        cfg[k] = v
    return cfg


def _resize(img: np.ndarray, w: int, h: int, nearest: bool = False) -> np.ndarray:
    import cv2
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_NEAREST if nearest else cv2.INTER_LINEAR)


# --- detector I/O -------------------------------------------------------------

def letterbox(rgb: np.ndarray, size: int):
    """Fit into size x size keeping aspect, grey padding centred (Ultralytics'
    convention). Returns (image, scale, (pad_x, pad_y))."""
    h, w = rgb.shape[:2]
    s = min(size / h, size / w)
    nw, nh = int(round(w * s)), int(round(h * s))
    px, py = (size - nw) // 2, (size - nh) // 2
    out = np.full((size, size, 3), 114, dtype=np.uint8)
    out[py:py + nh, px:px + nw] = _resize(rgb, nw, nh)
    return out, s, (px, py)


def yolo_preprocess(rgb: np.ndarray, size: int):
    img, s, pad = letterbox(rgb, size)
    blob = np.ascontiguousarray(img.transpose(2, 0, 1)[None].astype(np.float32) / 255.0)
    return blob, (s, pad, rgb.shape[:2])


def nms(boxes: np.ndarray, scores: np.ndarray, iou_thr: float) -> list[int]:
    order = np.argsort(-scores)
    keep = []
    while order.size:
        i = int(order[0])
        keep.append(i)
        rest = order[1:]
        xx0 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy0 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx1 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy1 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.clip(xx1 - xx0, 0, None) * np.clip(yy1 - yy0, 0, None)
        area = lambda b: (b[..., 2] - b[..., 0]) * (b[..., 3] - b[..., 1])  # noqa: E731
        o = inter / np.maximum(area(boxes[i]) + area(boxes[rest]) - inter, 1e-9)
        order = rest[o < iou_thr]
    return keep


def yolo_postprocess(out: np.ndarray, meta, classes: list[str], score_min: float,
                     iou_thr: float) -> list[dict]:
    """Ultralytics detect head output, (1, 4+nc, N) or (1, N, 4+nc), boxes as
    centre-xywh in letterboxed pixels and per-class scores already sigmoided,
    → [{bbox, cls, score}] in original-image pixels, class-wise NMS."""
    a = np.asarray(out, dtype=np.float32)[0]
    nc = len(classes)
    if a.shape[0] == 4 + nc and a.shape[1] != 4 + nc:
        a = a.T
    if a.shape[1] != 4 + nc:
        raise ValueError(f"detector output {out.shape} does not match {nc} classes")
    scores = a[:, 4:]
    cls = scores.argmax(axis=1)
    conf = scores[np.arange(len(a)), cls]
    sel = conf >= score_min
    if not sel.any():
        return []
    a, cls, conf = a[sel], cls[sel], conf[sel]
    s, (px, py), (h, w) = meta
    xy, wh = a[:, :2], a[:, 2:4]
    boxes = np.concatenate([xy - wh / 2, xy + wh / 2], axis=1)
    boxes[:, [0, 2]] = ((boxes[:, [0, 2]] - px) / s).clip(0, w)
    boxes[:, [1, 3]] = ((boxes[:, [1, 3]] - py) / s).clip(0, h)
    dets = []
    for k in np.unique(cls):
        idx = np.nonzero(cls == k)[0]
        for i in nms(boxes[idx], conf[idx], iou_thr):
            j = idx[i]
            dets.append({"bbox": boxes[j].tolist(), "cls": classes[int(k)], "score": float(conf[j])})
    return dets


# --- SAM I/O ------------------------------------------------------------------

def sam_preprocess(rgb: np.ndarray, size: int = 1024):
    """Longest side to `size`, normalised, zero-padded bottom/right (SAM's
    ResizeLongestSide). Returns (NCHW float32, scale)."""
    h, w = rgb.shape[:2]
    s = size / max(h, w)
    nw, nh = int(round(w * s)), int(round(h * s))
    img = (_resize(rgb, nw, nh).astype(np.float32) - SAM_MEAN) / SAM_STD
    out = np.zeros((size, size, 3), dtype=np.float32)
    out[:nh, :nw] = img
    return np.ascontiguousarray(out.transpose(2, 0, 1)[None]), s


def sam_prompt_arrays(prompt: seg_tracks.Prompt, scale: float):
    """Box corners (labels 2, 3) plus the blob centroid as a foreground point
    (label 1) when there is one, in the encoder's resized coordinates."""
    u0, v0, u1, v1 = prompt.bbox
    pts, labels = [[u0, v0], [u1, v1]], [2.0, 3.0]
    if prompt.point is not None:
        pts.insert(0, list(prompt.point))
        labels.insert(0, 1.0)
    coords = np.asarray(pts, dtype=np.float32)[None] * scale
    return coords, np.asarray(labels, dtype=np.float32)[None]


def sam_feeds(input_names: list[str], embedding: np.ndarray, coords, labels, orig_hw) -> dict:
    """Decoder feeds by input name: NanoSAM/MobileSAM decoders take five
    inputs, SAM's official ONNX export also wants orig_im_size."""
    feeds = {
        "image_embeddings": embedding,
        "point_coords": coords,
        "point_labels": labels,
        "mask_input": np.zeros((1, 1, 256, 256), dtype=np.float32),
        "has_mask_input": np.zeros((1,), dtype=np.float32),
        "orig_im_size": np.asarray(orig_hw, dtype=np.float32),
    }
    return {n: feeds[n] for n in input_names}


def sam_postprocess(outputs: dict, scale: float, orig_hw, size: int = 1024):
    """Decoder outputs by name → (bool mask at orig_hw, iou score)."""
    h, w = orig_hw
    iou = np.asarray(outputs.get("iou_predictions", [[1.0]]), dtype=np.float32).reshape(-1)
    best = int(iou.argmax())
    if "masks" in outputs and np.asarray(outputs["masks"]).shape[-2:] == (h, w):
        m = np.asarray(outputs["masks"])[0, min(best, np.asarray(outputs["masks"]).shape[1] - 1)]
        return m > 0, float(iou[best])
    low = np.asarray(outputs["low_res_masks"], dtype=np.float32)[0, best]
    up = _resize(low, size, size)
    nh, nw = int(round(h * scale)), int(round(w * scale))
    m = _resize(np.ascontiguousarray(up[:nh, :nw]), w, h)
    return m > 0, float(iou[best])


# --- backends -----------------------------------------------------------------

class OrtSession:
    def __init__(self, path: str):
        import onnxruntime as ort
        avail = ort.get_available_providers()
        prov = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider") if p in avail]
        self.s = ort.InferenceSession(str(path), providers=prov)
        self.input_names = [i.name for i in self.s.get_inputs()]
        self.output_names = [o.name for o in self.s.get_outputs()]

    def run(self, feeds: dict) -> dict:
        return dict(zip(self.output_names, self.s.run(self.output_names, feeds)))


class TrtSession:
    """A TensorRT 10 engine behind OrtSession's interface. Device buffers are
    reallocated only when a shape changes (the decoder's point count does)."""

    def __init__(self, path: str):
        import tensorrt as trt
        try:
            from cuda.bindings import runtime as cudart
        except ImportError:  # cuda-python < 12.6 layout
            from cuda import cudart
        self.trt, self.cudart = trt, cudart
        logger = trt.Logger(trt.Logger.WARNING)
        self.engine = trt.Runtime(logger).deserialize_cuda_engine(Path(path).read_bytes())
        if self.engine is None:
            raise RuntimeError(f"could not deserialize {path} (built for another TensorRT/GPU?)")
        self.ctx = self.engine.create_execution_context()
        names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        is_in = lambda n: self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT  # noqa: E731
        self.input_names = [n for n in names if is_in(n)]
        self.output_names = [n for n in names if not is_in(n)]
        self._buf: dict[str, tuple[int, int]] = {}   # name -> (ptr, nbytes)
        err, self.stream = cudart.cudaStreamCreate()
        self._check(err)

    def _check(self, err):
        if int(err) != 0:
            raise RuntimeError(f"CUDA error {err}")

    def _ptr(self, name: str, nbytes: int) -> int:
        ptr, size = self._buf.get(name, (0, 0))
        if size < nbytes:
            if ptr:
                self.cudart.cudaFree(ptr)
            err, ptr = self.cudart.cudaMalloc(nbytes)
            self._check(err)
            self._buf[name] = (ptr, nbytes)
        return self._buf[name][0]

    def run(self, feeds: dict) -> dict:
        cu, kind = self.cudart, self.cudart.cudaMemcpyKind
        for n in self.input_names:
            a = np.ascontiguousarray(feeds[n], dtype=self.trt.nptype(self.engine.get_tensor_dtype(n)))
            self.ctx.set_input_shape(n, a.shape)
            ptr = self._ptr(n, a.nbytes)
            self._check(cu.cudaMemcpyAsync(ptr, a.ctypes.data, a.nbytes,
                                           kind.cudaMemcpyHostToDevice, self.stream)[0])
            self.ctx.set_tensor_address(n, ptr)
        outs = {}
        for n in self.output_names:
            shape = tuple(self.ctx.get_tensor_shape(n))
            outs[n] = np.empty(shape, dtype=self.trt.nptype(self.engine.get_tensor_dtype(n)))
            self.ctx.set_tensor_address(n, self._ptr(n, outs[n].nbytes))
        if not self.ctx.execute_async_v3(self.stream):
            raise RuntimeError("TensorRT execute failed")
        for n, a in outs.items():
            self._check(cu.cudaMemcpyAsync(a.ctypes.data, self._buf[n][0], a.nbytes,
                                           kind.cudaMemcpyDeviceToHost, self.stream)[0])
        self._check(cu.cudaStreamSynchronize(self.stream)[0])
        return outs


def open_session(path: str, backend: str):
    """`path` names the ONNX file; trt opens the .engine built next to it."""
    p = Path(path) if Path(path).is_absolute() else REPO_ROOT / path
    if backend == "trt":
        return TrtSession(str(p.with_suffix(".engine")))
    return OrtSession(str(p))


class Detector:
    def __init__(self, session, classes: list[str], size: int, score_min: float, nms_iou: float):
        self.s, self.classes, self.size = session, list(classes), int(size)
        self.score_min, self.nms_iou = score_min, nms_iou

    def detect(self, rgb: np.ndarray) -> list[dict]:
        blob, meta = yolo_preprocess(rgb, self.size)
        out = self.s.run({self.s.input_names[0]: blob})
        return yolo_postprocess(next(iter(out.values())), meta, self.classes,
                                self.score_min, self.nms_iou)


class Segmenter:
    def __init__(self, encoder, decoder, size: int = 1024):
        self.enc, self.dec, self.size = encoder, decoder, int(size)
        self._emb = self._scale = self._hw = None

    def set_image(self, rgb: np.ndarray) -> None:
        x, self._scale = sam_preprocess(rgb, self.size)
        self._emb = next(iter(self.enc.run({self.enc.input_names[0]: x}).values()))
        self._hw = rgb.shape[:2]

    def segment(self, prompt: seg_tracks.Prompt):
        coords, labels = sam_prompt_arrays(prompt, self._scale)
        out = self.dec.run(sam_feeds(self.dec.input_names, self._emb, coords, labels, self._hw))
        return sam_postprocess(out, self._scale, self._hw, self.size)


# --- the per-frame step ---------------------------------------------------------

class Pipeline:
    """Detector + SAM + tracks for one frame. Model objects are injected, so
    the tests drive it with fakes."""

    def __init__(self, detector, segmenter, cfg: dict):
        self.det, self.seg = detector, segmenter
        keys = set(seg_tracks.DEFAULTS)
        self.cfg = {k: v for k, v in cfg.items() if k in keys}
        self.tracks = seg_tracks.TrackSet(self.cfg)

    def step(self, rgb: np.ndarray, blobs: list[dict], now: float, *,
             K=None, cam_from_odom=None, odom_from_cam=None, depth=None):
        """→ (bool mask HxW, info dict). Poses are (R, t) pairs; depth is the
        aligned depth image in metres, used to anchor tracks in odom."""
        h, w = rgb.shape[:2]
        dets = self.det.detect(rgb) if self.det is not None else []
        self.tracks.prune(now)
        predicted = {}
        if K is not None and cam_from_odom is not None:
            for t in self.tracks.tracks:
                predicted[t.id] = seg_tracks.predict_box(t, K, *cam_from_odom)
        prompts = seg_tracks.select_prompts(dets, blobs, self.tracks.tracks, predicted, self.cfg)
        mask = np.zeros((h, w), dtype=bool)
        results, rejected = [], 0
        if prompts:
            self.seg.set_image(rgb)
        for p in prompts:
            m, _score = self.seg.segment(p)
            if not seg_tracks.accept_mask(m, p.bbox, self.cfg):
                rejected += 1
                continue
            mask |= m
            box = seg_tracks.mask_bbox(m)
            results.append((p, box, _anchor(m, box, depth, K, odom_from_cam)))
        live = self.tracks.update(results, now)
        info = {
            "detections": len(dets), "prompts": len(prompts), "rejected": rejected,
            "tracks": [{"id": t.id, "cls": t.cls, "reason": t.reason,
                        "age_s": round(now - t.first_s, 1),
                        "unsupported_s": round(now - t.support_s, 1)} for t in live],
        }
        return mask, info


def _anchor(mask, box, depth, K, odom_from_cam) -> dict:
    """Median depth of the mask and its box centre in odom, for predict_box."""
    if depth is None or box is None or depth.shape != mask.shape:
        return {}
    z = depth[mask]
    z = z[np.isfinite(z) & (z > 0)]
    if z.size == 0:
        return {}
    d = float(np.median(z))
    out = {"depth_m": d}
    if K is not None and odom_from_cam is not None:
        fx, fy, cx, cy = K
        u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        p = np.array([(u - cx) * d / fx, (v - cy) * d / fy, d])
        R, t = odom_from_cam
        out["center_odom"] = (np.asarray(R) @ p + np.asarray(t)).tolist()
    return out


def oracle_mask(ids: np.ndarray, labels: dict, prefix: str = "/World/SimTest/actors/") -> np.ndarray:
    """Isaac instance-segmentation ids → mask of the world's actors. The labels
    table maps id (as a string) to a prim path or semantic label, depending on
    the Isaac release; either an actor path or an `actor:` label counts."""
    hit = [int(k) for k, v in labels.items()
           if (isinstance(v, str) and (v.startswith(prefix) or "actor:" in v))
           or (isinstance(v, dict) and any("actor:" in str(x) for x in v.values()))]
    return np.isin(ids, np.asarray(hit, dtype=ids.dtype)) if hit else np.zeros(ids.shape, bool)


def build_pipeline(cfg: dict) -> Pipeline:
    backend = cfg.get("backend", "ort")
    d, s = cfg["detector"], cfg["segmenter"]
    det = Detector(open_session(d["model"], backend), d["classes"], d["size"],
                   cfg.get("score_min", 0.25), cfg.get("nms_iou", 0.5))
    seg = Segmenter(open_session(s["encoder"], backend), open_session(s["decoder"], backend),
                    s.get("size", 1024))
    missing = (set(cfg.get("always", [])) | set(cfg.get("if_moving", []))) - set(d["classes"])
    if missing:
        raise ValueError(f"classes {sorted(missing)} are not in the exported vocabulary "
                         f"{d['classes']} — re-run scripts/dynamic_seg_models.sh export")
    return Pipeline(det, seg, cfg)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-id", default="robot_0")
    ap.add_argument("--platform", default=os.environ.get("SORTBOTS_PLATFORM", "sim"),
                    choices=("sim", "real"))
    ap.add_argument("--mode", choices=("model", "oracle"), default="model")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    a = ap.parse_args(argv)
    cfg = load_config(a.config, a.platform)

    import time

    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    from obstacle_cloud import quat_matrix
    from package_detector import rgb_from_image

    rid = a.robot_id
    odom_frame = f"{rid}/odom"
    pipeline = build_pipeline(cfg) if a.mode == "model" else None

    def stamp_s(st) -> float:
        return st.sec + st.nanosec * 1e-9

    class DynamicSegment(Node):
        def __init__(self):
            super().__init__(f"{rid}_dynamic_segment")
            qos = qos_profile_sensor_data   # a RELIABLE image sub kills Isaac's writers
            self._rgb = self._depth = self._info = None
            self._ids = None
            self._labels: dict = {}
            self._blobs: list[tuple[float, dict]] = []
            self._last_stamp = None
            self._tf = Buffer()
            self._tfl = TransformListener(self._tf, self)
            p = f"/{rid}/camera"
            if a.mode == "model":
                self.create_subscription(Image, f"{p}/rgb", lambda m: setattr(self, "_rgb", m), qos)
                self.create_subscription(Image, f"{p}/depth", lambda m: setattr(self, "_depth", m), qos)
                self.create_subscription(CameraInfo, f"{p}/camera_info",
                                         lambda m: setattr(self, "_info", m), qos)
                self.create_subscription(String, f"{p}/dynamic_blobs", self._on_blobs, qos)
            else:
                self.create_subscription(Image, f"{p}/instance_seg",
                                         lambda m: setattr(self, "_ids", m), qos)
                self.create_subscription(String, f"{p}/instance_seg_labels", self._on_labels, 10)
            self.pub = self.create_publisher(Image, f"{p}/dynamic_seg", qos)
            self.info_pub = self.create_publisher(String, f"/{rid}/dynamic_seg/info", 10)
            # Steady clock: under use_sim_time a node timer stretches by the
            # real-time factor (CLAUDE.md), and the model's budget is wall time.
            self.create_timer(1.0 / float(cfg.get("rate_hz", 5.0)), self._tick,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info(f"dynamic_segment {a.mode} ({cfg.get('backend', 'ort')}) "
                                   f"-> {p}/dynamic_seg at {cfg.get('rate_hz', 5.0)} Hz")

        def _on_blobs(self, msg: String):
            try:
                d = json.loads(msg.data)
            except ValueError:
                return
            for b in d.get("blobs", []):
                self._blobs.append((float(d.get("stamp", 0.0)), dict(b, source=d.get("source", "motion"))))
            self._blobs = self._blobs[-64:]

        def _on_labels(self, msg: String):
            try:
                self._labels = json.loads(msg.data)
            except ValueError:
                pass

        def _pose(self, target, source, stamp):
            for when in (Time.from_msg(stamp), Time()):
                try:
                    tf = self._tf.lookup_transform(target, source, when)
                    q, t = tf.transform.rotation, tf.transform.translation
                    return quat_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])
                except Exception:  # noqa: BLE001, S112 - tf2 raises several types; next stamp
                    continue
            return None

        def _publish(self, header, mask: np.ndarray, info: dict, t0: float):
            out = Image()
            out.header = header
            out.height, out.width = mask.shape
            out.encoding = "mono8"
            out.step = mask.shape[1]
            out.data = (mask.astype(np.uint8) * 255).tobytes()
            self.pub.publish(out)
            info.update({"stamp": stamp_s(header.stamp), "mode": a.mode,
                         "latency_ms": round((time.monotonic() - t0) * 1e3, 1),
                         "mask_frac": round(float(mask.mean()), 4)})
            self.info_pub.publish(String(data=json.dumps(info)))

        def _tick(self):
            t0 = time.monotonic()
            if a.mode == "oracle":
                m = self._ids
                if m is None or m.header.stamp == self._last_stamp:
                    return
                self._last_stamp = m.header.stamp
                dt = {"32SC1": np.int32, "32UC1": np.uint32}.get(m.encoding, np.uint32)
                ids = np.frombuffer(m.data, dtype=dt).reshape(m.height, m.width)
                self._publish(m.header, oracle_mask(ids, self._labels), {"tracks": []}, t0)
                return
            m = self._rgb
            if m is None or m.header.stamp == self._last_stamp:
                return
            self._last_stamp = m.header.stamp
            rgb = rgb_from_image(m.data, m.encoding, m.height, m.width)
            if rgb is None:
                return
            st = stamp_s(m.header.stamp)
            age = float(cfg.get("blob_max_age_s", 0.5))
            blobs = [b for s, b in self._blobs if abs(s - st) <= age]
            K = depth = cam_odom = odom_cam = None
            if self._info is not None:
                k = self._info.k
                K = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))
                odom_cam = self._pose(odom_frame, m.header.frame_id, m.header.stamp)
                if odom_cam is not None:
                    cam_odom = (odom_cam[0].T, -odom_cam[0].T @ odom_cam[1])
            d = self._depth
            if d is not None and (d.height, d.width) == (m.height, m.width):
                if d.encoding == "32FC1":
                    depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
                elif d.encoding in ("16UC1", "mono16"):
                    depth = np.frombuffer(d.data, dtype=np.uint16).reshape(d.height, d.width) * 0.001
            mask, info = pipeline.step(rgb, blobs, st, K=K, cam_from_odom=cam_odom,
                                       odom_from_cam=odom_cam, depth=depth)
            self._publish(m.header, mask, info, t0)

    rclpy.init(args=None)
    node = DynamicSegment()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
