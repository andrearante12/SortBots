"""Which objects to segment this frame, and remembering them after they stop.

nodes/dynamic_segment.py runs an open-vocabulary detector and SAM; this module
is the part that decides, with no model and no ROS, so tests/seg_tracks_test.py
can pin it:

  * select_prompts — detector boxes, the depth filter's motion and map-conflict
    blobs and live tracks become SAM prompts:
      - `always` classes (person, forklift, ...) on sight;
      - `if_moving` classes (box, cart, pallet, ...) only when a blob or a
        confirmed track backs them. Masking every box would delete shelves of
        stock from the map, and with them the features localization needs;
      - a blob no prompt covers is an unknown mover: segmented anyway, which
        is what makes this generalize past the class list;
      - a confirmed track nothing re-found this frame is re-prompted from its
        predicted box until track_ttl_s passes without support.
  * TrackSet — associates each accepted mask with a track by box overlap. A
    track is SUPPORTED by a detection or a blob, never by its own re-prompt, so
    a stopped unknown object stays masked for track_ttl_s and then becomes map,
    while a stopped person the detector still sees stays masked indefinitely.
    That is the "walks, stops, walks on" case frame differencing alone maps
    twice.

Boxes are [u0, v0, u1, v1) in full-resolution image pixels throughout.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

DEFAULTS = {
    "always": ["person", "forklift", "pallet jack", "robot"],
    "if_moving": ["box", "cart", "pallet", "trolley", "bin"],
    "score_min": 0.25,
    "blob_iou": 0.05,         # a blob backs a box at this overlap...
    "blob_inside": 0.5,       # ...or when this much of the blob is inside it
    "covered_iou": 0.3,       # a blob a prompt already covers adds no prompt
    "track_iou": 0.3,
    "dedupe_iou": 0.5,
    "max_prompts": 8,
    "track_ttl_s": 10.0,
    # A SAM mask must mostly lie inside the box that prompted it, and must not
    # cover most of the image: a box prompt on a person leaning on a shelf can
    # come back as the whole shelf, and that would delete the shelf from SLAM.
    "mask_in_box_min": 0.6,
    "box_margin_px": 16,
    "mask_frac_max": 0.4,
}

# Lower sorts first when prompts compete for max_prompts.
PRIORITY = {"always": 0, "track": 1, "if_moving": 2, "motion": 3, "conflict": 3}


@dataclass
class Prompt:
    bbox: list[float]
    reason: str                     # always | if_moving | motion | conflict | track
    cls: str | None = None
    score: float = 1.0
    track_id: int | None = None
    point: list[float] | None = None  # optional foreground point (blob centroid)


@dataclass
class Track:
    id: int
    bbox: list[float]
    cls: str | None
    reason: str
    first_s: float
    support_s: float
    depth_m: float | None = None
    center_odom: list[float] | None = None
    hits: int = 1
    history: list[str] = field(default_factory=list)


def iou(a, b) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    if inter <= 0:
        return 0.0
    area = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / area if area > 0 else 0.0


def inside_frac(inner, outer) -> float:
    """Fraction of `inner`'s area inside `outer`."""
    ix = max(0.0, min(inner[2], outer[2]) - max(inner[0], outer[0]))
    iy = max(0.0, min(inner[3], outer[3]) - max(inner[1], outer[1]))
    area = (inner[2] - inner[0]) * (inner[3] - inner[1])
    return ix * iy / area if area > 0 else 0.0


def _backed(box, blobs, c) -> bool:
    return any(iou(box, b["bbox"]) >= c["blob_iou"] or inside_frac(b["bbox"], box) >= c["blob_inside"]
               for b in blobs)


def select_prompts(dets: list[dict], blobs: list[dict], tracks: list[Track],
                   predicted: dict[int, list[float]] | None = None,
                   cfg: dict | None = None) -> list[Prompt]:
    """dets: [{bbox, cls, score}]; blobs: [{bbox, source, centroid?}];
    predicted: {track_id: bbox} where tracks are expected this frame (their
    last box when there is no pose)."""
    c = {**DEFAULTS, **(cfg or {})}
    always, moving = set(c["always"]), set(c["if_moving"])
    predicted = predicted or {}
    boxes = {t.id: predicted.get(t.id, t.bbox) for t in tracks}
    prompts: list[Prompt] = []
    for d in dets:
        if d.get("score", 1.0) < c["score_min"]:
            continue
        box = list(d["bbox"])
        if d["cls"] in always:
            prompts.append(Prompt(box, "always", d["cls"], d.get("score", 1.0)))
        elif d["cls"] in moving:
            tracked = any(iou(box, tb) >= c["track_iou"] for tb in boxes.values())
            if tracked or _backed(box, blobs, c):
                prompts.append(Prompt(box, "if_moving", d["cls"], d.get("score", 1.0)))
    for b in blobs:
        if any(iou(b["bbox"], p.bbox) >= c["covered_iou"] or inside_frac(b["bbox"], p.bbox) >= 0.8
               for p in prompts):
            continue
        prompts.append(Prompt(list(b["bbox"]), b.get("source", "motion"),
                              point=b.get("centroid")))
    for t in tracks:
        tb = boxes[t.id]
        if any(iou(tb, p.bbox) >= c["track_iou"] for p in prompts):
            continue
        prompts.append(Prompt(list(tb), "track", t.cls, track_id=t.id))
    prompts.sort(key=lambda p: (PRIORITY.get(p.reason, 9), -p.score))
    kept: list[Prompt] = []
    for p in prompts:
        if all(iou(p.bbox, k.bbox) < c["dedupe_iou"] for k in kept):
            kept.append(p)
    return kept[: int(c["max_prompts"])]


def mask_bbox(mask: np.ndarray) -> list[float] | None:
    vs, us = np.nonzero(mask)
    if us.size == 0:
        return None
    return [float(us.min()), float(vs.min()), float(us.max() + 1), float(vs.max() + 1)]


def accept_mask(mask: np.ndarray, prompt_box, cfg: dict | None = None) -> bool:
    """Reject the masks that would delete static structure (see DEFAULTS)."""
    c = {**DEFAULTS, **(cfg or {})}
    n = int(mask.sum())
    if n == 0 or n > c["mask_frac_max"] * mask.size:
        return False
    m = c["box_margin_px"]
    h, w = mask.shape
    u0, v0 = max(0, int(prompt_box[0] - m)), max(0, int(prompt_box[1] - m))
    u1, v1 = min(w, int(np.ceil(prompt_box[2] + m))), min(h, int(np.ceil(prompt_box[3] + m)))
    return mask[v0:v1, u0:u1].sum() >= c["mask_in_box_min"] * n


class TrackSet:
    def __init__(self, cfg: dict | None = None):
        self.c = {**DEFAULTS, **(cfg or {})}
        self.tracks: list[Track] = []
        self._next = 1

    def update(self, results: list[tuple[Prompt, list[float], dict]], now: float) -> list[Track]:
        """results: (prompt, accepted mask bbox, extra) per accepted mask, where
        extra may carry depth_m and center_odom. Returns the live tracks."""
        used: set[int] = set()
        for prompt, box, extra in results:
            track = None
            if prompt.track_id is not None:
                track = next((t for t in self.tracks if t.id == prompt.track_id), None)
            if track is None:
                best, best_iou = None, self.c["track_iou"]
                for t in self.tracks:
                    if t.id in used:
                        continue
                    o = iou(box, t.bbox)
                    if o >= best_iou:
                        best, best_iou = t, o
                track = best
            if track is None and prompt.reason == "track":
                # Its track expired in the meantime. Re-creating it from its
                # own re-prompt would make an unsupported track immortal.
                continue
            if track is None:
                track = Track(self._next, list(box), prompt.cls, prompt.reason, now, now)
                self._next += 1
                self.tracks.append(track)
            else:
                track.bbox = list(box)
                track.hits += 1
                if prompt.cls and not track.cls:
                    track.cls = prompt.cls
                if prompt.reason != "track":
                    track.support_s = now
                    track.reason = prompt.reason
            track.depth_m = extra.get("depth_m", track.depth_m)
            track.center_odom = extra.get("center_odom", track.center_odom)
            used.add(track.id)
        self.tracks = [t for t in self.tracks if now - t.support_s <= self.c["track_ttl_s"]]
        return self.tracks


def predict_box(track: Track, K, R_cam_odom, t_cam_odom) -> list[float]:
    """Where a track's box should be in the current camera: its odom-frame
    centre projected through the current pose, scaled by the depth change.
    Falls back to the last box without a centre or when it projects behind."""
    if track.center_odom is None or track.depth_m is None:
        return track.bbox
    fx, fy, cx, cy = K
    p = np.asarray(R_cam_odom, float) @ np.asarray(track.center_odom, float) + np.asarray(t_cam_odom, float)
    if p[2] <= 0.1:
        return track.bbox
    u, v = fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy
    s = track.depth_m / p[2]
    hw = (track.bbox[2] - track.bbox[0]) * s / 2
    hh = (track.bbox[3] - track.bbox[1]) * s / 2
    return [u - hw, v - hh, u + hw, v + hh]
