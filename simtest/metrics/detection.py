"""detection — does perception flag moving actors as dynamic obstacles?

Scores nodes/dynamic_obstacle_filter.py's /<rid>/dynamic_obstacles cloud
(transformed into the map frame by the probe) against ground truth, frame by
frame:

  * an actor is VISIBLE to a robot in a frame when it is within max_range_m
    and inside the head camera's horizontal field of view (configs/sensors/
    d435.json), assuming the head looks straight ahead. Occlusion is NOT
    modelled — an actor behind a rack still counts as visible, which makes
    recall a lower bound. Keep actor paths in the open if that matters.
  * a visible actor is DETECTED when at least min_points cloud points lie
    within match_m of its footprint outline.
  * a frame has a FALSE POSITIVE when at least min_points points are farther
    than match_m from every actor (static structure flagged as moving —
    the failure that makes Nav2 swerve around nothing).

The cloud is in the map frame via TF, i.e. through the robot's own SLAM pose,
so localization error shows up here as missed detections. match_m is the
tolerance for that.

latency: for each episode of continuous visibility, sim seconds until the
first detected frame — how long the robot is blind to a newly visible mover.
"""
from __future__ import annotations

import math
import statistics

from simtest import geometry
from simtest import world as wl

from . import Metric, MetricError, register


@register
class Detection(Metric):
    name = "detection"
    version = 1
    streams = {"dyn", "gt"}
    summary = "Dynamic-obstacle recall, false-positive rate and latency vs ground truth."
    params = {
        "robots": (list, None, "robots whose perception to score (default: all)"),
        "actors": (list, None, "actors to score against (default: all)"),
        "max_range_m": (float, 4.0, "visibility range — Nav2's obstacle_max_range"),
        "min_range_m": (float, 0.35, "closer than this the depth camera sees nothing"),
        "fov_margin_deg": (float, 5.0, "shrink the camera FOV by this per side"),
        "match_m": (float, 0.5, "point-to-actor-outline tolerance for a detection"),
        "min_points": (int, 3, "points needed to call a detection / a false positive"),
        "gap_s": (float, 1.0, "visibility gap that ends a latency episode"),
    }
    values = {
        "visible_frames": "robot-actor-frame triples with the actor in view",
        "detected_frames": "of those, how many had the actor flagged",
        "recall": "detected_frames / visible_frames",
        "frames": "dynamic_obstacles messages scored",
        "fp_frames": "frames with min_points or more points far from every actor",
        "fp_rate": "fp_frames / frames",
        "mean_latency_s": "mean time from coming into view to first detection",
        "max_latency_s": "worst such latency (episodes never detected excluded — see recall)",
        "episodes": "visibility episodes",
        "undetected_episodes": "visibility episodes with no detection at all",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.actors = {a["id"]: a for a in ctx.actors(self.p["actors"])}
        self.half_fov = ctx.camera_hfov_rad / 2.0 - math.radians(self.p["fov_margin_deg"])
        self.gt = None
        self.frames = 0
        self.fp_frames = 0
        self.visible = 0
        self.detected = 0
        self.episodes: dict[tuple, dict] = {}   # (rid, aid) -> open episode
        self.latencies: list[float] = []
        self.n_episodes = 0
        self.undetected = 0

    def check_context(self):
        if not self.actors:
            raise MetricError("detection: the world has no actors")

    def _close(self, key):
        ep = self.episodes.pop(key, None)
        if ep is None:
            return
        self.n_episodes += 1
        if ep["detected_at"] is None:
            self.undetected += 1
        else:
            self.latencies.append(ep["detected_at"] - ep["start"])

    def on_event(self, ev):
        if ev["s"] == "gt":
            self.gt = ev
            return
        rid = ev["rid"]
        if self.gt is None or (self.p["robots"] and rid not in self.p["robots"]):
            return
        rp = self.gt["robots"].get(rid)
        if rp is None:
            return
        self.frames += 1
        t = ev["t"]
        pts = ev["pts"]
        shapes = {}
        for aid, a in self.actors.items():
            ap = self.gt["actors"].get(aid)
            if ap is not None:
                shapes[aid] = (ap, wl.footprint(a, ap["x"], ap["y"], ap["yaw"]))

        stray = sum(1 for p in pts
                    if all(not geometry.contains(s, p, self.p["match_m"])
                           for _, s in shapes.values()))
        if stray >= self.p["min_points"]:
            self.fp_frames += 1

        for aid, (ap, shape) in shapes.items():
            key = (rid, aid)
            dx, dy = ap["x"] - rp["x"], ap["y"] - rp["y"]
            rng = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx) - rp["yaw"]
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            in_view = (self.p["min_range_m"] <= rng <= self.p["max_range_m"]
                       and abs(bearing) <= self.half_fov)
            ep = self.episodes.get(key)
            if ep is not None and (not in_view and t - ep["last_seen"] > self.p["gap_s"]):
                self._close(key)
                ep = None
            if not in_view:
                continue
            self.visible += 1
            hit = sum(1 for p in pts if geometry.contains(shape, p, self.p["match_m"]))
            detected = hit >= self.p["min_points"]
            self.detected += detected
            if ep is None:
                ep = self.episodes[key] = {"start": t, "last_seen": t, "detected_at": None}
            ep["last_seen"] = t
            if detected and ep["detected_at"] is None:
                ep["detected_at"] = t

    def result(self):
        for key in list(self.episodes):
            self._close(key)
        return {
            "visible_frames": self.visible,
            "detected_frames": self.detected,
            "recall": round(self.detected / self.visible, 3) if self.visible else None,
            "frames": self.frames,
            "fp_frames": self.fp_frames,
            "fp_rate": round(self.fp_frames / self.frames, 3) if self.frames else None,
            "mean_latency_s": round(statistics.fmean(self.latencies), 2)
                              if self.latencies else None,
            "max_latency_s": round(max(self.latencies), 2) if self.latencies else None,
            "episodes": self.n_episodes,
            "undetected_episodes": self.undetected,
        }
