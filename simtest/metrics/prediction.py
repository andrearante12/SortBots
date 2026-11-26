"""prediction — how well the obstacle tracker follows movers and foresees where they go.

Scores nodes/obstacle_detector.py's /<rid>/obstacle_tracks (the `trk`
stream: tracks with Kalman position, velocity, covariance and `pred`
look-ahead points, all transformed into the map frame by the probe) against
ground truth, frame by frame:

  * an actor is VISIBLE by the same range/field-of-view rule as `detection`
    (no occlusion, so recall is a lower bound);
  * a visible actor is ASSOCIATED with the nearest track within match_m;
  * for an associated actor, position and velocity error come from the
    track state, and PREDICTION error from each `pred` point against where
    the actor truly is that many seconds later — which is exact, because
    actor paths are analytic (simtest.world.pose_at), not re-read from a
    noisy stream: ADE is the mean over every horizon, FDE the mean at the
    last one;
  * MOVING recall is the share of frames in which a visible, truly moving
    actor had a track flagged `moving`;
  * a frame is a FALSE MOVING when a moving track sits far from every moving
    actor and every other robot (peers really do move);
  * FRAGMENTATION is how many extra track ids one actor collected per
    episode of continuous visibility.
"""
from __future__ import annotations

import math

from simtest import world as wl

from . import Metric, MetricError, register


@register
class Prediction(Metric):
    name = "prediction"
    version = 1
    streams = {"trk", "gt"}
    summary = "Tracker association, velocity and look-ahead error vs analytic ground truth."
    params = {
        "robots": (list, None, "robots whose tracker to score (default: all)"),
        "actors": (list, None, "actors to score against (default: all)"),
        "max_range_m": (float, 4.0, "visibility range — the detector's max_range_m"),
        "min_range_m": (float, 0.35, "closer than this the depth camera sees nothing"),
        "fov_margin_deg": (float, 5.0, "shrink the camera FOV by this per side"),
        "match_m": (float, 0.6, "track-to-actor-centre distance that counts as the same thing"),
        "peer_m": (float, 0.8, "a moving track this close to another robot's true pose is that robot"),
        "gap_s": (float, 1.0, "visibility gap that ends an episode"),
    }
    values = {
        "frames": "obstacle_tracks messages scored (robot-frames)",
        "visible_frames": "robot-actor-frames with the actor in view",
        "assoc_frames": "of those, how many had a track within match_m",
        "assoc_recall": "assoc_frames / visible_frames",
        "moving_visible_frames": "visible frames in which the actor was truly moving",
        "moving_recall": "share of those in which its track was flagged moving",
        "pos_rmse_m": "RMS track-to-actor centre error over associated frames",
        "vel_rmse_mps": "RMS track-velocity error over associated frames where the actor moves",
        "ade_m": "mean prediction error over every horizon (associated, actor moving)",
        "fde_m": "mean prediction error at the last horizon",
        "horizon_s": "the last horizon the tracker predicted to",
        "cv_fde_m": "what a naive constant-velocity extrapolation of the TRUE actor state scores at "
                    "that horizon — the floor any predictor on fixed paths should approach",
        "fragmentation": "extra track ids per visibility episode (0 = one id per episode)",
        "episodes": "visibility episodes",
        "false_moving_frames": "frames with a moving track far from every moving actor and robot",
        "false_moving_rate": "false_moving_frames / frames",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.actors = {a["id"]: a for a in ctx.actors(self.p["actors"])}
        self.half_fov = ctx.camera_hfov_rad / 2.0 - math.radians(self.p["fov_margin_deg"])
        self.gt = None
        self.frames = 0
        self.visible = 0
        self.assoc = 0
        self.mov_vis = 0
        self.mov_hit = 0
        self.pos_se: list[float] = []
        self.vel_se: list[float] = []
        self.pred_err: list[float] = []
        self.fde: list[float] = []
        self.cv_fde: list[float] = []
        self.horizon = None
        self.false_moving = 0
        self.episodes: dict[tuple, dict] = {}
        self.n_episodes = 0
        self.extra_ids = 0

    def check_context(self):
        if not self.actors:
            raise MetricError("prediction: the world has no actors")

    def _close(self, key):
        ep = self.episodes.pop(key, None)
        if ep is None:
            return
        self.n_episodes += 1
        self.extra_ids += max(0, len(ep["ids"]) - 1)

    def _in_view(self, rp, ap) -> bool:
        dx, dy = ap["x"] - rp["x"], ap["y"] - rp["y"]
        rng = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) - rp["yaw"]
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        return (self.p["min_range_m"] <= rng <= self.p["max_range_m"]
                and abs(bearing) <= self.half_fov)

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
        # World-clock time of this frame: the latest ground truth's wt, advanced
        # by however much later this message is (both are sim seconds).
        wt = self.gt["wt"] + (t - self.gt["t"])
        tracks = ev.get("tracks") or []
        used: set = set()
        moving_truth = []   # (x, y) of every truly moving actor + every other robot
        for aid, a in self.actors.items():
            ap = self.gt["actors"].get(aid)
            if ap is None:
                continue
            if ap.get("moving"):
                moving_truth.append((ap["x"], ap["y"], self.p["match_m"]))
            key = (rid, aid)
            in_view = self._in_view(rp, ap)
            ep = self.episodes.get(key)
            if ep is not None and not in_view and t - ep["last"] > self.p["gap_s"]:
                self._close(key)
                ep = None
            if not in_view:
                continue
            self.visible += 1
            if ep is None:
                ep = self.episodes[key] = {"last": t, "ids": set()}
            ep["last"] = t
            best, best_d = None, math.inf
            for tr in tracks:
                d = math.hypot(tr["pos"][0] - ap["x"], tr["pos"][1] - ap["y"])
                if d < best_d:
                    best, best_d = tr, d
            if ap.get("moving"):
                self.mov_vis += 1
            if best is None or best_d > self.p["match_m"]:
                continue
            self.assoc += 1
            used.add(id(best))
            ep["ids"].add(best["id"])
            self.pos_se.append(best_d * best_d)
            if ap.get("moving"):
                if best.get("moving"):
                    self.mov_hit += 1
                vx, vy = best.get("vel") or (0.0, 0.0)
                self.vel_se.append((vx - ap["vx"]) ** 2 + (vy - ap["vy"]) ** 2)
                preds = best.get("pred") or []
                for i, (h, px, py) in enumerate(preds):
                    truth = wl.pose_at(a["path"], wt + h)
                    err = math.hypot(px - truth["x"], py - truth["y"])
                    self.pred_err.append(err)
                    if i == len(preds) - 1:
                        self.fde.append(err)
                        self.horizon = h
                        # The naive baseline from the TRUE state: where constant
                        # velocity from the actor's real pose and speed lands.
                        self.cv_fde.append(math.hypot(
                            ap["x"] + ap["vx"] * h - truth["x"],
                            ap["y"] + ap["vy"] * h - truth["y"]))
        for other, op in self.gt["robots"].items():
            if other != rid:
                moving_truth.append((op["x"], op["y"], self.p["peer_m"]))
        stray = any(tr.get("moving") and id(tr) not in used
                    and all(math.hypot(tr["pos"][0] - x, tr["pos"][1] - y) > r
                            for x, y, r in moving_truth)
                    for tr in tracks)
        if stray:
            self.false_moving += 1

    def result(self):
        for key in list(self.episodes):
            self._close(key)

        def rms(v):
            return round(math.sqrt(sum(v) / len(v)), 3) if v else None

        def mean(v):
            return round(sum(v) / len(v), 3) if v else None
        return {
            "frames": self.frames,
            "visible_frames": self.visible,
            "assoc_frames": self.assoc,
            "assoc_recall": round(self.assoc / self.visible, 3) if self.visible else None,
            "moving_visible_frames": self.mov_vis,
            "moving_recall": round(self.mov_hit / self.mov_vis, 3) if self.mov_vis else None,
            "pos_rmse_m": rms(self.pos_se),
            "vel_rmse_mps": rms(self.vel_se),
            "ade_m": mean(self.pred_err),
            "fde_m": mean(self.fde),
            "horizon_s": self.horizon,
            "cv_fde_m": mean(self.cv_fde),
            "fragmentation": round(self.extra_ids / self.n_episodes, 3) if self.n_episodes else None,
            "episodes": self.n_episodes,
            "false_moving_frames": self.false_moving,
            "false_moving_rate": round(self.false_moving / self.frames, 3) if self.frames else None,
        }
