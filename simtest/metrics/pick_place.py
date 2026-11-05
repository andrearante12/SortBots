"""pick_place — did the package get picked, carried and set down where asked.

Scored from ground truth, never from what the robot says about itself: a pick
"succeeds" when the sim's magnet actually held the package (`gt.packages[id].held_by`
names the robot), and a place when the package is at rest, not held, at the spot the
task asked for. A robot that reports `placed` over the wrong spot fails here; so does
one that picks the wrong box (`false_picks`) or lets go in mid-air (`drops`).

Perception is scored on the same footing: the controller's detections (the `detect`
stream, plate centre + outward normal in the map frame) are matched against the true
plates, so a failed pick can be told apart as "never saw it" or "saw it, couldn't
reach it".

Reads the task's `package` and `place` from its own `dispatch` event, so a re-score
needs nothing but the recorded events.
"""
from __future__ import annotations

import math
import statistics

from .. import magnet
from . import Metric, MetricError, register


def _mean(xs):
    return round(statistics.fmean(xs), 4) if xs else None


@register
class PickPlace(Metric):
    name = "pick_place"
    version = 1
    streams = {"gt", "dispatch", "task", "detect"}
    summary = "Package picks and places, scored from ground truth; plate detection accuracy."
    params = {
        "place_tol_m": (float, 0.15, "a package is PLACED when it rests within this of the "
                                     "asked spot (xy)"),
        "place_z_tol_m": (float, 0.05, "and within this of the surface height asked for"),
        "detect_match_m": (float, 0.30, "a detected plate is the TRUE one when its centre is "
                                        "within this of it"),
    }
    values = {
        "planned": "package tasks the test asks for",
        "picked": "tasks whose package the robot's magnet actually held at some point",
        "placed": "tasks whose package came to rest, not held, at the asked spot",
        "success_rate": "placed / planned",
        "drops": "times a package was let go anywhere but its asked spot",
        "false_picks": "packages the robot picked that no dispatched task asked for",
        "mean_time_to_pick_s": "mean sim seconds from dispatch to the magnet holding the package",
        "max_place_error_m": "worst xy distance from the asked spot, over tasks that were picked",
        "max_place_z_error_m": "worst height error of the resting package, same tasks",
        "detected": "packages whose plate was detected at least once (right place, not while "
                    "held): the tasks' packages, or every package in the world when the "
                    "test has no package tasks",
        "held_at_end": "pick-only tasks (place: hold) whose package was still on the magnet at the end",
        "false_detections": "detected plates that matched no true plate",
        "mean_detect_err_m": "mean distance of a matched detection from the true plate centre",
        "mean_normal_err_deg": "mean angle of a matched detection's normal from the true normal",
        "first_detect_s": "sim seconds from t0 to the first matched detection",
    }

    def __init__(self, ctx, **params):
        super().__init__(ctx, **params)
        self.planned = [t for t in ctx.tasks if t.get("package")]
        self.specs = {p["id"]: p for p in (ctx.world or {}).get("packages", [])}
        # A test with no package tasks is a perception test (drive past the packages,
        # see the plates): `detected` then counts every package in the world.
        self.targets = {t["package"] for t in self.planned} or set(self.specs)
        self.tasks: dict[str, dict] = {}
        self.pk: dict[str, dict] = {}
        self.false_picks = 0
        self.drops = 0
        self.matched: set[str] = set()
        self.false_detections = 0
        self.det_err: list[float] = []
        self.det_ang: list[float] = []
        self.first_detect = None

    def check_context(self):
        if not self.specs:
            raise MetricError("pick_place: the test's world has no packages")
        missing = sorted({t["package"] for t in self.planned} - set(self.specs))
        if missing:
            raise MetricError(f"pick_place: package(s) {missing} are not in the test's world")

    # -- events -------------------------------------------------------------------
    def on_event(self, ev):
        s = ev["s"]
        if s == "dispatch":
            if ev.get("package"):
                self.tasks[ev["task_id"]] = {
                    "rid": ev["rid"], "package": ev["package"], "place": ev["place"],
                    "sent_at": ev["t"], "picked_at": None, "state": "SENT"}
        elif s == "task":
            rec = self.tasks.get(ev.get("task_id"))
            if rec is not None and rec["rid"] == ev.get("rid"):
                rec["state"] = ev.get("state")
        elif s == "gt":
            self._on_gt(ev)
        elif s == "detect":
            self._on_detect(ev)

    def _task_for(self, pid, rid, *, picked: bool):
        """The task a package event belongs to: before its pick, the earliest task
        still waiting for that package; after it, the most recently picked one."""
        recs = [r for r in self.tasks.values() if r["package"] == pid and r["rid"] == rid
                and (r["picked_at"] is not None) == picked]
        if not recs:
            return None
        return max(recs, key=lambda r: r["picked_at"]) if picked else \
            min(recs, key=lambda r: r["sent_at"])

    def _on_gt(self, ev):
        for pid, p in (ev.get("packages") or {}).items():
            prev = self.pk.get(pid)
            held = p.get("held_by")
            was = bool(prev and prev["held_by"])
            if held and not was:
                rec = self._task_for(pid, held, picked=False)
                if rec is None:
                    self.false_picks += 1
                else:
                    rec["picked_at"] = ev["t"]
            elif was and not held:
                rec = self._task_for(pid, prev["held_by"], picked=True)
                if rec is not None and (rec["place"] is None or  # a pick-only task never lets go
                                        self._xy_err(p, rec["place"]) > self.p["place_tol_m"]):
                    self.drops += 1
            self.pk[pid] = {"held_by": held, "pose": p}

    @staticmethod
    def _xy_err(pose, place) -> float:
        return math.hypot(pose["x"] - place["x"], pose["y"] - place["y"])

    def _on_detect(self, ev):
        truths = []
        for pid, st in self.pk.items():
            spec = self.specs.get(pid)
            if spec is None or st["held_by"]:
                continue
            centre, normal = magnet.magnet_face_pose(st["pose"], spec["size"], spec["magnet_face"])
            truths.append((pid, centre, normal))
        for pl in ev.get("plates", []):
            c = (pl["x"], pl["y"], pl["z"])
            best = min(truths, key=lambda t: math.dist(c, t[1]), default=None)
            if best is None or math.dist(c, best[1]) > self.p["detect_match_m"]:
                self.false_detections += 1
                continue
            pid, centre, normal = best
            self.matched.add(pid)
            self.det_err.append(math.dist(c, centre))
            n = (pl.get("nx", 0.0), pl.get("ny", 0.0), pl.get("nz", 0.0))
            norm = math.sqrt(sum(v * v for v in n)) or 1.0
            cosang = sum(a * b for a, b in zip(n, normal)) / norm
            self.det_ang.append(math.degrees(math.acos(max(-1.0, min(1.0, cosang)))))
            if self.first_detect is None:
                self.first_detect = ev["t"]

    # -- result -------------------------------------------------------------------
    def _outcome(self, rec) -> dict:
        st = self.pk.get(rec["package"])
        picked = rec["picked_at"] is not None
        out = {"picked": picked, "placed": False, "held": False, "xy_err_m": None, "z_err_m": None}
        if picked and st is not None and rec["place"] is None:
            out["held"] = bool(st["held_by"])
        elif picked and st is not None:
            out["xy_err_m"] = self._xy_err(st["pose"], rec["place"])
            out["z_err_m"] = abs(st["pose"]["z"] - rec["place"]["z"])
            out["placed"] = (not st["held_by"] and out["xy_err_m"] <= self.p["place_tol_m"]
                             and out["z_err_m"] <= self.p["place_z_tol_m"])
        return out

    def result(self):
        outs = [(rec, self._outcome(rec)) for rec in self.tasks.values()]
        picked = [o for _, o in outs if o["picked"]]
        placed = sum(o["placed"] for _, o in outs)
        held = sum(o["held"] for _, o in outs)
        xy = [o["xy_err_m"] for o in picked if o["xy_err_m"] is not None]
        z = [o["z_err_m"] for o in picked if o["z_err_m"] is not None]
        times = [rec["picked_at"] - rec["sent_at"] for rec, o in outs if o["picked"]]
        planned = len(self.planned)
        return {
            "planned": planned,
            "picked": len(picked),
            "placed": placed,
            "held_at_end": held,
            "success_rate": round((placed + held) / planned, 3) if planned else None,
            "drops": self.drops,
            "false_picks": self.false_picks,
            "mean_time_to_pick_s": _mean(times),
            "max_place_error_m": round(max(xy), 4) if xy else None,
            "max_place_z_error_m": round(max(z), 4) if z else None,
            "detected": len(self.matched & self.targets),
            "false_detections": self.false_detections,
            "mean_detect_err_m": _mean(self.det_err),
            "mean_normal_err_deg": _mean(self.det_ang),
            "first_detect_s": self.first_detect,
        }

    def detail(self):
        return {"tasks": {tid: {"package": rec["package"], "state": rec["state"],
                                "picked_at_s": rec["picked_at"], **self._outcome(rec)}
                          for tid, rec in self.tasks.items()}}
