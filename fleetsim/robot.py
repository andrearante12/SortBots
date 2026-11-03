"""One simulated robot: its own map, planner, local layer, task FSM and the
real fleet task auction (nodes/fleet_tasks.py FleetTasks, in-process).

What is real and what stands in:

  REAL      the auction, leases, commit, retry and restart rules —
            FleetTasks + commsim/core/protocol.py, imported, not copied.
  STAND-IN  task_manager.py: same states (IDLE -> NAV_TO_PICKUP -> PICKING ->
            NAV_TO_DROPOFF -> PLACING -> DONE / FAILED / CANCELLED), same
            fixed dwell, same status JSON fields FleetTasks reads.
  STAND-IN  Nav2: global A* on the robot's own belief (planner.py), a path
            follower that drives the smoothed path exactly (rotating in
            place at sharp turns), and a local layer that stops for what the
            robot can see or hear about, replans around it after
            `block_replan_s`, and gives up after `block_fail_s`.
  STAND-IN  RTAB-Map + obstacle_detector: sensing.py.

Peers are known three ways, as on the robots: seen in the camera cone,
heard in their /fleet/status (pose, at the status rate, subject to the
network), and as obstacle notices a peer shared. Never from ground truth.
"""
from __future__ import annotations

import json
import math
import random
import sys
from collections import deque
from pathlib import Path

import numpy as np

from .belief import Belief
from .planner import INF, Costmap, path_length
from .sensing import Sensor

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "nodes"))
import fleet_tasks as ft  # noqa: E402

ACTIVE = {"NAV_TO_PICKUP", "PICKING", "NAV_TO_DROPOFF", "PLACING"}


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def seg_dist(p, a, b) -> float:
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


class PathOracle:
    """Stands in for nodes/grid_dist.GridDistance under FleetTasks: bids
    measure drive distance with the SAME planner and belief the robot will
    drive on, so a bid and the drive it promises agree. Cached per costmap
    version; beyond `straight_beyond_m` (the base bid radius — a robot
    that far only bids in a widened rebid round) the straight line stands,
    because one A* per robot per announce is the dominant cost at scale."""

    def __init__(self, robot: "Robot", straight_beyond_m: float, enabled: bool):
        self.r = robot
        self.beyond = straight_beyond_m
        self.enabled = enabled
        self._cache: dict = {}
        self._ver = -1

    def path_length(self, a, b) -> float | None:
        if not self.enabled or math.dist(a, b) > self.beyond:
            return None
        cm = self.r.cost
        if cm.version != self._ver:
            self._cache.clear()
            self._ver = cm.version
        g = cm.grid
        key = (g.index(*a), g.index(*b))
        if key not in self._cache:
            p = cm.plan(a, b, goal_tol_m=self.r.nav["goal_tolerance_m"],
                        max_expansions=self.r.nav["max_expansions"])
            self._cache[key] = None if p is None else path_length(p)
        return self._cache[key]


class Robot:
    def __init__(self, sim, k: int, home: tuple, prior):
        self.sim = sim
        self.k = k
        self.node = k + 1            # network node id == FleetTasks wire id
        self.rid = f"robot_{k}"
        cfg = sim.cfg
        self.nav = cfg["nav"]
        self.rcfg = cfg["robots"]
        self.body_r = float(self.rcfg["footprint_radius_m"])
        self.vmax = float(self.rcfg["max_speed_mps"])
        self.accel = float(self.rcfg["accel_mps2"])
        self.yaw_rate = float(self.rcfg["yaw_rate_rps"])
        self.home = tuple(home)
        self.x, self.y, self.yaw, self.v = float(home[0]), float(home[1]), 0.0, 0.0
        self.alive = True
        self.rng = random.Random(sim.seed * 7919 + k)
        g = sim.grid
        self.belief = Belief(g, float(cfg["map_share"]["tile_m"]), prior)
        self.cost = Costmap(g, self.belief.state, self.nav, self.body_r)
        self.sensor = Sensor(g, cfg["sensing"])
        self._new_core()
        # navigation
        self.goal = None
        self.purpose = None
        self.path: list[tuple[float, float]] = []
        self.path_idx: list[int] = []
        self.seg = 0
        self.plan_failed_since = None
        self.blocked_since = None
        self.blocked_by = None
        self.block_replan_at = None
        self.next_replan_ok = 0.0
        self.next_periodic = 0.0
        self.replan_due = None
        self._retry_n = 0
        self.pause_until = 0.0
        self.overlay: list[tuple[float, np.ndarray]] = []
        # task FSM (task_manager stand-in)
        self.queue: deque = deque()
        self.task = None
        self.state = "IDLE"
        self.dwell_until = None
        self.idle_since = 0.0
        # perception
        self.dets: list[tuple] = []       # (kind, ident, x, y, t, vx, vy)
        self._det_prev: dict = {}         # (kind, ident) -> (x, y, t), for velocity
        self.remote_obs: list[dict] = []  # obstacle notices from peers
        self.bumps: list[tuple] = []      # where it physically bumped into something
        self._last_share = -math.inf
        self.resync_wait: dict = {}       # (origin, tile) -> time a resync may go again
        # schedules, phase-shifted per robot so the fleet doesn't tick in lockstep
        ph = self.rng.random()
        self.next_sense = ph / float(cfg["sensing"]["hz"])
        self._scan_pose = None
        self.next_status = ph / float(cfg["status"]["hz"])
        self.next_map_pub = ph * float(cfg["map_share"]["publish_s"])
        self.next_refresh = float(cfg["map_share"]["refresh_s"]) * (0.5 + ph)
        self.refresh_ptr = 0
        self.tokens = 0.0
        self.counters = {"replans": 0, "replan_sensed": 0, "replan_shared": 0,
                         "replan_blocked": 0, "plan_fail": 0, "blocked_s": 0.0,
                         "stuck": 0, "shared_changed": 0}

    def _new_core(self) -> None:
        sim = self.sim
        self.core = ft.FleetTasks(self.rid, sim.params, sim.waypoints, cfg=sim.ft_cfg, now=sim.now)
        self.core._wall = lambda: 0.0
        self.core._next_id = 0
        self.core.grid = PathOracle(self, float(sim.params.bid_radius_m),
                                    sim.cfg["tasks"]["bid_cost"] == "path")
        self.core.set_pose((self.x, self.y), sim.now)
        bat = sim.ft_cfg["battery"].get("sim_start_pct")
        if bat is not None:
            self.core.battery_pct = float(bat)
        # task_manager's latched task_status reaches a starting fleet_tasks
        # before any dispatch can; FleetTasks reads that first status for
        # restart adoption, and would otherwise "adopt" our first real task
        # by cancelling it.
        self.core.on_task_status(json.dumps({"task_id": None, "state": "IDLE",
                                             "queue_depth": 0, "queue": []}), sim.now)
        self.core.drain()

    # ---- the FleetTasks seam ----------------------------------------------

    def core_call(self, fn, *a, **kw):
        fn(*a, **kw)
        self.sim.handle_actions(self, self.core.drain())

    def mode(self) -> str:
        if not self.alive:
            return "dead"
        if self.state in ACTIVE:
            return "task"
        return "home" if self.purpose == "home" else "idle"

    # ---- task FSM -----------------------------------------------------------

    def dispatch(self, payload: dict) -> None:
        names = (payload["pickup"], payload["dropoff"])
        if any(n not in self.sim.waypoints for n in names):
            self._status(rejected={"task_id": payload["task_id"], "reason": "unknown waypoint"})
            return
        self.queue.append(payload)
        self.sim.record({"s": "dispatch", "rid": self.rid, "task_id": payload["task_id"],
                         "pickup": payload["pickup"], "dropoff": payload["dropoff"]})
        if self.purpose == "home":
            self._clear_goal()
        self._status()

    def cancel(self, which: str) -> None:
        if which == "all":
            self.queue.clear()
        else:
            self.queue = deque(t for t in self.queue if t["task_id"] != which)
        if self.task is not None and which in ("all", self.task["task_id"]):
            self._clear_goal()
            self._end_task("CANCELLED")
        else:
            self._status()

    def _set_state(self, state: str, **extra) -> None:
        self.state = state
        self._status(**extra)

    def _status(self, **extra) -> None:
        st = {"task_id": self.task["task_id"] if self.task else None, "state": self.state,
              "queue_depth": len(self.queue),
              "queue": [{"task_id": t["task_id"]} for t in self.queue], **extra}
        if self.task:
            st["task"] = {"task_id": self.task["task_id"], "pickup": self.task["pickup"],
                          "dropoff": self.task["dropoff"]}
            self.sim.record({"s": "task", "rid": self.rid, "task_id": self.task["task_id"],
                             "state": self.state})
        self.core_call(self.core.on_task_status, json.dumps(st), self.sim.now)

    def _end_task(self, final: str) -> None:
        self._set_state(final)
        self.task = None
        self.state = "IDLE"
        self.idle_since = self.sim.now
        self._status()

    def _fsm(self, now: float) -> None:
        if self.state in ("PICKING", "PLACING") and now >= self.dwell_until:
            if self.state == "PICKING":
                # One physical item per task: a second robot sent to the same
                # shelf (a reclaim during a split) finds it gone, and the
                # pick fails as task_manager's would.
                if not self.sim.take_item(self.task["task_id"], self.rid):
                    self.sim.record({"s": "nav", "rid": self.rid, "event": "empty_shelf",
                                     "task_id": self.task["task_id"]})
                    self._end_task("FAILED")
                    return
                self._set_state("NAV_TO_DROPOFF")
                self.set_goal(self.sim.waypoint_xy(self.task["dropoff"]), "task")
            else:
                self.sim.place_item(self.task["task_id"])
                self._end_task("DONE")
            return
        if self.state == "IDLE" and self.queue:
            self.task = self.queue.popleft()
            if self.purpose == "home":
                self._clear_goal()
            if self.task.get("carrying") and self.sim.item_at(self.task["task_id"]) == self.rid:
                self._set_state("NAV_TO_DROPOFF")
                self.set_goal(self.sim.waypoint_xy(self.task["dropoff"]), "task")
            else:
                self._set_state("NAV_TO_PICKUP")
                self.set_goal(self.sim.waypoint_xy(self.task["pickup"]), "task")
            return
        home_s = self.nav["return_home_s"]
        if (self.state == "IDLE" and self.goal is None and home_s is not None
                and now - self.idle_since >= float(home_s)
                and math.dist((self.x, self.y), self.home) > 0.5):
            self.set_goal(self.home, "home")

    def _nav_done(self, ok: bool) -> None:
        purpose = self.purpose
        self._clear_goal()
        if purpose == "home":
            self.idle_since = self.sim.now
            return
        if not ok:
            self._end_task("FAILED")
        elif self.state == "NAV_TO_PICKUP":
            self.dwell_until = self.sim.now + float(self.sim.cfg["tasks"]["dwell_s"])
            self._set_state("PICKING")
        elif self.state == "NAV_TO_DROPOFF":
            self.dwell_until = self.sim.now + float(self.sim.cfg["tasks"]["dwell_s"])
            self._set_state("PLACING")

    # ---- navigation ---------------------------------------------------------

    def _record_path(self) -> None:
        if self.sim.cfg["record"]["paths"]:
            self.sim.record({"s": "path", "rid": self.rid, "purpose": self.purpose,
                             "pts": [[round(x, 2), round(y, 2)] for x, y in self.path]})

    def _clear_goal(self) -> None:
        if self.path:
            self.path = []
            self._record_path()
        self.goal = self.purpose = None
        self.path, self.path_idx, self.seg = [], [], 0
        self.plan_failed_since = self.blocked_since = self.blocked_by = None
        self.block_replan_at = None
        self.replan_due = None
        self.v = 0.0

    def set_goal(self, xy, purpose: str) -> None:
        self._clear_goal()
        self.goal, self.purpose = tuple(xy), purpose
        self.next_periodic = self.sim.now + float(self.nav["replan_period_s"])
        self._replan("goal")

    def _replan(self, cause: str) -> bool:
        now = self.sim.now
        self.next_replan_ok = now + float(self.nav["min_replan_s"])
        self.replan_due = None
        path = self.cost.plan((self.x, self.y), self.goal,
                              goal_tol_m=float(self.nav["goal_tolerance_m"]),
                              max_expansions=int(self.nav["max_expansions"]))
        if cause != "goal":
            self.counters["replans"] += 1
            key = f"replan_{cause}"
            if key in self.counters:
                self.counters[key] += 1
            self.sim.record({"s": "nav", "rid": self.rid, "event": "replan", "cause": cause})
        if path is None:
            if self.path:
                self.path = []
                self._record_path()
            self.path, self.path_idx = [], []
            if self.plan_failed_since is None:
                self.plan_failed_since = now
                self.counters["plan_fail"] += 1
                self.sim.record({"s": "nav", "rid": self.rid, "event": "plan_failed",
                                 "cause": cause})
            return False
        self.plan_failed_since = None
        self._retry_n = 0
        self.path = [(self.x, self.y)] + self._smooth(path)[1:]
        g = self.sim.grid
        self.path_idx = [i for i in (g.index(*p) for p in path) if i is not None]
        self.seg = 0
        self._record_path()
        return True

    def _smooth(self, pts: list) -> list:
        """Line-of-sight pruning: drop a cell-centre while the straight line
        from the last kept point to the next stays out of lethal space and
        out of the expensive half of the inflation, so the shortcut keeps
        about the clearance A* chose instead of hugging the wall."""
        if len(pts) <= 2:
            return list(pts)
        cm, g = self.cost, self.sim.grid
        cap = 1.0 + 0.5 * cm.weight
        step = 0.5 * g.res

        def clear(a, b):
            n = max(1, int(math.dist(a, b) / step))
            for i in range(1, n):
                t = i / n
                j = g.index(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
                if j is None or cm.mult[j] > cap:
                    return False
            return True

        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not clear(pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def path_blocked_in_belief(self) -> bool:
        mult = self.cost.mult
        return any(mult[i] == INF for i in self.path_idx)

    def on_belief_change(self, changed: np.ndarray, cause: str) -> None:
        if not len(changed):
            return
        self.cost.update_cells(changed)
        if self.goal is not None and self.path and self.path_blocked_in_belief():
            self.replan_due = cause if self.replan_due is None else self.replan_due

    def _ahead(self, dist_m: float) -> list:
        """The remaining path from here, cut at dist_m."""
        pts = [(self.x, self.y)]
        left = dist_m
        for p in self.path[self.seg + 1:]:
            d = math.dist(pts[-1], p)
            if d >= left:
                f = left / max(d, 1e-9)
                pts.append((pts[-1][0] + f * (p[0] - pts[-1][0]),
                            pts[-1][1] + f * (p[1] - pts[-1][1])))
                break
            pts.append(p)
            left -= d
        return pts

    def obstacles(self, now: float) -> list[tuple]:
        """(x, y, radius, kind, ident) the local layer avoids: own camera
        detections, peers from Status, peers' obstacle notices."""
        out = []
        ttl = float(self.nav["detection_ttl_s"])
        ahead = float(self.nav["predict_s"])
        for kind, ident, x, y, t, vx, vy in self.dets:
            if now - t <= ttl:
                r = self.body_r if kind == "robot" else self.sim.actor_radius(ident)
                # Where it is now, and where it will be predict_s from now:
                # obstacle_detector's tracks carry a velocity, and a walker
                # crossing at 1.2 m/s covers a body width between two scans.
                dt = now - t
                out.append((x + vx * dt, y + vy * dt, r, kind, ident))
                if ahead > 0 and (vx or vy):
                    out.append((x + vx * (dt + ahead), y + vy * (dt + ahead), r, kind, ident))
        bump_ttl = float(self.nav["bump_ttl_s"])
        self.bumps = [b for b in self.bumps if now - b[2] <= bump_ttl]
        for x, y, _t in self.bumps:
            out.append((x, y, 0.1, "bump", -1))
        sim = self.sim
        st_ttl = float(sim.cfg["status"]["ttl_s"])
        fresh = np.flatnonzero((now - sim.seen_t[self.k] <= st_ttl))
        if len(fresh):
            xy = sim.seen_xy[self.k, fresh]
            near = np.hypot(xy[:, 0] - self.x, xy[:, 1] - self.y) < 4.0
            for j, (px, py) in zip(fresh[near].tolist(), xy[near].tolist()):
                if j != self.k:
                    out.append((px, py, self.body_r, "robot", j))
        keep = []
        for o in self.remote_obs:
            if now <= o["expires"]:
                keep.append(o)
                # Extrapolate at most 1 s: a walker turns around, and a notice
                # carried 3 s along a stale velocity put phantom obstacles
                # where no one was (walkers scenario, 2026-11-02).
                dt = min(now - o["t"], 1.0)
                out.append((o["x"] + o["vx"] * dt, o["y"] + o["vy"] * dt, o["r"], "notice", o["id"]))
        self.remote_obs = keep
        return out

    def _blocking(self, now: float):
        stop = float(self.nav["stop_distance_m"])
        ahead = self._ahead(stop)
        if len(ahead) < 2:
            return None
        hx, hy = math.cos(self.yaw), math.sin(self.yaw)
        margin = float(self.nav["clearance_margin_m"])
        # Skip the first few cm of the path: something right beside the robot
        # that is not in the way forward must not pin it in place.
        start = ahead[0]
        if len(ahead) >= 2 and math.dist(ahead[0], ahead[1]) > 0.1:
            f = 0.1 / math.dist(ahead[0], ahead[1])
            start = (ahead[0][0] + f * (ahead[1][0] - ahead[0][0]),
                     ahead[0][1] + f * (ahead[1][1] - ahead[0][1]))
        segs = [start] + ahead[1:]
        for ox, oy, r, kind, ident in self.obstacles(now):
            if math.hypot(ox - self.x, oy - self.y) > stop + r + self.body_r + 0.5:
                continue
            if (ox - self.x) * hx + (oy - self.y) * hy < -0.05:
                continue  # behind: driving on only gains distance
            need = self.body_r + r + margin
            here = math.hypot(ox - self.x, oy - self.y)
            closest = min(seg_dist((ox, oy), a, b) for a, b in zip(segs[:-1], segs[1:]))
            # In the way only if the path ahead comes within `need` AND closer
            # than the robot already is: a peer that stopped right beside or
            # in front of us must not pin us when the new path leads away.
            if closest < need and closest < here - 0.02:
                return (ox, oy, r, kind, ident)
        return None

    def _cluster(self, blk, now: float, link_m: float = 1.8, limit: int = 8) -> list:
        """The blocking obstacle plus every obstacle within link_m of one
        already taken (transitively), nearest first, at most `limit`."""
        obs = [(o[0], o[1], o[2]) for o in self.obstacles(now)
               if math.hypot(o[0] - self.x, o[1] - self.y) < 6.0]
        taken = [(blk[0], blk[1], blk[2])]
        grew = True
        while grew and len(taken) < limit:
            grew = False
            for o in obs:
                if o in taken:
                    continue
                if any(math.hypot(o[0] - t[0], o[1] - t[1]) <= link_m for t in taken):
                    taken.append(o)
                    grew = True
                    if len(taken) >= limit:
                        break
        return taken

    def _add_overlay(self, ox: float, oy: float, r: float, now: float) -> None:
        g = self.sim.grid
        c = g.cell(ox, oy)
        if c is None:
            return
        idx = []
        for dx, dy, _d in g.disc_offsets(r):
            nx, ny = c[0] + dx, c[1] + dy
            if 0 <= nx < g.width and 0 <= ny < g.height:
                idx.append(ny * g.width + nx)
        idx = np.array(idx, dtype=np.int64)
        self.cost.overlay.reshape(-1)[idx] = True
        self.cost.update_cells(idx)
        self.overlay.append((now + float(self.nav["overlay_ttl_s"]), idx))

    def _expire_overlay(self, now: float) -> None:
        if not self.overlay or self.overlay[0][0] > now:
            return
        keep = [(t, i) for t, i in self.overlay if t > now]
        gone = np.concatenate([i for t, i in self.overlay if t <= now])
        flat = self.cost.overlay.reshape(-1)
        flat[gone] = False
        for _t, i in keep:
            flat[i] = True
        self.overlay = keep
        self.cost.update_cells(gone)

    def _drive(self, dt: float, now: float) -> None:
        if self.goal is None:
            return
        nav = self.nav
        if not self.path:
            if self.plan_failed_since is not None and \
                    now - self.plan_failed_since >= float(nav["plan_fail_s"]):
                self.sim.record({"s": "nav", "rid": self.rid, "event": "gave_up", "cause": "no_path"})
                self._nav_done(False)
            elif now >= self.next_replan_ok:
                # A failed search floods the whole reachable floor; back off
                # 0.5, 1, 2, 4 s between retries instead of paying that
                # every min_replan_s.
                self._replan("retry")
                if not self.path:
                    self._retry_n += 1
                    self.next_replan_ok = now + min(4.0, float(nav["min_replan_s"]) * 2 ** self._retry_n)
            return
        if self.replan_due is not None and now >= self.next_replan_ok:
            self._replan(self.replan_due)
            if not self.path:
                return
        elif now >= self.next_periodic:
            self.next_periodic = now + float(nav["replan_period_s"])
            if now >= self.next_replan_ok:
                self._replan("periodic")
                if not self.path:
                    return
        end = self.path[-1]
        if math.dist((self.x, self.y), end) <= 0.05 or self.seg >= len(self.path) - 1:
            ok = math.dist((self.x, self.y), self.goal) <= float(nav["goal_tolerance_m"]) + 0.05
            self._nav_done(ok)
            return
        if now < self.pause_until:
            self.v = 0.0
            return
        rate = float(nav["hiccup_rate_hz"])
        if rate > 0 and self.rng.random() < rate * dt:
            self.pause_until = now + float(nav["hiccup_s"])
            self.sim.record({"s": "nav", "rid": self.rid, "event": "recovery"})
            self.v = 0.0
            return
        blk = self._blocking(now)
        if blk is not None:
            self.v = 0.0
            if self.blocked_since is None:
                self.blocked_since, self.blocked_by = now, blk[3]
                self.sim.record({"s": "nav", "rid": self.rid, "event": "blocked", "by": blk[3]})
            self.counters["blocked_s"] += dt
            waited = now - self.blocked_since
            if waited >= float(nav["block_fail_s"]):
                self.counters["stuck"] += 1
                self.sim.record({"s": "nav", "rid": self.rid, "event": "gave_up", "cause": "blocked"})
                self._nav_done(False)
                return
            # Replan around it. Facing a peer, the lower-priority robot (higher
            # id, fleet_radio's order) goes first; the other waits a second
            # interval, so two robots don't swerve into each other in step.
            first = blk[3] != "robot" or blk[4] < self.k
            due = float(nav["block_replan_s"]) * (1 if first else 2)
            if self.block_replan_at is None:
                self.block_replan_at = now + due
            if now >= self.block_replan_at and now >= self.next_replan_ok:
                self.block_replan_at = now + due  # still blocked after this: wait again
                # Mark the blocker and whatever is chained to it: a row of
                # parked peers has to be planned around as a row, or the new
                # path just runs into the next one.
                for ox, oy, r in self._cluster(blk, now):
                    self._add_overlay(ox, oy, r, now)
                self._replan("blocked")
            return
        if self.blocked_since is not None:
            self.sim.record({"s": "nav", "rid": self.rid, "event": "unblocked",
                             "after_s": round(now - self.blocked_since, 2)})
            self.blocked_since = self.blocked_by = self.block_replan_at = None
        # Heading: turn toward the current segment; drive only when roughly
        # facing it (rotate in place at a sharp corner, as a diff drive does).
        a, b = (self.x, self.y), self.path[self.seg + 1]
        want = math.atan2(b[1] - a[1], b[0] - a[0])
        err = wrap(want - self.yaw)
        turn = max(-self.yaw_rate * dt, min(self.yaw_rate * dt, err))
        self.yaw = wrap(self.yaw + turn)
        err = wrap(want - self.yaw)
        left = path_length([a] + self.path[self.seg + 1:])
        v_cap = min(self.vmax, math.sqrt(2 * self.accel * left))
        target = 0.0 if abs(err) > math.radians(30) else v_cap * math.cos(err)
        self.v = min(target, self.v + self.accel * dt) if target > self.v else target
        step = self.v * dt
        x, y, seg = self.x, self.y, self.seg
        while step > 1e-9 and seg < len(self.path) - 1:
            nxt = self.path[seg + 1]
            d = math.dist((x, y), nxt)
            if d <= step:
                x, y, step, seg = nxt[0], nxt[1], step - d, seg + 1
            else:
                f = step / d
                x, y, step = x + f * (nxt[0] - x), y + f * (nxt[1] - y), 0.0
        if (x, y) != (self.x, self.y):
            if self.sim.try_move(self, x, y):
                self.seg = seg
                # Turned past a corner: face the next segment over the
                # following steps rather than snapping.
            else:
                self.v = 0.0

    # ---- sensing --------------------------------------------------------------

    def sense(self, now: float) -> None:
        sim = self.sim
        free, occ, dets = self.sensor.scan(self.x, self.y, self.yaw, sim.static_flat,
                                           sim.robot_owner_flat, sim.actor_owner_flat, self.node)
        changed = self.belief.observe(free, occ, now)
        self.on_belief_change(changed, "sensed")
        out = []
        for kind, ident, x, y in dets:
            prev = self._det_prev.get((kind, ident))
            vx = vy = 0.0
            if prev is not None and 0 < now - prev[2] <= 1.0:
                vx, vy = (x - prev[0]) / (now - prev[2]), (y - prev[1]) / (now - prev[2])
            self._det_prev[(kind, ident)] = (x, y, now)
            out.append((kind, ident, x, y, now, vx, vy))
        self.dets = out
        if sim.cfg["obstacle_share"]["enabled"]:
            self._share_movers(now)

    def _share_movers(self, now: float) -> None:
        """Movers the camera sees that are NOT a peer this robot knows from
        Status (on the robots, dynamic_obstacle_filter tells peers apart the
        same way: by their self-reported pose) go out as obstacle notices."""
        cfg = self.sim.cfg["obstacle_share"]
        st_ttl = float(self.sim.cfg["status"]["ttl_s"])
        known = self.sim.seen_xy[self.k][now - self.sim.seen_t[self.k] <= st_ttl]
        for kind, ident, x, y, t, vx, vy in self.dets:
            if len(known) and np.min(np.hypot(known[:, 0] - x, known[:, 1] - y)) < 0.6:
                continue
            if now - self._last_share < float(cfg["min_interval_s"]):
                continue
            self._last_share = now
            r = self.sim.actor_radius(ident) if kind == "actor" else self.body_r
            self.sim.share_obstacle(self, {"x": x, "y": y, "vx": vx, "vy": vy, "r": r,
                                           "t": now, "expiry_s": float(cfg["expiry_s"]),
                                           "id": f"{self.rid}:{kind}{ident}"})

    def on_obstacle(self, o: dict, now: float) -> None:
        self.remote_obs.append({**o, "expires": o["t"] + o["expiry_s"]})

    # ---- one step ---------------------------------------------------------------

    def step(self, dt: float, now: float) -> None:
        if not self.alive:
            return
        self._expire_overlay(now)
        if now >= self.next_sense:
            # A robot standing still sees the same static world: scan it at
            # idle_hz (still enough to notice a peer or mover coming).
            pose = (round(self.x, 3), round(self.y, 3), round(self.yaw, 3))
            still = pose == self._scan_pose and self.goal is None
            sc = self.sim.cfg["sensing"]
            self.next_sense = now + 1.0 / float(sc["idle_hz"] if still else sc["hz"])
            self._scan_pose = pose
            self.sense(now)
        self.core.set_pose((self.x, self.y), now)
        self.core_call(self.core.tick, now)
        self._fsm(now)
        self._drive(dt, now)
        if self.core.battery_pct is not None:
            self.core.drive(dt, self.v > 0.02)
