#!/usr/bin/env python3
"""Frontier-based autonomous exploration for one SortBots robot.

Subscribes to the RTAB-Map-built `/{robot_id}/map` occupancy grid, finds
frontier cells (free cells bordering unknown space), clusters and scores them
by size vs distance, and drives Nav2's `navigate_to_pose` action toward the
best one — repeating until no reachable frontier remains. This is the
autonomous-mapping counterpart to nodes/task_manager.py's dispatched
pickup/dropoff FSM; the two arbitrate over the single navigate_to_pose action
server via `<robot>/explore_status` / `<robot>/explore_cmd` (see
task_manager.py's _on_explore_status / _on_dispatch).

Frontier detection needs REAL free space in `/map` to work at all — see
launch/sortbots_rtabmap_robot.launch.py's GRID_ARGS (Grid/RayTracing) and
docs/running.md's "Occupancy-grid tuning" section. Without it `/map` stays
almost entirely unknown and this node has nothing real to chase.

Multi-robot: explorers additionally publish/subscribe a SHARED (not
namespaced) `/explore/claims` topic so two robots don't both repeatedly send
goals into the same frontier cluster. This is deliberately NOT map fusion —
each robot still builds and owns its own RTAB-Map database; claim-sharing is
only a coordination signal for where NOT to plan next. True collaborative
SLAM (shared map, shared pose graph) is future work — see the plan.

No custom .msg/.srv package (see task_manager.py's docstring for why):
explore_status/explore_cmd/claims are all std_msgs/String carrying JSON,
matching the existing task_status/dispatch_task convention.

Run (system ROS 2 Jazzy sourced, NOT the Isaac venv or conda):

    source /opt/ros/jazzy/setup.bash
    python3 nodes/explorer.py --robot-id robot_0 --autostart

Control it manually instead of --autostart:

    ros2 topic pub /robot_0/explore_cmd std_msgs/String '{data: start}' --once
    ros2 topic pub /robot_0/explore_cmd std_msgs/String '{data: stop}' --once

Watch it work:

    ros2 topic echo /robot_0/explore_status
"""
from __future__ import annotations

import argparse
import json
import math
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "configs" / "explorer.yaml"

# RTAB-Map's /map is published latched (see sortbots_rtabmap_robot.launch.py /
# nav2_params.yaml's static_layer note) — match that QoS so a late-starting
# explorer still gets the current map immediately instead of waiting for the
# next update.
MAP_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

DEFAULTS = {
    "min_frontier_cells": 8,       # discard frontier clusters smaller than this
    "alpha": 1.5,                  # score = size / distance**alpha; higher = prefer nearby
    "inflate_radius_m": 0.4,       # matches nav2_params.yaml inflation_layer; keep goals off walls
    "blacklist_radius_m": 0.5,     # suppress candidates within this radius of a blacklisted point
    "blacklist_ttl_s": 120.0,      # forget a blacklist entry after this long (0 = never)
    "goal_timeout_s": 45.0,        # cancel + blacklist a goal that hasn't finished in this long
    "replan_period_s": 2.0,        # how often to look for a new/better frontier
    "done_after_empty_cycles": 5,  # consecutive empty replans before declaring exploration done
    "max_goal_distance_m": 8.0,    # ignore frontiers farther than this (bounds planning time)
    "occupied_thresh": 65,         # cell value >= this counts as an obstacle
    "claim_radius_m": 1.0,         # exclude frontiers this close to another robot's active claim
    "claim_ttl_s": 90.0,           # forget another robot's claim after this long
    "goal_standoff_max_m": 1.5,    # search radius for a pulled-back goal around a frontier cell
    "goal_clearance_m": 0.45,      # goal min distance from occupied cells (> inflation 0.4)
    "goal_unknown_clearance_m": 0.2,  # goal min distance from unknown cells
    "stuck_window_s": 15.0,        # stuck-watchdog sliding window
    "stuck_min_displacement_m": 0.15,  # less motion than this over the window => stuck
    "openness_radius_m": 0.8,      # window for the free-space fraction around a goal
    "openness_exp": 1.0,           # weight of openness in scoring; 0.0 disables the term
    "escape_after_failures": 2,    # consecutive failed goals before jumping to the farthest frontier
}


def load_config(path: Path | None) -> dict:
    cfg = dict(DEFAULTS)
    if path and path.exists():
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
        cfg.update(raw)
    return cfg


def _dilate4(mask: np.ndarray, iterations: int) -> np.ndarray:
    """Grow a boolean mask outward by `iterations` 4-connected steps.

    Cheap scipy/opencv-free morphological dilation for the inflation-radius
    check — grids here are a few hundred cells per side and iterations is
    single digits, so plain numpy shift-or is fast enough at replan cadence.
    """
    m = mask
    for _ in range(iterations):
        grown = m.copy()
        grown[:-1, :] |= m[1:, :]
        grown[1:, :] |= m[:-1, :]
        grown[:, :-1] |= m[:, 1:]
        grown[:, 1:] |= m[:, :-1]
        m = grown
    return m


class Frontier:
    """A connected cluster of frontier cells, in grid-index (row, col) space."""

    __slots__ = ("cells", "mean_iy", "mean_ix", "size")

    def __init__(self, cells: list[tuple[int, int]]):
        self.cells = cells
        self.mean_iy = sum(c[0] for c in cells) / len(cells)
        self.mean_ix = sum(c[1] for c in cells) / len(cells)
        self.size = len(cells)


def grid_masks(grid: np.ndarray, occupied_thresh: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(unknown, occupied, free) boolean masks for an occupancy `grid`.

    `grid` is (height, width) int16, values -1 (unknown) or 0..100 (occupancy
    %), matching nav_msgs/OccupancyGrid's row-major convention where
    increasing row = increasing world y and increasing col = increasing world
    x (same convention webui/app.js's worldToPixel/pixelToWorld assume).
    """
    unknown = grid < 0
    occupied = grid >= occupied_thresh
    free = (~unknown) & (~occupied)
    return unknown, occupied, free


def find_frontiers(
    unknown: np.ndarray, occupied: np.ndarray, free: np.ndarray, inflate_cells: int
) -> list[Frontier]:
    """Cluster frontier cells (free, adjacent to unknown) from grid_masks()."""
    has_unknown_neighbor = np.zeros_like(free)
    has_unknown_neighbor[:-1, :] |= unknown[1:, :]
    has_unknown_neighbor[1:, :] |= unknown[:-1, :]
    has_unknown_neighbor[:, :-1] |= unknown[:, 1:]
    has_unknown_neighbor[:, 1:] |= unknown[:, :-1]

    occ_inflated = _dilate4(occupied, inflate_cells) if inflate_cells > 0 else occupied
    frontier_mask = free & has_unknown_neighbor & (~occ_inflated)

    visited = np.zeros_like(frontier_mask)
    h, w = frontier_mask.shape
    frontiers: list[Frontier] = []
    ys, xs = np.nonzero(frontier_mask)
    for sy, sx in zip(ys.tolist(), xs.tolist()):
        if visited[sy, sx]:
            continue
        # BFS over 8-connected frontier cells. Pure Python, not vectorized:
        # grids here are a few hundred cells per side and this runs once per
        # replan tick, which is fast enough without scipy's labeled-components.
        cells: list[tuple[int, int]] = []
        q = deque([(sy, sx)])
        visited[sy, sx] = True
        while q:
            iy, ix = q.popleft()
            cells.append((iy, ix))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    ny, nx = iy + dy, ix + dx
                    if (
                        0 <= ny < h
                        and 0 <= nx < w
                        and frontier_mask[ny, nx]
                        and not visited[ny, nx]
                    ):
                        visited[ny, nx] = True
                        q.append((ny, nx))
        frontiers.append(Frontier(cells))
    return frontiers


def cell_to_world(iy: float, ix: float, info) -> tuple[float, float]:
    x = info.origin.position.x + (ix + 0.5) * info.resolution
    y = info.origin.position.y + (iy + 0.5) * info.resolution
    return x, y


class ExplorerNode(Node):
    def __init__(self, robot_id: str, cfg: dict, autostart: bool):
        super().__init__(f"{robot_id}_explorer")
        self.robot_id = robot_id
        self.cfg = cfg

        self.state = "exploring" if autostart else "stopped"
        self._map_info = None
        self._grid: np.ndarray | None = None
        self._empty_cycles = 0
        self._blacklist: list[list[float]] = []  # [x, y, monotonic_ts]
        self._other_claims: dict[str, tuple[float, float, float]] = {}  # robot_id -> (x, y, ts)

        self._goal_handle = None
        self._goal_target: tuple[float, float] | None = None
        self._goal_sent_at: float | None = None
        # (monotonic_ts, x, y) samples while a goal is active, for the stuck
        # watchdog in _tick(). Cleared on every new goal and on goal end.
        self._pose_history: deque[tuple[float, float, float]] = deque()
        # Consecutive goals that ended in stuck/timeout/abort. At
        # escape_after_failures, _plan_and_send switches to escape mode:
        # nearest-biased scoring has demonstrably trapped us in a dead-end
        # pocket (blacklist radius only kills one spot at a time, so the
        # next-nearest frontier in the SAME pocket wins again), and the way
        # out is the farthest frontier on the explored boundary instead.
        # Reset on any goal success.
        self._consec_failures = 0
        # Bumped on every _send_goal(); goal_response/nav_result callbacks
        # capture the generation they belong to and no-op if it's stale by
        # the time they fire. Necessary because Nav2's navigate_to_pose is a
        # single-goal action server: sending a new goal preempts whatever was
        # running, and the OLD goal's result future still completes (as
        # ABORTED/CANCELED) some time later. Without this guard, that late
        # callback reads self._goal_target — which by then already holds the
        # NEW goal — and misattributes the old failure to it, blacklisting a
        # target that's still actively navigating. Verified live: this is
        # what produced "every goal fails in ~2ms" symptoms even after fixing
        # the separate cancel-on-timeout race (see _cancel_active_goal).
        self._goal_generation = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, f"/{robot_id}/navigate_to_pose")

        self.create_subscription(OccupancyGrid, f"/{robot_id}/map", self._on_map, MAP_QOS)
        self.create_subscription(String, f"/{robot_id}/explore_cmd", self._on_cmd, 10)
        # Deliberately global, not /{robot_id}/-namespaced: every robot's
        # explorer needs to see every OTHER robot's claims.
        self.create_subscription(String, "/explore/claims", self._on_claim, 10)

        self.status_pub = self.create_publisher(String, f"/{robot_id}/explore_status", 10)
        self.frontier_pub = self.create_publisher(MarkerArray, f"/{robot_id}/frontiers", 10)
        self.claim_pub = self.create_publisher(String, "/explore/claims", 10)

        self.create_timer(cfg["replan_period_s"], self._tick)
        self.get_logger().info(
            f"explorer up for {robot_id} (autostart={autostart}); "
            f"min_frontier_cells={cfg['min_frontier_cells']} alpha={cfg['alpha']}"
        )
        self._publish_status()

    # -- map -------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid):
        self._map_info = msg.info
        self._grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)

    # -- external control (dashboard / task_manager) ----------------------

    def _on_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "start":
            if self.state != "exploring":
                self.get_logger().info("explore_cmd: start")
                self.state = "exploring"
                self._empty_cycles = 0
                self._consec_failures = 0
                self._publish_status()
        elif cmd == "stop":
            self.get_logger().info("explore_cmd: stop")
            self._cancel_active_goal()
            self.state = "stopped"
            self._publish_status()
        elif cmd == "pause":
            self.get_logger().info("explore_cmd: pause")
            self.state = "paused"
            self._publish_status()
        else:
            self.get_logger().warn(f"explore_cmd: unknown command {msg.data!r}")

    def _cancel_active_goal(self, cancel_on_server: bool = True):
        # cancel_on_server=False is for the timeout path: _tick() calls this
        # immediately before _plan_and_send() sends a REPLACEMENT goal in the
        # same tick, and Nav2's navigate_to_pose action server already
        # preempts on a new goal — no explicit cancel needed. Sending one
        # anyway is actively harmful: cancel_goal_async() is processed
        # asynchronously, so it reliably arrives at the server AFTER the new
        # goal has already preempted the old one, and bt_navigator's cancel
        # handling doesn't appear to verify the goal_id still matches — it
        # cancels whatever is CURRENTLY active, i.e. the brand new goal.
        # Verified live: every timeout was silently canceling its own
        # replacement within ~20ms, which read as "nearly every goal fails
        # instantly" until traced back to this. Reserve the real cancel for
        # explore_cmd=stop, where there is deliberately no replacement goal
        # about to preempt it.
        if cancel_on_server and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._goal_target = None
        self._goal_sent_at = None

    # -- frontier-claim sharing (the concrete slice of collaborative
    #    exploration this repo builds today — see module docstring) --------

    def _on_claim(self, msg: String):
        try:
            c = json.loads(msg.data)
            rid = c["robot_id"]
            if rid == self.robot_id:
                return
            self._other_claims[rid] = (float(c["x"]), float(c["y"]), time.monotonic())
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            pass

    def _publish_claim(self, x: float, y: float):
        msg = String()
        msg.data = json.dumps({"robot_id": self.robot_id, "x": x, "y": y})
        self.claim_pub.publish(msg)

    def _active_claims(self) -> list[tuple[float, float]]:
        ttl = self.cfg["claim_ttl_s"]
        now = time.monotonic()
        return [(x, y) for (x, y, ts) in self._other_claims.values() if now - ts < ttl]

    # -- blacklist ---------------------------------------------------------

    def _blacklist_point(self, x: float, y: float):
        self._blacklist.append([x, y, time.monotonic()])

    def _active_blacklist(self) -> list[tuple[float, float]]:
        ttl = self.cfg["blacklist_ttl_s"]
        now = time.monotonic()
        if ttl > 0:
            self._blacklist = [b for b in self._blacklist if now - b[2] < ttl]
        return [(b[0], b[1]) for b in self._blacklist]

    @staticmethod
    def _too_close(x: float, y: float, points: list[tuple[float, float]], radius: float) -> bool:
        return any(math.hypot(x - px, y - py) < radius for px, py in points)

    # -- robot pose ----------------------------------------------------------

    def _robot_pose(self) -> tuple[float, float] | None:
        try:
            tr = self.tf_buffer.lookup_transform(
                "map", f"{self.robot_id}/base_link", rclpy.time.Time()
            )
        except Exception:
            return None
        t = tr.transform.translation
        return (t.x, t.y)

    # -- planning loop ---------------------------------------------------

    def _tick(self):
        if self.state != "exploring":
            return
        if self._goal_handle is not None:
            if self._is_stuck():
                gx, gy = self._goal_target
                self.get_logger().warn(
                    f"stuck: moved <{self.cfg['stuck_min_displacement_m']}m in "
                    f"{self.cfg['stuck_window_s']:.0f}s en route to ({gx:.2f}, {gy:.2f}) "
                    f"— blacklisting + retargeting"
                )
                self._blacklist_point(gx, gy)
                self._cancel_active_goal(cancel_on_server=False)
                self._pose_history.clear()
                self._consec_failures += 1
            elif (
                self._goal_sent_at is not None
                and time.monotonic() - self._goal_sent_at > self.cfg["goal_timeout_s"]
            ):
                gx, gy = self._goal_target
                self.get_logger().warn(
                    f"goal to ({gx:.2f}, {gy:.2f}) timed out after "
                    f"{self.cfg['goal_timeout_s']:.0f}s — canceling + blacklisting"
                )
                self._blacklist_point(gx, gy)
                self._cancel_active_goal(cancel_on_server=False)
                self._consec_failures += 1
            else:
                self._publish_status()  # heartbeat while a goal is in flight
                return
        self._plan_and_send()

    def _is_stuck(self) -> bool:
        """True if the robot has barely moved over the sliding stuck window.

        Complements (does not replace) Nav2's own SimpleProgressChecker
        (0.5 m / 20 s): that one aborts a FollowPath attempt, which sends the
        BT into up to 6 clear/spin/wait/backup recovery rounds before the
        goal finally ABORTs back to us — multi-minute churn when the robot is
        genuinely wedged. This watchdog cuts that short. It firing DURING a
        Spin/BackUp recovery is intended, not a bug: 15 s without 0.15 m of
        net motion is exactly the corner-wedge case, and blacklist + pick a
        different frontier is the right answer. The cancel is
        cancel_on_server=False for the same reason as the timeout path — the
        replacement goal preempts, stale callbacks no-op via the generation
        counter.
        """
        pose = self._robot_pose()
        if pose is None:
            return False
        now = time.monotonic()
        self._pose_history.append((now, pose[0], pose[1]))
        window = self.cfg["stuck_window_s"]
        # Prune, but always keep one sample at age >= window as the anchor:
        # sampling happens at the replan tick (2 s), so dropping everything
        # older than the window outright would leave the oldest sample
        # perpetually ~1 tick younger than the window and the filled check
        # below would never pass.
        while len(self._pose_history) >= 2 and now - self._pose_history[1][0] >= window:
            self._pose_history.popleft()
        oldest = self._pose_history[0]
        if now - oldest[0] < window:
            return False  # window not filled yet
        max_disp = max(
            math.hypot(x - oldest[1], y - oldest[2]) for _, x, y in self._pose_history
        )
        return max_disp < self.cfg["stuck_min_displacement_m"]

    def _plan_and_send(self):
        if self._grid is None or self._map_info is None:
            return
        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn(
                f"no map->{self.robot_id}/base_link TF yet — can't plan",
                throttle_duration_sec=5.0,
            )
            return
        rx, ry = pose
        info = self._map_info
        res = info.resolution

        unknown, occupied, free = grid_masks(self._grid, self.cfg["occupied_thresh"])
        inflate_cells = max(0, round(self.cfg["inflate_radius_m"] / res))
        frontiers = find_frontiers(unknown, occupied, free, inflate_cells)
        frontiers = [f for f in frontiers if f.size >= self.cfg["min_frontier_cells"]]

        # Where a goal may actually be placed: known free space with clearance
        # from both obstacles and unknown, so Nav2 never has to drive INTO an
        # unmapped corner — the depth camera reveals the frontier just as well
        # from a standoff in open floor.
        clearance_cells = max(0, round(self.cfg["goal_clearance_m"] / res))
        unknown_clear_cells = max(0, round(self.cfg["goal_unknown_clearance_m"] / res))
        valid_goal = (
            free
            & ~_dilate4(occupied, clearance_cells)
            & ~_dilate4(unknown, unknown_clear_cells)
        )

        goals = [self._goal_point(f, info, valid_goal) for f in frontiers]

        blacklist = self._active_blacklist()
        claims = self._active_claims()
        open_cells = max(1, round(self.cfg["openness_radius_m"] / res))
        h, w = free.shape
        candidates = []  # (score, gx, gy, fx, fy)
        far = []  # (dist, gx, gy, fx, fy): valid but beyond max_goal_distance_m
        escape_pool = []  # (dist, gx, gy, fx, fy): every valid candidate, any distance
        for f, ((gx, gy), (fx, fy), (giy, gix)) in zip(frontiers, goals):
            dist = math.hypot(gx - rx, gy - ry)
            if self._too_close(gx, gy, blacklist, self.cfg["blacklist_radius_m"]):
                continue
            if self._too_close(gx, gy, claims, self.cfg["claim_radius_m"]):
                continue
            escape_pool.append((dist, gx, gy, fx, fy))
            if dist > self.cfg["max_goal_distance_m"]:
                far.append((dist, gx, gy, fx, fy))
                continue
            # Openness: fraction of known-free floor around the goal. Wide-open
            # frontiers beat tight-corner ones of similar size/distance —
            # corners are where the robot gets wedged. openness_exp 0.0 is the
            # documented off-switch (term becomes 1.0, old scoring exactly).
            patch = free[
                max(0, giy - open_cells) : min(h, giy + open_cells + 1),
                max(0, gix - open_cells) : min(w, gix + open_cells + 1),
            ]
            openness = float(patch.mean()) if patch.size else 0.0
            denom = max(dist, 0.05) ** self.cfg["alpha"]
            score = f.size * openness ** self.cfg["openness_exp"] / denom
            candidates.append((score, gx, gy, fx, fy))

        candidates.sort(key=lambda c: c[0], reverse=True)
        if self._consec_failures >= self.cfg["escape_after_failures"] and escape_pool:
            # Escape mode. Consecutive stuck/timeout/abort goals mean the
            # nearest-biased scoring is feeding us frontiers inside the same
            # unreachable dead-end pocket (blacklisting removes them one spot
            # at a time, 15+ s each). Stop nibbling at the pocket: jump to the
            # FARTHEST valid frontier on the explored boundary — by
            # construction the most distant edge of known space from wherever
            # we're wedged — and let Nav2 route back out through mapped
            # territory. max_goal_distance_m is deliberately ignored here.
            # Normal scoring resumes after the next goal success.
            escape_pool.sort(key=lambda c: c[0], reverse=True)
            _dist, gx, gy, fx, fy = escape_pool[0]
            self.get_logger().warn(
                f"escape: {self._consec_failures} consecutive failed goals — "
                f"jumping to farthest frontier ({gx:.2f}, {gy:.2f}), {_dist:.1f}m away"
            )
            candidates = [(0.0, gx, gy, fx, fy)]
        elif not candidates and far:
            # Backtracking out of a fully-mapped dead end. When the robot
            # finishes a deep pocket, every remaining frontier can be farther
            # than max_goal_distance_m — with a hard cap that read as "no
            # valid frontiers" and could end exploration with the map half
            # done. No DFS-style path memory is needed: frontier selection is
            # global over the whole map and Nav2's planner routes through
            # known free space, so "backtrack" is simply "goal = nearest
            # remaining frontier, wherever it is". The cap stays as a
            # preference (bounded planning) rather than a filter.
            far.sort(key=lambda c: c[0])
            _dist, gx, gy, fx, fy = far[0]
            self.get_logger().info(
                f"no frontiers within {self.cfg['max_goal_distance_m']:.0f}m — "
                f"backtracking to nearest remaining frontier at ({gx:.2f}, {gy:.2f})"
            )
            candidates = [(0.0, gx, gy, fx, fy)]
        best_xy = (candidates[0][1], candidates[0][2]) if candidates else None
        self._publish_frontier_markers([g[0] for g in goals], best=best_xy)

        if not candidates:
            self._empty_cycles += 1
            self.get_logger().info(
                f"no valid frontiers "
                f"({self._empty_cycles}/{self.cfg['done_after_empty_cycles']} empty cycles, "
                f"{len(frontiers)} raw clusters, {len(blacklist)} blacklisted, {len(claims)} claimed)"
            )
            if self._empty_cycles >= self.cfg["done_after_empty_cycles"]:
                self._finish()
            else:
                self._publish_status()
            return

        self._empty_cycles = 0
        _score, gx, gy, fx, fy = candidates[0]
        self._send_goal(gx, gy, fx, fy, rx, ry)

    def _goal_point(
        self, f: Frontier, info, valid_mask: np.ndarray
    ) -> tuple[tuple[float, float], tuple[float, float], tuple[int, int]]:
        """Standoff goal for one frontier cluster.

        Returns ((gx, gy), (fx, fy), (giy, gix)): the goal in world coords,
        the frontier look-at point in world coords, and the goal's grid cell.
        The goal is the valid_mask cell (known free, clear of obstacles AND
        unknown — see _plan_and_send) nearest the frontier within
        goal_standoff_max_m, i.e. pulled back into open mapped floor instead
        of sitting on the frontier edge itself, which is frequently a wall
        corner the robot then wedges itself into.
        """
        # The actual frontier cell nearest the cluster's centroid — guarantees
        # the look-at point lands on a real frontier cell, not a
        # possibly-invalid averaged point (clusters can be concave / L-shaped).
        fr_iy, fr_ix = min(
            f.cells, key=lambda c: (c[0] - f.mean_iy) ** 2 + (c[1] - f.mean_ix) ** 2
        )
        fxy = cell_to_world(fr_iy, fr_ix, info)

        half = max(0, round(self.cfg["goal_standoff_max_m"] / info.resolution))
        h, w = valid_mask.shape
        y0, x0 = max(0, fr_iy - half), max(0, fr_ix - half)
        ys, xs = np.nonzero(valid_mask[y0 : min(h, fr_iy + half + 1), x0 : min(w, fr_ix + half + 1)])
        if ys.size:
            i = int(np.argmin((ys + y0 - fr_iy) ** 2 + (xs + x0 - fr_ix) ** 2))
            giy, gix = int(ys[i]) + y0, int(xs[i]) + x0
        else:
            # No valid standoff cell nearby — fall back to the frontier cell
            # itself (pre-standoff behavior; find_frontiers' inflation check
            # already keeps it off inflated walls).
            giy, gix = fr_iy, fr_ix
        return cell_to_world(giy, gix, info), fxy, (giy, gix)

    def _send_goal(self, gx: float, gy: float, fx: float, fy: float, rx: float, ry: float):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("navigate_to_pose server not available")
            return
        # Face the frontier FROM THE GOAL — the camera should look at the
        # unknown space we're trying to reveal once the robot arrives. (The
        # old code aimed along robot->goal, a heading that's only right if
        # the approach happens to be a straight line.)
        if math.hypot(fx - gx, fy - gy) > 1e-3:
            yaw = math.atan2(fy - gy, fx - gx)
        else:
            yaw = math.atan2(gy - ry, gx - rx)  # goal == frontier cell fallback

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._goal_generation += 1
        gen = self._goal_generation
        target = (gx, gy)
        self._goal_target = target
        self._goal_sent_at = time.monotonic()
        self._pose_history.clear()  # fresh stuck window for the new goal
        self._publish_claim(gx, gy)
        self.get_logger().info(f"exploring -> ({gx:.2f}, {gy:.2f})")

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, gen, target)
        )
        self._publish_status()

    def _on_goal_response(self, future, gen: int, target: tuple[float, float]):
        if gen != self._goal_generation:
            return  # superseded by a newer goal before this one was even accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            # Do NOT blacklist on rejection. A rejection is Nav2 refusing to
            # even try — in practice bt_navigator still activating during
            # bringup (the action server exists before the BT is ready, so
            # wait_for_server() passes and the goal bounces). It says nothing
            # about the frontier itself. Blacklisting here killed two fresh
            # explore runs on 2026-08-01: the robot's very first frontier is
            # often its ONLY frontier, so reject -> blacklist -> 30 empty
            # cycles -> "exploration done" with the robot never having moved.
            # Clearing state lets _tick re-pick (usually the same) frontier on
            # the next 2s cycle; genuine navigation failures still get
            # blacklisted via the ABORTED path in _on_nav_result.
            self.get_logger().warn(
                f"frontier goal rejected (Nav2 not ready?): {target} — will retry"
            )
            self._goal_handle = None
            self._goal_target = None
            self._goal_sent_at = None
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_nav_result(f, gen, target)
        )

    def _on_nav_result(self, future, gen: int, target: tuple[float, float]):
        if gen != self._goal_generation:
            # A late result for a goal we've already moved past (preempted by
            # a newer goal, e.g. via the timeout path in _tick). Do NOT touch
            # self._goal_target/_goal_handle here — that state now belongs to
            # the newer goal, which may still be actively navigating.
            return
        status = future.result().status
        self._goal_handle = None
        self._goal_target = None
        self._goal_sent_at = None
        self._pose_history.clear()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"reached frontier {target}")
            self._consec_failures = 0
        else:
            self.get_logger().warn(f"frontier goal to {target} ended status={status} — blacklisting")
            self._blacklist_point(*target)
            self._consec_failures += 1
        self._publish_status()

    def _finish(self):
        self.get_logger().info("exploration done — no reachable frontiers left")
        self.state = "done"
        self._publish_status()

    # -- visualization + status -------------------------------------------

    def _publish_frontier_markers(self, goal_points: list[tuple[float, float]], best):
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (gx, gy) in enumerate(goal_points):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "frontiers"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = gx
            m.pose.position.y = gy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            is_best = best is not None and abs(gx - best[0]) < 1e-6 and abs(gy - best[1]) < 1e-6
            r = 0.12 if is_best else 0.07
            m.scale.x = m.scale.y = m.scale.z = r * 2
            m.color = ColorRGBA(
                r=1.0 if is_best else 0.95,
                g=0.6 if is_best else 0.75,
                b=0.0 if is_best else 0.15,
                a=1.0 if is_best else 0.7,
            )
            arr.markers.append(m)
        self.frontier_pub.publish(arr)

    def _publish_status(self):
        known = total = None
        if self._grid is not None:
            known = int(np.count_nonzero(self._grid >= 0))
            total = int(self._grid.size)
        status = {
            "state": self.state,
            "current_goal": (
                {"x": self._goal_target[0], "y": self._goal_target[1]}
                if self._goal_target
                else None
            ),
            "blacklisted": len(self._blacklist),
            # Cheap live proxy, NOT a scientific coverage metric: fraction of
            # the current grid's own extent that is non-unknown. The grid
            # itself grows as RTAB-Map explores, so this is "how filled-in is
            # what we've drawn so far", not "% of the true room mapped".
            "explored_pct": round(100.0 * known / total, 1) if known and total else None,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--autostart",
        action="store_true",
        help="Start exploring immediately instead of waiting for explore_cmd=start.",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)

    rclpy.init()
    node = ExplorerNode(args.robot_id, cfg, args.autostart)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
