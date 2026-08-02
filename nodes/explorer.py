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
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
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
    "frontier_consumed_radius_m": 1.0,  # goal is satisfied once no frontier remains this close to it
    "goal_min_age_s": 3.0,         # don't run the consumed check before a goal is this old
    "blacklist_merge_radius_m": 0.75,   # re-blacklisting within this bumps strikes instead of adding
    "blacklist_ttl_growth": 3.0,   # ttl = blacklist_ttl_s * growth**(strikes-1)
    "blacklist_permanent_strikes": 3,   # at/above this many strikes an entry never expires
    "blacklist_radius_growth_m": 0.25,  # per-strike widening of one entry's suppression radius
    "blacklist_radius_max_m": 1.5,      # cap on that widening (keep below aisle width)
    "escape_pocket_radius_m": 1.5,      # disc blacklisted around the robot when escape fires
    "startup_spin": True,          # 360-degree spin before the first frontier goal
    "startup_spin_yaw": 6.283185,  # 2*pi
    "startup_spin_timeout_s": 60.0,     # wall-clock give-up waiting for, then running, the spin
    "reference_free_area_m2": 0.0,      # denominator for coverage_pct; 0 disables
    "max_run_time_s": 0.0,         # wall-clock budget before _finish() fires; 0 disables
    "steer_weight": 4.0,           # operator hint: peak score multiplier is 1 + this
    "steer_decay_m": 3.0,          # hint influence falls off e-fold per this many metres
    "steer_radius_m": 4.0,         # frontiers this close to a hint ignore max_goal_distance_m
    "steer_lock": True,            # restrict candidates to the hinted area, not just prefer it
    "steer_ttl_s": 300.0,          # forget a hint after this long; 0 = never expire
    "hard_escape_cooldown_s": 90.0,  # min gap between hard-escape spins
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
        # [x, y, monotonic_ts, strikes, radius] — see _blacklist_point for why
        # entries escalate instead of accumulating as independent neighbours.
        self._blacklist: list[list[float]] = []
        self._other_claims: dict[str, tuple[float, float, float]] = {}  # robot_id -> (x, y, ts)
        # Bumped on every /map message. _frontier_consumed() compares it
        # against the seq a goal was chosen on, so a goal can never be
        # abandoned on the same grid that produced it.
        self._map_seq = 0

        self._goal_handle = None
        self._goal_target: tuple[float, float] | None = None
        # (fx, fy): the frontier look-at point the active goal was chosen to
        # reveal, and the map seq it was chosen on. Both drive
        # _frontier_consumed(); cleared wherever _goal_target is cleared.
        self._goal_frontier: tuple[float, float] | None = None
        self._goal_map_seq: int | None = None
        self._goal_sent_at: float | None = None
        # Robot pose when the active goal went out, so _goal_progress() can
        # tell "revealed the frontier by driving there" from "sat still while
        # it evaporated" — see the consumed branch in _tick.
        self._goal_start_pose: tuple[float, float] | None = None
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

        # Run statistics, all published in _publish_status() and summarized by
        # _finish(). Reset together on explore_cmd=start.
        self._started_at = time.monotonic()
        self._goals_sent = 0
        self._goals_succeeded = 0
        self._goals_failed = 0
        self._goals_consumed = 0   # abandoned early because the frontier was already revealed
        self._rejections = 0       # consecutive goals Nav2 refused to accept
        self._no_tf_ticks = 0      # consecutive ticks with no map->base_link transform
        self._last_clusters = 0
        self._last_candidates = 0

        # Startup spin (see _begin_startup_spin): "pending" -> "running" ->
        # "done". Published as `phase`, deliberately NOT as a `state` value —
        # task_manager.py gates dispatch on state == "exploring" and would
        # grab navigate_to_pose mid-spin if the explorer looked idle.
        self._spin_state = "pending" if (autostart and cfg["startup_spin"]) else "done"
        self._spin_deadline: float | None = None
        self._spin_reason = "startup"          # or "hard escape", for logging
        self._last_hard_escape_at: float | None = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, f"/{robot_id}/navigate_to_pose")
        self.spin_client = ActionClient(self, Spin, f"/{robot_id}/spin")

        # Operator steering hint: (x, y, monotonic_ts) or None. A soft bias on
        # frontier scoring, deliberately NOT a goal — see _steer_bonus.
        self._steer_hint: tuple[float, float, float] | None = None

        self.create_subscription(OccupancyGrid, f"/{robot_id}/map", self._on_map, MAP_QOS)
        self.create_subscription(String, f"/{robot_id}/explore_cmd", self._on_cmd, 10)
        self.create_subscription(String, f"/{robot_id}/explore_hint", self._on_hint, 10)
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
        # frombuffer, not np.array(list): rclpy hands int8[] over as an
        # array.array, and reading it as a buffer is ~360x cheaper than
        # element-wise conversion (measured 4.7 ms -> 0.013 ms on a 380x746
        # grid). This runs at map rate, so it's worth the fallback branch for
        # the plain-list case.
        try:
            flat = np.frombuffer(msg.data, dtype=np.int8).astype(np.int16)
        except (TypeError, BufferError):
            flat = np.array(msg.data, dtype=np.int16)
        self._grid = flat.reshape(msg.info.height, msg.info.width)
        self._map_seq += 1

    # -- external control (dashboard / task_manager) ----------------------

    def _on_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "start":
            if self.state != "exploring":
                self.get_logger().info("explore_cmd: start")
                self.state = "exploring"
                self._empty_cycles = 0
                self._consec_failures = 0
                self._no_tf_ticks = 0
                self._rejections = 0
                self._started_at = time.monotonic()
                # Spin on a manual start too, but only if we've never sent a
                # goal — a stop/start in the middle of a run shouldn't cost
                # another full rotation.
                if self.cfg["startup_spin"] and self._goals_sent == 0:
                    self._spin_state = "pending"
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

    # -- operator steering -------------------------------------------------

    def _on_hint(self, msg: String):
        """`{"x": .., "y": ..}` — "explore over there", or `{}` to clear.

        Deliberately a scoring BIAS rather than a goal. The dashboard's
        click-to-nav publishes straight to navigate_to_pose, which the
        explorer's next goal preempts about a second later — so operator
        commands were simply overrun, and the only way to stop that was to
        stop exploring altogether. A hint instead makes frontiers near the
        clicked point win ties, so the robot heads that way while still
        choosing its own goals, still avoiding walls, still backtracking and
        escaping on its own. Nothing about autonomy is suspended, which is
        why this can't strand the robot if the operator clicks somewhere
        unreachable: an unreachable frontier just fails and gets blacklisted
        as usual, and the hint expires after steer_ttl_s.
        """
        try:
            h = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn(f"explore_hint: bad payload {msg.data!r}")
            return
        if not h or "x" not in h or "y" not in h:
            self._steer_hint = None
            self.get_logger().info("explore_hint: cleared — back to unbiased scoring")
            return
        try:
            x, y = float(h["x"]), float(h["y"])
        except (TypeError, ValueError):
            self.get_logger().warn(f"explore_hint: non-numeric point {msg.data!r}")
            return
        self._steer_hint = (x, y, time.monotonic())
        self.get_logger().info(
            f"explore_hint: steering toward ({x:.2f}, {y:.2f}) "
            f"for {self.cfg['steer_ttl_s']:.0f}s — retargeting now"
        )

        # Act on it IMMEDIATELY rather than waiting for the current goal to
        # end. _tick returns early while a goal is healthy, so otherwise a
        # hint sits queued behind whatever is running — and a goal that isn't
        # failing can hold the robot for up to goal_timeout_s. Observed live
        # 2026-08-02: a hint waited 11 s for the active goal to abort, and the
        # goal after it ran 75 s to timeout before anything could change.
        # Pointing at the map should mean "go there", not "go there once
        # you're finished".
        #
        # Not a failure: no blacklist, no _consec_failures. Bump the
        # generation before cancelling so the CANCELED result this produces
        # isn't misread as a navigation failure by _on_nav_result — the same
        # trap the no-replacement path in _tick documents.
        if self._goal_handle is not None:
            superseded = self._goal_handle
            self._cancel_active_goal(cancel_on_server=False)
            if not self._plan_and_send():
                self._goal_generation += 1
                superseded.cancel_goal_async()
        else:
            self._plan_and_send()
        self._publish_status()

    def _active_hint(self) -> tuple[float, float] | None:
        if self._steer_hint is None:
            return None
        x, y, ts = self._steer_hint
        ttl = self.cfg["steer_ttl_s"]
        if ttl > 0 and time.monotonic() - ts >= ttl:
            self._steer_hint = None
            self.get_logger().info("explore_hint: expired — back to unbiased scoring")
            return None
        return (x, y)

    def _hint_area_explored(self, hint, unknown: np.ndarray, info) -> bool:
        """True once there is no unknown space left near the operator's click.

        This — not "are there frontiers within steer_radius_m" — is the right
        question to end a steer lock on. Frontiers are free cells ADJACENT TO
        unknown, so they sit on the boundary of unexplored space and never
        inside it; clicking into the middle of an unmapped area (the obvious
        way to say "go explore there") has no frontier near it at all until
        the robot has already arrived. Testing for unknown cells instead means
        the lock holds while there is genuinely something left to reveal, and
        ends when the area is mapped.

        A hint outside the current grid counts as NOT explored — the grid
        grows as RTAB-Map maps, and a click past its present edge is exactly
        the "go somewhere new" case.
        """
        res = info.resolution
        ix = int((hint[0] - info.origin.position.x) / res)
        iy = int((hint[1] - info.origin.position.y) / res)
        r = max(1, round(self.cfg["steer_radius_m"] / res))
        h, w = unknown.shape
        y0, y1 = max(0, iy - r), min(h, iy + r + 1)
        x0, x1 = max(0, ix - r), min(w, ix + r + 1)
        if y0 >= y1 or x0 >= x1:
            return False
        return not bool(unknown[y0:y1, x0:x1].any())

    def _steer_bonus(self, gx: float, gy: float, hint) -> float:
        """Multiplier in [1, 1 + steer_weight] for a goal's distance to `hint`.

        Exponential rather than a hard radius so there is no cliff: a frontier
        just outside the operator's click is still preferred over one on the
        far side of the map, and the bias degrades smoothly into normal
        scoring instead of switching off.
        """
        if hint is None:
            return 1.0
        d = math.hypot(gx - hint[0], gy - hint[1])
        return 1.0 + self.cfg["steer_weight"] * math.exp(-d / max(0.1, self.cfg["steer_decay_m"]))

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
        self._goal_frontier = None
        self._goal_map_seq = None
        self._goal_start_pose = None
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

    def _blacklist_point(self, x: float, y: float, radius: float | None = None):
        """Suppress future goals near (x, y), escalating on repeat failures.

        A flat TTL meant a genuinely unreachable frontier — behind a rack,
        behind sim geometry the depth camera can't see past, in a pocket the
        planner can't route into — came back every blacklist_ttl_s forever.
        That burned goal cycles on a known-dead target AND made _finish()
        structurally unreachable: there was always "one more" candidate, so
        _empty_cycles could never run up to done_after_empty_cycles.

        Re-blacklisting within blacklist_merge_radius_m of an existing entry
        now bumps that entry's STRIKE count instead of appending a neighbour.
        Each strike multiplies its TTL and widens its suppression radius, and
        at blacklist_permanent_strikes it stops expiring entirely. Growth is
        deliberately PER-ENTRY rather than driven by the global
        _consec_failures: that counter has no spatial meaning, so keying off
        it would inflate suppression map-wide, including in areas the robot
        has never had trouble with.
        """
        merge = self.cfg["blacklist_merge_radius_m"]
        for b in self._blacklist:
            if math.hypot(x - b[0], y - b[1]) < merge:
                b[2] = time.monotonic()
                b[3] += 1
                b[4] = min(
                    self.cfg["blacklist_radius_max_m"],
                    self.cfg["blacklist_radius_m"]
                    + (b[3] - 1) * self.cfg["blacklist_radius_growth_m"],
                )
                ttl = self._blacklist_ttl(b)
                self.get_logger().info(
                    f"blacklist strike {int(b[3])} at ({b[0]:.2f}, {b[1]:.2f}) "
                    f"— radius {b[4]:.2f}m, "
                    + ("PERMANENT" if math.isinf(ttl) else f"ttl {ttl:.0f}s")
                )
                return
        self._blacklist.append([
            x, y, time.monotonic(), 1,
            self.cfg["blacklist_radius_m"] if radius is None else radius,
        ])

    def _blacklist_ttl(self, b: list[float]) -> float:
        base = self.cfg["blacklist_ttl_s"]
        if base <= 0 or b[3] >= self.cfg["blacklist_permanent_strikes"]:
            return math.inf
        return base * self.cfg["blacklist_ttl_growth"] ** (b[3] - 1)

    def _active_blacklist(self) -> list[tuple[float, float, float]]:
        """Live entries as (x, y, radius). Sole owner of the expiry pruning."""
        now = time.monotonic()
        self._blacklist = [b for b in self._blacklist if now - b[2] < self._blacklist_ttl(b)]
        return [(b[0], b[1], b[4]) for b in self._blacklist]

    @staticmethod
    def _blacklisted(x: float, y: float, entries: list[tuple[float, float, float]]) -> bool:
        """Per-entry radii, unlike _too_close's single shared radius."""
        return any(math.hypot(x - bx, y - by) < br for bx, by, br in entries)

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

        budget = self.cfg["max_run_time_s"]
        if budget > 0 and time.monotonic() - self._started_at > budget:
            # Wall-clock backstop. Everything else that ends a run depends on
            # the frontier set going empty, which a stalled perception stack
            # or a genuinely unreachable remainder can prevent indefinitely.
            # This guarantees the run terminates and the final map gets saved.
            self.get_logger().warn(f"max_run_time_s ({budget:.0f}s) reached — finishing")
            self._cancel_active_goal()
            self._finish()
            return

        if self._spin_state == "pending":
            self._begin_startup_spin()
            self._publish_status()
            return
        if self._spin_state == "running":
            self._check_startup_spin()
            self._publish_status()
            return

        superseded = None
        if self._goal_handle is not None:
            # Order is load-bearing: a wedged robot must trip the stuck
            # watchdog (which blacklists and counts a failure) rather than be
            # excused by a "consumed" verdict it happens to also satisfy.
            reason = self._goal_fault()
            if reason is None and self._frontier_consumed():
                reason = "consumed"
            if reason is None:
                self._publish_status()  # heartbeat while a goal is in flight
                return

            superseded = self._goal_handle
            gx, gy = self._goal_target
            if reason == "consumed":
                fx, fy = self._goal_frontier
                self._goals_consumed += 1
                # Reset the failure streak IF the robot actually travelled to
                # earn this. Consumption is the normal healthy outcome — live
                # 2026-08-02 it ended 24 of 28 goals, so STATUS_SUCCEEDED
                # essentially never fires and _consec_failures, which only
                # reset on that, climbed monotonically into permanent escape
                # mode (3 pointless 9-14 m cross-map jumps in 10 minutes).
                # But resetting unconditionally would be just as wrong: a
                # wedged robot whose frontier gets cleared by someone else's
                # raytrace would keep zeroing the counter and never escalate.
                # Requiring real displacement separates "revealed it by
                # driving there" from "it evaporated while we sat still".
                moved = self._goal_progress()
                if moved >= self.cfg["stuck_min_displacement_m"]:
                    self._consec_failures = 0
                self.get_logger().info(
                    f"frontier at ({fx:.2f}, {fy:.2f}) already revealed en route to "
                    f"({gx:.2f}, {gy:.2f}) after {moved:.2f}m — retargeting (not a failure)"
                )
            elif reason == "stuck":
                self.get_logger().warn(
                    f"stuck: moved <{self.cfg['stuck_min_displacement_m']}m in "
                    f"{self.cfg['stuck_window_s']:.0f}s en route to ({gx:.2f}, {gy:.2f}) "
                    f"— blacklisting + retargeting"
                )
                self._blacklist_point(gx, gy)
                self._consec_failures += 1
                self._pose_history.clear()
            else:  # "timeout"
                self.get_logger().warn(
                    f"goal to ({gx:.2f}, {gy:.2f}) timed out after "
                    f"{self.cfg['goal_timeout_s']:.0f}s — canceling + blacklisting"
                )
                self._blacklist_point(gx, gy)
                self._consec_failures += 1
            self._cancel_active_goal(cancel_on_server=False)

        if not self._plan_and_send() and superseded is not None:
            # _cancel_active_goal(cancel_on_server=False) above relies on a
            # REPLACEMENT goal preempting the old one on the server. No
            # replacement went out (no map, no TF, or no candidates), so
            # nothing will — and Nav2 would keep driving to a goal this node
            # no longer tracks, invisible to the stuck watchdog. Cancelling
            # for real is safe here for exactly the reason it's unsafe on the
            # normal path: there is no brand-new goal for it to race against.
            #
            # Bump the generation FIRST, to disown the goal before cancelling
            # it. Otherwise the CANCELED result we are about to cause comes
            # back with a generation that still matches, and _on_nav_result
            # scores our own housekeeping as a navigation failure — blacklists
            # the target and increments _consec_failures. Observed live
            # 2026-08-02: that misattribution blacklisted the only frontier on
            # the map within seconds of startup and drove a spurious escape.
            # On the normal path _send_goal does this bump for us; this branch
            # is the only one where nothing otherwise would.
            self._goal_generation += 1
            superseded.cancel_goal_async()

    def _goal_progress(self) -> float:
        """Straight-line distance covered since the active goal was sent."""
        if self._goal_start_pose is None:
            return 0.0
        pose = self._robot_pose()
        if pose is None:
            return 0.0
        return math.hypot(pose[0] - self._goal_start_pose[0], pose[1] - self._goal_start_pose[1])

    def _goal_fault(self) -> str | None:
        """"stuck" | "timeout" | None for the goal currently in flight."""
        if self._is_stuck():
            return "stuck"
        if (
            self._goal_sent_at is not None
            and time.monotonic() - self._goal_sent_at > self.cfg["goal_timeout_s"]
        ):
            return "timeout"
        return None

    def _frontier_consumed(self) -> bool:
        """True once nothing frontier-like remains near what this goal targeted.

        Not a failure — a saved trip. The depth camera reveals a frontier from
        4-5 m out (RTAB-Map Grid/RangeMax 5.0, costmap raytrace_max_range
        5.0), but the goal is a standoff only goal_standoff_max_m (1.5 m)
        away, so without this the robot spends its last several metres — plus
        a rotate-in-place to satisfy yaw_goal_tolerance — arriving to look at
        space it already mapped.

        Deliberately does NOT re-run find_frontiers(): this sits on _tick's
        hot early-return path. It recomputes the SAME predicate find_frontiers
        uses (free & has-unknown-neighbour & ~dilate(occupied)) over a small
        window around the target only — measured ~0.1 ms against ~7.5 ms for
        the full clustering pass on a 380x746 grid. The window is padded by
        the inflation radius and dilated identically so the answer is
        bit-identical to what find_frontiers would say about those cells;
        without the pad this would keep goals alive that find_frontiers would
        never re-select.
        """
        if self._goal_frontier is None or self._grid is None or self._map_info is None:
            return False
        if (
            self._goal_sent_at is None
            or time.monotonic() - self._goal_sent_at < self.cfg["goal_min_age_s"]
        ):
            return False
        # The real anti-thrash guard, and it's exact rather than tuned: the
        # candidate came out of grid_masks() on map seq N, so on seq N this
        # cell IS a frontier by construction and the check below could only
        # ever say "still there". Requiring a NEW map bounds abandonment to
        # RTAB-Map's publish rate no matter how fast _tick runs.
        if self._map_seq == self._goal_map_seq:
            return False

        info = self._map_info
        res = info.resolution
        fx, fy = self._goal_frontier
        ix = int((fx - info.origin.position.x) / res)
        iy = int((fy - info.origin.position.y) / res)
        r = max(1, round(self.cfg["frontier_consumed_radius_m"] / res))
        pad = max(0, round(self.cfg["inflate_radius_m"] / res)) + 1
        h, w = self._grid.shape
        y0, y1 = max(0, iy - r), min(h, iy + r + 1)
        x0, x1 = max(0, ix - r), min(w, ix + r + 1)
        if y0 >= y1 or x0 >= x1:
            return True  # the target fell outside the grid entirely

        py0, py1 = max(0, y0 - pad), min(h, y1 + pad)
        px0, px1 = max(0, x0 - pad), min(w, x1 + pad)
        sub = self._grid[py0:py1, px0:px1]
        unknown, occupied, free = grid_masks(sub, self.cfg["occupied_thresh"])
        has_unknown_neighbor = np.zeros_like(free)
        has_unknown_neighbor[:-1, :] |= unknown[1:, :]
        has_unknown_neighbor[1:, :] |= unknown[:-1, :]
        has_unknown_neighbor[:, :-1] |= unknown[:, 1:]
        has_unknown_neighbor[:, 1:] |= unknown[:, :-1]
        mask = free & has_unknown_neighbor & ~_dilate4(occupied, pad - 1)
        return not bool(mask[y0 - py0 : y1 - py0, x0 - px0 : x1 - px0].any())

    # -- startup spin ------------------------------------------------------

    def _begin_startup_spin(self):
        """One full rotation before the first frontier goal.

        The robot has a single forward-facing depth camera, so at t=0 it knows
        nothing about 3/4 of its surroundings and the first frontier choice is
        very nearly a guess — frequently the only candidate, which is exactly
        the fragility that killed two fresh runs (see _on_goal_response).
        A 2*pi spin costs ~8 s of sim time and hands the scorer a full
        Grid/RangeMax disc instead of a cone. It also functions as a per-run
        regression test for behavior_server's local_frame (configs/
        nav2_params.yaml): before that fix, this exact action aborted
        instantly on every attempt.
        """
        if self._spin_deadline is None:
            self._spin_deadline = time.monotonic() + self.cfg["startup_spin_timeout_s"]
        if time.monotonic() > self._spin_deadline:
            self.get_logger().warn("startup spin: /spin server never appeared — skipping")
            self._spin_state = "done"
            return
        if not self.spin_client.server_is_ready():
            return  # behavior_server still coming up; retry next tick

        goal = Spin.Goal()
        goal.target_yaw = float(self.cfg["startup_spin_yaw"])
        # MUST be set: nav2_behaviors' Spin only enforces a timeout when
        # command_time_allowance_ > 0, so the zero default means a spin that
        # stalls (e.g. odom TF drops out) never gives up.
        goal.time_allowance = Duration(sec=20)
        self.get_logger().info(
            f"startup spin: rotating {goal.target_yaw:.2f} rad for an initial 360-degree view"
        )
        self._spin_state = "running"
        self._spin_reason = "startup"
        self.spin_client.send_goal_async(goal).add_done_callback(
            lambda f: self._on_spin_response(f, retry=True)
        )

    def _check_startup_spin(self):
        if self._spin_deadline is not None and time.monotonic() > self._spin_deadline:
            self.get_logger().warn("startup spin: deadline passed — carrying on without it")
            self._spin_state = "done"

    def _on_spin_response(self, future, retry: bool = False):
        handle = future.result()
        if not handle.accepted:
            # server_is_ready() only proves the action server EXISTS. During
            # bringup behavior_server advertises it while still inactive and
            # answers "Action server is inactive. Rejecting the goal." —
            # observed live 2026-08-02, rejected 176 ms before
            # lifecycle_manager activated it. Same race the navigate_to_pose
            # rejection path in _on_goal_response documents, so treat it the
            # same way: retry rather than give up. _begin_startup_spin's
            # deadline is what eventually calls it off.
            if retry:
                self.get_logger().warn(
                    "startup spin rejected (behavior_server not active yet) — retrying",
                    throttle_duration_sec=5.0,
                )
                self._spin_state = "pending"
            else:
                self.get_logger().warn("spin rejected — carrying on without it")
                self._spin_state = "done"
            return
        handle.get_result_async().add_done_callback(self._on_spin_result)

    def _on_spin_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"{self._spin_reason} spin complete — recoveries are functional")
        else:
            self.get_logger().warn(
                f"spin ended status={status} — carrying on. If the behavior_server log "
                f"shows a TF error, check its local_frame in configs/nav2_params.yaml "
                f"(must be {self.robot_id}/odom, not nav2's bare 'odom' default) — that "
                f"same setting also disables every BT recovery."
            )
        self._spin_state = "done"

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
        # sampling happens once per replan tick, so dropping everything older
        # than the window outright would leave the oldest sample perpetually
        # ~1 tick younger than the window and the filled check below would
        # never pass.
        while len(self._pose_history) >= 2 and now - self._pose_history[1][0] >= window:
            self._pose_history.popleft()
        oldest = self._pose_history[0]
        if now - oldest[0] < window:
            return False  # window not filled yet
        max_disp = max(
            math.hypot(x - oldest[1], y - oldest[2]) for _, x, y in self._pose_history
        )
        return max_disp < self.cfg["stuck_min_displacement_m"]

    def _maybe_hard_escape_spin(self) -> bool:
        """Spin in place when escape mode alone hasn't broken the deadlock.

        At twice escape_after_failures the robot has failed repeatedly AND
        failed to escape, which usually means it is physically wedged with a
        stale local costmap. With a forward-only depth camera, rotating in
        place is the highest-information-per-second action available: it
        re-raytraces every direction at once. Fire-and-forget — _tick's spin
        branch waits it out, then replans (still in escape mode, since
        _consec_failures is untouched).
        """
        if self._consec_failures < 2 * self.cfg["escape_after_failures"]:
            return False
        if self._spin_state != "done" or not self.spin_client.server_is_ready():
            return False
        # Cooldown. The trigger is a LEVEL (_consec_failures stays high until
        # a goal succeeds), not an edge, so without this the spin re-fires the
        # instant the previous one finishes and the robot spins forever
        # instead of exploring. Observed live 2026-08-02: once the counter got
        # stuck high, this became an 8-second spin/complete/spin loop that ran
        # until the run was killed. One spin per cooldown window is plenty —
        # the point is to re-observe once, then go try a goal.
        now = time.monotonic()
        if self._last_hard_escape_at is not None and \
                now - self._last_hard_escape_at < self.cfg["hard_escape_cooldown_s"]:
            return False
        self._last_hard_escape_at = now
        goal = Spin.Goal()
        goal.target_yaw = math.pi
        goal.time_allowance = Duration(sec=20)
        self.get_logger().warn(
            f"hard escape: {self._consec_failures} consecutive failures — "
            f"spinning in place to re-observe before retargeting"
        )
        self._spin_state = "running"
        self._spin_reason = "hard escape"
        self._spin_deadline = time.monotonic() + self.cfg["startup_spin_timeout_s"]
        # retry=False: a rejection here means Nav2 is unhealthy, and unlike at
        # bringup there's no reason to expect that to resolve on its own —
        # fall through to the escape goal rather than spin-retrying forever.
        self.spin_client.send_goal_async(goal).add_done_callback(
            lambda f: self._on_spin_response(f, retry=False)
        )
        return True

    def _plan_and_send(self) -> bool:
        """Pick and dispatch the next frontier goal. True iff one went out.

        The return value matters: _tick() clears the previous goal WITHOUT
        cancelling it on the server, on the assumption that the replacement
        preempts it. Every early return here breaks that assumption, so
        _tick needs to know.
        """
        if self._grid is None or self._map_info is None:
            return False
        if self._maybe_hard_escape_spin():
            return False
        pose = self._robot_pose()
        if pose is None:
            self._no_tf_ticks += 1
            self.get_logger().warn(
                f"no map->{self.robot_id}/base_link TF yet — can't plan "
                f"({self._no_tf_ticks} ticks)",
                throttle_duration_sec=5.0,
            )
            # A silent forever-hang otherwise: this path returns before
            # _empty_cycles is touched, so a lost map->base_link transform
            # (RTAB-Map losing odometry) leaves the explorer heart-beating in
            # "exploring" with nothing to end the run.
            if self._no_tf_ticks == 30:
                self.get_logger().error(
                    f"no map->{self.robot_id}/base_link TF for 30 consecutive ticks — "
                    f"RTAB-Map odometry is probably lost; exploration is stalled"
                )
            elif self._no_tf_ticks > 300:
                self.get_logger().error("TF has been missing far too long — finishing")
                self._finish()
            return False
        self._no_tf_ticks = 0
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
        hint = self._active_hint()
        open_cells = max(1, round(self.cfg["openness_radius_m"] / res))
        h, w = free.shape
        candidates = []  # (score, gx, gy, fx, fy)
        far = []  # (dist, gx, gy, fx, fy): valid but beyond max_goal_distance_m
        escape_pool = []  # (dist, gx, gy, fx, fy): every valid candidate, any distance
        for f, ((gx, gy), (fx, fy), (giy, gix)) in zip(frontiers, goals):
            dist = math.hypot(gx - rx, gy - ry)
            if self._blacklisted(gx, gy, blacklist):
                continue
            if self._too_close(gx, gy, claims, self.cfg["claim_radius_m"]):
                continue
            escape_pool.append((dist, gx, gy, fx, fy))
            # A hinted frontier ignores the distance cap. Without this,
            # clicking the far side of the warehouse would do nothing at all:
            # those frontiers land in `far`, which is only consulted when
            # there are no candidates whatsoever, so the bias could never
            # apply to exactly the case the operator most wants it for.
            hinted = hint is not None and math.hypot(gx - hint[0], gy - hint[1]) <= self.cfg["steer_radius_m"]
            if dist > self.cfg["max_goal_distance_m"] and not hinted:
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
            score *= self._steer_bonus(gx, gy, hint)
            candidates.append((score, gx, gy, fx, fy))

        candidates.sort(key=lambda c: c[0], reverse=True)

        # Region lock: while a hint is live, head for whichever frontier is
        # NEAREST the operator's click, regardless of score.
        #
        # A multiplicative bias was not enough — at alpha 1.0 a frontier 2 m
        # away beats an equal one 15 m away by 7.5x, which swamps
        # steer_weight, so a far click was honoured for one goal and then
        # abandoned. Nor is "restrict to frontiers within steer_radius_m of
        # the click" right, which is what this used to do: a frontier is a
        # free cell ADJACENT TO unknown, so it lives at the EDGE of unexplored
        # space, never inside it. Clicking into the middle of an unmapped area
        # — the obvious gesture for "go explore there" — therefore found no
        # frontier within the radius and released instantly. Observed live
        # 2026-08-02: every hint released ~13 s after being set, and the robot
        # wandered 8-10 m away.
        #
        # Nearest-to-hint has no such geometry assumption. As the robot
        # explores toward the click the frontier boundary advances into the
        # region, so it keeps working inward until the area really is mapped —
        # which is what _hint_area_explored below actually tests.
        if hint is not None and self.cfg["steer_lock"] and candidates:
            if self._hint_area_explored(hint, unknown, info):
                self.get_logger().info(
                    f"explore_hint: area around ({hint[0]:.2f}, {hint[1]:.2f}) is mapped "
                    f"— releasing steer lock"
                )
                self._steer_hint = None
                hint = None
            else:
                candidates.sort(key=lambda c: math.hypot(c[1] - hint[0], c[2] - hint[1]))
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
            if hint is not None:
                # Operator intent beats the heuristic. Escape is a GUESS about
                # where to go when wedged; a hint is someone telling us. Since
                # escape bypasses the scored candidate list entirely, without
                # this branch a hint is silently ignored for as long as escape
                # stays engaged — which, in a run where goals keep failing, is
                # most of the time. Observed live 2026-08-02: operator clicks
                # had no effect whatsoever while escape was active.
                # Still an escape, so the distance cap stays off and we still
                # leave the pocket — just aimed where we were asked.
                # Nearest to the hint, same rule the steer lock uses above —
                # not the steer_bonus ordering, whose exponential decay
                # flattens to ~1.0 for every candidate once the hint is more
                # than a few decay lengths from all of them, silently falling
                # back to "farthest" exactly when the operator most wants
                # otherwise.
                escape_pool.sort(key=lambda c: math.hypot(c[1] - hint[0], c[2] - hint[1]))
                _dist, gx, gy, fx, fy = escape_pool[0]
                self.get_logger().warn(
                    f"escape: {self._consec_failures} consecutive failed goals — "
                    f"steering to the frontier nearest your hint ({gx:.2f}, {gy:.2f}), "
                    f"{_dist:.1f}m away"
                )
            else:
                escape_pool.sort(key=lambda c: c[0], reverse=True)
                _dist, gx, gy, fx, fy = escape_pool[0]
                self.get_logger().warn(
                    f"escape: {self._consec_failures} consecutive failed goals — "
                    f"jumping to farthest frontier ({gx:.2f}, {gy:.2f}), {_dist:.1f}m away"
                )
            # Pocket-blacklist where we're wedged. Blacklisting only the goals
            # actually attempted removes the pocket one blacklist_radius_m
            # spot at a time, so scoring keeps handing back the NEXT untried
            # frontier in the SAME pocket. A disc around the robot kills the
            # whole pocket at once. Applied AFTER the escape target is picked,
            # and only when that target is outside the disc — otherwise the
            # escape would blacklist its own destination and the next tick
            # would throw it away.
            if _dist > self.cfg["escape_pocket_radius_m"]:
                self._blacklist_point(rx, ry, radius=self.cfg["escape_pocket_radius_m"])
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
        self._last_clusters = len(frontiers)
        self._last_candidates = len(candidates)

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
            return False

        self._empty_cycles = 0
        _score, gx, gy, fx, fy = candidates[0]
        return self._send_goal(gx, gy, fx, fy, rx, ry)

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

    def _send_goal(
        self, gx: float, gy: float, fx: float, fy: float, rx: float, ry: float
    ) -> bool:
        # server_is_ready(), not wait_for_server(2.0): this runs inside the
        # timer callback on a single-threaded executor, so a blocking wait
        # stalls map callbacks and every other timer for the full timeout —
        # and buys nothing, since _tick retries every cycle anyway.
        if not self.nav_client.server_is_ready():
            self.get_logger().warn(
                "navigate_to_pose server not ready — retrying next tick",
                throttle_duration_sec=5.0,
            )
            return False
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
        self._goal_frontier = (fx, fy)
        self._goal_map_seq = self._map_seq
        self._goal_start_pose = (rx, ry)
        self._goal_sent_at = time.monotonic()
        self._goals_sent += 1
        self._pose_history.clear()  # fresh stuck window for the new goal
        self._publish_claim(gx, gy)
        self.get_logger().info(f"exploring -> ({gx:.2f}, {gy:.2f})")

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, gen, target)
        )
        self._publish_status()
        return True

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
            # the next cycle; genuine navigation failures still get
            # blacklisted via the ABORTED path in _on_nav_result.
            self._rejections += 1
            self.get_logger().warn(
                f"frontier goal rejected (Nav2 not ready?): {target} — will retry"
            )
            if self._rejections == 10:
                # Persistent rejection means a Nav2 lifecycle node died. The
                # loop below would otherwise retry silently forever without
                # ever incrementing _empty_cycles, so nothing ends the run.
                self.get_logger().error(
                    "10 consecutive goal rejections — bt_navigator/controller_server is "
                    "probably down. Check the bringup log; exploration cannot proceed."
                )
            self._goal_handle = None
            self._goal_target = None
            self._goal_frontier = None
            self._goal_map_seq = None
            self._goal_start_pose = None
            self._goal_sent_at = None
            return
        self._rejections = 0
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
        self._goal_frontier = None
        self._goal_map_seq = None
        self._goal_start_pose = None
        self._goal_sent_at = None
        self._pose_history.clear()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"reached frontier {target}")
            self._goals_succeeded += 1
            self._consec_failures = 0
        else:
            self.get_logger().warn(f"frontier goal to {target} ended status={status} — blacklisting")
            self._blacklist_point(*target)
            self._goals_failed += 1
            self._consec_failures += 1
        self._publish_status()

    def _area_stats(self) -> tuple[float, float, float]:
        """(free_m2, occupied_m2, known_m2) for the current grid."""
        if self._grid is None or self._map_info is None:
            return (0.0, 0.0, 0.0)
        cell = self._map_info.resolution ** 2
        occ = int(np.count_nonzero(self._grid >= self.cfg["occupied_thresh"]))
        known = int(np.count_nonzero(self._grid >= 0))
        return ((known - occ) * cell, occ * cell, known * cell)

    def _finish(self):
        self.state = "done"
        elapsed = time.monotonic() - self._started_at
        free_m2, occ_m2, known_m2 = self._area_stats()
        ref = self.cfg["reference_free_area_m2"]
        permanent = sum(
            1 for b in self._blacklist if b[3] >= self.cfg["blacklist_permanent_strikes"]
        )
        shape = "?x?" if self._grid is None else f"{self._grid.shape[1]}x{self._grid.shape[0]}"
        res = self._map_info.resolution if self._map_info is not None else float("nan")
        self.get_logger().info(
            "exploration done — no reachable frontiers left\n"
            "  ==== run summary ====\n"
            f"  elapsed    : {elapsed:.0f} s wall (~{elapsed / 60:.1f} min; sim runs ~0.45x real time)\n"
            f"  grid       : {shape} @ {res:.3f} m\n"
            f"  known area : {known_m2:.1f} m2 (free {free_m2:.1f}, occupied {occ_m2:.1f})\n"
            + (f"  coverage   : {100.0 * free_m2 / ref:.1f}% of the {ref:.1f} m2 reference free floor\n"
               if ref > 0 else "")
            + f"  goals      : {self._goals_sent} sent / {self._goals_succeeded} reached / "
              f"{self._goals_failed} failed / {self._goals_consumed} consumed-early / "
              f"{self._rejections} rejected\n"
            f"  blacklist  : {len(self._blacklist)} entries ({permanent} permanent)\n"
            f"  save it    : scripts/save_map.sh --run <name> --label final"
        )
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
        """JSON on explore_status.

        webui/app.js reads state/current_goal/blacklisted/explored_pct and
        task_manager.py reads state (comparing it to "exploring"), so those
        four keep their exact types and meanings — in particular `state` never
        gains a new value (the startup spin is reported as `phase` instead)
        and `blacklisted` stays an int. Everything else here is additive; JSON
        consumers ignore keys they don't ask for.
        """
        known = total = None
        if self._grid is not None:
            known = int(np.count_nonzero(self._grid >= 0))
            total = int(self._grid.size)
        free_m2, occ_m2, known_m2 = self._area_stats()
        hint = self._active_hint()
        status = {
            "state": self.state,
            "phase": self._spin_state,
            "current_goal": (
                {"x": self._goal_target[0], "y": self._goal_target[1]}
                if self._goal_target
                else None
            ),
            "blacklisted": len(self._blacklist),
            # Cheap live proxy, NOT a coverage metric: fraction of the current
            # grid's own extent that is non-unknown. The grid grows as
            # RTAB-Map explores, so this is "how filled-in is what we've drawn
            # so far" and it can go DOWN — on the most complete map this repo
            # has ever built it reads 43.8%. Use coverage_pct / free_area_m2
            # below for anything you intend to defend as coverage.
            "explored_pct": round(100.0 * known / total, 1) if total else None,
            "known_area_m2": round(known_m2, 1),
            "free_area_m2": round(free_m2, 1),
            "occupied_area_m2": round(occ_m2, 1),
            "frontier_clusters": self._last_clusters,
            "frontier_candidates": self._last_candidates,
            "goals_sent": self._goals_sent,
            "goals_succeeded": self._goals_succeeded,
            "goals_failed": self._goals_failed,
            "goals_consumed": self._goals_consumed,
            "goal_rejections": self._rejections,
            "no_tf_ticks": self._no_tf_ticks,
            "elapsed_s": round(time.monotonic() - self._started_at, 1),
            # via _active_hint() so an expired hint stops being advertised
            # even while a goal is in flight and _plan_and_send isn't running.
            "steer_hint": ({"x": hint[0], "y": hint[1]} if hint else None),
        }
        ref = self.cfg["reference_free_area_m2"]
        if ref > 0:
            # Free floor mapped as a fraction of the best prior full map's
            # free floor. A ratio of scalar areas, deliberately not a
            # cell-wise overlay: the two maps have no common frame. Exceeding
            # 100% means this run mapped more than the reference did, which is
            # the honest answer rather than a bug.
            status["coverage_pct"] = round(100.0 * free_m2 / ref, 1)
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
