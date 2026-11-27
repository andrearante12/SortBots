"""The run: true world, robots, gateway, network, clock, recording.

Time is hybrid. Messages are events at their exact arrival time (a heap);
motion, sensing and protocol ticks advance in fixed `sim.step_s` steps.
Everything between two step boundaries is processed in time order before
the step runs. Nothing reads the wall clock, and every random draw comes
from generators seeded by the run's seed, so a seed reproduces a run
exactly (tests/test_sim.py checks it).

The true world is the only thing robots never read directly. They sense it
(sensing.py), physically bump into it (try_move), and hear about each other
through the network. The recording does read it, for ground truth: the
`gt` stream has the same schema Isaac's /sim/ground_truth produces, so
simtest's metrics score fleetsim runs and Isaac runs alike.

Who assigns tasks is `dispatch.method` (fleetsim/dispatch/: the real
auction, or a central dispatcher at the gateway node), and what robots may
know beyond their own sensing and the network is `info.*` (fleetsim/info.py).
What each robot is (speed, service times, capabilities) is its robot type
(`robots.types`, config.robot_profiles), and a task may require a capability.
"""
from __future__ import annotations

import fnmatch
import heapq
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

from . import config as C
from . import dispatch as D
from . import grid as G
from . import info as PI
from . import network as N
from .planner import FREE, OCC, Costmap, path_length
from .robot import Robot

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "nodes"))
import fleet_radio  # noqa: E402
import fleet_tasks as ft  # noqa: E402
from commsim.core.protocol import Params  # noqa: E402
from simtest import world as world_lib  # noqa: E402
from simtest.events import MapGrid  # noqa: E402
from waypoints import Waypoint  # noqa: E402

GW = 0
MAP_HEADER_B = 32
RESYNC_B = 24
OBSTACLE_B = 64
INTENT_HEADER_B = 24  # robot id, priority key, send time


def deep_merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for k, v in (over or {}).items():
        out[k] = deep_merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


class Sim:
    def __init__(self, scenario: dict, recorder=None):
        self.scn = scenario
        cfg = self.cfg = scenario["config"]
        self.seed = int(scenario["seed"])
        self.np_rng = np.random.default_rng(self.seed)
        self.now = 0.0
        self.step_s = float(cfg["sim"]["step_s"])
        self.duration = float(scenario["duration_s"])
        self._heap: list = []
        self._seq = 0
        self.recorder = recorder
        self.events: list[dict] | None = [] if recorder is None else None
        self.notes: list[str] = []

        self._build_world()
        n = int(cfg["robots"]["count"])
        self.n = n
        self.ft_cfg = deep_merge(ft.load_config(), cfg.get("fleet_tasks") or {})
        proto = self.ft_cfg.setdefault("protocol", {})
        if (cfg.get("fleet_tasks") or {}).get("protocol", {}).get("quorum_nodes") is None:
            proto["quorum_nodes"] = n + 1  # every node, gateway included (DES does the same)
        self.ft_cfg["speed_mps"] = float(cfg["robots"]["max_speed_mps"])
        self.params = Params.from_config(self.ft_cfg)
        # Robot types (A6): one profile per robot index. Untyped, every
        # profile is the base robots.* values and nothing reads them
        # differently from before types existed.
        self.profiles = C.robot_profiles(cfg)
        self.typed = cfg["robots"]["types"] is not None
        self.unservable: set[str] = set()  # announced tasks no robot in this fleet can do
        # The gateway's task side and every robot's (Robot._new_core) come
        # from the dispatch method; with `auction` these are the FleetTasks
        # objects this file built itself before the layer existed.
        self.dispatch = D.build(self)
        self.gw = self.dispatch.gateway
        self.info = PI.build(self)
        self.peers_truth = cfg["info"]["peers"] == "truth"
        self.tasks_ideal = cfg["info"]["dispatch"] == "truth"
        # kill: gateway. While it is down, operator requests wait for it (the
        # gateway is where they enter the fleet) and it hears and says nothing.
        self.gw_alive = True
        self._held: list[tuple] = []

        homes = self._homes(n)
        prior = self.static.copy() if cfg["map_share"]["prior"] == "truth" \
            or cfg["info"]["map"] == "truth" else None
        self.robot_owner = np.zeros((self.grid.height, self.grid.width), dtype=np.int32)
        self.robot_owner_flat = self.robot_owner.reshape(-1)
        self.robots: list[Robot] = []
        self._body: dict[int, np.ndarray] = {}
        for k in range(n):
            r = Robot(self, k, homes[k], prior)
            self.robots.append(r)
            cells = self._disc_cells(r.x, r.y, r.body_r)
            self._body[k] = cells
            self.robot_owner_flat[cells] = r.node
        self.net = N.build(cfg["network"], n + 1, self.np_rng, self.static, self.grid, GW)
        self.seen_t = np.full((n, n), -np.inf)
        self.seen_xy = np.zeros((n, n, 2))
        self.live_t = np.full((n + 1, n), -np.inf)
        self._pending_status: list = []
        self.contacts_active: set = set()
        self._contacts_now: set = set()
        self._pending_announces = 0
        self.items: dict[str, str] = {}
        self.announced: list[dict] = []
        self.counts = {"contacts": 0, "announced": 0}
        self._schedule_inputs()

    @classmethod
    def world_only(cls, scenario: dict) -> "Sim":
        """The true floor, waypoints, gateway and (via _homes) parking spots
        exactly as a run of `scenario` builds them, and nothing else: no
        robots, network or schedule. For the scenario editor's map
        (fleetsim/preview.py), which has to show the floor a run gets without
        running one, so it calls the same _build_world rather than a copy."""
        self = cls.__new__(cls)
        self.scn = scenario
        self.cfg = scenario["config"]
        self.seed = int(scenario["seed"])
        self.notes = []
        self._build_world()
        return self

    # ---- world -------------------------------------------------------------

    def _build_world(self) -> None:
        w = self.cfg["world"]
        cell = float(w["cell_m"])
        self.gateway_xy = None
        homes = []
        if w["source"] == "synthetic":
            occ, grid, wps, homes, gw = G.synthetic_checked(
                w.get("synthetic") or {}, cell,
                clearance_m=float(self.cfg["robots"]["footprint_radius_m"]) + 0.5 * cell,
                tol_m=float(self.cfg["nav"]["goal_tolerance_m"]))
            self.gateway_xy = gw
            self.free_lost = 0.0
        elif w["source"] == "map":
            fine, fgrid, wps = G.load_saved_map(w["map"])
            occ, grid, self.free_lost = G.downsample(fine, fgrid, cell)
            if self.free_lost > 0.05:
                self.notes.append(f"downsampling {w['map']} to {grid.res:.2f} m cells turned "
                                  f"{100 * self.free_lost:.0f}% of its free floor into obstacle; "
                                  "narrow passages may be closed (world.cell_m)")
        else:
            raise ValueError(f"world.source: expected synthetic | map, got {w['source']!r}")
        self.grid = grid
        self.static = occ.astype(bool)
        self.static_flat = self.static.reshape(-1)
        self.base_static = self.static.copy()
        self.static_version = 0  # bumped whenever the true grid changes (a blockage)
        self._map_homes = homes
        rng = np.random.default_rng(self.seed + 1)
        if not wps:
            spots = G.free_spots(occ, grid, int(w["random_waypoints"]), rng,
                                 clearance_m=0.6, spacing_m=2.0)
            wps = {f"p_{i}": p for i, p in enumerate(spots)}
            self.notes.append(f"the map has no waypoints; using {len(wps)} random spots p_0..")
        self.waypoints = {name: Waypoint({"name": name, "x": x, "y": y}) for name, (x, y) in wps.items()}
        for name, (x, y) in wps.items():
            if self.static_flat[grid.index(x, y) or 0]:
                self.notes.append(f"waypoint {name} sits in an occupied cell at {grid.res:.2f} m")
        if self.gateway_xy is None:
            g = w.get("gateway")
            self.gateway_xy = tuple(g) if g else (grid.ox + grid.width * grid.res / 2,
                                                  grid.oy + grid.height * grid.res / 2)
        # Scripted movers from a simtest world (configs/worlds/): the same
        # actors, paths and timing Isaac plays.
        self.actors = list(self.scn.get("actors") or [])
        self.actor_owner = np.zeros((grid.height, grid.width), dtype=np.int16)
        self.actor_owner_flat = self.actor_owner.reshape(-1)
        self.actor_pose: dict[str, dict] = {}

    def _homes(self, n: int) -> list:
        homes = list(self._map_homes)
        if self.cfg["world"].get("homes"):
            homes = [tuple(h) for h in self.cfg["world"]["homes"]]
        if len(homes) < n:
            rng = np.random.default_rng(self.seed + 2)
            homes += G.free_spots(self.static, self.grid, n - len(homes), rng,
                                  clearance_m=0.6, spacing_m=1.2, avoid=homes)
        if len(homes) < n:
            raise ValueError(f"room for only {len(homes)} robots on this map, asked for {n}")
        return homes[:n]

    # One physical item per task: on its shelf, on a robot, or delivered.
    def item_at(self, task_id: str) -> str:
        return self.items.get(task_id, "shelf")

    def take_item(self, task_id: str, rid: str) -> bool:
        if self.item_at(task_id) != "shelf":
            return False
        self.items[task_id] = rid
        return True

    def place_item(self, task_id: str) -> None:
        self.items[task_id] = "delivered"

    def waypoint_xy(self, name: str) -> tuple[float, float]:
        w = self.waypoints[name]
        return (w.x, w.y)

    def actor_radius(self, ident: int) -> float:
        return world_lib.half_extent(self.actors[ident])

    def _disc_cells(self, x: float, y: float, r: float) -> np.ndarray:
        g = self.grid
        k = int(math.ceil(r / g.res)) + 1
        c = g.cell(x, y)
        if c is None:
            return np.zeros(0, dtype=np.int64)
        xs = np.arange(max(0, c[0] - k), min(g.width, c[0] + k + 1))
        ys = np.arange(max(0, c[1] - k), min(g.height, c[1] + k + 1))
        cx = g.ox + (xs + 0.5) * g.res
        cy = g.oy + (ys + 0.5) * g.res
        m = (cx[None, :] - x) ** 2 + (cy[:, None] - y) ** 2 <= r * r
        yy, xx = np.nonzero(m)
        return (ys[yy] * g.width + xs[xx]).astype(np.int64)

    def try_move(self, r: Robot, x: float, y: float) -> bool:
        """Physics: a body cannot move into a wall, a peer or a mover. A move
        that would is refused and counted as a contact — the robot drove
        into something its local layer didn't stop it for."""
        cells = self._disc_cells(x, y, r.body_r)
        hit, where = None, None
        if len(cells) == 0 or self.static_flat[cells].any():
            hit = ("static", None)
        else:
            own = self.robot_owner_flat[cells]
            other = own[(own != 0) & (own != r.node)]
            if len(other):
                o = self.robots[int(other[0]) - 1]
                hit, where = ("robot", o.rid), (o.x, o.y)
            elif self.actor_owner_flat[cells].any():
                a = int(self.actor_owner_flat[cells][self.actor_owner_flat[cells] != 0][0]) - 1
                p = self.actor_pose[self.actors[a]["id"]]
                hit, where = ("actor", self.actors[a]["id"]), (p["x"], p["y"])
        if hit is not None:
            # The robot drove into it only if it was ahead; something that
            # came at its side or back while it crept forward came at it.
            by = "robot"
            if where is not None and (where[0] - r.x) * math.cos(r.yaw) \
                    + (where[1] - r.y) * math.sin(r.yaw) <= 0.0:
                by = hit[0]
            self._contact(r, hit[0], hit[1], by=by)
            # What it bumped is now an obstacle to its local layer, so it
            # stops pushing and, after block_replan_s, plans around it.
            r.bumps.append((x, y, self.now))
            return False
        old = self._body[r.k]
        self.robot_owner_flat[old[self.robot_owner_flat[old] == r.node]] = 0
        self.robot_owner_flat[cells] = r.node
        self._body[r.k] = cells
        r.x, r.y = x, y
        return True

    def _contact(self, r: Robot, kind: str, other, by: str) -> None:
        """`by` is who moved: "robot" (its local layer let it drive into
        something) or "actor" (a mover walked into it)."""
        key = (r.rid, kind, other)
        self._contacts_now.add(key)
        if key not in self.contacts_active:
            self.counts["contacts"] += 1
            self.record({"s": "contact", "rid": r.rid, "kind": kind, "other": other, "by": by,
                         "x": round(r.x, 3), "y": round(r.y, 3)})

    def _move_actors(self) -> None:
        if not self.actors:
            return
        self.actor_owner[:] = 0
        for i, a in enumerate(self.actors):
            p = world_lib.pose_at(a["path"], self.now)
            self.actor_pose[a["id"]] = p
            cells = self._disc_cells(p["x"], p["y"], world_lib.half_extent(a))
            self.actor_owner_flat[cells] = i + 1
            # A mover never yields (simtest/world.py): walking into a robot is a
            # contact whoever moved.
            hit = self.robot_owner_flat[cells]
            for node in np.unique(hit[hit != 0]).tolist():
                self._contact(self.robots[node - 1], "actor", a["id"], by="actor")

    # ---- scheduling ------------------------------------------------------------

    def at(self, t: float, fn, *a) -> None:
        self._seq += 1
        heapq.heappush(self._heap, (t, self._seq, fn, a))

    def _schedule_inputs(self) -> None:
        for t in self.scn.get("tasks") or []:
            self.at(float(t["at_s"]), self._announce, t["pickup"], t["dropoff"], None,
                    C.requirement(t.get("requires")))
            self._pending_announces += 1
        arr = self.scn.get("task_arrival")
        if arr:
            rng = np.random.default_rng(self.seed + 3)
            names = list(self.waypoints)
            picks = [n for n in names if fnmatch.fnmatch(n, arr.get("pickups", "*"))]
            drops = [n for n in names if fnmatch.fnmatch(n, arr.get("dropoffs", "*"))]
            if not picks or not drops:
                raise ValueError(f"task_arrival: no waypoints match {arr.get('pickups')!r} / "
                                 f"{arr.get('dropoffs')!r}")
            t, k = float(arr.get("start_s", 0.0)), 0
            until = float(arr.get("until_s", self.duration))
            count = arr.get("count")
            if arr.get("count_per_robot") is not None:
                count = int(round(float(arr["count_per_robot"]) * self.n))
            rate = (float(arr["rate_hz"]) if "rate_hz" in arr
                    else float(arr["rate_per_robot_hz"]) * self.n)
            # Which arrivals need a capability: one draw per arrival from a
            # generator of their own, so the stream itself (times, pickups,
            # dropoffs) is the same with and without `requires`, and a mixed
            # fleet is compared with a plain one on the very same tasks.
            shares = C.arrival_requires(arr["requires"]) if arr.get("requires") else None
            req_rng = np.random.default_rng([self.seed, 0xA6]) if shares else None
            while True:
                t += float(rng.exponential(1.0 / rate))
                if t >= until or (count is not None and k >= int(count)):
                    break
                p = picks[int(rng.integers(len(picks)))]
                d = drops[int(rng.integers(len(drops)))]
                req = ()
                if shares:
                    u = float(req_rng.random())
                    for cap, f in shares:
                        if u < f:
                            req = (cap,)
                            break
                        u -= f
                self.at(t, self._announce, p, d, None, req)
                self._pending_announces += 1
                k += 1
        self._targets: dict[int, tuple] = {}          # kill {select}: (robot, what matched)
        self._blockages: dict[int, tuple] = {}        # blockages up now: index -> cell slices
        self._waiting_since: dict[int, float] = {}    # waiting for its spot to clear, since
        self._blockage_wait: dict[int, float] = {}    # how long it waited, once it is up
        self._blockage_gone: set[int] = set()         # cleared before it ever appeared
        for i, d in enumerate(self.scn.get("disturbances") or []):
            self.at(float(d["at_s"]), self._disturb, i, d, True)
            # A kill {select} schedules its own revival when it fires: with
            # within_s it may fire late, and the outage is still duration_s.
            if d.get("duration_s") is not None and not isinstance(d.get("kill"), dict):
                self.at(float(d["at_s"]) + float(d["duration_s"]), self._disturb, i, d, False)

    def _announce(self, pickup: str, dropoff: str, requested: float | None = None,
                  requires: tuple = ()) -> None:
        if not self.gw_alive:
            self._held.append((pickup, dropoff, self.now, requires))
            return
        self._pending_announces -= 1
        tid = self.gw.request(pickup, dropoff, self.now, requires=requires)
        if tid is not None:
            self.counts["announced"] += 1
            rec = {"task_id": f"{ft.TASK_PREFIX}{tid}", "pickup": pickup, "dropoff": dropoff,
                   "at_s": round(self.now, 3)}
            if requested is not None:  # held while the gateway was down
                rec["requested_s"] = round(requested, 3)
            if requires:
                rec["requires"] = list(requires)
            self.announced.append(rec)
            self.record({"s": "announce", **rec})
            self._task_bound(rec)
        self.handle_actions(None, self.gw.drain())

    def able(self, requires) -> list[int]:
        """Indices of the robots whose type has every capability in `requires`."""
        need = set(requires)
        return [k for k, p in enumerate(self.profiles) if need <= set(p["capabilities"])]

    def _task_bound(self, rec: dict) -> None:
        """The task's lower bound on its delivery time, for the `bounds`
        metric (docs/fleetsim.md "Benchmark metrics"): the shortest drive
        pickup -> dropoff on the true layout at max speed, plus the pick and
        place dwells. No method can beat it, whatever robot it sends.

        With robot types, the best robot this fleet could send: the minimum
        over the types that have the task's capabilities and at least one
        robot of drive / their max speed + their pick + place times. A task
        no robot can do (none of the types that can has a robot) gets no
        bound; it is noted, and left out of the stop condition so the run
        can still end at tasks_done.

        Shortest LENGTH, not the robots' inflation-weighted cost (cost_weight
        0, lethal band only, so the same planning radius the robots plan
        with), then pulled taut: the robots drive a smoothed path, and a raw
        8-connected path is up to 8 % longer than that. The base layout, not
        the floor at announce time: a blockage only lengthens a drive, and
        one that clears later would leave a bound above a real delivery.
        Reads no RNG and touches nothing the robots read, so a run's other
        events are exactly what they were without it."""
        if not hasattr(self, "_bound_cm"):  # built on the first announce
            truth = np.where(self.base_static, OCC, FREE).astype(np.int8)
            nav = {**self.cfg["nav"], "inflation_m": 0.0, "cost_weight": 0.0, "unknown_cost": 0.0}
            self._bound_cm = Costmap(self.grid, truth, nav,
                                     float(self.cfg["robots"]["footprint_radius_m"]))
            self._bound_m: dict = {}  # (pickup, dropoff) -> metres | None
        key = (rec["pickup"], rec["dropoff"])
        if key not in self._bound_m:
            self._bound_m[key] = self._shortest_drive_m(*map(self.waypoint_xy, key))
        d = self._bound_m[key]
        if d is None:
            self.notes.append(f"{rec['task_id']}: no path {key[0]} -> {key[1]} on the true map")
        extra = {}
        if not self.typed:
            lb = None if d is None else (d / float(self.cfg["robots"]["max_speed_mps"])
                                         + 2 * float(self.cfg["tasks"]["dwell_s"]))
        else:
            req = tuple(rec.get("requires") or ())
            able = self.able(req)
            # One profile per distinct type: the bound is per type, not per robot.
            kinds = {(p["max_speed_mps"], p["pick_s"], p["place_s"])
                     for p in (self.profiles[k] for k in able)}
            lb = None if d is None or not kinds else min(d / v + (pick + place)
                                                         for v, pick, place in kinds)
            if req:
                # `able` feeds the bounds metric's capacity per capability.
                extra = {"requires": list(req), "able": len(able)}
            if not able:
                self.unservable.add(rec["task_id"])
                self.notes.append(f"{rec['task_id']} requires {list(req)}: no robot in this "
                                  f"fleet of {self.n} has it; it stays unassigned, and the run "
                                  "does not wait for it")
        self.record({"s": "task_bound", "task_id": rec["task_id"],
                     "lb_s": None if lb is None else round(lb, 2),
                     "drive_m": None if d is None else round(d, 2), **extra})

    def _shortest_drive_m(self, a, b) -> float | None:
        cm, g = self._bound_cm, self.grid
        pts = cm.plan(a, b, goal_tol_m=float(self.cfg["nav"]["goal_tolerance_m"]),
                      max_expansions=g.size)
        if pts is None:
            return None
        if cm.lethal_xy(*a):
            # plan() prefixes the pickup's own cell when it is lethal (a shelf
            # spot inside the rack's band); a robot sent there stopped at the
            # nearest free cell instead, and never drives that hop.
            pts = pts[1:]
        step = 0.5 * g.res

        def clear(p, q):
            n = max(1, int(math.dist(p, q) / step))
            return all(not cm.lethal_xy(p[0] + i / n * (q[0] - p[0]), p[1] + i / n * (q[1] - p[1]))
                       for i in range(1, n))

        taut, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = i + 1
            while j + 1 < len(pts) and clear(pts[i], pts[j + 1]):
                j += 1
            taut.append(pts[j])
            i = j
        return path_length(taut)

    def _node(self, name: str) -> int:
        if name == "gateway":
            return GW
        return ft.robot_num(name)

    def _members(self, m) -> set:
        """One partition member: a node name, or {first: F} / {last: F}, the
        first or last F of the robots by index (rounded, at least one), so
        one scenario splits a fleet of 2 and of 20 alike."""
        if isinstance(m, dict):
            (side, f), = m.items()
            k = min(self.n, max(1, int(math.floor(float(f) * self.n + 0.5))))
            ks = range(k) if side == "first" else range(self.n - k, self.n)
            return {j + 1 for j in ks}
        return {self._node(m)}

    def _group(self, spec, groups) -> set:
        if spec == "rest":
            named = set().union(*(self._group(g, []) for g in groups if g != "rest"))
            return set(range(self.n + 1)) - named
        return set().union(*(self._members(m) for m in (spec if isinstance(spec, list) else [spec])))

    def _select(self, how: str) -> tuple[Robot | None, str | None]:
        """kill: {select: how}: the lowest-index live robot doing that now,
        falling back to any busy robot, then to any live one."""
        live = [r for r in self.robots if r.alive]
        busy = [r for r in live if r.task is not None]
        pools = {"carrying": [r for r in busy if self.item_at(r.task["task_id"]) == r.rid],
                 "fetching": [r for r in busy if r.state in ("NAV_TO_PICKUP", "AWAIT_PICK",
                                                             "PICKING")],
                 "busy": busy, "any": live}
        for name in dict.fromkeys((how, "busy", "any")):
            if pools[name]:
                return pools[name][0], name
        return None, None

    def _name(self, node: int) -> str:
        return "gateway" if node == GW else f"robot_{node - 1}"

    def _disturb(self, i: int, d: dict, on: bool) -> None:
        now = self.now
        rec = {k: v for k, v in d.items() if k != "at_s"}
        if "kill" in d:
            if isinstance(d["kill"], dict):
                if on:
                    # within_s: wait up to that long for a robot that is doing
                    # it. A fleet of 2 at half load often has nobody carrying
                    # at the scheduled moment, and the kill fell back to an
                    # idle robot, which tests nothing (2026-11-26).
                    sel = d["kill"]["select"]
                    r, matched = self._select(sel)
                    until = float(d["at_s"]) + float(d["kill"].get("within_s", 0))
                    if matched != sel and now < until:
                        self.at(now + self.step_s, self._disturb, i, d, True)
                        return
                    self._targets[i] = (r, matched)
                    if r is None:
                        self.notes.append(f"disturbances[{i}]: no live robot to kill at {now:.0f} s")
                    if d.get("duration_s") is not None:
                        self.at(now + float(d["duration_s"]), self._disturb, i, d, False)
                r, matched = self._targets.get(i, (None, None))
                # On and off alike: the replay pairs the two by their whole body.
                rec.update(kill=r.rid if r else None, select=d["kill"]["select"], matched=matched)
                node = None if r is None else r.node
            else:
                node = self._node(d["kill"])
            if node == GW:
                if on:
                    self.gw_alive = False
                    self.net.kill(GW, now)
                else:
                    self._revive_gateway()
            elif node is not None:
                r = self.robots[node - 1]
                if on:
                    r.alive, r.v = False, 0.0
                    self.net.kill(r.node, now)
                else:
                    self._revive(r, bool(d.get("fresh", False)))
        elif "revive" in d and on:
            if d["revive"] == "gateway":
                self._revive_gateway()
            else:
                self._revive(self.robots[self._node(d["revive"]) - 1], bool(d.get("fresh", False)))
        elif "partition" in d:
            groups = d["partition"]
            sets = [self._group(g, groups) for g in groups]
            for j, a in enumerate(sets):
                for b in sets[j + 1:]:
                    for x in a:
                        for y in b:
                            (self.net.block if on else self.net.unblock)(x, y, now)
            if any(isinstance(m, dict) for g in groups if g != "rest"
                   for m in (g if isinstance(g, list) else [g])):
                # The replay reads robot names: record whom a fraction meant.
                rec["partition"] = [g if g == "rest" else [self._name(x) for x in sorted(s)]
                                    for g, s in zip(groups, sets)]
                rec["spec"] = groups
        elif "blockage" in d:
            if not self._blockage(i, d, on):
                return
            if i in self._blockage_wait:  # on and off alike, as for kill above
                rec["waited_s"] = round(self._blockage_wait[i], 2)
        self.record({"s": "disturb", "on": on, **rec})

    def _blockage(self, i: int, d: dict, on: bool) -> bool:
        """Put blockage i up or take it down; False if nothing changed (yet).

        The true floor is the static map plus every blockage up now, rebuilt
        on each change: a cleared blockage used to restore the static map
        under its whole rect, deleting any other blockage overlapping it, so
        one obstacle "moving" by half its length left a hole (2026-11-26).
        A blockage waits for its spot to be clear of robots and movers (a
        pallet is set down beside a robot, not on it); one that appeared on
        top of a robot trapped it, since every move from inside a wall is
        refused."""
        x0, y0, x1, y1 = d["blockage"]["rect"]
        g = self.grid
        c0, c1 = g.cell(min(x0, x1), min(y0, y1)), g.cell(max(x0, x1), max(y0, y1))
        if c0 is None or c1 is None:
            raise ValueError(f"blockage {d['blockage']} is off the map")
        sl = (slice(c0[1], c1[1] + 1), slice(c0[0], c1[0] + 1))
        if on:
            if i in self._blockage_gone:  # its time ran out while it waited
                return False
            near = (slice(max(0, c0[1] - 1), c1[1] + 2), slice(max(0, c0[0] - 1), c1[0] + 2))
            if self.robot_owner[near].any() or self.actor_owner[near].any():
                self._waiting_since.setdefault(i, self.now)
                self.at(self.now + self.step_s, self._disturb, i, d, True)
                return False
            self._blockages[i] = sl
            if i in self._waiting_since:
                self._blockage_wait[i] = self.now - self._waiting_since.pop(i)
        else:
            if self._blockages.pop(i, None) is None:
                self._blockage_gone.add(i)
                self.notes.append(f"disturbances[{i}]: blockage never appeared (its spot "
                                  "was never clear of robots)")
                return False
        np.copyto(self.static, self.base_static)
        for s in self._blockages.values():
            self.static[s] = True
        # The central planners' drive-time fields and info.map: truth both key
        # off these (fleetsim/dispatch/costs.py, fleetsim/info.py).
        self.static_version += 1
        if self.info is not None:
            self.info.map_changed(sl)
        return True

    def _revive(self, r: Robot, fresh: bool) -> None:
        r.alive = True
        self.net.revive(r.node, self.now)
        # Its Status and map timers stood still while it was dead; without
        # this it sent one of each per step until they caught up (a 120 s
        # outage: a minute of Status at twice the rate).
        r.next_status = max(r.next_status, self.now)
        r.next_map_pub = max(r.next_map_pub, self.now)
        if fresh:  # a reboot: empty task state, re-learned from the fleet
            r.queue.clear()
            r.task, r.state = None, "IDLE"
            r._clear_goal()
            r._new_core()

    def _revive_gateway(self) -> None:
        """Back with the state it had (a power cut, not a reinstall), then
        the requests that waited for it, in order. Their announce carries
        `requested_s`, when the operator asked."""
        if self.gw_alive:
            return
        self.gw_alive = True
        self.net.revive(GW, self.now)
        held, self._held = self._held, []
        for pickup, dropoff, t, requires in held:
            self._announce(pickup, dropoff, requested=t, requires=requires)

    # ---- traffic --------------------------------------------------------------

    def _all_nodes(self) -> np.ndarray:
        return np.arange(self.n + 1)

    def _robots_near(self, xy, radius: float, exclude: int | None = None) -> np.ndarray:
        pos = np.array([(r.x, r.y) for r in self.robots])
        near = np.flatnonzero(np.hypot(pos[:, 0] - xy[0], pos[:, 1] - xy[1]) <= radius) + 1
        return near[near != exclude] if exclude is not None else near

    def handle_actions(self, owner, actions) -> None:
        src = GW if owner is None else owner.node
        for kind, val in actions:
            if kind == "publish":
                got = ft.decode(val)
                if got is None:
                    continue
                sender, m = got
                # protocol.py "Addressing": an Ack goes to its one receiver, a
                # repeat to the peers still missing; everything else fleet-wide.
                to = m.get("to")
                dsts = (self._all_nodes() if to is None
                        else np.array(to if isinstance(to, list) else [to], dtype=np.int64))
                if self.tasks_ideal:
                    # info.dispatch: truth — instantly, always, through any
                    # partition, and off the radio. Through the heap at the
                    # current time, so a reply is never handled inside the
                    # call that caused it.
                    for node in dsts.tolist():
                        if node != src:
                            self.at(self.now, self._deliver_task, node, (sender, m))
                    continue
                self._send("tasks", src, dsts, len(val) + 8, True,
                           self._deliver_task, (sender, m))
            elif kind == "dispatch" and owner is not None:
                owner.dispatch(val)
            elif kind == "cancel" and owner is not None:
                owner.cancel(val)
            elif kind == "pick" and owner is not None:
                owner.permit_pick(val)
            elif kind == "log" and self.cfg["record"]["fleet_log"]:
                self.record({"s": "fleet_log", "rid": "gateway" if owner is None else owner.rid,
                             "text": val})

    def _send(self, kind, src, dsts, size, reliable, handler, payload, deadline=None) -> None:
        d, arr, retry, retry_t = self.net.send(kind, src, dsts, size, reliable, self.now, deadline)
        for node, t in zip(d.tolist(), arr.tolist()):
            self.at(t, handler, node, payload)
        if len(retry):
            dl = deadline if deadline is not None else self.now + self.net.lease_s
            self.at(retry_t, self._send, kind, src, retry, size, reliable, handler, payload, dl)

    def _deliver_task(self, node: int, payload) -> None:
        sender, m = payload
        if node == GW:
            if not self.gw_alive:  # in flight when it went down
                return
            self.gw.on_message(sender, m, self.now)
            self.handle_actions(None, self.gw.drain())
            return
        r = self.robots[node - 1]
        if r.alive:
            r.core_call(r.core.on_message, sender, m, self.now)

    def _status_tick(self, r: Robot) -> None:
        cfg = self.cfg["status"]
        owned = r.core.owned()
        raw = fleet_radio.encode_status(r.rid, r.x, r.y, r.yaw, r.v, 0.0, r.mode(), self.now,
                                        owned=owned, battery_pct=r.core.battery_pct)
        if cfg["recipients"] == "all":
            dsts = self._all_nodes()
        else:
            dsts = np.r_[GW, self._robots_near((r.x, r.y), float(cfg["radius_m"]))]
        it = r.intent_for_status(self.now)
        extra = 0 if it is None else 4 + 12 * len(it["pts"])  # fleetsim/intent.py points
        d, arr, _, _ = self.net.send("status", r.node, dsts, len(raw) + 8 + extra, False,
                                     self.now, udp=True)
        if len(d):
            self._pending_status.append((r.k, d, arr,
                                         (r.x, r.y, r.mode(), r.core.battery_pct, it)))

    def _session_liveness(self) -> None:
        """status.session_liveness: a live zenoh session to a peer counts as
        hearing it (Allocator.heard_network, what the commsim DES does). With
        neighbour-only Status a robot hears few peers' Status, and without
        this it could never see a majority for the quorum. NOT what
        nodes/fleet_tasks.py does today: it only counts messages."""
        now = self.now
        reach = self.net.reach & self.net.alive_phys[None, :]
        for r in self.robots:
            if not r.alive:
                continue
            for node in np.flatnonzero(reach[r.node]).tolist():
                if node != r.node:
                    r.core.heard_network(now, node)

    def _apply_status(self) -> None:
        if not self._pending_status:
            return
        now, poll = self.now, float(self.cfg["status"]["liveness_poll_s"])
        keep = []
        for src, d, arr, (x, y, mode, bat, it) in self._pending_status:
            due = arr <= now
            if not due.all():
                keep.append((src, d[~due], arr[~due], (x, y, mode, bat, it)))
            dd = d[due]
            if not len(dd):
                continue
            rob = dd[dd > 0] - 1
            self.seen_t[rob, src] = now
            self.seen_xy[rob, src] = (x, y)
            if it is not None:
                for k in rob.tolist():
                    if self.robots[k].alive:
                        self.robots[k].on_intent(it, now)
            stale = dd[now - self.live_t[dd, src] >= poll]
            if not len(stale):
                continue
            self.live_t[stale, src] = now
            rid = f"robot_{src}"
            for node in stale.tolist():
                if node == GW:
                    if not self.gw_alive:
                        continue
                    self.dispatch.gateway_status(src, now, bat, mode, (x, y))
                    self.handle_actions(None, self.gw.drain())
                else:
                    rr = self.robots[node - 1]
                    if rr.alive:
                        rr.core_call(rr.core.on_peer_status, rid, now, battery_pct=bat, mode=mode)
        self._pending_status = keep

    # map sharing ---------------------------------------------------------------

    def _map_bytes(self, n_cells: int) -> int:
        c = self.cfg["map_share"]
        return MAP_HEADER_B + int(math.ceil(n_cells * self.grid.res ** 2 * float(c["bytes_per_m2"])))

    def _map_dsts(self, r: Robot, tile: int) -> np.ndarray:
        c = self.cfg["map_share"]
        if c["recipients"] == "all":
            d = np.arange(1, self.n + 1)
            return d[d != r.node]
        return self._robots_near(r.belief.tile_centre(tile), float(c["radius_m"]), exclude=r.node)

    def _send_cells(self, r: Robot, tile: int, payload: tuple, snapshot: bool,
                    dsts: np.ndarray, budget: bool = True) -> bool:
        """Chunk and send one tile's cells; False (nothing sent) if the rate
        cap has no room for it."""
        c = self.cfg["map_share"]
        idx, st, ts = payload
        per_chunk = max(1, int((float(c["chunk_bytes"]) - MAP_HEADER_B)
                               / max(self.grid.res ** 2 * float(c["bytes_per_m2"]), 1e-9)))
        chunks = [slice(i, i + per_chunk) for i in range(0, max(len(idx), 1), per_chunk)]
        total = sum(self._map_bytes(len(idx[s])) for s in chunks) * max(1, len(dsts))
        if budget and r.tokens < total:
            return False
        r.tokens -= total
        for s in chunks:
            seq = r.belief.out_seq.get(tile, 0) if snapshot else r.belief.next_seq(tile)
            self._send("map", r.node, dsts, self._map_bytes(len(idx[s])), False, self._deliver_map,
                       (r.node, tile, seq, snapshot, (idx[s], st[s], ts[s])))
        return True

    def _map_tick(self, r: Robot) -> None:
        c = self.cfg["map_share"]
        rate = float(c["rate_cap_bps"]) / 8.0
        r.tokens = min(r.tokens + rate * float(c["publish_s"]), 2 * rate * float(c["publish_s"]))
        for tile, idx in r.belief.take_dirty().items():
            if not self._send_cells(r, tile, r.belief.cells_payload(idx), False,
                                    self._map_dsts(r, tile)):
                r.belief.dirty[idx] = True  # over the cap: try again next tick
        if self.now >= r.next_refresh and r.belief.observed_tiles:
            tiles = r.belief.observed_tiles
            tile = tiles[r.refresh_ptr % len(tiles)]
            if self._send_cells(r, tile, r.belief.snapshot(tile), True, self._map_dsts(r, tile)):
                r.refresh_ptr += 1
            r.next_refresh = self.now + float(c["refresh_s"]) / len(tiles)

    def _deliver_map(self, node: int, payload) -> None:
        origin, tile, seq, snapshot, cells = payload
        r = self.robots[node - 1]
        if not r.alive:
            return
        gap = r.belief.check_seq(origin, tile, seq, snapshot)
        changed = r.belief.merge(*cells)
        r.counters["shared_changed"] += len(changed)
        r.on_belief_change(changed, "shared")
        key = (origin, tile)
        if gap and self.now >= r.resync_wait.get(key, -math.inf):
            r.resync_wait[key] = self.now + float(self.cfg["map_share"]["resync_backoff_s"])
            self._send("resync", r.node, np.array([origin]), RESYNC_B, True, self._deliver_resync,
                       (r.node, tile))

    def _deliver_resync(self, node: int, payload) -> None:
        asker, tile = payload
        o = self.robots[node - 1]
        if o.alive:
            self._send_cells(o, tile, o.belief.snapshot(tile), True, np.array([asker]), budget=False)

    def share_intent(self, r: Robot, m: dict) -> None:
        """A path intent (fleetsim/intent.py): best-effort, like Status, to
        robots within intent.radius_m. ~12 B a point plus a header."""
        dsts = self._robots_near((r.x, r.y), float(self.cfg["intent"]["radius_m"]),
                                 exclude=r.node)
        if len(dsts):
            self._send("intent", r.node, dsts, INTENT_HEADER_B + 12 * len(m["pts"]), False,
                       self._deliver_intent, m)

    def _deliver_intent(self, node: int, m: dict) -> None:
        r = self.robots[node - 1]
        if r.alive:
            r.on_intent(m, self.now)

    def share_obstacle(self, r: Robot, o: dict) -> None:
        dsts = self._robots_near((r.x, r.y), float(self.cfg["obstacle_share"]["radius_m"]),
                                 exclude=r.node)
        self._send("obstacle", r.node, dsts, OBSTACLE_B, True, self._deliver_obstacle, o)

    def _deliver_obstacle(self, node: int, o: dict) -> None:
        r = self.robots[node - 1]
        if r.alive:
            r.on_obstacle(o, self.now)

    # ---- recording ----------------------------------------------------------------

    def record(self, ev: dict) -> None:
        ev = {"s": ev["s"], "t": round(self.now, 4), **{k: v for k, v in ev.items() if k != "s"}}
        if self.recorder is not None:
            self.recorder.write(ev)
        else:
            self.events.append(ev)

    def _gt(self) -> None:
        self.record({"s": "gt", "wt": round(self.now, 3),
                     "robots": {r.rid: {"x": round(r.x, 3), "y": round(r.y, 3),
                                        "yaw": round(r.yaw, 3)} for r in self.robots},
                     "actors": {k: {kk: (round(vv, 3) if isinstance(vv, float) else vv)
                                    for kk, vv in p.items()} for k, p in self.actor_pose.items()}})

    def _belief_sample(self) -> None:
        """Each robot's map against the truth. Errors on the occupied side
        are counted on obstacle SURFACES only (occupied cells with a free
        4-neighbour): a camera never sees inside a pallet, and planning
        only needs its faces."""
        truth = self.static
        free = ~truth
        surf = truth.copy()
        inner = np.ones_like(truth)
        inner[1:, :] &= truth[:-1, :]
        inner[:-1, :] &= truth[1:, :]
        inner[:, 1:] &= truth[:, :-1]
        inner[:, :-1] &= truth[:, 1:]
        surf &= ~inner
        free_n = int(free.sum())
        # The fleet's best knowledge: per cell, the newest observation any
        # robot holds. `stale` is how far a robot is from it — the gap map
        # sharing exists to close, whatever the truth is.
        if self.robots:
            stamps = np.stack([r.belief.stamp for r in self.robots])
            newest = stamps.argmax(axis=0)
            states = np.stack([r.belief.state for r in self.robots])
            best = np.take_along_axis(states, newest[None], axis=0)[0]
            seen = stamps.max(axis=0) > -1e8  # someone actually observed it (not just a prior)
        rows = {}
        for k, r in enumerate(self.robots):
            s = r.belief.state
            known_free = int(((s == FREE) & free).sum())
            rows[r.rid] = {"known_free": round(known_free / max(free_n, 1), 4),
                           "wrong_free": int(((s == FREE) & surf).sum()),
                           "wrong_occ": int(((s == OCC) & free).sum()),
                           "stale": int((seen & (s != best)).sum()),
                           "known": int((s != -1).sum())}
        self.record({"s": "belief", "free_cells": free_n, "surface_cells": int(surf.sum()),
                     "cell_m": self.grid.res, "robots": rows})

    def _grid_json(self, data: np.ndarray) -> dict:
        g = self.grid
        return MapGrid(g.width, g.height, g.res, g.ox, g.oy, data).to_json()

    def _world(self) -> None:
        """The floor as the run starts, once: everything the replay draws
        that no other stream carries. Blockages arrive later as `disturb`."""
        self.record({"s": "world",
                     "grid": self._grid_json(np.where(self.static, OCC, FREE).astype(np.int8)),
                     "waypoints": {k: [round(w.x, 3), round(w.y, 3)] for k, w in self.waypoints.items()},
                     "homes": {r.rid: [round(r.home[0], 3), round(r.home[1], 3)] for r in self.robots},
                     "gateway": [round(float(v), 3) for v in self.gateway_xy],
                     "body_r": float(self.cfg["robots"]["footprint_radius_m"]),
                     "actors": {a["id"]: round(float(world_lib.half_extent(a)), 3) for a in self.actors},
                     # Robot types (A6), only when set: an untyped run's
                     # world event is what it was before types existed.
                     **({"types": {r.rid: r.profile["type"] for r in self.robots}}
                        if self.typed else {})})

    def _belief_maps(self) -> None:
        sel = self.cfg["record"]["belief_maps"]
        for r in self.robots:
            if sel == "all" or r.rid in sel:
                self.record({"s": "belief_map", "rid": r.rid, "grid": self._grid_json(r.belief.state)})

    def _links(self) -> None:
        # One-hop links only: routes are paths over these, and the viewer can
        # walk them. Blocked and dead links are already out of the table.
        tab = self.net.link_table()
        self.record({"s": "links", "links": [[e["a"], e["b"], round(e["snr_db"])] for e in tab]})

    def _net_sample(self) -> None:
        # record.net_nodes: who sent and heard what, and each node's load, for
        # the replay's comms monitor. +3.3 kB a sample at 50 robots (2026-11-26).
        nodes = self.net.take_nodes() if self.cfg["record"]["net_nodes"] else {}
        self.record({"s": "net", "model": self.cfg["network"]["model"],
                     "kinds": self.net.take_stats(), **self.net.summary(), **nodes})

    def _board_sample(self) -> None:
        b = self.gw.board()
        states: dict[str, int] = {}
        for t in b["tasks"]:
            states[t["state"]] = states.get(t["state"], 0) + 1
        self.record({"s": "board", "tasks": states,
                     "alarms": sum(1 for a in b["alarms"] if a["active"]),
                     "robots_alive": sum(1 for x in b["robots"] if x["alive"])})

    # ---- main loop ------------------------------------------------------------------

    def _tasks_done(self) -> bool:
        if not self.announced or self._pending_announces > 0:
            return False
        # A task no robot can do would hold the run to its full duration;
        # it is reported (notes, task_bound's able: 0) and not waited for.
        return all(self.dispatch.task_done(rec["task_id"]) for rec in self.announced
                   if rec["task_id"] not in self.unservable)

    def run(self) -> dict:
        cfg = self.cfg
        rec = cfg["record"]
        wall0 = time.monotonic()
        self.record({"s": "start", "wall": time.time(), "sim": 0.0,
                     "fleetsim": {"robots": self.n, "network": cfg["network"]["model"],
                                  "cell_m": self.grid.res, "seed": self.seed,
                                  "grid": [self.grid.width, self.grid.height],
                                  "gateway": list(self.gateway_xy), "notes": self.notes}})
        self._world()
        nxt = {"gt": 0.0, "belief": 0.0, "net": float(rec["net_s"]), "board": float(rec["board_s"]),
               "session": 0.0, "bmap": 0.0, "links": 0.0}
        links_on = float(rec["links_s"]) > 0 and hasattr(self.net, "link_table")
        stop = self.scn.get("stop") or {}
        settle_until = None
        reason = "duration"
        dt = self.step_s
        t = 0.0
        self.net.update(np.array([self.gateway_xy] + [(r.x, r.y) for r in self.robots]), 0.0)
        while t < self.duration - 1e-9:
            t_next = round(t + dt, 9)
            while self._heap and self._heap[0][0] <= t_next:
                et, _s, fn, a = heapq.heappop(self._heap)
                self.now = max(self.now, et)
                fn(*a)
            self.now = t = t_next
            self._contacts_now = set()
            self._move_actors()
            self.net.update(np.array([self.gateway_xy] + [(r.x, r.y) for r in self.robots]), t)
            self._apply_status()
            if cfg["status"]["session_liveness"] and t >= nxt["session"]:
                nxt["session"] += float(cfg["status"]["liveness_poll_s"])
                self._session_liveness()
            if self.info is not None:
                self.info.step(t)
            if self.gw_alive:
                self.gw.tick(t)
                self.handle_actions(None, self.gw.drain())
            for r in self.robots:
                r.step(dt, t)
            for r in self.robots:
                if r.alive and t >= r.next_status:
                    r.next_status += 1.0 / float(cfg["status"]["hz"])
                    self._status_tick(r)
                if r.alive and cfg["map_share"]["enabled"] and t >= r.next_map_pub:
                    r.next_map_pub += float(cfg["map_share"]["publish_s"])
                    self._map_tick(r)
            self.contacts_active = self._contacts_now
            if t >= nxt["gt"]:
                nxt["gt"] += 1.0 / float(rec["gt_hz"])
                self._gt()
            if rec["belief_s"] and t >= nxt["belief"]:
                nxt["belief"] += float(rec["belief_s"])
                self._belief_sample()
            if t >= nxt["net"]:
                nxt["net"] += float(rec["net_s"])
                self._net_sample()
            if rec["belief_map_s"] and t >= nxt["bmap"]:
                nxt["bmap"] += float(rec["belief_map_s"])
                self._belief_maps()
            if links_on and t >= nxt["links"]:
                nxt["links"] += float(rec["links_s"])
                self._links()
            if t >= nxt["board"]:
                nxt["board"] += float(rec["board_s"])
                self._board_sample()
            if stop.get("until") == "tasks_done" and settle_until is None and self._tasks_done():
                settle_until = t + float(stop.get("settle_s", 0.0))
            if settle_until is not None and t >= settle_until:
                reason = "tasks_done"
                break
        self._gt()
        if rec["belief_s"]:
            self._belief_sample()
        if rec["belief_map_s"]:
            self._belief_maps()
        self._net_sample()
        self._board_sample()
        wall = time.monotonic() - wall0
        summary = {"sim_s": round(t, 2), "wall_s": round(wall, 2),
                   "speedup": round(t / wall, 2) if wall > 0 else None, "reason": reason,
                   "robots": self.n, "announced": self.counts["announced"],
                   "contacts": self.counts["contacts"],
                   "nav": {k: round(sum(r.counters[k] for r in self.robots), 2)
                           for k in self.robots[0].counters} if self.robots else {},
                   "notes": self.notes}
        extra = self.dispatch.summary()
        if extra is not None:
            summary["dispatch"] = extra
        self.record({"s": "end", "reason": reason, "sim_s": round(t, 3), "wall_s": round(wall, 3),
                     "summary": summary})
        return summary
