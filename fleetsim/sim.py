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

from . import grid as G
from . import network as N
from .planner import FREE, OCC
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
        self.gw = ft.FleetTasks(None, self.params, self.waypoints, gateway=True,
                                cfg=self.ft_cfg, now=0.0)
        self.gw._wall = lambda: 0.0
        self.gw._next_id = 0

        homes = self._homes(n)
        prior = self.static.copy() if cfg["map_share"]["prior"] == "truth" else None
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

    # ---- world -------------------------------------------------------------

    def _build_world(self) -> None:
        w = self.cfg["world"]
        cell = float(w["cell_m"])
        self.gateway_xy = None
        homes = []
        if w["source"] == "synthetic":
            occ, grid, wps, homes, gw = G.synthetic_warehouse(w.get("synthetic") or {}, cell)
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
            self.at(float(t["at_s"]), self._announce, t["pickup"], t["dropoff"])
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
            while True:
                t += float(rng.exponential(1.0 / float(arr["rate_hz"])))
                if t >= until or (count is not None and k >= int(count)):
                    break
                p = picks[int(rng.integers(len(picks)))]
                d = drops[int(rng.integers(len(drops)))]
                self.at(t, self._announce, p, d)
                self._pending_announces += 1
                k += 1
        for d in self.scn.get("disturbances") or []:
            self.at(float(d["at_s"]), self._disturb, d, True)
            if d.get("duration_s") is not None:
                self.at(float(d["at_s"]) + float(d["duration_s"]), self._disturb, d, False)

    def _announce(self, pickup: str, dropoff: str) -> None:
        self._pending_announces -= 1
        tid = self.gw.request(pickup, dropoff, self.now)
        if tid is not None:
            self.counts["announced"] += 1
            rec = {"task_id": f"{ft.TASK_PREFIX}{tid}", "pickup": pickup, "dropoff": dropoff,
                   "at_s": round(self.now, 3)}
            self.announced.append(rec)
            self.record({"s": "announce", **rec})
        self.handle_actions(None, self.gw.drain())

    def _node(self, name: str) -> int:
        if name == "gateway":
            return GW
        return ft.robot_num(name)

    def _group(self, spec, groups) -> set:
        if spec == "rest":
            named = set().union(*(self._group(g, []) for g in groups if g != "rest"))
            return set(range(self.n + 1)) - named
        return {self._node(x) for x in spec}

    def _disturb(self, d: dict, on: bool) -> None:
        now = self.now
        if "kill" in d:
            r = self.robots[self._node(d["kill"]) - 1]
            if on:
                r.alive, r.v = False, 0.0
                self.net.kill(r.node, now)
            else:
                self._revive(r, bool(d.get("fresh", False)))
        elif "revive" in d and on:
            self._revive(self.robots[self._node(d["revive"]) - 1], bool(d.get("fresh", False)))
        elif "partition" in d:
            groups = d["partition"]
            sets = [self._group(g, groups) for g in groups]
            for i, a in enumerate(sets):
                for b in sets[i + 1:]:
                    for x in a:
                        for y in b:
                            (self.net.block if on else self.net.unblock)(x, y, now)
        elif "block_link" in d:
            a, b = (self._node(x) for x in d["block_link"])
            (self.net.block if on else self.net.unblock)(a, b, now)
        elif "blockage" in d:
            x0, y0, x1, y1 = d["blockage"]["rect"]
            g = self.grid
            c0, c1 = g.cell(min(x0, x1), min(y0, y1)), g.cell(max(x0, x1), max(y0, y1))
            if c0 is None or c1 is None:
                raise ValueError(f"blockage {d['blockage']} is off the map")
            sl = (slice(c0[1], c1[1] + 1), slice(c0[0], c1[0] + 1))
            self.static[sl] = True if on else self.base_static[sl]
        self.record({"s": "disturb", "on": on, **{k: v for k, v in d.items() if k != "at_s"}})

    def _revive(self, r: Robot, fresh: bool) -> None:
        r.alive = True
        self.net.revive(r.node, self.now)
        if fresh:  # a reboot: empty task state, re-learned from the fleet
            r.queue.clear()
            r.task, r.state = None, "IDLE"
            r._clear_goal()
            r._new_core()

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
                self._send("tasks", src, self._all_nodes(), len(val) + 8, True,
                           self._deliver_task, (sender, m))
            elif kind == "dispatch" and owner is not None:
                owner.dispatch(val)
            elif kind == "cancel" and owner is not None:
                owner.cancel(val)
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
            self.gw.on_message(sender, m, self.now)
            self.handle_actions(None, self.gw.drain())
            return
        r = self.robots[node - 1]
        if r.alive:
            r.core_call(r.core.on_message, sender, m, self.now)

    def _status_tick(self, r: Robot) -> None:
        cfg = self.cfg["status"]
        owned = r.core.alloc.owned()
        raw = fleet_radio.encode_status(r.rid, r.x, r.y, r.yaw, r.v, 0.0, r.mode(), self.now,
                                        owned=owned, battery_pct=r.core.battery_pct)
        if cfg["recipients"] == "all":
            dsts = self._all_nodes()
        else:
            dsts = np.r_[GW, self._robots_near((r.x, r.y), float(cfg["radius_m"]))]
        d, arr, _, _ = self.net.send("status", r.node, dsts, len(raw) + 8, False, self.now,
                                     udp=True)
        if len(d):
            self._pending_status.append((r.k, d, arr, (r.x, r.y, r.mode(), r.core.battery_pct)))

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
                    r.core.alloc.heard_network(now, peer=node)

    def _apply_status(self) -> None:
        if not self._pending_status:
            return
        now, poll = self.now, float(self.cfg["status"]["liveness_poll_s"])
        keep = []
        for src, d, arr, (x, y, mode, bat) in self._pending_status:
            due = arr <= now
            if not due.all():
                keep.append((src, d[~due], arr[~due], (x, y, mode, bat)))
            dd = d[due]
            if not len(dd):
                continue
            rob = dd[dd > 0] - 1
            self.seen_t[rob, src] = now
            self.seen_xy[rob, src] = (x, y)
            stale = dd[now - self.live_t[dd, src] >= poll]
            if not len(stale):
                continue
            self.live_t[stale, src] = now
            rid = f"robot_{src}"
            for node in stale.tolist():
                if node == GW:
                    self.gw.on_peer_status(rid, now, battery_pct=bat, mode=mode)
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
                     "actors": {a["id"]: round(float(world_lib.half_extent(a)), 3) for a in self.actors}})

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
        self.record({"s": "net", "model": self.cfg["network"]["model"],
                     "kinds": self.net.take_stats(), **self.net.summary()})

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
        a = self.gw.alloc
        for rec in self.announced:
            t = a.tasks.get(int(rec["task_id"][len(ft.TASK_PREFIX):]))
            if t is not None and t.state != "done":
                return False
        return True

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
        self.record({"s": "end", "reason": reason, "sim_s": round(t, 3), "wall_s": round(wall, 3),
                     "summary": summary})
        return summary
