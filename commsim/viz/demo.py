"""Live comms model behind the operator view's comms overlay: the real DES
(commsim/des/sim.py — the same radio model, batman-style routing and
core/protocol.py allocator the sweeps use) stepped in real time.

webui/serve.py owns one `Hub` and streams its events to the page as
Server-Sent Events (GET /api/commsim/events); the page's commands come back as
JSON (POST /api/commsim/cmd). There is no server here any more: the operator
view is the one GUI (CLAUDE.md, "One GUI"), and this module is its engine.

In the overlay the model's robots stand where the REAL robots are (the page
moves them as their poses change), its racks are the explored map's
occupied cells (`layout` command, commsim/des/layout.py from_occupancy) and
its gateway is the operator's gateway pin. What is real and what is not: the
radio range, contention, routing, every message's size and timing, leases
and silence detection are the model's. The overlay never posts tasks to it —
the real fleet's auction is on /fleet/task_board — so its robots never drive.
"""
from __future__ import annotations

import argparse
import json
import math
import queue
import sys
import threading
import time
import traceback
from pathlib import Path

from commsim.core.fleet import load_config
from commsim.des import radio as R
from commsim.des.layout import Layout, from_occupancy
from commsim.des.sim import CLAIMED, GW, Frame, Sim

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
BASE_CFG = REPO / "commsim/configs/base.yaml"

# DES frame -> page kind (colours/labels live in webui/operator_comms.js)
MSG_KIND = {"Status": "status", "StatusLite": "status", "Pose": "status", "TaskAnnounce": "announce",
            "Bid": "bid", "Claim": "claim", "LeaseBeacon": "lease", "TaskEvent": "event",
            "ObstacleEvent": "obstacle", "ObstacleList": "obstacle", "MapUpdate": "map",
            "LinkHealth": "linkhealth", "KA": "ka", "Custom": "custom", "PathMsg": "status"}


def frame_kind(f: Frame) -> tuple[str, dict]:
    """Kind plus the message identity (type, src, dst) of one frame."""
    if f.kind == "ack":
        return "tcp_ack", {}
    if f.kind == "ogm":
        return "ogm", {}
    if f.kind == "bg":
        return "other", {}
    msg = f.msg
    if f.kind == "seg":
        if f.seg is None or not f.seg.msgs:
            return "zenoh", {"dst": f.final}
        msg = f.seg.msgs[0]
    if not msg:
        return "other", {}
    x = {"src": msg.get("src"), "dst": f.final, "type": msg.get("type")}
    if msg.get("data") and isinstance(msg["data"], dict) and "task_id" in msg["data"]:
        x["task_id"] = msg["data"]["task_id"]
    return MSG_KIND.get(msg.get("type"), "other"), x


class DemoSim:
    """One live simulation. `emit` receives JSON-able dict events; all
    methods are called from the stepping thread (commands come through
    `queue`), so nothing here locks."""

    def __init__(self, emit, n: int, variant: str, layout: "str | Layout", seed: int = 1,
                 positions: list | None = None, range_m: float | None = None):
        self.emit = emit
        custom = layout if isinstance(layout, Layout) else None
        self.layout_name = "map" if custom else layout
        self.n, self.variant, self.seed = n, variant, seed
        cfg = load_config(BASE_CFG)
        des = cfg.setdefault("commsim_des", {})
        des["task_rate_per_robot_hz"] = 0.0   # tasks come from the viewer
        des["obstacle_rate_hz"] = 1e-9          # (expovariate needs > 0; mean 30 years)
        self.cfg = cfg
        self.sim = Sim(cfg, variant, n, seed=seed, layout="open_floor" if custom else layout,
                       warmup_s=0.0, steady_s=1e12, status_transport="udp", positions=positions)
        self.sim.obstacle_rate = 1e-9
        if custom:
            # Sim builds its layout by name; swap the map's in before start()
            # runs the first topo(), and keep every node on the floor.
            self.sim.layout = custom
            if not positions:
                self.sim.nodes[GW].pos = tuple(custom.gateway_pos)
            self._clamp_all()
        self.band0 = self.sim.bands[0]
        self.last_routes: dict = {}
        self.last_pos: dict = {}
        self.task_info: dict = {}
        self.custom_seq = 0
        self._hook()
        if range_m:
            rp = self.sim.rp
            rp.tx_dbm += 10.0 * rp.pl_exponent * math.log10(range_m / max(self.open_air_range(False), 1.0))
        self.sim.start()
        self.t_last_now = -1.0

    # ---- hooks into the model -------------------------------------------------

    def us(self, t: float) -> int:
        return int(round(t * 1e6))

    def _hook(self) -> None:
        sim = self.sim
        for ch in sim.ch.values():
            ch._end = self._wrap_end(ch, ch._end)
        orig_topo, orig_deliver, orig_on_event = sim.topo, sim.deliver, sim.on_event
        orig_done = sim.task_done

        def topo():
            orig_topo()
            self._routes_changed()
        sim.topo = topo

        def deliver(dst, msg, hops, via_bcast=False):
            orig_deliver(dst, msg, hops, via_bcast)
            if sim.nodes[dst].alive:
                # `hops` from the model accumulates over TCP retransmissions (a
                # segment tried twice over two hops reports 4); the viewer wants
                # the path it travelled, so report the route length instead.
                self.emit({"t": self.us(sim.now), "k": "app_rx", "src": msg["src"], "dst": dst,
                           "type": msg["type"], "lat_us": self.us(sim.now - msg["t0"]),
                           "hops": self.route_hops(msg["src"], dst) or hops, "tries_hops": hops,
                           "kind": MSG_KIND.get(msg["type"], "other")})
        sim.deliver = deliver

        def on_event(r, e):
            orig_on_event(r, e)
            k = e["event"]
            ev = {"t": self.us(sim.now), "k": "fleet", "node": r.id, "event": k}
            for key in ("peer", "task_id", "cost", "owner", "to", "reason", "round"):
                if key in e:
                    ev[key] = e[key]
            if k == "robot_silent":
                self.emit({"t": ev["t"], "k": "robot_silent", "obs": r.id, "peer": e["peer"]})
            elif k == "robot_back":
                self.emit({"t": ev["t"], "k": "robot_back", "obs": r.id, "peer": e["peer"]})
            self.emit(ev)
            if k == "claimed":
                self._task(e["task_id"], "claimed", owner=r.id, cost=e.get("cost"))
            elif k == "committed":
                self._task(e["task_id"], "carrying", owner=r.id)
            elif k in ("stop_task", "task_yielded") and r.id == self.task_owner(e["task_id"]):
                self._task(e["task_id"], "open")
            elif k == "lease_expired" and r.is_gw:
                self._task(e["task_id"], "open")
        sim.on_event = on_event

        def task_done(tid):
            orig_done(tid)
            self._task(tid, "done")
        sim.task_done = task_done

    def _wrap_end(self, ch, orig):
        sim = self.sim

        def end(winners):
            now = sim.now
            before = []
            for s in winners:
                f = s.q[0]
                before.append((s, f, ch._airtime(f), s.retries))
            drops0 = sim.m["mac_drops"]
            orig(winners)
            collided = len(winners) > 1
            for s, f, dur, retries in before:
                kind, x = frame_kind(f)
                popped = not (s.q and s.q[0] is f)
                e = {"t": self.us(now - dur), "k": "air", "ta": f.src,
                     "ra": -1 if f.nh is None else f.nh, "kind": kind, "size": f.size,
                     "band": ch.band, "dur": self.us(dur), "rate": 0.0, "mseq": 0}
                if f.nh is None:
                    e["ack"] = None
                    e["col"] = collided
                else:
                    e["ack"] = bool(popped and not collided and sim.m["mac_drops"] == drops0)
                    if collided:
                        e["col"] = True
                    if not popped:
                        e["retry"] = s.retries
                    if f.final is not None:
                        e["dest"] = f.final
                        e["hop"] = f.hops
                        if f.nh == f.final and e["ack"]:
                            e["ok"] = "ok"
                        elif not e["ack"] and popped:
                            e["ok"] = "lost"
                for key in ("src", "dst", "type", "task_id"):
                    if key in x and x[key] is not None:
                        e[key] = x[key]
                self.emit(e)
        return end

    def _routes_changed(self) -> None:
        sim = self.sim
        cur = {}
        for (node, dst), (nh, band) in sim.next_hop.items():
            tq = sim.tq[band].get((node, nh), 0.0)
            cur[(node, dst)] = (nh, int(round(tq * 255)))
        t = self.us(sim.now)
        for key, val in cur.items():
            if self.last_routes.get(key) != val:
                self.emit({"t": t, "k": "route", "node": key[0], "dst": key[1], "nh": val[0], "tq": val[1]})
        for key in self.last_routes:
            if key not in cur:
                self.emit({"t": t, "k": "route", "node": key[0], "dst": key[1], "nh": -1, "tq": 0})
        self.last_routes = cur

    def route_hops(self, src: int, dst: int) -> int:
        cur, n = src, 0
        while cur != dst and n < len(self.sim.nodes):
            h = self.sim.hop(cur, dst)
            if h is None:
                return 0
            cur, n = h[0], n + 1
        return n

    def task_owner(self, tid):
        t = self.sim.nodes[GW].alloc.tasks.get(tid)
        return t.owner if t is not None and t.state == CLAIMED else None

    def _task(self, tid, state, **kw) -> None:
        info = self.task_info.get(tid)
        if info is None:
            return
        info["state"] = state
        if "owner" in kw:
            info["owner"] = kw["owner"]
        if state in ("open", "done"):
            info["owner"] = None
        self.emit({"t": self.us(self.sim.now), "k": "task", "task_id": tid, "state": state,
                   "pickup": info["pickup"], "dropoff": info["dropoff"], "owner": info.get("owner"),
                   **{k: v for k, v in kw.items() if k != "owner"}})

    # ---- what the page needs first -------------------------------------------

    def snr_matrix(self) -> list:
        n1 = len(self.sim.nodes)
        m = [[0.0] * n1 for _ in range(n1)]
        for (i, j), s in self.sim.snr[self.band0].items():
            m[i][j] = round(s, 1)
        return m

    def open_air_range(self, gateway: bool) -> float:
        """Distance at which the weakest unicast rate is lost with no racks in
        the way — the range ring the page draws while a robot is dragged."""
        rp = self.sim.rp
        lo, hi = 1.0, 2000.0
        for _ in range(40):
            mid = (lo + hi) / 2
            s = R.snr_db(rp, self.band0, mid, 0, 0, gateway=gateway)
            if R.pick_unicast_rate(s, ht=bool(rp.ht), cap=rp.max_rate_mbps):
                lo = mid
            else:
                hi = mid
        return round(lo, 1)

    def set_range(self, metres: float) -> None:
        """Scale transmit power so the open-air range is `metres`. The model's
        placeholder 20 dBm reaches ~210 m in free space — further than the
        50 m floor is wide — so without this a demo could never show a hop
        through the gateway; racks are the only thing that shortens links."""
        rp = self.sim.rp
        r0 = self.open_air_range(False)
        rp.tx_dbm += 10.0 * rp.pl_exponent * math.log10(max(metres, 1.0) / max(r0, 1.0))
        self.sim.topo()
        self.emit({"t": self.us(self.sim.now), "k": "snr", "snr": self.snr_matrix(),
                   "range_m": {"robot": self.open_air_range(False), "gateway": self.open_air_range(True)}})

    def header(self) -> dict:
        sim, lay = self.sim, self.sim.layout
        return {
            "name": f"live demo · {self.variant}", "n": len(sim.nodes), "transport": "udp status, zenoh control",
            "hz": sim.status_hz, "orig_ms": int(sim.orig_s * 1000), "duration_s": None, "t0_ns": 0,
            "live": True, "variant": self.variant, "bands": sim.bands,
            "layout": {"name": self.layout_name, "width": lay.width, "height": lay.height,
                       "racks": lay.racks, "groups": lay.groups, "shelves": lay.shelves, "aisles_y": lay.aisles_y,
                       "cross_x": list(lay.cross_x), "gateway_pos": list(lay.gateway_pos)},
            "nodes": [{"id": i, "ns": f"n{i}", "ip": f"10.42.0.{i + 1}", "mac": f"02:00:00:00:{i:02x}:00",
                       "pos": list(r.pos), "gateway": r.is_gw, "alive": r.alive}
                      for i, r in sorted(sim.nodes.items())],
            "snr": self.snr_matrix(),
            # The routes as they stand. Building a DemoSim runs topo() and emits
            # its routes BEFORE the Hub can send this header, and a page clears
            # its routes on init: without these it showed none until one changed.
            "routes": [[node, dst, nh, tq] for (node, dst), (nh, tq) in sorted(self.last_routes.items())],
            "range_m": {"robot": self.open_air_range(False), "gateway": self.open_air_range(True)},
            "knobs": {"status_timeout_s": sim.params.status_timeout_s, "status_hz": sim.status_hz,
                      "orig_interval_ms": int(sim.orig_s * 1000),
                      "zenoh_lease_ms": int(sim.lease_s * 1000), "task_lease_s": sim.params.task_lease_s,
                      "bid_window_s": sim.params.bid_window_s, "snr_link_min_db": 4.0,
                      "speed_mps": sim.speed,
                      # What an obstacle costs a link, so the overlay can say why one failed.
                      "rack_db": sim.rp.rack_db_g24, "shelf_db": sim.rp.shelf_db,
                      "gateway_rack_factor": sim.rp.gateway_rack_factor},
            "counts": {"air_by_kind": {}},
            "tasks": [dict(task_id=k, **v) for k, v in self.task_info.items()],
            "limits": [
                "the real DES: radio range, contention, routing, auction and leases are the model's",
                "tasks and obstacles arrive when you ask; robots stand where you put them until they win a task",
                "single contention domain per band; range rings are open-air (racks shorten real links)",
            ],
        }

    # ---- commands from the page -----------------------------------------------

    def _clamp(self, x: float, y: float) -> tuple:
        lay = self.sim.layout
        return (min(max(x, 0.0), lay.width), min(max(y, 0.0), lay.height))

    def _clamp_all(self) -> None:
        for r in self.sim.nodes.values():
            r.pos = self._clamp(*r.pos)

    def set_layout(self, layout: Layout) -> None:
        """New racks from the map (it grows while the robots explore). In
        place, not a reset: link state, routes and silence survive."""
        self.sim.layout = layout
        self.layout_name = "map"
        self._clamp_all()
        self.sim.topo()
        t = self.us(self.sim.now)
        self.emit({"t": t, "k": "layout", "width": layout.width, "height": layout.height,
                   "racks": len(set(layout.groups)) if layout.groups else len(layout.racks),
                   # The geometry itself, for the overlay's "which obstacle hurt this link".
                   "rects": layout.racks, "groups": layout.groups, "shelves": layout.shelves})
        for i, r in sorted(self.sim.nodes.items()):
            self.emit({"t": t, "k": "pos", "id": i, "pos": list(r.pos)})
        self.emit({"t": t, "k": "snr", "snr": self.snr_matrix()})

    def set_gateway(self, x: float, y: float) -> None:
        gw = self.sim.nodes[GW]
        gw.pos = self._clamp(x, y)
        self.sim.layout.gateway_pos = gw.pos
        self.sim.topo()
        self.emit({"t": self.us(self.sim.now), "k": "pos", "id": GW, "pos": list(gw.pos)})
        self.emit({"t": self.us(self.sim.now), "k": "snr", "snr": self.snr_matrix()})

    def move(self, rid: int, x: float, y: float) -> None:
        r = self.sim.nodes.get(rid)
        if r is None or r.is_gw:
            return
        r.pos = self._clamp(x, y)
        if r.task is not None:  # re-plan the drive from where it now stands
            tid = r.task
            r.task, r.route = None, []
            r.start_task(tid)
        self.sim.topo()
        self.emit({"t": self.us(self.sim.now), "k": "pos", "id": rid, "pos": list(r.pos)})
        self.emit({"t": self.us(self.sim.now), "k": "snr", "snr": self.snr_matrix()})

    def obstacle(self, rid: int) -> None:
        r = self.sim.nodes.get(rid)
        if r is None or r.is_gw or not r.alive:
            return
        self.sim._publish_obstacle(r)
        self.emit({"t": self.us(self.sim.now), "k": "obstacle", "id": rid, "pos": list(r.pos)})

    def task(self, pickup=None, dropoff=None) -> int:
        sim = self.sim
        rng = sim.rng
        pickup = tuple(pickup) if pickup else sim.layout.random_spot(rng)
        dropoff = tuple(dropoff) if dropoff else sim.layout.random_spot(rng)
        sim.task_seq += 1
        tid = sim.task_seq
        sim.tasks[tid] = {"t": sim.now, "done": False, "claimed_at": None, "eligible": True}
        self.task_info[tid] = {"pickup": list(pickup), "dropoff": list(dropoff), "state": "announced",
                               "owner": None, "t": self.us(sim.now)}
        gw = sim.nodes[GW]
        gw.alloc.announce(tid, pickup, dropoff, sim.now)
        sim.flush(gw)
        self._task(tid, "announced")
        return tid

    def cancel(self, tid: int) -> None:
        gw = self.sim.nodes[GW]
        if tid in gw.alloc.tasks:
            gw.alloc.cancel(tid, self.sim.now)
            self.sim.flush(gw)
            self._task(tid, "cancelled")

    def link(self, a: int, b: int, action: str) -> None:
        if action == "block":
            self.sim.block_link(a, b)
        else:
            self.sim.unblock_link(a, b)
        self.emit({"t": self.us(self.sim.now), "k": "link", "a": a, "b": b, "action": action})

    def node(self, rid: int, action: str) -> None:
        if rid not in self.sim.nodes:
            return
        if action == "kill":
            self.sim.kill(rid)
        else:
            self.sim.revive(rid, fresh=(action == "reboot"))
        self.emit({"t": self.us(self.sim.now), "k": "node", "id": rid,
                   "action": "revive" if action != "kill" else "kill"})

    def send(self, src: int, dst: int, size: int = 200, reliable: bool = True) -> None:
        if src not in self.sim.nodes or dst not in self.sim.nodes or src == dst:
            return
        self.custom_seq += 1
        self.sim.publish(src, "Custom", int(size), [dst], {"seq": self.custom_seq}, reliable=reliable)
        self.emit({"t": self.us(self.sim.now), "k": "fleet", "node": src, "event": "sent_custom",
                   "to": dst, "seq": self.custom_seq, "size": int(size)})

    # ---- stepping ---------------------------------------------------------------

    def step(self, until: float) -> None:
        """Run the model to `until` and leave its clock THERE: Sim.now only
        moves when an event pops, so a loop that targets `now + dt` with the
        first event further out than dt never runs anything (found 2026-10-20)."""
        sim = self.sim
        sim.step(until)
        sim.now = max(sim.now, until)
        t = self.us(sim.now)
        for i, r in sim.nodes.items():
            p = (round(r.pos[0], 2), round(r.pos[1], 2))
            if self.last_pos.get(i) != p:
                self.last_pos[i] = p
                if i in self.last_pos and r.route:  # only robots that are driving
                    self.emit({"t": t, "k": "pos", "id": i, "pos": list(p)})
        if sim.now - self.t_last_now >= 0.1:
            self.t_last_now = sim.now
            owned = {i: [tid for tid, _ in r.alloc.owned()] for i, r in sim.nodes.items() if not r.is_gw}
            self.emit({"t": t, "k": "now", "owned": owned,
                       "busy": {b: round(c.busy_acc, 3) for b, c in sim.ch.items()}})


# ---- engine --------------------------------------------------------------------------

class Hub:
    """Owns the sim thread, fans events out to SSE clients, applies commands.

    webui/serve.py makes one, lazily, on the first /api/commsim/events. The
    thread steps the model only while someone is subscribed: serve.py runs on
    the Jetson too, and a DES spinning for nobody would burn its CPU."""

    def __init__(self, n: int = 2, variant: str = "V3", layout: "str | Layout" = "open_floor",
                 range_m: float | None = 8.0):
        self.clients: list[queue.Queue] = []
        self.cmds: queue.Queue = queue.Queue()
        self.lock = threading.Lock()
        self.speed = 1.0
        self.paused = False
        self.verbose = False
        self.args = (n, variant, layout)
        self.range_m = range_m
        self.sim = DemoSim(self.emit, n, variant, layout, range_m=range_m)
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def emit(self, ev: dict) -> None:
        data = json.dumps(ev, separators=(",", ":"))
        with self.lock:
            for q in self.clients:
                if q.qsize() < 20000:
                    q.put(data)

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue()
        with self.lock:
            self.clients.append(q)
        q.put(json.dumps({"k": "init", "header": self.sim.header(), "speed": self.speed,
                          "paused": self.paused}, separators=(",", ":")))
        return q

    def unsubscribe(self, q) -> None:
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)

    def reset(self, n: int | None = None, variant: str | None = None, layout=None,
              positions: list | None = None) -> None:
        n0, v0, l0 = self.args
        # The map's layout outlives a reset (a robot joined): only a new
        # `layout` command replaces it.
        self.args = (n or n0, variant or v0, layout or (self.sim.sim.layout if self.sim.layout_name == "map" else l0))
        keep = self.sim.open_air_range(False)
        gw = self.sim.sim.nodes[GW].pos
        if positions is None and self.sim.layout_name == "map":
            positions = [list(gw)]
        elif positions is not None and len(positions) and positions[0] is None:
            positions = [list(gw)] + positions[1:]
        self.sim = DemoSim(self.emit, *self.args, positions=positions, range_m=keep)
        self.emit({"k": "init", "header": self.sim.header(), "speed": self.speed, "paused": self.paused})

    def apply(self, c: dict) -> None:
        s, op = self.sim, c.get("op")
        if op == "move":
            s.move(int(c["id"]), float(c["x"]), float(c["y"]))
        elif op == "obstacle":
            s.obstacle(int(c["id"]))
        elif op == "task":
            s.task(c.get("pickup"), c.get("dropoff"))
        elif op == "cancel":
            s.cancel(int(c["task_id"]))
        elif op == "link":
            s.link(int(c["a"]), int(c["b"]), c.get("action", "block"))
        elif op == "node":
            s.node(int(c["id"]), c.get("action", "kill"))
        elif op == "send":
            s.send(int(c["src"]), int(c["dst"]), int(c.get("size", 200)), bool(c.get("reliable", True)))
        elif op == "range":
            s.set_range(float(c["metres"]))
        elif op == "layout":
            s.set_layout(from_occupancy(str(c["occ"]), int(c["w"]), int(c["h"]), float(c["cell"]),
                                        gateway_pos=s.sim.nodes[GW].pos))
        elif op == "gateway":
            s.set_gateway(float(c["x"]), float(c["y"]))
        elif op == "speed":
            self.speed = max(0.05, min(20.0, float(c["speed"])))
            self.emit({"k": "ctl", "speed": self.speed, "paused": self.paused})
        elif op == "pause":
            self.paused = bool(c.get("paused", True))
            self.emit({"k": "ctl", "speed": self.speed, "paused": self.paused})
        elif op == "reset":
            pos = c.get("positions")
            self.reset(c.get("n"), c.get("variant"), None, pos)

    def loop(self) -> None:
        last = time.monotonic()
        report = last
        while True:
            if time.monotonic() - report > 2.0 and self.verbose:
                report = time.monotonic()
                print(f"[commsim] sim t={self.sim.sim.now:.1f} s, {len(self.clients)} client(s), "
                      f"queue {len(self.sim.sim._q)}, paused={self.paused}", file=sys.stderr, flush=True)
            try:
                while True:
                    self.apply(self.cmds.get_nowait())
            except queue.Empty:
                pass
            except Exception as exc:  # a bad command must not stop the sim
                traceback.print_exc()
                self.emit({"k": "error", "text": f"{type(exc).__name__}: {exc}"})
            now = time.monotonic()
            dt, last = now - last, now
            with self.lock:
                watched = bool(self.clients)
            if watched and not self.paused:
                try:
                    self.sim.step(self.sim.sim.now + min(dt, 0.5) * self.speed)
                except Exception as exc:
                    traceback.print_exc()
                    self.emit({"k": "error", "text": f"sim: {type(exc).__name__}: {exc}"})
                    self.paused = True
            time.sleep(0.02 if watched else 0.25)


def main(argv=None) -> int:
    argparse.ArgumentParser(description=__doc__,
                            formatter_class=argparse.RawDescriptionHelpFormatter).parse_args(argv)
    print("The comms model now runs inside the operator view (CLAUDE.md, \"One GUI\"):\n"
          "    python3 webui/serve.py --port 8081    # or scripts/run_console.sh\n"
          "then open http://localhost:8081/operator.html and turn on Comms in the map tools.",
          file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
