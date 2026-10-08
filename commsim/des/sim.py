"""commsim discrete-event model: the fleet's comms stack, n robots, one run.

Layers modeled (plan §2), each with its main simplification:
  - Medium: slotted CSMA/CA per band (DIFS, random backoff with frozen
    counters, binary exponential CW, 7 retries, ACKs). ONE contention domain
    per band — every node defers to every other. That overstates contention
    on large floors and hides hidden terminals; the emulation (wmediumd) is
    the cross-check.
  - Mesh routing: batman-adv-like best-TQ path (OGM loss, hop penalty),
    recomputed every topo tick; OGM airtime is charged in full. Reroute
    after a failure waits `reroute_delay_s`.
  - Middleware: zenohd full mesh of TCP sessions (explicit connect to every
    peer), one copy per subscriber router, TCP with cwnd, delayed ACKs, RTO
    retransmission, in-order delivery, keepalives on idle sessions, and
    best-effort drop under backlog.
  - App: the real core.protocol.Allocator per robot (auction, leases), task
    workload, aisle-following motion, status/obstacle/map/link-health traffic.

All tunables come from the run config; PLACEHOLDER values are documented in
configs/base.yaml and des/radio.py.
"""
from __future__ import annotations

import heapq
import math
import random
import time
from collections import defaultdict, deque

from commsim.core.protocol import Allocator, Params, CLAIMED, COMPLETE
from . import radio as R
from .layout import LAYOUTS, path_length

GW = 0
QLIMIT = 1000                 # per-interface tx queue (frames), tail drop
# MEASURED in emulation (M1, 3-node line, 2026-10-03): 1-hop p50 3.89 ms,
# 2-hop p50 4.89 ms -> ~2.9 ms fixed rclpy/zenohd/TCP stack delay end to end,
# ~1.0 ms per extra hop of which ~0.25 ms is airtime. The per-hop part
# includes wmediumd's userspace forwarding — replace with hardware numbers.
# Stack delay = 2.4 ms + Exp(mean 0.7 ms): fits the emulated 1-hop p50/p95
# (3.89 / 5.67 ms) — the real stack's scheduling jitter, not just a constant.
STACK_BASE_S, STACK_JITTER_S = 2.4e-3, 0.7e-3
PROC_S = 0.75e-3
BG_AIRTIME_S = 1e-3           # one background (campus) frame
TOPO_DT = 1.0
HOP_PENALTY = 15 / 255

VARIANTS = {
    # mesh: batman multi-hop vs single AP; split: V1 separate messages;
    # neighbor: neighbor-cell status + local auctions; bcast_status: V5 one-hop
    # status broadcast; flood_obstacles: V6 reliable obstacle broadcast.
    "phase0": dict(mesh=False, bands=["g24"], split=False, neighbor=True),
    "V1": dict(mesh=True, bands=["g24"], split=True, neighbor=False),
    "V2": dict(mesh=True, bands=["g24"], split=False, neighbor=False),
    "V3": dict(mesh=True, bands=["g24"], split=False, neighbor=True),
    "V4": dict(mesh=True, bands=["g24", "g5"], split=False, neighbor=True),
    "V5": dict(mesh=True, bands=["g24", "g5"], split=False, neighbor=True, bcast_status=True),
    "V6": dict(mesh=True, bands=["g24", "g5"], split=False, neighbor=True, bcast_status=True,
               flood_obstacles=True),
}
CONTROL = {"Status", "Pose", "StatusLite", "PathMsg", "ObstacleList", "Bid", "Claim",
           "ObstacleEvent"}
STATUS_TYPES = {"Status", "Pose", "StatusLite", "PathMsg"}
RELIABLE = {"TaskAnnounce", "Bid", "Claim", "TaskEvent", "ObstacleEvent", "LeaseBeacon"}


def pct(xs, q):
    if not xs:
        return math.nan
    s = sorted(xs)
    return s[min(len(s) - 1, int(math.ceil(q / 100 * len(s))) - 1)]


# ---------------------------------------------------------------------------
# medium

class Frame:
    __slots__ = ("kind", "size", "src", "nh", "final", "band", "seg", "ack", "msg",
                 "prev_band", "fid", "hops")

    def __init__(self, kind, size, src, final=None, seg=None, ack=None, msg=None, fid=None):
        self.kind, self.size, self.src, self.final = kind, size, src, final
        self.seg, self.ack, self.msg, self.fid = seg, ack, msg, fid
        self.nh = self.band = self.prev_band = None
        self.hops = 0


class Station:
    __slots__ = ("node", "band", "q", "cw", "retries", "bo", "join")

    def __init__(self, node, band):
        self.node, self.band = node, band
        self.q = deque()
        self.cw, self.retries, self.bo, self.join = R.CW_MIN, 0, None, 0.0


class Channel:
    def __init__(self, sim, band):
        self.sim, self.band = sim, band
        self.slot = R.SLOT_US * 1e-6
        self.difs = R.difs_us(band) * 1e-6
        self.busy_until = 0.0
        self.idle_since = 0.0
        # dict, not set: a set of Station objects iterates in id()/memory order,
        # which differs between processes — same seed, different contention
        # winners, different results. Insertion order is deterministic.
        self.cont: dict = {}
        self.ver = 0
        self.busy_acc = 0.0
        self.tx = 0
        self.collisions = 0

    def enqueue(self, st: Station, f: Frame) -> bool:
        if len(st.q) >= QLIMIT:
            self.sim.m["q_drops"] += 1
            return False
        st.q.append(f)
        if st not in self.cont:
            if st.bo is None:
                st.bo = self.sim.rng.randint(0, st.cw)
            st.join = self.sim.now
            self.cont[st] = None
            self._reschedule()
        return True

    def _cand(self, st):
        return max(st.join, self.idle_since + self.difs) + st.bo * self.slot

    def _reschedule(self):
        if self.sim.now < self.busy_until or not self.cont:
            return
        t = min(self._cand(s) for s in self.cont)
        self.ver += 1
        self.sim.at(max(t, self.sim.now), self._resolve, self.ver)

    def _airtime(self, f: Frame) -> float:
        if f.kind == "bg":
            return BG_AIRTIME_S
        if f.nh is None:  # broadcast
            return R.frame_airtime_us(f.size, self.sim.rp.multicast_rate_mbps, self.band,
                                      unicast=False, legacy=True) * 1e-6
        rt = self.sim.rate[self.band].get((f.src, f.nh))
        rate = rt[0] if rt else R.HT_RATES[0][0]
        return R.frame_airtime_us(f.size, rate, self.band, unicast=True,
                                  legacy=not self.sim.rp.ht) * 1e-6

    def _resolve(self, ver):
        if ver != self.ver or not self.cont:
            return
        now = self.sim.now
        cands = [(self._cand(s), s) for s in self.cont]
        tmin = min(c for c, _ in cands)
        winners = []
        for c, s in cands:
            if c - tmin < 1e-9:
                winners.append(s)
            else:
                s.bo = max(0, int(round((c - tmin) / self.slot)))
                s.join = -math.inf
        for s in winners:
            self.cont.pop(s, None)
        dur = max(self._airtime(s.q[0]) for s in winners)
        self.busy_until = now + dur
        self.tx += len(winners)
        if len(winners) > 1:
            self.collisions += 1
        w0, w1 = self.sim.window
        span = max(0.0, min(self.busy_until, w1) - max(now, w0))
        self.busy_acc += span
        if span:
            for s in winners:
                k = _airtime_label(s.q[0])
                self.sim.air_by[k] += span / len(winners)
        self.sim.at(self.busy_until, self._end, winners)

    def _end(self, winners):
        sim = self.sim
        now = sim.now
        self.idle_since = now
        collided = len(winners) > 1
        for s in winners:
            f = s.q[0]
            done = True
            if f.kind == "bg":
                pass
            elif f.nh is None:
                if not collided:
                    sim.on_bcast_air(s.node, f, self.band)
            else:
                ok = not collided and sim.link_ok(f.src, f.nh, self.band, f.size)
                if ok:
                    f.prev_band = self.band
                    sim.at(now + PROC_S, sim.on_frame_rx, f.nh, f)
                else:
                    s.retries += 1
                    if s.retries <= R.RETRY_LIMIT:
                        done = False
                        s.cw = min(2 * s.cw + 1, R.CW_MAX)
                    else:
                        sim.m["mac_drops"] += 1
            if done:
                s.q.popleft()
                s.retries, s.cw = 0, R.CW_MIN
            if s.q:
                s.bo = sim.rng.randint(0, s.cw)
                s.join = now
                self.cont[s] = None
            else:
                s.bo = None
        self._reschedule()


def _airtime_label(f) -> str:
    if f.kind == "seg":
        if f.seg is None:
            return "seg:raw"
        if not f.seg.msgs:
            return "seg:fragment"
        return "seg:" + f.seg.msgs[0]["type"]
    if f.kind == "ack":
        return "tcp_ack"
    return f.kind


# ---------------------------------------------------------------------------
# zenoh over TCP

class Segment:
    __slots__ = ("flow", "seq", "msgs", "payload", "tries", "hops", "sent_t", "piggy", "epoch")

    def __init__(self, flow, seq, msgs, payload):
        self.flow, self.seq, self.msgs, self.payload = flow, seq, msgs, payload
        self.tries, self.hops, self.sent_t, self.piggy = 0, 0, 0.0, None
        self.epoch = flow.epoch


class Flow:
    """zenohd router-to-router TCP session, one direction (src -> dst).

    Session lifecycle: when nothing arrives for one zenoh transport lease the
    session is closed (both directions, in-flight data lost) and the router
    re-dials every RECONNECT_S until a route exists. Each new session is a new
    `epoch`; stale segments/ACKs from an old epoch are ignored. The first DES
    version had no close/reconnect: after an outage, a receiver could wait
    forever behind a lost sequence number (60 s "gaps" in the first scenario
    runs, 2026-10-03)."""
    # Linux-like RTO: SRTT + 4*RTTVAR, floor 200 ms, initial 1 s, Karn's rule.
    # A FIXED 200 ms RTO was the first version: under queueing, ~75% of its
    # retransmissions were spurious and the extra load collapsed the channel.
    CWND, RTO_MIN, RTO_INIT, BE_BACKLOG = 10, 0.2, 1.0, 32
    RECONNECT_S = 1.0  # PLACEHOLDER: zenoh connect retry period

    def __init__(self, sim, src, dst):
        self.sim, self.src, self.dst = sim, src, dst
        self.rev = None  # the opposite direction of the same TCP connection
        self.epoch = 0
        self.app_t = 0.0  # time the last message was handed to the receiving app
        self.down = False
        self.retry_at = 0.0
        self._fresh()

    def _fresh(self):
        self.next_seq = 0
        self.inflight: dict = {}
        self.backlog: deque = deque()
        self.last_tx = self.sim.now
        self.srtt = None
        self.rttvar = 0.0
        self.rto = self.RTO_INIT
        self.expected = 0
        self.ooo: dict = {}
        self.unacked = 0
        self.ack_pending = False
        # opened_at, not last_rx: last_rx must only ever mean "data actually
        # arrived" — liveness feeds it to the allocator. Resetting last_rx on
        # close/reopen made an isolated robot "hear" its own reconnect attempts
        # and stop 5 s after its lease (found 2026-10-03, worst-case reroute).
        self.opened_at = self.sim.now
        if not hasattr(self, "last_rx"):
            self.last_rx = self.sim.now

    def close(self):
        """Lease expired: tear down both directions of this TCP session."""
        for f in (self, self.rev):
            f.epoch += 1
            f.down = True
            f.retry_at = self.sim.now + self.RECONNECT_S
            f._fresh()

    def reopen(self):
        for f in (self, self.rev):
            f.epoch += 1
            f.down = False
            f._fresh()

    def send(self, msg):
        if self.down:
            # no session: zenoh has nowhere to put it (no queue for absent peers)
            self.sim.m["session_down_drops"] += 1
            return
        if len(self.inflight) >= self.CWND:
            if not msg["reliable"] and len(self.backlog) >= self.BE_BACKLOG:
                self.sim.m["be_drops"] += 1
                return
            self.backlog.append(msg)
            return
        self._send_batch([msg])

    def _send_batch(self, msgs):
        payload = R.ZENOH_PER_BATCH + sum(m["wire"] for m in msgs)
        # Larger-than-MSS messages (map patches) go out as several segments;
        # the msgs ride on the last one — TCP delivers in order anyway.
        while payload > R.TCP_MSS:
            self._tx(Segment(self, self._seq(), [], R.TCP_MSS))
            payload -= R.TCP_MSS
        self._tx(Segment(self, self._seq(), msgs, payload))

    def _seq(self):
        s = self.next_seq
        self.next_seq += 1
        return s

    def _tx(self, seg):
        self.inflight[seg.seq] = seg
        seg.tries += 1
        seg.sent_t = self.last_tx = self.sim.now
        # Piggyback a pending ACK for the reverse session (its receiver is us).
        rev = self.rev
        if rev.unacked:
            rev.unacked = 0
            seg.piggy = rev.expected
        f = Frame("seg", R.UNICAST_FRAME_OVERHEAD + seg.payload, self.src, final=self.dst, seg=seg)
        self.sim.net_send(self.src, f)
        self.sim.at(self.sim.now + min(60.0, self.rto * 2 ** (seg.tries - 1)), self._rto, seg)

    def _rto(self, seg):
        if seg.epoch != self.epoch or self.inflight.get(seg.seq) is not seg:
            return
        if not self.sim.nodes[self.src].alive:
            return
        self.sim.m["tcp_retx"] += 1
        self._tx(seg)

    def on_ack(self, ack, epoch):
        if epoch != self.epoch:
            return
        now = self.sim.now
        for s in [s for s in self.inflight if s < ack]:
            seg = self.inflight.pop(s)
            if seg.tries == 1:  # Karn: never sample a retransmitted segment
                r = now - seg.sent_t
                if self.srtt is None:
                    self.srtt, self.rttvar = r, r / 2
                else:
                    self.rttvar = 0.75 * self.rttvar + 0.25 * abs(self.srtt - r)
                    self.srtt = 0.875 * self.srtt + 0.125 * r
                self.rto = max(self.RTO_MIN, self.srtt + 4 * self.rttvar)
        while self.backlog and len(self.inflight) < self.CWND:
            batch, size = [], R.ZENOH_PER_BATCH
            while self.backlog and (not batch or size + self.backlog[0]["wire"] <= R.TCP_MSS):
                m = self.backlog.popleft()
                batch.append(m)
                size += m["wire"]
            self._send_batch(batch)

    # receiver side
    def on_seg(self, seg):
        sim = self.sim
        if seg.epoch != self.epoch:
            return  # a segment from a session that has since been torn down
        self.last_rx = sim.now
        if seg.piggy is not None:
            self.rev.on_ack(seg.piggy, self.rev.epoch)
        if seg.seq == self.expected:
            self._deliver(seg)
            while self.expected in self.ooo:
                self._deliver(self.ooo.pop(self.expected))
        elif seg.seq > self.expected:
            self.ooo[seg.seq] = seg
            self._send_ack()
            return
        else:
            self._send_ack()
            return
        self.unacked += 1
        if self.unacked >= 2:
            self._send_ack()
        elif not self.ack_pending:
            self.ack_pending = True
            sim.at(sim.now + 0.04, self._delayed_ack)

    def _deliver(self, seg):
        self.expected = seg.seq + 1
        for m in seg.msgs:
            d = STACK_BASE_S + self.sim.rng.expovariate(1 / STACK_JITTER_S)
            # TCP hands the app bytes in order: jitter may delay a message but
            # never let it overtake an earlier one on the same session
            t = max(self.sim.now + d, self.app_t + 1e-9)
            self.app_t = t
            self.sim.at(t, self.sim.deliver, self.dst, m, seg.hops)

    def _delayed_ack(self):
        self.ack_pending = False
        if self.unacked:  # still unacked: nothing outbound carried it
            self._send_ack()

    def _send_ack(self):
        self.unacked = 0
        f = Frame("ack", R.UNICAST_FRAME_OVERHEAD, self.dst, final=self.src,
                  ack=(self, self.expected, self.epoch))
        self.sim.net_send(self.dst, f)


# ---------------------------------------------------------------------------
# nodes / app

class Robot:
    def __init__(self, sim, rid, pos):
        self.sim, self.id, self.pos = sim, rid, pos
        self.alive = True
        self.is_gw = rid == GW
        self.route: list = []
        self.task = None
        self.leg = 0
        self.replanned = False
        self.new_obstacles: list = []
        self.map_seq = 0
        self.last_map_rx: dict = {}
        self.map_seq_at_sub: dict = {}
        self.last_pose_rx: dict = {}
        self.heard_since: dict = {}
        self.subs_cells: set = set()
        self.status_n = 0
        self.alloc = Allocator(rid, sim.params, None if self.is_gw else self.cost,
                               random.Random(sim.seed * 7919 + rid), is_gateway=self.is_gw)

    def cost(self, pickup):
        d = path_length(self.sim.layout.route(self.pos, pickup))
        return d / self.sim.speed, math.dist(self.pos, pickup)

    # motion
    def start_task(self, tid):
        t = self.alloc.tasks[tid]
        lay = self.sim.layout
        self.task = tid
        self.route = lay.route(self.pos, t.pickup)[1:] + lay.route(t.pickup, t.dropoff)[1:]
        self.leg = len(lay.route(self.pos, t.pickup)) - 1  # waypoints until pickup reached
        self.replanned = True

    def stop_task(self):
        self.task, self.route = None, []

    def move(self, dt):
        step = self.sim.speed * dt
        while self.route and step > 0:
            tgt = self.route[0]
            d = math.dist(self.pos, tgt)
            if d <= step:
                self.pos, step = tgt, step - d
                self.route.pop(0)
            else:
                f = step / d
                self.pos = (self.pos[0] + f * (tgt[0] - self.pos[0]),
                            self.pos[1] + f * (tgt[1] - self.pos[1]))
                step = 0
        if self.task is not None and not self.route:
            tid = self.task
            self.stop_task()
            self.alloc.complete(tid, self.sim.now)
            self.sim.task_done(tid)
            self.sim.flush(self)


class Sim:
    def __init__(self, cfg: dict, variant: str, n: int, seed: int = 1, bg_share: float = 0.0,
                 layout: str = "warehouse", warmup_s: float = 20.0, steady_s: float = 60.0,
                 lease_beacon_hz: float | None = None, transport_liveness: bool = True,
                 single_connect: bool | None = None, status_only: bool = False,
                 status_transport: str | None = None,
                 fixed_snr: dict | None = None, positions: list | None = None,
                 track_gaps: bool = False,
                 overrides: dict | None = None):
        self.cfg, self.variant, self.n, self.seed = cfg, variant, n, seed
        self.v = dict(VARIANTS[variant], **(overrides or {}))
        self.rng = random.Random(seed)
        self.params = Params.from_config(cfg)
        self.rp = R.RadioParams.from_config(cfg)
        self.layout = LAYOUTS[layout]()
        pc = cfg["protocol"]
        self.status_hz = pc["status_hz"]
        self.cell = float(cfg.get("commsim_des", {}).get("cell_size_m", 10.0))
        self.speed = float(cfg.get("commsim_des", {}).get("robot_speed_mps", 0.5))
        self.task_rate = float(cfg.get("commsim_des", {}).get("task_rate_per_robot_hz", 1 / 200))
        self.obstacle_rate = float(cfg.get("commsim_des", {}).get("obstacle_rate_hz", 0.02))
        self.map_patch_bytes = int(cfg.get("commsim_des", {}).get("map_patch_bytes", 3000))
        if lease_beacon_hz is not None:
            self.params.lease_beacon_hz = lease_beacon_hz
        self.lease_beacon_hz = self.params.lease_beacon_hz
        if single_connect is None:
            single_connect = cfg["zenoh"].get("connect", "lower_id") == "lower_id"
        self.status_transport = status_transport or pc.get("status_transport", "tcp")
        self.transport_liveness = transport_liveness
        self.bg_share = bg_share
        self.status_only = status_only    # M1 cross-check: Status (+keepalives) only
        self.fixed_snr = fixed_snr        # {(i, j): dB} replaces propagation (cross-check)
        self.lease_s = cfg["zenoh"]["transport_lease_ms"] / 1000
        self.ka_s = self.lease_s / cfg["zenoh"]["keep_alive"]
        self.orig_s = cfg["batman"]["orig_interval_ms"] / 1000
        self.window = (warmup_s, warmup_s + steady_s)
        self.end = warmup_s + steady_s
        self.bands = self.v["bands"]
        self.now = 0.0
        self._q: list = []
        self._seq = 0
        self.m = defaultdict(int)
        self.air_by = defaultdict(float)
        self.lat = defaultdict(list)
        self.hops = []
        self.st_sent = defaultdict(int)
        self.st_recv = defaultdict(int)
        self.auction = []
        self.map_lags = []
        self.stale = [0, 0]
        self.dup_episodes = 0
        self.dup_transient = 0
        self.abandoned = 0
        self.false_silent = 0
        self.isolated_events = 0
        self.lease_expired = 0
        self.session_drops = 0
        # Failure injection. `blocked` = the radio link is physically gone (frames
        # fail now); `route_blocked`/`route_dead` = batman has noticed (applied
        # reroute_delay_s later). MEASURED in emulation: 1.95 s at OGM 500 ms and
        # 1.94 s at 250 ms (diamond, plink block, 2026-10-03).
        self.blocked: set = set()
        self.route_blocked: set = set()
        self.route_dead: set = set()
        self.snr_off: dict = {}
        # Default: the measured WORST-case recovery for this OGM interval (longer
        # detour); the first scenario runs used the best case (1.95 s) throughout.
        worst = {int(k): v for k, v in cfg.get("measured", {}).get("reroute_worst_s", {}).items()}
        self.reroute_delay_s = float(cfg.get("commsim_des", {}).get(
            "reroute_delay_s", worst.get(int(cfg["batman"]["orig_interval_ms"]), 1.95)))
        self.status_rx = defaultdict(list) if track_gaps else None
        self.sub_hist: list = []  # (t, frozenset of (src, dst) subscribed pairs) per topo tick
        self.unreach: dict = defaultdict(list)  # (src, dst) -> topo-tick times with no route
        self.dup_log: list = []  # (t, task_id, owners, owners_mutually_reachable)
        self.seen_flood: set = set()
        self.conn = [0, 0]
        self.claim_costs: list = []

        rng = random.Random(seed + 1)
        self.nodes = {GW: Robot(self, GW, self.layout.gateway_pos)}
        for rid in range(1, n + 1):
            self.nodes[rid] = Robot(self, rid, self.layout.random_spot(rng))
        if positions:
            for rid, pos in enumerate(positions):
                self.nodes[rid].pos = tuple(pos)
        self.ch = {b: Channel(self, b) for b in self.bands}
        self.st = {(i, b): Station(i, b) for i in self.nodes for b in self.bands}
        self.flows = {(a, b): Flow(self, a, b) for a in self.nodes for b in self.nodes if a != b}
        for (a, b), fl in self.flows.items():
            fl.rev = self.flows[(b, a)]
        # MEASURED (M1): when both routers `connect` to each other (spec: explicit
        # connect to every peer), a pair holds TWO TCP sessions; data uses one, the
        # other idles on keepalives (+ their ACKs). single_connect = lower id dials.
        self.single_connect = single_connect
        self.idle_flows = []
        if not single_connect:
            for a in self.nodes:
                for b in self.nodes:
                    if a < b:
                        f1, f2 = Flow(self, a, b), Flow(self, b, a)
                        f1.rev, f2.rev = f2, f1
                        self.idle_flows += [f1, f2]
        self.tasks: dict = {}
        self.task_seq = 0
        self.snr = {b: {} for b in self.bands}
        self.rate = {b: {} for b in self.bands}
        self.tq = {b: {} for b in self.bands}
        self.next_hop: dict = {}
        self.dup_since: dict = {}
        self.orphan_since: dict = {}

    # ---- event loop ------------------------------------------------------

    def at(self, t, fn, *a):
        self._seq += 1
        heapq.heappush(self._q, (t, self._seq, fn, a))

    def every(self, period, fn, phase=0.0):
        def tick():
            fn()
            self.at(self.now + period, tick)
        self.at(phase, tick)

    def in_window(self, t):
        return self.window[0] <= t < self.window[1]

    # ---- topology / routing ---------------------------------------------

    def topo(self):
        lay, rp = self.layout, self.rp
        ids = [i for i in self.nodes if i not in self.route_dead]
        for b in self.bands:
            snr, rate, tq = {}, {}, {}
            thr_b = R.bcast_threshold(rp.multicast_rate_mbps)
            for ii, i in enumerate(ids):
                for j in ids[ii + 1:]:
                    a, c = self.nodes[i].pos, self.nodes[j].pos
                    if self.fixed_snr is not None:
                        s = self.fixed_snr.get((min(i, j), max(i, j)), -20.0)
                    else:
                        racks, shelves = lay.crossings(a, c)
                        s = R.snr_db(rp, b, math.dist(a, c), racks, shelves, gateway=GW in (i, j))
                    s += self.snr_off.get(frozenset((i, j)), 0.0)
                    snr[(i, j)] = snr[(j, i)] = s
                    r = R.pick_unicast_rate(s, ht=bool(rp.ht), cap=rp.max_rate_mbps)
                    if r:
                        rate[(i, j)] = rate[(j, i)] = r
                    q = (1 - R.per(s, thr_b, 100)) ** 2
                    if q > 0.1:
                        tq[(i, j)] = tq[(j, i)] = q
            self.snr[b], self.rate[b], self.tq[b] = snr, rate, tq
        self._routes(ids)
        self._subscriptions()
        if self.status_rx is not None:
            self.sub_hist.append((self.now, frozenset(
                (p, r.id) for r in self.nodes.values() for p in r.heard_since)))
            for a in self.nodes:
                for b in self.nodes:
                    if a != b and self.hop(a, b) is None:
                        self.unreach[(a, b)].append(self.now)

    def _routes(self, ids):
        if not self.v["mesh"]:
            return
        best = {}
        for b in self.bands:
            for (i, j), q in self.tq[b].items():
                if frozenset((i, j, b)) in self.route_blocked:
                    continue
                if q > best.get((i, j), (0, None))[0]:
                    best[(i, j)] = (q, b)
        adj = defaultdict(list)
        for (i, j), (q, b) in best.items():
            adj[i].append((j, -math.log(q * (1 - HOP_PENALTY))))
        nh = {}
        for dst in ids:  # Dijkstra toward each originator
            dist = {dst: 0.0}
            pq = [(0.0, dst)]
            while pq:
                d, u = heapq.heappop(pq)
                if d > dist.get(u, math.inf):
                    continue
                for v, w in adj[u]:
                    if d + w < dist.get(v, math.inf):
                        dist[v] = d + w
                        nh[(v, dst)] = u
                        heapq.heappush(pq, (d + w, v))
        self.next_hop = {k: (u, best[(k[0], u)][1]) for k, u in nh.items()}

    def hop(self, node, dst, prev_band=None):
        if not self.v["mesh"]:
            # single AP at the gateway: everything relays through it
            nxt = dst if GW in (node, dst) else GW
            b = self.bands[0]
            return (nxt, b) if (node, nxt) in self.rate[b] else None
        h = self.next_hop.get((node, dst))
        if h is None or len(self.bands) < 2 or prev_band is None:
            return h
        # batman-adv interface alternating: prefer the other band on the next
        # hop when it is nearly as good, so a relay can rx and tx at once.
        u, b = h
        other = self.bands[1] if b == self.bands[0] else self.bands[0]
        if b == prev_band and frozenset((node, u, other)) not in self.route_blocked \
                and self.tq[other].get((node, u), 0) >= 0.9 * self.tq[b].get((node, u), 1):
            return (u, other)
        return h

    # ---- failure injection ------------------------------------------------

    def block_link(self, i, j, bands=None):
        for b in bands or self.bands:
            self.blocked.add(frozenset((i, j, b)))
        def notice():
            for b in bands or self.bands:
                if frozenset((i, j, b)) in self.blocked:
                    self.route_blocked.add(frozenset((i, j, b)))
            self.topo()
        self.at(self.now + self.reroute_delay_s, notice)

    def unblock_link(self, i, j, bands=None):
        for b in bands or self.bands:
            self.blocked.discard(frozenset((i, j, b)))
        def notice():  # batman needs a couple of OGMs to trust a returning link
            for b in bands or self.bands:
                if frozenset((i, j, b)) not in self.blocked:
                    self.route_blocked.discard(frozenset((i, j, b)))
            self.topo()
        self.at(self.now + 2 * self.orig_s, notice)

    def kill(self, rid):
        self.nodes[rid].alive = False
        def notice():
            if not self.nodes[rid].alive:
                self.route_dead.add(rid)
                self.topo()
        self.at(self.now + self.reroute_delay_s, notice)

    def revive(self, rid, fresh=False):
        r = self.nodes[rid]
        r.alive = True
        if fresh:  # reboot: empty allocator; tasks re-learned from the fleet
            r.alloc = Allocator(rid, self.params, None if r.is_gw else r.cost,
                                random.Random(self.seed * 7919 + rid + 1), is_gateway=r.is_gw)
            r.stop_task()
        def notice():
            self.route_dead.discard(rid)
            self.topo()
        self.at(self.now + 2 * self.orig_s, notice)

    def transit_load(self):
        """How many (src, dst) routes each node relays and each link carries."""
        relay, links = defaultdict(int), defaultdict(int)
        ids = list(self.nodes)
        for s_ in ids:
            for d in ids:
                if s_ == d:
                    continue
                cur, hops = s_, 0
                while cur != d and hops < 20:
                    h = self.next_hop.get((cur, d))
                    if h is None:
                        break
                    links[frozenset((cur, h[0]))] += 1
                    if h[0] != d:
                        relay[h[0]] += 1
                    cur, hops = h[0], hops + 1
        return relay, links

    def path_ok(self, a, b):
        """Does the current route a -> b physically carry frames right now?"""
        cur, hops = a, 0
        while cur != b:
            h = self.hop(cur, b)
            if h is None or hops > 20:
                return False
            nxt, band = h
            if not self.nodes[nxt].alive or frozenset((cur, nxt, band)) in self.blocked \
                    or (cur, nxt) not in self.rate[band]:
                return False
            cur, hops = nxt, hops + 1
        return True

    def link_ok(self, i, j, band, size):
        if not self.nodes[j].alive or frozenset((i, j, band)) in self.blocked:
            return False
        rt = self.rate[band].get((i, j))
        if rt is None:
            return False
        return self.rng.random() >= R.per(self.snr[band][(i, j)], rt[1], size)

    def net_send(self, node, f: Frame):
        if not self.nodes[node].alive:
            return
        h = self.hop(node, f.final, f.prev_band)
        if h is None:
            self.m["no_route"] += 1
            return
        f.src, (f.nh, f.band) = node, h
        self.ch[f.band].enqueue(self.st[(node, f.band)], f)

    def on_frame_rx(self, node, f: Frame):
        f.hops += 1
        if f.seg is not None:
            f.seg.hops += 1
        if node == f.final:
            if f.kind == "udp":
                d = STACK_BASE_S + self.rng.expovariate(1 / STACK_JITTER_S)
                self.at(self.now + d, self.deliver, node, f.msg, f.hops)
            elif f.kind == "seg":
                f.seg.flow.on_seg(f.seg)
            elif f.kind == "ack":
                fl, ack, epoch = f.ack
                fl.on_ack(ack, epoch)
            return
        self.net_send(node, f)

    def on_bcast_air(self, src, f: Frame, band):
        if f.kind == "ogm":
            return
        thr = R.bcast_threshold(self.rp.multicast_rate_mbps)
        for r in self.nodes.values():
            if r.id == src or not r.alive:
                continue
            s = self.snr[band].get((src, r.id))
            if s is None or self.rng.random() < R.per(s, thr, f.size):
                continue
            if f.kind == "bstatus":
                self.deliver(r.id, f.msg, 1, via_bcast=True)
            elif f.kind == "flood":
                key = (f.fid, r.id)
                if key in self.seen_flood:
                    continue
                self.seen_flood.add(key)
                self.deliver(r.id, f.msg, 1)
                # batman-adv floods: each node rebroadcasts once on each band
                for b in self.bands:
                    g = Frame("flood", f.size, r.id, msg=f.msg, fid=f.fid)
                    self.ch[b].enqueue(self.st[(r.id, b)], g)

    # ---- publishing ------------------------------------------------------

    def cell_of(self, pos):
        return (int(pos[0] // self.cell), int(pos[1] // self.cell))

    def _subscriptions(self):
        """Each robot subscribes to its own cell, the 8 neighbors and the
        cells on its route — for map regions in every variant, and for Status
        too when neighbor-only (V3+). Peers leaving that set are forgotten."""
        for r in self.nodes.values():
            if r.is_gw or not r.alive:
                continue
            cx, cy = self.cell_of(r.pos)
            cells = {(cx + dx, cy + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)}
            for p in r.route:
                cells.add(self.cell_of(p))
            r.subs_cells = cells
        for r in self.nodes.values():
            if r.is_gw:
                continue
            for p in self.nodes.values():
                if p.id in (r.id, GW):
                    continue
                if self.cell_of(p.pos) in r.subs_cells:
                    if p.id not in r.heard_since:
                        r.heard_since[p.id] = self.now
                        r.map_seq_at_sub[p.id] = p.map_seq
                elif p.id in r.heard_since:
                    del r.heard_since[p.id]
                    if self.v["neighbor"]:
                        r.alloc.forget(p.id)

    def status_subscribers(self, pub):
        if not self.v["neighbor"]:
            return [i for i in self.nodes if i not in (pub.id, GW)]
        return self.region_subscribers(pub)

    def region_subscribers(self, pub):
        c = self.cell_of(pub.pos)
        return [r.id for r in self.nodes.values()
                if r.id not in (pub.id, GW) and r.alive and c in r.subs_cells]

    def all_robots(self, exclude):
        return [i for i in self.nodes if i != exclude]

    def publish(self, pub_id, typ, size, dsts, data=None, reliable=None):
        msg = {"type": typ, "src": pub_id, "t0": self.now, "data": data,
               "reliable": typ in RELIABLE if reliable is None else reliable,
               "wire": R.CDR_ENCAP + size + R.ZENOH_PER_MSG}
        udp = self.status_transport == "udp" and typ in STATUS_TYPES
        for d in dsts:
            if d == pub_id:
                continue
            if typ in ("Status", "Pose") and self.in_window(self.now):
                self.st_sent[(pub_id, d)] += 1
            if udp:
                # PROTOTYPE: best-effort UDP unicast over bat0 — batman still
                # routes it multi-hop, but no TCP ACK, retransmit or backoff.
                f = Frame("udp", R.MAC_OVERHEAD + R.BATMAN_UNICAST + R.IP_UDP
                          + R.UDP_STATUS_OVERHEAD + R.CDR_ENCAP + size, pub_id, final=d, msg=msg)
                self.net_send(pub_id, f)
            else:
                self.flows[(pub_id, d)].send(msg)

    def deliver(self, dst, msg, hops, via_bcast=False):
        r = self.nodes[dst]
        if not r.alive:
            return
        typ, src, now = msg["type"], msg["src"], self.now
        if self.in_window(msg["t0"]):
            if typ in CONTROL:
                self.lat[typ].append(now - msg["t0"])
                self.hops.append(hops)
            if typ in ("Status", "Pose") and (not via_bcast or src in r.heard_since or not self.v["neighbor"]):
                self.st_recv[(src, dst)] += 1
        if self.status_rx is not None and typ in ("Status", "Pose", "StatusLite"):
            self.status_rx[(src, dst)].append(now)
        a, d = r.alloc, msg["data"]
        if typ in ("Status", "StatusLite", "Pose"):
            r.last_pose_rx[src] = now
            if typ != "Pose":
                if self.v["neighbor"] and not r.is_gw and src not in r.heard_since:
                    # Overheard (V5 broadcast) or a straggler that was in flight
                    # when we unsubscribed: use its leases, but don't restart
                    # silence-tracking for a peer we no longer follow.
                    a.on_lease_renewal(src, d["owned"], now)
                else:
                    a.on_status(src, d["owned"], now)
        elif typ == "LeaseBeacon":
            a.on_lease_renewal(src, d["owned"], now)
        elif typ == "TaskAnnounce":
            a.on_announce(src, d["task_id"], d["pickup"], d["dropoff"], now)
        elif typ == "Bid":
            a.on_bid(src, d["task_id"], d["cost"], now)
        elif typ == "Claim":
            a.on_claim(src, d["task_id"], d["cost"], now)
            t = self.tasks.get(d["task_id"])
            if r.is_gw and t and t.get("claimed_at") is None:
                t["claimed_at"] = now
                if self.in_window(t["t"]) and t["eligible"] and not t.get("rebid"):
                    self.auction.append(now - t["t"])
        elif typ == "TaskEvent":
            a.on_task_event(src, d["task_id"], d["kind"], now)
        elif typ == "MapUpdate":
            r.last_map_rx[src] = max(r.last_map_rx.get(src, 0), d["seq"])
        self.flush(r)

    def flush(self, r: Robot):
        out, ev = r.alloc.drain()
        for m in out:
            t = m["type"]
            if t == "TaskAnnounce":
                self.publish(r.id, t, 37, self.all_robots(r.id),
                             {k: m[k] for k in ("task_id", "pickup", "dropoff")})
            elif t == "Bid":
                self.publish(r.id, t, 24, self.bid_dsts(r, m["task_id"]), m)
            elif t == "Claim":
                self.publish(r.id, t, 28, self.all_robots(r.id), m)
            elif t == "LeaseBeacon":  # fleet-wide, from core.protocol (finding 1 fix)
                self.publish(r.id, t, 20 + 8 * len(m["owned"]), self.all_robots(r.id), m)
            elif t == "TaskEvent":
                self.publish(r.id, t, 18, self.all_robots(r.id), m)
        for e in ev:
            self.on_event(r, e)

    def bid_dsts(self, r, tid):
        if not self.v["neighbor"]:
            return self.all_robots(r.id)
        t = r.alloc.tasks[tid]
        rad = r.alloc._radius(t.round)
        return [GW] + [p.id for p in self.nodes.values() if p.id not in (r.id, GW)
                       and math.dist(p.pos, t.pickup) <= rad]

    def on_event(self, r, e):
        k = e["event"]
        w = self.in_window(self.now)
        if k == "no_winner" and r.is_gw and e["task_id"] in self.tasks:
            # nobody could take it this round (all near robots busy): the wait
            # that follows is fleet capacity, not auction/comms latency
            self.tasks[e["task_id"]]["rebid"] = True
        if k == "claimed":
            r.start_task(e["task_id"])
            if w:
                self.claim_costs.append(e["cost"])
        elif k in ("stop_task", "task_yielded"):
            if r.task == e["task_id"]:
                r.stop_task()
        elif k == "robot_silent" and w:
            peer = self.nodes[e["peer"]]
            if peer.alive:
                self.false_silent += 1
        elif k == "isolated" and w:
            self.isolated_events += 1
        elif k == "lease_expired" and w:
            self.lease_expired += 1

    # ---- periodic processes ---------------------------------------------

    def status_tick(self, r: Robot):
        if not r.alive or (r.is_gw and not self.status_only):
            return
        if self.status_only:  # M1 mirror: one owned task, all-to-all incl. gateway
            msg_size = 52 + 8
            if self.in_window(self.now):
                pass
            self.publish(r.id, "Status", msg_size, self.all_robots(r.id), {"owned": []},
                         reliable=False)
            return
        r.status_n += 1
        owned = r.alloc.owned()
        if self.v["split"]:
            # V1 naive: pose 3 Hz, path 3 Hz, status 1 Hz, obstacle list 1 Hz — all
            # to every robot and the gateway. Rates PLACEHOLDER.
            everyone = self.all_robots(r.id)
            self.publish(r.id, "Pose", 38, everyone, reliable=False)
            self.publish(r.id, "PathMsg", 14 + 4 + 8 * 10, everyone, reliable=False)
            if r.status_n % 3 == 0:
                self.publish(r.id, "StatusLite", 14 + 5 + 8 * len(owned) + 4, everyone,
                             {"owned": owned}, reliable=False)
                self.publish(r.id, "ObstacleList", 14 + 4 + 24 * 3, everyone, reliable=False)
            return
        path = r.replanned
        r.replanned = False
        # MEASURED CDR: 64 B with one owned task and no path, 144 B with 10 waypoints
        size = 52 + 8 * len(owned) + (80 if path else 0) + 28 * len(r.new_obstacles)
        r.new_obstacles = []
        data = {"owned": owned}
        if self.v.get("bcast_status"):
            # V5: one radio transmission per band reaches every neighbor in range
            b = self.bands[r.status_n % len(self.bands)]
            msg = {"type": "Status", "src": r.id, "t0": self.now, "data": data,
                   "reliable": False, "wire": 0}
            if self.in_window(self.now):
                for d in self.status_subscribers(r):
                    self.st_sent[(r.id, d)] += 1
            f = Frame("bstatus", R.MAC_OVERHEAD + R.IP_UDP + 4 + size, r.id, msg=msg)
            self.ch[b].enqueue(self.st[(r.id, b)], f)
        else:
            # V2: every robot + the gateway at the full rate. V3+: neighbor cells.
            dsts = self.status_subscribers(r) + ([] if self.v["neighbor"] else [GW])
            self.publish(r.id, "Status", size, dsts, data, reliable=False)
        if self.v["neighbor"] and r.status_n % 3 == 0:
            self.publish(r.id, "Status", size, [GW], data, reliable=False)  # gateway at 1 Hz

    def ogm_tick(self, node):
        if not self.nodes[node].alive:
            return
        nb = R.OGM_BYTES * len(self.nodes)
        for b in self.bands:
            while nb > 0:
                chunk = min(nb, R.OGM_AGGREGATE_MAX)
                self.ch[b].enqueue(self.st[(node, b)], Frame("ogm", R.MAC_OVERHEAD + chunk, node))
                nb -= chunk
            nb = R.OGM_BYTES * len(self.nodes)

    def keepalive_tick(self):
        now = self.now
        sessions = [self.flows[(a, b)] for a in self.nodes for b in self.nodes if a < b] \
            + self.idle_flows[0::2]
        for f in sessions:  # f and f.rev are the two directions of one TCP session
            a, b = self.nodes[f.src], self.nodes[f.dst]
            if f.down:
                if now >= f.retry_at:
                    # a re-dial is a TCP handshake: it needs a path that actually
                    # carries frames now, not just one batman still believes in
                    if a.alive and b.alive and self.path_ok(f.src, f.dst) and self.path_ok(f.dst, f.src):
                        f.reopen()
                    else:
                        f.retry_at = now + Flow.RECONNECT_S
                continue
            stale = max(now - max(f.last_rx, f.opened_at), now - max(f.rev.last_rx, f.rev.opened_at))
            if stale > self.lease_s and now > 5:
                f.close()
                if self.in_window(now) and f in self.flows.values():
                    self.session_drops += 1
                continue
            for fl in (f, f.rev):
                if self.nodes[fl.src].alive and now - fl.last_tx >= self.ka_s - 1e-9:
                    fl.send({"type": "KA", "src": fl.src, "t0": now, "data": None,
                             "reliable": False, "wire": 4})
        for r in self.nodes.values():
            live, last = False, 0.0
            for p in self.nodes:
                if p == r.id:
                    continue
                fl = self.flows[(p, r.id)]
                if fl.down:
                    continue
                last = max(last, fl.last_rx)
                if now - fl.last_rx <= self.lease_s:
                    live = True
            if live and self.transport_liveness:
                r.alloc.heard_network(last)

    def task_arrival(self):
        self.task_seq += 1
        gw = self.nodes[GW]
        pickup, dropoff = self.layout.random_spot(self.rng), self.layout.random_spot(self.rng)
        # Auction time is a comms metric only when someone could bid at once:
        # a task waiting for a robot to free up is a fleet-capacity wait.
        rad = self.cfg["protocol"]["bid_radius_m"]
        eligible = any(r.task is None and r.alive and math.dist(r.pos, pickup) <= rad
                       for r in self.nodes.values() if not r.is_gw)
        self.tasks[self.task_seq] = {"t": self.now, "done": False, "claimed_at": None,
                                     "eligible": eligible}
        gw.alloc.announce(self.task_seq, pickup, dropoff, self.now)
        self.flush(gw)
        lam = self.task_rate * self.n
        self.at(self.now + self.rng.expovariate(lam), self.task_arrival)

    def task_arrival_once(self):
        """One extra task now (burst scenario), outside the Poisson stream."""
        self.task_seq += 1
        gw = self.nodes[GW]
        pickup, dropoff = self.layout.random_spot(self.rng), self.layout.random_spot(self.rng)
        rad = self.cfg["protocol"]["bid_radius_m"]
        eligible = any(r.task is None and r.alive and math.dist(r.pos, pickup) <= rad
                       for r in self.nodes.values() if not r.is_gw)
        self.tasks[self.task_seq] = {"t": self.now, "done": False, "claimed_at": None,
                                     "eligible": eligible}
        gw.alloc.announce(self.task_seq, pickup, dropoff, self.now)
        self.flush(gw)

    def task_done(self, tid):
        if tid in self.tasks:
            self.tasks[tid]["done"] = True

    def obstacle_arrival(self, r):
        self._publish_obstacle(r)
        self.at(self.now + self.rng.expovariate(self.obstacle_rate), self.obstacle_arrival, r)

    def obstacle_burst(self, rate_hz: float, duration_s: float):
        """Extra Poisson obstacle arrivals per robot over [now, now+duration).
        Raising obstacle_rate alone did not do this: each robot's next arrival
        was already scheduled at the base rate (mean 50 s), so a 10 s "10x"
        burst added about one obstacle in total."""
        for r in self.nodes.values():
            if r.is_gw:
                continue
            t = self.now + self.rng.expovariate(rate_hz)
            while t < self.now + duration_s:
                self.at(t, self._publish_obstacle, r)
                t += self.rng.expovariate(rate_hz)

    def _publish_obstacle(self, r):
        if r.alive:
            if self.v["split"]:
                pass  # V1 sends a periodic ObstacleList instead
            elif self.v.get("flood_obstacles"):
                self._seq += 1
                msg = {"type": "ObstacleEvent", "src": r.id, "t0": self.now, "data": None,
                       "reliable": True, "wire": 0}
                fid = ("obs", self._seq)
                self.seen_flood.add((fid, r.id))
                for b in self.bands:
                    self.ch[b].enqueue(self.st[(r.id, b)],
                                       Frame("flood", R.MAC_OVERHEAD + R.BATMAN_BCAST + 32 + 8, r.id,
                                             msg=msg, fid=fid))
            else:
                self.publish(r.id, "ObstacleEvent", 48, self.all_robots(r.id))
            r.new_obstacles.append(1)

    def map_tick(self, r):
        if r.alive and not r.is_gw:
            r.map_seq += 1
            # Map regions are subscription-scoped in every design (spec: "Region
            # subscribers"); V1/V2 differ from V3 in status, not maps.
            dsts = self.region_subscribers(r)
            self.publish(r.id, "MapUpdate", self.map_patch_bytes, dsts, {"seq": r.map_seq},
                         reliable=False)
        lo, hi = self.cfg["map"]["update_period_s"]
        self.at(self.now + self.rng.uniform(lo, hi), self.map_tick, r)

    def control_tick(self):
        dt = 0.1
        for r in self.nodes.values():
            if not r.alive:
                continue
            if not r.is_gw and not self.status_only:
                r.move(dt)
            r.alloc.tick(self.now)
            self.flush(r)

    def sample_tick(self):
        """1 Hz: ownership invariants, map lag, planning-view staleness."""
        if not self.in_window(self.now):
            return
        n_idle = sum(1 for r in self.nodes.values() if not r.is_gw and r.alive and r.task is None)
        # "Abandoned" needs an idle robot that was actually free for it: in an
        # oversubscribed fleet tasks queue for robots — capacity, not a fault.
        idle = n_idle > 0
        for tid, t in self.tasks.items():
            if t["done"]:
                continue
            owners = [r.id for r in self.nodes.values() if r.alive and
                      (v := r.alloc.tasks.get(tid)) and v.state == CLAIMED and v.owner == r.id]
            if len(owners) > 1:
                self.dup_log.append((self.now, tid, tuple(owners), all(
                    self.hop(a, b) is not None for a in owners for b in owners if a != b)))
                s = self.dup_since.setdefault(tid, self.now)
                if self.now - s >= 1.0 and s >= 0:
                    self.dup_episodes += 1
                    self.dup_since[tid] = -1e9  # count each episode once
            else:
                if tid in self.dup_since and self.dup_since[tid] > 0:
                    self.dup_transient += 1
                self.dup_since.pop(tid, None)
            if not owners and idle and self.now - t["t"] > 5 and self._unowned_count() <= n_idle:
                s = self.orphan_since.setdefault(tid, self.now)
                if s >= 0 and self.now - s > 30.0:
                    self.abandoned += 1
                    self.orphan_since[tid] = -1e9
            elif owners:
                self.orphan_since.pop(tid, None)
        ids = [i for i, r in self.nodes.items() if r.alive]
        for i in ids:
            for j in ids:
                if i != j:
                    self.conn[1] += 1
                    self.conn[0] += self.hop(i, j) is not None
        thr = self.cfg["protocol"]["staleness_threshold_s"]
        for r in self.nodes.values():
            if r.is_gw or not r.alive:
                continue
            peers = r.heard_since if self.v["neighbor"] else \
                {p: 0 for p in self.nodes if p not in (r.id, GW)}
            for p, since in peers.items():
                if self.now - since < 2.0:
                    continue
                age = self.now - r.last_pose_rx.get(p, -math.inf)
                self.stale[1] += 1
                if age > thr:
                    self.stale[0] += 1
            for p, since in r.heard_since.items():
                if self.now - since < 2.0:
                    continue
                base = r.map_seq_at_sub.get(p, 0)
                self.map_lags.append(self.nodes[p].map_seq - max(r.last_map_rx.get(p, 0), base))

    def _unowned_count(self):
        n = 0
        for tid, t in self.tasks.items():
            if t["done"]:
                continue
            if not any(r.alive and (v := r.alloc.tasks.get(tid)) and v.state == CLAIMED
                       and v.owner == r.id for r in self.nodes.values()):
                n += 1
        return n

    def background(self, band):
        st = self.st.setdefault((-1, band), Station(-1, band))
        self.ch[band].enqueue(st, Frame("bg", 0, -1))
        lam = self.bg_share / BG_AIRTIME_S
        self.at(self.now + self.rng.expovariate(lam), self.background, band)

    # ---- run ---------------------------------------------------------------

    def run(self) -> dict:
        t_wall = time.time()
        rng = self.rng
        self.topo()
        self.every(TOPO_DT, self.topo, TOPO_DT)
        self.every(0.1, self.control_tick, 0.05)
        self.every(self.ka_s, self.keepalive_tick, 0.25)
        self.every(1.0, self.sample_tick, 0.5)
        for r in self.nodes.values():
            if self.v["mesh"]:
                self.every(self.orig_s, lambda i=r.id: self.ogm_tick(i), rng.uniform(0, self.orig_s))
            if self.status_only:
                self.every(1 / self.status_hz, lambda r=r: self.status_tick(r),
                           rng.uniform(0, 1 / self.status_hz))
                continue
            if r.is_gw:
                continue
            self.every(1 / self.status_hz, lambda r=r: self.status_tick(r),
                       rng.uniform(0, 1 / self.status_hz))
            self.every(1.0, lambda r=r: self.publish(r.id, "LinkHealth", 50, [GW], reliable=False),
                       rng.uniform(0, 1))
            self.at(rng.uniform(0, 5), self.map_tick, r)
            self.at(rng.expovariate(self.obstacle_rate), self.obstacle_arrival, r)
        if not self.v["mesh"]:
            # AP beacons, 10 Hz, ~150 B at the lowest OFDM rate's airtime class
            def beacon():
                self.ch[self.bands[0]].enqueue(self.st[(GW, self.bands[0])],
                                               Frame("ogm", 150, GW))
            self.every(0.1024, beacon, 0.01)
        if not self.status_only:
            self.at(rng.uniform(0, 2), self.task_arrival)
        if self.bg_share > 0:
            for b in self.bands:
                self.at(0.0, self.background, b)

        events = 0
        while self._q:
            t, _, fn, a = heapq.heappop(self._q)
            if t > self.end:
                break
            self.now = t
            fn(*a)
            events += 1
        return self.results(events, time.time() - t_wall)

    def results(self, events, wall) -> dict:
        span = self.window[1] - self.window[0]
        ctl = [x for k in CONTROL for x in self.lat.get(k, [])]
        pairs = [(self.st_recv[k] / v) for k, v in self.st_sent.items() if v >= 30]
        sent = sum(self.st_sent.values())
        res = {
            "variant": self.variant, "n": self.n, "seed": self.seed, "bg_share": self.bg_share,
            "lease_beacon_hz": self.lease_beacon_hz, "status_transport": self.status_transport,
            "single_connect": self.single_connect,
            "status_timeout_s": self.params.status_timeout_s, "zenoh_lease_s": self.lease_s,
            # effective settings, so a row says what it ran with — scripts once
            # inherited base.yaml defaults that later changed (code review 2026-10-04)
            "orig_interval_ms": int(self.cfg["batman"]["orig_interval_ms"]),
            "max_open_bids": self.params.max_open_bids,
            "reroute_delay_s": self.reroute_delay_s,
            "events": events, "wall_s": round(wall, 1),
            "ctl_samples": len(ctl),
            "ctl_p50_ms": pct(ctl, 50) * 1e3, "ctl_p95_ms": pct(ctl, 95) * 1e3,
            "ctl_p99_ms": pct(ctl, 99) * 1e3,
            "status_delivery": sum(self.st_recv.values()) / sent if sent else math.nan,
            "status_pair_min": min(pairs) if pairs else math.nan,
            "false_silent": self.false_silent,
            "dup_episodes": self.dup_episodes, "dup_transient": self.dup_transient,
            "abandoned": self.abandoned,
            "auction_n": len(self.auction),
            "auction_p95_s": pct(self.auction, 95), "auction_max_s": max(self.auction, default=math.nan),
            "map_lag_p95": pct(self.map_lags, 95),
            "stale_frac": self.stale[0] / self.stale[1] if self.stale[1] else math.nan,
            "mean_hops": sum(self.hops) / len(self.hops) if self.hops else math.nan,
            "isolated_events": self.isolated_events, "lease_expired": self.lease_expired,
            "session_drops": self.session_drops,
            "conn_frac": self.conn[0] / self.conn[1] if self.conn[1] else math.nan,
            "claim_cost_mean_s": (sum(self.claim_costs) / len(self.claim_costs)) if self.claim_costs else math.nan,
            "claims": len(self.claim_costs),
            "tasks_done": sum(1 for t in self.tasks.values() if t["done"]),
        }
        for b in ("g24", "g5"):
            res[f"busy_{b}"] = self.ch[b].busy_acc / span if b in self.ch else math.nan
        for k in ("mac_drops", "q_drops", "be_drops", "no_route", "tcp_retx", "session_down_drops"):
            res[k] = self.m[k]
        for k in sorted(self.lat):
            res[f"p95_{k}_ms"] = pct(self.lat[k], 95) * 1e3
        tot = sum(self.air_by.values()) or 1.0
        res["airtime_share"] = {k: round(v / tot, 3) for k, v in
                                sorted(self.air_by.items(), key=lambda kv: -kv[1])}
        res["failed"] = failures(res)
        return res


# Proposed thresholds (plan §6). Order = reporting order only.
def failures(r: dict) -> list:
    f = []
    busy = max(x for x in (r["busy_g24"], r["busy_g5"]) if x == x)
    if busy > 0.50:
        f.append("busy>50%")
    if r["ctl_p95_ms"] > 150 or r["ctl_p99_ms"] > 500 or r["ctl_samples"] == 0:
        f.append("latency")
    pair_min = r["status_pair_min"]
    if not r["status_delivery"] >= 0.95 or (pair_min == pair_min and pair_min < 0.85):
        f.append("status_delivery")
    if r["false_silent"] > 0:
        f.append("false_silent")
    if r["dup_episodes"] > 0 or r["abandoned"] > 0:
        f.append("ownership")
    if r["auction_n"] and (r["auction_p95_s"] > 1.5 or r["auction_max_s"] > 3.0):
        f.append("auction")
    if r["map_lag_p95"] > 2:
        f.append("map_divergence")
    return f
