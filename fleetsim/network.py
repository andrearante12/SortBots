"""Message delivery between fleet nodes: who hears what, how late.

Two models behind one interface (`Network.send`), picked by
`network.model`:

  ideal   every live, unpartitioned pair is connected; fixed delay, optional
          random loss on best-effort traffic. For protocol logic, or as a
          baseline to subtract the radio from a result.
  radio   the commsim DES's radio arithmetic (commsim/des/radio.py: path
          loss, rack/shelf attenuation, 802.11 rate choice, frame error
          curve, frame airtime — all its PLACEHOLDER values included),
          batman-style best-quality multi-hop routes, and channel load from
          the airtime the fleet actually sends. Walls come from the TRUE
          map via commsim/des/layout.py from_occupancy, the same extraction
          the operator view's Comms overlay uses.

How the radio model is cheaper than the DES (which simulates every frame's
CSMA/CA contention, TCP windows and zenoh sessions) — and what that costs:

  * Per topology epoch (`topo_s`) it computes, for every node pair, the route
    and its aggregates: hop count, probability a frame survives every hop
    (each hop: 1 - PER^(retries+1)), and airtime per frame and per byte.
    A message then costs a few numpy ops for ALL its recipients at once.
  * Contention is a load factor, not a simulation: airtime offered in the
    last epoch / epoch length, either one domain for the whole fleet (the
    DES's own simplification, which it calibrated against the emulation) or
    per node over its carrier-sense neighbours (`contention: spatial`, for
    floors much larger than one radio's reach — not calibrated yet). Delay
    per hop grows as 1/(1 - load); past 95% load, best-effort traffic loses
    the excess and reliable traffic queues.
  * Reliable traffic (zenoh over TCP) retransmits after an RTO that doubles,
    and is dropped once retries outlast the transport lease, as a dead zenoh
    session would drop it. Delivery is in order per (sender, receiver).
  * Routing reacts to robots MOVING immediately (each epoch). To an injected
    failure (link cut, robot killed) it reacts after `reroute_delay_s`, the
    measured batman reroute, and frames sent over the dead link meanwhile
    are lost or, if reliable, retried.
  * One band. batman OGM airtime is charged (it is what grows as n^2);
    zenoh keepalives on idle sessions are not.

Every copy is unicast: zenoh routes one copy per subscriber, and Status
over UDP is sent to each recipient — so fleet-wide traffic costs n copies,
which is what the DES found breaks first at scale.
"""
from __future__ import annotations

import math
from collections import defaultdict

import numpy as np

from commsim.des import layout as L
from commsim.des import radio as R

# commsim/des/sim.py, measured in emulation (M1, 2026-10-03)
STACK_BASE_S, STACK_JITTER_S = 2.4e-3, 0.7e-3
PROC_S = 0.75e-3
HOP_PENALTY = 15 / 255
LOAD_CAP = 0.95
# Per-frame bytes UDP Status saves over a zenoh/TCP message (radio.py):
# IP_TCP + ZENOH_PER_MSG vs IP_UDP + UDP_STATUS_OVERHEAD.
UDP_SAVING_B = (R.IP_TCP + R.ZENOH_PER_MSG) - (R.IP_UDP + R.UDP_STATUS_OVERHEAD)


class Network:
    """Shared delivery maths. Subclasses fill the per-epoch matrices in
    `_compute()`: reach (routing has a path), p (a frame survives the path),
    hops, air_f / air_b (path airtime per frame / per byte)."""

    def __init__(self, n_nodes: int, cfg: dict, rng: np.random.Generator):
        self.n = n_nodes
        self.cfg = cfg
        self.rng = rng
        self.pos = np.zeros((n_nodes, 2))
        self.alive_phys = np.ones(n_nodes, dtype=bool)
        self.alive_route = np.ones(n_nodes, dtype=bool)
        self.blocked_phys: set[frozenset] = set()
        self.blocked_route: set[frozenset] = set()
        self._notices: list[tuple[float, str, object, bool]] = []
        self.last_rel = np.zeros((n_nodes, n_nodes))
        self.load = np.zeros(n_nodes)        # per-node load factor this epoch
        self.backlog = np.zeros(n_nodes)     # s of reliable traffic queued past LOAD_CAP
        self.air = np.zeros(n_nodes)         # airtime offered this epoch (s)
        self.next_topo = -math.inf
        self._last_topo: float | None = None
        self.topo_s = 1.0
        self.reroute_delay_s = 0.0
        self.restore_delay_s = 0.0
        self.rto_s = float(cfg.get("rto_s", 0.2))
        self.lease_s = float(cfg.get("transport_lease_s", 4.0))
        self.stats = defaultdict(lambda: {"sent": 0, "delivered": 0, "lost": 0,
                                          "bytes": 0, "lat": []})
        n = n_nodes
        self.reach = np.ones((n, n), dtype=bool)
        self.p = np.ones((n, n))
        self.hops = np.ones((n, n))
        self.air_f = np.zeros((n, n))
        self.air_b = np.zeros((n, n))
        self.lat0 = np.zeros((n, n))

    # ---- failures ---------------------------------------------------------

    def block(self, i: int, j: int, now: float) -> None:
        self.blocked_phys.add(frozenset((i, j)))
        self._notices.append((now + self.reroute_delay_s, "block", frozenset((i, j)), True))
        self.next_topo = -math.inf

    def unblock(self, i: int, j: int, now: float) -> None:
        self.blocked_phys.discard(frozenset((i, j)))
        self._notices.append((now + self.restore_delay_s, "block", frozenset((i, j)), False))
        self.next_topo = -math.inf

    def kill(self, i: int, now: float) -> None:
        self.alive_phys[i] = False
        self._notices.append((now + self.reroute_delay_s, "alive", i, False))
        self.next_topo = -math.inf

    def revive(self, i: int, now: float) -> None:
        self.alive_phys[i] = True
        self._notices.append((now + self.restore_delay_s, "alive", i, True))
        self.next_topo = -math.inf

    def pending_notice(self, now: float) -> float | None:
        due = [t for t, *_ in self._notices if t > now]
        return min(due) if due else None

    def _apply_notices(self, now: float) -> bool:
        changed, keep = False, []
        for t, kind, key, on in self._notices:
            if t > now:
                keep.append((t, kind, key, on))
                continue
            changed = True
            if kind == "block":
                if on and key in self.blocked_phys:
                    self.blocked_route.add(key)
                elif not on and key not in self.blocked_phys:
                    self.blocked_route.discard(key)
            else:
                self.alive_route[key] = on and bool(self.alive_phys[key])
        self._notices = keep
        return changed

    # ---- epochs -----------------------------------------------------------

    def update(self, pos: np.ndarray, now: float) -> None:
        """Positions change every step; the topology every topo_s or when
        a failure notice falls due."""
        self.pos = pos
        noticed = self._apply_notices(now)
        if now < self.next_topo and not noticed:
            return
        epoch = 0.0 if self._last_topo is None else now - self._last_topo
        self._last_topo = now
        self.next_topo = now + self.topo_s
        self._load_from_air(epoch)
        self._compute()

    def _load_from_air(self, epoch: float) -> None:
        self.load[:] = 0.0
        self.air[:] = 0.0

    def _compute(self) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def _phys_mask(self) -> np.ndarray:
        n = self.n
        m = np.outer(self.alive_phys, self.alive_phys)
        for key in self.blocked_phys:
            i, j = tuple(key)
            m[i, j] = m[j, i] = False
        np.fill_diagonal(m, True)
        return m

    def _route_mask(self) -> np.ndarray:
        m = np.outer(self.alive_route, self.alive_route)
        for key in self.blocked_route:
            i, j = tuple(key)
            m[i, j] = m[j, i] = False
        np.fill_diagonal(m, True)
        return m

    # ---- delivery ---------------------------------------------------------

    def send(self, kind: str, src: int, dsts: np.ndarray, size: int, reliable: bool,
             now: float, deadline: float | None = None, udp: bool = False):
        """One message from src to each of dsts.

        -> (delivered dsts, arrival times, retry dsts, retry time). Retry is
        only ever non-empty for reliable traffic whose path is cut by a
        failure routing hasn't noticed yet: the caller re-sends to those at
        the retry time (TCP retransmitting into the reroute), until
        `deadline` (the transport lease from the first send)."""
        st = self.stats[kind]
        dsts = np.asarray(dsts, dtype=np.int64)
        dsts = dsts[dsts != src]
        empty = (dsts[:0], np.zeros(0), dsts[:0], None)
        if not len(dsts):
            return empty
        first_try = deadline is None
        if deadline is None:
            deadline = now + self.lease_s
        if first_try:
            st["sent"] += len(dsts)
            st["bytes"] += size * len(dsts)
        if not self.alive_phys[src]:
            st["lost"] += len(dsts)
            return empty
        reach = self.reach[src, dsts] & self.alive_phys[dsts]
        p = np.where(reach, self.p[src, dsts], 0.0)
        frames = max(1, math.ceil(size / R.TCP_MSS))
        # air_f is priced with zenoh-over-TCP framing; Status over UDP
        # (commsim's adopted transport) carries less per frame.
        wire = size - (UDP_SAVING_B * frames if udp else 0)
        air = self.air_f[src, dsts] * frames + self.air_b[src, dsts] * wire
        rho = np.maximum(self.load[src], self.load[dsts])
        slow = 1.0 / (1.0 - np.minimum(rho, LOAD_CAP))
        lat = (STACK_BASE_S + self.rng.exponential(STACK_JITTER_S, len(dsts))
               + self.lat0[src, dsts] + air * slow)
        retry_dsts, retry_t = dsts[:0], None
        if reliable:
            ok = p > 0
            fails = np.zeros(len(dsts))
            if ok.any():
                fails[ok] = self.rng.geometric(np.clip(p[ok], 1e-9, 1.0)) - 1
            # RTO doubling: k failures cost rto * (2^k - 1) before the success.
            wait = self.rto_s * (np.power(2.0, fails) - 1.0)
            lat = lat + wait + np.maximum(self.backlog[src], self.backlog[dsts])
            ok &= now + lat <= deadline
            self.air[src] += float((air * (fails + 1.0))[reach].sum())
            # Cut by a failure routing will notice before the lease runs out:
            # TCP keeps retransmitting and gets through on the new route.
            due = self.pending_notice(now)
            cut = reach & (p <= 0)
            if due is not None and due < deadline and cut.any():
                retry_dsts, retry_t = dsts[cut], due + self.rto_s
            lost_n = int((~ok).sum()) - len(retry_dsts)
        else:
            over = np.where(rho > LOAD_CAP, 1.0 - LOAD_CAP / np.maximum(rho, 1e-9), 0.0)
            ok = self.rng.random(len(dsts)) < p * (1.0 - over)
            self.air[src] += float(air[reach].sum())
            lost_n = int((~ok).sum())
        st["lost"] += max(0, lost_n)
        d, arr = dsts[ok], now + lat[ok]
        if reliable and len(d):
            arr = np.maximum(arr, self.last_rel[src, d] + 1e-6)  # in order per pair
            self.last_rel[src, d] = arr
        st["delivered"] += len(d)
        st["lat"].append(arr - now)
        return d, arr, retry_dsts, retry_t

    def take_stats(self) -> dict:
        """Per-kind counters and latency percentiles since the last call."""
        out = {}
        for kind, st in self.stats.items():
            lat = np.concatenate(st["lat"]) if st["lat"] else np.zeros(0)
            out[kind] = {"sent": st["sent"], "delivered": st["delivered"], "lost": st["lost"],
                         "bytes": st["bytes"],
                         "p50_ms": round(float(np.percentile(lat, 50)) * 1e3, 2) if len(lat) else None,
                         "p95_ms": round(float(np.percentile(lat, 95)) * 1e3, 2) if len(lat) else None}
        self.stats.clear()
        return out

    def summary(self) -> dict:
        reach = self.reach & np.outer(self.alive_phys, self.alive_phys)
        n_alive = int(self.alive_phys.sum())
        pairs = max(1, n_alive * (n_alive - 1))
        return {"load_max": round(float(self.load.max()), 3),
                "load_mean": round(float(self.load[self.alive_phys].mean()), 3) if n_alive else 0.0,
                "reach_frac": round((int(reach.sum()) - n_alive) / pairs, 3),
                "hops_max": int(np.where(reach, self.hops, 0).max())}


class IdealNetwork(Network):
    def __init__(self, n_nodes, cfg, rng):
        super().__init__(n_nodes, cfg, rng)
        c = cfg.get("ideal", {})
        self.delay = float(c.get("delay_s", 0.003))
        self.loss = float(c.get("loss", 0.0))

    def _compute(self) -> None:
        route = self._route_mask()
        phys = self._phys_mask()
        self.reach = route
        self.p = np.where(phys, 1.0 - self.loss, 0.0)
        self.hops = np.ones((self.n, self.n))
        self.lat0 = np.full((self.n, self.n), max(0.0, self.delay - STACK_BASE_S))

    def send(self, kind, src, dsts, size, reliable, now, deadline=None, udp=False):
        if reliable:  # an ideal reliable link never loses a frame
            saved, self.p = self.p, np.where(self.p > 0, 1.0, 0.0)
            try:
                return super().send(kind, src, dsts, size, reliable, now, deadline, udp)
            finally:
                self.p = saved
        return super().send(kind, src, dsts, size, reliable, now, deadline, udp)


class RadioNetwork(Network):
    def __init__(self, n_nodes, cfg, rng, static_occ: np.ndarray, grid, gateway: int = 0):
        super().__init__(n_nodes, cfg, rng)
        c = cfg.get("radio", {})
        self.band = str(c.get("band", "g24"))
        self.topo_s = float(c.get("topo_s", 1.0))
        self.reroute_delay_s = float(c.get("reroute_delay_s", 2.0))
        self.restore_delay_s = 2 * float(c.get("ogm_interval_s", 0.25))
        self.ogm_s = float(c.get("ogm_interval_s", 0.25))
        self.contention = str(c.get("contention", "global"))
        self.background = float(c.get("background_load", 0.0))
        self.ref_bytes = int(c.get("ref_frame_bytes", 300))
        self.rp = R.RadioParams(**{k: v for k, v in (c.get("radio_model") or {}).items()
                                   if k in R.RadioParams.__dataclass_fields__})
        self.gateway = gateway
        self.layout = _radio_layout(static_occ, grid, float(c.get("radio_cell_m", 0.5)))
        self._rects = _rect_arrays(self.layout)
        self.snr = np.full((n_nodes, n_nodes), -100.0)

    def _load_from_air(self, epoch: float) -> None:
        if epoch <= 0:
            self.load[:] = self.background
            self.air[:] = 0.0
            return
        air = self.air + self._ogm_air(epoch)
        if self.contention == "spatial":
            cs = self.snr >= (self.rp.cs_dbm - self.rp.noise_dbm)
            np.fill_diagonal(cs, True)
            cs &= np.outer(self.alive_phys, self.alive_phys)
            self.load = cs.astype(float) @ air / epoch + self.background
        else:
            self.load[:] = float(air[self.alive_phys].sum()) / epoch + self.background
        over = np.maximum(self.load - LOAD_CAP, 0.0)
        self.backlog = np.maximum(0.0, self.backlog + over * epoch - np.where(over > 0, 0.0, epoch))
        self.air[:] = 0.0

    def _ogm_air(self, epoch: float) -> np.ndarray:
        """batman IV: every node rebroadcasts every originator's OGM each
        interval, aggregated up to 512 B, at the multicast rate."""
        n_alive = int(self.alive_phys.sum())
        nbytes = R.OGM_BYTES * n_alive
        frames = max(1, math.ceil(nbytes / R.OGM_AGGREGATE_MAX))
        per = frames * (R.frame_airtime_us(min(nbytes, R.OGM_AGGREGATE_MAX) + R.MAC_OVERHEAD,
                                           self.rp.multicast_rate_mbps, self.band, unicast=False,
                                           legacy=True)
                        + R.difs_us(self.band) + R.CW_MIN / 2 * R.SLOT_US) * 1e-6
        return np.where(self.alive_phys, per * epoch / self.ogm_s, 0.0)

    def _compute(self) -> None:
        n, rp, band = self.n, self.rp, self.band
        pos = self.pos
        iu, ju = np.triu_indices(n, 1)
        a, b = pos[iu], pos[ju]
        dist = np.hypot(*(a - b).T)
        racks, shelves = _crossings(self._rects, a, b)
        gw = (iu == self.gateway) | (ju == self.gateway)
        rack_db = rp.rack_db_g24 if band == "g24" else rp.rack_db_g5
        rack_loss = racks * rack_db * np.where(gw, rp.gateway_rack_factor, 1.0)
        pl = (R.fspl_1m_db(band) + 10 * rp.pl_exponent * np.log10(np.maximum(dist, 1.0))
              + rack_loss + shelves * rp.shelf_db)
        snr_u = rp.tx_dbm - pl - rp.noise_dbm
        snr = np.full((n, n), -100.0)
        snr[iu, ju] = snr[ju, iu] = snr_u
        self.snr = snr
        # Rate per pair: the fastest the SNR clears with margin (commsim's
        # Minstrel stand-in), the slowest if it clears only the bare threshold.
        table = [rt for rt in (R.HT_RATES if rp.ht else R.LEGACY_RATES) if rt[0] <= rp.max_rate_mbps]
        rate = np.zeros_like(snr_u)
        thr = np.zeros_like(snr_u)
        for r_mbps, th in table:
            pick = snr_u >= th + R.RATE_MARGIN_DB
            rate = np.where(pick, r_mbps, rate)
            thr = np.where(pick, th, thr)
        low = (rate == 0) & (snr_u >= table[0][1])
        rate = np.where(low, table[0][0], rate)
        thr = np.where(low, table[0][1], thr)
        has_rate = rate > 0
        per_u = _per(snr_u, thr, self.ref_bytes)
        hop_ok = 1.0 - per_u ** (R.RETRY_LIMIT + 1)
        tq = (1.0 - _per(snr_u, R.bcast_threshold(rp.multicast_rate_mbps), 100)) ** 2
        usable = has_rate & (tq > 0.1)
        safe_rate = np.where(has_rate, rate, 1.0)
        attempts = 1.0 / np.maximum(1.0 - per_u, 1e-3)
        fixed_us = (R.HT_PREAMBLE_US + R.SIGNAL_EXT_US[band] + R.SIFS_US[band]
                    + R.frame_airtime_us(R.ACK_BYTES, R.ACK_RATE_MBPS, band, unicast=False, legacy=True)
                    + R.difs_us(band) + R.CW_MIN / 2 * R.SLOT_US
                    + 8.0 * (R.UNICAST_FRAME_OVERHEAD + R.ZENOH_PER_MSG) / safe_rate)
        hop_af = fixed_us * 1e-6 * attempts
        hop_ab = 8.0 / safe_rate * 1e-6 * attempts

        route = self._route_mask()
        phys = self._phys_mask()
        D = np.full((n, n), np.inf)
        cost = -np.log(np.maximum(tq, 1e-12) * (1 - HOP_PENALTY))
        P = np.zeros((n, n))
        AF, AB = np.zeros((n, n)), np.zeros((n, n))
        H = np.zeros((n, n))
        ok_u = usable & route[iu, ju]
        D[iu[ok_u], ju[ok_u]] = D[ju[ok_u], iu[ok_u]] = cost[ok_u]
        pu = np.where(phys[iu, ju], hop_ok, 0.0)
        P[iu, ju] = P[ju, iu] = pu
        AF[iu, ju] = AF[ju, iu] = hop_af
        AB[iu, ju] = AB[ju, iu] = hop_ab
        H[iu, ju] = H[ju, iu] = 1.0
        np.fill_diagonal(D, 0.0)
        np.fill_diagonal(P, 1.0)
        np.fill_diagonal(H, 0.0)
        # Floyd-Warshall on route cost, carrying the chosen path's aggregates.
        for k in range(n):
            via = D[:, k:k + 1] + D[k:k + 1, :]
            better = via < D - 1e-12
            if not better.any():
                continue
            D = np.where(better, via, D)
            P = np.where(better, P[:, k:k + 1] * P[k:k + 1, :], P)
            AF = np.where(better, AF[:, k:k + 1] + AF[k:k + 1, :], AF)
            AB = np.where(better, AB[:, k:k + 1] + AB[k:k + 1, :], AB)
            H = np.where(better, H[:, k:k + 1] + H[k:k + 1, :], H)
        self.reach = np.isfinite(D)
        self.p, self.air_f, self.air_b, self.hops = P, AF, AB, H
        self.lat0 = H * PROC_S

    def link_table(self) -> list[dict]:
        """Usable one-hop links (for a viewer or a test), SNR rounded."""
        n = self.n
        out = []
        for i in range(n):
            for j in range(i + 1, n):
                if self.reach[i, j] and self.hops[i, j] == 1:
                    out.append({"a": i, "b": j, "snr_db": round(float(self.snr[i, j]), 1)})
        return out


def _per(snr, thresh, nbytes):
    """commsim radio.per, vectorised."""
    shift = 1.5 * math.log10(1000.0 / max(nbytes, 50))
    x = snr - (thresh - shift)
    return 1.0 / (1.0 + np.exp(np.clip(2.0 * x, -60, 60)))


def _radio_layout(static_occ: np.ndarray, grid, cell_m: float) -> L.Layout:
    """The true static map as commsim racks/shelves: max-pool to the radio
    cell, then from_occupancy (blobs >= 2 m^2 are racks, smaller ones light
    shelving, speckle dropped). Coordinates come back in metres from the
    grid's corner; the layout is shifted into the map frame here."""
    k = max(1, int(round(cell_m / grid.res)))
    h, w = static_occ.shape
    H, W = (h + k - 1) // k, (w + k - 1) // k
    pad = np.zeros((H * k, W * k), dtype=bool)
    pad[:h, :w] = static_occ
    coarse = pad.reshape(H, k, W, k).any(axis=(1, 3))
    occ = "".join("1" if v else "0" for v in coarse.ravel())
    lay = L.from_occupancy(occ, W, H, grid.res * k)
    ox, oy = grid.ox, grid.oy
    lay.racks = [(x0 + ox, y0 + oy, x1 + ox, y1 + oy) for x0, y0, x1, y1 in lay.racks]
    lay.shelves = [(x0 + ox, y0 + oy, x1 + ox, y1 + oy) for x0, y0, x1, y1 in lay.shelves]
    return lay


def _rect_arrays(lay: L.Layout) -> dict:
    racks = np.array(lay.racks, dtype=float).reshape(-1, 4)
    groups = np.array(lay.groups if lay.groups else range(len(lay.racks)), dtype=np.int64)
    order = np.argsort(groups, kind="stable")
    racks, groups = racks[order], groups[order]
    starts = np.flatnonzero(np.r_[True, np.diff(groups) != 0]) if len(groups) else np.zeros(0, int)
    return {"racks": racks, "starts": starts,
            "shelves": np.array(lay.shelves, dtype=float).reshape(-1, 4)}


def _seg_hits(a: np.ndarray, b: np.ndarray, rects: np.ndarray, chunk: int = 4096) -> np.ndarray:
    """(pairs, rects) bool: does segment a-b pass through each rectangle's
    interior. layout._seg_hits_rect (Liang-Barsky), vectorised."""
    out = np.zeros((len(a), len(rects)), dtype=bool)
    if not len(rects) or not len(a):
        return out
    x0, y0, x1, y1 = (rects[:, i][None, :] for i in range(4))
    for s in range(0, len(a), chunk):
        ax, ay = a[s:s + chunk, 0:1], a[s:s + chunk, 1:2]
        dx, dy = b[s:s + chunk, 0:1] - ax, b[s:s + chunk, 1:2] - ay
        t0 = np.zeros((len(ax), len(rects)))
        t1 = np.ones((len(ax), len(rects)))
        hit = np.ones((len(ax), len(rects)), dtype=bool)
        for p, q in ((-dx, ax - x0), (dx, x1 - ax), (-dy, ay - y0), (dy, y1 - ay)):
            par = np.abs(p) < 1e-12
            hit &= ~(par & (q < 0))
            with np.errstate(divide="ignore", invalid="ignore"):
                r = q / np.where(par, 1.0, p)
            neg, pos = (~par) & (p < 0), (~par) & (p > 0)
            t0 = np.where(neg, np.maximum(t0, r), t0)
            t1 = np.where(pos, np.minimum(t1, r), t1)
        out[s:s + chunk] = hit & (t1 - t0 > 1e-9)
    return out


def _crossings(rects: dict, a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(rack groups crossed, shelves crossed) per pair — Layout.crossings,
    vectorised. A link through one wall made of many rectangles pays for
    ONE rack (from_occupancy's groups)."""
    racks = np.zeros(len(a))
    if len(rects["racks"]):
        hits = _seg_hits(a, b, rects["racks"])
        racks = np.logical_or.reduceat(hits, rects["starts"], axis=1).sum(axis=1)
    shelves = _seg_hits(a, b, rects["shelves"]).sum(axis=1) if len(rects["shelves"]) else np.zeros(len(a))
    return racks.astype(float), shelves.astype(float)


def build(cfg: dict, n_nodes: int, rng, static_occ, grid, gateway: int = 0) -> Network:
    model = cfg.get("model", "ideal")
    if model == "ideal":
        return IdealNetwork(n_nodes, cfg, rng)
    if model == "radio":
        return RadioNetwork(n_nodes, cfg, rng, static_occ, grid, gateway)
    raise ValueError(f"network.model: expected ideal | radio, got {model!r}")
