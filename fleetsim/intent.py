"""Path intents: where each robot will be, and when, so the lower-priority
robot of a future conflict waits somewhere safe instead of meeting it.

Design (2026-11-17; docs/fleetsim.md "Path intents"):

  * A robot with a path announces the next `horizon_s` of it as a few
    timestamped points (`announce`), on every replan and at `hz`, to the
    robots within `radius_m`. Times are offsets from the send, so the
    receiver re-bases them on its own clock: no shared clock is assumed.
  * Every `check_hz` a robot predicts its own path at nominal speed and
    compares it, time step by time step, with each fresher-than-`ttl_s`
    intent of a peer that outranks it (`first_conflict`). Lower `prio`
    wins: (task class, -aging bucket, robot index), so a robot carrying an
    item outranks one fetching, which outranks one going home, and a robot
    that has spent long yielding climbs within its class.
  * On a conflict it looks back along its own prediction for the last point
    that keeps clear of the peer's whole announced path (`safe_hold`), and
    holds there when it gets there. It only yields when the conflict clears
    within `max_wait_s`: two robots CAN pass in a 2.4 m aisle, and waiting at
    the aisle mouth while a peer drives its whole length costs more than the
    local layer's swerve. No safe point, or too long a wait: the local layer
    (stop, replan around) handles it, as before intents.
  * A holding robot announces nothing moving (one point), so nobody yields to
    a robot that is standing still; standing robots are the local layer's job.

The prediction ignores acceleration and turning in place, which is what
`speed_factor` and the separation margin absorb; it is refreshed at `hz`, so
the error stays bounded by about a second of driving.
"""
from __future__ import annotations

import math

import numpy as np


def along(pts: list, v: float, horizon_s: float, ds: float) -> tuple[np.ndarray, np.ndarray]:
    """Positions every ds seconds along a polyline driven at speed v, for
    horizon_s. -> (offsets s, xy (n, 2)). Past the end it stays at the end."""
    n = int(horizon_s / ds) + 1
    taus = np.arange(n) * ds
    p = np.asarray(pts, dtype=float).reshape(-1, 2)
    if len(p) < 2 or v <= 0:
        return taus, np.repeat(p[:1], n, axis=0)
    seg = np.hypot(*np.diff(p, axis=0).T)
    cum = np.r_[0.0, np.cumsum(seg)]
    s = np.minimum(taus * v, cum[-1])
    return taus, np.c_[np.interp(s, cum, p[:, 0]), np.interp(s, cum, p[:, 1])]


def announce(pts: list, v: float, horizon_s: float, max_pts: int = 12) -> list:
    """The intent's points: [dx..] as [[x, y, dt], ...] at the path's own
    vertices (where it turns), cut at horizon_s, thinned to max_pts."""
    p = [tuple(map(float, q)) for q in pts]
    if len(p) < 2 or v <= 0:
        return [[round(p[0][0], 2), round(p[0][1], 2), 0.0]] if p else []
    out, t = [[p[0][0], p[0][1], 0.0]], 0.0
    for a, b in zip(p[:-1], p[1:]):
        d = math.dist(a, b)
        if t + d / v >= horizon_s:
            f = (horizon_s - t) * v / max(d, 1e-9)
            out.append([a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), horizon_s])
            break
        t += d / v
        out.append([b[0], b[1], t])
    if len(out) > max_pts:  # keep the ends, thin the middle evenly
        keep = np.unique(np.round(np.linspace(0, len(out) - 1, max_pts)).astype(int))
        out = [out[i] for i in keep]
    return [[round(x, 2), round(y, 2), round(dt, 2)] for x, y, dt in out]


def peer_at(pts_abs: np.ndarray, times: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """A peer's announced position at absolute times. -> (xy, valid): valid
    only within its announcement (before the first point it was AT the first
    point; after the last one we know nothing)."""
    t = pts_abs[:, 2]
    xy = np.c_[np.interp(times, t, pts_abs[:, 0]), np.interp(times, t, pts_abs[:, 1])]
    return xy, times <= t[-1]


def first_conflict(own_xy: np.ndarray, times: np.ndarray, pts_abs: np.ndarray,
                   sep_m: float) -> int | None:
    """Index of the first time step at which we would be within sep_m of the
    peer, or None."""
    if len(pts_abs) < 2:
        return None  # standing still: the local layer's
    pxy, ok = peer_at(pts_abs, times)
    close = ok & (np.hypot(*(own_xy - pxy).T) < sep_m)
    hit = np.flatnonzero(close)
    return int(hit[0]) if len(hit) else None


def clears_at(own_xy: np.ndarray, times: np.ndarray, pts_abs: np.ndarray, sep_m: float,
              c: int) -> float:
    """When the peer is last within sep_m of the spot we would conflict at,
    as an offset from times[0]. Minus when we reach the hold spot, that is
    the wait a yield costs."""
    spot = own_xy[c]
    dense = np.arange(times[0], pts_abs[-1, 2] + 1e-9, max(times[1] - times[0], 0.05)) \
        if len(times) > 1 else times
    pxy, _ = peer_at(pts_abs, dense)
    near = np.flatnonzero(np.hypot(*(pxy - spot).T) < sep_m)
    return float(dense[near[-1]] - times[0]) if len(near) else 0.0


def safe_hold(own_xy: np.ndarray, c: int, pts_abs: np.ndarray, now: float, clear_m: float,
              ds: float = 0.25) -> int | None:
    """The last index before c whose point keeps clear_m from everywhere the
    peer has announced it will be from now on. None if even where we stand
    is in its way."""
    dense = np.arange(now, pts_abs[-1, 2] + 1e-9, ds)
    if not len(dense):
        return c - 1 if c > 0 else None
    pxy, _ = peer_at(pts_abs, dense)
    for i in range(c - 1, -1, -1):
        if np.min(np.hypot(*(pxy - own_xy[i]).T)) >= clear_m:
            return i
    return None


def outranks(a: tuple, b: tuple) -> bool:
    """Priority keys compare lexicographically; lower is higher priority."""
    return tuple(a) < tuple(b)
