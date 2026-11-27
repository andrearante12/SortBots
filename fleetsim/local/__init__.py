"""The local layer: what a robot does, each step, about what is near it.

Plan item A7 (docs/fleetsim-plan.md); docs/fleetsim.md "Local avoidance".
`nav.local` picks the method:

  stop_replan  (default) stop for what is ahead on the path, replan around
               it after nav.block_replan_s, give the leg up after
               nav.block_fail_s (fleetsim/local/stop_replan.py). The robots'
               Nav2 stand-in, and what every run did before this seam.
  orca         Optimal Reciprocal Collision Avoidance (fleetsim/local/orca.py):
               a velocity each step, from half-planes per neighbour.

What stays outside the seam, in robot.py and sim.py: the global planner
(A* on the robot's own belief), the path follower, the intent hold
(`Robot.hold`, fleetsim/intent.py), the passage hold (`Robot.pass_hold`,
fleetsim/traffic.py) and the contact model (`Sim.try_move`).

The interface. While a robot drives a path and nothing holds it, `Robot._drive`
asks its layer once per step

    decision = robot.local.decide(dt, now)

and does what the answer says:

  Decision(DRIVE)                   follow the path at the follower's own speed
  Decision(DRIVE, vel=(vx, vy))     track this world-frame velocity instead
                                    (`Robot._track`: a diff drive turns toward
                                    it and drives the part along its heading;
                                    path progress follows where it went)
  Decision(STOP, by)                stand still this step. `by` is what for:
                                    robot | actor | notice | bump, the cause
                                    the nav and utilisation metrics book the
                                    stop to
  Decision(STOP, by, around=discs)  ... and first mark these (x, y, r) discs
                                    lethal on the costmap for
                                    nav.overlay_ttl_s (`Overlay`, never
                                    shared) and replan around them
  Decision(GIVE_UP, by)             the leg fails, as a Nav2 abort: the task
                                    FSM gets FAILED

The robot keeps the books, so every method is measured the same way: a STOP
opens a stop if none is open (a `blocked` event; `blocked_s` counts its
steps), a DRIVE closes it (`unblocked`), a GIVE_UP records `gave_up` cause
blocked. `Robot.blocked_since` is when the open stop began: a method reads
it for its own timers and never writes it.

A method is a class built per robot:

  __init__(robot)    read its settings from robot.nav
  decide(dt, now)    -> Decision
  reset()            drop per-stop state: the robot calls it when a stop
                     closes (a DRIVE, an intent or passage hold starting) and
                     when its goal is cleared (a new leg, arrival, a cancel)

It reads what the robot knows, never ground truth unless `info.peers:
truth`: `perceive(robot, now)` (camera detections with a velocity, peers
heard on Status with the twist Status carries, peers' obstacle notices,
bumps), the robot's pose, velocity, path and costmap, and
`robot.preferred_velocity()` (the path follower's velocity). It may call
`robot.sim.record` for its own events. It must not move the robot or
touch the costmap itself: the decision does that.

To add a method: a module here with a class as above, an entry in
METHODS, and its knobs under `nav:` in fleetsim/configs/base.yaml.
"""
from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

DRIVE, STOP, GIVE_UP = "drive", "stop", "give_up"


class Decision(NamedTuple):
    act: str                       # DRIVE | STOP | GIVE_UP
    vel: tuple | None = None       # DRIVE: (vx, vy) to track; None = the follower's own speed
    by: str | None = None          # STOP, GIVE_UP: what the robot stopped for
    around: list | None = None     # STOP: (x, y, r) discs to replan around


FOLLOW = Decision(DRIVE)


def seg_dist(p, a, b) -> float:
    """Distance from point p to the segment a-b."""
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def perceive(r, now: float, predict_s: float = 0.0, status_m: float = 4.0) -> list[tuple]:
    """(x, y, radius, kind, ident, vx, vy, src) for what robot `r` knows is
    near it, in this order: own camera detections (src "camera"; kind robot
    or actor, at the nearest point the cone saw, moved on to `now` along its
    velocity), each followed by where it will be predict_s from now when it
    moves; bumps (kind and src "bump", 0.1 m, where it physically ran into
    something); peers heard on Status within status_m (src "status", kind
    robot, the pose as sent and the twist Status carries); peers' obstacle
    notices (kind and src "notice", carried along their velocity at most
    1 s). Prunes expired bumps and notices as it goes."""
    out = []
    ttl = float(r.nav["detection_ttl_s"])
    for kind, ident, x, y, t, vx, vy in r.dets:
        if now - t <= ttl:
            rad = r.body_r if kind == "robot" else r.sim.actor_radius(ident)
            # Where it is now, and where it will be predict_s from now:
            # obstacle_detector's tracks carry a velocity, and a walker
            # crossing at 1.2 m/s covers a body width between two scans.
            dt = now - t
            out.append((x + vx * dt, y + vy * dt, rad, kind, ident, vx, vy, "camera"))
            if predict_s > 0 and (vx or vy):
                out.append((x + vx * (dt + predict_s), y + vy * (dt + predict_s), rad, kind, ident,
                            vx, vy, "camera"))
    bump_ttl = float(r.nav["bump_ttl_s"])
    r.bumps = [b for b in r.bumps if now - b[2] <= bump_ttl]
    for x, y, _t in r.bumps:
        out.append((x, y, 0.1, "bump", -1, 0.0, 0.0, "bump"))
    sim = r.sim
    st_ttl = float(sim.cfg["status"]["ttl_s"])
    fresh = np.flatnonzero((now - sim.seen_t[r.k] <= st_ttl))
    if len(fresh):
        xy = sim.seen_xy[r.k, fresh]
        near = np.hypot(xy[:, 0] - r.x, xy[:, 1] - r.y) < status_m
        idx = fresh[near]
        for j, (px, py), (vx, vy) in zip(idx.tolist(), xy[near].tolist(),
                                          sim.seen_v[r.k, idx].tolist()):
            if j != r.k:
                out.append((px, py, r.body_r, "robot", j, vx, vy, "status"))
    keep = []
    for o in r.remote_obs:
        if now <= o["expires"]:
            keep.append(o)
            # Extrapolate at most 1 s: a walker turns around, and a notice
            # carried 3 s along a stale velocity put phantom obstacles
            # where no one was (walkers scenario, 2026-11-02).
            dt = min(now - o["t"], 1.0)
            out.append((o["x"] + o["vx"] * dt, o["y"] + o["vy"] * dt, o["r"], "notice", o["id"],
                        o["vx"], o["vy"], "notice"))
    r.remote_obs = keep
    return out


def cluster(blk: tuple, obs: list, link_m: float = 1.8, limit: int = 8) -> list:
    """The blocking disc (x, y, r) plus every disc of `obs` within link_m of
    one already taken (transitively), in the order found, at most `limit`:
    a row of parked peers has to be planned around as a row, or the new
    path just runs into the next one."""
    taken = [blk]
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


class Overlay:
    """Short-lived lethal discs on the robot's costmap (planner.Costmap's
    `overlay` mask): what the local layer replans around. Each lasts
    nav.overlay_ttl_s. Never in the belief, so never shared."""

    def __init__(self, r):
        self.r = r
        self.marks: list[tuple[float, np.ndarray]] = []  # (expires, flat cell indices)

    def add(self, ox: float, oy: float, rad: float, now: float) -> None:
        r = self.r
        g = r.sim.grid
        c = g.cell(ox, oy)
        if c is None:
            return
        idx = []
        for dx, dy, _d in g.disc_offsets(rad):
            nx, ny = c[0] + dx, c[1] + dy
            if 0 <= nx < g.width and 0 <= ny < g.height:
                idx.append(ny * g.width + nx)
        idx = np.array(idx, dtype=np.int64)
        r.cost.overlay.reshape(-1)[idx] = True
        r.cost.update_cells(idx)
        self.marks.append((now + float(r.nav["overlay_ttl_s"]), idx))

    def expire(self, now: float) -> None:
        if not self.marks or self.marks[0][0] > now:
            return
        keep = [(t, i) for t, i in self.marks if t > now]
        gone = np.concatenate([i for t, i in self.marks if t <= now])
        flat = self.r.cost.overlay.reshape(-1)
        flat[gone] = False
        for _t, i in keep:
            flat[i] = True
        self.marks = keep
        self.r.cost.update_cells(gone)


def _methods() -> dict:
    from .orca import Orca
    from .stop_replan import StopReplan
    return {"stop_replan": StopReplan, "orca": Orca}


METHODS = ("stop_replan", "orca")


def build(r):
    """Robot r's local layer, as nav.local says."""
    name = r.nav["local"]
    try:
        return _methods()[name](r)
    except KeyError:
        raise ValueError(f"nav.local: {' | '.join(METHODS)}, got {name!r}") from None
