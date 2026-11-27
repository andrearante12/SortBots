"""nav.local: orca — Optimal Reciprocal Collision Avoidance.

van den Berg, Guy, Lin, Manocha, "Reciprocal n-body collision avoidance"
(ISRR 2009), in the formulation of the RVO2 library (Apache-2.0): its
Agent::computeNewVelocity and linear programs, ported to pure python. RVO2
itself is C++ and fleetsim runs on the system python3 with numpy only.

Each step, for a robot driving a path:

  1. The preferred velocity is the path follower's (Robot.preferred_velocity:
     toward the next path point at the follower's speed cap).
  2. Neighbours, from what the robot knows (fleetsim/local perceive), never
     ground truth unless info.peers: truth: peers heard on Status (their
     centre, carried along the twist Status carries for the Status's age),
     peers the camera sees whose Status is stale (the nearest point seen,
     as the centre of a body-sized disc, with the camera's velocity),
     movers the camera sees and peers' obstacle notices about them. The
     nearest nav.orca.max_neighbors within neighbor_m. What the camera saw
     is kept as a track for track_s after its last sighting, carried along
     its velocity, as obstacle_detector's tracks outlive a frame: the scan
     replaces the detections every 0.2 s, and a walker at the edge of the
     cone came and went every scan, so the robot turned toward its dodge
     and back to its path in turn and never moved off its line
     (walkers, 2026-11-26).
  3. One half-plane of allowed velocities per neighbour: the velocity
     obstacle for time_horizon_s, cut off and projected as RVO2 does. A
     peer moving faster than reciprocal_mps is taken to be avoiding too
     and gets half the correction; a peer standing still (parked, picking,
     held, dead) and every mover (walkers never yield) get all of it.
  4. Walls stay with the global planner: the path is already clear of
     them. What ORCA adds is a guard so a dodge never steers into one: the
     occupied cells (and costmap overlays) within wall_m, nearest per 22.5
     degrees, each a half-plane v . n <= (d - lethal radius) /
     wall_horizon_s, never tighter than the preferred velocity itself (so
     the guard never fights the path, e.g. into a dock), and never
     demanding a move away (0 is always allowed). Bumps (where the robot
     physically ran into something it did not perceive) are half-planes
     too: no velocity toward them. These are RVO2's "obstacle lines":
     hard, kept by the fallback.
  5. A 2D linear program finds the allowed velocity closest to the preferred
     one inside the max-speed circle. When the half-planes leave nothing,
     the 3D fallback finds the velocity that violates the agent half-planes
     least (RVO2's linearProgram3), the wall and bump ones kept.

How a diff drive drives it (an approximation, documented here because ORCA
assumes a holonomic agent that takes the new velocity at once): Robot._track
turns toward the chosen velocity at yaw_rate and drives the part of it
along the heading, none while more than 30 degrees off (the path follower's
rule), accelerating at accel. So the robot lags the velocity ORCA assumed it
took. margin_m on top of both radii is the slack for that lag (RVO2's
radius inflation, the simplest form of NH-ORCA's tracking-error bound).
Two checks on the step it will really take (Robot.track_step): it never
closes on a neighbour to within contact distance (it stands still instead,
booked as a stop for that neighbour), and Robot._track never moves from a
cell the costmap holds free into a lethal one, whatever was asked.

Inside the comfort distance (body + its radius + margin_m), RVO2's
collision branch would demand the whole separation within one time step, a
sideways jump a diff drive can't make. Here the disc's radius shrinks to
just under the distance instead: ORCA then forbids closing in, nothing more.
Physical contact is still sim.try_move's.

When nothing is near and the robot is on its path, it drives the path
exactly as stop_replan does (DRIVE with no velocity). When the chosen
speed falls under stall_mps (and until it is back over resume_mps), the
robot stands still and books a stop for the neighbour that most excludes
the preferred velocity, and from there on it is stop_replan's backstop:
stopped nav.block_replan_s (twice that facing a peer with the lower
index), it marks that neighbour and what is chained to it on the costmap
and replans around them; stopped nav.block_fail_s, it gives the leg up.
That is what clears a peer parked across a single-lane aisle, which no
velocity can pass.

Cost: the neighbour loop and the linear programs are pure python on a
handful of floats; the wall candidates are cached per (cell, costmap
version) and binned with numpy.
"""
from __future__ import annotations

import math

import numpy as np

from . import DRIVE, FOLLOW, GIVE_UP, STOP, Decision, cluster, perceive

EPS = 1e-5
WALL_BINS = 16


# ---- RVO2's linear programs (Agent.cpp), on (px, py, dx, dy) lines -------------
# A line is a point and a unit direction; the allowed side is its left:
# det(direction, point - v) <= 0.

def _lp1(lines, i, radius, opt, direction_opt):
    px, py, dx, dy = lines[i]
    dot = px * dx + py * dy
    disc = dot * dot + radius * radius - (px * px + py * py)
    if disc < 0.0:
        return None  # the max-speed circle lies wholly outside line i
    sq = math.sqrt(disc)
    t_left, t_right = -dot - sq, -dot + sq
    for j in range(i):
        qx, qy, ex, ey = lines[j]
        denom = dx * ey - dy * ex
        numer = ex * (py - qy) - ey * (px - qx)
        if abs(denom) <= EPS:
            if numer < 0.0:
                return None  # parallel, and line i lies outside line j
            continue
        t = numer / denom
        if denom >= 0.0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)
        if t_left > t_right:
            return None
    ox, oy = opt
    if direction_opt:
        t = t_right if ox * dx + oy * dy > 0.0 else t_left
    else:
        t = dx * (ox - px) + dy * (oy - py)
        t = t_left if t < t_left else t_right if t > t_right else t
    return (px + t * dx, py + t * dy)


def _lp2(lines, radius, opt, direction_opt):
    ox, oy = opt
    if direction_opt:
        res = (ox * radius, oy * radius)
    elif ox * ox + oy * oy > radius * radius:
        n = math.hypot(ox, oy)
        res = (ox / n * radius, oy / n * radius)
    else:
        res = (ox, oy)
    for i, (px, py, dx, dy) in enumerate(lines):
        if dx * (py - res[1]) - dy * (px - res[0]) > 0.0:
            nxt = _lp1(lines, i, radius, opt, direction_opt)
            if nxt is None:
                return i, res
            res = nxt
    return len(lines), res


def _lp3(lines, n_static, begin, radius, res):
    distance = 0.0
    for i in range(begin, len(lines)):
        px, py, dx, dy = lines[i]
        if dx * (py - res[1]) - dy * (px - res[0]) <= distance:
            continue
        proj = list(lines[:n_static])
        for j in range(n_static, i):
            qx, qy, ex, ey = lines[j]
            det = dx * ey - dy * ex
            if abs(det) <= EPS:
                if dx * ex + dy * ey > 0.0:
                    continue  # same direction
                pt = (0.5 * (px + qx), 0.5 * (py + qy))
            else:
                s = (ex * (py - qy) - ey * (px - qx)) / det
                pt = (px + s * dx, py + s * dy)
            nx, ny = ex - dx, ey - dy
            n = math.hypot(nx, ny)
            proj.append((pt[0], pt[1], nx / n, ny / n))
        n_ok, nxt = _lp2(proj, radius, (-dy, dx), True)
        if n_ok == len(proj):
            res = nxt  # else float error: keep the last result, as RVO2 does
        distance = dx * (py - res[1]) - dy * (px - res[0])
    return res


def solve(lines: list, n_static: int, vmax: float, pref: tuple) -> tuple[float, float]:
    """The velocity closest to `pref` within vmax that every line allows;
    the first n_static lines are hard, the rest give way least-violated
    first when not all can hold."""
    n_ok, res = _lp2(lines, vmax, pref, False)
    if n_ok < len(lines):
        res = _lp3(lines, n_static, n_ok, vmax, res)
    return res


def agent_line(px, py, rvx, rvy, radius, inv_tau, resp, vx, vy) -> tuple:
    """ORCA half-plane for one neighbour at relative position (px, py), with
    relative velocity (rvx, rvy) = own - its, combined radius, 1 / time
    horizon, our share of the correction and our own velocity. The
    neighbour must be further than `radius` (see the module docstring)."""
    dist_sq = px * px + py * py
    r_sq = radius * radius
    wx, wy = rvx - inv_tau * px, rvy - inv_tau * py
    w_sq = wx * wx + wy * wy
    dot1 = wx * px + wy * py
    if dot1 < 0.0 and dot1 * dot1 > r_sq * w_sq:
        # Project on the cut-off circle.
        w_len = math.sqrt(w_sq)
        ux, uy = wx / w_len, wy / w_len
        dx, dy = uy, -ux
        k = radius * inv_tau - w_len
        cx, cy = k * ux, k * uy
    else:
        # Project on a leg.
        leg = math.sqrt(max(dist_sq - r_sq, 0.0))
        if px * wy - py * wx > 0.0:  # left leg
            dx = (px * leg - py * radius) / dist_sq
            dy = (px * radius + py * leg) / dist_sq
        else:  # right leg
            dx = -(px * leg + py * radius) / dist_sq
            dy = -(-px * radius + py * leg) / dist_sq
        dot2 = rvx * dx + rvy * dy
        cx, cy = dot2 * dx - rvx, dot2 * dy - rvy
    return (vx + resp * cx, vy + resp * cy, dx, dy)


def half_plane(nx: float, ny: float, c: float) -> tuple:
    """The line for v . n <= c (n a unit vector)."""
    return (c * nx, c * ny, -ny, nx)


class Orca:
    def __init__(self, r):
        self.r = r
        self.nav = r.nav
        c = r.nav["orca"]
        self.inv_tau = 1.0 / float(c["time_horizon_s"])
        self.tau_w = float(c["wall_horizon_s"])
        self.near_m = float(c["neighbor_m"])
        self.max_n = int(c["max_neighbors"])
        self.margin = float(c["margin_m"])
        self.wall_m = float(c["wall_m"])
        self.recip = float(c["reciprocal_mps"])
        self.stall = float(c["stall_mps"])
        self.resume = float(c["resume_mps"])
        self.track_s = float(c["track_s"])
        self.tracks: dict = {}  # (kind, ident) -> (x, y, vx, vy, rad, t) of the last sighting
        self._heard: set = set()  # peers heard on Status at the last neighbours()
        self.replan_at = None
        self._wall_key = None
        self._wall_pts = np.zeros((0, 2))

    def reset(self) -> None:
        self.replan_at = None

    # ---- what is near ----------------------------------------------------------

    def neighbours(self, now: float) -> tuple[list, list]:
        """-> ([(dist, x, y, vx, vy, radius, kind, ident)] nearest first, at
        most max_neighbors; [(x, y)] bumps)."""
        r = self.r
        seen = perceive(r, now, 0.0, self.near_m)
        heard = self._heard = {o[4] for o in seen if o[7] == "status"}
        out, bumps = [], []
        seen_t = r.sim.seen_t[r.k]
        cam = set()
        for x, y, rad, kind, ident, vx, vy, src in seen:
            if src == "bump":
                bumps.append((x, y))
                continue
            if src == "camera":
                cam.add((kind, ident))
                self.tracks[(kind, ident)] = (x, y, vx, vy, rad, now)
            if src == "camera" and kind == "robot" and ident in heard:
                continue  # its Status gives its centre; the camera, the nearest point seen
            if src == "status":
                age = now - float(seen_t[ident])
                x, y = x + vx * age, y + vy * age
            d = math.hypot(x - r.x, y - r.y)
            if d <= self.near_m:
                out.append((d, x, y, vx, vy, rad, kind, ident))
        for key, (x, y, vx, vy, rad, t) in list(self.tracks.items()):
            if key in cam:
                continue
            if now - t > self.track_s:
                del self.tracks[key]
                continue
            if key[0] == "robot" and key[1] in heard:
                continue
            x, y = x + vx * (now - t), y + vy * (now - t)
            d = math.hypot(x - r.x, y - r.y)
            if d <= self.near_m:
                out.append((d, x, y, vx, vy, rad, key[0], key[1]))
        out.sort(key=lambda o: o[0])
        return out[:self.max_n], bumps

    def _wall_points(self) -> np.ndarray:
        """Centres of occupied (or overlaid) cells near the robot's cell, cached
        per (cell, costmap version)."""
        r = self.r
        g, cm = r.sim.grid, r.cost
        key = (g.index(r.x, r.y), cm.version)
        if key != self._wall_key:
            self._wall_key = key
            c = g.cell(r.x, r.y)
            if c is None:
                self._wall_pts = np.zeros((0, 2))
            else:
                k = int(math.ceil(self.wall_m / g.res)) + 1
                x0, x1 = max(0, c[0] - k), min(g.width, c[0] + k + 1)
                y0, y1 = max(0, c[1] - k), min(g.height, c[1] + k + 1)
                occ = (cm.state[y0:y1, x0:x1] == 100) | cm.overlay[y0:y1, x0:x1]
                ys, xs = np.nonzero(occ)
                self._wall_pts = np.column_stack((g.ox + (xs + x0 + 0.5) * g.res,
                                                  g.oy + (ys + y0 + 0.5) * g.res))
        return self._wall_pts

    def wall_lines(self, pref: tuple) -> list:
        r = self.r
        pts = self._wall_points()
        if not len(pts):
            return []
        dx, dy = pts[:, 0] - r.x, pts[:, 1] - r.y
        d = np.hypot(dx, dy)
        m = (d <= self.wall_m) & (d > 1e-6)
        if not m.any():
            return []
        dx, dy, d = dx[m], dy[m], d[m]
        b = ((np.arctan2(dy, dx) + math.pi) * (WALL_BINS / (2 * math.pi))).astype(np.int64) % WALL_BINS
        order = np.argsort(d, kind="stable")
        _, first = np.unique(b[order], return_index=True)
        sel = order[first]
        r_l = r.cost.r_lethal
        out = []
        for ddx, ddy, dd in zip(dx[sel].tolist(), dy[sel].tolist(), d[sel].tolist()):
            nx, ny = ddx / dd, ddy / dd
            c = max(0.0, (dd - r_l) / self.tau_w, pref[0] * nx + pref[1] * ny)
            out.append(half_plane(nx, ny, c))
        return out

    # ---- the decision ------------------------------------------------------------

    def velocity(self, now: float):
        """-> (chosen velocity, preferred velocity, neighbours, agent lines,
        bumps)."""
        r = self.r
        pref = r.preferred_velocity()
        nbrs, bumps = self.neighbours(now)
        if not nbrs and not bumps:
            return pref, pref, nbrs, [], bumps
        vx, vy = r.velocity()
        static = self.wall_lines(pref)
        for bx, by in bumps:
            d = math.hypot(bx - r.x, by - r.y)
            if d > 1e-6:
                static.append(half_plane((bx - r.x) / d, (by - r.y) / d, 0.0))
        lines = []
        for d, ox, oy, ovx, ovy, rad, kind, _ident in nbrs:
            if d < 0.02:
                lines.append(None)
                continue
            radius = r.body_r + rad + self.margin
            if d <= radius:
                radius = d - 0.01  # inside the comfort distance: forbid closing in, no more
            resp = 0.5 if kind == "robot" and ovx * ovx + ovy * ovy > self.recip * self.recip \
                else 1.0
            lines.append(agent_line(ox - r.x, oy - r.y, vx - ovx, vy - ovy, radius, self.inv_tau,
                                    resp, vx, vy))
        agents = [ln for ln in lines if ln is not None]
        v = solve(static + agents, len(static), r.vmax, pref)
        return v, pref, nbrs, lines, bumps

    def contact_step(self, dt: float, now: float, v: tuple, nbrs: list):
        """The neighbour the tracker's next step toward `v` would close on to
        within contact distance (body + its radius + 2 cm), or None. ORCA
        assumes the robot takes `v` at once; a diff drive moves along its
        heading, and when no velocity is safe (a walker coming head-on faster
        than the robot can turn away) the fallback's least-bad one can still
        step into it. Standing still is then the safe move, as stop_replan's.
        A camera track is checked where it was last seen too, not only where
        its velocity carried it: close up, the nearest-point stand-in's
        velocity is noise (a walker read as -2.05, +1.69 m/s while pacing
        along x; bench_walkers seed 3, 2026-11-26), and the carried track
        was 0.3 m from the walker the robot then stepped into."""
        r = self.r
        x, y, _yaw, sp = r.track_step(dt, v)
        if sp * dt <= 1e-9:
            return None
        pts = [(o[1], o[2], o) for o in nbrs]
        for (kind, ident), (tx, ty, _vx, _vy, rad, t) in self.tracks.items():
            if now - t <= self.track_s and not (kind == "robot" and ident in self._heard):
                pts.append((tx, ty, (math.hypot(tx - r.x, ty - r.y), tx, ty, 0.0, 0.0, rad,
                                     kind, ident)))
        for px, py, o in pts:
            d = math.hypot(px - x, py - y)
            if d < math.hypot(px - r.x, py - r.y) and d < r.body_r + o[5] + 0.02:
                return o
        return None

    def decide(self, dt: float, now: float) -> Decision:
        r = self.r
        v, pref, nbrs, lines, bumps = self.velocity(now)
        if not nbrs and not bumps and r.on_path():
            return FOLLOW
        sp = math.hypot(v[0], v[1])
        need = self.resume if r.blocked_since is not None else self.stall
        blk = None
        if nbrs:
            blk = self.contact_step(dt, now, v, nbrs)
        if blk is None and (sp >= need or math.hypot(pref[0], pref[1]) < need or not nbrs):
            return Decision(DRIVE, vel=v)
        if blk is None:
            # Stalled: book a stop for the neighbour that most excludes the
            # preferred velocity, and from here on it is stop_replan's backstop.
            worst, blk = -math.inf, nbrs[0]
            for o, ln in zip(nbrs, lines):
                if ln is not None:
                    viol = ln[2] * (ln[1] - pref[1]) - ln[3] * (ln[0] - pref[0])
                    if viol > worst:
                        worst, blk = viol, o
        by = blk[6]
        since = now if r.blocked_since is None else r.blocked_since
        if now - since >= float(self.nav["block_fail_s"]):
            return Decision(GIVE_UP, by=by)
        first = by != "robot" or blk[7] < r.k
        due = float(self.nav["block_replan_s"]) * (1 if first else 2)
        if self.replan_at is None:
            self.replan_at = now + due
        if now >= self.replan_at and now >= r.next_replan_ok:
            self.replan_at = now + due
            obs = [(o[1], o[2], o[5]) for o in nbrs if o[0] < 6.0]
            return Decision(STOP, by=by, around=cluster((blk[1], blk[2], blk[5]), obs))
        return Decision(STOP, by=by)
