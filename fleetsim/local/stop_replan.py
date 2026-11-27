"""nav.local: stop_replan — the default local layer, the robots' Nav2 stand-in.

Each step it looks nav.stop_distance_m along the path for anything the
robot knows of (fleetsim/local perceive: camera detections and where a
moving one will be nav.predict_s ahead, bumps, peers heard on Status,
peers' obstacle notices). Something whose centre comes within body + its
radius + nav.clearance_margin_m of the path ahead, and closer than the
robot already is, stops the robot. Stopped nav.block_replan_s, it marks the
blocker and whatever is chained to it lethal for nav.overlay_ttl_s and
replans around them; facing a peer, the robot with the lower index waits a
second interval, so two robots don't swerve into each other in step.
Stopped nav.block_fail_s, it gives the leg up (a Nav2 abort after its
recoveries).

This is what robot.py did before the local layer had a seam (A7,
2026-11-26), moved here unchanged: a default run is event-for-event what it
was.
"""
from __future__ import annotations

import math

from . import FOLLOW, GIVE_UP, STOP, Decision, cluster, perceive, seg_dist


class StopReplan:
    def __init__(self, r):
        self.r = r
        self.nav = r.nav
        self.replan_at = None  # when the open stop next replans around its blocker

    def reset(self) -> None:
        self.replan_at = None

    def obstacles(self, now: float) -> list[tuple]:
        """(x, y, radius, kind, ident, ...) this layer avoids."""
        return perceive(self.r, now, float(self.nav["predict_s"]))

    def blocking(self, now: float):
        """The first obstacle in the way along the path ahead, as (x, y,
        radius, kind, ident), or None."""
        r = self.r
        stop = float(self.nav["stop_distance_m"])
        ahead = r.ahead(stop)
        if len(ahead) < 2:
            return None
        hx, hy = math.cos(r.yaw), math.sin(r.yaw)
        margin = float(self.nav["clearance_margin_m"])
        # Skip the first few cm of the path: something right beside the robot
        # that is not in the way forward must not pin it in place.
        start = ahead[0]
        if len(ahead) >= 2 and math.dist(ahead[0], ahead[1]) > 0.1:
            f = 0.1 / math.dist(ahead[0], ahead[1])
            start = (ahead[0][0] + f * (ahead[1][0] - ahead[0][0]),
                     ahead[0][1] + f * (ahead[1][1] - ahead[0][1]))
        segs = [start] + ahead[1:]
        for ox, oy, rad, kind, ident, *_ in self.obstacles(now):
            if math.hypot(ox - r.x, oy - r.y) > stop + rad + r.body_r + 0.5:
                continue
            if (ox - r.x) * hx + (oy - r.y) * hy < -0.05:
                continue  # behind: driving on only gains distance
            need = r.body_r + rad + margin
            here = math.hypot(ox - r.x, oy - r.y)
            closest = min(seg_dist((ox, oy), a, b) for a, b in zip(segs[:-1], segs[1:]))
            # In the way only if the path ahead comes within `need` AND closer
            # than the robot already is: a peer that stopped right beside or
            # in front of us must not pin us when the new path leads away.
            if closest < need and closest < here - 0.02:
                return (ox, oy, rad, kind, ident)
        return None

    def decide(self, dt: float, now: float) -> Decision:
        r, nav = self.r, self.nav
        blk = self.blocking(now)
        if blk is None:
            return FOLLOW
        since = now if r.blocked_since is None else r.blocked_since
        if now - since >= float(nav["block_fail_s"]):
            return Decision(GIVE_UP, by=blk[3])
        # Replan around it. Facing a peer, the lower-priority robot (higher
        # id, fleet_radio's order) goes first; the other waits a second
        # interval, so two robots don't swerve into each other in step.
        first = blk[3] != "robot" or blk[4] < r.k
        due = float(nav["block_replan_s"]) * (1 if first else 2)
        if self.replan_at is None:
            self.replan_at = now + due
        if now >= self.replan_at and now >= r.next_replan_ok:
            self.replan_at = now + due  # still blocked after this: wait again
            # Mark the blocker and whatever is chained to it.
            obs = [(o[0], o[1], o[2]) for o in self.obstacles(now)
                   if math.hypot(o[0] - r.x, o[1] - r.y) < 6.0]
            return Decision(STOP, by=blk[3], around=cluster((blk[0], blk[1], blk[2]), obs))
        return Decision(STOP, by=blk[3])
