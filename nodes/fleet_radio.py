#!/usr/bin/env python3
"""Fleet mesh radio — intent + status broadcasts (sim DDS stand-in for WiFi).

Autonomy must not look up peer `map -> <other>/base_link` on shared /tf
(that is an oracle). Each robot self-reports pose/twist/mode on
`/fleet/status` and goal/corridor intent on `/fleet/intent`. Schema stays
mesh-shaped so a real WiFi transport can replace the ROS topics later
without rewriting explorers.

This file is both:
  - importable helpers (JSON encode/parse, TTL filtering) used by explorer
    and offline tests, and
  - a runnable node that publishes `/fleet/status` from own TF + odom +
    explore_status (mode).

    python3 nodes/fleet_radio.py --robot-id robot_0
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CFG = REPO_ROOT / "configs" / "fleet_radio.yaml"

STATUS_TOPIC = "/fleet/status"
INTENT_TOPIC = "/fleet/intent"

DEFAULTS = {
    "status_hz": 5.0,
    "status_ttl_s": 1.0,
    "intent_keepalive_s": 2.0,
    "intent_ttl_s": 90.0,
    "claim_radius_m": 1.0,
    "peer_avoid_radius_m": 0.8,
    "corridor_radius_m": 0.6,
    "yield_enabled": True,
    "yield_wait_s": 3.0,
    "yield_horizon_m": 3.0,
    "yield_max_total_s": 12.0,
}


def load_config(path: Path | None = None) -> dict:
    cfg = dict(DEFAULTS)
    p = path or DEFAULT_CFG
    if p.exists():
        with open(p) as f:
            raw = yaml.safe_load(f) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"{p}: expected a mapping")
        cfg.update(raw)
    return cfg


def encode_status(
    robot_id: str,
    x: float,
    y: float,
    yaw: float,
    vx: float = 0.0,
    wz: float = 0.0,
    mode: str = "idle",
    stamp: float | None = None,
) -> str:
    return json.dumps({
        "robot_id": robot_id,
        "pose": {"x": float(x), "y": float(y), "yaw": float(yaw)},
        "twist": {"vx": float(vx), "wz": float(wz)},
        "mode": mode,
        "stamp": float(time.time() if stamp is None else stamp),
    })


def encode_intent(
    robot_id: str,
    x: float,
    y: float,
    *,
    corridor: list[list[float]] | None = None,
    priority: int = 0,
    expires_at: float | None = None,
    released: bool = False,
    intent_ttl_s: float = 90.0,
) -> str:
    data = {
        "robot_id": robot_id,
        "goal": {"x": float(x), "y": float(y)},
        "priority": int(priority),
        "expires_at": float(
            expires_at if expires_at is not None else time.time() + intent_ttl_s
        ),
    }
    if corridor:
        data["corridor"] = [[float(px), float(py)] for px, py in corridor]
    if released:
        data["released"] = True
    return json.dumps(data)


def parse_status(raw: str) -> dict | None:
    try:
        c = json.loads(raw)
        rid = c["robot_id"]
        pose = c["pose"]
        return {
            "robot_id": rid,
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "yaw": float(pose.get("yaw", 0.0)),
            "vx": float((c.get("twist") or {}).get("vx", 0.0)),
            "wz": float((c.get("twist") or {}).get("wz", 0.0)),
            "mode": str(c.get("mode") or "idle"),
            "stamp": float(c.get("stamp") or 0.0),
        }
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None


def parse_intent(raw: str) -> dict | None:
    try:
        c = json.loads(raw)
        rid = c["robot_id"]
        if c.get("released"):
            return {"robot_id": rid, "released": True}
        goal = c["goal"]
        corridor = c.get("corridor") or []
        return {
            "robot_id": rid,
            "x": float(goal["x"]),
            "y": float(goal["y"]),
            "corridor": [[float(p[0]), float(p[1])] for p in corridor],
            "priority": int(c.get("priority", 0)),
            "expires_at": float(c.get("expires_at") or (time.time() + 90.0)),
            "released": False,
        }
    except (json.JSONDecodeError, KeyError, TypeError, ValueError, IndexError):
        return None


def active_peer_poses(
    statuses: dict[str, tuple[dict, float]],
    *,
    now: float,
    ttl_s: float,
    self_id: str,
) -> list[tuple[float, float]]:
    """statuses: robot_id -> (parsed_status, monotonic_recv_ts)."""
    out = []
    for rid, (st, ts) in statuses.items():
        if rid == self_id:
            continue
        if now - ts > ttl_s:
            continue
        out.append((st["x"], st["y"]))
    return out


def active_peer_intents(
    intents: dict[str, tuple[dict, float]],
    *,
    now_wall: float,
    now_mono: float,
    ttl_s: float,
    self_id: str,
) -> list[dict]:
    """intents: robot_id -> (parsed_intent, monotonic_recv_ts)."""
    out = []
    for rid, (it, ts) in intents.items():
        if rid == self_id or it.get("released"):
            continue
        if now_wall > it["expires_at"]:
            continue
        if now_mono - ts > ttl_s:
            continue
        out.append(it)
    return out


def corridor_points(intents: list[dict]) -> list[tuple[float, float]]:
    pts = []
    for it in intents:
        pts.append((it["x"], it["y"]))
        for x, y in it.get("corridor") or []:
            pts.append((x, y))
    return pts


def segments_intersect(
    a0: tuple[float, float],
    a1: tuple[float, float],
    b0: tuple[float, float],
    b1: tuple[float, float],
    *,
    pad_m: float = 0.5,
) -> bool:
    """Rough corridor conflict: any endpoint within pad of the other segment,
    or classic 2D segment intersection."""
    def dist_point_seg(p, u, v):
        ux, uy = u
        vx, vy = v
        px, py = p
        dx, dy = vx - ux, vy - uy
        len2 = dx * dx + dy * dy
        if len2 < 1e-12:
            return math.hypot(px - ux, py - uy)
        t = max(0.0, min(1.0, ((px - ux) * dx + (py - uy) * dy) / len2))
        return math.hypot(px - (ux + t * dx), py - (uy + t * dy))

    if (
        dist_point_seg(a0, b0, b1) <= pad_m
        or dist_point_seg(a1, b0, b1) <= pad_m
        or dist_point_seg(b0, a0, a1) <= pad_m
        or dist_point_seg(b1, a0, a1) <= pad_m
    ):
        return True

    def orient(p, q, r):
        return (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])

    def on_seg(p, q, r):
        return (
            min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9
            and min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9
        )

    o1 = orient(a0, a1, b0)
    o2 = orient(a0, a1, b1)
    o3 = orient(b0, b1, a0)
    o4 = orient(b0, b1, a1)
    if o1 * o2 < 0 and o3 * o4 < 0:
        return True
    if abs(o1) < 1e-9 and on_seg(a0, b0, a1):
        return True
    if abs(o2) < 1e-9 and on_seg(a0, b1, a1):
        return True
    if abs(o3) < 1e-9 and on_seg(b0, a0, b1):
        return True
    if abs(o4) < 1e-9 and on_seg(b0, a1, b1):
        return True
    return False


def should_yield(
    own_priority: int,
    own_start: tuple[float, float],
    own_goal: tuple[float, float],
    peer_intents: list[dict],
    *,
    pad_m: float = 0.6,
    horizon_m: float | None = None,
) -> dict | None:
    """Return the blocking peer intent if we should yield, else None.

    `horizon_m` limits the conflict test to the leading stretch of our own
    route rather than its whole length. Only the part we are about to drive
    can actually conflict with anyone: both robots replan continuously, so a
    crossing 10 m ahead says nothing about where either will be by the time
    it's reached. Without the limit, an exploration goal is routinely 10-12 m
    out (configs/explorer.yaml max_goal_distance_m, and escape mode ignores
    even that), and a straight line that long crosses most of the warehouse —
    so it intersects SOME peer corridor almost always and yielding becomes
    the steady state instead of the exception. Observed live 2026-08-09:
    robot_1 yielded 38 consecutive times and never sent a single goal while
    robot_0 held one long corridor.
    """
    own_leg_end = own_goal
    if horizon_m is not None and horizon_m > 0.0:
        dx, dy = own_goal[0] - own_start[0], own_goal[1] - own_start[1]
        length = math.hypot(dx, dy)
        if length > horizon_m:
            frac = horizon_m / length
            own_leg_end = (own_start[0] + dx * frac, own_start[1] + dy * frac)

    for it in peer_intents:
        if it["priority"] > own_priority:
            continue  # lower number = higher priority; we outrank them
        corridor = it.get("corridor") or []
        # Treat peer corridor as start->…->goal; if empty, just goal disc.
        peer_pts = corridor + [[it["x"], it["y"]]]
        prev = peer_pts[0]
        conflict = False
        for p in peer_pts[1:]:
            # own_leg_end, not own_goal: crossing test is over the stretch we
            # are about to drive. The endpoint-proximity checks below stay on
            # the TRUE goal, since two robots wanting the same spot is a real
            # conflict at any range.
            if segments_intersect(own_start, own_leg_end, tuple(prev), tuple(p), pad_m=pad_m):
                conflict = True
                break
            prev = p
        if not conflict and len(peer_pts) == 1:
            if math.hypot(own_goal[0] - it["x"], own_goal[1] - it["y"]) <= pad_m:
                conflict = True
            elif math.hypot(own_start[0] - it["x"], own_start[1] - it["y"]) <= pad_m:
                conflict = True
        if conflict:
            return it
    return None


def priority_for_robot_id(robot_id: str) -> int:
    """robot_0 -> 0 (highest), robot_1 -> 1, … unknown -> 100."""
    if robot_id.startswith("robot_"):
        try:
            return int(robot_id.split("_", 1)[1])
        except ValueError:
            pass
    return 100


# -- runnable status broadcaster ------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument("--config", type=Path, default=DEFAULT_CFG)
    args = parser.parse_args(argv)

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from std_msgs.msg import String
    import tf2_ros

    cfg = load_config(args.config)
    robot_id = args.robot_id

    STATUS_QOS = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
    )

    class FleetRadioNode(Node):
        def __init__(self):
            super().__init__(f"{robot_id}_fleet_radio")
            self._mode = "idle"
            self._vx = 0.0
            self._wz = 0.0
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.pub = self.create_publisher(String, STATUS_TOPIC, STATUS_QOS)
            self.create_subscription(
                String, f"/{robot_id}/explore_status", self._on_explore, 10
            )
            self.create_subscription(
                Odometry, f"/{robot_id}/odom", self._on_odom, 10
            )
            # Dashboard teleop / Nav2 both end on cmd_vel; use as twist fallback.
            self.create_subscription(Twist, f"/{robot_id}/cmd_vel", self._on_cmd, 10)
            period = 1.0 / max(0.5, float(cfg["status_hz"]))
            self.create_timer(period, self._tick)
            self.get_logger().info(
                f"fleet_radio status on {STATUS_TOPIC} at {cfg['status_hz']} Hz "
                f"(self-reported; peers must not TF-eavesdrop)"
            )

        def _on_explore(self, msg: String):
            try:
                d = json.loads(msg.data)
                # explorer states: exploring/stopped/done; phase for spin.
                st = d.get("state") or "idle"
                phase = d.get("phase") or ""
                if st == "exploring":
                    self._mode = "explore"
                elif st in ("stopped", "done"):
                    self._mode = "idle" if st == "stopped" else "idle"
                else:
                    self._mode = st
                if phase in ("pending", "running"):
                    self._mode = "explore"
                if d.get("stuck"):
                    self._mode = "stuck"
            except (json.JSONDecodeError, TypeError, ValueError):
                pass

        def _on_odom(self, msg: Odometry):
            self._vx = float(msg.twist.twist.linear.x)
            self._wz = float(msg.twist.twist.angular.z)

        def _on_cmd(self, msg: Twist):
            # Prefer commanded twist when odom is quiet at rest.
            if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
                self._vx = float(msg.linear.x)
                self._wz = float(msg.angular.z)

        def _tick(self):
            try:
                tr = self.tf_buffer.lookup_transform(
                    "map", f"{robot_id}/base_link", rclpy.time.Time()
                )
            except Exception:
                return
            t = tr.transform.translation
            q = tr.transform.rotation
            # yaw from quaternion (z-up)
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            msg = String()
            msg.data = encode_status(
                robot_id, t.x, t.y, yaw, self._vx, self._wz, self._mode
            )
            self.pub.publish(msg)

    rclpy.init(args=argv)
    node = FleetRadioNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
