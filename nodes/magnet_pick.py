#!/usr/bin/env python3
"""The magnet pick controller: turns task_manager's pick_request into arm and base motion.

Every decision is in nodes/pick_planner.py (and the arm maths in nodes/arm_kin.py); this
node is the wiring, and it replaces nodes/scripted_pick.py's recorded joint sequences:

    task_manager --[[/<id>/pick_request: JSON]]--> this node
        {"cmd": "attach", "package", "task_id", "size", "face", "at": {x, y}}
        {"cmd": "detach", "package", "task_id", "size", "face", "place": {x, y, z, yaw}}
        {"cmd": "cancel"}
    this node   --[[/<id>/arm_joint_cmd: JointState, all six joints]]--> Isaac
    this node   --[[/<id>/pick_cmd: "attach" | "detach"]]-----------> Isaac (the magnet)
    this node   --[[/<id>/cmd_vel: Twist]]-------------------------> Isaac (docking only)
    this node   --[[/<id>/pick_result: "picked" | "placed" | "failed"]]--> task_manager
    this node   --[[/<id>/pick_status: JSON, latched]]--> the dashboard, tests

WHAT IT PERCEIVES, and this is the thing to be honest about. `--perception camera` (the
default) aims at /<id>/packages, the plates nodes/package_detector.py finds in the head
camera, and stands where its own localization (TF map -> base_link) says it stands.
`--perception oracle` aims at the simulator's ground truth for the requested package
instead, with no camera in the loop: it exists so that a failed pick can be told apart,
"the camera missed it" from "it saw it and the dock or the arm got it wrong", by running
the same test both ways (scenario run key `perception`, docs/manipulation.md). Oracle mode
is a sim-only diagnostic and says so in /pick_status. Both modes take the one bit of
"is the magnet holding something" from the simulator's ground truth; on the real arm that
is the microswitch behind the plunger.

TIME. The planner runs on SIM seconds (ground truth's `t`): the arm and base move in sim
time, so its deadlines have to. The timer that ticks it is the steady clock, as
nodes/explorer.py does it, because a timer on the node clock stretches by the real-time
factor under use_sim_time (CLAUDE.md).

Environment: system python3 + ROS 2 Jazzy. Nothing else (no torch: nodes/arm_kin.py).

    python3 nodes/magnet_pick.py --robot-id robot_0 [--perception oracle]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(REPO_ROOT))

import arm_kin  # noqa: E402
import pick_planner  # noqa: E402
from simtest import magnet  # noqa: E402

DEFAULT_CONFIG = REPO_ROOT / "configs" / "magnet_pick.yaml"
# Order is fixed: spawn_warehouse._read_arm_cmd drops a command missing any of them.
ARM_JOINT_NAMES = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]
STATUS_EVERY_S = 0.5
PLATE_STALE_S = 1.0       # a /packages message older than this (in sim time) is not evidence


# -- pure helpers (tested without ROS) ---------------------------------------------------

def parse_request(text: str) -> dict | None:
    """A pick_request's JSON, or None if it isn't one we can act on."""
    try:
        req = json.loads(text)
    except ValueError:
        return None
    if not isinstance(req, dict) or req.get("cmd") not in ("attach", "detach", "cancel"):
        return None
    # `at` may be null: the work order doesn't say where, depth geometry finds it
    if req["cmd"] == "attach" and not all(k in req for k in ("package", "size", "face", "at")):
        return None
    if req["cmd"] == "detach" and not all(k in req for k in ("package", "size", "face", "place")):
        return None
    return req


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def oracle_plates(gt: dict, req: dict | None) -> list[dict]:
    """The requested package's plate from ground truth: what a perfect camera would
    report, from anywhere, at any range. Empty while it is held (or not requested)."""
    if not req or req.get("cmd") != "attach":
        return []
    pkg = (gt.get("packages") or {}).get(req["package"])
    if not pkg or pkg.get("held_by"):
        return []
    centre, normal = magnet.magnet_face_pose(pkg, req["size"], req["face"])
    return [{"x": centre[0], "y": centre[1], "z": centre[2], "nx": normal[0], "ny": normal[1],
             "nz": normal[2], "conf": 1.0, "age_s": 0.0, "marker": req.get("marker")}]


def camera_plates(msg: dict | None, now: float, stale_s: float = PLATE_STALE_S) -> list[dict]:
    """The detector's latest plates, aged to `now` (sim seconds), or [] if it has gone
    quiet: a node that died must not leave its last answer looking fresh."""
    if not msg:
        return []
    behind = now - float(msg.get("stamp", now))
    if behind > stale_s:
        return []
    return [dict(p, age_s=float(p.get("age_s", 0.0)) + max(0.0, behind))
            for p in msg.get("plates", [])]


def compose2d(a: tuple, b: tuple) -> tuple:
    """Pose `b` (x, y, yaw) given in the frame of pose `a`, in a's parent frame."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def fresh_boxes(msg: dict | None, now: float, stale_s: float = PLATE_STALE_S) -> list[dict]:
    """Depth geometry's latest boxes, or [] if they are old: same rule as the plates."""
    if not msg or now - float(msg.get("stamp", now)) > stale_s:
        return []
    return list(msg.get("boxes", []))


def load_config(path: Path) -> dict:
    try:
        return yaml.safe_load(Path(path).read_text()) or {}
    except (OSError, yaml.YAMLError):
        return {}


# -- the node ------------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot-id", default="robot_0")
    ap.add_argument("--perception", choices=("camera", "oracle"), default="camera",
                    help="camera: aim at /<id>/packages. oracle: aim at ground truth (sim only; "
                         "separates perception failures from dock/arm failures)")
    ap.add_argument("--urdf", type=Path, default=arm_kin.URDF_PATH)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    a = ap.parse_args(argv)

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    rid = a.robot_id
    cfg = load_config(a.config)
    stream_hz = float(cfg.pop("stream_hz", 20.0))
    kin = arm_kin.ArmKin(a.urdf)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class MagnetPick(Node):
        def __init__(self):
            super().__init__(f"{rid}_magnet_pick")
            self.planner = pick_planner.PickPlanner(kin, cfg)
            self.oracle = a.perception == "oracle"
            self.gt: dict | None = None
            self.packages_msg: dict | None = None
            self.boxes_msg: dict | None = None
            self.req: dict | None = None
            self.last_phase = None
            self.last_status = -1e9
            self.tf = Buffer()
            self._tfl = TransformListener(self.tf, self)
            self.arm_pub = self.create_publisher(JointState, f"/{rid}/arm_joint_cmd", 10)
            self.pick_cmd_pub = self.create_publisher(String, f"/{rid}/pick_cmd", 10)
            self.vel_pub = self.create_publisher(Twist, f"/{rid}/cmd_vel", 10)
            self.result_pub = self.create_publisher(String, f"/{rid}/pick_result", 10)
            self.status_pub = self.create_publisher(String, f"/{rid}/pick_status", latched)
            self.create_subscription(String, f"/{rid}/pick_request", self.on_request, 10)
            self.create_subscription(String, "/sim/ground_truth", self.on_gt, 10)
            self.create_subscription(String, f"/{rid}/packages", self.on_packages, 10)
            self.create_subscription(String, f"/{rid}/boxes", self.on_boxes, 10)
            self.create_timer(1.0 / stream_hz, self.step,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.log(f"up for {rid}, perception={a.perception}")
            self.publish_status()

        def log(self, msg):
            self.get_logger().info(msg)

        # -- inputs ------------------------------------------------------------------------
        def on_gt(self, m):
            try:
                self.gt = json.loads(m.data)
            except ValueError:
                pass

        def on_packages(self, m):
            try:
                self.packages_msg = json.loads(m.data)
            except ValueError:
                pass

        def on_boxes(self, m):
            try:
                self.boxes_msg = json.loads(m.data)
            except ValueError:
                pass

        def now(self) -> float | None:
            return None if self.gt is None else float(self.gt["t"])

        def base_pose(self):
            """The robot's pose in the frame the plates are in. Oracle: ground truth for
            both, so they agree by construction. Camera: our own localization, the same
            TF chain the detector used to put the plates in the map."""
            if self.oracle:
                r = (self.gt or {}).get("robots", {}).get(rid)
                return (r["x"], r["y"], r["yaw"]) if r else None
            def pose2d(tr):
                t, q = tr.transform.translation, tr.transform.rotation
                return (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))

            # In two halves, as the detector does (package_detector.Detector.map_from): one
            # lookup would be held back to the age of SLAM's slow map -> odom.
            latest = rclpy.time.Time()
            try:
                return compose2d(pose2d(self.tf.lookup_transform("map", f"{rid}/odom", latest)),
                                 pose2d(self.tf.lookup_transform(f"{rid}/odom", f"{rid}/base_link", latest)))
            except Exception:  # noqa: BLE001 - a chain with no odom frame of this name
                pass
            try:
                return pose2d(self.tf.lookup_transform("map", f"{rid}/base_link", latest))
            except Exception:  # noqa: BLE001 - no map yet: no pose, the planner waits
                return None

        def on_request(self, m):
            req = parse_request(m.data)
            now = self.now()
            if req is None:
                self.get_logger().error(f"unusable pick_request {m.data!r}")
                if m.data.strip() in ("attach", "detach"):
                    self.reply("failed")      # the old bare-string protocol has no package to aim at
                return
            if req["cmd"] == "cancel":
                self.log("cancel")
                self.emit(self.planner.cancel(now or 0.0))
                self.req = None
                return
            if now is None:
                self.get_logger().error("pick_request before any ground truth: refusing")
                return self.reply("failed")
            self.log(f"{req['cmd']} {req['package']}")
            self.req = req
            self.planner.request(req, now)

        # -- outputs -----------------------------------------------------------------------
        def reply(self, text):
            self.result_pub.publish(String(data=text))

        def emit(self, d):
            if d.q is not None:
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = list(ARM_JOINT_NAMES)
                js.position = [float(v) for v in d.q]
                self.arm_pub.publish(js)
            if d.pick_cmd is not None:
                self.pick_cmd_pub.publish(String(data=d.pick_cmd))
            if d.vel is not None:
                tw = Twist()
                tw.linear.x, tw.linear.y, tw.angular.z = (float(v) for v in d.vel)
                self.vel_pub.publish(tw)
            if d.result:
                self.log(f"-> {d.result}" + (f" ({d.note})" if d.note else ""))
                self.reply(d.result)

        def publish_status(self, d=None):
            req = self.req or {}
            aim = self.planner.target if req.get("cmd") == "attach" else None
            self.status_pub.publish(String(data=json.dumps({
                "phase": self.planner.phase, "busy": self.planner.busy,
                # The robot's state as far as perception goes: while a pick or place runs it
                # is looking for packages, not driving among obstacles.
                "mode": "manipulation" if self.planner.busy else "nav",
                "marker": req.get("marker") if self.planner.busy else None,
                "arm_err_rad": self.planner.arm_err, "dock_shift_m": self.planner.dock_shift,
                "target": [round(v, 3) for v in aim["c"]] if aim else None,
                "perception": a.perception, "cmd": req.get("cmd"), "package": req.get("package"),
                "note": getattr(d, "note", "") if d else ""})))

        # -- the loop ----------------------------------------------------------------------
        def step(self):
            now = self.now()
            if now is None:
                return
            plates = (oracle_plates(self.gt, self.req) if self.oracle
                      else camera_plates(self.packages_msg, now))
            held = bool(((self.gt.get("magnet") or {}).get(rid) or {}).get("held"))
            d = self.planner.tick(pick_planner.Obs(
                now=now, base=self.base_pose(), plates=plates, held=held,
                q=(self.gt.get("arm") or {}).get(rid), boxes=fresh_boxes(self.boxes_msg, now)))
            self.emit(d)
            if d.result:
                self.req = None
            if d.phase != self.last_phase or now - self.last_status > STATUS_EVERY_S:
                if d.phase != self.last_phase:
                    self.log(f"phase {d.phase}")
                self.last_phase, self.last_status = d.phase, now
                self.publish_status(d)

    rclpy.init()
    node = MagnetPick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
