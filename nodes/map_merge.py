#!/usr/bin/env python3
"""Fuse every robot's own RTAB-Map grid into one world-anchored `/map`.

Each robot's RTAB-Map instance owns and publishes its OWN occupancy grid
(`/{robot_id}/map`) in its OWN SLAM frame (`{robot_id}/map` — see
launch/sortbots_rtabmap_robot.launch.py's `map_frame_id`). Two robots'
`{robot_id}/map` origins are unrelated: this is deliberately NOT collaborative
SLAM (no shared pose graph, no inter-robot loop closure) — see
nodes/explorer.py's module docstring for why that's future work.

What makes the fleet collaborative today is this node: it looks up the static
`map -> {robot_id}/map` transform launch/sortbots_bringup.launch.py publishes
for each robot (from that robot's known spawn pose, configs/robots.yaml), uses
it to resample every robot's grid into one shared `map`-frame grid, and
publishes the union as `/map` — occupied cells win over free, free wins over
unknown, when two robots' grids disagree about the same world cell. Nav2's
`global_frame: map` (configs/nav2_params.yaml) and every explorer's default
`--map-topic /map` (nodes/explorer.py) both point at this fused grid, so a
robot plans and picks frontiers using space ANY robot has mapped — the actual
payoff of running a fleet instead of independent single-robot demos.

Runs correctly with exactly one robot too (anchor + passthrough) — this is
the same code path the single-robot scenarios use, not a special case.

Also republishes every robot's anchor onto its own `/map_anchors` topic (a
tf2_msgs/TFMessage, TRANSIENT_LOCAL) so the web dashboard can place every
robot on one canvas — see _tick()'s comment for why this can't just piggyback
on /tf.

The actual fusion math lives in nodes/map_fuse.py, split out so it has no
rclpy import and can be unit-tested offline (tests/map_merge_test.py); this
file is the thin ROS wrapper around it.

Run (system ROS 2 Jazzy sourced, NOT the Isaac venv or conda):

    source /opt/ros/jazzy/setup.bash
    python3 nodes/map_merge.py --robot-ids robot_0,robot_1

Watch it work:

    ros2 topic echo /map --once
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parent))
from map_fuse import GridInfo, Transform2D, fuse_grids, yaw_from_quaternion  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "configs" / "map_merge.yaml"

# Matches nodes/explorer.py's MAP_QOS — RTAB-Map's /{robot_id}/map is latched,
# and /map (published here) is latched the same way so a late-starting
# consumer (a joining robot's explorer, the dashboard) gets the current fused
# grid immediately instead of waiting for the next publish_period_s tick.
MAP_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

# depth = enough for one sample per robot; TRANSIENT_LOCAL so a dashboard
# tab opened at any time gets every robot's current anchor immediately, not
# just whichever one happens to still be "in flight" when it connects.
ANCHOR_QOS = QoSProfile(
    depth=8,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

DEFAULTS = {
    "publish_period_s": 1.0,
    "occupied_thresh": 50,
}


def load_config(path: Path | None) -> dict:
    cfg = dict(DEFAULTS)
    if path and path.exists():
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
        cfg.update(raw)
    return cfg


class MapMergeNode(Node):
    def __init__(self, robot_ids: list[str], cfg: dict):
        super().__init__("map_merge")
        self.robot_ids = robot_ids
        self.cfg = cfg
        self._grids: dict[str, tuple[np.ndarray, GridInfo]] = {}
        self._prev_out_info: GridInfo | None = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", MAP_QOS)
        # Dedicated topic for the map -> <rid>/map anchors, deliberately NOT
        # rebroadcast onto /tf — see _tick()'s comment for why sharing /tf
        # with Isaac's ~60 Hz-per-robot odometry stream silently drops these.
        self.anchor_pub = self.create_publisher(TFMessage, "/map_anchors", ANCHOR_QOS)

        for rid in robot_ids:
            self.create_subscription(
                OccupancyGrid, f"/{rid}/map", self._make_on_map(rid), MAP_QOS
            )

        self.create_timer(cfg["publish_period_s"], self._tick)

    def _make_on_map(self, robot_id: str):
        def _on_map(msg: OccupancyGrid):
            info = GridInfo(
                resolution=msg.info.resolution,
                origin_x=msg.info.origin.position.x,
                origin_y=msg.info.origin.position.y,
                width=msg.info.width,
                height=msg.info.height,
            )
            data = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
            self._grids[robot_id] = (data, info)

        return _on_map

    def _lookup_raw_transform(self, robot_id: str) -> TransformStamped | None:
        try:
            return self.tf_buffer.lookup_transform("map", f"{robot_id}/map", rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return None

    def _tick(self):
        now = self.get_clock().now().to_msg()
        sources = []
        anchors = []
        fused_ids = []
        for rid, (data, info) in self._grids.items():
            raw = self._lookup_raw_transform(rid)
            if raw is None:
                continue

            rebroadcast = TransformStamped()
            rebroadcast.header.stamp = now
            rebroadcast.header.frame_id = "map"
            rebroadcast.child_frame_id = f"{rid}/map"
            rebroadcast.transform = raw.transform
            anchors.append(rebroadcast)

            t = raw.transform.translation
            q = raw.transform.rotation
            transform = Transform2D(tx=t.x, ty=t.y, yaw_rad=yaw_from_quaternion(q.x, q.y, q.z, q.w))
            sources.append((data, info, transform))
            fused_ids.append(rid)

        # Published on our OWN /map_anchors topic, deliberately NOT
        # rebroadcast onto the shared /tf: the dashboard's /tf subscription
        # throttles to 1 message/50ms (webui/app.js), and Isaac publishes
        # each robot's own odometry TF at ~60 Hz continuously — against that
        # traffic, a once-a-tick anchor message from here essentially never
        # wins the throttle window, so the browser would silently and
        # persistently drop it. Nav2/RTAB-Map/native tf2 tools never had this
        # problem (their subscriptions aren't rosbridge-relayed or throttled)
        # — this is purely for the one consumer that IS. TRANSIENT_LOCAL
        # (ANCHOR_QOS) still means a dashboard tab opened at any time gets
        # every robot's current anchor immediately, same guarantee the
        # original one-shot /tf_static anchors were SUPPOSED to give but
        # couldn't, since rosbridge_server doesn't replay historical
        # messages to a newly-subscribing websocket client either.
        if anchors:
            self.anchor_pub.publish(TFMessage(transforms=anchors))

        if not sources:
            return

        resolution = sources[0][1].resolution
        fused, out_info = fuse_grids(
            sources, resolution, self.cfg["occupied_thresh"], self._prev_out_info
        )
        # Log when the contributor set or output extent changes — silent
        # fusion is what let a robot_1-remapped-to-robot_0 camera bug look
        # like "the merge is fine" (one growing blob) for a whole run.
        # Extent growth is the dashboard's own tell that /map spans both
        # spawn anchors, so mirror it here for the bringup log.
        sig = (tuple(fused_ids), out_info.width, out_info.height)
        if sig != getattr(self, "_last_fuse_sig", None):
            free = int((fused == 0).sum())
            self.get_logger().info(
                f"fused {len(sources)} robot(s) [{', '.join(fused_ids)}] -> "
                f"{out_info.width}x{out_info.height} "
                f"origin=({out_info.origin_x:.2f},{out_info.origin_y:.2f}) "
                f"free={free}"
            )
            self._last_fuse_sig = sig
        self._prev_out_info = out_info
        self._publish(fused, out_info)

    def _publish(self, data: np.ndarray, info: GridInfo):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info = MapMetaData()
        msg.info.resolution = float(info.resolution)
        msg.info.width = int(info.width)
        msg.info.height = int(info.height)
        origin = Pose()
        origin.position.x = float(info.origin_x)
        origin.position.y = float(info.origin_y)
        msg.info.origin = origin
        msg.data = data.reshape(-1).tolist()
        self.map_pub.publish(msg)


def main() -> None:
    # This node re-publishes a TF edge (the map -> <rid>/map anchor, see
    # MapMergeNode._tick's rebroadcast) that other nodes compose across, so
    # its clock domain MUST agree with the rest of the sim-time-stamped tree
    # — unlike every other plain-ExecuteProcess node in this repo (explorer.py,
    # task_manager.py), which only ever READ TF and never publish a new edge,
    # so a wall-clock/sim-time mismatch never mattered for them. Without
    # use_sim_time, self.get_clock().now() here returns wall-clock time,
    # which — mixed into a chain where every other edge is sim-time-stamped —
    # broke tf2 composition for BOTH robots outright (every explorer logged
    # "no map->base_link TF yet" forever). launch/sortbots_bringup.launch.py
    # passes `--ros-args -p use_sim_time:=true` for exactly this reason;
    # rclpy.utilities.remove_ros_args strips that before OUR OWN argparse
    # sees it (rclpy.init() picks it up from sys.argv directly).
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--robot-ids",
        default="robot_0",
        help="Comma-separated robot ids to fuse, e.g. robot_0,robot_1.",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    robot_ids = [r.strip() for r in args.robot_ids.split(",") if r.strip()]
    cfg = load_config(args.config)

    rclpy.init(args=sys.argv)
    node = MapMergeNode(robot_ids, cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
