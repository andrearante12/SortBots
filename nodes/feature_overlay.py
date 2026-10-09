#!/usr/bin/env python3
"""Republish RTAB-Map's visual-odometry keypoints as small JSON for the dashboard.

`rgbd_odometry` already publishes every feature it found in the current frame
on `/<id>/odom_info` (rtabmap_msgs/OdomInfo), tagged by word id as matched
and/or inlier against the local map. That message is large (local map points,
covariance, ...), so the browser never subscribes to it directly: this node
reduces it to `/<id>/odom_features`, a `std_msgs/String` JSON the camera tile
draws as a canvas overlay. No second image stream goes through web_video_server
(which livelocks on long-lived streams — see webui/app.js).

Real platform only: in sim Isaac publishes odom, `rgbd_odometry` never runs and
there is no `odom_info`.

    python3 nodes/feature_overlay.py --robot-id robot_0
"""
from __future__ import annotations

import argparse
import json

DEFAULTS = {
    "max_points": 400,   # keep each message to a few KB
}

DETECTED, MATCHED, INLIER = 0, 1, 2


def build_payload(keys, kps, matches, inliers, *, lost, features, n_matches,
                  n_inliers, stamp, width, height,
                  max_points=DEFAULTS["max_points"]):
    """Reduce one OdomInfo to the overlay payload (pure; no ROS types).

    `keys` are word ids, `kps` the parallel (x, y, size) tuples, `matches` and
    `inliers` the word ids tracked against the local map. When there are more
    keypoints than `max_points`, inliers are kept first and then matches, so
    the cap never hides the points that are actually driving the pose.
    """
    inl = set(inliers)
    mat = set(matches)
    pts = []
    for k, (x, y, size) in zip(keys, kps):
        state = INLIER if k in inl else MATCHED if k in mat else DETECTED
        pts.append([int(round(x)), int(round(y)), int(round(size)), state])
    if len(pts) > max_points:
        pts.sort(key=lambda p: -p[3])
        pts = pts[:max_points]
    return {
        "stamp": float(stamp),
        "w": int(width),
        "h": int(height),
        "lost": bool(lost),
        "features": int(features),
        "matches": int(n_matches),
        "inliers": int(n_inliers),
        "pts": pts,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument("--max-points", type=int, default=DEFAULTS["max_points"])
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import String
    from rtabmap_msgs.msg import OdomInfo

    robot_id = args.robot_id

    class FeatureOverlay(Node):
        def __init__(self):
            super().__init__(f"{robot_id}_feature_overlay")
            self._wh = (0, 0)
            # odom_info is published RELIABLE; a depth-1 queue keeps a slow
            # consumer from ever lagging behind the newest frame.
            self._qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            self._sub = None
            self.create_subscription(
                CameraInfo, f"/{robot_id}/camera/camera_info", self._on_cam,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.pub = self.create_publisher(
                String, f"/{robot_id}/odom_features", 1)
            # rclpy deserialises the WHOLE OdomInfo (local-map points and all)
            # before our callback runs — measured ~35% of a core on the Orin
            # Nano at ~3 Hz, on a board whose CPU is already saturated by
            # RTAB-Map. So only hold the odom_info subscription while someone
            # is subscribed to odom_features (the dashboard does so only while
            # its sensors view is on screen).
            self.create_timer(1.0, self._follow_demand)
            self.get_logger().info(
                f"feature overlay: /{robot_id}/odom_info -> /{robot_id}/odom_features "
                f"(on demand)")

        def _follow_demand(self):
            wanted = self.pub.get_subscription_count() > 0
            if wanted and self._sub is None:
                self._sub = self.create_subscription(
                    OdomInfo, f"/{robot_id}/odom_info", self._on_info, self._qos)
            elif not wanted and self._sub is not None:
                self.destroy_subscription(self._sub)
                self._sub = None

        def _on_cam(self, msg):
            self._wh = (int(msg.width), int(msg.height))

        def _on_info(self, msg):
            kps = [(v.pt.x, v.pt.y, v.size) for v in msg.words_values]
            payload = build_payload(
                list(msg.words_keys), kps,
                list(msg.word_matches), list(msg.word_inliers),
                lost=msg.lost, features=msg.features,
                n_matches=msg.matches, n_inliers=msg.inliers,
                stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                width=self._wh[0], height=self._wh[1],
                max_points=args.max_points)
            out = String()
            out.data = json.dumps(payload, separators=(",", ":"))
            self.pub.publish(out)

    rclpy.init()
    node = FeatureOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
