#!/usr/bin/env python3
"""Log /<id>/odom poses and rtabmap's per-frame OdomInfo to CSV.

odom.csv: stamp, x y z qx qy qz qw, lost (covariance >= 9999 is RTAB-Map's
"lost" marker, as tasks/real_perception_plan.md L2 found the hard way).
obstacles.csv: depth stamp, latency to here, point count.
keyframes.csv: compressed RGB-D keyframe size (the offload link's load).
odom_info.csv: stamp, lost, inliers, features, update time (s) — the
Jetson's own numbers in docs/jetson.md come from the same fields.

    python3 odom_logger.py --out /out
"""
import argparse
import csv
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rtabmap_msgs.msg import OdomInfo
from rtabmap_msgs.msg import RGBDImage
from sensor_msgs.msg import PointCloud2


class Logger(Node):
    def __init__(self, a):
        super().__init__("odom_logger")
        out = Path(a.out)
        self.fo = open(out / "odom.csv", "w", newline="")
        self.wo = csv.writer(self.fo)
        self.wo.writerow(["stamp", "x", "y", "z", "qx", "qy", "qz", "qw", "lost"])
        self.fi = open(out / "odom_info.csv", "w", newline="")
        self.wi = csv.writer(self.fi)
        self.wi.writerow(["stamp", "lost", "inliers", "features", "update_s"])
        # Obstacle latency: depth frame stamp -> cloud received here. Same host
        # clock on both ends, so it is the real end-to-end delay.
        self.fb = open(out / "obstacles.csv", "w", newline="")
        self.wb = csv.writer(self.fb)
        self.wb.writerow(["stamp", "latency_s", "points"])
        self.create_subscription(PointCloud2, f"/{a.robot_id}/obstacles", self.on_obst,
                                 qos_profile_sensor_data)
        # Offload link: bytes per compressed keyframe the robot would send.
        self.fk = open(out / "keyframes.csv", "w", newline="")
        self.wk = csv.writer(self.fk)
        self.wk.writerow(["stamp", "bytes"])
        self.create_subscription(RGBDImage, f"/{a.robot_id}/rgbd_image/compressed", self.on_kf,
                                 QoSProfile(depth=10))
        self.create_subscription(Odometry, f"/{a.robot_id}/odom", self.on_odom, QoSProfile(depth=100))
        self.create_subscription(OdomInfo, f"/{a.robot_id}/odom_info", self.on_info, QoSProfile(depth=100))

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        lost = int(m.pose.covariance[0] >= 9999)
        self.wo.writerow([f"{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}",
                          p.x, p.y, p.z, q.x, q.y, q.z, q.w, lost])
        self.fo.flush()

    def on_obst(self, m):
        now = self.get_clock().now().nanoseconds
        st = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        self.wb.writerow([f"{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}",
                          f"{(now - st) / 1e9:.4f}", m.width])
        self.fb.flush()

    def on_kf(self, m):
        self.wk.writerow([f"{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}",
                          len(m.rgb_compressed.data) + len(m.depth_compressed.data)])
        self.fk.flush()

    def on_info(self, m):
        self.wi.writerow([f"{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}",
                          int(m.lost), m.inliers, m.features, f"{m.time_estimation:.4f}"])
        self.fi.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--robot-id", default="robot_0")
    a = ap.parse_args()
    rclpy.init()
    n = Logger(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
