#!/usr/bin/env python3
"""Replay a TUM RGB-D sequence as the robot's D435 topics.

Publishes what launch/sortbots_realsense.launch.py publishes on hardware:
`/<id>/camera/{rgb,depth,camera_info}` (rgb8, 16UC1 mm depth aligned to
color, one camera_info) plus the static TF base_link -> optical frame, so the
real-robot SLAM launch runs on it unchanged.

Stamps are wall-clock "now" at publish time, as on the Jetson (the V4L2
driver stamps with system time, docs/jetson.md). Each published stamp is
written next to its dataset timestamp in --stamps, so poses can be scored
against the sequence's ground truth afterwards (score.py).

    python3 tum_player.py --seq ~/datasets/tum/rgbd_dataset_freiburg2_pioneer_360 \
        --stamps /out/stamps.csv
"""
import argparse
import bisect
import csv
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# Freiburg 2 Kinect, from the TUM dataset page. Depth is registered to RGB.
FR2_K = (520.908620, 521.007327, 325.141442, 249.701764)
FR1_K = (517.306408, 516.469215, 318.643040, 255.313989)
FR3_K = (535.4, 539.2, 320.1, 247.6)


def read_list(path: Path) -> list[tuple[float, str]]:
    out = []
    for line in path.read_text().splitlines():
        if line and not line.startswith("#"):
            t, f = line.split()[:2]
            out.append((float(t), f))
    return out


def associate(rgb, depth, max_dt=0.02):
    """Nearest depth for each rgb frame, as the TUM associate.py tool does."""
    dts = [t for t, _ in depth]
    pairs = []
    for t, f in rgb:
        i = bisect.bisect_left(dts, t)
        best = min((j for j in (i - 1, i) if 0 <= j < len(dts)), key=lambda j: abs(dts[j] - t))
        if abs(dts[best] - t) <= max_dt:
            pairs.append((t, f, depth[best][1]))
    return pairs


class Player(Node):
    def __init__(self, a):
        super().__init__("tum_player")
        self.a = a
        seq = Path(a.seq).expanduser()
        self.seq = seq
        self.pairs = associate(read_list(seq / "rgb.txt"), read_list(seq / "depth.txt"))
        name = seq.name
        self.K = FR1_K if "freiburg1" in name else FR3_K if "freiburg3" in name else FR2_K
        base = f"/{a.robot_id}/camera"
        self.frame = f"{a.robot_id}/camera_color_optical_frame"
        self.p_rgb = self.create_publisher(Image, f"{base}/rgb", qos_profile_sensor_data)
        self.p_depth = self.create_publisher(Image, f"{base}/depth", qos_profile_sensor_data)
        self.p_info = self.create_publisher(CameraInfo, f"{base}/camera_info", qos_profile_sensor_data)
        # base_link -> optical. TUM's Kinect sits level on the Pioneer, so
        # base_link is the camera body frame dropped to the floor: optical
        # axes (z fwd, x right, y down) rotated into base axes (x fwd, y left, z up).
        tf = TransformStamped()
        tf.header.frame_id = f"{a.robot_id}/base_link"
        tf.child_frame_id = self.frame
        tf.transform.translation.z = a.cam_height
        q = euler_to_quat(-math.pi / 2, 0.0, -math.pi / 2)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        self.tfb = StaticTransformBroadcaster(self)
        self.tfb.sendTransform(tf)
        self.out = open(a.stamps, "w", newline="")
        self.w = csv.writer(self.out)
        self.w.writerow(["ros_stamp", "dataset_stamp"])

    def run(self):
        a = self.a
        time.sleep(a.lead_in)
        t0_data = self.pairs[0][0]
        t0_wall = time.monotonic()
        for t, frgb, fdepth in self.pairs:
            target = t0_wall + (t - t0_data) / a.rate
            delay = target - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            rgb = cv2.cvtColor(cv2.imread(str(self.seq / frgb), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
            d = cv2.imread(str(self.seq / fdepth), cv2.IMREAD_UNCHANGED)  # uint16, 5000 per metre
            d_mm = (d.astype(np.uint32) // 5).astype(np.uint16)          # -> D435's 16UC1 mm
            stamp = self.get_clock().now().to_msg()
            self.p_rgb.publish(image(rgb, "rgb8", stamp, self.frame))
            self.p_depth.publish(image(d_mm, "16UC1", stamp, self.frame))
            self.p_info.publish(self.info(stamp, rgb.shape))
            self.w.writerow([f"{stamp.sec}.{stamp.nanosec:09d}", f"{t:.6f}"])
            rclpy.spin_once(self, timeout_sec=0.0)
        self.out.close()


    def info(self, stamp, shape):
        fx, fy, cx, cy = self.K
        m = CameraInfo()
        m.header.stamp, m.header.frame_id = stamp, self.frame
        m.height, m.width = shape[0], shape[1]
        m.distortion_model = "plumb_bob"
        m.d = [0.0] * 5
        m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m


def image(arr, enc, stamp, frame):
    m = Image()
    m.header.stamp, m.header.frame_id = stamp, frame
    m.height, m.width = arr.shape[0], arr.shape[1]
    m.encoding = enc
    m.step = arr.strides[0]
    m.data = arr.tobytes()
    return m


def euler_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq", required=True)
    ap.add_argument("--stamps", required=True)
    ap.add_argument("--robot-id", default="robot_0")
    ap.add_argument("--rate", type=float, default=1.0, help="playback speed factor")
    ap.add_argument("--cam-height", type=float, default=0.4)
    ap.add_argument("--lead-in", type=float, default=8.0, help="s to wait for subscribers")
    a = ap.parse_args()
    rclpy.init()
    p = Player(a)
    p.run()
    p.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
