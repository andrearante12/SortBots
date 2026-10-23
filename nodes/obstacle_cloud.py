#!/usr/bin/env python3
"""Live obstacles from raw depth, independent of SLAM: /<id>/obstacles.

What is in front of the camera NOW, at camera rate, in base_link — for the
Nav2 local costmap and the dashboard's live layer. It reads raw
`camera/depth` and needs only the static camera mount, so it keeps working
when odometry is lost, when SLAM is busy, and when SLAM runs on another
machine (tasks/slam_offload.md). RTAB-Map's map is "where things were when
we passed by" and adds a node only every 5-10 cm of motion
(tasks/real_perception_plan.md L4); this is the other half.

Cheap on purpose (Orin Nano CPU budget): every `--stride`-th pixel, one
matrix multiply, two masks. 848x480 at stride 4 is ~25k points per frame.

    python3 nodes/obstacle_cloud.py --robot-id robot_0
"""
from __future__ import annotations

import argparse
import math

import numpy as np

DEFAULTS = {
    "stride": 4,            # 848x480 -> 212x120
    "min_range_m": 0.25,    # D435 min-z is ~0.2 m at 848x480
    "max_range_m": 4.0,     # D435 depth noise grows ~quadratically past ~4 m
    "min_height_m": 0.05,   # above the floor plane: floor isn't an obstacle
    "max_height_m": 1.5,    # same band as nav2_params.yaml obstacle_layer
}


def depth_to_metres(data: bytes, encoding: str, h: int, w: int) -> np.ndarray | None:
    if encoding in ("16UC1", "mono16"):
        return np.frombuffer(data, dtype=np.uint16).reshape(h, w).astype(np.float32) * 0.001
    if encoding == "32FC1":
        return np.frombuffer(data, dtype=np.float32).reshape(h, w)
    return None


def obstacle_points(depth_m: np.ndarray, K, T_base_opt: np.ndarray, *,
                    stride: int = DEFAULTS["stride"],
                    min_range_m: float = DEFAULTS["min_range_m"],
                    max_range_m: float = DEFAULTS["max_range_m"],
                    min_height_m: float = DEFAULTS["min_height_m"],
                    max_height_m: float = DEFAULTS["max_height_m"]) -> np.ndarray:
    """(N, 3) float32 points in base_link inside the obstacle band.

    K = (fx, fy, cx, cy) of the image depth is aligned to; T_base_opt is the
    4x4 base_link <- optical-frame transform (the camera mount)."""
    fx, fy, cx, cy = K
    z = depth_m[::stride, ::stride]
    v, u = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    ok = np.isfinite(z) & (z >= min_range_m) & (z <= max_range_m)
    z, u, v = z[ok], u[ok].astype(np.float32), v[ok].astype(np.float32)
    pts = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)
    pts = pts @ T_base_opt[:3, :3].T.astype(np.float32) + T_base_opt[:3, 3].astype(np.float32)
    band = (pts[:, 2] >= min_height_m) & (pts[:, 2] <= max_height_m)
    return pts[band].astype(np.float32)


def quat_matrix(x, y, z, w, t) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = t
    return m


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot-id", default="robot_0")
    for k, v in DEFAULTS.items():
        ap.add_argument("--" + k.replace("_", "-"), type=type(v), default=v)
    a = ap.parse_args(argv)
    band = {k: getattr(a, k) for k in DEFAULTS}

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
    from tf2_ros import Buffer, TransformListener

    rid = a.robot_id

    class ObstacleCloud(Node):
        def __init__(self):
            super().__init__(f"{rid}_obstacle_cloud")
            self.K = None
            self.T = None
            self.tf = Buffer()
            self._tfl = TransformListener(self.tf, self)
            self.pub = self.create_publisher(PointCloud2, f"/{rid}/obstacles", qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f"/{rid}/camera/camera_info",
                                     self.on_info, qos_profile_sensor_data)
            self.create_subscription(Image, f"/{rid}/camera/depth", self.on_depth,
                                     qos_profile_sensor_data)

        def on_info(self, m):
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

        def on_depth(self, m):
            if self.K is None:
                return
            if self.T is None:
                # The mount is static: look it up once, then never touch TF
                # again (a per-frame lookup is the expensive part in rclpy).
                try:
                    tr = self.tf.lookup_transform(f"{rid}/base_link", m.header.frame_id,
                                                  rclpy.time.Time())
                except Exception:
                    return
                r, t = tr.transform.rotation, tr.transform.translation
                self.T = quat_matrix(r.x, r.y, r.z, r.w, (t.x, t.y, t.z))
                self._drop_tf()
            d = depth_to_metres(m.data, m.encoding, m.height, m.width)
            if d is None:
                return
            pts = obstacle_points(d, self.K, self.T, **band)
            out = PointCloud2()
            out.header.stamp = m.header.stamp      # the depth frame's time, so
            out.header.frame_id = f"{rid}/base_link"  # latency is measurable
            out.height, out.width = 1, len(pts)
            out.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                          for i, n in enumerate("xyz")]
            out.is_bigendian, out.is_dense = False, True
            out.point_step, out.row_step = 12, 12 * len(pts)
            out.data = pts.tobytes()
            self.pub.publish(out)

        def _drop_tf(self):
            self._tfl.unregister()
            self._tfl = None

    rclpy.init()
    n = ObstacleCloud()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
