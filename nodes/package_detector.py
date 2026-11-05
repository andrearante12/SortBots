#!/usr/bin/env python3
"""Magnet plates in front of the robot, in the map frame: /<id>/packages.

What the pick controller (nodes/magnet_pick.py) aims at. Reads the head camera's RGB
and aligned depth, finds every package's plate (nodes/package_vision.py explains how
and what it cannot do), moves each to the map frame through TF and publishes them as
JSON on `std_msgs/String`, the same way every other node in this repo talks:

    /<id>/packages   {stamp, frame: "map", plates: [{x, y, z, nx, ny, nz, conf, age_s, px}]}

`x, y, z` is the plate's centre and `nx, ny, nz` its OUTWARD unit normal. `age_s` is
how long ago the camera last actually saw it (0 = this frame): a plate that has left
the view is remembered for `track.drop_after_s` sim-seconds, so the controller can keep
docking on it while the robot turns. `px` is the plate's bounding box in the image,
for an overlay.

No image is ever published (nothing here can starve Isaac's image writers), and every
image subscription is sensor-data QoS: a RELIABLE image subscriber kills Isaac's image
writers at around sim-t=100-140 s (CLAUDE.md).

Frames: the image's own `frame_id` is the optical frame (`<id>/camera_optical`), looked up
to `map` in two halves (Detector.map_from), and it needs only the usual SLAM and
static-mount TF. In the sim that TF follows the head (simtest/head.py), so any pan and
tilt work; on the real robot it is still a fixed mount.

    python3 nodes/package_detector.py --robot-id robot_0 [--platform real]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

import box_geometry  # noqa: E402
import package_vision as pv  # noqa: E402
from obstacle_cloud import quat_matrix  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "configs" / "package_detector.yaml"


def load_config(path: Path, platform: str) -> dict:
    """The YAML with its `sim:` or `real:` block laid over the top-level keys."""
    raw = yaml.safe_load(Path(path).read_text()) or {}
    cfg = {k: v for k, v in raw.items() if k not in ("sim", "real")}
    for k, v in (raw.get(platform) or {}).items():
        cfg[k] = v
    return cfg


def rgb_from_image(data: bytes, encoding: str, h: int, w: int) -> np.ndarray | None:
    """An Image message's bytes as RGB uint8 (H, W, 3), or None for an encoding we
    don't read."""
    chans = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if chans is None:
        return None
    img = np.frombuffer(data, dtype=np.uint8).reshape(h, w, chans)
    if encoding.startswith("bgr"):
        img = img[..., [2, 1, 0]]
    return np.ascontiguousarray(img[..., :3])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--robot-id", default="robot_0")
    ap.add_argument("--platform", default=os.environ.get("SORTBOTS_PLATFORM", "sim"),
                    choices=("sim", "real"))
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    a = ap.parse_args(argv)
    cfg = load_config(a.config, a.platform)

    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    rid = a.robot_id
    track = cfg.get("track", {})

    class Detector(Node):
        def __init__(self):
            super().__init__(f"{rid}_package_detector")
            self.K = None
            self.rgb = None          # (stamp_s, ndarray)
            self.depth = None        # (stamp_s, ndarray, frame_id)
            self.tracker = pv.Tracker(track.get("assoc_m", 0.15), track.get("ema", 0.4),
                                      track.get("drop_after_s", 2.0))
            self.tf = Buffer()
            self._tfl = TransformListener(self.tf, self)
            self.pub = self.create_publisher(String, f"/{rid}/packages", 10)
            self.boxes_pub = self.create_publisher(String, f"/{rid}/boxes", 10)
            self.manipulating = False
            self.create_subscription(String, f"/{rid}/pick_status", self.on_status, 10)
            self.create_subscription(CameraInfo, f"/{rid}/camera/camera_info",
                                     self.on_info, qos_profile_sensor_data)
            self.create_subscription(Image, f"/{rid}/camera/rgb", self.on_rgb,
                                     qos_profile_sensor_data)
            self.create_subscription(Image, f"/{rid}/camera/depth", self.on_depth,
                                     qos_profile_sensor_data)
            # Steady clock, not the node's: with use_sim_time a node timer's period
            # stretches by the real-time factor (CLAUDE.md), and this is a responsiveness
            # rate, not a deadline that stands for distance.
            self.create_timer(1.0 / float(cfg.get("rate_hz", 10.0)), self.step,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))

        def on_status(self, m):
            try:
                self.manipulating = json.loads(m.data).get("mode") == "manipulation"
            except ValueError:
                pass

        def on_info(self, m):
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

        def on_rgb(self, m):
            img = rgb_from_image(bytes(m.data), m.encoding, m.height, m.width)
            if img is not None:
                self.rgb = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, img)

        def on_depth(self, m):
            d = pv.depth_to_metres(bytes(m.data), m.encoding, m.height, m.width)
            if d is not None:
                self.depth = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, d,
                              m.header.frame_id)

        def map_from(self, frame):
            """map <- camera, as fresh as it can be had.

            Asking for map <- camera in one lookup gets the latest time the WHOLE chain
            covers, which is as old as SLAM's slow map -> odom: moving at 0.3 m/s that put
            the plate 8 cm behind where it was (2026-11-05). So the halves are taken apart.
            The fast one is NOT looked up at the image's stamp: that made it worse, 15 cm,
            the same direction, so the stamp itself runs behind the picture."""
            def matrix(tr):
                r, t = tr.transform.rotation, tr.transform.translation
                return quat_matrix(r.x, r.y, r.z, r.w, (t.x, t.y, t.z))

            odom = f"{rid}/odom"
            try:
                return (matrix(self.tf.lookup_transform("map", odom, rclpy.time.Time()))
                        @ matrix(self.tf.lookup_transform(odom, frame, rclpy.time.Time())))
            except Exception:  # noqa: BLE001 - a chain with no odom frame of this name
                try:
                    return matrix(self.tf.lookup_transform("map", frame, rclpy.time.Time()))
                except Exception:  # noqa: BLE001
                    return None

        def step(self):
            if self.K is None or self.rgb is None or self.depth is None:
                return
            (t_rgb, rgb), (t_d, depth, frame) = self.rgb, self.depth
            if abs(t_rgb - t_d) > 0.15 or rgb.shape[:2] != depth.shape:
                return           # not the same instant, or not the same size: wait
            T = self.map_from(frame)
            if T is None:
                return           # no map yet: try the next frame
            if self.manipulating:       # geometry is for when a pick has actually started
                self.boxes_pub.publish(String(data=json.dumps({
                    "stamp": round(max(t_rgb, t_d), 3), "frame": "map",
                    "w": int(rgb.shape[1]), "h": int(rgb.shape[0]),
                    "boxes": box_geometry.find_boxes(depth, self.K, T, cfg.get("boxes"))})))
            plates = [pv.to_map(p, T) for p in pv.detect_plates(rgb, depth, self.K, cfg)]
            out = self.tracker.update(plates, max(t_rgb, t_d))
            self.pub.publish(String(data=json.dumps({
                "stamp": round(max(t_rgb, t_d), 3), "frame": "map",
                "w": int(rgb.shape[1]), "h": int(rgb.shape[0]),
                "plates": [{k: (round(v, 4) if isinstance(v, float) else v)
                            for k, v in p.items() if k not in ("last", "hits")} for p in out]})))

    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
