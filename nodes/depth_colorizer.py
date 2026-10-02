#!/usr/bin/env python3
"""Colourise head depth for the dashboard, like the RealSense viewer does.

`/<id>/camera/depth` is 16UC1 millimetres (real) or 32FC1 metres (sim), which
a browser can't show — web_video_server would hand back a near-black image.
This node publishes `/<id>/camera/depth_color` (bgr8, JET colormap, near = red,
far = blue, no-return = black), decimated and rate-limited so it stays cheap on
a board whose CPU is already saturated by RTAB-Map.

    python3 nodes/depth_colorizer.py --robot-id robot_0
"""
from __future__ import annotations

import argparse

import numpy as np

DEFAULTS = {
    "min_m": 0.3,        # D435 minimum useful range
    "max_m": 4.5,        # matches dynamic_obstacle_filter max_range_m
    "decimate": 2,       # stride before colouring: 4x fewer pixels
    "rate_hz": 5.0,      # the tile polls at ~2.5 Hz; no point colouring 30 fps
}


def colorize_depth(depth_m: np.ndarray, *, min_m=DEFAULTS["min_m"],
                   max_m=DEFAULTS["max_m"], decimate=DEFAULTS["decimate"]):
    """float32 metres (HxW) -> uint8 BGR (H/d x W/d x 3). Invalid -> black.

    Near maps to red (colormap high end) the way the RealSense viewer shows it.
    """
    import cv2

    d = depth_m[::decimate, ::decimate] if decimate > 1 else depth_m
    valid = np.isfinite(d) & (d >= min_m) & (d <= max_m)
    norm = np.zeros(d.shape, dtype=np.uint8)
    scaled = (max_m - d[valid]) / (max_m - min_m)
    norm[valid] = np.clip(scaled * 254.0, 0, 254).astype(np.uint8) + 1
    img = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    img[~valid] = 0
    return img


def depth_to_metres(msg_data, encoding: str, h: int, w: int):
    """Raw Image payload -> float32 metres, or None for an unsupported encoding."""
    if encoding == "32FC1":
        return np.frombuffer(msg_data, dtype=np.float32).reshape(h, w)
    if encoding in ("16UC1", "mono16"):
        raw = np.frombuffer(msg_data, dtype=np.uint16).reshape(h, w)
        return raw.astype(np.float32) * 0.001
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument("--rate-hz", type=float, default=DEFAULTS["rate_hz"])
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    robot_id = args.robot_id

    class DepthColorizer(Node):
        def __init__(self):
            super().__init__(f"{robot_id}_depth_colorizer")
            # Steady clock, not the node clock: with use_sim_time a node timer
            # stretches by the real-time factor (see nodes/explorer.py), and
            # this is a wall-clock CPU limiter.
            self._min_period = 1.0 / max(args.rate_hz, 0.1)
            self._last = 0.0
            self.create_subscription(
                Image, f"/{robot_id}/camera/depth", self._on_depth,
                qos_profile_sensor_data)
            self.pub = self.create_publisher(
                Image, f"/{robot_id}/camera/depth_color", qos_profile_sensor_data)
            self.get_logger().info(
                f"depth colorizer: /{robot_id}/camera/depth -> depth_color "
                f"@ <= {args.rate_hz:g} Hz")

        def _on_depth(self, msg):
            import time
            now = time.monotonic()
            if now - self._last < self._min_period:
                return
            # No "skip when nobody subscribes" gate: web_video_server's
            # /snapshot subscribes per request, so the count is 0 between polls
            # and a gate would starve the tile. Colouring a decimated frame at
            # <= 5 Hz is cheap enough to run unconditionally.
            depth = depth_to_metres(msg.data, msg.encoding, msg.height, msg.width)
            if depth is None:
                return
            self._last = now
            img = colorize_depth(depth)
            out = Image()
            out.header = msg.header
            out.height, out.width = img.shape[:2]
            out.encoding = "bgr8"
            out.is_bigendian = 0
            out.step = out.width * 3
            out.data = img.tobytes()
            self.pub.publish(out)

    rclpy.init()
    node = DepthColorizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
