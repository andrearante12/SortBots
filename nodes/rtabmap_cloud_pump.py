#!/usr/bin/env python3
"""Periodic nudge that keeps RTAB-Map's assembled-map topics flowing.

`rtabmap_ros`'s `cloud_map` / `cloud_ground` / `cloud_obstacles` / `octomap_*`
publishers are lazy: `MapsManager` only (re)computes and publishes them when
it decides a subscriber is present. In this setup that subscriber-detection
never fires on its own once the dashboard's rosbridge (rclpy) subscription is
live — confirmed by leaving a subscriber attached across a full RTAB-Map
restart and 25+ seconds of active mapping with zero publishes. The one thing
that *does* reliably force a fresh publish is calling the node's own
`publish_map` service, so this node just calls it on a timer.

Run (system ROS 2 Jazzy sourced, NOT the Isaac venv or conda):

    source /opt/ros/jazzy/setup.bash
    python3 nodes/rtabmap_cloud_pump.py --robot-id robot_0
"""
from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from rtabmap_msgs.srv import PublishMap

PUMP_PERIOD_SEC = 3.0


class RtabmapCloudPump(Node):
    def __init__(self, robot_id: str):
        super().__init__("rtabmap_cloud_pump")
        self.cli = self.create_client(PublishMap, f"/{robot_id}/rtabmap/publish_map")
        self.get_logger().info("waiting for RTAB-Map's publish_map service...")
        self.cli.wait_for_service()
        self.get_logger().info("connected — pumping the map every %.1fs" % PUMP_PERIOD_SEC)
        self.timer = self.create_timer(PUMP_PERIOD_SEC, self._pump)

    def _pump(self):
        req = PublishMap.Request()
        req.global_map = True
        req.optimized = True
        req.graph_only = False
        future = self.cli.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        exc = future.exception()
        if exc is not None:
            self.get_logger().warn(f"publish_map call failed: {exc}")
        else:
            self.get_logger().info("publish_map call completed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_0")
    args = parser.parse_args()

    rclpy.init()
    node = RtabmapCloudPump(args.robot_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
