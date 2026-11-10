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
    def __init__(self, robot_id: str, period: float = PUMP_PERIOD_SEC):
        super().__init__("rtabmap_cloud_pump")
        self.cli = self.create_client(PublishMap, f"/{robot_id}/rtabmap/publish_map")
        self.get_logger().info("waiting for RTAB-Map's publish_map service...")
        self.cli.wait_for_service()
        self.get_logger().info("connected — pumping the map every %.1fs" % period)
        self._inflight = False
        # Only the FIRST call loads node data (graph_only=False); every later
        # one is graph_only=True. Measured offline 2026-10-15 against a
        # 1,059-node Grid/3D database: each graph_only=False call re-reads
        # every node's images, scans and grids out of sqlite, and rtabmap's
        # RSS climbs ~1.3 GB per call and never comes back (mostly glibc
        # holding the freed heap, ~0.2 GB a genuine leak). At one call per
        # 3 s that was ~20 GB in two minutes — the live run that OOM'd the
        # desktop. graph_only=True is flat. It still assembles the full
        # cloud because MapsManager keeps every local grid it has seen, and
        # nodes mapped live go into that cache as they're created; it just
        # can't fill the cache cold (nothing published), hence one full load
        # first — the nodes a --resume/--localize run inherits from the DB.
        self._primed = False
        self.timer = self.create_timer(period, self._pump)

    def _pump(self):
        # A global assemble can outlast the period on a loaded board; firing
        # another on top of it just queues redundant full-map rebuilds in
        # rtabmap and makes the stall worse.
        if self._inflight:
            return
        self._inflight = True
        req = PublishMap.Request()
        req.global_map = True
        req.optimized = True
        req.graph_only = self._primed
        future = self.cli.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        self._inflight = False
        exc = future.exception()
        if exc is not None:
            # Re-prime: a failed call may mean rtabmap restarted with an
            # empty cache.
            self._primed = False
            self.get_logger().warn(f"publish_map call failed: {exc}")
        else:
            if not self._primed:
                self.get_logger().info("publish_map: map cache primed, graph-only from here")
            self._primed = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="robot_0")
    parser.add_argument("--period", type=float, default=PUMP_PERIOD_SEC)
    args = parser.parse_args()

    rclpy.init()
    node = RtabmapCloudPump(args.robot_id, args.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
