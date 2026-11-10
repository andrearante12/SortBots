#!/usr/bin/env python3
"""Hand one robot over from "localizing on a SHARED saved map" to its session's
mode: stay read-only, or (extend) switch RTAB-Map to mapping and start exploring.

Why this exists. A saved map's coordinates are those of the robot that built it
(the primary, robot_0): RTAB-Map's map frame starts wherever that robot's odom
started, i.e. its spawn. When every robot of a multi-robot run loads a copy of
that map (launch/sortbots_bringup.launch.py, shared_map:=true), each robot's
<rid>/map frame IS the primary's frame, so a secondary robot:

  1. starts somewhere other than the map's origin. The bringup passes RTAB-Map
     `initial_pose` = spawn(rid) - spawn(primary); without it RTAB-Map assumes the
     LAST pose saved in the database (2026-10-21: robot_1 came up 3.1 m and 16 deg
     off, at wherever the mapping session had ended).
  2. must not MAP before it is placed: in mapping mode an unplaced robot starts a
     new session whose nodes sit at the wrong offset, which fused into /map and
     /fleet/recon_cloud is exactly the doubled, rotated shelving seen 2026-10-21.
     So every robot starts in localization mode (Mem/IncrementalMemory=false,
     the only mode `initial_pose` applies in) and this node flips an extend
     session to mapping afterwards.

Two levels of "placed":
  - seeded:    SETTLE_SEC of RTAB-Map running on the seeded pose with no visual
               match. Good enough to DRIVE (a sim spawn is exact, and measured
               2026-10-21 robot_1 tracked truth within 6 cm across the warehouse),
               so exploring starts here. From a viewpoint the map never saw
               (robot_1's spawn, on a map robot_0 built) RTAB-Map rejects every
               hypothesis, so without this the robot could never move at all.
  - localized: an accepted visual match (loop_closure_id or proximity_detection_id
               on /<rid>/info). ONLY this switches an extend session to mapping:
               set_mode_mapping drops the seeded pose and restarts the robot at
               the map origin (2026-10-21: robot_1 jumped 8.6 m to robot_0's
               spawn) unless a match has tied it into the old graph. A seeded
               robot keeps driving read-only and switches the moment it
               recognises somewhere. Note `localization_pose` is NOT a signal: this stack runs with
pub_loc_pose_only_when_localizing=false, so it is published on every update,
localized or not.

Publishes /<rid>/map_handoff (std_msgs/String JSON)
{"state": "localizing" | "localized" | "seeded" | "mapping" | "failed", ...},
re-sent every few seconds since rosbridge subscribers are volatile.

    python3 nodes/map_handoff.py --robot-id robot_1 --then mapping --explore
"""
from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rtabmap_msgs.msg import Info
from std_msgs.msg import String
from std_srvs.srv import Empty

# Steady clock, not sim time: "is the stack up yet" waits, which under sim time
# would stretch by the real-time factor (CLAUDE.md, timers vs use_sim_time).
TICK_SEC = 2.0
# Measured from rtabmap's FIRST info message (i.e. once its database is loaded,
# which alone takes ~30 s for a 2.4 GB map), not from node start.
SETTLE_SEC = 15.0


class MapHandoff(Node):
    def __init__(self, robot_id: str, then: str, explore: bool, settle: float):
        super().__init__(f"{robot_id}_map_handoff")
        self.robot_id, self.then, self.explore, self.settle = robot_id, then, explore, settle
        self.state, self.detail = "localizing", ""
        self.first_info: float | None = None
        self.explore_sent = False
        # rtabmap's SERVICES live under its node, /<rid>/rtabmap/ (like publish_map,
        # see nodes/rtabmap_cloud_pump.py), but its topics are remapped up to
        # /<rid>/ (checked live with `ros2 node info /robot_1/rtabmap`, 2026-10-21).
        self.status_pub = self.create_publisher(String, f"/{robot_id}/map_handoff", 1)
        self.explore_pub = self.create_publisher(String, f"/{robot_id}/explore_cmd", 1)
        self.create_subscription(Info, f"/{robot_id}/info", self._on_info, 1)
        self.mapping_cli = self.create_client(Empty, f"/{robot_id}/rtabmap/set_mode_mapping")
        self.create_timer(TICK_SEC, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"{robot_id}: waiting to be placed on the shared map; then {then}"
                               f"{' + explore' if explore else ''}")

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps({"state": self.state, "then": self.then, "detail": self.detail})
        self.status_pub.publish(msg)

    def _on_info(self, info: Info) -> None:
        if self.first_info is None:
            self.first_info = time.monotonic()
        if self.state in ("localizing", "seeded") and (info.loop_closure_id > 0 or info.proximity_detection_id > 0):
            self._placed("localized", f"matched map node {info.loop_closure_id or info.proximity_detection_id}")

    def _tick(self) -> None:
        if (self.state == "localizing" and self.first_info is not None
                and time.monotonic() - self.first_info > self.settle):
            self.state = "seeded"
            self.detail = ("no visual match yet; driving read-only on the seeded pose"
                           + (", mapping from the first match" if self.then == "mapping" else ""))
            self.get_logger().info(f"{self.robot_id} seeded: {self.detail}")
            self._start_explore()
        self._publish_status()

    def _placed(self, how: str, detail: str) -> None:
        self.state, self.detail = how, detail
        self.get_logger().info(f"{self.robot_id} {how}: {detail}")
        if self.then == "mapping":
            if not self.mapping_cli.wait_for_service(timeout_sec=5.0):
                self.state, self.detail = "failed", "set_mode_mapping unavailable; left read-only"
                self.get_logger().error(f"{self.robot_id}: {self.detail}")
            else:
                self.mapping_cli.call_async(Empty.Request()).add_done_callback(self._on_mapping)
        else:
            self._start_explore()
        self._publish_status()

    def _on_mapping(self, _future) -> None:
        self.state = "mapping"
        self.get_logger().info(f"{self.robot_id} switched to mapping: extending the shared map")
        self._start_explore()
        self._publish_status()

    def _start_explore(self) -> None:
        # Once: a seeded robot already started exploring before its mapping switch.
        if self.explore and not self.explore_sent:
            self.explore_sent = True
            msg = String()
            msg.data = "start"
            self.explore_pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--then", choices=("localize", "mapping"), default="localize")
    parser.add_argument("--explore", action="store_true", help="send explore_cmd=start once placed")
    parser.add_argument("--settle", type=float, default=SETTLE_SEC)
    args, _ros = parser.parse_known_args()
    rclpy.init()
    node = MapHandoff(args.robot_id, args.then, args.explore, args.settle)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
