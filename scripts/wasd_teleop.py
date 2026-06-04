#!/usr/bin/env python3
"""WASD keyboard teleop for the SortBots warehouse demo.

A drop-in, friendlier replacement for `teleop_twist_keyboard` (whose
default bindings are the awkward i/j/l/, cluster). Publishes
`geometry_msgs/Twist` to `/<robot_id>/cmd_vel`, which `spawn_warehouse.py`
reads off the OG SubscribeTwist node each tick.

Run under SYSTEM ROS 2 Jazzy (NOT the Isaac venv — it needs system
`rclpy`, and `activate_isaac.sh` refuses a ROS-sourced shell anyway):

    source /opt/ros/jazzy/setup.bash
    python3 scripts/wasd_teleop.py --robot-id robot_0

Keys (set-and-continue, like teleop_twist_keyboard — the robot keeps the
last command until you change it or hit space):

    w / s : forward / backward      (linear.x)
    a / d : turn left / right       (angular.z)
    q / e : strafe left / right     (linear.y — the XLeRobot base is holonomic)
    space : stop
    z / x : slower / faster (scales both linear and angular)
    k     : quit (sends a stop first)

The XLeRobot DOF mapping matches `spawn_warehouse._drive_velocities`:
linear.x -> root_x_axis_joint, linear.y -> root_y_axis_joint,
angular.z -> root_z_rotation_joint.
"""
from __future__ import annotations

import argparse
import os
import sys
import termios
import tty
from select import select

# ROS 2 Jazzy's rclpy lives under /usr/bin/python3 (with its PyYAML dep); a
# conda/pyenv python3 on PATH will fail to import it. Transparently re-exec
# under the system interpreter so `python3 scripts/wasd_teleop.py` just works.
if os.environ.get("_WASD_REEXEC") != "1":
    try:
        import rclpy  # noqa: F401
    except ModuleNotFoundError:
        os.environ["_WASD_REEXEC"] = "1"
        os.execv("/usr/bin/python3", ["/usr/bin/python3", *sys.argv])

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = __doc__.split("Keys", 1)[1]

# key -> (dvx, dvy, dwz) in units of the current speed/turn scale.
MOVE_BINDINGS = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 0.0, 1.0),
    "d": (0.0, 0.0, -1.0),
    "q": (0.0, 1.0, 0.0),
    "e": (0.0, -1.0, 0.0),
}


class WasdTeleop(Node):
    def __init__(self, cmd_vel_topic: str, speed: float, turn: float, rate_hz: float):
        super().__init__("wasd_teleop")
        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.speed = speed
        self.turn = turn
        self.vx = self.vy = self.wz = 0.0
        # Republish continuously so the OG SubscribeTwist always sees a fresh
        # value (and so a stop actually zeros the held command).
        self.create_timer(1.0 / rate_hz, self._publish)

    def _publish(self) -> None:
        t = Twist()
        t.linear.x = self.vx * self.speed
        t.linear.y = self.vy * self.speed
        t.angular.z = self.wz * self.turn
        self.pub.publish(t)

    def apply_key(self, key: str) -> bool:
        """Update the target twist. Returns False to request quit."""
        if key == "k" or key == "\x03":  # k or Ctrl-C
            self.vx = self.vy = self.wz = 0.0
            return False
        if key == " ":
            self.vx = self.vy = self.wz = 0.0
        elif key in MOVE_BINDINGS:
            self.vx, self.vy, self.wz = MOVE_BINDINGS[key]
        elif key == "z":
            self.speed *= 0.9
            self.turn *= 0.9
        elif key == "x":
            self.speed *= 1.1
            self.turn *= 1.1
        else:
            return True
        self.get_logger().info(
            f"vx={self.vx * self.speed:+.2f} vy={self.vy * self.speed:+.2f} "
            f"wz={self.wz * self.turn:+.2f}  (speed={self.speed:.2f} turn={self.turn:.2f})"
        )
        return True


def _read_key(timeout: float) -> str | None:
    """Non-blocking single-char read from stdin (terminal in raw mode)."""
    if select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


def main() -> int:
    p = argparse.ArgumentParser(description="WASD teleop for SortBots robots.")
    p.add_argument("--robot-id", default="robot_0", help="Robot namespace (default robot_0).")
    p.add_argument("--cmd-vel-topic", default=None, help="Override the full cmd_vel topic.")
    p.add_argument("--speed", type=float, default=0.3, help="Linear speed m/s (default 0.3).")
    p.add_argument("--turn", type=float, default=0.6, help="Angular speed rad/s (default 0.6).")
    p.add_argument("--rate", type=float, default=20.0, help="Publish rate Hz (default 20).")
    args = p.parse_args()

    topic = args.cmd_vel_topic or f"/{args.robot_id}/cmd_vel"

    if not sys.stdin.isatty():
        print("ERROR: wasd_teleop needs an interactive terminal (a TTY).", file=sys.stderr)
        return 2

    rclpy.init()
    node = WasdTeleop(topic, args.speed, args.turn, args.rate)
    print(f"WASD teleop -> {topic}")
    print(HELP)

    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        running = True
        while running and rclpy.ok():
            # Spin the node (fires the publish timer) then poll the keyboard.
            rclpy.spin_once(node, timeout_sec=0.0)
            key = _read_key(timeout=1.0 / args.rate)
            if key is not None:
                running = node.apply_key(key.lower())
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        # Make sure we leave the robot stopped.
        node.vx = node.vy = node.wz = 0.0
        node._publish()
        node.destroy_node()
        rclpy.shutdown()
        print("\nwasd_teleop: stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
