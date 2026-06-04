"""Phase 4 regression: `/robot_0/cmd_vel` Twist drives the sim robot.

Publishes a constant `geometry_msgs/Twist` on `/robot_0/cmd_vel`, reads
`/robot_0/odom`, asserts >0.3 m of motion. Also confirms `robot_1` (no
cmd_vel published) stays put. Re-publishes with `angular.z = 0.5` to
verify the `root_z_rotation_joint` velocity-drive edit lets yaw through.

Result file: `/tmp/isaac_cmd_vel_result.txt`.
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))


def _emit(line: str) -> None:
    out = "/tmp/isaac_cmd_vel_result.txt"
    with open(out, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


pytestmark = pytest.mark.skipif(
    os.environ.get("AMENT_PREFIX_PATH") is not None,
    reason="ROS 2 is sourced in the launching shell; bundled rclpy would clash.",
)


def _spawn_one(world, name: str, position, xlerobot_usd_path: Path, d435: dict, mpu: dict):
    import omni.kit.commands
    import omni.usd
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.sensors.camera import Camera
    from pxr import Gf, UsdGeom

    prim_path = f"/World/{name}"
    add_reference_to_stage(usd_path=str(xlerobot_usd_path), prim_path=prim_path)

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*position))

    robot = Robot(prim_path=prim_path, name=name)
    world.scene.add(robot)

    cam_path = f"{prim_path}/head_camera_rgb_optical_frame/cam"
    cam = Camera(prim_path=cam_path, resolution=(320, 240), frequency=15)
    cam_prim = stage.GetPrimAtPath(cam_path)
    ucam = UsdGeom.Camera(cam_prim)
    ucam.CreateFocalLengthAttr(float(d435["focal_length_mm"]))
    ucam.CreateHorizontalApertureAttr(float(d435["horizontal_aperture_mm"]))
    ucam.CreateVerticalApertureAttr(float(d435["vertical_aperture_mm"]))
    ucam.CreateClippingRangeAttr(Gf.Vec2f(*[float(v) for v in d435["clipping_range_m"]]))

    parent_path = f"{prim_path}/base_link"
    imu_path = f"{parent_path}/imu_sensor"
    omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path="/imu_sensor",
        parent=parent_path,
        sensor_period=1.0 / float(mpu["internal_rate_hz"]),
        translation=Gf.Vec3d(*[float(v) for v in mpu["mount_offset_m"]]),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    return robot, cam, cam_path, imu_path


def test_cmd_vel(world, xlerobot_usd_path):
    open("/tmp/isaac_cmd_vel_result.txt", "w").close()
    _emit("=== test_cmd_vel ===")

    repo_root = Path(__file__).resolve().parents[2]
    with open(repo_root / "configs" / "sensors" / "d435.json") as f:
        d435 = json.load(f)
    with open(repo_root / "configs" / "sensors" / "mpu6050.json") as f:
        mpu = json.load(f)

    import omni.graph.core as og
    import omni.kit.app
    import omni.timeline
    from isaacsim.core.utils.types import ArticulationAction

    from _ros2_graphs import build_robot_graphs  # type: ignore
    from build_warehouse import SPAWN_POSITIONS  # type: ignore

    def _read_cmd_vel(name: str):
        """Return (linear, angular) np arrays from the OG SubscribeTwist."""
        base = f"/World/{name}/CmdVelGraph/SubscribeTwist.outputs"
        try:
            lin = og.Controller.attribute(f"{base}:linearVelocity").get()
            ang = og.Controller.attribute(f"{base}:angularVelocity").get()
        except Exception:
            return np.zeros(3), np.zeros(3)
        return (
            np.array([float(lin[0]), float(lin[1]), float(lin[2])]),
            np.array([float(ang[0]), float(ang[1]), float(ang[2])]),
        )

    def _apply_cmd_vel_step():
        """Bridge OG SubscribeTwist → ArticulationAction.joint_velocities.
        Mirrors `spawn_warehouse.py`'s step loop.
        """
        for name, (robot, _, _, _) in robots.items():
            lin, ang = _read_cmd_vel(name)
            v = np.zeros(len(robot.dof_names))
            v[0] = lin[0]
            v[1] = lin[1]
            v[2] = ang[2]
            robot.apply_action(ArticulationAction(joint_velocities=v))

    robots = {}
    for name in ("robot_0", "robot_1"):
        robot, cam, cam_path, imu_path = _spawn_one(
            world, name, SPAWN_POSITIONS[name], xlerobot_usd_path, d435, mpu
        )
        robots[name] = (robot, cam, cam_path, imu_path)

    world.reset()
    for name, (_, cam, _, _) in robots.items():
        cam.initialize()
        cam.set_local_pose(orientation=np.array([0.0, 1.0, 0.0, 0.0]))

    for name, (_, _, cam_path, imu_path) in robots.items():
        build_robot_graphs(
            robot_prim_path=f"/World/{name}",
            chassis_subpath="base_link",
            camera_prim_path=cam_path,
            namespace=name,
            imu_prim_path=imu_path,
            rgb_resolution=(320, 240),
        )

    app = omni.kit.app.get_app()
    for _ in range(3):
        app.update()
    omni.timeline.get_timeline_interface().play()

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    if not rclpy.ok():
        rclpy.init()
    node = Node("phase4_cmd_vel_tester")

    pub = node.create_publisher(Twist, "/robot_0/cmd_vel", 10)
    r0_odom: list[Odometry] = []
    r1_odom: list[Odometry] = []
    node.create_subscription(Odometry, "/robot_0/odom", r0_odom.append, 10)
    node.create_subscription(Odometry, "/robot_1/odom", r1_odom.append, 10)

    # Phase 1: forward motion. Publish Twist {linear.x = 0.2} for ~5 s.
    twist = Twist()
    twist.linear.x = 0.2
    twist.angular.z = 0.0

    try:
        deadline = time.time() + 7.0
        step = 0
        while time.time() < deadline:
            pub.publish(twist)
            _apply_cmd_vel_step()
            world.step(render=True)
            rclpy.spin_once(node, timeout_sec=0.001)
            step += 1

        assert r0_odom, "no robot_0 odom received"
        assert r1_odom, "no robot_1 odom received"

        p0 = r0_odom[0].pose.pose.position
        pN = r0_odom[-1].pose.pose.position
        dx_r0 = ((pN.x - p0.x) ** 2 + (pN.y - p0.y) ** 2) ** 0.5
        _emit(f"  robot_0 dx={dx_r0:.4f} m over {len(r0_odom)} samples")

        p0 = r1_odom[0].pose.pose.position
        pN = r1_odom[-1].pose.pose.position
        dx_r1 = ((pN.x - p0.x) ** 2 + (pN.y - p0.y) ** 2) ** 0.5
        _emit(f"  robot_1 dx={dx_r1:.4f} m over {len(r1_odom)} samples")

        assert dx_r0 > 0.3, f"robot_0 didn't drive far enough: {dx_r0}"
        assert dx_r1 < 0.05, f"robot_1 moved without cmd_vel: {dx_r1}"

        # Phase 2: angular motion — confirm the xlerobot.json velocity-drive
        # edit actually let yaw through (Phase 3 had this as position drive).
        yaw_twist = Twist()
        yaw_twist.linear.x = 0.0
        yaw_twist.angular.z = 0.5

        # Sample yaw via the orientation quaternion's z component (approx).
        prev_q = r0_odom[-1].pose.pose.orientation
        prev_yaw = 2.0 * np.arctan2(prev_q.z, prev_q.w)

        r0_odom.clear()
        deadline = time.time() + 4.0
        while time.time() < deadline:
            pub.publish(yaw_twist)
            _apply_cmd_vel_step()
            world.step(render=True)
            rclpy.spin_once(node, timeout_sec=0.001)

        last_q = r0_odom[-1].pose.pose.orientation
        last_yaw = 2.0 * np.arctan2(last_q.z, last_q.w)
        dyaw = abs(last_yaw - prev_yaw)
        # unwrap one revolution if wrapping happened
        if dyaw > np.pi:
            dyaw = 2 * np.pi - dyaw
        _emit(f"  robot_0 dyaw={dyaw:.4f} rad")
        assert dyaw > 0.1, f"yaw didn't change with cmd_vel.angular.z=0.5: {dyaw}"

        _emit("test_cmd_vel: OK")
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
