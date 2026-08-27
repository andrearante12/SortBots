"""Phase 4 regression: `/robot_0/imu` publishes MPU 6050-shaped messages.

Subscribes to the IMU topic; asserts frame_id, gravity-Z component,
and reasonable angular velocity when stationary.

Result file: `/tmp/isaac_imu_topic_result.txt`.
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
    out = "/tmp/isaac_imu_topic_result.txt"
    with open(out, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


pytestmark = pytest.mark.skipif(
    os.environ.get("AMENT_PREFIX_PATH") is not None,
    reason="ROS 2 is sourced in the launching shell; bundled rclpy would clash.",
)


def test_imu_topic(world, xlerobot_usd_path):
    open("/tmp/isaac_imu_topic_result.txt", "w").close()
    _emit("=== test_imu_topic ===")

    repo_root = Path(__file__).resolve().parents[2]
    with open(repo_root / "configs" / "sensors" / "d435.json") as f:
        d435 = json.load(f)
    with open(repo_root / "configs" / "sensors" / "mpu6050.json") as f:
        mpu = json.load(f)

    import omni.kit.app
    import omni.kit.commands
    import omni.timeline
    import omni.usd
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.sensors.camera import Camera
    from pxr import Gf, UsdGeom

    from _ros2_graphs import build_robot_graphs  # type: ignore
    from build_warehouse import SPAWN_POSITIONS  # type: ignore

    name = "robot_0"
    pos = SPAWN_POSITIONS[name]
    prim_path = f"/World/{name}"
    add_reference_to_stage(usd_path=str(xlerobot_usd_path), prim_path=prim_path)

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))

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

    world.reset()
    cam.initialize()
    cam.set_local_pose(orientation=np.array([0.0, 1.0, 0.0, 0.0]))

    build_robot_graphs(
        robot_prim_path=prim_path,
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
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    if not rclpy.ok():
        rclpy.init()
    node = Node("phase4_imu_tester")

    received: list[Imu] = []
    node.create_subscription(Imu, "/robot_0/imu", received.append, 10)

    try:
        deadline = time.time() + 6.0
        while time.time() < deadline and len(received) < 5:
            world.step(render=True)
            rclpy.spin_once(node, timeout_sec=0.001)

        assert len(received) >= 5, f"only {len(received)} IMU messages received"
        msg = received[-1]
        _emit(f"  count={len(received)}")
        _emit(f"  frame_id={msg.header.frame_id!r}")
        _emit(
            "  linear_acc=("
            f"{msg.linear_acceleration.x:+.3f}, "
            f"{msg.linear_acceleration.y:+.3f}, "
            f"{msg.linear_acceleration.z:+.3f})"
        )
        _emit(
            "  angular_vel=("
            f"{msg.angular_velocity.x:+.3f}, "
            f"{msg.angular_velocity.y:+.3f}, "
            f"{msg.angular_velocity.z:+.3f})"
        )

        assert msg.header.frame_id == "robot_0/imu_link", msg.header.frame_id

        # `IsaacReadIMU` with readGravity=True reports +g in the Z axis when
        # the IMU's Z is world-up (the IMU schema's local +Z is up by default).
        # The sign convention matches "what an accelerometer would read while
        # sitting still" — accel reads +g pointing UP. Allow 0.5 m/s² slop.
        assert abs(abs(msg.linear_acceleration.z) - 9.81) < 0.5, (
            f"|az| should be ~9.81 m/s² (gravity), got {msg.linear_acceleration.z}"
        )

        # Stationary robot: angular velocity should be near zero.
        ang_mag = (
            msg.angular_velocity.x**2
            + msg.angular_velocity.y**2
            + msg.angular_velocity.z**2
        ) ** 0.5
        assert ang_mag < 0.1, f"angular velocity too high for stationary robot: {ang_mag}"

        _emit("test_imu_topic: OK")
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
