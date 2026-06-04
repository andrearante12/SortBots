"""Phase 4 regression: `/robot_0/camera/camera_info` matches D435 spec.

Uses the same `world` + `xlerobot_usd_path` fixtures as Phase 3. Builds
the per-robot graphs (now including the CameraInfoHelper), plays the
timeline, and same-process `rclpy` subscribes to the camera_info topic.

Result file: `/tmp/isaac_camera_info_result.txt`.
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
    out = "/tmp/isaac_camera_info_result.txt"
    with open(out, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


pytestmark = pytest.mark.skipif(
    os.environ.get("AMENT_PREFIX_PATH") is not None,
    reason="ROS 2 is sourced in the launching shell; bundled rclpy would clash.",
)


def test_camera_info(world, xlerobot_usd_path):
    open("/tmp/isaac_camera_info_result.txt", "w").close()
    _emit("=== test_camera_info ===")

    repo_root = Path(__file__).resolve().parents[2]
    with open(repo_root / "configs" / "sensors" / "d435.json") as f:
        d435 = json.load(f)
    with open(repo_root / "configs" / "sensors" / "mpu6050.json") as f:
        mpu = json.load(f)

    import omni.kit.app
    import omni.kit.commands
    import omni.timeline
    from isaacsim.core.api.robots import Robot
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.sensors.camera import Camera
    from pxr import Gf, UsdGeom
    import omni.usd

    from _ros2_graphs import build_robot_graphs  # type: ignore

    from build_warehouse import SPAWN_POSITIONS  # type: ignore

    prim_path = "/World/robot_0"
    add_reference_to_stage(usd_path=str(xlerobot_usd_path), prim_path=prim_path)

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*SPAWN_POSITIONS["robot_0"]))

    robot = Robot(prim_path=prim_path, name="robot_0", position=np.array(SPAWN_POSITIONS["robot_0"]))
    world.scene.add(robot)

    cam_path = f"{prim_path}/head_camera_rgb_optical_frame/cam"
    cam = Camera(prim_path=cam_path, resolution=(d435["width"], d435["height"]), frequency=d435["fps"])

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
        namespace="robot_0",
        imu_prim_path=imu_path,
        rgb_resolution=(d435["width"], d435["height"]),
    )
    _emit("graphs built")

    app = omni.kit.app.get_app()
    for _ in range(3):
        app.update()
    omni.timeline.get_timeline_interface().play()

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo

    if not rclpy.ok():
        rclpy.init()
    node = Node("phase4_camera_info_tester")

    received: list[CameraInfo] = []
    node.create_subscription(
        CameraInfo, "/robot_0/camera/camera_info", received.append, 10
    )

    try:
        deadline = time.time() + 8.0
        while time.time() < deadline and not received:
            world.step(render=True)
            rclpy.spin_once(node, timeout_sec=0.005)

        assert received, "no /robot_0/camera/camera_info message received"
        ci = received[0]
        _emit(f"  width={ci.width} height={ci.height}")
        _emit(f"  K[0]={ci.k[0]:.2f} K[4]={ci.k[4]:.2f} K[2]={ci.k[2]:.2f} K[5]={ci.k[5]:.2f}")
        _emit(f"  distortion_model={ci.distortion_model!r} D={list(ci.d)}")
        _emit(f"  frame_id={ci.header.frame_id!r}")

        assert ci.width == d435["width"]
        assert ci.height == d435["height"]
        # Intrinsic tolerance: derived fx via aperture formula is ~600 vs
        # datasheet ~605; allow 5%.
        assert abs(ci.k[0] - d435["fx"]) / d435["fx"] < 0.05, f"fx={ci.k[0]}"
        assert abs(ci.k[4] - d435["fy"]) / d435["fy"] < 0.05, f"fy={ci.k[4]}"
        assert abs(ci.k[2] - d435["cx"]) < 2.0, f"cx={ci.k[2]}"
        assert abs(ci.k[5] - d435["cy"]) < 2.0, f"cy={ci.k[5]}"
        assert ci.header.frame_id == "robot_0/camera_optical"
        # D435 aligned color output is rectified — D coeffs should be zeros.
        assert all(abs(d) < 1e-6 for d in ci.d), f"non-zero distortion: {list(ci.d)}"

        _emit("test_camera_info: OK")
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
