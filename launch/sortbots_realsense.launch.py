"""Real Intel RealSense D435 driver, remapped onto the sim's camera topics.

The robot-side stand-in for what scripts/spawn_warehouse.py publishes in Isaac:
`/<robot_id>/camera/{rgb,depth,camera_info}` plus a TF chain from
`<robot_id>/base_link` to the image frame. Everything downstream — the
dashboard's camera pane, nodes/dynamic_obstacle_filter.py, RTAB-Map — is
unchanged between sim and real because of this file.

Included by launch/sortbots_bringup.launch.py when platform:=real (which is
how scripts/run_robot.sh runs it). Standalone, inside docker/jetson's
container (scripts/jetson.sh shell):

    ros2 launch ./launch/sortbots_realsense.launch.py robot_id:=robot_0

Tuning lives in configs/sensors/d435_real.yaml (profile, mount, filters).
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(REPO_ROOT, "configs", "sensors", "d435_real.yaml")


def _actions(context):
    rid = context.launch_configurations["robot_id"]
    with open(CONFIG) as f:
        cfg = yaml.safe_load(f)
    xyz = [float(v) for v in cfg["mount"]["xyz"]]
    rpy = [float(v) for v in cfg["mount"]["rpy"]]
    filters = cfg.get("filters") or {}

    # Node `camera` in namespace <rid> publishes /<rid>/camera/<stream>/...;
    # the remaps below rename those onto the sim's names. Full resolved names
    # on the left, not `~/...`: private-name remaps are easy to get subtly
    # wrong, and a remap that doesn't match fails silently (topic just keeps
    # its old name, and the dashboard pane stays black).
    base = f"/{rid}/camera"
    driver = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace=rid,
        output="screen",
        parameters=[{
            "camera_name": "camera",
            # tf_prefix is what makes this multi-robot-safe: frames become
            # <rid>/camera_link, <rid>/camera_color_optical_frame, ... — the
            # same <ns>/ prefixing the sim uses (scripts/_ros2_graphs.py).
            "tf_prefix": f"{rid}/",
            "enable_color": True,
            "enable_depth": True,
            "rgb_camera.color_profile": cfg["profile"],
            "rgb_camera.color_format": "RGB8",
            "depth_module.depth_profile": cfg["profile"],
            # Aligned depth shares color's intrinsics, so ONE camera_info
            # serves both — exactly the sim's single-camera_info layout.
            "align_depth.enable": True,
            # Plain D435 (8086:0b07) has no IMU; a D435i would, but RTAB-Map
            # would want it fused with a filter first — see docs/jetson.md.
            "enable_gyro": False,
            "enable_accel": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "pointcloud.enable": False,
            "spatial_filter.enable": bool(filters.get("spatial", False)),
            "temporal_filter.enable": bool(filters.get("temporal", False)),
            "hole_filling_filter.enable": bool(filters.get("hole_filling", False)),
            # Re-enumerations after a USB hiccup are routine on the Jetson;
            # keep retrying rather than dying and taking SLAM's input with it.
            "wait_for_device_timeout": -1.0,
            "reconnect_timeout": 6.0,
        }],
        remappings=[
            (f"{base}/color/image_raw", f"{base}/rgb"),
            (f"{base}/aligned_depth_to_color/image_raw", f"{base}/depth"),
            (f"{base}/color/camera_info", f"{base}/camera_info"),
        ],
    )

    mount = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{rid}_camera_mount",
        output="screen",
        arguments=[
            "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
            "--roll", str(rpy[0]), "--pitch", str(rpy[1]), "--yaw", str(rpy[2]),
            "--frame-id", f"{rid}/base_link",
            "--child-frame-id", f"{rid}/camera_link",
        ],
    )
    return [driver, mount]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value="robot_0",
            description="Namespace / TF prefix for this robot's camera.",
        ),
        OpaqueFunction(function=_actions),
    ])
