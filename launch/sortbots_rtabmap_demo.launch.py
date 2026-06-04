"""SortBots RTAB-Map single-robot demo launch.

Self-contained launch that runs:
1. `rgbd_odometry` — RTAB-Map's visual odometry from RGB-D + IMU,
   publishes its own `odom → camera_optical` TF (no dependency on the
   sim's `/robot_0/odom` topic).
2. `rtabmap` — SLAM that consumes RGB-D + visual odometry, builds the
   2D + 3D map.
3. `rtabmap_viz` — RTAB-Map's GUI.

All nodes use `use_sim_time: true` so they consume the sim's `/clock`
topic (published by `scripts/spawn_warehouse.py`'s `/World/ClockGraph`).

Frame convention: everything is anchored on
`robot_0/camera_optical` as the base for the demo — RTAB-Map's visual
odometry tracks the camera and the map is built in its own
`robot_0/odom` frame. The Phase 4 sim already publishes
`camera_info.header.frame_id = robot_0/camera_optical`, which is the
hook here.

Run from a shell with system ROS 2 Jazzy sourced:

    source /opt/ros/jazzy/setup.bash
    ros2 launch ./launch/sortbots_rtabmap_demo.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_time = {"use_sim_time": True}
    frame_id = "robot_0/camera_optical"
    odom_frame_id = "robot_0/odom"
    map_frame_id = "map"

    remaps = [
        ("rgb/image", "/robot_0/camera/rgb"),
        ("depth/image", "/robot_0/camera/depth"),
        ("rgb/camera_info", "/robot_0/camera/camera_info"),
        ("imu", "/robot_0/imu"),
    ]

    rgbd_odom = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        namespace="robot_0",
        output="screen",
        parameters=[{
            **sim_time,
            "frame_id": frame_id,
            "odom_frame_id": odom_frame_id,
            "subscribe_rgbd": False,
            "approx_sync": True,
            "queue_size": 10,
            "wait_imu_to_init": False,
            "publish_tf": True,
        }],
        remappings=remaps + [("odom", "rgbd_odometry/odom")],
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="robot_0",
        output="screen",
        parameters=[{
            **sim_time,
            "frame_id": frame_id,
            "odom_frame_id": odom_frame_id,
            "map_frame_id": map_frame_id,
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_rgbd": False,
            "subscribe_odom": True,
            "approx_sync": True,
            "queue_size": 10,
            "sync_queue_size": 10,
            "wait_imu_to_init": False,
            "Reg/Force3DoF": "true",
            "Optimizer/Slam2D": "true",
            "publish_tf_map": True,
        }],
        remappings=remaps + [("odom", "rgbd_odometry/odom")],
        arguments=["--delete_db_on_start"],
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        namespace="robot_0",
        output="screen",
        parameters=[{
            **sim_time,
            "frame_id": frame_id,
            "odom_frame_id": odom_frame_id,
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_odom": True,
            "approx_sync": True,
            "queue_size": 10,
            "sync_queue_size": 10,
            "wait_imu_to_init": False,
        }],
        remappings=remaps + [("odom", "rgbd_odometry/odom")],
    )

    return LaunchDescription([
        rgbd_odom,
        rtabmap,
        rtabmap_viz,
    ])
