"""Workstation half of offloaded SLAM: RTAB-Map for a real robot whose
Jetson runs `slam_role:=robot` (tasks/slam_offload.md).

The robot keeps everything time-critical — the D435 driver, visual odometry,
the live obstacle cloud — and sends only a rate-capped compressed RGB-D
keyframe stream (/<id>/rgbd_image/compressed, ~2 Hz) plus its /<id>/odom.
This machine runs the expensive part: RTAB-Map's graph, loop closure, Grid/3D
ray tracing and the reconstruction clouds, and its database is what
scripts/recon/rtabmap_to_colmap.py turns into Gaussian Splatting input.

    # robot (Jetson container):  scripts/run_robot.sh --slam-role robot
    # workstation, clean shell with ROS 2 Jazzy sourced:
    ros2 launch ./launch/sortbots_slam_workstation.launch.py robot_id:=robot_0

Both ends must see each other's DDS traffic: the robot's default
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST keeps it private, so set
ROS_STATIC_PEERS to the other machine's address on both (docs/jetson.md).

Uses the real-robot RTAB-Map parameters (camera_odometry:=true selects them;
slam_role:=workstation keeps odometry itself off this machine).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="robot_0"),
        DeclareLaunchArgument(
            "database_path",
            # Not the robot's ~/.ros/sortbots_<id>.db: this is a different
            # machine's map, and maps.sh save expects to find it by name.
            default_value=["~/.ros/sortbots_", robot_id, "_ws.db"],
        ),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument("delete_db_on_start", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(HERE, "sortbots_rtabmap_robot.launch.py")),
            launch_arguments={
                "robot_id": robot_id,
                "slam_role": "workstation",
                "camera_odometry": "true",
                "use_sim_time": "false",
                "rviz": "false",
                "depth_filter": "false",
                "wait_imu_to_init": "false",
                "database_path": LaunchConfiguration("database_path"),
                "localization": LaunchConfiguration("localization"),
                "delete_db_on_start": LaunchConfiguration("delete_db_on_start"),
            }.items(),
        ),
    ])
