"""Per-robot RTAB-Map launch for the SortBots warehouse demo.

Wraps `rtabmap_launch/launch/rtabmap.launch.py` with the topic remaps
matching what `scripts/spawn_warehouse.py` publishes. Defaults to
`robot_0`; pass `robot_id:=robot_1` to map the other XLeRobot.

Run after `sudo apt install ros-jazzy-rtabmap-ros` from a shell with
system ROS 2 Jazzy sourced (NOT the Isaac venv):

    source /opt/ros/jazzy/setup.bash
    ros2 launch ./launch/sortbots_rtabmap_robot.launch.py robot_id:=robot_0

The launch file lives loose under `launch/`; it's not built into a ROS 2
package because the only thing it adds over the upstream launch is the
SortBots topic remaps + an opinionated argument set.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value="robot_0",
            description="Which spawned robot to feed RTAB-Map (robot_0 or robot_1).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description=(
                "Use /clock sim time. MUST be true against spawn_warehouse.py: "
                "the Isaac ROS 2 bridge stamps every message with simulation "
                "time (seconds since sim start), not wall clock, so with "
                "use_sim_time=false RTAB-Map treats all data as ~decades old "
                "and every TF lookup fails — the map stays empty."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("rtabmap_launch"),
                    "launch",
                    "rtabmap.launch.py",
                ])
            ),
            launch_arguments={
                "rgb_topic":          ["/", robot_id, "/camera/rgb"],
                "depth_topic":        ["/", robot_id, "/camera/depth"],
                "camera_info_topic":  ["/", robot_id, "/camera/camera_info"],
                "imu_topic":          ["/", robot_id, "/imu"],
                # RTAB-Map waits for the first IMU sample before initializing.
                # Useful because the sim publishes IMU at ~60 Hz; without this
                # RTAB-Map's odom estimator can race the publish.
                "wait_imu_to_init":   "true",
                "frame_id":           [robot_id, "/base_link"],
                "odom_frame_id":      [robot_id, "/odom"],
                "odom_topic":         ["/", robot_id, "/odom"],
                # Use the sim's perfect odom as the motion prior; don't ask
                # RTAB-Map to compute visual odometry.
                "visual_odometry":    "false",
                "subscribe_rgbd":     "false",
                "approx_sync":        "true",
                "rviz":               "true",
                "namespace":          robot_id,
                "use_sim_time":       use_sim_time,
            }.items(),
        ),
    ])
