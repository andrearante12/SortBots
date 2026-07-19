"""Nav2 navigation stack for the SortBots warehouse demo.

No map_server / amcl: RTAB-Map (`sortbots_rtabmap_robot.launch.py`, run
first) owns SLAM and publishes the `map` topic + map->odom TF. This launch
file brings up only the planning/control/behavior layer, namespaced to
match RTAB-Map's topics.

Single-robot MVP: `configs/nav2_params.yaml` hardcodes TF frame ids to
`robot_0/base_link` / `robot_0/odom`. Multi-robot Nav2 is a Phase 5+
concern; this launch file's `robot_id` arg only controls ROS topic
namespacing (kept for consistency with the RTAB-Map launch file), not the
frame ids baked into the params.

Run after RTAB-Map is already publishing (see docs/isaac_sim_phase4.md),
from a shell with system ROS 2 Jazzy sourced (NOT the Isaac venv):

    source /opt/ros/jazzy/setup.bash
    ros2 launch ./launch/sortbots_nav2.launch.py

Requires (not yet installed as of this writing — see docs/setup.md):
    sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \\
        ros-jazzy-depth-image-proc
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    default_params = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "configs",
        "nav2_params.yaml",
    )

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "smoother_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value="robot_0",
            description="Which spawned robot to navigate (topic namespace only; "
            "frame ids in nav2_params.yaml are hardcoded to robot_0 — see module docstring).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock sim time. Must match sortbots_rtabmap_robot.launch.py.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Full path to the Nav2 params file.",
        ),

        GroupAction([
            PushRosNamespace(robot_id),

            # Isaac only publishes a raw depth Image; obstacle_layer needs a
            # PointCloud2. Converts /camera/depth -> /camera/depth/points
            # (namespaced automatically by PushRosNamespace above).
            ComposableNodeContainer(
                name="depth_to_cloud_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::PointCloudXyzNode",
                        name="depth_to_cloud",
                        remappings=[
                            ("image_rect", "camera/depth"),
                            ("camera_info", "camera/camera_info"),
                            ("points", "camera/depth/points"),
                        ],
                        parameters=[{"use_sim_time": use_sim_time}],
                    ),
                ],
                output="screen",
            ),

            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[params_file],
                # Raw output goes to velocity_smoother first, not the robot.
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[params_file],
                remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "node_names": lifecycle_nodes,
                }],
            ),
        ]),
    ])
