"""Nav2 navigation stack for the SortBots warehouse demo.

No map_server / amcl: RTAB-Map (`sortbots_rtabmap_robot.launch.py`, run
first) owns SLAM and publishes the `map` topic + map->odom TF. This launch
file brings up only the planning/control/behavior layer, namespaced to
match RTAB-Map's topics.

Multi-robot: `configs/nav2_params.yaml` hardcodes a few TF frame ids and one
absolute topic to `robot_0` (see the file's own header). At launch time this
file rewrites `configs/nav2_params.yaml` for the ACTUAL `robot_id` before
handing it to any Nav2 node — see `_rewrite_robot_0()` below for why that's
plain string substitution rather than nav2_common's RewrittenYaml (the usual
tool for this): RewrittenYaml matches by leaf KEY name across the whole
file, but `global_frame` appears 4 times here wanting 3 DIFFERENT outcomes
(one per-robot rewrite, three left as the shared `map` frame) — a key-name
match would stomp all four identically.

Run after RTAB-Map is already publishing (see docs/isaac_sim_phase4.md),
from a shell with system ROS 2 Jazzy sourced (NOT the Isaac venv):

    source /opt/ros/jazzy/setup.bash
    ros2 launch ./launch/sortbots_nav2.launch.py robot_id:=robot_0

Requires (not yet installed as of this writing — see docs/setup.md):
    sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \\
        ros-jazzy-depth-image-proc
"""
import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode

LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "smoother_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]


def _substitute_robot_0(node, robot_id: str):
    if isinstance(node, dict):
        return {k: _substitute_robot_0(v, robot_id) for k, v in node.items()}
    if isinstance(node, list):
        return [_substitute_robot_0(v, robot_id) for v in node]
    if isinstance(node, str) and "robot_0" in node:
        return node.replace("robot_0", robot_id)
    return node


def _rewrite_robot_0(params_path: str, robot_id: str) -> str:
    """Return a copy of `params_path` with every "robot_0" substring in a
    string VALUE replaced by `robot_id`. A value-level substring replace,
    not a key-name substitution — see the module docstring for why: it only
    ever touches the handful of values that actually hardcode robot_0
    (`robot_0/base_link`, `robot_0/odom`, `/robot_0/camera/depth/points`,
    `/robot_0/map`), and structurally cannot touch `global_frame: map`
    since "robot_0" isn't a substring of "map" — no matter how the
    surrounding yaml keys are nested, renamed, or reordered.

    Always writes a fresh temp file, even for robot_id == "robot_0" (a
    no-op replace) — one code path, no special-casing the default robot.
    """
    with open(params_path) as f:
        data = yaml.safe_load(f)
    data = _substitute_robot_0(data, robot_id)
    out = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", prefix=f"nav2_params_{robot_id}_", delete=False
    )
    yaml.dump(data, out)
    out.close()
    return out.name


def _make_nav2_group(context, use_sim_time, params_file):
    robot_id = LaunchConfiguration("robot_id").perform(context)
    rewritten_params = _rewrite_robot_0(params_file.perform(context), robot_id)

    return [GroupAction([
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
                parameters=[rewritten_params],
                # Raw output goes to velocity_smoother first, not the robot.
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[rewritten_params],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[rewritten_params],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[rewritten_params],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[rewritten_params],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[rewritten_params],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[rewritten_params],
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
                    "node_names": LIFECYCLE_NODES,
                }],
            ),
        ])]


def generate_launch_description():
    default_params = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "configs",
        "nav2_params.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value="robot_0",
            description=(
                "Which spawned robot to navigate. Controls both ROS topic "
                "namespacing AND the per-robot frame ids rewritten into "
                "nav2_params.yaml at launch time — see module docstring."
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock sim time. Must match sortbots_rtabmap_robot.launch.py.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Full path to the Nav2 params file (rewritten per-robot before use).",
        ),
        OpaqueFunction(
            function=_make_nav2_group,
            args=[LaunchConfiguration("use_sim_time"), LaunchConfiguration("params_file")],
        ),
    ])
