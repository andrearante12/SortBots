"""Unified ROS-2-side bringup for the SortBots warehouse demo.

Brings up everything the browser dashboard needs, in one launch, by
including the three existing launch files plus the task-dispatch node:

  1. RTAB-Map SLAM        (sortbots_rtabmap_robot.launch.py) — map + TF
  2. Nav2 planning/control (sortbots_nav2.launch.py)         — navigate_to_pose
  3. Web dashboard stack  (sortbots_webui.launch.py)         — rosbridge +
                                                               web_video_server +
                                                               serve.py
  4. nodes/task_manager.py — the pickup->dropoff dispatch FSM

This does NOT start Isaac Sim (`scripts/spawn_warehouse.py`) — that runs in
its own Python venv and must stay a separate process. Launch the sim first
(or use `scripts/run_demo.sh`, which starts the sim and then this file).

Run from a shell with system ROS 2 Jazzy sourced (NOT the Isaac venv or
conda), AND with conda's bin dirs off the front of PATH — same env-hygiene
rule as sortbots_webui.launch.py: if `which python3` resolves into
`miniconda3/...` instead of `/usr/bin`, rosbridge_websocket's
`#!/usr/bin/env python3` shebang grabs conda's Python and crashes with
`ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`.

    source /opt/ros/jazzy/setup.bash
    which python3   # must print /usr/bin/python3, not .../miniconda3/...
    ros2 launch ./launch/sortbots_bringup.launch.py

Then open http://localhost:8081/ (or the tailnet URL it prints). Toggle
pieces off for partial bringups, e.g. SLAM + webui only:

    ros2 launch ./launch/sortbots_bringup.launch.py nav2:=false task_manager:=false

Startup ordering is tolerant by design: Nav2's lifecycle manager autostarts
and retries until RTAB-Map's map + TF are up, and task_manager only touches
the navigate_to_pose action once a task is dispatched — so no explicit
sequencing between the includes is needed.
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(LAUNCH_DIR)


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    nav2 = LaunchConfiguration("nav2")
    webui = LaunchConfiguration("webui")
    task_manager = LaunchConfiguration("task_manager")
    dashboard_port = LaunchConfiguration("dashboard_port")

    def _include(filename):
        return PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, filename))

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value="robot_0",
            description="Which spawned robot to map/navigate (robot_0 or robot_1).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock sim time. Must be true against spawn_warehouse.py.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Open RTAB-Map's rviz. Default false — the web dashboard replaces it.",
        ),
        DeclareLaunchArgument(
            "nav2",
            default_value="true",
            description="Start the Nav2 stack (sortbots_nav2.launch.py).",
        ),
        DeclareLaunchArgument(
            "webui",
            default_value="true",
            description="Start the dashboard stack (rosbridge + web_video_server + serve.py).",
        ),
        DeclareLaunchArgument(
            "task_manager",
            default_value="true",
            description="Start nodes/task_manager.py, the pickup->dropoff dispatch FSM.",
        ),
        DeclareLaunchArgument(
            "dashboard_port",
            default_value="8081",
            description="Port for the dashboard's HTTP server (webui/serve.py).",
        ),

        # 1. RTAB-Map SLAM (rviz off by default here).
        IncludeLaunchDescription(
            _include("sortbots_rtabmap_robot.launch.py"),
            launch_arguments={
                "robot_id": robot_id,
                "use_sim_time": use_sim_time,
                "rviz": rviz,
            }.items(),
        ),

        # 2. Nav2 planning/control/behavior stack.
        IncludeLaunchDescription(
            _include("sortbots_nav2.launch.py"),
            launch_arguments={
                "robot_id": robot_id,
                "use_sim_time": use_sim_time,
            }.items(),
            condition=IfCondition(nav2),
        ),

        # 3. Dashboard stack (rosbridge 9090 + web_video_server 8080 + serve.py).
        IncludeLaunchDescription(
            _include("sortbots_webui.launch.py"),
            launch_arguments={"dashboard_port": dashboard_port}.items(),
            condition=IfCondition(webui),
        ),

        # 4. Task-dispatch FSM. Loose script (no ament package yet) — same
        # invocation as its module docstring. Uses the launching shell's
        # python3, which the env-hygiene rule above guarantees is /usr/bin.
        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "nodes", "task_manager.py"),
                "--robot-id", robot_id,
            ],
            output="screen",
            condition=IfCondition(task_manager),
        ),
    ])
