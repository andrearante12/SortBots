"""Brings up the SortBots web dashboard stack: rosbridge + web_video_server
+ the dashboard's own static file server (`webui/serve.py`).

Doesn't start Nav2, RTAB-Map, `nodes/task_manager.py`, or Isaac — those run
separately (see docs/setup.md and the other launch files in this dir). This
file only owns the three processes the browser talks to directly.

Run from a shell with system ROS 2 Jazzy sourced (NOT the Isaac venv or
conda), AND with conda's bin dirs off the front of PATH — verified live:
if `which python3` resolves into `miniconda3/...` instead of `/usr/bin`,
`rosbridge_websocket`'s `#!/usr/bin/env python3` shebang grabs conda's
Python (e.g. 3.13) instead of the system 3.12 that ROS 2 Jazzy's compiled
`_rclpy_pybind11` extension is built for, and it crashes on launch with
`ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`. `conda
deactivate` alone does NOT fix this if conda's init hook unconditionally
prepends its bin dirs to PATH in `.bashrc` — check `which python3` first:

    source /opt/ros/jazzy/setup.bash
    which python3   # must print /usr/bin/python3, not .../miniconda3/...
    # if it doesn't:
    export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH"
    ros2 launch ./launch/sortbots_webui.launch.py

Then open http://localhost:8081/. Requires (see docs/setup.md):
    sudo apt install ros-jazzy-rosbridge-suite ros-jazzy-web-video-server
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    dashboard_port = LaunchConfiguration("dashboard_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "dashboard_port",
            default_value="8081",
            description="Port for webui/serve.py (the dashboard's own HTTP server).",
        ),

        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            parameters=[{
                "port": 9090,
                # RTAB-Map's cloud_map (3D reconstruction panel) easily exceeds
                # rosbridge's 1MB default: ~900KB raw -> ~1.2MB once base64'd for
                # JSON, and it only grows as the robot explores. Past that default,
                # rosbridge silently fragments the message (op: "fragment") instead
                # of sending it whole; the vendored webui/vendor/roslib.min.js has
                # no fragment-reassembly support, so the browser just never
                # receives a usable message — no error, the panel simply never
                # updates. Room for a much bigger map before hitting this again.
                "max_message_size": 20_000_000,
            }],
        ),
        Node(
            package="web_video_server",
            executable="web_video_server",
            name="web_video_server",
            output="screen",
            # use_sim_time stays False here even though every other node in the
            # demo runs on sim time: with use_sim_time=true this node repeatedly
            # livelocked (~90% CPU spin, accepts TCP connections but never
            # handles the request or sends a byte, SIGINT ignored — needs
            # SIGKILL) within minutes of serving MJPEG streams. It only
            # re-encodes frames as they arrive, so wall-clock time is fine.
            # (It later livelocked without sim time too — see the watchdog
            # below and the snapshot-polling rationale in webui/app.js.)
            parameters=[{"port": 8080, "use_sim_time": False}],
        ),
        # Kills and relaunches web_video_server whenever it livelocks (a
        # recurring jazzy bug under long-lived MJPEG streams — see the script's
        # header). The dashboard now polls /snapshot instead of streaming,
        # which avoids the main trigger, but the watchdog stays as a backstop
        # for anything else (rviz plugins, curl, old tabs) opening streams.
        ExecuteProcess(
            cmd=["bash", os.path.join(REPO_ROOT, "nodes", "web_video_watchdog.sh")],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "webui", "serve.py"),
                "--port", dashboard_port,
            ],
            output="screen",
        ),
        # Print the tailnet URL (+ QR) to open the dashboard from a phone. All
        # three servers above bind 0.0.0.0, so any tailnet device can reach
        # them — see scripts/webui_url.py and docs/running.md.
        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "scripts", "webui_url.py"),
                "--port", dashboard_port,
            ],
            output="screen",
        ),
    ])
