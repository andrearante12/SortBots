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

Multi-robot: pass `robot_ids:=robot_0,robot_1` (comma-separated) to bring up
a full independent RTAB-Map + Nav2 + task_manager stack per robot — TF is
already multi-robot-safe (see scripts/_ros2_graphs.py's frame-prefixing) and
configs/nav2_params.yaml is rewritten per-robot at launch time (see
sortbots_nav2.launch.py). The dashboard (webui) stays singular regardless of
robot count. `localization`/`database_path`/`explore` apply ONLY to the
single robot named by `robot_id` (default robot_0) — other robots in
`robot_ids` get their own auto-computed map path, matching the multi-robot
scope this was built and tested against. `robot_id` itself does not need to
appear in `robot_ids` for its map settings to be ignored cleanly; it's
simplest to just include it.

Each robot's RTAB-Map owns its own SLAM at `<rid>/map`, unrelated to any
other robot's. This file anchors every robot into a shared `map` frame with a
static `map -> <rid>/map` transform published from that robot's known spawn
pose (configs/robots.yaml, keyed by the `scene` arg — must match
scripts/spawn_warehouse.py --scene), then runs nodes/map_merge.py once to
fuse all the anchored grids into one world-anchored `/map`. That fused `/map`
is what Nav2's global_frame and every explorer plan against by default — see
nodes/map_merge.py's docstring for the full picture.
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(LAUNCH_DIR)


def _include(filename):
    return PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, filename))


def _spawn_pose(scene: str, robot_id: str) -> tuple[float, float, float]:
    """(x, y, z) for `robot_id` in `scene`, from configs/robots.yaml — the
    same roster scripts/spawn_warehouse.py reads to place the robot in Isaac.
    Read directly (not imported) since this launch file has no Python
    dependency on scripts/.
    """
    import yaml

    with open(os.path.join(REPO_ROOT, "configs", "robots.yaml")) as f:
        roster = yaml.safe_load(f)
    for r in roster["robots"]:
        if r["id"] == robot_id:
            return tuple(r["spawn"][scene])
    raise KeyError(f"{robot_id!r} not in configs/robots.yaml")


def _make_per_robot_actions(
    context,
    use_sim_time,
    rviz,
    nav2,
    task_manager,
    scripted_pick,
    localization,
    database_path,
    delete_db_on_start,
    explore,
    explore_autostart,
    robot_id_cfg,
    robot_ids_cfg,
    scene_cfg,
):
    primary_robot_id = robot_id_cfg.perform(context)
    raw = robot_ids_cfg.perform(context).strip()
    robot_ids = [r.strip() for r in raw.split(",") if r.strip()] if raw else [primary_robot_id]
    scene = scene_cfg.perform(context)

    actions = []
    for rid in robot_ids:
        # Anchors this robot's SLAM into the shared world frame: RTAB-Map
        # publishes its OWN map at <rid>/map (sortbots_rtabmap_robot.launch.py
        # sets map_frame_id), starting from wherever its odom happened to
        # init — which is this robot's known spawn pose. nodes/map_merge.py
        # looks this transform up to resample every robot's grid into one
        # `/map`; get it wrong and two robots' grids fuse at the wrong offset.
        x, y, z = _spawn_pose(scene, rid)
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{rid}_map_anchor",
            output="screen",
            arguments=[
                "--x", str(x), "--y", str(y), "--z", str(z),
                "--frame-id", "map", "--child-frame-id", f"{rid}/map",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ))

        # Before RTAB-Map: strip movers from depth so SLAM never sees them.
        actions.append(ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "nodes", "dynamic_obstacle_filter.py"),
                "--robot-id", rid,
            ],
            output="screen",
        ))

        rtabmap_args = {"robot_id": rid, "use_sim_time": use_sim_time, "rviz": rviz}
        # Map-lifecycle overrides (localization / an explicit --map path /
        # delete_db_on_start) are a single-robot concept today — see module
        # docstring; every OTHER robot in a multi-robot robot_ids list falls
        # through to sortbots_rtabmap_robot.launch.py's own per-robot default
        # for THOSE.
        #
        # database_path is different and must be set EXPLICITLY for every
        # robot, never omitted: IncludeLaunchDescription does not isolate a
        # launch-configuration name the PARENT already declared, and this
        # file's own top-level `database_path` argument (for the primary
        # robot) already occupies that name in the shared launch context by
        # the time a secondary robot's Include runs — so
        # sortbots_rtabmap_robot.launch.py's `DeclareLaunchArgument` sees it
        # as already-set and silently reuses the PRIMARY robot's resolved
        # path instead of recomputing its own default. Verified live: with
        # this omitted, robot_1's rtabmap opened robot_0's sqlite database
        # ("Deleted database ...sortbots_robot_0.db"), collided with robot_0
        # actually writing it, and crashed FATAL ("attempt to write a
        # readonly database") — which then meant robot_1 never published its
        # map -> odom TF, so its whole tree was disconnected from `map`.
        if rid == primary_robot_id:
            rtabmap_args["localization"] = localization
            rtabmap_args["delete_db_on_start"] = delete_db_on_start
            rtabmap_args["database_path"] = database_path
        else:
            rtabmap_args["database_path"] = ["~/.ros/sortbots_", rid, ".db"]
        actions.append(IncludeLaunchDescription(
            _include("sortbots_rtabmap_robot.launch.py"),
            launch_arguments=rtabmap_args.items(),
        ))

        actions.append(IncludeLaunchDescription(
            _include("sortbots_nav2.launch.py"),
            launch_arguments={"robot_id": rid, "use_sim_time": use_sim_time}.items(),
            condition=IfCondition(nav2),
        ))

        # Mesh status broadcaster (self-reported pose — peers must not TF-eavesdrop).
        actions.append(ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "nodes", "fleet_radio.py"),
                "--robot-id", rid,
            ],
            output="screen",
        ))

        actions.append(ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "nodes", "task_manager.py"),
                "--robot-id", rid,
            ],
            output="screen",
            condition=IfCondition(task_manager),
        ))

        # The other half of the dispatch loop: task_manager publishes
        # pick_request and blocks on pick_result, so without this every task
        # stalls in PICKING until PICK_TIMEOUT_SEC. Primary robot only —
        # spawn_warehouse.py only applies arm commands to robot_0, so starting
        # it for a secondary robot would publish into a void.
        if rid == primary_robot_id:
            actions.append(ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(REPO_ROOT, "nodes", "scripted_pick.py"),
                    "--robot-id", rid,
                ],
                output="screen",
                condition=IfCondition(scripted_pick),
            ))

        # Every robot in robot_ids explores, not just the primary — with
        # /map fused across robots (nodes/map_merge.py) and fleet mesh
        # intent/status (nodes/fleet_radio.py + explorer /fleet/intent),
        # this is what makes a multi-robot run collaborative.
        actions.append(GroupAction(
            condition=IfCondition(explore),
            actions=[
                # use_sim_time is REQUIRED, not cosmetic: every timeout in
                # configs/explorer.yaml (goal_timeout_s, stuck_window_s,
                # blacklist TTLs) is really a distance budget, and the robot
                # covers those metres in SIM time. On the wall clock they are
                # worth ~45% of their configured value — see nodes/explorer.py's
                # _now() for the live failure that caused.
                ExecuteProcess(
                    cmd=[
                        "python3",
                        os.path.join(REPO_ROOT, "nodes", "explorer.py"),
                        "--robot-id", rid,
                        "--autostart",
                        "--ros-args", "-p", ["use_sim_time:=", use_sim_time],
                    ],
                    output="screen",
                    condition=IfCondition(explore_autostart),
                ),
                ExecuteProcess(
                    cmd=[
                        "python3",
                        os.path.join(REPO_ROOT, "nodes", "explorer.py"),
                        "--robot-id", rid,
                        "--ros-args", "-p", ["use_sim_time:=", use_sim_time],
                    ],
                    output="screen",
                    condition=UnlessCondition(explore_autostart),
                ),
            ],
        ))

    # Fuses every robot's own /<rid>/map into the world-anchored /map (see
    # that node's docstring). Once per bringup, not per robot — it needs the
    # full roster to fuse anything.
    actions.append(ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(REPO_ROOT, "nodes", "map_merge.py"),
            "--robot-ids", ",".join(robot_ids),
            # Required, not cosmetic: this node rebroadcasts a TF edge onto
            # /tf (see its own _tick()), so its clock must agree with the
            # rest of the sim-time-stamped tree — see nodes/map_merge.py's
            # main() for what broke without this.
            "--ros-args", "-p", ["use_sim_time:=", use_sim_time],
        ],
        output="screen",
    ))
    # Same anchors, 3D: fuse /<rid>/recon_cloud -> /fleet/recon_cloud for the
    # dashboard (collaborative fusion, not collaborative SLAM).
    actions.append(ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(REPO_ROOT, "nodes", "recon_cloud_merge.py"),
            "--robot-ids", ",".join(robot_ids),
            "--ros-args", "-p", ["use_sim_time:=", use_sim_time],
        ],
        output="screen",
    ))
    return actions


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    robot_ids = LaunchConfiguration("robot_ids")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    nav2 = LaunchConfiguration("nav2")
    webui = LaunchConfiguration("webui")
    task_manager = LaunchConfiguration("task_manager")
    scripted_pick = LaunchConfiguration("scripted_pick")
    dashboard_port = LaunchConfiguration("dashboard_port")
    localization = LaunchConfiguration("localization")
    database_path = LaunchConfiguration("database_path")
    delete_db_on_start = LaunchConfiguration("delete_db_on_start")
    explore = LaunchConfiguration("explore")
    explore_autostart = LaunchConfiguration("explore_autostart")
    scene = LaunchConfiguration("scene")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_id",
            default_value="robot_0",
            description=(
                "The 'primary' robot: which map/localization/explore settings "
                "below apply to. Also the sole robot brought up if robot_ids "
                "is left empty."
            ),
        ),
        DeclareLaunchArgument(
            "robot_ids",
            default_value="",
            description=(
                "Comma-separated robot ids to bring up, e.g. 'robot_0,robot_1'. "
                "Empty (default) means just [robot_id] — the original "
                "single-robot behavior, unchanged."
            ),
        ),
        DeclareLaunchArgument(
            "scene",
            default_value="nvidia",
            description=(
                "Which warehouse scene each robot spawned in — must match "
                "scripts/spawn_warehouse.py --scene. Selects the spawn pose "
                "(configs/robots.yaml) used to anchor each robot's SLAM frame "
                "into the shared `map` frame; wrong value here misaligns the "
                "fused /map (nodes/map_merge.py) even though each robot's own "
                "SLAM still works fine."
            ),
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
            "scripted_pick",
            default_value="true",
            description=(
                "Start nodes/scripted_pick.py, which answers task_manager's "
                "pick_request by playing back configs/arm_poses.yaml. Primary "
                "robot only. Set false to drive the arm purely from the "
                "dashboard's arm pad."
            ),
        ),
        DeclareLaunchArgument(
            "dashboard_port",
            default_value="8081",
            description="Port for the dashboard's HTTP server (webui/serve.py).",
        ),
        DeclareLaunchArgument(
            "localization",
            default_value="false",
            description=(
                "Localize against the saved map instead of building one. "
                "Nav2 is unaffected — it reads the same `map` topic either way."
            ),
        ),
        DeclareLaunchArgument(
            "database_path",
            # Mirrors the default in sortbots_rtabmap_robot.launch.py; kept in
            # both because IncludeLaunchDescription passes a concrete value and
            # can't defer to the includee's own default.
            default_value=["~/.ros/sortbots_", robot_id, ".db"],
            description="Where the map is saved to / loaded from.",
        ),
        DeclareLaunchArgument(
            "delete_db_on_start",
            default_value="true",
            description=(
                "Start mapping from an empty database. Set false (with "
                "localization:=false) to RESUME mapping into an existing "
                "database instead of wiping it — 'keep exploring from where "
                "a prior session left off', as opposed to localization:=true "
                "which is read-only and never grows the map. Ignored when "
                "localization:=true (never deletes what you're localizing "
                "against — see sortbots_rtabmap_robot.launch.py)."
            ),
        ),
        DeclareLaunchArgument(
            "explore",
            default_value="false",
            description=(
                "Start nodes/explorer.py — frontier-based autonomous mapping. "
                "Arbitrates with task_manager over the shared navigate_to_pose "
                "action server; see task_manager.py's module docstring."
            ),
        ),
        DeclareLaunchArgument(
            "explore_autostart",
            default_value="true",
            description=(
                "Whether the explorer starts exploring immediately on launch "
                "(true, the 'hands off' default used by run_demo.sh --explore) "
                "or waits for an explicit explore_cmd=start (false, for "
                "dashboard-driven start/stop). Ignored if explore:=false."
            ),
        ),

        # Dashboard stack (rosbridge 9090 + web_video_server 8080 + serve.py).
        # Singular regardless of robot count — see module docstring.
        IncludeLaunchDescription(
            _include("sortbots_webui.launch.py"),
            launch_arguments={"dashboard_port": dashboard_port}.items(),
            condition=IfCondition(webui),
        ),

        # Per-robot stack: RTAB-Map SLAM, Nav2, task_manager, and (primary
        # robot only) the explorer. Built by _make_per_robot_actions() via
        # OpaqueFunction because robot_ids needs to be resolved (split on
        # comma) at launch time, not at generate_launch_description() time.
        OpaqueFunction(
            function=_make_per_robot_actions,
            args=[
                use_sim_time, rviz, nav2, task_manager, scripted_pick,
                localization, database_path, delete_db_on_start,
                explore, explore_autostart, robot_id, robot_ids, scene,
            ],
        ),
    ])
