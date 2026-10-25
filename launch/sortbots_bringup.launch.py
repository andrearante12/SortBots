"""Unified ROS-2-side bringup for the SortBots warehouse demo.

Brings up everything the browser dashboard needs, in one launch, by
including the three existing launch files plus the task-dispatch node:

  1. RTAB-Map SLAM        (sortbots_rtabmap_robot.launch.py) — map + TF
  2. Nav2 planning/control (sortbots_nav2.launch.py)         — navigate_to_pose
  3. Web dashboard stack  (sortbots_webui.launch.py)         — rosbridge +
                                                               web_video_server +
                                                               serve.py
  4. nodes/task_manager.py — the pickup->dropoff dispatch FSM
  5. nodes/fleet_tasks.py  — the fleet task auction feeding it (per robot),
                             plus one gateway taking /fleet/task_request

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
robot count. `localization`/`database_path` apply to the robot named by
`robot_id` (default robot_0); other robots in `robot_ids` get their own map
path (~/.ros/sortbots_<rid>.db) and, by default, map from empty. With
`shared_map:=true` (what run_demo.sh passes for a multi-robot run on a saved
map) every robot opens its own copy of that map instead, localizes on it, and
in resume mode then extends it — see nodes/map_handoff.py.

Each robot's RTAB-Map owns its own SLAM at `<rid>/map`, unrelated to any
other robot's. This file anchors every robot into a shared `map` frame with a
static `map -> <rid>/map` transform published from that robot's known spawn
pose (configs/robots.yaml, keyed by the `scene` arg — must match
scripts/spawn_warehouse.py --scene), then runs nodes/map_merge.py once to
fuse all the anchored grids into one world-anchored `/map`. That fused `/map`
is what Nav2's global_frame and every explorer plan against by default — see
nodes/map_merge.py's docstring for the full picture.

Real robot: `platform:=real` (what scripts/run_robot.sh passes, inside
docker/jetson's container) adds the RealSense driver per robot
(sortbots_realsense.launch.py, which publishes the sim's camera topic names),
switches RTAB-Map to visual odometry with no IMU wait, and anchors each robot
with its `real` spawn entry in configs/robots.yaml. Everything else is the
same graph. run_robot.sh also turns Nav2/task_manager/scripted_pick off,
because without a base driver there is nothing for them to move.
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
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
    platform_cfg,
    slam_role_cfg,
    shared_map_cfg,
    robot_stagger_cfg,
):
    primary_robot_id = robot_id_cfg.perform(context)
    raw = robot_ids_cfg.perform(context).strip()
    robot_ids = [r.strip() for r in raw.split(",") if r.strip()] if raw else [primary_robot_id]
    platform = platform_cfg.perform(context)
    if platform not in ("sim", "real"):
        raise ValueError(f"platform:={platform!r} — expected sim or real")
    # A real robot has no warehouse scene; its anchor is the `real` spawn
    # entry. Forced rather than trusted so a forgotten scene:= can't anchor
    # hardware at a sim spawn pose (the fused /map would be offset by it).
    scene = "real" if platform == "real" else scene_cfg.perform(context)
    slam_role = slam_role_cfg.perform(context)
    if slam_role not in ("all", "robot"):
        # "workstation" is its own launch (sortbots_slam_workstation.launch.py):
        # it has no camera, no obstacles and no fleet radio to start.
        raise ValueError(f"slam_role:={slam_role!r} — expected all or robot")
    if slam_role == "robot" and platform != "real":
        raise ValueError("slam_role:=robot is for platform:=real (sim has no Jetson budget)")
    as_bool = lambda cfg: cfg.perform(context).lower() in ("true", "1")
    # Shared map: every robot opens its OWN copy of one saved map (run_demo.sh
    # makes the copies) instead of only the primary. See nodes/map_handoff.py for
    # the frame argument; in short, all their <rid>/map frames are then the
    # primary's, each robot starts read-only, and map_handoff switches an extend
    # session to mapping once that robot has localized.
    shared = as_bool(shared_map_cfg) and len(robot_ids) > 1
    localize_only = as_bool(localization)
    primary_spawn = _spawn_pose(scene, primary_robot_id)
    stagger_s = float(robot_stagger_cfg.perform(context))

    actions = []
    for idx, rid in enumerate(robot_ids):
        first = len(actions)
        # Anchors this robot's SLAM into the shared world frame: RTAB-Map
        # publishes its OWN map at <rid>/map (sortbots_rtabmap_robot.launch.py
        # sets map_frame_id), starting from wherever its odom happened to
        # init — which is this robot's known spawn pose. nodes/map_merge.py
        # looks this transform up to resample every robot's grid into one
        # `/map`; get it wrong and two robots' grids fuse at the wrong offset.
        spawn = _spawn_pose(scene, rid)
        # Shared map: the map was built in the primary's frame, so EVERY robot's
        # <rid>/map sits at the primary's spawn. Anchoring each at its own spawn
        # would draw the same shelves twice, offset by the spawns' difference.
        x, y, z = primary_spawn if shared else spawn
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

        # Real hardware: the D435 driver, publishing under the sim's topic
        # names so nothing below needs to know which platform this is.
        if platform == "real":
            actions.append(IncludeLaunchDescription(
                _include("sortbots_realsense.launch.py"),
                launch_arguments={"robot_id": rid}.items(),
            ))

        # Dashboard camera tiles: a colourised depth view (any platform, it
        # only needs camera/depth) and, on real hardware, RTAB-Map's tracked
        # keypoints as JSON for the canvas overlay (sim never runs
        # rgbd_odometry, so there is no odom_info to reduce).
        actions.append(ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "nodes", "depth_colorizer.py"),
                "--robot-id", rid,
            ],
            output="screen",
        ))
        if platform == "real":
            actions.append(ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(REPO_ROOT, "nodes", "feature_overlay.py"),
                    "--robot-id", rid,
                ],
                output="screen",
            ))

        # Obstacles in front of the robot, from geometry rather than frame
        # differencing: clusters in base_link plus a `moving` flag from odom
        # tracks (docs/obstacle_detection.md). Every platform — it reads raw
        # camera/depth, so it doesn't depend on the depth filter below, and the
        # sensors view draws its boxes over the head camera.
        actions.append(ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "nodes", "obstacle_detector.py"),
                "--robot-id", rid,
            ],
            output="screen",
        ))
        # Live obstacles from raw depth, at camera rate, in base_link: what is
        # in front of the robot NOW, independent of odometry and SLAM (and of
        # where SLAM runs). Real robot only for now — sim's Nav2 reads the
        # dynamic_obstacle_filter's clouds below.
        if platform == "real":
            actions.append(ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(REPO_ROOT, "nodes", "obstacle_cloud.py"),
                    "--robot-id", rid,
                ],
                output="screen",
            ))

        # Before RTAB-Map: strip movers from depth so SLAM never sees them.
        # Skipped on the real robot: one robot means no peer to paint into the
        # map, Nav2 (the other consumer) is off, and on the Orin Nano the
        # filter was ~1 core plus a 1.6 MB republish per frame (measured
        # 2026-10-02) — SLAM reads raw camera/depth instead.
        depth_filter = platform != "real"
        if depth_filter:
            actions.append(ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(REPO_ROOT, "nodes", "dynamic_obstacle_filter.py"),
                    "--robot-id", rid,
                ],
                output="screen",
            ))

        rtabmap_args = {"robot_id": rid, "use_sim_time": use_sim_time, "rviz": rviz,
                        "depth_filter": "true" if depth_filter else "false"}
        if platform == "real":
            # Camera-only hardware: no odom publisher and no IMU yet, so
            # RTAB-Map supplies odom itself and must not wait on /imu.
            rtabmap_args["camera_odometry"] = "true"
            rtabmap_args["wait_imu_to_init"] = "false"
            rtabmap_args["slam_role"] = slam_role
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
        #
        # localization and delete_db_on_start leak the same way, so they are now
        # set explicitly for every robot too. Before 2026-10-21 they weren't: a
        # resume run's delete_db_on_start=false reached robot_1, which then
        # resumed whatever ~/.ros/sortbots_robot_1.db earlier runs had left
        # (a stale 2.2 GB map, fused over the real one as ghost shelving).
        if shared:
            # Read-only until localized; map_handoff flips an extend run to mapping.
            rtabmap_args["localization"] = "true"
            rtabmap_args["delete_db_on_start"] = "false"
        elif rid == primary_robot_id:
            rtabmap_args["localization"] = localization
            rtabmap_args["delete_db_on_start"] = delete_db_on_start
        else:
            # Not sharing the map: a secondary robot always maps from empty.
            rtabmap_args["localization"] = "false"
            rtabmap_args["delete_db_on_start"] = "true"
        rtabmap_args["database_path"] = (database_path if rid == primary_robot_id
                                         else ["~/.ros/sortbots_", rid, ".db"])
        # Set for every robot, for the same leak reason. On a shared map it is the
        # robot's spawn in the PRIMARY's frame, which is the map's frame.
        rtabmap_args["initial_pose"] = (
            f"{spawn[0] - primary_spawn[0]:.3f} {spawn[1] - primary_spawn[1]:.3f} 0 0 0 0" if shared else "")
        actions.append(IncludeLaunchDescription(
            _include("sortbots_rtabmap_robot.launch.py"),
            launch_arguments=rtabmap_args.items(),
        ))

        if shared:
            actions.append(ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(REPO_ROOT, "nodes", "map_handoff.py"),
                    "--robot-id", rid,
                    "--then", "localize" if localize_only else "mapping",
                    *(["--explore"] if as_bool(explore) and as_bool(explore_autostart) else []),
                ],
                output="screen",
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

        # Fleet auction (commsim's allocator): wins tasks for this robot and
        # hands them to task_manager as `fleet-<id>`. Every robot bids: tasks
        # are nav-only, so any robot can finish any of them. --platform picks
        # the same waypoint set task_manager resolves against, so a bid prices
        # the pose the robot will actually drive to.
        actions.append(ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "nodes", "fleet_tasks.py"),
                "--robot-id", rid, "--platform", platform,
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
        # use_sim_time is REQUIRED, not cosmetic: every timeout in
        # configs/explorer.yaml (goal_timeout_s, stuck_window_s, blacklist
        # TTLs) is really a distance budget, and the robot covers those metres
        # in SIM time. On the wall clock they are worth ~45% of their configured
        # value — see nodes/explorer.py's _now() for the live failure that caused.
        explorer_cmd = lambda autostart: [
            "python3", os.path.join(REPO_ROOT, "nodes", "explorer.py"), "--robot-id", rid,
            *(["--autostart"] if autostart else []),
            "--ros-args", "-p", ["use_sim_time:=", use_sim_time],
        ]
        if shared:
            # Never autostart on a shared map: map_handoff sends explore_cmd=start
            # once this robot is localized (and mapping), because an unlocalized
            # robot exploring is exactly what draws a second, offset map.
            explorers = [ExecuteProcess(cmd=explorer_cmd(False), output="screen")]
        else:
            explorers = [
                ExecuteProcess(cmd=explorer_cmd(True), output="screen",
                               condition=IfCondition(explore_autostart)),
                ExecuteProcess(cmd=explorer_cmd(False), output="screen",
                               condition=UnlessCondition(explore_autostart)),
            ]
        actions.append(GroupAction(condition=IfCondition(explore), actions=explorers))

        # Each robot's stack starts robot_stagger_s after the previous one's.
        # All at once, six robots meant ~140 processes importing, discovering
        # each other over DDS and walking Nav2's lifecycle together: load
        # average 178 on 32 threads (2026-10-24), and Nav2 lifecycle managers
        # timed out on bt_navigator/controller_server and aborted: 1 robot's
        # Nav2 in one run, 2 in the next. Staggered by 8 s: 0, then 1. It
        # softens the spike but doesn't make six full stacks fit a 32-thread
        # laptop. Four come up either way (configs/tests/fleet_scale_smoke.yaml).
        if idx and stagger_s > 0:
            mine = actions[first:]
            del actions[first:]
            actions.append(TimerAction(period=idx * stagger_s, actions=mine))

    # Fleet gateway: takes /fleet/task_request, announces, keeps
    # /fleet/task_board. Once per bringup, like the operator's laptop it stands
    # in for. Sim only: a real robot's bringup is one of several on the mesh,
    # and each starting a gateway would put several wire-id-0 senders on
    # /fleet/tasks. The real gateway is the laptop (commsim/hw/README.md).
    if platform != "real":
        actions.append(ExecuteProcess(
            cmd=["python3", os.path.join(REPO_ROOT, "nodes", "fleet_tasks.py"), "--gateway",
                 "--platform", platform],
            output="screen",
            condition=IfCondition(task_manager),
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
    platform = LaunchConfiguration("platform")
    slam_role = LaunchConfiguration("slam_role")
    shared_map = LaunchConfiguration("shared_map")
    robot_stagger_s = LaunchConfiguration("robot_stagger_s")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_stagger_s",
            default_value="8.0",
            description=(
                "Seconds between one robot's stack starting and the next's (wall "
                "clock). Staggering keeps a large fleet's startup from timing out "
                "Nav2's lifecycle managers; 0 starts every robot at once."
            ),
        ),
        DeclareLaunchArgument(
            "shared_map",
            default_value="false",
            description=(
                "Every robot in robot_ids opens its own copy of the saved map "
                "(run_demo.sh makes them: ~/.ros/sortbots_<rid>.db), localizes on "
                "it, and with localization:=false then extends it. Without this "
                "only the primary uses the map and the others map from empty. "
                "Ignored for a single robot. See nodes/map_handoff.py."
            ),
        ),
        DeclareLaunchArgument(
            "platform",
            default_value="sim",
            description=(
                "sim (Isaac publishes sensors + odom) or real (RealSense "
                "driver + RTAB-Map visual odometry; scene forced to 'real'). "
                "See the module docstring."
            ),
        ),
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
            description=(
                "Start nodes/task_manager.py, the pickup->dropoff dispatch FSM, "
                "and the fleet auction that feeds it (nodes/fleet_tasks.py: one "
                "per robot plus the gateway)."
            ),
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
            "slam_role",
            default_value="all",
            description=(
                "all: odometry + RTAB-Map mapping here. robot (platform:=real "
                "only): odometry, live obstacles and a compressed keyframe "
                "stream; mapping runs on a workstation via "
                "launch/sortbots_slam_workstation.launch.py. See "
                "tasks/slam_offload.md."
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
                explore, explore_autostart, robot_id, robot_ids, scene, platform,
                slam_role,
                shared_map,
                robot_stagger_s,
            ],
        ),
    ])
