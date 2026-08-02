"""Per-robot RTAB-Map launch for the SortBots warehouse demo.

Wraps `rtabmap_launch/launch/rtabmap.launch.py` with the topic remaps
matching what `scripts/spawn_warehouse.py` publishes. Defaults to
`robot_0`; pass `robot_id:=robot_1` to map the other XLeRobot.

Run after `sudo apt install ros-jazzy-rtabmap-ros` from a shell with
system ROS 2 Jazzy sourced (NOT the Isaac venv):

    source /opt/ros/jazzy/setup.bash
    ros2 launch ./launch/sortbots_rtabmap_robot.launch.py robot_id:=robot_0

Map the warehouse, then come back and navigate on the map you built:

    # build a map (fresh DB each time)
    ros2 launch ./launch/sortbots_rtabmap_robot.launch.py
    # ...later: reopen it read-only and localize against it
    ros2 launch ./launch/sortbots_rtabmap_robot.launch.py localization:=true

The launch file lives loose under `launch/`; it's not built into a ROS 2
package because the only thing it adds over the upstream launch is the
SortBots topic remaps + an opinionated argument set.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Occupancy-grid tuning, passed to RTAB-Map as command-line parameter flags.
#
# Without these the projected /map is almost entirely -1 (unknown) even in
# rooms the robot has driven through, because with Grid/RayTracing off the
# only cells marked free are ones where a depth point actually landed on the
# ground. Frontier-based exploration is literally "a free cell adjacent to an
# unknown cell", so a map with no believable free space makes the explorer
# chase phantom frontiers a metre in front of itself.
#
#   Grid/3D=true         3D voxel local grids. This is what gives the
#                        dashboard's reconstruction panel (and the octomap_*
#                        topics) real geometry — with `false` the assembled
#                        cloud_map is the 2D grid re-emitted as points, a
#                        15 cm-tall pancake across the whole warehouse.
#
#                        It is also THIS BUILD'S OWN DEFAULT. Parameters.h
#                        (rtabmap-0.22, lines 860-864) defaults Grid/3D to
#                        true `#ifdef RTABMAP_OCTOMAP` and false otherwise,
#                        and this build has OctoMap (it advertises
#                        octomap_binary/octomap_full/octomap_grid/...).
#                        An earlier version of this comment claimed `false`
#                        was "the reason ray tracing works at all", reading
#                        Grid/RayTracing's "if Grid/3D=true, RTAB-Map should
#                        be BUILT WITH OctoMap support, otherwise 3D ray
#                        tracing is ignored" as a prohibition rather than the
#                        build precondition it is. It is satisfied here.
#
#                        The 2D /map is unaffected — Grid/3D's own docs say
#                        "a 2D map can be still generated if checked", and
#                        OccupancyGrid::assemble() transforms each cell with
#                        its true z and then keeps only x/y. What it does
#                        cost is memory and time: empty cells go from
#                        O(area) to O(volume) per node, which is why
#                        Grid/RangeMax below is now load-bearing.
#   Grid/RayTracing=true the actual fix — fills the space between the sensor
#                        and each occupied cell with free. Under Grid/3D it
#                        runs a throwaway per-node OctoMap and still emits
#                        groundCells/obstacleCells/emptyCells; emptyCells is
#                        the free-space carrier the explorer depends on.
#   Grid/RangeMax=5.0    already the upstream default; set explicitly because
#                        we depend on it. Also honest about the real D435,
#                        whose usable depth dies around 5-6 m — the sim's USD
#                        clippingRange goes to 50 m, which would otherwise let
#                        the sim map far more per frame than hardware can.
#                        Under Grid/3D=true it is the only thing bounding the
#                        ray-traced VOLUME per node, so it is also the first
#                        lever to reach for if rtabmap starts eating RAM.
#   Grid/MaxObstacleHeight=1.5
#                        default 0.0 means "disabled", i.e. everything above
#                        the robot gets projected down into the 2D map. 1.5 m
#                        keeps shelf tops and ceiling structure out of it.
#                        Note this now doubles as the reconstruction's height
#                        ceiling: raising it to ~2.5 m makes a prettier 3D
#                        panel but pushes shelf tops back into the 2D /map,
#                        so the two uses are in tension. 1.5 m is already
#                        20x the pre-Grid/3D pancake.
#   Mem/InitWMWithAllNodes=true
#                        Working Memory otherwise initializes with only the
#                        PREVIOUS session's nodes (RTAB-Map's own docs: "When
#                        false, it is initialized with nodes of the previous
#                        session") — every earlier session's nodes sit
#                        loaded-but-inactive in Long-Term Memory and only
#                        reactivate as the robot happens to revisit them.
#                        On a `--resume` run (mapping mode against an
#                        existing multi-session database) that means /map
#                        starts nearly empty even though the full graph is
#                        intact — confirmed live: WM=706 nodes total but
#                        only 3 active in the projected grid, and the
#                        explorer immediately declared "done" against that
#                        near-empty view. `true` loads the WHOLE database
#                        active at startup instead. Safe on a from-scratch
#                        mapping run too — nothing in LTM yet, so it's a
#                        no-op there. (Localization mode already sets this
#                        via upstream rtabmap.launch.py; mapping mode never
#                        did, which is the gap this closes.)
#
# Grid/Sensor is left alone: its default (1 = depth image) is already correct,
# and note it REPLACED the older Grid/FromDepth, which no longer exists.
GRID_ARGS = (
    "--Grid/3D true "
    "--Grid/RayTracing true "
    "--Grid/RangeMax 5.0 "
    "--Grid/MaxObstacleHeight 1.5 "
    "--Mem/InitWMWithAllNodes true "
    # Grid/3D floods the projected 2D map with empty-cell evidence (3D rays
    # sweep a volume; every empty voxel above/around an obstacle projects onto
    # its column), so with the default weights (hit 0.7 / miss 0.4) obstacles
    # get out-voted: measured 2026-08-01, occupied cells fell from 7,361 (2D
    # grids, ~5 min in) to 659 (3D grids, same duration) while free DOUBLED.
    # Weight hits harder and misses lighter so a wall seen a few times
    # survives the ray-traced empties stacked on top of it.
    "--GridGlobal/ProbHit 0.8 "
    "--GridGlobal/ProbMiss 0.45"
)


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    localization = LaunchConfiguration("localization")
    database_path = LaunchConfiguration("database_path")
    delete_db_on_start = LaunchConfiguration("delete_db_on_start")
    rtabmap_args = LaunchConfiguration("rtabmap_args")

    # RTAB-Map takes --delete_db_on_start as a flag inside `args`, so it can't
    # just be a bool passed through. Forced OFF in localization mode: deleting
    # the database you are about to localize against is a footgun, not a
    # preference, and it would silently turn a localization run into a mapping
    # run that starts from nothing.
    delete_db_flag = PythonExpression([
        "'--delete_db_on_start' if '", delete_db_on_start,
        "'.lower() in ('true', '1') and '", localization,
        "'.lower() not in ('true', '1') else ''",
    ])

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
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description=(
                "Open RTAB-Map's rviz view. Default true for standalone use; "
                "the unified bringup (sortbots_bringup.launch.py) sets this "
                "false because the web dashboard replaces rviz for the demo."
            ),
        ),
        DeclareLaunchArgument(
            "localization",
            default_value="false",
            description=(
                "Reopen `database_path` read-only and localize against it "
                "instead of mapping. Upstream rtabmap.launch.py implements "
                "this by setting Mem/IncrementalMemory=false + "
                "Mem/InitWMWithAllNodes=true. Nav2 needs no change either "
                "way — its static_layer consumes the same `map` topic in "
                "both modes."
            ),
        ),
        DeclareLaunchArgument(
            "database_path",
            # Per-robot, so two robots' RTAB-Map instances can't fight over one
            # file. Upstream defaults to a single shared ~/.ros/rtabmap.db,
            # which this repo was silently inheriting and appending to on every
            # run — sessions accumulated across unrelated demos with nothing
            # naming or clearing the file.
            default_value=["~/.ros/sortbots_", robot_id, ".db"],
            description="Where the map is saved to / loaded from.",
        ),
        DeclareLaunchArgument(
            "delete_db_on_start",
            default_value="true",
            description=(
                "Start mapping from an empty database. Default true so a "
                "mapping run means what it says; ignored when "
                "localization:=true."
            ),
        ),
        DeclareLaunchArgument(
            "rtabmap_args",
            default_value=GRID_ARGS,
            description=(
                "RTAB-Map command-line parameter flags. Defaults to the "
                "occupancy-grid tuning that makes /map usable for frontier "
                "exploration (see GRID_ARGS above) — overriding this replaces "
                "that tuning wholesale, so copy what you still want."
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
                "rviz":               rviz,
                # Upstream's rtabmap_viz is a SEPARATE argument from rviz and
                # defaults to true, so RTAB-Map's own Qt/OpenGL 3D viewer was
                # running even under the unified bringup, which sets rviz=false
                # precisely because the web dashboard replaces it. That was
                # merely wasteful before; with Grid/3D=true it renders the full
                # 3D cloud, and on an 8 GB card already hosting Isaac Sim the
                # contention is enough to starve Isaac's camera render products
                # — the sim keeps stepping while RGB/depth quietly stop, which
                # shows up as "SLAM: stale" and a map that freezes mid-run.
                # Tie it to the same flag so standalone use still gets a GUI.
                "rtabmap_viz":        rviz,
                "namespace":          robot_id,
                "use_sim_time":       use_sim_time,
                # Map persistence. Without these, upstream silently defaults to
                # database_path=~/.ros/rtabmap.db and localization=false, so
                # there was no way to reopen a map once built.
                "localization":       localization,
                "database_path":      database_path,
                # `args` is upstream's current name; `rtabmap_args` is its
                # deprecated alias (rtabmap.launch.py:47 defaults one to the
                # other). --delete_db_on_start is a flag *inside* this string,
                # which is why it can't be its own launch argument upstream.
                "args":               [rtabmap_args, " ", delete_db_flag],
            }.items(),
        ),

        # Keeps cloud_map/cloud_ground/cloud_obstacles/octomap_* flowing — see
        # nodes/rtabmap_cloud_pump.py's docstring for why this is needed.
        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "nodes", "rtabmap_cloud_pump.py"),
                "--robot-id", robot_id,
            ],
            output="screen",
        ),

        # Bounds what the dashboard actually pulls over rosbridge. With
        # Grid/3D=true cloud_map grows without limit as the map does, and the
        # vendored roslib can't reassemble fragments — see the relay's
        # docstring for why a point budget beats a bigger max_message_size.
        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "nodes", "recon_cloud_relay.py"),
                "--robot-id", robot_id,
            ],
            output="screen",
        ),
    ])
