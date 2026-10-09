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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
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

# Appended to GRID_ARGS ONLY on the real robot (camera_odometry:=true), never in
# sim. Measured live 2026-10-02 on the Orin Nano (load average ~16 on 6 cores):
# the board is CPU-bound, so every default here is a trade between map density
# and the per-node cost of building it.
#   RGBD/LinearUpdate, RGBD/AngularUpdate  0.1 -> 0.05  a node is added every
#       5 cm / ~3 deg instead of 10 cm / ~6 deg, so slow walks still lay down
#       a dense cloud instead of one sparse slice per 10 cm (tasks/
#       real_perception_plan.md L4).
#   Rtabmap/DetectionRate  1 -> 2 Hz  the cap that otherwise throttles the
#       node rate regardless of the two thresholds above.
#   Grid/RangeMax  5.0 -> 4.0  D435 depth is noisy past ~4 m and Grid/3D ray
#       tracing cost grows with the swept volume; the filter's own
#       max_range_m was already 4.5.
REAL_GRID_ARGS = (
    "--RGBD/LinearUpdate 0.05 "
    "--RGBD/AngularUpdate 0.05 "
    "--Rtabmap/DetectionRate 2 "
    "--Grid/RangeMax 4.0"
)


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    localization = LaunchConfiguration("localization")
    database_path = LaunchConfiguration("database_path")
    delete_db_on_start = LaunchConfiguration("delete_db_on_start")
    rtabmap_args = LaunchConfiguration("rtabmap_args")
    camera_odometry = LaunchConfiguration("camera_odometry")
    depth_filter = LaunchConfiguration("depth_filter")
    wait_imu_to_init = LaunchConfiguration("wait_imu_to_init")
    slam_role = LaunchConfiguration("slam_role")
    keyframe_rate = LaunchConfiguration("keyframe_rate")
    keyframe_decimation = LaunchConfiguration("keyframe_decimation")

    def role_is(*roles):
        return PythonExpression([
            "'", slam_role, "' in (", ", ".join(f"'{r}'" for r in roles), ")"])

    # Mapping on the robot itself yields the CPU to odometry and the live
    # obstacle cloud: on the Orin Nano the three compete, and the bench
    # (tasks/slam_offload.md) measured obstacle latency p99 going from 16 ms to
    # 170-220 ms when RTAB-Map shared an unpinned CPU budget with them.
    # Real robot mapping on itself only — sim has a whole desktop to spare,
    # and a workstation running slam_role:=workstation has nothing to yield to.
    slam_prefix = PythonExpression([
        "'nice -n 10' if '", camera_odometry, "'.lower() in ('true', '1') and '",
        slam_role, "' == 'all' else ''",
    ])

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

    # Real-only extras (REAL_GRID_ARGS) ride on camera_odometry, the same flag
    # that already means "this is the real robot" in this file. Later flags
    # win in RTAB-Map's CLI parsing, so they override GRID_ARGS' Grid/RangeMax.
    real_extra_args = PythonExpression([
        "'", REAL_GRID_ARGS, "' if '", camera_odometry,
        "'.lower() in ('true', '1') else ''",
    ])
    # SLAM's depth: the mover-stripped depth_static, or raw camera/depth when
    # the filter isn't running (single real robot — see sortbots_bringup).
    depth_source = PythonExpression([
        "'/camera/depth_static' if '", depth_filter,
        "'.lower() in ('true', '1') else '/camera/depth'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "depth_filter",
            default_value="true",
            description=(
                "True: RTAB-Map reads camera/depth_static from "
                "nodes/dynamic_obstacle_filter.py. False: raw camera/depth "
                "(the filter must then not be started — sortbots_bringup "
                "does that for platform:=real). The filter costs ~1 core and "
                "republishes a 1.6 MB image per frame; with no peer robots "
                "and Nav2 off there is nothing for it to protect."
            ),
        ),
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
        # NOT named "visual_odometry": that is also an argument of upstream's
        # rtabmap.launch.py, which the include below always sets to false,
        # and IncludeLaunchDescription shares one launch context (see the
        # rgb_topic_relay comment) — so a same-named arg of ours was silently
        # overwritten and the odometry Node's condition read false. Seen live
        # 2026-09-30: no rgbd_odometry process, no /odom publisher, no error.
        DeclareLaunchArgument(
            "camera_odometry",
            default_value="false",
            description=(
                "Run rgbd_odometry on RAW depth (see the Node below), which "
                "then PUBLISHES /<robot_id>/odom and <robot_id>/odom -> "
                "base_link. False in "
                "sim, where Isaac publishes perfect odom. True on the real "
                "robot until a base driver publishes wheel/optical odom — two "
                "odom sources would fight over the same TF edge, so flip this "
                "back off the day nodes/base_driver.py exists."
            ),
        ),
        DeclareLaunchArgument(
            "wait_imu_to_init",
            default_value="true",
            description=(
                "Block RTAB-Map's init on the first /<robot_id>/imu sample. "
                "True in sim (IMU at ~60 Hz). MUST be false on hardware with "
                "no IMU publisher — a plain D435 has none — or RTAB-Map waits "
                "forever and the map never starts."
            ),
        ),
        DeclareLaunchArgument(
            "slam_role",
            default_value="all",
            description=(
                "Where SLAM runs (tasks/slam_offload.md). all: odometry AND "
                "RTAB-Map mapping on this machine (sim, and the robot until a "
                "workstation is in the loop). robot: only what must be on the "
                "robot — odometry plus a compressed RGB-D keyframe stream "
                "(/<id>/rgbd_image/compressed) for the workstation. "
                "workstation: RTAB-Map built from that keyframe stream and the "
                "robot's /<id>/odom; publishes the map, <id>/map -> <id>/odom "
                "and the reconstruction clouds back."
            ),
        ),
        DeclareLaunchArgument(
            "keyframe_rate",
            default_value="2.0",
            description=(
                "slam_role:=robot — Hz of the compressed keyframe stream. "
                "RTAB-Map only adds a node at Rtabmap/DetectionRate (2 Hz on "
                "real) anyway, so frames beyond that are network load for "
                "nothing. ~180 kB per 640x480 keyframe (JPEG + PNG depth)."
            ),
        ),
        DeclareLaunchArgument(
            "keyframe_decimation",
            # 2 = half resolution, a third of the bytes: 2.7 -> 0.95 Mb/s per
            # robot. Full resolution is ~27% of a 2.4 GHz mesh channel per robot
            # per hop, so two robots would cross the 50% busy line where the
            # comms model starts failing. Costs some SLAM accuracy (bench at 2x
            # speed: ATE 0.7 -> 1.15 m); set 1 on a wired or 5 GHz-only link.
            default_value="2",
            description=(
                "slam_role:=robot — image decimation before compression "
                "(1 = full resolution, ~2.7 Mb/s per robot; 2 = half, ~1 Mb/s)."
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
            condition=IfCondition(role_is("all", "workstation")),
            launch_description_source=PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("rtabmap_launch"),
                    "launch",
                    "rtabmap.launch.py",
                ])
            ),
            launch_arguments={
                "rgb_topic":          ["/", robot_id, "/camera/rgb"],
                # depth_static: nodes/dynamic_obstacle_filter.py strips movers
                # so RTAB-Map does not permanently paint peer robots into the
                # SLAM grid. Raw /camera/depth still feeds Nav2 via the filter's
                # dynamic_obstacles cloud (+ depth_static points for static).
                "depth_topic":        ["/", robot_id, depth_source],
                # Upstream rtabmap.launch.py does NOT remap rgb/image from
                # rgb_topic directly — it bakes rgb_topic_relay /
                # depth_topic_relay inside an OpaqueFunction via
                # LaunchConfiguration('rgb_topic').perform(context) and then
                # DeclareLaunchArgument's the result. IncludeLaunchDescription
                # shares the parent launch context across every robot we
                # spawn, so the SECOND robot's Include sees those relay
                # names already set by the first and silently keeps
                # robot_0's topics. Verified live 2026-08-09 on
                # explore_fleet: robot_1's rtabmap cmdline had
                #   -r rgb/image:=/robot_0/camera/rgb
                #   -r depth/image:=/robot_0/camera/depth_static
                #   -r rgb/camera_info:=/robot_1/camera/camera_info
                # (camera_info is a lazy LaunchConfiguration, not a baked
                # relay, so it alone resolved correctly). robot_1 was
                # therefore building its cloud from robot_0's cameras —
                # 3D recon froze on the dashboard for robot_1 while the
                # bots kept moving. compressed:=false (our default) means
                # relay == topic, so pass both explicitly per robot the
                # same way bringup already forces per-robot database_path.
                "rgb_topic_relay":    ["/", robot_id, "/camera/rgb"],
                "depth_topic_relay":  ["/", robot_id, depth_source],
                "camera_info_topic":  ["/", robot_id, "/camera/camera_info"],
                "imu_topic":          ["/", robot_id, "/imu"],
                # RTAB-Map waits for the first IMU sample before initializing.
                # Useful because the sim publishes IMU at ~60 Hz; without this
                # RTAB-Map's odom estimator can race the publish.
                "wait_imu_to_init":   wait_imu_to_init,
                "frame_id":           [robot_id, "/base_link"],
                "odom_frame_id":      [robot_id, "/odom"],
                "odom_topic":         ["/", robot_id, "/odom"],
                # Left unset, this defaults to the bare "map" — the same name
                # every OTHER robot's RTAB-Map would also use for its own,
                # unrelated map origin. Per-robot SLAM lives at <robot_id>/map;
                # the bare "map" frame is reserved for the world-anchored,
                # merged grid that launch/sortbots_bringup.launch.py publishes
                # a static map -> <robot_id>/map transform into (see
                # nodes/map_merge.py).
                "map_frame_id":       [robot_id, "/map"],
                # ALWAYS false upstream. Sim: Isaac publishes perfect odom.
                # Real: camera_odometry:=true starts OUR rgbd_odometry below
                # instead of upstream's, because upstream feeds its odometry
                # the same depth_topic as SLAM — the filtered depth_static —
                # and odometry cannot track on that. Measured live 2026-09-30
                # on the D435: every frame "Registration failed ... 0/20
                # inliers" for 20 min straight on depth_static (7.8 Hz, ~0.5 s
                # behind, pixels blanked frame-to-frame), while a second
                # rgbd_odometry on raw /camera/depth tracked at once (quality
                # ~180, 0 failures). SLAM keeps depth_static; odometry gets raw.
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
                # slam_role:=workstation: RTAB-Map reads the robot's compressed
                # keyframe stream instead of raw images (upstream's
                # rgbd_relay_uncompress turns <rgbd_topic>/compressed into
                # <rgbd_topic_relay>). The relay name is passed explicitly for
                # the same shared-launch-context reason as rgb_topic_relay.
                "subscribe_rgbd":     PythonExpression(["'true' if '", slam_role, "' == 'workstation' else 'false'"]),
                "compressed":         PythonExpression(["'true' if '", slam_role, "' == 'workstation' else 'false'"]),
                "rgbd_topic":         ["/", robot_id, "/rgbd_image"],
                "rgbd_topic_relay":   ["/", robot_id, "/rgbd_image_relay"],
                "launch_prefix":      slam_prefix,
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
                "args":               [rtabmap_args, " ", real_extra_args, " ", delete_db_flag],
            }.items(),
        ),

        # Visual odometry on RAW depth — see the "visual_odometry" comment in
        # the include above for why this isn't upstream's odometry node.
        # Publishes /<id>/odom and <id>/odom -> <id>/base_link, which is
        # exactly what Isaac publishes in sim, so RTAB-Map can't tell the two
        # apart.
        Node(
            condition=IfCondition(PythonExpression([
                "'", camera_odometry, "'.lower() in ('true', '1') and '",
                slam_role, "' != 'workstation'"])),
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            namespace=robot_id,
            output="screen",
            parameters=[{
                "frame_id": [robot_id, "/base_link"],
                "odom_frame_id": [robot_id, "/odom"],
                "publish_tf": True,
                "approx_sync": True,
                "wait_imu_to_init": False,
                "use_sim_time": use_sim_time,
                # Default 0 = once lost, stay lost FOREVER: the pose freezes,
                # RTAB-Map drops every frame, and the map and 3D view go stale
                # while the camera feed still looks alive. 1 = reset on the
                # next frame instead; RTAB-Map starts a new map session and
                # re-links it to the old one on the next loop closure.
                "Odom/ResetCountdown": "1",
                # CPU cost knobs (Orin Nano, 2026-10-02: odometry ran ~3 Hz and
                # ~60% of a core at the D435's 848x480). Half-resolution
                # features are ~4x cheaper and plenty for a 4 m indoor scene;
                # fewer features and a smaller local map cut matching/PnP time.
                # Defaults were 1 / 1000 / 2000.
                "Odom/ImageDecimation": "2",
                "Vis/MaxFeatures": "600",
                "OdomF2M/MaxSize": "1200",
                # Frame-to-frame, not the default frame-to-map: no local feature
                # map to match against and update, so each frame is cheaper and
                # odometry keeps more of the camera's frames on a CPU-bound Orin.
                # Bench (tasks/slam_offload.md, TUM pioneer_slam at 2x speed,
                # Jetson CPU budget, 3 runs each): 6.6 -> 9.9 Hz, odometry ATE
                # 2.4 -> 1.3 m, SLAM ATE 2.6 -> 0.7 m. At 1x speed SLAM ATE is
                # unchanged (0.19 m); short-window drift is ~1-2 points higher.
                "Odom/Strategy": "1",
            }],
            remappings=[
                ("rgb/image", ["/", robot_id, "/camera/rgb"]),
                ("depth/image", ["/", robot_id, "/camera/depth"]),
                ("rgb/camera_info", ["/", robot_id, "/camera/camera_info"]),
                ("odom", ["/", robot_id, "/odom"]),
            ],
        ),

        # slam_role:=robot — the keyframe stream the workstation maps from:
        # RTAB-Map's own RGBDImage with JPEG rgb + PNG depth, rate-capped. The
        # D435 driver stamps color and aligned depth identically, so exact sync.
        # Raw 30 Hz images never leave the robot.
        Node(
            condition=IfCondition(role_is("robot")),
            package="rtabmap_sync",
            executable="rgbd_sync",
            name="rgbd_sync",
            namespace=robot_id,
            output="screen",
            parameters=[{
                "approx_sync": False,
                "compressed_rate": keyframe_rate,
                "decimation": keyframe_decimation,
                "use_sim_time": use_sim_time,
            }],
            remappings=[
                ("rgb/image", ["/", robot_id, "/camera/rgb"]),
                ("depth/image", ["/", robot_id, "/camera/depth"]),
                ("rgb/camera_info", ["/", robot_id, "/camera/camera_info"]),
            ],
        ),

        # Keeps cloud_map/cloud_ground/cloud_obstacles/octomap_* flowing — see
        # nodes/rtabmap_cloud_pump.py's docstring for why this is needed.
        ExecuteProcess(
            condition=IfCondition(role_is("all", "workstation")),
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "nodes", "rtabmap_cloud_pump.py"),
                "--robot-id", robot_id,
                # Each publish_map re-assembles the WHOLE global cloud; on the
                # CPU-bound Jetson that is the periodic stall. The dashboard
                # throttles to 3 s anyway, so a slower pump costs little.
                "--period", PythonExpression([
                    "'6.0' if '", camera_odometry,
                    "'.lower() in ('true', '1') else '3.0'",
                ]),
            ],
            output="screen",
        ),

        # Bounds what the dashboard actually pulls over rosbridge. With
        # Grid/3D=true cloud_map grows without limit as the map does, and the
        # vendored roslib can't reassemble fragments — see the relay's
        # docstring for why a point budget beats a bigger max_message_size.
        ExecuteProcess(
            condition=IfCondition(role_is("all", "workstation")),
            cmd=[
                "python3",
                os.path.join(REPO_ROOT, "nodes", "recon_cloud_relay.py"),
                "--robot-id", robot_id,
            ],
            output="screen",
        ),
    ])
