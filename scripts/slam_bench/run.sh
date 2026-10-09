#!/usr/bin/env bash
# One SLAM bench run: replay a TUM sequence as the D435 topics, run the
# robot-side perception stack under a CPU budget, log odometry, export the
# SLAM graph, score against ground truth. Everything runs in the
# sortbots-slam-bench image (docker/slam_bench); nothing touches host ROS.
#
#   scripts/slam_bench/run.sh --seq ~/datasets/tum/rgbd_dataset_freiburg2_pioneer_360 \
#       --variant current --out /tmp/bench/current
#
# Variants (what runs where — tasks/slam_offload.md explains each):
#   launch_offload  the real launch files in slam_role robot / workstation
#   launch     the real-robot launch file as shipped (sortbots_rtabmap_robot,
#              camera_odometry:=true): odometry + full RTAB-Map on the "Jetson"
#   current    the same parameters run node by node (checks the bench itself)
#   odom_only  only odometry on the "Jetson" (what stays on the robot after offload)
#   offload    odom_only + keyframe compression on the "Jetson", RTAB-Map on the
#              "workstation" fed only by the compressed keyframe stream
#   current_nice  `current` with mapping at nice 10 (priority, no offload)
# ODOM_EXTRA="Key=value ..." / SLAM_EXTRA="--Key value ..." override RTAB-Map parameters.
#
# CPU budget: the "Jetson" containers get --cpuset-cpus $JETSON_CPUSET with a
# --cpus $JETSON_CPUS quota (calibrated in tasks/slam_offload.md so that the
# shipped odometry runs at the Orin Nano's measured speed). The player,
# logger and "workstation" run on other cores, unconstrained.
set -euo pipefail

SEQ="" VARIANT="current" OUT="" RATE=1.0
JETSON_CPUSET="${JETSON_CPUSET:-12-17}" JETSON_CPUS="${JETSON_CPUS:-2.0}"
WS_CPUSET="${WS_CPUSET:-0-7}" TOOL_CPUSET="${TOOL_CPUSET:-8-11}"
ODOM_EXTRA="${ODOM_EXTRA:-}" SLAM_EXTRA="${SLAM_EXTRA:-}"
while [ $# -gt 0 ]; do
  case "$1" in
    --seq) SEQ="$2"; shift 2 ;;
    --variant) VARIANT="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    *) echo "unknown arg $1" >&2; exit 2 ;;
  esac
done
[ -n "$SEQ" ] && [ -n "$OUT" ] || { echo "usage: $0 --seq DIR --out DIR [--variant V]" >&2; exit 2; }
SEQ="$(cd "$SEQ" && pwd)"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
rm -f "$OUT"/*.csv "$OUT"/*.db "$OUT"/*.txt "$OUT"/*.log "$OUT"/*.yaml
TAG="sb$$"
# Own DDS domain, localhost only: never cross-talk with a sim or robot on the LAN.
DOM=$(( 40 + $$ % 50 ))
ENVS=(-e ROS_DOMAIN_ID=$DOM -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e HOME=/tmp
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/repo/configs/dds/fastdds_large_shm.xml)

# Memory caps, per container, swap included (--memory-swap = --memory, so a
# leak is OOM-killed instead of swapping the laptop to a halt). An uncapped
# run shut this 15 GB laptop down on 2026-10-08. Sum stays under 8 GB: the
# robot side gets roughly the Jetson's share of its 8 GB (measured ~1 GB for
# odometry + RTAB-Map + obstacles), the "workstation" the same headroom.
declare -A MEM=([jetson]=3g [workstation]=3g [logger]=1g)
dk() {  # name cpuset quota cmd...
  local name="$1" cpuset="$2" quota="$3"; shift 3
  local q=(); [ "$quota" != "-" ] && q=(--cpus "$quota")
  local m="${MEM[$name]:-1g}"
  docker run -d --rm --name "${TAG}_$name" --network host --ipc host \
    --memory "$m" --memory-swap "$m" --cpuset-cpus "$cpuset" "${q[@]}" --user "$(id -u):$(id -g)" "${ENVS[@]}" \
    -v "$REPO:/repo:ro" -v "$SEQ:/seq:ro" -v "$OUT:/out" \
    sortbots-slam-bench bash -c "source /opt/ros/jazzy/setup.bash; $*" >/dev/null
}
cleanup() { docker ps -q --filter "name=${TAG}_" | xargs -r docker stop -t 3 >/dev/null 2>&1 || true; }
trap cleanup EXIT

# The shipped real-robot odometry parameters (launch/sortbots_rtabmap_robot.launch.py),
# as a params file: RTAB-Map's own parameters must be STRINGS, and `-p K:=2`
# on the command line types them as integers (the node aborts).
# ODOM_EXTRA="Key=value ..." adds or overrides RTAB-Map parameters.
{
  echo "/**:"
  echo "  ros__parameters:"
  echo "    frame_id: robot_0/base_link"
  echo "    odom_frame_id: robot_0/odom"
  echo "    publish_tf: true"
  echo "    approx_sync: true"
  echo "    wait_imu_to_init: false"
  declare -A RP=([Odom/ResetCountdown]=1 [Odom/ImageDecimation]=2 [Vis/MaxFeatures]=600 [OdomF2M/MaxSize]=1200)
  for kv in $ODOM_EXTRA; do RP[${kv%%=*}]="${kv#*=}"; done
  for k in "${!RP[@]}"; do echo "    $k: \"${RP[$k]}\""; done
} > "$OUT/odom_params.yaml"
ODOM_R="-r rgb/image:=/robot_0/camera/rgb -r depth/image:=/robot_0/camera/depth \
 -r rgb/camera_info:=/robot_0/camera/camera_info -r odom:=/robot_0/odom"
ODOM="ros2 run rtabmap_odom rgbd_odometry --ros-args -r __ns:=/robot_0 --params-file /out/odom_params.yaml $ODOM_R > /out/odom.log 2>&1"

# The shipped SLAM arguments: GRID_ARGS + REAL_GRID_ARGS.
SLAM_ARGS="--delete_db_on_start --Grid/3D true --Grid/RayTracing true --Grid/RangeMax 5.0 \
 --Grid/MaxObstacleHeight 1.5 --Mem/InitWMWithAllNodes true --GridGlobal/ProbHit 0.8 \
 --GridGlobal/ProbMiss 0.45 --RGBD/LinearUpdate 0.05 --RGBD/AngularUpdate 0.05 \
 --Rtabmap/DetectionRate 2 --Grid/RangeMax 4.0 $SLAM_EXTRA"
SLAM="ros2 run rtabmap_slam rtabmap $SLAM_ARGS --ros-args -r __ns:=/robot_0 \
 -p frame_id:=robot_0/base_link -p odom_frame_id:=robot_0/odom -p map_frame_id:=robot_0/map \
 -p subscribe_depth:=true -p approx_sync:=true -p wait_imu_to_init:=false \
 -p database_path:=/out/slam.db \
 -r rgb/image:=/robot_0/camera/rgb -r depth/image:=/robot_0/camera/depth \
 -r rgb/camera_info:=/robot_0/camera/camera_info -r odom:=/robot_0/odom > /out/slam.log 2>&1"
# Live obstacles stay on the robot in every variant: the thing that must not
# slow down when SLAM is busy.
OBST="python3 /repo/nodes/obstacle_cloud.py --robot-id robot_0 > /out/obstacle.log 2>&1"
# Offload link: the robot sends compressed RGB-D keyframes (JPEG rgb + PNG depth,
# RTAB-Map's own RGBDImage) at KF_RATE Hz, decimated by KF_DECIM; the
# workstation decompresses and runs SLAM on them with the robot's odometry.
KF_RATE="${KF_RATE:-2}" KF_DECIM="${KF_DECIM:-1}"
SYNC="ros2 run rtabmap_sync rgbd_sync --ros-args -r __ns:=/robot_0 -p approx_sync:=false \
 -p compressed_rate:=$KF_RATE.0 -p decimation:=$KF_DECIM \
 -r rgb/image:=/robot_0/camera/rgb -r depth/image:=/robot_0/camera/depth \
 -r rgb/camera_info:=/robot_0/camera/camera_info > /out/sync.log 2>&1"
RELAY="ros2 run rtabmap_util rgbd_relay --ros-args -r __ns:=/robot_0 -p uncompress:=true \
 -r rgbd_image:=/robot_0/rgbd_image/compressed -r rgbd_image_relay:=/robot_0/rgbd_image_ws > /out/relay.log 2>&1"
SLAM_RGBD="ros2 run rtabmap_slam rtabmap $SLAM_ARGS --ros-args -r __ns:=/robot_0 \
 -p frame_id:=robot_0/base_link -p odom_frame_id:=robot_0/odom -p map_frame_id:=robot_0/map \
 -p subscribe_depth:=false -p subscribe_rgbd:=true -p approx_sync:=true -p wait_imu_to_init:=false \
 -p database_path:=/out/slam.db -r rgbd_image:=/robot_0/rgbd_image_ws -r odom:=/robot_0/odom > /out/slam.log 2>&1"
PUMP="python3 /repo/nodes/rtabmap_cloud_pump.py --robot-id robot_0 --period 6.0 > /out/pump.log 2>&1"

case "$VARIANT" in
  launch)
    dk jetson "$JETSON_CPUSET" "$JETSON_CPUS" \
      "ros2 launch /repo/launch/sortbots_rtabmap_robot.launch.py robot_id:=robot_0 \
       camera_odometry:=true use_sim_time:=false rviz:=false depth_filter:=false \
       wait_imu_to_init:=false database_path:=/out/slam.db > /out/launch.log 2>&1 & ($OBST) & wait" ;;
  launch_offload)
    # The real launch files in their offload roles (what run_robot.sh
    # --slam-role robot + sortbots_slam_workstation.launch.py start).
    dk jetson "$JETSON_CPUSET" "$JETSON_CPUS" \
      "ros2 launch /repo/launch/sortbots_rtabmap_robot.launch.py robot_id:=robot_0 \
       slam_role:=robot camera_odometry:=true use_sim_time:=false rviz:=false \
       depth_filter:=false wait_imu_to_init:=false > /out/launch.log 2>&1 & ($OBST) & wait"
    dk workstation "$WS_CPUSET" - \
      "ros2 launch /repo/launch/sortbots_slam_workstation.launch.py robot_id:=robot_0 \
       database_path:=/out/slam.db > /out/launch_ws.log 2>&1" ;;
  current)
    dk jetson "$JETSON_CPUSET" "$JETSON_CPUS" "($ODOM) & ($SLAM) & ($PUMP) & ($OBST) & wait" ;;
  odom_only)
    dk jetson "$JETSON_CPUSET" "$JETSON_CPUS" "($ODOM) & ($OBST) & wait" ;;
  offload)
    dk jetson "$JETSON_CPUSET" "$JETSON_CPUS" "($ODOM) & ($OBST) & ($SYNC) & wait"
    dk workstation "$WS_CPUSET" - "($RELAY) & ($SLAM_RGBD) & ($PUMP) & wait" ;;
  current_nice)
    # Same work as `current`, but mapping runs at nice 10: obstacles and
    # odometry win the CPU whenever both want it.
    dk jetson "$JETSON_CPUSET" "$JETSON_CPUS" "($ODOM) & (nice -n 10 bash -c '$SLAM') & (nice -n 10 bash -c '$PUMP') & ($OBST) & wait" ;;
  *) echo "unknown variant $VARIANT" >&2; exit 2 ;;
esac

dk logger "$TOOL_CPUSET" - "python3 /repo/scripts/slam_bench/odom_logger.py --out /out > /out/logger.log 2>&1"
# Player in the foreground: the run lasts as long as the sequence.
docker run --rm --name "${TAG}_player" --network host --ipc host --memory 1g --memory-swap 1g --cpuset-cpus "$TOOL_CPUSET" \
  --user "$(id -u):$(id -g)" "${ENVS[@]}" -v "$REPO:/repo:ro" -v "$SEQ:/seq:ro" -v "$OUT:/out" \
  sortbots-slam-bench bash -c "source /opt/ros/jazzy/setup.bash; \
  python3 /repo/scripts/slam_bench/tum_player.py --seq /seq --stamps /out/stamps.csv --rate $RATE" \
  > "$OUT/player.log" 2>&1
sleep 3
# CPU actually used by the "Jetson" side, from the container's cgroup.
for c in jetson workstation; do
  id=$(docker ps -q --filter "name=${TAG}_$c")
  [ -n "$id" ] && docker stats --no-stream --format "$c {{.CPUPerc}} {{.MemUsage}}" "$id" >> "$OUT/cpu.txt" || true
done
cleanup
trap - EXIT
if [ -f "$OUT/slam.db" ]; then
  # SLAM graph: optimized base_link poses, ROS stamps. Format 10 = ROS axes;
  # format 1 converts to motion-capture/optical axes and scores the wrong frame.
  docker run --rm --memory 2g --memory-swap 2g --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$OUT:/out" sortbots-slam-bench bash -c \
    "source /opt/ros/jazzy/setup.bash; rtabmap-export --poses --poses_format 10 --output_dir /out --output slam /out/slam.db" \
    > "$OUT/export.log" 2>&1 || true
fi
python3 "$REPO/scripts/slam_bench/score.py" "$OUT" --gt "$SEQ/groundtruth.txt" \
  $( [ -f "$OUT/slam_poses.txt" ] && echo --poses "$OUT/slam_poses.txt" )
