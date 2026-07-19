#!/usr/bin/env bash
# Record a rosbag2 of every topic the web dashboard consumes, so the dashboard
# can be tested later without Isaac Sim, without a GPU and without a display.
#
# This is the ONLY step in the test-data pipeline that needs the sim running:
#
#   scripts/record_dashboard_bag.sh          # <- needs the demo up (this file)
#   scripts/bag_to_fixture.py <bag>          # offline
#   node webui/tests/dashboard_test.mjs      # offline, no ROS at all
#   scripts/replay_dashboard_bag.sh <bag>    # offline, real rosbridge + browser
#
# Run from a shell with system ROS 2 Jazzy sourced and conda OFF the front of
# PATH — the same env-hygiene rule as launch/sortbots_webui.launch.py, because
# `ros2` is a Python entry point and conda's interpreter can't load rclpy's
# compiled extension:
#
#     source /opt/ros/jazzy/setup.bash
#     which python3      # must print /usr/bin/python3
#     scripts/record_dashboard_bag.sh --duration 60
#
# While it records, DRIVE THE ROBOT and dispatch a task — a bag of a stationary
# robot with no plan and no loop closure can't exercise the trail, the planned
# path, the task queue or the SLAM badge.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_ID="${ROBOT_ID:-robot_0}"
DURATION=60
OUTPUT=""

usage() {
    sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    echo
    echo "Options:"
    echo "  --duration SEC   how long to record (default: ${DURATION})"
    echo "  --output DIR     bag directory (default: data/bags/dashboard_<timestamp>)"
    echo "  -h, --help       this message"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration) DURATION="$2"; shift 2 ;;
        --output)   OUTPUT="$2";   shift 2 ;;
        -h|--help)  usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "${OUTPUT}" ]]; then
    OUTPUT="${REPO_ROOT}/data/bags/dashboard_$(date +%Y%m%d_%H%M%S)"
fi

# Everything webui/app.js subscribes to, plus /tf(_static) for the robot pose
# and /odom as an independent cross-check on that pose. Explicit list, never
# `-a`: the sim publishes depth and camera_info streams that would balloon the
# bag without the dashboard ever reading them.
TOPICS=(
    "/${ROBOT_ID}/map"
    "/${ROBOT_ID}/global_costmap/costmap"
    "/${ROBOT_ID}/plan"
    "/${ROBOT_ID}/info"
    "/${ROBOT_ID}/cloud_map"
    "/${ROBOT_ID}/task_status"
    "/${ROBOT_ID}/camera/rgb"
    "/${ROBOT_ID}/camera/chase/rgb"
    "/${ROBOT_ID}/odom"
    "/tf"
    "/tf_static"
)

if ! command -v ros2 >/dev/null 2>&1; then
    echo "error: 'ros2' not found. Source ROS 2 Jazzy first:" >&2
    echo "    source /opt/ros/jazzy/setup.bash" >&2
    exit 1
fi

echo "checking which topics are live..."
LIVE="$(ros2 topic list 2>/dev/null || true)"
MISSING=()
for t in "${TOPICS[@]}"; do
    grep -qx -- "$t" <<<"${LIVE}" || MISSING+=("$t")
done

# A bag that's quietly missing cloud_map or the cameras looks fine until the
# fixture turns out to have empty panels, so this is loud and specific.
if [[ ${#MISSING[@]} -gt 0 ]]; then
    echo
    echo "WARNING: ${#MISSING[@]} of ${#TOPICS[@]} topics are NOT currently published:"
    printf '  - %s\n' "${MISSING[@]}"
    echo
    echo "They'll be recorded as empty. Common causes:"
    echo "  /${ROBOT_ID}/map, /info, /cloud_map  -> RTAB-Map not up (sortbots_rtabmap_robot.launch.py)"
    echo "  /${ROBOT_ID}/global_costmap/costmap, /plan -> Nav2 not up (sortbots_nav2.launch.py)"
    echo "  /${ROBOT_ID}/task_status            -> nodes/task_manager.py not running"
    echo "  /${ROBOT_ID}/camera/*               -> Isaac Sim not publishing yet"
    echo "  /plan, /task_status                 -> normal until you dispatch a task"
    echo
    read -r -p "Record anyway? [y/N] " reply
    [[ "${reply}" =~ ^[Yy]$ ]] || { echo "aborted."; exit 1; }
fi

mkdir -p "$(dirname "${OUTPUT}")"

echo
echo "recording ${#TOPICS[@]} topics for ${DURATION}s -> ${OUTPUT}"
echo "DRIVE THE ROBOT NOW (and dispatch a task) so the bag has motion in it."
echo

# zstd file compression: the raw sensor_msgs/Image streams dominate the size
# and compress well. --max-cache-size keeps memory bounded on the image topics.
ros2 bag record \
    --output "${OUTPUT}" \
    --compression-mode file \
    --compression-format zstd \
    --max-cache-size 100000000 \
    "${TOPICS[@]}" &
RECORD_PID=$!

# Stop cleanly on Ctrl-C too, so a shortened recording is still a valid bag.
trap 'kill -INT "${RECORD_PID}" 2>/dev/null || true' INT TERM
sleep "${DURATION}" || true
kill -INT "${RECORD_PID}" 2>/dev/null || true
wait "${RECORD_PID}" 2>/dev/null || true

echo
echo "=== bag info ==="
ros2 bag info "${OUTPUT}" || true

cat <<EOF

=== recorded: ${OUTPUT} ===

Check above that every topic has a non-zero message count. Then build the
committed test fixture (no ROS needed after this point):

    python3 ${REPO_ROOT}/scripts/bag_to_fixture.py ${OUTPUT}
    node ${REPO_ROOT}/webui/tests/dashboard_test.mjs

Bags live under data/, which is gitignored — only the small derived fixture
in webui/testdata/ gets committed.
EOF
