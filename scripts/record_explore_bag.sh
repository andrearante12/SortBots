#!/usr/bin/env bash
# Record a rosbag2 of an autonomous exploration run — map, TF, odom, explorer
# decisions, task/costmap/plan state — for later review or replay.
#
# Deliberately NOT scripts/record_dashboard_bag.sh: that one targets a short
# (~60s) fixture-building recording and includes full-rate camera/rgb +
# camera/chase/rgb, which is fine at 60s but not at exploration-run length
# (minutes to tens of minutes). Raw sensor_msgs/Image at ~60 Hz is roughly
# 70-170 MB/s uncompressed depending on stream — a 10 minute exploration run
# would be tens of GB. This script omits camera/rgb, camera/depth and
# cloud_map entirely; scripts/record_explore_bag.sh's caller is expected to
# grab periodic /snapshot JPEGs separately (cheap, decimated) if visual
# reference is wanted — see the --frames-dir option.
#
# Unlike record_dashboard_bag.sh, this is NON-INTERACTIVE (no "topics
# missing, record anyway?" prompt) and does not block for a fixed duration —
# it records until SIGINT/SIGTERM, so a caller can stop it exactly when the
# explorer reports "exploration done" (see nodes/explorer.py's status topic).
#
# Run from a shell with system ROS 2 Jazzy sourced (NOT the Isaac venv):
#
#     source /opt/ros/jazzy/setup.bash
#     scripts/record_explore_bag.sh --output data/runs/<name>/bag
#     # ... Ctrl-C when done, or send SIGINT from another process ...
set -euo pipefail

ROBOT_ID="${ROBOT_ID:-robot_0}"
OUTPUT=""

usage() {
    sed -n '2,25p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    echo
    echo "Options:"
    echo "  --output DIR   bag directory (required)"
    echo "  -h, --help     this message"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output) OUTPUT="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "${OUTPUT}" ]]; then
    echo "error: --output DIR is required" >&2
    usage >&2
    exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "error: 'ros2' not found. Source ROS 2 Jazzy first:" >&2
    echo "    source /opt/ros/jazzy/setup.bash" >&2
    exit 1
fi

# Low-bandwidth topics only: map/TF/odom/status/decisions, not raw sensor
# streams. camera_info is tiny (a few hundred bytes, low rate) and useful
# later for reprojecting any separately-saved snapshot frames.
TOPICS=(
    "/clock"
    "/${ROBOT_ID}/map"
    "/${ROBOT_ID}/global_costmap/costmap"
    "/${ROBOT_ID}/plan"
    "/${ROBOT_ID}/info"
    "/${ROBOT_ID}/task_status"
    "/${ROBOT_ID}/explore_status"
    "/${ROBOT_ID}/explore_cmd"
    "/${ROBOT_ID}/frontiers"
    "/${ROBOT_ID}/odom"
    "/${ROBOT_ID}/cmd_vel"
    "/${ROBOT_ID}/camera/camera_info"
    "/explore/claims"
    "/tf"
    "/tf_static"
)

mkdir -p "$(dirname "${OUTPUT}")"

echo "recording ${#TOPICS[@]} topics -> ${OUTPUT} (Ctrl-C or SIGINT to stop)"
ros2 bag record \
    --output "${OUTPUT}" \
    --compression-mode file \
    --compression-format zstd \
    --max-cache-size 100000000 \
    "${TOPICS[@]}"
