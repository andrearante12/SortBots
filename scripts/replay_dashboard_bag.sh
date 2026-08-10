#!/usr/bin/env bash
# Replay a recorded dashboard bag through the REAL web stack — rosbridge,
# web_video_server and webui/serve.py — with no Isaac Sim running.
#
# This is the "eyeball it in a real browser" path. For fast automated checks
# use the fixture instead:  node webui/tests/dashboard_test.mjs
#
# It's also how you verify fixture fidelity: the messages the browser receives
# here come through the real rosbridge serializer, which is the same
# `extract_values` that scripts/bag_to_fixture.py uses to build the fixture.
#
# Run from a shell with system ROS 2 Jazzy sourced and conda OFF the front of
# PATH — rosbridge_websocket's `#!/usr/bin/env python3` shebang will otherwise
# grab conda's interpreter and die on `rclpy._rclpy_pybind11`. See the header of
# launch/sortbots_webui.launch.py for the full explanation.
#
#     source /opt/ros/jazzy/setup.bash
#     which python3      # must print /usr/bin/python3
#     scripts/replay_dashboard_bag.sh data/bags/dashboard_20260719_120000/
#
# Then open http://localhost:8081/. Ctrl-C stops everything.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOOP="--loop"
RATE="1.0"
BAG=""

usage() {
    echo "usage: $(basename "$0") [--no-loop] [--rate X] <bag-dir>"
    echo
    echo "  --no-loop   play the bag once instead of looping"
    echo "  --rate X    playback rate multiplier (default: ${RATE})"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-loop) LOOP="";        shift ;;
        --rate)    RATE="$2";      shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *)         BAG="$1";       shift ;;
    esac
done

if [[ -z "${BAG}" ]]; then
    echo "error: no bag given" >&2
    usage >&2
    exit 2
fi
if [[ ! -d "${BAG}" ]]; then
    echo "error: no such bag directory: ${BAG}" >&2
    exit 2
fi
if ! command -v ros2 >/dev/null 2>&1; then
    echo "error: 'ros2' not found. Run: source /opt/ros/jazzy/setup.bash" >&2
    exit 1
fi

PIDS=()
cleanup() {
    echo
    echo "shutting down..."
    for pid in "${PIDS[@]}"; do kill "${pid}" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "=== bag ==="
ros2 bag info "${BAG}" || true

echo
echo "starting the dashboard stack (rosbridge :9090, web_video_server :8080, serve.py :8081)..."
ros2 launch "${REPO_ROOT}/launch/sortbots_webui.launch.py" &
PIDS+=($!)

# Give rosbridge and web_video_server time to bind before messages start
# arriving — web_video_server in particular only advertises the image topics it
# has already seen, so playing the bag too early leaves the camera panels empty.
sleep 5

echo
echo "playing ${BAG} (rate ${RATE}${LOOP:+, looping})..."
echo "open http://localhost:8081/  —  Ctrl-C to stop"
echo
# No --clock: nothing here needs a /clock source (there's no node doing TF
# lookups or waiting on sim time), and web_video_server is deliberately kept on
# wall clock — see the note in launch/sortbots_webui.launch.py.
# shellcheck disable=SC2086
ros2 bag play "${BAG}" --rate "${RATE}" ${LOOP} &
PIDS+=($!)

wait
