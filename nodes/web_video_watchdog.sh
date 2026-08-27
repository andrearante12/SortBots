#!/usr/bin/env bash
# Auto-restarts web_video_server when it wedges.
#
# web_video_server (jazzy) recurrently livelocks while serving the dashboard's
# MJPEG streams: CPU climbs to ~90%, it still accepts TCP connections but
# never handles the request or sends a byte, and SIGINT is ignored (needs
# SIGKILL). Observed on 3 separate instances within ~5-20 min of streaming,
# with and without use_sim_time, so it's not config — it's the workload
# (streams being opened/aborted as dashboard tabs load, retry, and close).
#
# This probes the lightweight /snapshot endpoint every PROBE_SEC; two
# consecutive zero-byte responses = wedged -> SIGKILL + relaunch. The webui's
# <img> retry logic (webui/app.js wireCameraStream) then reconnects open tabs
# automatically: the kill turns their silent hang into an error event, which
# triggers the 3s retry against the fresh instance.
set -u

PORT=8080
TOPIC="${1:-/robot_0/camera/rgb}"
PROBE_SEC=10
# Resolved via the ROS 2 environment this script inherits from `ros2 launch`
# (sortbots_webui.launch.py), not hardcoded to /opt/ros/jazzy: that path
# doesn't exist on a from-source install (questing has no binary Jazzy
# packages — see scripts/install_ros2_jazzy.sh). A hardcoded path here meant
# every restart silently failed with "No such file or directory" and the
# watchdog looped forever instead of ever bringing the server back
# (diagnosed 2026-08-27).
SERVER_BIN="$(ros2 pkg prefix web_video_server)/lib/web_video_server/web_video_server"

fails=0
while true; do
  # qos_profile=sensor_data, same as the dashboard (webui/app.js): the probe
  # makes web_video_server open a fresh subscription to an Isaac image topic,
  # and a RELIABLE one can back-pressure Isaac's image writer — measured live
  # 2026-08-01, this probe alone (every 10s, no dashboard open) preceded the
  # rgb/depth publishers dying at sim-t~100-140s on every run, while camera_info
  # (different publisher, never probed) survived. BEST_EFFORT cannot block the
  # writer, so the probe stays harmless no matter how web_video_server behaves.
  bytes=$(curl -s --max-time 4 -o /dev/null -w '%{size_download}' \
    "http://localhost:${PORT}/snapshot?topic=${TOPIC}&qos_profile=sensor_data" 2>/dev/null || echo 0)
  if [ "${bytes:-0}" -gt 0 ]; then
    fails=0
  else
    fails=$((fails + 1))
    echo "[watchdog] probe failed (${fails}/2, ${bytes:-0} bytes)"
  fi
  if [ "$fails" -ge 2 ]; then
    echo "[watchdog] web_video_server wedged — restarting"
    pkill -9 -f "web_video_server/web_video_server" 2>/dev/null
    sleep 1
    "$SERVER_BIN" --ros-args -p port:="$PORT" -p use_sim_time:=false &
    fails=0
    sleep 5   # grace period for startup before the next probe
  fi
  sleep "$PROBE_SEC"
done
