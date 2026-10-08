#!/usr/bin/env bash
# SortBots on the real robot (Jetson Orin Nano + RealSense D435) — host-side
# wrapper around the Jazzy container in docker/jetson/.
#
#   scripts/jetson.sh build     # build the image (first time ~10-20 min)
#   scripts/jetson.sh console   # start the dashboard console in the container
#   scripts/jetson.sh shell     # a shell in the running container, ROS sourced
#   scripts/jetson.sh logs      # follow the console's output
#   scripts/jetson.sh stop      # stop the robot pipeline, the console, the container
#   scripts/jetson.sh status    # container + camera + console at a glance
#
# Then open http://<jetson>:8081/ (the console prints the tailnet URL when the
# host has tailscale) and start a scenario from the Scenarios tab — on the
# Jetson it lists only `platform: real` scenarios, which launch
# scripts/run_robot.sh instead of Isaac. See docs/jetson.md.
#
# Run it on the HOST, from a plain shell — it only needs docker. Never source
# the host's /opt/ros/humble for SortBots: everything ROS runs in the container.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE="${SORTBOTS_IMAGE:-sortbots-jetson:jazzy}"
NAME="${SORTBOTS_CONTAINER:-sortbots}"
PORT="${SORTBOTS_DASHBOARD_PORT:-8081}"

running() { [[ "$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)" == "true" ]]; }

# The dashboard URL to open from anywhere: the host's MagicDNS name on the
# tailnet, else the LAN hostname. Plain http on purpose — see
# scripts/webui_url.py for why the 3-port dashboard can't sit behind https.
dashboard_url() {
  local dns
  dns=$(tailscale status --json 2>/dev/null | python3 -c \
    "import json,sys; print(json.load(sys.stdin)['Self']['DNSName'].rstrip('.'))" 2>/dev/null)
  echo "http://${dns:-$(hostname)}:$PORT/"
}

# Tailscale is the dashboard's ONLY perimeter (serve.py --control is an
# unauthenticated process launcher bound to 0.0.0.0), so share the host's
# tailscale CLI + daemon socket into the container read-only: that is what
# lets scripts/webui_url.py print the tailnet URL from in there. The binary is
# statically linked, so the host's copy runs fine on the container's 24.04.
TAILSCALE_MOUNTS=()
if [[ -x /usr/bin/tailscale && -S /var/run/tailscale/tailscaled.sock ]]; then
  TAILSCALE_MOUNTS=(-v /usr/bin/tailscale:/usr/bin/tailscale:ro
                    -v /var/run/tailscale:/var/run/tailscale:ro)
fi

preflight() {
  command -v docker >/dev/null || { echo "ERROR: docker not installed"; exit 1; }
  docker info >/dev/null 2>&1 || {
    echo "ERROR: can't talk to docker — is $USER in the docker group (re-login after adding)?"
    exit 1
  }
  docker image inspect "$IMAGE" >/dev/null 2>&1 || {
    echo "ERROR: image $IMAGE not built yet:  scripts/jetson.sh build"
    exit 1
  }
  # Without these, libusb in the container can open the D435 only as root:
  # the rules are what make /dev/bus/usb/*/* mode 0666 for Intel's VID.
  if [[ ! -f /etc/udev/rules.d/99-realsense-libusb.rules && \
        ! -f /lib/udev/rules.d/99-realsense-libusb.rules ]]; then
    echo "WARNING: no 99-realsense-libusb.rules on the host — the camera may be"
    echo "         unreadable from the container. See docs/jetson.md (host setup)."
  fi
  # One process owns the camera. A host-side Humble realsense node (from
  # ros-humble-realsense2-camera) grabbing it first makes the container's
  # driver fail with "failed to set power state" / "Device or resource busy".
  if pgrep -f realsense2_camera_node >/dev/null 2>&1 && ! running; then
    echo "ERROR: a realsense2_camera_node is already running on the host and"
    echo "       owns the D435. Stop it first (pkill -f realsense2_camera_node)."
    exit 1
  fi
}

case "${1:-}" in
  build)
    exec docker build -t "$IMAGE" "$REPO_ROOT/docker/jetson"
    ;;

  console)
    preflight
    if running; then
      echo "[jetson] container '$NAME' is already running:  scripts/jetson.sh logs"
      exit 0
    fi
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    mkdir -p "$HOME/.ros"
    MEDIA_MAJOR=$(awk '$2 == "media" {print $1}' /proc/devices)
    [[ -n "$MEDIA_MAJOR" ]] || { echo "ERROR: no 'media' major in /proc/devices — uvcvideo not loaded?"; exit 1; }
    # --network host: the dashboard's three ports (8080/8081/9090) are served
    #   straight from the host, and webui_url.py sees the real interfaces.
    # --ipc host: Fast DDS shared-memory transport between the nodes.
    # /dev bind + device-cgroup rules: the ROS-packaged librealsense on Jazzy
    #   is built with the V4L2 backend (NOT RSUSB — checked 2026-09-28: its
    #   .so carries v4l_uvc_device and "No RealSense devices were found!" with
    #   only /dev/bus/usb passed). It opens the host kernel's uvcvideo nodes,
    #   /dev/video* (major 81) and /dev/media* (dynamic major, read from
    #   /proc/devices), plus usb (189) for resets. Binding all of /dev rather
    #   than --device per node is what survives re-enumeration: the D435
    #   drops and reappears on every hardware reset the driver issues, and a
    #   --device snapshot would lose it.
    # --user: files the pipeline writes into the repo (data/sessions, maps/)
    #   stay owned by you, not root. HOME points at the bind-mounted ~/.ros's
    #   parent so run_robot.sh's ~/.ros/sortbots_<id>.db is the host's.
    # ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST: the dashboard runs HERE, so no
    #   DDS traffic needs to leave the Jetson — and a desktop sim on the same
    #   LAN, which also publishes /robot_0/*, can't cross-talk with the robot.
    docker run -d --name "$NAME" \
      --runtime nvidia \
      --network host --ipc host \
      --device-cgroup-rule 'c 189:* rmw' \
      --device-cgroup-rule 'c 81:* rmw' \
      --device-cgroup-rule "c $MEDIA_MAJOR:* rmw" \
      -v /dev:/dev \
      -v /run/udev:/run/udev:ro \
      --user "$(id -u):$(id -g)" \
      -e HOME="$HOME" \
      -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      -e SORTBOTS_PLATFORM=real \
      -v "$HOME/.ros:$HOME/.ros" \
      -v "$REPO_ROOT:$REPO_ROOT" \
      "${TAILSCALE_MOUNTS[@]}" \
      -w "$REPO_ROOT" \
      "$IMAGE" \
      bash "$REPO_ROOT/scripts/run_console.sh" --port "$PORT" >/dev/null || exit 1
    echo "[jetson] console starting in container '$NAME'."
    echo "         dashboard: $(dashboard_url)   logs: scripts/jetson.sh logs"
    echo "         (from any device signed in to the same Tailscale tailnet)"
    ;;

  shell)
    running || { echo "[jetson] container '$NAME' is not running:  scripts/jetson.sh console"; exit 1; }
    exec docker exec -it -w "$REPO_ROOT" "$NAME" \
      bash -c "source /opt/ros/jazzy/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; exec bash -i"
    ;;

  logs)
    exec docker logs -f "$NAME"
    ;;

  stop)
    if running; then
      # run_console.sh stop takes the robot pipeline down too (it calls
      # run_robot.sh stop on this platform), then the container itself.
      docker exec "$NAME" bash "$REPO_ROOT/scripts/run_console.sh" stop || true
    fi
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    echo "[jetson] stopped."
    ;;

  status)
    if running; then echo "container : $NAME running"; else echo "container : not running"; fi
    if lsusb 2>/dev/null | grep -qi "8086:0b07"; then echo "camera    : D435 on USB"
    else echo "camera    : D435 NOT found on USB"; fi
    if curl -fsS -o /dev/null "http://localhost:$PORT/api/scenarios" 2>/dev/null; then
      echo "dashboard : $(dashboard_url)"
    else
      echo "dashboard : not answering on :$PORT"
    fi
    ;;

  *)
    sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
    [[ -z "${1:-}" || "${1:-}" == "-h" || "${1:-}" == "--help" ]] && exit 0
    exit 2
    ;;
esac
