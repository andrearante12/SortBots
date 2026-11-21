#!/usr/bin/env bash
# SortBots on the real robot (Jetson Orin Nano + RealSense D435) — host-side
# wrapper around the Jazzy container in docker/jetson/.
#
#   scripts/jetson.sh build     # build the image (first time ~10-20 min)
#   scripts/jetson.sh console   # start the dashboard console in the container
#   scripts/jetson.sh shell     # a shell in the running container, ROS sourced
#   scripts/jetson.sh bridge start BUNDLE   # fleet mesh bridge (docs/commsim.md step 7)
#   scripts/jetson.sh seg start # dynamic_segment's TensorRT server, on the host (stop | status)
#   scripts/jetson.sh logs      # follow the console's output
#   scripts/jetson.sh stop      # stop the robot pipeline, the console, the container
#   scripts/jetson.sh status    # container + camera + console at a glance
#   scripts/jetson.sh update    # pull main (fast-forward), rebuild if needed, restart the console
#
# On boot the console is started by sortbots-console.service, which
# scripts/jetson_host_setup.sh installs (once, with sudo).
#
# Then open http://<jetson>:8081/ (the console prints the tailnet URL when the
# host has tailscale) and start a scenario from the Scenarios tab — on the
# Jetson it lists only `platform: real` scenarios, which launch
# scripts/run_robot.sh instead of Isaac. See docs/real-robot.md.
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
    echo "         unreadable from the container. See docs/real-robot.md (host setup)."
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
    # Wait for NTP before starting anything that speaks DDS. The Orin has no RTC battery, so it
    # boots at the last saved time and NTP jumps the clock forward seconds later (40 min on
    # 2026-11-16). Fast DDS times participant liveness on the wall clock: rosbridge started
    # before the jump never discovered the robot pipeline started after it, and the page sat on
    # "Waiting for the robots" with every topic flowing inside the container. Twice.
    for _ in $(seq 120); do
      [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" == "yes" ]] && break
      [[ $_ == 1 ]] && echo "[jetson] waiting for the clock to sync (NTP) before starting the console..."
      sleep 1
    done
    [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" == "yes" ]] || \
      echo "[jetson] WARNING: clock still not NTP-synced after 2 min; starting anyway (a later jump can strand rosbridge)"
    # dynamic_seg is on by default for robot runs (webui/session.py REAL_RUN_DEFAULTS) and its
    # models run here on the host: bring the server up with the console, boot included. A
    # failure only warns: runs still start, without the movers masked.
    bash "$SCRIPT_DIR/jetson.sh" seg start || \
      echo "[jetson] WARNING: no dynamic_seg server; robot runs will log that they can't reach it"
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
    #   --device snapshot would lose it. 166/188 are ttyACM/ttyUSB: the servo
    #   boards (nodes/base_driver.py opens the wheels' by /dev/serial/by-id).
    # --user: files the pipeline writes into the repo (data/sessions, maps/)
    #   stay owned by you, not root. HOME points at the bind-mounted ~/.ros's
    #   parent so run_robot.sh's ~/.ros/sortbots_<id>.db is the host's.
    # --group-add: --user uid:gid drops supplementary groups, and the checkout is
    #   shared through the owner's group (setgid, g+w). Without them a teammate's
    #   console couldn't create data/sessions/<id> ("Permission denied" on every
    #   scenario start, seen 2026-11-21 with gilbert's console on andre's checkout).
    # ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST: the dashboard runs HERE, so no
    #   DDS traffic needs to leave the Jetson — and a desktop sim on the same
    #   LAN, which also publishes /robot_0/*, can't cross-talk with the robot.
    # FASTRTPS_DEFAULT_PROFILES_FILE: shared-memory segment big enough for a
    #   whole D435 frame (configs/dds/fastdds_large_shm.xml explains why).
    # ROS_STATIC_PEERS: the workstation that runs offloaded SLAM
    #   (SORTBOTS_WORKSTATION=<its address> scripts/jetson.sh console), the one
    #   exception to LOCALHOST discovery. Unset = robot stays private.
    docker run -d --name "$NAME" \
      --runtime nvidia \
      --network host --ipc host \
      --device-cgroup-rule 'c 189:* rmw' \
      --device-cgroup-rule 'c 81:* rmw' \
      --device-cgroup-rule "c $MEDIA_MAJOR:* rmw" \
      --device-cgroup-rule 'c 166:* rmw' \
      --device-cgroup-rule 'c 188:* rmw' \
      -v /dev:/dev \
      -v /run/udev:/run/udev:ro \
      --user "$(id -u):$(id -g)" \
      $(for g in $(id -G); do printf -- '--group-add %s ' "$g"; done) \
      -e HOME="$HOME" \
      -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      -e ROS_STATIC_PEERS="${SORTBOTS_WORKSTATION:-}" \
      -e FASTRTPS_DEFAULT_PROFILES_FILE="$REPO_ROOT/configs/dds/fastdds_large_shm.xml" \
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
    # At boot (systemd sets INVOCATION_ID), start configs/jetson.yaml's `autostart` scenario
    # once the console answers, so the robot comes up in the operator view on its own. A manual
    # `jetson.sh console` leaves the robot stack to the operator, as before.
    scenario=$(sed -n 's/^autostart:[[:space:]]*\([A-Za-z0-9_-]*\).*/\1/p' "$REPO_ROOT/configs/jetson.yaml" 2>/dev/null)
    if [[ -n "${INVOCATION_ID:-}" && -n "$scenario" ]]; then
      for _ in $(seq 60); do
        curl -fsS -o /dev/null "http://localhost:$PORT/api/session" 2>/dev/null && break
        sleep 1
      done
      if curl -fsS -X POST -H 'Content-Type: application/json' -d "{\"scenario\": \"$scenario\"}" \
           "http://localhost:$PORT/api/session/start" -o /dev/null 2>/dev/null; then
        echo "[jetson] boot: robot stack starting ($scenario)"
      else
        echo "[jetson] boot: could not start $scenario (start it from the operator view)"
      fi
    fi
    ;;

  shell)
    running || { echo "[jetson] container '$NAME' is not running:  scripts/jetson.sh console"; exit 1; }
    exec docker exec -it -w "$REPO_ROOT" "$NAME" \
      bash -c "source /opt/ros/jazzy/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; exec bash -i"
    ;;

  bridge)
    # The fleet mesh bridge, inside the container (it needs the image's
    # glibc). BUNDLE is this robot's commsim.hw.deploy folder; it must be
    # under the repo or ~/.ros, the only host paths the container sees.
    running || { echo "[jetson] container '$NAME' is not running:  scripts/jetson.sh console"; exit 1; }
    case "${2:-}" in
      start)
        [[ -n "${3:-}" ]] || { echo "usage: scripts/jetson.sh bridge start BUNDLE_DIR"; exit 2; }
        bundle=$(realpath "$3")
        [[ "$bundle" == "$REPO_ROOT"/* || "$bundle" == "$HOME/.ros"/* ]] || {
          echo "ERROR: $bundle isn't visible in the container: put it under $REPO_ROOT or ~/.ros"; exit 1; }
        exec docker exec "$NAME" "$REPO_ROOT/scripts/fleet_bridge.sh" start "$bundle" ;;
      stop|status)
        exec docker exec "$NAME" "$REPO_ROOT/scripts/fleet_bridge.sh" "$2" ;;
      *) echo "usage: scripts/jetson.sh bridge start BUNDLE_DIR | stop | status"; exit 2 ;;
    esac
    ;;

  seg)
    # dynamic_segment's TensorRT engines, on the HOST (scripts/dynamic_seg_server.py):
    # the container has no TensorRT, the host's python3 does (JetPack). cuda-python
    # goes into models/.pylib (gitignored, beside the engines) on first start.
    SEG_PID=/tmp/sortbots_seg_server.pid SEG_LOG=/tmp/sortbots_seg_server.log
    seg_up() { [[ -f "$SEG_PID" ]] && kill -0 "$(cat "$SEG_PID")" 2>/dev/null; }
    case "${2:-status}" in
      start)
        seg_up && { echo "[jetson] seg server already running (pid $(cat "$SEG_PID"))"; exit 0; }
        ls "$REPO_ROOT"/models/*.engine >/dev/null 2>&1 || {
          echo "[jetson] no engines in $REPO_ROOT/models: scripts/dynamic_seg_models.sh engines"; exit 1; }
        /usr/bin/python3 -c 'import tensorrt' 2>/dev/null || {
          echo "[jetson] the host's python3 has no tensorrt (python3-libnvinfer)"; exit 1; }
        if ! PYTHONPATH="$REPO_ROOT/models/.pylib" /usr/bin/python3 -c 'import cuda' 2>/dev/null; then
          echo "[jetson] installing cuda-python into models/.pylib"
          /usr/bin/python3 -m pip install -q --target "$REPO_ROOT/models/.pylib" "cuda-python>=12.6,<12.7" || exit 1
        fi
        PYTHONPATH="$REPO_ROOT/models/.pylib" setsid /usr/bin/python3 "$REPO_ROOT/scripts/dynamic_seg_server.py" \
          >"$SEG_LOG" 2>&1 < /dev/null &
        echo $! > "$SEG_PID"
        sleep 2
        seg_up && echo "[jetson] seg server up (pid $(cat "$SEG_PID")), log $SEG_LOG" \
               || { echo "[jetson] seg server died:"; tail -20 "$SEG_LOG"; exit 1; } ;;
      stop)
        seg_up && kill "$(cat "$SEG_PID")" && echo "[jetson] seg server stopped" || echo "[jetson] seg server not running"
        rm -f "$SEG_PID" ;;
      status)
        seg_up && echo "[jetson] seg server running (pid $(cat "$SEG_PID"))" || echo "[jetson] seg server not running"
        [[ -f "$SEG_LOG" ]] && tail -5 "$SEG_LOG" ;;
      *) echo "usage: scripts/jetson.sh seg start | stop | status"; exit 2 ;;
    esac
    ;;

  logs)
    exec docker logs -f "$NAME"
    ;;

  stop)
    if running; then
      # run_console.sh stop takes the robot pipeline down too (it calls
      # run_robot.sh stop on this platform), then the container itself.
      docker exec "$NAME" bash "$REPO_ROOT/scripts/run_console.sh" stop || true
      # The container shares the host's /dev/shm (--ipc host): Fast DDS
      # segments of killed processes outlive it and stay in RAM. Clean the
      # zombies (segments no live process holds) before the container goes.
      docker exec "$NAME" bash -c "source /opt/ros/jazzy/setup.bash; fastdds shm clean" \
        >/dev/null 2>&1 || true
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

  update)
    # The Jetson's checkout is a deploy copy of BRANCH (default main): only
    # ever fast-forwarded, never merged into, so an update can't leave it
    # half-merged with nobody at a keyboard to fix it. Uncommitted tracked
    # changes block it — the machine is shared, and a switch or pull would
    # otherwise carry someone's edits along or refuse halfway.
    # Not at boot. A boot keeps the checkout as it was (branch and commit): the
    # robot comes back running what it ran, and a branch under test isn't swapped
    # for main by a power blip (2026-11-16). Older installs of
    # sortbots-console.service still call `update` before `console`; systemd sets
    # INVOCATION_ID for its services, so that call is recognised and skipped.
    if [[ -n "${INVOCATION_ID:-}" ]]; then
      echo "[jetson] boot: keeping $(git -C "$REPO_ROOT" branch --show-current) @ $(git -C "$REPO_ROOT" log --oneline -1) (update is manual)"
      exit 0
    fi
    branch="${2:-${SORTBOTS_BRANCH:-main}}"
    cd "$REPO_ROOT" || exit 1
    if ! git diff --quiet || ! git diff --cached --quiet; then
      echo "ERROR: uncommitted changes in $REPO_ROOT — commit or stash them first:"
      git status --short --untracked-files=no
      exit 1
    fi
    # A switch over files this user can't replace half-applies: HEAD moves,
    # those files keep the old branch's content (seen 2026-11-16 — other
    # users' files in a non-setgid checkout). Refuse up front instead.
    unwritable=$(find . -path ./.git -prune -o \( -type d ! -writable \) -print -quit 2>/dev/null)
    if [[ -n "$unwritable" ]]; then
      echo "ERROR: $unwritable (and maybe more) isn't writable by $USER — fix ownership first:"
      echo "       sudo scripts/jetson_host_setup.sh   (step 7 makes the checkout group-shared)"
      exit 1
    fi
    old=$(git rev-parse HEAD)
    git fetch origin "$branch" || exit 1
    if [[ "$(git branch --show-current)" != "$branch" ]]; then
      echo "[jetson] switching $(git branch --show-current) -> $branch"
      git switch "$branch" 2>/dev/null || git switch -c "$branch" --track "origin/$branch" || exit 1
    fi
    git merge --ff-only "origin/$branch" || {
      echo "ERROR: local $branch has diverged from origin/$branch; not touching it."; exit 1; }
    new=$(git rev-parse HEAD)
    if [[ "$old" == "$new" ]]; then
      echo "[jetson] already at $(git log --oneline -1)"
    else
      echo "[jetson] $(git log --oneline -1)"
    fi
    # git lfs (maps/) only if the host has it; the console doesn't need it.
    git lfs version >/dev/null 2>&1 && git lfs pull origin >/dev/null 2>&1 || true
    if [[ "$old" != "$new" ]] && ! git diff --quiet "$old" "$new" -- docker/jetson; then
      echo "[jetson] docker/jetson changed — rebuilding $IMAGE"
      docker build -t "$IMAGE" "$REPO_ROOT/docker/jetson" || exit 1
    fi
    # Restart only what was running: the new code is on disk, but the
    # container's processes still run the old. A stopped console stays stopped.
    if running; then
      "$0" stop && "$0" console
    fi
    ;;

  *)
    sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
    [[ -z "${1:-}" || "${1:-}" == "-h" || "${1:-}" == "--help" ]] && exit 0
    exit 2
    ;;
# `update` rewrites this very file mid-run, and bash reads scripts as it goes:
# exiting on the same line as esac means nothing after the parsed case is ever
# read from the (possibly new) file.
esac; exit $?
