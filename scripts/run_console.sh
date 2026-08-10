#!/usr/bin/env bash
# SortBots dashboard console — the long-lived half of the stack.
#
# Starts ONLY the things the browser talks to, and keeps them up across sim
# runs:
#   * rosbridge_websocket  (9090)  \ via launch/sortbots_webui.launch.py
#   * web_video_server     (8080)  /      with serve:=false
#   * webui/serve.py --control (8081), started here, from THIS clean shell
#
# Then open the dashboard and drive everything else from its Scenarios tab:
# picking a scenario there runs scripts/run_demo.sh for you (Isaac Sim +
# RTAB-Map + Nav2 + task_manager), and Stop tears that back down — while this
# console, and the page you're looking at, stay up.
#
# WHY THIS EXISTS. The dashboard used to be part of the pipeline: the bringup
# started rosbridge/web_video_server/serve.py, and `run_demo.sh stop` killed
# them. A "start the sim" button cannot live inside the thing it starts — the
# first click would kill the page mid-request. So the dashboard moves out here,
# and run_demo.sh learns --keep-console (which webui/session.py always passes)
# to leave these processes alone and to launch the bringup with webui:=false.
#
# WHY serve.py IS NOT IN THE LAUNCH FILE. Everything `ros2 launch` starts
# inherits a shell with ROS 2 sourced, and run_demo.sh exits 1 when
# AMENT_PREFIX_PATH is set (Isaac's venv and system ROS 2 must never share a
# shell). serve.py has to be able to spawn it, so it is started here instead,
# from the clean shell you ran this script in. webui/session.py scrubs the
# environment again on the way out as a backstop.
#
# Run it from a CLEAN terminal — do NOT `source /opt/ros/...` first:
#
#     scripts/run_console.sh                 # foreground; Ctrl-C stops the console
#     scripts/run_console.sh --port 8081
#     scripts/run_console.sh stop            # stop the console AND any running sim
#
# The plain CLI path is untouched: `scripts/run_demo.sh --explore` still works
# exactly as before with no console running (the tab then just explains that
# it can't launch anything).
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
CONSOLE_LOG="/tmp/sortbots_console.log"
PORT=8081

stop_console() {
  # The sim first (it's the noisy half), then our own processes. --keep-console
  # is deliberately NOT passed here: this path is "shut everything down".
  bash "$REPO_ROOT/scripts/run_demo.sh" stop
  pkill -INT -f "sortbots_webui.launch.py" 2>/dev/null || true
  sleep 1
  for p in rosbridge_websocket web_video_server "webui/serve.py" \
           webui_url.py web_video_watchdog.sh; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
}

if [[ "${1:-}" == "stop" ]]; then
  echo "[run_console] stopping the console and any running pipeline..."
  stop_console
  echo "[run_console] stopped."
  exit 0
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2;;
    -h|--help) sed -n '2,37p' "$0" | sed 's/^# \{0,1\}//'; exit 0;;
    *) echo "[run_console] unknown arg: $1"; exit 2;;
  esac
done

# serve.py must stay ROS-free so it can spawn run_demo.sh — same check
# run_demo.sh itself makes, for the same reason.
if [[ -n "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "ERROR: this shell has ROS 2 sourced (AMENT_PREFIX_PATH set)."
  echo "       Open a fresh terminal and run again WITHOUT sourcing ROS first."
  exit 1
fi

if ! [[ -f "$ROS_SETUP" ]]; then
  echo "ERROR: $ROS_SETUP not found — is ROS 2 Jazzy installed?"
  exit 1
fi

# Anything already holding the ports would make this look like it worked while
# the browser talked to a stale server.
if pgrep -f "webui/serve.py" >/dev/null 2>&1 || pgrep -f rosbridge_websocket >/dev/null 2>&1; then
  echo "[run_console] a console (or a bringup-owned dashboard) is already running."
  echo "              stop it first:  scripts/run_console.sh stop"
  exit 1
fi

# ---- rosbridge + web_video_server (needs ROS 2 sourced) ----
# The PATH prepend is the conda-shebang workaround documented in
# launch/sortbots_webui.launch.py: rosbridge_websocket's `#!/usr/bin/env
# python3` must resolve to the system 3.12, not conda's.
echo "[run_console] starting rosbridge + web_video_server..."
setsid bash -c "source '$ROS_SETUP'; \
  export PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin':\"\$PATH\"; \
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0; \
  exec ros2 launch '$REPO_ROOT/launch/sortbots_webui.launch.py' \
       serve:=false dashboard_port:=$PORT" \
  >"$CONSOLE_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  echo
  echo "[run_console] shutting the console down (the sim, if any, keeps running —"
  echo "              use 'scripts/run_console.sh stop' to take everything down)."
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  pkill -INT -f "sortbots_webui.launch.py" 2>/dev/null || true
  sleep 1
  for p in rosbridge_websocket web_video_server web_video_watchdog.sh; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
}
# EXIT only: SIGINT interrupts the foreground serve.py below, bash then exits
# and runs this once. Trapping INT as well would just run it twice.
trap cleanup EXIT

sleep 3
python3 "$REPO_ROOT/scripts/webui_url.py" --port "$PORT" 2>/dev/null || \
  echo "[run_console] dashboard: http://localhost:$PORT/"

cat <<EOF

============================================================
  SortBots dashboard console is UP
    * Dashboard      : http://localhost:$PORT/  (or the tailnet URL above)
    * Scenarios tab  : pick a scenario and hit Start to launch the sim
    * Session logs   : $REPO_ROOT/data/sessions/<timestamp>_<scenario>/
  Logs : $CONSOLE_LOG
  Stop : Ctrl-C here (console only), or
         scripts/run_console.sh stop  (console + sim)
============================================================

EOF

# Foreground on purpose: Ctrl-C here is how you stop the console. Deliberately
# NOT exec'd — exec would replace this shell and take the EXIT trap with it,
# leaving rosbridge and web_video_server orphaned on Ctrl-C.
python3 "$REPO_ROOT/webui/serve.py" --port "$PORT" --control
