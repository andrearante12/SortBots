#!/usr/bin/env bash
# Drop-in for `bash` in every "source ROS in its own subshell" call site:
#
#   setsid scripts/ros_bash.sh -c "source /opt/ros/jazzy/setup.bash; exec ros2 launch ..."
#
# With ROS 2 Jazzy installed on the host this is a plain `exec bash` — no
# behaviour change. Without it (Jazzy only ships apt packages for Ubuntu 24.04,
# so a 26.04 host can't have it), the same command runs inside a 24.04
# distrobox that does:
#
#   distrobox create --name jazzy --image ubuntu:24.04 --nvidia
#
# distrobox shares the host's PID, network and IPC namespaces, $HOME and the
# repo path, so the in-box ROS stack is indistinguishable from a host one to
# everything else: DDS (shared-memory transport included) reaches Isaac on the
# host, the dashboard ports bind on the host, and `pgrep -f` / `pkill -f` from
# the host see in-box processes under their real PIDs. SORTBOTS_ROS_BOX picks
# the box; default "jazzy".
set -u

ROS_SETUP="/opt/ros/jazzy/setup.bash"
BOX="${SORTBOTS_ROS_BOX:-jazzy}"

if [[ -f "$ROS_SETUP" ]]; then
  exec bash "$@"
fi

if ! command -v distrobox >/dev/null 2>&1; then
  echo "ros_bash: $ROS_SETUP not found and distrobox isn't installed —" >&2
  echo "          install ROS 2 Jazzy, or set up the '$BOX' box (docs/quickstart.md §4)." >&2
  exit 127
fi

# Callers start us as `setsid ... &` from non-interactive bash, which launches
# async jobs with SIGINT/SIGQUIT IGNORED — and bash can't trap a signal that
# was ignored on entry, so the relay below would silently never fire. Re-exec
# once with the defaults restored (2026-10-06: caught by a killpg test that
# left the in-box process orphaned).
if [[ -z "${_ROS_BASH_SIGRESET:-}" ]]; then
  _ROS_BASH_SIGRESET=1 exec env --default-signal=INT,QUIT "$0" "$@"
fi
unset _ROS_BASH_SIGRESET

# `podman exec` does NOT forward signals: SIGINT to this process (run_console's
# `kill -INT $LAUNCH_PID`, session.py's killpg on the rosbag recorder) would
# stop the client and orphan the in-box process — and rosbag2 only indexes
# its file on a clean SIGINT. So the in-box side setsid's into its own process
# group, writes its pgid here, and we relay signals to that group by hand; with
# a shared PID namespace the in-box pgid is a valid host pgid.
PGID_FILE="$(mktemp "${TMPDIR:-/tmp}/ros_bash.XXXXXX")"
trap 'rm -f "$PGID_FILE" "$PGID_FILE.go"' EXIT

# -T: no tty. Every non-interactive call site redirects stdout to a log, and a
# tty would also put the in-box process in the foreground job of a terminal it
# doesn't own. The interactive teleop window still gets one.
#
# The podman client also gets its own session: left in our process group, a
# killpg reaches it directly, it exits at once, and the in-box process loses
# its stdout mid-shutdown — rosbag2's final log lines (and its exit) then hit a
# broken pipe. Kept apart, the client lives until the in-box side exits.
TTY_FLAG=(-T) DETACH=(setsid)
[[ -t 0 && -t 1 ]] && TTY_FLAG=() DETACH=()

# The in-box side waits for "$PGID_FILE.go" before exec'ing the real command,
# so the cgroup move below lands before it forks anything (children inherit
# their parent's cgroup at fork; nothing moves them afterwards). Bounded at
# ~10 s so a host side that never answers can't wedge the launch.
"${DETACH[@]}" distrobox enter "${TTY_FLAG[@]}" "$BOX" -- \
  setsid --wait bash -c 'echo $$ >"$0"
    for _ in $(seq 200); do [[ -e "$0.go" ]] && break; sleep 0.05; done
    exec bash "$@"' "$PGID_FILE" "$@" &
CHILD=$!

relay() {
  local sig="$1" pgid=""
  pgid="$(cat "$PGID_FILE" 2>/dev/null)"
  if [[ -n "$pgid" ]]; then
    kill -s "$sig" -- "-$pgid" 2>/dev/null || true
  else
    kill -s "$sig" "$CHILD" 2>/dev/null || true
  fi
}
for sig in INT TERM HUP; do
  trap "relay $sig" "$sig"
done

# `podman exec` starts the in-box process in the CONTAINER's cgroup
# (.../libpod-<id>.scope/container), not ours — so run_demo.sh's resource
# fence (a capped systemd scope it re-execs into) silently covered Isaac but
# not the ROS stack. 2026-10-19: rtabmap reached 13 GB and was killed by the
# GLOBAL OOM killer from inside libpod's cgroup while the fence sat unused.
# With a shared PID namespace the in-box pid is a host pid, and both cgroups
# sit under this user's delegated systemd tree, so we can simply move it into
# whatever cgroup we're in. Best effort: if the move is refused, say so and
# carry on unfenced rather than fail the launch.
for _ in $(seq 600); do [[ -s "$PGID_FILE" ]] && break; sleep 0.05; done
INBOX_PID="$(cat "$PGID_FILE" 2>/dev/null)"
OUR_CG="/sys/fs/cgroup$(cut -d: -f3- /proc/self/cgroup 2>/dev/null)"
if [[ -n "$INBOX_PID" && -w "$OUR_CG/cgroup.procs" ]]; then
  echo "$INBOX_PID" >"$OUR_CG/cgroup.procs" 2>/dev/null || \
    echo "ros_bash: couldn't move in-box pid $INBOX_PID into ${OUR_CG#/sys/fs/cgroup} — running outside its resource limits" >&2
fi
touch "$PGID_FILE.go"

# `wait` returns early (>128) whenever a trapped signal arrives; keep waiting
# until the child has actually gone so the exit status is the real one.
while :; do
  wait "$CHILD"
  status=$?
  kill -0 "$CHILD" 2>/dev/null || break
done
exit "$status"
