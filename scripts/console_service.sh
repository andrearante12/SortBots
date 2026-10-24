#!/usr/bin/env bash
# Keep the dashboard console up on THIS machine, across logouts and reboots, so
# anyone on the tailnet can open the operator view and start/stop sims from it
# (Session panel) without a terminal. A systemd USER service: it runs as you,
# from this checkout, needs no sudo, and touches nothing outside
# ~/.config/systemd/user/.
#
#     scripts/console_service.sh install     # write + enable + start the service
#     scripts/console_service.sh status      # is it up? (systemd's view + the API)
#     scripts/console_service.sh uninstall   # stop + disable + remove it
#
# The console itself costs no GPU (rosbridge, web_video_server, serve.py: ~2% of
# a core, ~180 MB). Only a running sim holds the GPU, and the page's Stop all /
# Session > Stop ends it while the console stays up.
#
# What it serves is whatever branch is checked out here: switch branches and the
# site reflects it after the next restart (systemctl --user restart
# sortbots-console). Per machine and per user: other machines, and the Jetson
# (scripts/jetson.sh console), are untouched unless someone runs this there.
#
# Linger: without it, user services only run while you're logged in. install
# tries `loginctl enable-linger`; from a remote or non-interactive shell polkit
# may refuse, and it says so — run that one command yourself.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
UNIT=sortbots-console.service
UNIT_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
PORT="${SORTBOTS_CONSOLE_PORT:-8081}"

write_unit() {
  mkdir -p "$UNIT_DIR"
  # PATH is pinned to the system one: conda's python3 leading PATH breaks every
  # `#!/usr/bin/env python3` that needs rclpy (CLAUDE.md), and a user manager's
  # environment can carry whatever the login shell exported.
  # ExecStop is `run_console.sh stop`, not just a kill: a running sim lives in its
  # own systemd scope (run_demo.sh), outside this unit's cgroup, so stopping the
  # service must take it down explicitly or Isaac would keep the GPU.
  cat >"$UNIT_DIR/$UNIT" <<EOF
[Unit]
Description=SortBots dashboard console (operator view on :$PORT, rosbridge, web_video_server)
Documentation=file://$REPO_ROOT/docs/running.md
After=network-online.target
Wants=network-online.target
StartLimitIntervalSec=300
StartLimitBurst=5

[Service]
Type=simple
WorkingDirectory=$REPO_ROOT
Environment=PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
UnsetEnvironment=AMENT_PREFIX_PATH CONDA_PREFIX
ExecStart=/bin/bash $REPO_ROOT/scripts/run_console.sh --port $PORT
ExecStop=/bin/bash $REPO_ROOT/scripts/run_console.sh stop
Restart=on-failure
RestartSec=10
TimeoutStopSec=60

[Install]
WantedBy=default.target
EOF
}

case "${1:-}" in
  install)
    # A console started by hand (sim_ctl.sh console start) holds the ports the
    # service needs; take it down first. Leaves no sim running either.
    if ss -ltnH "sport = :$PORT" 2>/dev/null | grep -q .; then
      echo "[console_service] stopping the console already on :$PORT so the service can own it"
      bash "$REPO_ROOT/scripts/run_console.sh" stop >/dev/null 2>&1 || true
      sleep 2
    fi
    write_unit
    systemctl --user daemon-reload
    systemctl --user enable --now "$UNIT" || { echo "[console_service] failed to start; see: journalctl --user -u $UNIT"; exit 1; }
    if [[ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" != "yes" ]]; then
      if loginctl enable-linger "$USER" 2>/dev/null; then
        echo "[console_service] linger on: the console starts at boot, logged in or not"
      else
        echo "[console_service] NOTE: could not enable linger from here. Run once yourself:"
        echo "                      loginctl enable-linger $USER"
        echo "                  Until then the console runs only while you're logged in."
      fi
    fi
    for _ in $(seq 1 60); do
      curl -sf -o /dev/null "http://127.0.0.1:$PORT/api/session" && { echo "[console_service] up: http://localhost:$PORT/"; exit 0; }
      sleep 1
    done
    echo "[console_service] service started but the dashboard didn't answer in 60 s; see: journalctl --user -u $UNIT"
    exit 1
    ;;
  uninstall)
    systemctl --user disable --now "$UNIT" 2>/dev/null || true
    rm -f "$UNIT_DIR/$UNIT"
    systemctl --user daemon-reload
    echo "[console_service] removed (linger, if you enabled it, is left as is: loginctl disable-linger $USER)"
    ;;
  status)
    systemctl --user --no-pager status "$UNIT" 2>&1 | head -5
    echo "linger: $(loginctl show-user "$USER" -p Linger --value 2>/dev/null)"
    if curl -sf -o /dev/null "http://127.0.0.1:$PORT/api/session"; then echo "dashboard: up on :$PORT"; else echo "dashboard: not answering on :$PORT"; fi
    ;;
  *)
    sed -n '2,23p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
