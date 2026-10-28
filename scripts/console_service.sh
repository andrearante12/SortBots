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
#     scripts/console_service.sh deploy      # host origin/main from its own checkout, kept current
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
# deploy: the PROD form. The service above serves whatever branch is checked out in
# the shared working tree, so a teammate's `git checkout` changes the public site.
# `deploy` instead keeps a dedicated worktree of main (~/SortBots-prod, override with
# SORTBOTS_PROD_DIR), points the service at it, and installs a timer that fast-forwards
# it every 5 min. It restarts the console only when main moved AND no sim is running
# (a restart runs ExecStop, which takes a live sim down); otherwise it waits for the
# next tick. Run it once after this lands on main; nothing else is needed.
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
PROD_DIR="${SORTBOTS_PROD_DIR:-$HOME/SortBots-prod}"
TIMER=sortbots-console-deploy

# The checkout the unit runs from: $1, else this script's own repo.
write_unit() {
  local REPO_ROOT="${1:-$REPO_ROOT}"
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
# ExecStop kills serve.py (run_console.sh stop), so the main shell exits 137/143:
# that is a clean stop, not a failure (it read "failed" in status, 2026-10-24).
SuccessExitStatus=130 137 143
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
    systemctl --user disable --now "$TIMER.timer" 2>/dev/null || true
    systemctl --user disable --now "$UNIT" 2>/dev/null || true
    rm -f "$UNIT_DIR/$UNIT" "$UNIT_DIR/$TIMER.service" "$UNIT_DIR/$TIMER.timer"
    systemctl --user daemon-reload
    echo "[console_service] removed (linger, if you enabled it, is left as is: loginctl disable-linger $USER)"
    ;;
  status)
    systemctl --user --no-pager status "$UNIT" 2>&1 | head -5
    echo "linger: $(loginctl show-user "$USER" -p Linger --value 2>/dev/null)"
    if curl -sf -o /dev/null "http://127.0.0.1:$PORT/api/session"; then echo "dashboard: up on :$PORT"; else echo "dashboard: not answering on :$PORT"; fi
    ;;
  deploy)
    auto=0; [[ "${2:-}" == "--auto" ]] && auto=1
    if [[ ! -e "$PROD_DIR/.git" ]]; then
      git -C "$REPO_ROOT" fetch -q origin main || { echo "[deploy] fetch failed"; exit 1; }
      git -C "$REPO_ROOT" worktree add "$PROD_DIR" main || exit 1
    fi
    before="$(git -C "$PROD_DIR" rev-parse HEAD)"
    git -C "$PROD_DIR" fetch -q origin main || { echo "[deploy] fetch failed; leaving prod as is"; exit 1; }
    # ff-only: a prod checkout with local commits is a mistake to surface, not paper over.
    git -C "$PROD_DIR" merge -q --ff-only origin/main || { echo "[deploy] $PROD_DIR can't fast-forward to origin/main"; exit 1; }
    after="$(git -C "$PROD_DIR" rev-parse HEAD)"
    if [[ ! -x "$PROD_DIR/scripts/console_service.sh" ]]; then
      echo "[deploy] main has no console_service.sh yet: merge the console-service PR first"; exit 1
    fi
    # Unit already runs from prod and main didn't move: nothing to do (the timer's usual tick).
    if grep -q "WorkingDirectory=$PROD_DIR\$" "$UNIT_DIR/$UNIT" 2>/dev/null && [[ "$before" == "$after" ]]; then
      [[ $auto == 1 ]] || echo "[deploy] already serving main @ ${after:0:7}"
      exit 0
    fi
    # Liveness is pgrep, never a pid (CLAUDE.md).
    if pgrep -f spawn_warehouse.py >/dev/null; then
      echo "[deploy] a sim is running; a restart would stop it. Deferring until it ends."; exit 5
    fi
    write_unit "$PROD_DIR"
    cat >"$UNIT_DIR/$TIMER.service" <<EOF
[Unit]
Description=Fast-forward the hosted SortBots console to origin/main
[Service]
Type=oneshot
Environment=PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ExecStart=/bin/bash $PROD_DIR/scripts/console_service.sh deploy --auto
EOF
    cat >"$UNIT_DIR/$TIMER.timer" <<EOF
[Unit]
Description=Every 5 min: host origin/main
[Timer]
OnBootSec=2min
OnUnitActiveSec=5min
[Install]
WantedBy=timers.target
EOF
    systemctl --user daemon-reload
    systemctl --user enable --now "$TIMER.timer"
    systemctl --user restart "$UNIT" || { echo "[deploy] restart failed; see: journalctl --user -u $UNIT"; exit 1; }
    echo "[deploy] serving main @ ${after:0:7} from $PROD_DIR"
    ;;
  *)
    sed -n '2,32p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
