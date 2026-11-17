#!/usr/bin/env bash
# One-time Jetson host setup so the robot comes back by itself after any
# reboot or power cut: on Wi-Fi, on the tailnet, and serving the dashboard.
#
#   sudo scripts/jetson_host_setup.sh [WIFI_CONNECTION]   # default: eduroam
#
# Idempotent — rerun it after pulling a new version. It never touches Wi-Fi
# credentials: the NetworkManager profile must already exist and work.
# What it installs, and the failure each piece prevents (all seen 2026-11-16):
#
#  1. Wi-Fi profile: retry forever. NM's default gives up after 4 failed
#     attempts and leaves the robot offline until someone reboots it.
#  2. Wi-Fi power save off. With it on, an idle Jetson stops answering
#     inbound SSH/Tailscale until it sends something itself.
#  3. Persistent journal. The default is RAM-only, so the logs of whatever
#     took the robot down vanish with the reboot that brings it back.
#  4. tailscaled after network-online. Started before Wi-Fi, its first login
#     fails and backs off; with the clock-jump of a board that has no RTC
#     battery (it boots at the last saved time, then NTP jumps it) the robot
#     sat off the tailnet ~2 min after Wi-Fi was already up.
#  5. sortbots-net-watchdog.timer: every minute, re-up Wi-Fi if it's down,
#     restart tailscaled if it has been offline two checks running.
#  6. sortbots-console.service: `scripts/jetson.sh console` at boot as the
#     repo's owner, so http://<jetson>:8081/ is up without anyone logging in,
#     on whatever branch and commit are checked out. Boot never pulls: a power
#     blip must not swap a branch under test for main (2026-11-16). Update by
#     hand with `scripts/jetson.sh update [BRANCH]`.
#  8. sortbots-efi-watchdog.service: about 1 boot in 4 (2026-11-16), a call into the board's
#     UEFI firmware (efi_rts_wq) never returns. It pins a CPU in the kernel, tailscaled hangs
#     unkillably behind it, and the console waits on tailscaled, so the robot looks alive (LED,
#     fan, Wi-Fi) but is unreachable. The service follows the kernel log for that one signature
#     and forces a reboot (sysrq b; with the firmware lock held, the kernel resets through PSCI
#     instead), logging each one to /var/lib/sortbots/efi-resets.log first. The reboot watchdog
#     also drops to 60 s: a reboot stuck the same way waited out systemd's 10 min default.
#  7. The checkout is shared by every user (all in the owner's group), but
#     without setgid dirs each user's new files land in THEIR group: 247 of
#     liam's files were unwritable to andre, and a branch switch half-applied
#     — HEAD moved, those files didn't. Group-own everything, g+rwX, setgid
#     dirs, and core.sharedRepository so .git stays shared too.
set -euo pipefail

[[ $(id -u) == 0 ]] || { echo "run with sudo: sudo $0 $*"; exit 1; }
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OWNER="$(stat -c %U "$REPO_ROOT")"
WIFI="${1:-eduroam}"
WIFI_DEV="$(nmcli -t -f DEVICE,TYPE dev | awk -F: '$2=="wifi"{print $1; exit}')"

echo "[setup] repo $REPO_ROOT (owner $OWNER), wifi '$WIFI' on ${WIFI_DEV:-?}"

# 1 + 2 ----------------------------------------------------------------------
nmcli -t -f NAME con show | grep -qx "$WIFI" || {
  echo "ERROR: no NetworkManager connection '$WIFI' — create and test it first"; exit 1; }
nmcli con modify "$WIFI" connection.autoconnect yes connection.autoconnect-retries 0 \
  connection.autoconnect-priority 10 802-11-wireless.powersave 2
cat > /etc/NetworkManager/conf.d/99-sortbots-wifi-powersave.conf <<'EOF'
[connection]
wifi.powersave = 2
EOF
[[ -n "$WIFI_DEV" ]] && iw dev "$WIFI_DEV" set power_save off 2>/dev/null || true

# 3 --------------------------------------------------------------------------
mkdir -p /etc/systemd/journald.conf.d /var/log/journal
cat > /etc/systemd/journald.conf.d/sortbots-persistent.conf <<'EOF'
[Journal]
Storage=persistent
SystemMaxUse=300M
EOF
systemd-tmpfiles --create --prefix /var/log/journal
systemctl restart systemd-journald
# A restart alone keeps writing to /run until the next boot; flush moves it now.
journalctl --flush

# 4 --------------------------------------------------------------------------
systemctl enable NetworkManager-wait-online.service >/dev/null 2>&1 || true
mkdir -p /etc/systemd/system/tailscaled.service.d
cat > /etc/systemd/system/tailscaled.service.d/sortbots-wait-online.conf <<'EOF'
[Unit]
Wants=network-online.target
After=network-online.target time-sync.target
EOF

# 5 --------------------------------------------------------------------------
cat > /usr/local/sbin/sortbots-net-watchdog <<EOF
#!/bin/bash
# Installed by $REPO_ROOT/scripts/jetson_host_setup.sh — edit that, not this.
WIFI='$WIFI'; DEV='$WIFI_DEV'; FLAG=/run/sortbots-net-watchdog.offline
if ! nmcli -t -f DEVICE,STATE dev | grep -q "^\$DEV:connected\$"; then
    # dev connect, not con up \$WIFI: NM picks the best saved network in range,
    # so a robot carried somewhere without \$WIFI still joins the others.
    logger -t net-watchdog "\$DEV not connected, asking NetworkManager to reconnect"
    nmcli radio wifi on
    timeout 60 nmcli dev connect "\$DEV" >/dev/null 2>&1 || true
    exit 0
fi
iw dev "\$DEV" set power_save off 2>/dev/null || true
# Two offline checks in a row before acting: one can be a login in progress.
if tailscale status --json 2>/dev/null | python3 -c 'import sys,json; sys.exit(0 if json.load(sys.stdin)["Self"].get("Online", True) else 1)'; then
    rm -f "\$FLAG"
elif [[ -e "\$FLAG" ]]; then
    logger -t net-watchdog "tailscale offline for 2 checks with wifi up, restarting tailscaled"
    rm -f "\$FLAG"; systemctl restart tailscaled
else
    touch "\$FLAG"
fi
EOF
chmod 755 /usr/local/sbin/sortbots-net-watchdog
cat > /etc/systemd/system/sortbots-net-watchdog.service <<'EOF'
[Unit]
Description=SortBots: keep Wi-Fi and Tailscale up
After=NetworkManager.service tailscaled.service
[Service]
Type=oneshot
ExecStart=/usr/local/sbin/sortbots-net-watchdog
EOF
cat > /etc/systemd/system/sortbots-net-watchdog.timer <<'EOF'
[Unit]
Description=SortBots: network watchdog every 30 s
[Timer]
OnBootSec=45
OnUnitActiveSec=30
[Install]
WantedBy=timers.target
EOF

# 6 --------------------------------------------------------------------------
# oneshot + RemainAfterExit: jetson.sh console starts a detached container and
# returns. Docker owns the process from there; the unit just says "started".
cat > /etc/systemd/system/sortbots-console.service <<EOF
[Unit]
Description=SortBots dashboard console (docker container 'sortbots')
Wants=network-online.target tailscaled.service
After=network-online.target docker.service tailscaled.service
Requires=docker.service
[Service]
Type=oneshot
RemainAfterExit=yes
User=$OWNER
WorkingDirectory=$REPO_ROOT
ExecStart=$REPO_ROOT/scripts/jetson.sh console
ExecStop=$REPO_ROOT/scripts/jetson.sh stop
TimeoutStartSec=120
[Install]
WantedBy=multi-user.target
EOF

# 7 --------------------------------------------------------------------------
GROUP="$(stat -c %G "$REPO_ROOT")"
chgrp -R "$GROUP" "$REPO_ROOT"
chmod -R g+rwX "$REPO_ROOT"
find "$REPO_ROOT" -type d -exec chmod g+s {} +
sudo -u "$OWNER" git -C "$REPO_ROOT" config core.sharedRepository group

# 8 --------------------------------------------------------------------------
mkdir -p /var/lib/sortbots /etc/systemd/system.conf.d
cat > /usr/local/sbin/sortbots-efi-watchdog <<'WDOG'
#!/bin/bash
# Installed by scripts/jetson_host_setup.sh (step 8): edit that, not this.
# Waits for the kernel's RCU stall report naming efi_rts_wq (a UEFI runtime call that never
# returned), then forces a reboot. Nothing else triggers it.
dmesg --follow --nopager 2>/dev/null | grep -m1 -q "efi_rts_wq" || exit 0
logger -p user.crit -t efi-watchdog "UEFI runtime call stuck (efi_rts_wq RCU stall): forcing a reboot"
echo "$(date -Is) uptime $(cut -d' ' -f1 /proc/uptime)s: efi_rts_wq stall, forced reboot" >> /var/lib/sortbots/efi-resets.log
journalctl --sync 2>/dev/null; sync
echo s > /proc/sysrq-trigger; sleep 1
echo b > /proc/sysrq-trigger
WDOG
chmod 755 /usr/local/sbin/sortbots-efi-watchdog
cat > /etc/systemd/system/sortbots-efi-watchdog.service <<'UNIT'
[Unit]
Description=SortBots: reboot when a UEFI runtime call hangs (efi_rts_wq stall)
After=local-fs.target systemd-journald.service
[Service]
Type=simple
ExecStart=/usr/local/sbin/sortbots-efi-watchdog
Restart=on-failure
[Install]
WantedBy=multi-user.target
UNIT
cat > /etc/systemd/system.conf.d/sortbots-watchdog.conf <<'CONF'
[Manager]
# A reboot stuck in the firmware waited out the 10 min default before the hardware reset it.
RebootWatchdogSec=60s
CONF

systemctl daemon-reload
systemctl reload NetworkManager
systemctl enable docker tailscaled >/dev/null
systemctl enable --now sortbots-net-watchdog.timer
systemctl enable sortbots-console.service
systemctl enable --now sortbots-efi-watchdog.service
systemctl daemon-reexec   # picks up RebootWatchdogSec
# Start it now too, unless a console is already up (jetson.sh console is a
# no-op then, so this is safe either way).
systemctl start sortbots-console.service || echo "WARNING: console didn't start — journalctl -u sortbots-console"

echo "[setup] done:"
echo "  wifi     : $(nmcli -g connection.autoconnect,connection.autoconnect-retries con show "$WIFI" | tr '\n' ' ')(autoconnect, retries 0=forever)"
echo "  journal  : $(ls -d /var/log/journal/*/ >/dev/null 2>&1 && echo persistent || echo 'NOT persistent yet')"
echo "  watchdog : $(systemctl is-active sortbots-net-watchdog.timer)"
echo "  console  : $(systemctl is-enabled sortbots-console.service), $(systemctl is-active sortbots-console.service)"
echo "  efi wdog : $(systemctl is-active sortbots-efi-watchdog.service) (forced reboots so far: $(wc -l < /var/lib/sortbots/efi-resets.log 2>/dev/null || echo 0))"
echo "Also: Tailscale admin console → Machines → $(hostname) → Disable key expiry."
