#!/usr/bin/env bash
# Network-isolation check: fleet traffic stays on OUR subnet, never on campus
# Wi-Fi (eduroam). Same script for the emulation (run inside a robot's
# namespace) and the hardware testbed (run on a Jetson or the operator laptop).
#
#   commsim/scripts/check_isolation.sh --subnet 10.42.0.0/24 [--configs DIR]
#
# Static half: every generated zenohd config listens/connects only inside the
# subnet (or loopback for listen), scouting off, SHM off (core/fleet.py check).
# Runtime half, if a zenohd is running here: its listening sockets and every
# established 7447 connection must be on the subnet or loopback.
# Exit 0 = clean, 1 = violation, 2 = usage.
set -euo pipefail

SUBNET="" CONFIGS="" PORT=7447 UDP_PORT=7450
while [ $# -gt 0 ]; do
  case "$1" in
    --subnet) SUBNET="$2"; shift 2 ;;
    --configs) CONFIGS="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    *) echo "usage: $0 --subnet CIDR [--configs DIR] [--port 7447]" >&2; exit 2 ;;
  esac
done
[ -n "$SUBNET" ] || { echo "--subnet is required" >&2; exit 2; }

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
rc=0

if [ -n "$CONFIGS" ]; then
  # /usr/bin/python3, not whatever leads PATH: conda's python3 can shadow it and lacks PyYAML.
  (cd "$REPO" && /usr/bin/python3 -m commsim.core.fleet check "$CONFIGS" --subnet "$SUBNET") || rc=1
fi

# UDP Status (port $UDP_PORT): the socket must be bound to the mesh IP, never
# to 0.0.0.0 — that would also accept packets on campus Wi-Fi.
udp="$(ss -Hulnp "( sport = :$UDP_PORT )" 2>/dev/null | awk '{print "udp", $4}')"
if [ -n "$udp" ]; then
  badu="$(SUBNET="$SUBNET" ADDRS="$udp" /usr/bin/python3 - <<'PY'
import ipaddress, os
net = ipaddress.ip_network(os.environ["SUBNET"])
for line in os.environ["ADDRS"].splitlines():
    kind, ep = line.split()
    host = ep.rsplit(":", 1)[0].strip("[]").split("%")[0]
    if host in ("*", "0.0.0.0", "::") or ipaddress.ip_address(host) not in net:
        print(f"status {kind} socket {ep}: not bound to the fleet subnet {net}")
PY
)"
  if [ -n "$badu" ]; then echo "$badu" | sed 's/^/VIOLATION /' >&2; rc=1; fi
fi

# ROS 2 runs the router as `rmw_zenohd` — `pgrep -x zenohd` never matched it
if pgrep -f "zenohd" >/dev/null 2>&1; then
  # Local and peer addresses of zenohd's TCP sockets, one per line.
  addrs="$( { ss -Htlnp 2>/dev/null | awk '/zenohd/ {print "listen", $4}'
              ss -Htnp state established "( sport = :$PORT or dport = :$PORT )" 2>/dev/null \
                | awk '{print "peer", $4}'; } )"
  bad="$(SUBNET="$SUBNET" ADDRS="$addrs" /usr/bin/python3 - <<'PY'
import ipaddress, os
net = ipaddress.ip_network(os.environ["SUBNET"])
for line in os.environ["ADDRS"].splitlines():
    kind, ep = line.split()
    host = ep.rsplit(":", 1)[0].strip("[]").split("%")[0]
    if host in ("*", "0.0.0.0", "::"):
        print(f"{kind} {ep}: zenohd bound to every interface")
        continue
    a = ipaddress.ip_address(host)
    if not (a in net or a.is_loopback):
        print(f"{kind} {ep}: outside {net}")
PY
)"
  if [ -n "$bad" ]; then
    echo "$bad" | sed 's/^/VIOLATION /' >&2
    rc=1
  fi
else
  echo "no zenohd running here; runtime TCP check skipped" >&2
fi

[ $rc -eq 0 ] && echo "isolation OK ($SUBNET)"
exit $rc
