#!/usr/bin/env bash
# Provision the commsim emulation VM (Ubuntu 24.04). Run INSIDE the VM:
#
#   sudo ./commsim/vm/provision_vm.sh
#
# Never on the host: the host is the Isaac machine (Ubuntu 25.10, no apt ROS),
# and this installs ROS 2 Jazzy system-wide plus kernel modules that need root.
# Idempotent; re-running only fills gaps.
set -euo pipefail

[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }
. /etc/os-release
[ "${VERSION_ID}" = "24.04" ] || { echo "expected Ubuntu 24.04, got ${VERSION_ID}" >&2; exit 1; }
[ -z "${AMENT_PREFIX_PATH:-}" ] || { echo "ROS is sourced in this shell; use a clean one" >&2; exit 1; }

# Pin Mininet-WiFi: its master moves and we only use hwsim + wmediumd + nodes.
MNWIFI_REF="${MNWIFI_REF:-master}"   # TODO(M0): replace with the commit that passes the M0 smoke test
MNWIFI_DIR=/opt/mininet-wifi
export DEBIAN_FRONTEND=noninteractive

apt-get update
# mac80211_hwsim ships in linux-modules-extra on Ubuntu, not the base image.
apt-get install -y "linux-modules-extra-$(uname -r)" || \
  echo "WARN: no linux-modules-extra for $(uname -r) — use a generic (not -kvm) kernel" >&2
apt-get install -y curl git iw batctl wpasupplicant tcpdump chrony iperf3 \
  python3-pip python3-venv python3-yaml python3-numpy python3-pandas

# ROS 2 Jazzy (official apt source package).
if [ ! -f /etc/apt/sources.list.d/ros2.sources ]; then
  v="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
       | grep -F tag_name | awk -F'"' '{print $4}')"
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${v}/ros2-apt-source_${v}.${UBUNTU_CODENAME}_all.deb"
  dpkg -i /tmp/ros2-apt-source.deb
  apt-get update
fi
apt-get install -y ros-jazzy-ros-base ros-jazzy-rmw-zenoh-cpp ros-jazzy-rmw-cyclonedds-cpp \
  ros-dev-tools

# Mininet-WiFi with wmediumd (-W). -l wifi deps, -n mininet deps, -f openflow, -v vswitch.
if [ ! -d "$MNWIFI_DIR/.git" ]; then
  git clone https://github.com/intrig-unicamp/mininet-wifi "$MNWIFI_DIR"
fi
git -C "$MNWIFI_DIR" fetch --quiet origin && git -C "$MNWIFI_DIR" checkout --quiet "$MNWIFI_REF"
(cd "$MNWIFI_DIR" && util/install.sh -Wlnfv)

# DES + analysis deps in a venv that can still see system rclpy.
python3 -m venv --system-site-packages /opt/commsim-venv
/opt/commsim-venv/bin/pip install --quiet simpy pyarrow pytest

# Smoke test: the two modules the emulation is built on load and unload.
modprobe mac80211_hwsim radios=2 && modprobe batman-adv
iw dev | grep -q Interface && echo "hwsim radios: OK"
batctl -v
rmmod mac80211_hwsim
echo "provisioned. next: build commsim/ws (colcon) and run the M0 smoke test."
