# SortBots robot image for the Jetson: ROS 2 Jazzy on Ubuntu 24.04 userspace,
# running on the JetPack 6 (L4T R36, Ubuntu 22.04) host kernel.
#
# WHY A CONTAINER. JetPack 6 is Ubuntu 22.04, whose ROS 2 is Humble; the
# desktop sim track is Jazzy, and launch files / message types / scripts all
# assume Jazzy (ROS_SETUP=/opt/ros/jazzy/setup.bash in several places). An
# in-place Ubuntu upgrade breaks the L4T BSP, so the robot gets Jazzy here
# instead and the host keeps NVIDIA's supported 22.04. The host's own
# ros-humble-* packages are never used by SortBots.
#
# Built and run by scripts/jetson.sh — see docs/jetson.md.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# realsense2_camera pulls ros-jazzy-librealsense2, which is built with the
# V4L2 backend: it drives the D435 through the HOST kernel's uvcvideo nodes
# (/dev/video*, /dev/media*), which scripts/jetson.sh passes in. JetPack's
# stock uvcvideo is enough for color + depth; it lacks the RealSense metadata
# patches, so per-frame metadata (exact hardware timestamps) is unavailable
# and the driver falls back to system time — fine for RTAB-Map at 30 Hz.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-realsense2-camera \
      ros-jazzy-rosbridge-suite \
      ros-jazzy-web-video-server \
      ros-jazzy-rtabmap-ros \
      ros-jazzy-navigation2 \
      ros-jazzy-depth-image-proc \
      ros-jazzy-imu-filter-madgwick \
      ros-jazzy-tf2-tools \
      python3-yaml \
      python3-numpy \
      python3-qrcode \
      sqlite3 \
      procps \
      usbutils \
      curl \
      unzip \
      ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# zenoh-bridge-ros2dds: carries /fleet/* (and nothing else) between this
# robot's local DDS and the mesh — scripts/fleet_bridge.sh, run here via
# `scripts/jetson.sh bridge`. In the image, not on the host, because the
# binary needs glibc 2.38 and JetPack 6 has 2.35. Version and checksum match
# scripts/fleet_bridge.sh.
ARG ZENOH_BRIDGE_VERSION=1.10.1
ARG ZENOH_BRIDGE_SHA256=fdb64d942d4b6beccbe9b1f8a359a01a2fca95a023c27535bf97b233603fad5f
RUN zip=zenoh-plugin-ros2dds-${ZENOH_BRIDGE_VERSION}-aarch64-unknown-linux-gnu-standalone.zip \
    && curl -fsSL -o /tmp/$zip \
       https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_BRIDGE_VERSION}/$zip \
    && echo "${ZENOH_BRIDGE_SHA256}  /tmp/$zip" | sha256sum -c - \
    && unzip -q /tmp/$zip -d /opt/zenoh-bridge-ros2dds \
    && rm /tmp/$zip

# The image's ros_entrypoint.sh sources /opt/ros/jazzy/setup.bash, which sets
# AMENT_PREFIX_PATH — and scripts/run_console.sh exits 1 when that is set
# (serve.py must stay ROS-free so it can spawn the pipeline launcher; see that
# script's header). Every SortBots script sources ROS itself, in a subshell.
ENTRYPOINT []
CMD ["bash"]
