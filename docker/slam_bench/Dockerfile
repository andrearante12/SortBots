# Offline SLAM bench (x86 workstation): the Jetson image's perception
# packages, without the camera/web stack, for replaying recorded RGB-D
# sequences through RTAB-Map under a CPU budget. See tasks/slam_offload.md.
FROM ros:jazzy-ros-base
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rtabmap-ros \
      ros-jazzy-depth-image-proc \
      ros-jazzy-cv-bridge \
      ros-jazzy-image-transport-plugins \
      ros-jazzy-rmw-zenoh-cpp \
      python3-yaml python3-numpy python3-opencv \
      sqlite3 procps \
    && rm -rf /var/lib/apt/lists/*
ENTRYPOINT []
CMD ["bash"]
