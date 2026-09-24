FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive
ARG BUILD_JOBS=4

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    file \
    git \
    python3-colcon-common-extensions \
    python3-matplotlib \
    python3-numpy \
    python3-scipy \
    python3-yaml \
    qtbase5-dev \
    ros-humble-interactive-markers \
    ros-humble-rviz2 \
    x11vnc \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/lidar_workshop
COPY . .

RUN BUILD_JOBS="${BUILD_JOBS}" bash scripts/build.sh

COPY scripts/docker-entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
