#!/usr/bin/env bash
set -eo pipefail
LAB_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_LOG_DIR="$LAB_ROOT/artifacts/ros_log"
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
source "$LAB_ROOT/install/setup.bash"
cd "$LAB_ROOT"
python3 tools/verify_topics.py "$@"
