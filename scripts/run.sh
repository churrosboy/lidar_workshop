#!/usr/bin/env bash
set -eo pipefail
LAB_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_LOG_DIR="$LAB_ROOT/artifacts/ros_log"
export GL5_REGION_FILE="${GL5_REGION_FILE:-$LAB_ROOT/config/gl5_region.json}"
export GL5_OBSTACLE_PARAMS_FILE="${GL5_OBSTACLE_PARAMS_FILE:-$LAB_ROOT/src/gl5_detection/config/obstacles.yaml}"
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
source "$LAB_ROOT/install/setup.bash"
params_args=()
if [[ -n "${GL5_PARAMS_FILE:-}" ]]; then
  params_args+=("params_file:=$GL5_PARAMS_FILE")
fi
exec ros2 launch gl5_bringup gl5.launch.xml "${params_args[@]}" "$@"
