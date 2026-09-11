#!/usr/bin/env bash
set -eo pipefail
LAB_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_LOG_DIR="$LAB_ROOT/artifacts/ros_log"
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
ACTION="${1:-status}"
case "$ACTION" in
  edit|undo|finish|cancel|clear|save|load)
    exec timeout 10 ros2 service call "/gl5/region/$ACTION" std_srvs/srv/Trigger '{}'
    ;;
  status)
    exec timeout 10 ros2 topic echo /gl5/obstacle_state --once --qos-durability transient_local
    ;;
  *) echo "Usage: bash scripts/region.sh {edit|undo|finish|cancel|clear|save|load|status}" >&2; exit 2 ;;
esac
