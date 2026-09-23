#!/usr/bin/env bash
set -eo pipefail
LAB_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_LOG_DIR="$LAB_ROOT/artifacts/ros_log"
export GL5_REGION_FILE="${GL5_REGION_FILE:-$LAB_ROOT/config/gl5_region.json}"
export GL5_OBSTACLE_PARAMS_FILE="${GL5_OBSTACLE_PARAMS_FILE:-$LAB_ROOT/src/gl5_detection/config/obstacles.yaml}"
# Bags by name: the ones shipped in <repo>/bags, then the user folder
# (windows/.local/bags, mounted at /windows-runtime, on Windows).
if [[ -z "${GL5_BAGS_DIR:-}" ]]; then
  if [[ -d /windows-runtime ]]; then GL5_BAGS_DIR=/windows-runtime/bags; else GL5_BAGS_DIR="$LAB_ROOT/bags"; fi
fi
bag_dirs=("$LAB_ROOT/bags" "$GL5_BAGS_DIR")
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
source "$LAB_ROOT/install/setup.bash"
params_args=()
if [[ -n "${GL5_PARAMS_FILE:-}" ]]; then
  params_args+=("params_file:=$GL5_PARAMS_FILE")
fi
# Bag playback: bash scripts/run.sh [other args] bag:=<name>
# <name> is a folder in <repo>/bags or $GL5_BAGS_DIR; an absolute path (containing /) is passed through unchanged.
launch_args=()
for arg in "$@"; do
  if [[ "$arg" == bag:=* ]]; then
    bag="${arg#bag:=}"
    if [[ -n "$bag" && "$bag" != */* ]]; then
      for dir in "${bag_dirs[@]}"; do
        if [[ -f "$dir/$bag/metadata.yaml" ]]; then bag="$dir/$bag"; break; fi
      done
    fi
    if [[ -n "$bag" && ! -f "$bag/metadata.yaml" ]]; then
      echo "Bag not found: $bag (a rosbag2 folder holding metadata.yaml is expected)" >&2
      echo "Available bags:" >&2
      for dir in "${bag_dirs[@]}"; do
        for meta in "$dir"/*/metadata.yaml; do
          [[ -f "$meta" ]] && echo "  $(basename "$(dirname "$meta")")   ($dir)" >&2
        done
      done
      exit 1
    fi
    arg="bag:=$bag"
  fi
  launch_args+=("$arg")
done
exec ros2 launch gl5_bringup gl5.launch.xml "${params_args[@]}" "${launch_args[@]}"
