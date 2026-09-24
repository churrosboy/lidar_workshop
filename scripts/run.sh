#!/usr/bin/env bash
set -eo pipefail
LAB_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_LOG_DIR="$LAB_ROOT/artifacts/ros_log"
export GL5_REGION_FILE="${GL5_REGION_FILE:-$LAB_ROOT/config/gl5_region.json}"
export GL5_OBSTACLE_PARAMS_FILE="${GL5_OBSTACLE_PARAMS_FILE:-$LAB_ROOT/src/gl5_detection/config/obstacles.yaml}"
mkdir -p "$ROS_LOG_DIR"
source /opt/ros/humble/setup.bash
source "$LAB_ROOT/install/setup.bash"

# Bags by name: the ones in <repo>/bags (bind-mounted by mac/start-bag.sh), then the runtime folder.
bag_dirs=("$LAB_ROOT/bags" "${GL5_BAGS_DIR:-/mac-runtime/bags}")

list_bags() {
  echo "Available bags:" >&2
  local found=1 meta
  for dir in "${bag_dirs[@]}"; do
    for meta in "$dir"/*/metadata.yaml; do
      [[ -f "$meta" ]] || continue
      echo "  $(basename "$(dirname "$meta")")   ($dir)" >&2
      found=0
    done
  done
  [[ $found -eq 0 ]] || echo "  (none found in ${bag_dirs[*]})" >&2
}

# Bag playback: bash scripts/run.sh bag:=<name> [other args]
# <name> is a folder in one of bag_dirs; a path (containing /) is passed through unchanged.
launch_args=()
have_bag=1
for arg in "$@"; do
  if [[ "$arg" == bag:=* ]]; then
    have_bag=0
    bag="${arg#bag:=}"
    if [[ -n "$bag" && "$bag" != */* ]]; then
      for dir in "${bag_dirs[@]}"; do
        if [[ -f "$dir/$bag/metadata.yaml" ]]; then bag="$dir/$bag"; break; fi
      done
    fi
    if [[ ! -f "$bag/metadata.yaml" ]]; then
      echo "Bag not found: $bag (a rosbag2 folder holding metadata.yaml is expected)" >&2
      list_bags
      exit 1
    fi
    arg="bag:=$bag"
  fi
  launch_args+=("$arg")
done

# This branch plays recordings only, so there is nothing to run without a bag.
if [[ $have_bag -ne 0 ]]; then
  echo "No bag given. Usage: bash scripts/run.sh bag:=<name> [name:=value ...]" >&2
  list_bags
  exit 1
fi

exec ros2 launch gl5_bringup gl5.launch.xml "${launch_args[@]}"
