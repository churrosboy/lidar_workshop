#!/usr/bin/env bash
set -eo pipefail
LAB_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$LAB_ROOT"
colcon build --base-paths src --packages-up-to gl5_bringup --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
