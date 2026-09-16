#!/usr/bin/env bash
set -eo pipefail
LAB_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$LAB_ROOT/SOSLAB_SDK" -B "$LAB_ROOT/SOSLAB_SDK/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$LAB_ROOT/SOSLAB_SDK/build" -j "${BUILD_JOBS:-4}"
source /opt/ros/humble/setup.bash
cd "$LAB_ROOT"
colcon build --base-paths src --packages-up-to gl5_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
