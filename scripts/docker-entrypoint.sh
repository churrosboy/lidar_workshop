#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/lidar_workshop/install/setup.bash

exec "$@"
