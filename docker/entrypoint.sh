#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
export KELO_DATA_DIR="${KELO_DATA_DIR:-/data}"
mkdir -p "$KELO_DATA_DIR/maps" "$KELO_DATA_DIR/bags"
exec "$@"
