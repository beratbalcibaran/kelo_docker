#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
export KELO_DATA_DIR="${KELO_DATA_DIR:-/data}"
mkdir -p "$KELO_DATA_DIR/maps" "$KELO_DATA_DIR/bags"
IFACE="${KELO_DDS_IFACE:-}"
if [ -n "$IFACE" ]; then
  if [ "$(cat /sys/class/net/"$IFACE"/operstate 2>/dev/null)" = up ]; then
    export CYCLONEDDS_URI="${CYCLONEDDS_URI},<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$IFACE\"/></Interfaces></General></Domain></CycloneDDS>"
    echo "[kelo] DDS interface: $IFACE"
  else
    echo "[kelo] WARNING: $IFACE is not up (cable unplugged?); DDS uses automatic interface selection, robot will not be visible"
  fi
fi
exec "$@"
