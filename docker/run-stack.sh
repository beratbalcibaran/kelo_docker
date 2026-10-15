#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
set -u
set -m

SOURCE="${KELO_SOURCE:-hardware}"
MODE="${KELO_MODE:-navigation}"
case "$MODE" in production) MODE=navigation ;; hardware) MODE=sensors ;; esac
ODOM="${KELO_ODOM_SOURCE:-robot}"
MAP="${KELO_MAP:-/data/maps/current.yaml}"
BAG="${KELO_BAG:-}"
INITIAL="${KELO_INITIAL_POSE:-}"
EXTRA="${KELO_EXTRA_ARGS:-}"
PID=""

cleanup() {
  echo "[kelo] shutting down..."
  if [ -n "$PID" ] && kill -INT "$PID" 2>/dev/null; then
    for _ in $(seq 1 100); do kill -0 "$PID" 2>/dev/null || break; sleep 0.1; done
    kill -TERM "$PID" 2>/dev/null
  fi
  wait
  exit 0
}
trap cleanup TERM INT

if [ "$MODE" = navigation ]; then
  while [ ! -f "$MAP" ]; do
    echo "[kelo] ERROR: no map: $MAP (map first with KELO_MODE=mapping). Waiting..."
    sleep 60 & wait $!
  done
fi
if [ "$SOURCE" = replay ] && [ ! -e "$BAG" ]; then
  echo "[kelo] ERROR: KELO_SOURCE=replay requires KELO_BAG=<bag directory> (not found: '$BAG')"; exit 2
fi

ARGS=(source:="$SOURCE" mode:="$MODE" odom_source:="$ODOM" map:="$MAP")
[ -n "$BAG" ] && ARGS+=(bag:="$BAG")
[ -n "$INITIAL" ] && ARGS+=(initial_pose:="$INITIAL")
echo "[kelo] ros2 launch kelo_bringup bringup.launch.py ${ARGS[*]} $EXTRA"
ros2 launch kelo_bringup bringup.launch.py "${ARGS[@]}" $EXTRA &
PID=$!
wait "$PID"
echo "[kelo] launch ended, container exiting"
