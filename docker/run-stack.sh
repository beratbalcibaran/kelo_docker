#!/bin/bash
source /opt/ros/jazzy/setup.bash
# in the image; the native install (sim_e2e_test.sh E2E_NATIVE=1) has sourced its workspace already
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
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

# automatic start on the robot (KELO_AUTO=1, default): wait until the robot driver answers on the cable,
# write its settings (topics, odom_mode, ROS_DOMAIN_ID) and only then start. Nothing on the robot computer
# is changed: the driver must be running there.
ROBOT_ENV="${KELO_ROBOT_ENV:-/data/robot.env}"
load_robot_env() { [ -f "$ROBOT_ENV" ] && { set -a; . "$ROBOT_ENV"; set +a; echo "[kelo] robot settings: $(tr '\n' ' ' < "$ROBOT_ENV")"; }; }
load_robot_env
if [ "$SOURCE" = hardware ] && [ "${KELO_AUTO:-1}" = 1 ]; then
  n=0
  until ros2 run kelo_bringup robot_discover --apply --env-file "$ROBOT_ENV"; do
    n=$((n + 1))
    echo "[kelo] automatic: robot driver not found yet (cable in? driver started on the robot computer?), try $n, again in 10 s"
    sleep 10 & wait $!
  done
  load_robot_env
  echo "[kelo] automatic: robot found, starting $MODE"
fi

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
