#!/usr/bin/env bash
# The robot computer, from the Jetson: copies robot/ there and controls its kelo_robot container
# (platform driver and joystick) over ssh.
#
#   robot.sh deploy            copy robot/ to ~/kelo_robot on the robot computer
#   robot.sh time              set the robot computer's clock from the Jetson (asks for its sudo password)
#   robot.sh status            robot computer, container, driver, joystick, joypad
#   robot.sh up                start the container (first time: also builds the driver)
#   robot.sh down              stop joystick, driver and container
#   robot.sh start [args]      start the driver; launch arguments are passed on, e.g. fix_odom_rotation:=false
#   robot.sh stop              stop joystick and driver (the container stays up)
#   robot.sh restart [args]    stop, then start
#   robot.sh joy [stop]        joystick straight to the driver (LB deadman, A speed level, RB stop).
#                              Refused if anything else publishes /cmd_vel, such as the stack
#   robot.sh joy safe          joystick with the stack on, through its collision monitor and velocity gate
#   robot.sh log [driver|joy]  last lines of the output
#   robot.sh build             colcon build in the container (after deploy; then restart)
#   robot.sh reset             reset the wheels' error flags
#
# KELO_HOST overrides the address (default kelo@192.168.1.102).
# Only one driver at a time: it does not start if another container on the robot computer runs one.

set -euo pipefail

KELO_HOST=${KELO_HOST:-kelo@192.168.1.102}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SSH=(ssh -o ConnectTimeout=4 -o BatchMode=yes "$KELO_HOST")

case "${1:-}" in
  deploy|time|status|up|down|start|stop|restart|joy|log|build|reset) ;;
  *) sed -n '5,18p' "$0" | sed 's/^# //'; exit 2 ;;
esac

if [ "$1" = time ]; then
  # no chrony here: this changes nothing that stays on the robot computer after a reboot
  ssh -t -o ConnectTimeout=4 "$KELO_HOST" "sudo date -s @$(date +%s.%N)"
  echo "[robot] offset (robot - jetson): $(( $("${SSH[@]}" date +%s) - $(date +%s) )) s (robot_check gives the precise value)"
  exit 0
fi

if ! "${SSH[@]}" true 2> /dev/null; then
  echo "[robot] cannot reach $KELO_HOST: robot on? cable connected? ssh key copied (ssh-copy-id $KELO_HOST)?"
  exit 1
fi

if [ "$1" = deploy ]; then
  tar -C "$ROOT/robot" -cf - . | "${SSH[@]}" 'mkdir -p ~/kelo_robot && tar -C ~/kelo_robot -xf -'
  echo "[robot] copied to $KELO_HOST:~/kelo_robot. Next: robot.sh build, then robot.sh restart"
  exit 0
fi

# the rest runs on the robot computer
"${SSH[@]}" bash -s -- "$@" <<'REMOTE'
set -e
C=kelo_robot
WS=/root/kelo_ws
DIR=~/kelo_robot/docker
ROS="source /opt/ros/humble/setup.bash; source $WS/install/setup.bash"

container_up() { [ -n "$(docker ps -q -f name="^${C}\$")" ]; }
inc() { docker exec "$C" bash -c "$1"; }
# [x] in the patterns: pgrep/pkill do not match their own command line (bash -c "pgrep -f ...")
running() { container_up && inc "pgrep -f '$1' > /dev/null"; }

build() {
  inc "cd $WS && source /opt/ros/humble/setup.bash && colcon build 2>&1 | tail -4"
}

ensure_up() {
  if ! container_up; then
    [ -f "$DIR/docker-compose.yml" ] || { echo "[robot] $DIR not found: run robot.sh deploy first"; exit 1; }
    (cd "$DIR" && docker compose up -d 2>&1 | grep -v DISPLAY || true)
    sleep 1
    container_up || { echo "[robot] container $C does not start"; exit 1; }
  fi
  if ! inc "test -f $WS/install/setup.bash"; then
    echo "[robot] driver not built yet: colcon build (a few minutes)"
    build
  fi
}

other_driver() {
  local c
  for c in $(docker ps --format '{{.Names}}'); do
    [ "$c" = "$C" ] && continue
    if docker exec "$c" pgrep -f '[p]latform_driver_node' > /dev/null 2>&1; then echo "$c"; return; fi
  done
}

stop_procs() {  # <patterns...>: Ctrl+C, then killed after 5 s
  local p any
  for p in "$@"; do inc "pkill -INT -f '$p'" || true; done
  for _ in 1 2 3 4 5; do
    any=0
    for p in "$@"; do if running "$p"; then any=1; fi; done
    [ "$any" = 0 ] && return
    sleep 1
  done
  for p in "$@"; do inc "pkill -KILL -f '$p'" || true; done
}

driver_start() {
  ensure_up
  if running '[p]latform_driver_node'; then echo "[robot] driver already on (robot.sh restart to change its arguments)"; return; fi
  local o; o=$(other_driver)
  if [ -n "$o" ]; then echo "[robot] container $o already runs a driver. Stop it first: only one driver at a time"; exit 1; fi
  docker exec -d "$C" bash -c "$ROS; exec ros2 launch kelo_tulip_ros2 platform_driver.launch.py $* > /tmp/kelo_driver.out 2>&1"
  echo "[robot] driver starting (EtherCAT)..."
  for _ in $(seq 1 15); do
    sleep 1
    if inc "grep -q 'initialized' /tmp/kelo_driver.out"; then
      inc "grep -E 'initialized|fix_odom_rotation=' /tmp/kelo_driver.out | sed 's/.*platform_driver\]: //'"
      return
    fi
    if inc "grep -qE 'Found only|EtherCAT init failed|Failed to initialize' /tmp/kelo_driver.out"; then
      echo "[robot] ERROR: EtherCAT does not start. Emergency stop released? Wheels powered?"
      inc "grep -E 'Found only|EtherCAT|ERROR' /tmp/kelo_driver.out | tail -4"
      driver_stop > /dev/null; exit 1
    fi
    running '[p]latform_driver_node' || break
  done
  echo "[robot] ERROR: the driver is not running. Last lines:"; inc "tail -8 /tmp/kelo_driver.out"; exit 1
}

driver_stop() {
  if ! running '[p]latform_driver_node'; then echo "[robot] driver already off"; return; fi
  stop_procs '[p]latform_driver.launch.py' '[p]latform_driver_node'
  echo "[robot] driver off"
}

# publishers or subscribers of a topic, by count: Humble does not see the node names of the Jazzy stack
topic_count() { inc "$ROS; timeout 10 ros2 topic info $1 --no-daemon 2> /dev/null" | sed -n "s/^$2 count: //p"; }

joy_start() {
  local mode=${1:-direct} args=""
  ensure_up
  ls /dev/input/js* > /dev/null 2>&1 || { echo "[robot] no joypad on the robot computer (/dev/input/js*)"; exit 1; }
  running '[p]latform_driver_node' || echo "[robot] note: driver off, the joystick moves nothing (robot.sh start)"
  if running '[t]eleop_joystick'; then
    echo "[robot] joystick already on: robot.sh joy stop first"; exit 1
  fi
  if [ "$mode" = safe ]; then
    args="--ros-args -r /cmd_vel:=/cmd_vel_teleop"
    [ "$(topic_count /cmd_vel_teleop Subscription)" -gt 0 ] 2> /dev/null \
      || echo "[robot] note: stack off, the joystick moves nothing (nobody reads /cmd_vel_teleop)"
  else
    # both write /cmd_vel, also when idle: they would cancel each other out
    if [ "$(topic_count /cmd_vel Publisher)" -gt 0 ] 2> /dev/null; then
      echo "[robot] REFUSED: something else publishes /cmd_vel. Stop the stack, or use robot.sh joy safe"; exit 1
    fi
  fi
  running '[j]oy_node' || docker exec -d "$C" bash -c "$ROS; exec ros2 run joy joy_node > /tmp/kelo_joy.out 2>&1"
  docker exec -d "$C" bash -c "$ROS; exec ros2 run kelo_teleop teleop_joystick $args > /tmp/kelo_teleop.out 2>&1"
  sleep 3
  if running '[j]oy_node' && running '[t]eleop_joystick'; then
    if [ "$mode" = safe ]; then
      echo "[robot] joystick on ($mode): hold LB to move, A speed level (starts at 25%), RB stop. Limits and collision monitor of the stack apply"
    else
      echo "[robot] joystick on: hold LB to move, A speed level (starts at 25%), RB stop. Stop it before starting the stack"
    fi
  else
    echo "[robot] ERROR: joystick did not start"; inc "tail -5 /tmp/kelo_joy.out /tmp/kelo_teleop.out" || true; exit 1
  fi
}

joy_stop() {
  if ! running '[j]oy_node' && ! running '[t]eleop_joystick'; then echo "[robot] joystick already off"; return; fi
  stop_procs '[t]eleop_joystick' '[j]oy_node'    # teleop first: it exits with a zero velocity
  echo "[robot] joystick off"
}

status() {
  echo "[robot] reachable: $(hostname), $(date '+%F %T')"
  local o; o=$(other_driver)
  [ -z "$o" ] || echo "  WARNING      : container $o runs a driver"
  if ! container_up; then echo "  container    : off (robot.sh up)"; return; fi
  echo "  container    : on"
  if running '[p]latform_driver_node'; then echo "  driver       : on"; else echo "  driver       : off"; fi
  if running '[j]oy_node' || running '[t]eleop_joystick'; then echo "  joystick     : on"; else echo "  joystick     : off"; fi
  if ls /dev/input/js* > /dev/null 2>&1; then echo "  joypad       : connected"; else echo "  joypad       : not connected"; fi
}

cmd=$1; shift
case "$cmd" in
  status)  status ;;
  up)      ensure_up; echo "[robot] container $C on" ;;
  down)    if container_up; then joy_stop; driver_stop; docker stop "$C" > /dev/null; fi; echo "[robot] container $C off" ;;
  start)   driver_start "$@" ;;
  stop)    if container_up; then joy_stop; driver_stop; else echo "[robot] container off"; fi ;;
  restart) if container_up; then joy_stop; driver_stop; fi; driver_start "$@" ;;
  joy)     case "${1:-}" in
             stop) joy_stop ;;
             safe) joy_start safe ;;
             "")   joy_start ;;
             *)    echo "[robot] joy: use stop, safe or nothing"; exit 2 ;;
           esac ;;
  log)     case "${1:-driver}" in
             joy) inc "tail -15 /tmp/kelo_joy.out /tmp/kelo_teleop.out 2> /dev/null" || echo "[robot] no joystick log" ;;
             *)   inc "tail -20 /tmp/kelo_driver.out 2> /dev/null" || echo "[robot] no driver log" ;;
           esac ;;
  build)   ensure_up; build ;;
  reset)   inc "$ROS; ros2 topic pub --once /reset std_msgs/msg/Empty '{}' > /dev/null"; echo "[robot] error flags reset" ;;
esac
REMOTE
