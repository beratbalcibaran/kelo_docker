#!/usr/bin/env bash
set -u
IMG="${IMG:-kelo-slam:latest}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PHASES="${*:-A B C D}"
WORK="${E2E_DIR:-$(mktemp -d "${TMPDIR:-/tmp}/kelo_e2e.XXXXXX")}"
mkdir -p "$WORK/data"
NAME="${E2E_NAME:-kelo-e2e}"
DOMAIN="${E2E_DOMAIN:-77}"
FAIL=0
RESULTS=()
R='source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash;'
NATIVE="${E2E_NATIVE:-0}"
DATA=/data
if [ "$NATIVE" = 1 ]; then
  WS="${KELO_WS:-$(cd "$ROOT/.." && pwd)/ws}"
  [ -f "$WS/install/setup.bash" ] || { echo "[e2e] native: no workspace at $WS (set KELO_WS)"; exit 2; }
  R="source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash;"
  DATA="$WORK/data"
  DOMAIN="${E2E_DOMAIN:-79}"
  export ROS_DOMAIN_ID=$DOMAIN KELO_DATA_DIR="$DATA" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
         CYCLONEDDS_URI="file://$ROOT/docker/cyclonedds.xml"
  unset DISPLAY
  STACK_PID=""
fi

if [ "$NATIVE" = 1 ]; then
run() { bash -c "$R $*"; }
runi() { bash -c "$R $*"; }
else
run() { docker exec "$NAME" bash -c "$R $*"; }
runi() { docker exec -i "$NAME" bash -c "$R $*"; }
fi
record() { echo "$1"; RESULTS+=("$1"); [[ "$1" == FAIL* ]] && FAIL=1; return 0; }
check() {
  if eval "$2" >"$WORK/last.txt" 2>&1; then record "OK   $1"
  else record "FAIL $1"; sed 's/^/     /' "$WORK/last.txt" | tail -8; fi
}
e2e() {
  local name="$1"; shift
  if run "timeout 900 ros2 run kelo_bringup kelo_e2e $*" >"$WORK/last.txt" 2>&1
  then record "OK   $name"; else record "FAIL $name"; fi
  grep '\[e2e\]' "$WORK/last.txt" | sed 's/^/     /'
}
if [ "$NATIVE" = 1 ]; then
kill_stack() {
  [ -n "$STACK_PID" ] || return 0
  kill -TERM "$STACK_PID" 2>/dev/null
  for _ in $(seq 1 300); do pgrep -s "$STACK_PID" >/dev/null || break; sleep 0.1; done
  if pgrep -s "$STACK_PID" >/dev/null; then
    echo "[e2e] stack did not stop within 30 s, killing: $(pgrep -s "$STACK_PID" -l | awk '{print $2}' | sort -u | tr '\n' ' ')"
    pkill -KILL -s "$STACK_PID"
  fi
  STACK_PID=""
}
start() {
  local mode="$1"; shift
  kill_stack
  local envs=(KELO_SOURCE=sim KELO_MODE="$mode" KELO_MAP="$DATA/maps/current.yaml")
  while [ $# -gt 0 ]; do if [ "$1" = -e ]; then envs+=("$2"); shift 2; else shift; fi; done
  : >"$WORK/stack.log"
  env --default-signal=INT,QUIT "${envs[@]}" setsid bash -c "$R exec bash $ROOT/docker/run-stack.sh" >"$WORK/stack.log" 2>&1 &
  STACK_PID=$!
}
start_cmd() { kill_stack; : >"$WORK/stack.log"; env --default-signal=INT,QUIT setsid bash -c "$R exec $*" >"$WORK/stack.log" 2>&1 & STACK_PID=$!; }
stop() { kill_stack; cp "$WORK/stack.log" "$WORK/$1.log" 2>/dev/null; }
wait_log() { local end=$((SECONDS + $2)); until grep -q "$1" "$WORK/stack.log" 2>/dev/null; do
  [ $SECONDS -gt $end ] && return 1; sleep 2; done; }
else
start() {
  local mode="$1"; shift
  docker rm -f "$NAME" >/dev/null 2>&1
  docker run -d --name "$NAME" --network bridge -e ROS_DOMAIN_ID=$DOMAIN -e KELO_SOURCE=sim \
    -e KELO_MODE="$mode" "$@" -v "$WORK/data":/data "$IMG" /run-stack.sh >/dev/null || exit 1
}
start_cmd() { docker rm -f "$NAME" >/dev/null 2>&1
  docker run -d --name "$NAME" --network bridge -e ROS_DOMAIN_ID=$DOMAIN "$IMG" bash -c "$R $*" >/dev/null; }
stop() { docker logs "$NAME" >"$WORK/$1.log" 2>&1; docker stop -t 20 "$NAME" >/dev/null 2>&1
         docker rm -f "$NAME" >/dev/null 2>&1; }
wait_log() { local end=$((SECONDS + $2)); until docker logs "$NAME" 2>&1 | grep -q "$1"; do
  [ $SECONDS -gt $end ] && return 1; sleep 2; done; }
fi
tf() { run "timeout 12 ros2 run tf2_ros tf2_echo $1 $2 2>&1 | grep -q Translation"; }
hz() { run "timeout 8 ros2 topic hz $1 2>/dev/null | grep -q average"; }
skip() { record "SKIP $1 (marvelmind: false)"; }

MMPY='from kelo_bringup.common import load_hardware; print(int(bool(load_hardware()["sensors"]["marvelmind"])))'
if [ "$NATIVE" = 1 ]; then MM=$(bash -c "$R python3 -c '$MMPY'")
else MM=$(docker run --rm -v "$WORK/data":/data "$IMG" bash -c "$R python3 -c '$MMPY'"); fi
[ "$MM" = 1 ] || MM=0
echo "[e2e] marvelmind: $([ $MM = 1 ] && echo true || echo false)"

echo "[e2e] working directory: $WORK ($([ "$NATIVE" = 1 ] && echo "native, $WS" || echo "docker, $IMG"), ROS domain $DOMAIN)"
if [ "$NATIVE" = 1 ]; then trap 'kill_stack; bash -c "$R ros2 daemon stop" >/dev/null 2>&1' EXIT
else trap 'docker rm -f "$NAME" >/dev/null 2>&1' EXIT; fi

if [[ " $PHASES " == *" A "* ]]; then
  echo "=== A: mapping (source:=sim, faulty odometry, all sensors)"
  start mapping
  wait_log "All requested topics are subscribed" 90 || true
  sleep 5
  check "gate node running"                "run ros2 node list | grep -q cmd_vel_gate"
  check "robot_discover refuses next to a stack" \
        "run ros2 run kelo_bringup robot_discover --domains $DOMAIN --time 1 | grep -q \"stack is running\""
  check "TF odom->base_link (odom_filter)" "tf odom base_link"
  check "TF base_link->laser"              "tf base_link laser"
  check "TF base_link->camera optical"    "tf base_link camera_depth_optical_frame"
  if [ $MM = 1 ]; then check "TF base_link->hedgehog" "tf base_link hedgehog_link"
  else skip "TF base_link->hedgehog"; fi
  for t in /scan /odom /odom_corrected /camera/camera/depth/image_rect_raw /camera/points_filtered \
           /camera/points_clearing; do check "$t arriving" "hz $t"; done
  for t in /hedgehog_pos_ang /marvelmind/pose; do
    if [ $MM = 1 ]; then check "$t arriving" "hz $t"; else skip "$t arriving"; fi
  done
  e2e "mapping tour (teleop chain, no collisions)" tour
  check "1.0 m/s into the gate -> /cmd_vel = 0.30 (gate limit)" \
    "run \"(ros2 topic pub -r 20 /cmd_vel_teleop_safe geometry_msgs/msg/Twist '{linear: {x: 1.0}}' >/dev/null &) ; sleep 2; timeout 5 ros2 topic echo --once /cmd_vel | grep -E 'x: 0\\.(29|3)'\""
  sleep 1
  check "watchdog: zero velocity when commands stop" \
    "run \"pkill -f '[t]opic pub' ; sleep 1.5; timeout 5 ros2 topic echo --once /cmd_vel | grep -E 'x: 0\\.0$'\""
  check "safety_stop -> zero velocity" \
    "run \"(ros2 topic pub -r 20 /cmd_vel_teleop geometry_msgs/msg/Twist '{linear: {x: 0.2}}' >/dev/null &) ; (ros2 topic pub -r 10 /kelo/safety_stop std_msgs/msg/Bool '{data: true}' >/dev/null &) ; sleep 2; timeout 5 ros2 topic echo --once /kelo/gate_state | grep SAFETY_STOP\""
  run "pkill -f '[t]opic pub'" >/dev/null 2>&1; sleep 3
  check "map->odom TF (slam_toolbox)" "tf map odom"
  check "save_map" "run ros2 run kelo_bringup save_map e2e"
  if [ $MM = 1 ]; then
    check "Marvelmind alignment (mm_calibrate)" \
      "run 'ros2 run kelo_bringup mm_calibrate \$(ls -t $DATA/mm_samples_*.csv | head -1)'"
  else skip "Marvelmind alignment (mm_calibrate)"; fi
  stop A
  check "bag recording closed cleanly (metadata.yaml)" "ls $WORK/data/bags/map_sim_*/metadata.yaml"
fi

if [[ " $PHASES " == *" B "* ]]; then
  echo "=== B: navigation (unknown start heading) + safety scenarios"
  start navigation -e KELO_EXTRA_ARGS="sim_yaw0:=1.0472"
  sleep 15
  if [ $MM = 1 ]; then
    check "localize: Marvelmind position + turn in place -> LOCALIZED (heading unknown)" \
      "run timeout 200 ros2 run kelo_bringup localize"
  else
    check "localize: operator position (--x/--y) + turn in place -> LOCALIZED (heading unknown)" \
      "run timeout 200 ros2 run kelo_bringup localize --x 0 --y 0"
  fi
  grep '\[localize\]' "$WORK/last.txt" | sed 's/^/     /'
  e2e "AMCL pose ~ true pose" localization
  for sc in "obstacle table" "obstacle low" "obstacle person" "teleop_obstacle table" "teleop_obstacle low" \
            "dropout camera" "dropout lidar" "teleop_dropout camera" "teleop_dropout lidar" "near_wall" \
            "detour box far" "detour table far" "detour table sudden" "cornered" "route basic" "route skip" "route pause" "route person" "route loop" "route foreign" \
            "people stand" "people leave" "people cross"; do
    e2e "safety: $sc" $sc
  done
  e2e "Send Goal: direct motion along the body axis" goal_motion
  stop B
  echo "=== B2: no alignment -> localize automatic global mode"
  mv "$WORK/data/marvelmind_alignment.yaml" "$WORK/alignment.bak" 2>/dev/null
  rm -f "$WORK/data/last_pose.yaml"
  start navigation -e KELO_EXTRA_ARGS="sim_yaw0:=1.0472"
  sleep 15
  check "localize: no alignment -> switches to GLOBAL mode, LOCALIZED" \
    "run timeout 240 ros2 run kelo_bringup localize && grep -q GLOBAL $WORK/last.txt"
  grep '\[localize\]' "$WORK/last.txt" | sed 's/^/     /'
  e2e "AMCL pose ~ true pose (global localization)" localization
  e2e "second 'Localize' keeps a good estimate; --force re-localizes" relocalize
  e2e "robot carried away -> LOST, goal refused, Localize finds it again" kidnap
  stop B2
  echo "=== B3: navigation restarted where the robot stands -> last pose restored"
  LP=$(LC_ALL=C awk '/^x:/{x=$2} /^y:/{y=$2} /^yaw_deg:/{a=$2} END{if (x != "") printf "sim_x0:=%.3f sim_y0:=%.3f sim_yaw0:=%.4f", x+1, y+1, a*3.14159265/180}' "$WORK/data/last_pose.yaml" 2>/dev/null)
  echo "[e2e] restart at the last saved pose: ${LP:-none saved}"
  start navigation -e KELO_EXTRA_ARGS="$LP"
  sleep 10
  e2e "last pose restored at start (no Localize)" restored
  stop B3
  mv "$WORK/alignment.bak" "$WORK/data/marvelmind_alignment.yaml" 2>/dev/null
fi

if [[ " $PHASES " == *" C "* ]]; then
  echo "=== C: replay (A's bag) -> same stack -> map comparison"
  BAG=$(ls -d "$WORK"/data/bags/map_sim_* 2>/dev/null | head -1)
  LIVE=$(ls "$WORK"/data/maps/e2e_*.yaml 2>/dev/null | grep -v '_camera\.yaml$' | head -1)
  if [ -z "$BAG" ] || [ -z "$LIVE" ]; then
    record "FAIL replay: phase A bag/map missing"
  else
    check "replay map ~ live map (F1 >= 0.85)" \
      "OUT=$WORK/replay REPLAY_NATIVE=$NATIVE $ROOT/scripts/replay_regression.sh $BAG $LIVE --min-f1 0.85"
    sed 's/^/     /' "$WORK/replay/compare.txt" 2>/dev/null
    check "live map ~ reference (test/data, F1 >= 0.80)" \
      "$([ "$NATIVE" = 1 ] && echo "bash -c \"$R ros2 run kelo_bringup map_compare $LIVE $ROOT/src/kelo_bringup/test/data/sim_reference.yaml --min-f1 0.80\"" || echo "docker run --rm -v $WORK/data:/data -v $ROOT/src/kelo_bringup/test/data:/ref:ro $IMG bash -c \"$R ros2 run kelo_bringup map_compare /data/maps/$(basename "$LIVE") /ref/sim_reference.yaml --min-f1 0.80\"")"
    sed 's/^/     /' "$WORK/last.txt"
  fi
fi

if [[ " $PHASES " == *" D "* ]]; then
  echo "=== D: odometry + odom_test calibrate-rotate (CALIBRATION ONLY, no gate)"
  start sensors -e KELO_EXTRA_ARGS="sim_x0:=4.4 sim_y0:=1.9"
  sleep 15
  e2e "faulty odometry + odom_filter correction" odom
  check "calibration is REFUSED while the stack (gate) runs" \
    "echo CALIBRATE | runi ros2 run kelo_bringup odom_test calibrate-rotate --rate 1.5 --measured-turns 3 2>&1 | grep -q REFUSED"
  stop D1
  start_cmd ros2 run kelo_bringup sim_robot
  sleep 6
  check "1.5 rad/s, 3 turns: k ~ 8.6 (5% ramp allowance)" \
    "echo CALIBRATE | runi ros2 run kelo_bringup odom_test calibrate-rotate --rate 1.5 --turns 3 --measured-turns 3 2>&1 | grep -oE 'k = real/odom = [0-9.]+' | awk '{v=\$NF; print v; exit !(v > 8.17 && v < 9.03)}'"
  sed 's/^/     k = /' "$WORK/last.txt"
  check "gate limit UNCHANGED (safety.yaml max_wz: 0.80)" \
    "grep -qE '^ +max_wz: 0.80' $ROOT/src/kelo_bringup/config/safety.yaml"
  stop D2
fi

echo
echo "================ SUMMARY ($WORK)"
printf '%s\n' "${RESULTS[@]}"
[ "$FAIL" -eq 0 ] && echo "ALL TESTS PASSED" || { echo "SOME TESTS FAILED"; exit 1; }
