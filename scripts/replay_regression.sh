#!/usr/bin/env bash
set -u
IMG="${IMG:-kelo-slam:latest}"
BAG="${1:?bag directory required}"
REF="${2:?reference map yaml required}"
shift 2
MIN_F1=0.85
ODOM=robot
while [ $# -gt 0 ]; do
  case "$1" in
    --min-f1) MIN_F1="$2"; shift 2 ;;
    --odom) ODOM="$2"; shift 2 ;;
    *) echo "unknown argument: $1"; exit 2 ;;
  esac
done
BAG="$(cd "$BAG" && pwd)"
REF_DIR="$(cd "$(dirname "$REF")" && pwd)"
REF_NAME="$(basename "$REF")"
OUT="${OUT:-$PWD/data/replay_out/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
NAME="kelo-replay-$$"
DUR=$(awk '/^  duration:/{f=1} f&&/nanoseconds:/{print int($2/1e9); exit}' "$BAG/metadata.yaml" 2>/dev/null)
DUR="${DUR:-600}"
echo "[replay] bag=$BAG (~${DUR} s) ref=$REF odom_source=$ODOM -> $OUT"

docker rm -f "$NAME" >/dev/null 2>&1
docker run -d --name "$NAME" --network bridge -e ROS_DOMAIN_ID=78 -e KELO_DATA_DIR=/data \
  -v "$BAG":/bag:ro -v "$REF_DIR":/ref:ro -v "$OUT":/data "$IMG" bash -c "
  source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash
  ros2 launch kelo_bringup bringup.launch.py source:=replay mode:=mapping bag:=/bag \
    odom_source:=$ODOM record:=false" >/dev/null || exit 1

cleanup() { docker logs "$NAME" > "$OUT/replay.log" 2>&1; docker rm -f "$NAME" >/dev/null 2>&1; }
trap cleanup EXIT

END=$((SECONDS + DUR + 120))
until docker logs "$NAME" 2>&1 | grep -q REPLAY_FINISHED; do
  [ $SECONDS -gt $END ] && { echo "[replay] FAIL: timeout (bag did not finish)"; exit 1; }
  docker ps -q -f name="$NAME" | grep -q . || { echo "[replay] FAIL: container stopped"; exit 1; }
  sleep 5
done
sleep 5
docker exec "$NAME" bash -c "source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash;
  ros2 run kelo_bringup save_map replay" || { echo "[replay] FAIL: save_map"; exit 1; }
docker exec "$NAME" bash -c "source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash;
  ros2 run kelo_bringup map_compare /data/maps/current.yaml /ref/$REF_NAME --min-f1 $MIN_F1" | tee "$OUT/compare.txt"
exit "${PIPESTATUS[0]}"
