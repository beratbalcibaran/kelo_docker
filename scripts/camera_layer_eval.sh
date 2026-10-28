#!/usr/bin/env bash
# Task 1 evaluation in the simulator: two-layer map (lidar + camera layer) vs lidar only.
#   1. mapping drive in config/sim_world_tables.yaml (a person walks by), bag recorded
#   2. the same bag mapped twice: camera_layer false / true, then the camera layer rebuilt
#      from the bag on the finished map (camera_layer_rebuild)
#   3. the maps vs a reference built from the world (map_compare + per-object coverage)
#   4. navigation goals of the world eval section on each map (success, reroutes, stops)
# usage: scripts/camera_layer_eval.sh [--bag <mapping bag dir>] [--skip-nav]
set -u
IMG="${IMG:-kelo-slam:latest}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${OUT:-$ROOT/data/eval/task1_$(date +%Y%m%d_%H%M%S)}"
WORLD_NAME=sim_world_tables.yaml
WORLD=/ros2_ws/install/kelo_bringup/share/kelo_bringup/config/$WORLD_NAME
BAG=""
NAV=1
while [ $# -gt 0 ]; do
  case "$1" in
    --bag) BAG="$(cd "$2" && pwd)"; shift 2 ;;
    --skip-nav) NAV=0; shift ;;
    *) echo "unknown argument: $1"; exit 2 ;;
  esac
done
NAME=kelo-task1
DOMAIN=79
R='source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash;'
mkdir -p "$OUT"
FAIL=0
trap 'docker rm -f "$NAME" >/dev/null 2>&1' EXIT

log() { echo "[task1] $*"; }
run() { docker exec "$NAME" bash -c "$R $*"; }
hw() { mkdir -p "$1"; printf 'mapping:\n  camera_layer: %s\n' "$2" > "$1/hardware.yaml"; }
start() {
  local data="$1"; shift
  docker rm -f "$NAME" >/dev/null 2>&1
  docker run -d --name "$NAME" --network bridge -e ROS_DOMAIN_ID=$DOMAIN -v "$data":/data "$@" \
    "$IMG" /run-stack.sh >/dev/null || exit 1
}
stop() { docker logs "$NAME" >"$1" 2>&1; docker stop -t 20 "$NAME" >/dev/null 2>&1; docker rm -f "$NAME" >/dev/null 2>&1; }
wait_log() { local end=$((SECONDS + $2)); until docker logs "$NAME" 2>&1 | grep -q "$1"; do
  [ $SECONDS -gt $end ] && return 1; sleep 3; done; }

if [ -z "$BAG" ]; then
  log "1/4 mapping drive in $WORLD_NAME (bag recorded)"
  hw "$OUT/live" true
  start "$OUT/live" -e KELO_SOURCE=sim -e KELO_MODE=mapping -e KELO_EXTRA_ARGS="sim_world:=$WORLD"
  wait_log "All requested topics are subscribed" 90 || true
  sleep 8
  run "timeout 900 ros2 run kelo_bringup kelo_e2e world_tour $WORLD" | grep '\[e2e\]' | tee "$OUT/mapping_tour.txt"
  grep -q PASS "$OUT/mapping_tour.txt" || { log "FAIL: mapping drive"; FAIL=1; }
  sleep 3
  run ros2 run kelo_bringup save_map live | tee "$OUT/live_save.txt"
  stop "$OUT/live/stack.log"
  BAG=$(ls -d "$OUT"/live/bags/map_sim_* | head -1)
fi
log "bag: $BAG"
echo "$BAG" > "$OUT/bag.txt"

DUR=$(awk '/^  duration:/{f=1} f&&/nanoseconds:/{print int($2/1e9); exit}' "$BAG/metadata.yaml" 2>/dev/null)
DUR="${DUR:-600}"
for cfg in lidar camera; do
  layer=false; [ $cfg = camera ] && layer=true
  log "2/4 replay the same bag, camera_layer: $layer"
  hw "$OUT/$cfg" $layer
  docker rm -f "$NAME" >/dev/null 2>&1
  docker run -d --name "$NAME" --network bridge -e ROS_DOMAIN_ID=$DOMAIN -v "$BAG":/bag:ro \
    -v "$OUT/$cfg":/data "$IMG" bash -c "$R ros2 launch kelo_bringup bringup.launch.py source:=replay \
    mode:=mapping bag:=/bag record:=false" >/dev/null || exit 1
  wait_log REPLAY_FINISHED $((DUR + 180)) || { log "FAIL: replay $cfg did not finish"; FAIL=1; }
  sleep 5
  run ros2 run kelo_bringup save_map $cfg | tee "$OUT/${cfg}_save.txt"
  stop "$OUT/$cfg/replay.log"
done

log "2/4 rebuild the camera layer from the bag on the finished map"
rm -rf "$OUT/rebuilt"; cp -a "$OUT/camera" "$OUT/rebuilt"
docker rm -f "$NAME" >/dev/null 2>&1
docker run --rm --name "$NAME" --network bridge -v "$BAG":/bag:ro -v "$OUT/rebuilt":/data "$IMG" \
  bash -c "$R ros2 run kelo_bringup camera_layer_rebuild /bag" | tee "$OUT/rebuilt/rebuild.log"
grep -q '^Saved:' "$OUT/rebuilt/rebuild.log" || { log "FAIL: camera layer rebuild"; FAIL=1; }

log "3/4 maps vs reference"
docker run --rm -v "$OUT":/out "$IMG" bash -c "$R
  python3 -m kelo_bringup.layer_eval reference $WORLD /out/reference
  for c in lidar camera rebuilt; do
    flag=''; [ \$c != lidar ] && flag=--camera
    ros2 run kelo_bringup map_compare /out/\$c/maps/current.yaml /out/reference.yaml --min-f1 0 \$flag > /out/compare_\$c.txt
    python3 -m kelo_bringup.layer_eval coverage $WORLD /out/\$c/maps/current.yaml \$flag > /out/coverage_\$c.yaml
    python3 -m kelo_bringup.layer_eval false $WORLD /out/\$c/maps/current.yaml \$flag > /out/false_\$c.yaml
  done
  python3 -m kelo_bringup.layer_eval figure /out/maps.png /out/reference.yaml \
    'lidar only=/out/lidar/maps/current.yaml' \
    'lidar + camera layer (mapping run)=/out/camera/maps/current.yaml:camera' \
    'lidar + camera layer (rebuilt)=/out/rebuilt/maps/current.yaml:camera'" || FAIL=1
cat "$OUT"/compare_*.txt

if [ $NAV -eq 1 ]; then
  for cfg in lidar camera rebuilt; do
    log "4/4 navigation goals on the $cfg map"
    start "$OUT/$cfg" -e KELO_SOURCE=sim -e KELO_MODE=navigation -e KELO_INITIAL_POSE="0 0 0" \
      -e KELO_EXTRA_ARGS="sim_world:=$WORLD"
    sleep 25
    run "timeout 2400 ros2 run kelo_bringup kelo_e2e world_goals $WORLD /data/goals.json" \
      | grep '\[e2e\]' | tee "$OUT/goals_$cfg.txt"
    stop "$OUT/$cfg/nav.log"
    grep -q "camera layer:" "$OUT/$cfg/nav.log" && log "$cfg: camera layer loaded" || log "$cfg: no camera layer"
  done
fi

docker run --rm -v "$OUT":/out "$IMG" bash -c "$R python3 - <<'EOF'
import json, os, re, yaml
out = '/out'
lines = ['# Task 1 results (simulator, sim_world_tables.yaml)', '', 'Bag: ' + open(f'{out}/bag.txt').read().strip(), '',
         'Maps side by side: maps.png (black LiDAR map, orange cells added by the camera layer)', '',
         '## Maps vs reference (map_compare, tol 0.10 m)', '',
         '| map | precision | recall | F1 | occupied cells | phantom cells (> 0.10 m from any object) |',
         '|---|---|---|---|---|---|']
LABELS = (('lidar', 'lidar only'), ('camera', 'lidar + camera layer (mapping run)'),
          ('rebuilt', 'lidar + camera layer (rebuilt)'))
for c, label in LABELS:
    t = open(f'{out}/compare_{c}.txt').read()
    v = dict(re.findall(r'(precision|recall|F1)=([0-9.]+)', t))
    cells = re.search(r'cells (\d+)', t).group(1)
    fa = yaml.safe_load(open(f'{out}/false_{c}.yaml'))
    lines.append(f\"| {label} | {v['precision']} | {v['recall']} | {v['F1']} | {cells} | {fa['false']} |\")
cov = {c: yaml.safe_load(open(f'{out}/coverage_{c}.yaml')) for c, _ in LABELS}
lines += ['', '## Share of each object outline found in the map', '',
          '| object | ' + ' | '.join(l for _, l in LABELS) + ' |', '|---|---|---|---|']
lines += [f\"| {k} | \" + ' | '.join(f\"{cov[c][k]:.2f}\" for c, _ in LABELS) + ' |' for k in cov['lidar']]
for c, label in LABELS:
    path = f'{out}/{c}/goals.json'
    if not os.path.isfile(path):
        continue
    rows = json.load(open(path))
    lines += ['', f'## Navigation goals: {label}', '',
              'Goals in sequence from the previous goal. After a failed goal the robot is put on that goal '
              '(sim teleport, AMCL re-seeded), so each goal is measured on its own.', '',
              '| goal | target | result | time s | path m | reroutes | stops | collision monitor stops | collisions |',
              '|---|---|---|---|---|---|---|---|---|']
    for r in rows:
        lines.append(f\"| {r['goal']} | {r.get('target')} | {'OK' if r.get('success') else 'FAILED'} | {r.get('time_s')} | \"
                     f\"{r.get('path_m')} | {r.get('reroutes')} | {r.get('stops')} | {r.get('cm_stops')} | {r.get('collisions')} |\")
    ok = [r for r in rows if r.get('success')]
    tot = lambda k: sum(r.get(k, 0) or 0 for r in rows)
    lines.append(f\"| **total** | | **{len(ok)}/{len(rows)}** | {tot('time_s'):.0f} | {tot('path_m'):.1f} | {tot('reroutes')} | \"
                 f\"{tot('stops')} | {tot('cm_stops')} | {tot('collisions')} |\")
open(f'{out}/results.md', 'w').write('\n'.join(lines) + '\n')
print('\n'.join(lines))
EOF" || FAIL=1
log "results: $OUT/results.md"
exit $FAIL
