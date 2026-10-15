#!/usr/bin/env bash
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMG="${IMG:-kelo-slam:latest}"
BUILD=1
E2E=1
for a in "$@"; do
  case "$a" in
    --no-build) BUILD=0 ;;
    --unit-only) E2E=0 ;;
    *) echo "unknown argument: $a"; exit 2 ;;
  esac
done
cd "$ROOT" || exit 1
STATUS=()
FAIL=0

if [ $BUILD -eq 1 ]; then
  echo "=== building image ($IMG)"
  if docker compose --profile production build; then STATUS+=("OK   image build")
  else STATUS+=("FAIL image build"); echo "image build failed"; printf '%s\n' "${STATUS[@]}"; exit 1; fi
fi

echo "=== unit tests (pytest)"
if docker run --rm "$IMG" bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash \
     && cd /ros2_ws/src/kelo_bringup && python3 -m pytest test -q"; then STATUS+=("OK   unit tests")
else STATUS+=("FAIL unit tests"); FAIL=1; fi

if [ $E2E -eq 1 ]; then
  echo "=== end-to-end (sim)"
  if "$ROOT/scripts/sim_e2e_test.sh"; then STATUS+=("OK   end-to-end (sim_e2e_test.sh)")
  else STATUS+=("FAIL end-to-end (sim_e2e_test.sh)"); FAIL=1; fi
fi

echo
echo "================ test_all SUMMARY"
printf '%s\n' "${STATUS[@]}"
exit $FAIL
