#!/usr/bin/env bash
# The unit and node tests from the built workspace, after steps/07_build_ours.sh. The nav node tests (-m ros) and the
# planner and cot node tests need rclpy and the colcon-built corridor_msgs: REQUIRE_ROS makes them fail instead of
# skipping where ROS is missing, and a SKIPPED line in the ros run fails the verdict too.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"   # before env.sh, whose default is the demo domain 42
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
source ./steps/lib.sh
export REQUIRE_ROS=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
LOG=~/deletdis/log/17_tests.log; exec > >(tee -a "$LOG") 2>&1
echo "=== $(date -Is) tests on ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
if [ "$ROS_DOMAIN_ID" = "42" ]; then
  echo "ROS_DOMAIN_ID is the demo domain 42 and the node tests publish on live topics: run ROS_DOMAIN_ID=77 ./steps/17_tests.sh"
  echo "TESTS FAIL"; exit 1
fi
fail=0
lines=()
run() {   # run <label> <pytest args...>: one pytest run; its output stays in $out for the caller
  local label=$1 rc line; shift
  out=/tmp/tests17_${label// /_}.txt
  echo "--- $label: python3 -m pytest -q -rs -p no:cacheprovider $*"
  python3 -m pytest -q -rs -p no:cacheprovider "$@" 2>&1 | tee "$out"
  rc=${PIPESTATUS[0]}
  line=$(grep -E '(passed|failed|skipped|error|deselected|no tests ran)' "$out" | tail -1)
  [ "$rc" -eq 0 ] || fail=1
  lines+=("$label: ${line:-no pytest summary} (exit $rc)")
}
run "nav ros" ws/src/corridor_nav/test -m ros
if grep -q '^SKIPPED' "$out"; then echo "ros-marked tests were skipped: the nav node tests did not run"; fail=1; fi
run "nav pure" ws/src/corridor_nav/test -m "not ros"
run planner ws/src/corridor_planner/test
run cot ws/src/corridor_cot/test
[ -d tests ] && run tests tests
echo "--- summary"
printf '%s\n' "${lines[@]}"
[ "$fail" -eq 0 ] && echo "TESTS PASS" || echo "TESTS FAIL"
echo "=== $(date -Is) done"
exit "$fail"
