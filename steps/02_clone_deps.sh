#!/usr/bin/env bash
# Day 1 step 2: foxglove_bridge sources into the workspace; show what rosdep would install, install nothing.
set -eo pipefail
cd ~/deletdis/ros2-corridor-swarm
. ./env.sh
cd ws/src
[ -d foxglove_bridge ] || git clone -q --depth 1 https://github.com/foxglove/ros-foxglove-bridge.git foxglove_bridge
echo "foxglove_bridge @ $(git -C foxglove_bridge rev-parse --short HEAD) ($(git -C foxglove_bridge rev-parse --abbrev-ref HEAD))"
echo "--- package.xml dependencies ---"
grep -o -E '<(build_|exec_|)depend[^>]*>[^<]+' foxglove_bridge/package.xml | sed 's/.*>//' | sort -u | tr '\n' ' '; echo
if grep -q 'rosx_introspection' foxglove_bridge/package.xml && ! ros2 pkg list | grep -qx rosx_introspection; then
  [ -d rosx_introspection ] || git clone -q --depth 1 https://github.com/facontidavide/rosx_introspection.git
  echo "rosx_introspection cloned @ $(git -C rosx_introspection rev-parse --short HEAD)"
fi
cd ..
echo "--- rosdep simulate (nothing installed) ---"
rosdep install --from-paths src --ignore-src -y --simulate 2>&1 | grep -v '^$' | tail -20
