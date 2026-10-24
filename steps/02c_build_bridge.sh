#!/usr/bin/env bash
# Day 1 step 2c: rosx_introspection from source (foxglove_bridge 0.8.5 needs it), then build everything.
set -eo pipefail
cd ~/deletdis/ros2-corridor-swarm
. ./env.sh
cd ws/src
[ -d rosx_introspection ] || git clone -q --depth 1 https://github.com/facontidavide/rosx_introspection.git
echo "rosx_introspection @ $(git -C rosx_introspection rev-parse --short HEAD); deps: $(grep -o -E '<(build_|exec_|)depend[^>]*>[^<]+' rosx_introspection/package.xml | sed 's/.*>//' | sort -u | tr '\n' ' ')"
cd ..
echo "--- rosdep simulate ---"
sim=$(rosdep install --from-paths src --ignore-src -y --simulate 2>&1 | grep -v '^$' || true)
echo "${sim:-nothing to install}"
if echo "$sim" | grep -q 'ros-jazzy'; then echo "ABORT: rosdep wants ROS apt packages"; exit 1; fi
pkgs=$(echo "$sim" | grep -o 'apt-get install -y [^ ]*' | awk '{print $4}' | tr '\n' ' ')
[ -z "$pkgs" ] || echo "$SUDO_PASS" | sudo -S -k apt-get install -y -q $pkgs 2>&1 | tail -1
time colcon build --symlink-install --packages-up-to foxglove_bridge corridor_sim --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -8
source install/setup.bash
echo "executables: $(ros2 pkg executables corridor_sim | tr '\n' ' ') | foxglove: $(ros2 pkg executables foxglove_bridge | head -1)"
cd .. && git add -A && git commit -q -m "feat: foxglove_bridge 0.8.5 and rosx_introspection from source, corridor_sim with a vehicle_sim node

ros-foxglove-bridge main is ROS 1 only now; 0.8.5 is the newest tag with ros2_foxglove_bridge.
rosx_introspection cloned because rosdep would otherwise pull ros-jazzy-rosx-introspection from apt.
apt: libwebsocketpp-dev nlohmann-json3-dev (what rosdep resolved)
build: colcon build --symlink-install --packages-up-to foxglove_bridge corridor_sim" && git log --oneline -1
