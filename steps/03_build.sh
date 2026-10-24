#!/usr/bin/env bash
# Day 1 step 3: build foxglove_bridge without its tests (websocketpp 0.8.2 does not compile as C++20) and corridor_sim.
set -eo pipefail
cd ~/deletdis/ros2-corridor-swarm
. ./env.sh
cd ws
time colcon build --symlink-install --packages-select foxglove_bridge corridor_sim --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF 2>&1 | tail -6
source install/setup.bash
echo "executables: $(ros2 pkg executables corridor_sim | tr '\n' ' ') | foxglove: $(ros2 pkg executables foxglove_bridge | head -1)"
cd .. && git add -A && git commit -q -m "feat: foxglove_bridge 0.8.5 and rosx_introspection from source, corridor_sim with a vehicle_sim node

ros-foxglove-bridge main is ROS 1 only now; 0.8.5 is the newest tag with ros2_foxglove_bridge.
rosx_introspection cloned because rosdep would otherwise pull ros-jazzy-rosx-introspection from apt.
apt: libwebsocketpp-dev nlohmann-json3-dev rapidjson-dev (what rosdep resolved)
build: colcon build --symlink-install --packages-up-to foxglove_bridge corridor_sim --cmake-args -DBUILD_TESTING=OFF
BUILD_TESTING=OFF because the bridge's smoke_test is compiled as C++20 and websocketpp 0.8.2 does not parse under it." && git log --oneline -1
