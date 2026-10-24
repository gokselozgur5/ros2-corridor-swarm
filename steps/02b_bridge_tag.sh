#!/usr/bin/env bash
# Day 1 step 2b: ros-foxglove-bridge main is ROS 1 only now; take the newest tag that still ships ros2_foxglove_bridge, then build.
set -eo pipefail
cd ~/deletdis/ros2-corridor-swarm
. ./env.sh
cd ws/src
rm -rf foxglove_bridge
git clone -q https://github.com/foxglove/ros-foxglove-bridge.git foxglove_bridge
cd foxglove_bridge
tag=""
for t in $(git tag --sort=-v:refname); do
  if git ls-tree -d "$t" ros2_foxglove_bridge >/dev/null 2>&1 && [ -n "$(git ls-tree -d "$t" ros2_foxglove_bridge)" ]; then tag="$t"; break; fi
done
[ -n "$tag" ] || { echo "no tag with ros2_foxglove_bridge found"; exit 1; }
git checkout -q "$tag"
echo "foxglove_bridge tag $tag ($(git log -1 --format=%cs)) ros2 dir: $(ls ros2_foxglove_bridge | head -3 | tr '\n' ' ')"
cd ../..
echo "--- rosdep simulate ---"
sim=$(rosdep install --from-paths src --ignore-src -y --simulate 2>&1 | grep -v '^$' || true)
echo "$sim"
if echo "$sim" | grep -q 'ros-jazzy'; then echo "ABORT: rosdep wants ROS apt packages"; exit 1; fi
pkgs=$(echo "$sim" | grep -o 'apt-get install -y [^ ]*' | awk '{print $4}' | tr '\n' ' ')
[ -z "$pkgs" ] || echo "$SUDO_PASS" | sudo -S -k apt-get install -y -q $pkgs 2>&1 | tail -1
time colcon build --symlink-install --packages-select foxglove_bridge corridor_sim --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -6
source install/setup.bash
echo "executables: $(ros2 pkg executables corridor_sim | tr '\n' ' ') | foxglove: $(ros2 pkg executables foxglove_bridge | head -1)"
cd .. && git add -A && git commit -q -m "feat: foxglove_bridge (ROS 2 tag $tag) from source and corridor_sim with a vehicle_sim node

ros-foxglove-bridge main is ROS 1 only now; tag $tag is the newest with ros2_foxglove_bridge.
apt: libwebsocketpp-dev nlohmann-json3-dev (what rosdep resolved; no ROS apt packages)
build: colcon build --symlink-install --packages-select foxglove_bridge corridor_sim" && git log --oneline -1
