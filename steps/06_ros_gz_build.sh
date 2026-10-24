#!/usr/bin/env bash
# Issue #2: ros_gz (branch jazzy) from source against the source Jazzy underlay and Gazebo Harmonic.
set -e
LOG=~/deletdis/log/06_ros_gz_build.log; mkdir -p "$(dirname "$LOG")"; exec > >(tee "$LOG") 2>&1
echo "=== $(date -Is) start"
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
vcs import ws/src < ws/deps.repos
export GZ_VERSION=harmonic
sudo apt-get install -y -q libcli11-dev libgflags-dev libdart6.13-all-dev
echo "--- rosdep simulate (must list no ros-jazzy-* package)"
rosdep install --from-paths ws/src/ros_gz --ignore-src -y --simulate || true
cd ws
colcon build --symlink-install --packages-up-to ros_gz_bridge ros_gz_sim \
  --cmake-args -DBUILD_TESTING=OFF -DGZ_VERSION=harmonic
echo "exit=$?"
echo "=== $(date -Is) done"
