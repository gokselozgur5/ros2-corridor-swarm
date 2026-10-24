#!/usr/bin/env bash
# foxglove_msgs for the map overlay: only ros/src/foxglove_msgs out of the foxglove-sdk checkout is built.
set -e
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
vcs import ws/src < ws/deps.repos
rm -rf ws/src/foxglove_msgs ws/build/foxglove_msgs ws/install/foxglove_msgs
touch ws/src/foxglove-sdk/COLCON_IGNORE
ln -sfn ../foxglove-sdk/ros/src/foxglove_msgs ws/src/foxglove_msgs    # colcon follows the symlink, the ignore stays on the rest of the sdk
cd ws
colcon build --symlink-install --packages-select foxglove_msgs --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 interface show foxglove_msgs/msg/GeoJSON | tail -1
