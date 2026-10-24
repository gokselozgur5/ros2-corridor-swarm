#!/usr/bin/env bash
# Issue #5: ros2_control, ros2_controllers, gz_ros2_control and their dependencies from source.
set -e
LOG=~/deletdis/log/09_ros2_control_build.log; mkdir -p "$(dirname "$LOG")"; exec > >(tee "$LOG") 2>&1
echo "=== $(date -Is) start"
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
vcs import ws/src < ws/deps.repos
# Only what we use from the big repos: controllers other than diff_drive and joint_state_broadcaster are skipped.
for p in ws/src/ros2_controllers/*/; do
  case "$(basename $p)" in diff_drive_controller|joint_state_broadcaster|ros2_controllers|ros2_controllers_test_nodes) ;; *) touch "$p/COLCON_IGNORE";; esac
done
for p in ws/src/diagnostics/*/; do case "$(basename $p)" in diagnostic_updater|diagnostic_aggregator|diagnostics|self_test) ;; *) touch "$p/COLCON_IGNORE";; esac; done
for p in ws/src/gz_ros2_control/*/; do case "$(basename $p)" in gz_ros2_control|gz_ros2_control_demos) touch "$p/COLCON_IGNORE";; esac; done
rm -f ws/src/gz_ros2_control/gz_ros2_control/COLCON_IGNORE
for p in ws/src/ros2_control/*/; do case "$(basename $p)" in rqt_controller_manager|ros2_control_test_assets|ros2_control) touch "$p/COLCON_IGNORE";; esac; done
rm -f ws/src/ros2_control/ros2_control_test_assets/COLCON_IGNORE   # hardware_interface tests reference it; cheap
cd ws
# rsl declares libexpected-dev (an apt key) rather than the tl_expected package, so colcon cannot order it: build it first.
colcon build --symlink-install --packages-select tl_expected tl-expected --cmake-args -DBUILD_TESTING=OFF -DEXPECTED_BUILD_TESTS=OFF
source install/setup.bash
colcon build --symlink-install --packages-up-to gz_ros2_control diff_drive_controller joint_state_broadcaster \
  --cmake-args -DBUILD_TESTING=OFF
echo "exit=$?"
echo "=== $(date -Is) done"
