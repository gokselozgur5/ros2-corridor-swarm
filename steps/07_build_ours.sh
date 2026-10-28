#!/usr/bin/env bash
# Build our packages plus xacro; then the Phase 1/2 proofs are run by hand from steps/08_proofs.sh.
set -e
LOG=~/deletdis/log/07_build_ours.log; mkdir -p "$(dirname "$LOG")"; exec > >(tee "$LOG") 2>&1
echo "=== $(date -Is) start"
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
vcs import ws/src < ws/deps.repos
python3 -m pip install --user --break-system-packages -q anthropic brotli zstandard 2>&1 | grep -v WARNING || true   # pip brotli: noble python3-brotli breaks httpx2 decoding
cd ws
colcon build --symlink-install --packages-up-to xacro corridor_msgs corridor_sim corridor_nav corridor_description corridor_gz corridor_planner corridor_cot \
  --cmake-args -DBUILD_TESTING=OFF
echo "exit=$?"
echo "=== $(date -Is) done"
