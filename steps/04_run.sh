#!/usr/bin/env bash
# Day 1 step 4: git hygiene for the cloned sources, then launch and prove the topics and the bridge port.
set -eo pipefail
cd "${CORRIDOR_ROOT:-$HOME/deletdis/ros2-corridor-swarm}"
. ./env.sh
# Cloned sources are not ours: track them as a vcs repos file, not as embedded repos.
cat > ws/deps.repos <<'REPOS'
repositories:
  foxglove_bridge:
    type: git
    url: https://github.com/foxglove/ros-foxglove-bridge.git
    version: 0.8.5
  rosx_introspection:
    type: git
    url: https://github.com/facontidavide/rosx_introspection.git
    version: main
REPOS
grep -qxF 'ws/src/foxglove_bridge/' .gitignore || printf 'ws/src/foxglove_bridge/\nws/src/rosx_introspection/\n' >> .gitignore
git rm -r -q --cached ws/src/foxglove_bridge ws/src/rosx_introspection 2>/dev/null || true
git add -A && git commit -q -m "chore: cloned sources via ws/deps.repos (vcs import ws/src < ws/deps.repos), not embedded repos" && git log --oneline -1
# Launch in its own tmux window so it keeps running after this script ends.
tmux kill-window -t ros2:run 2>/dev/null || true
tmux new-window -d -t ros2 -n run -c ~/deletdis/ros2-corridor-swarm
tmux send-keys -t ros2:run ". ./env.sh && ros2 launch corridor_sim day1.launch.py 2>&1 | tee ~/deletdis/log/day1_launch.log" Enter
sleep 6
echo "--- launch log ---"; tail -n 8 ~/deletdis/log/day1_launch.log
echo "--- topics ---"; timeout 5 ros2 topic list | tr '\n' ' '; echo
echo "--- hz /vehicle/fix ---"; timeout 6 ros2 topic hz /vehicle/fix --window 20 2>&1 | grep -m1 'average rate' || echo "no hz line"
echo "--- one fix ---"; timeout 6 ros2 topic echo --once /vehicle/fix 2>&1 | grep -E 'latitude|longitude|frame_id' | tr '\n' ' '; echo
echo "--- tf ---"; timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -m1 'Translation'
echo "--- port 8765 ---"; ss -ltnp 2>/dev/null | grep 8765 || echo "8765 not listening"
