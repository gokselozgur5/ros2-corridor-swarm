#!/usr/bin/env bash
# Issue #13, debug path: Gazebo camera frame -> describe_scene tool -> image block (no model call).
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
source ./steps/lib.sh
LOG=~/deletdis/log/10_scene_proof.log; exec > >(tee -a "$LOG") 2>&1
echo "=== $(date -Is) scene proof"
cleanup() { kill_ours; sleep 1; }
cleanup
ros2 launch corridor_gz sim.launch.py > /tmp/sim13.log 2>&1 &
sleep 15
ros2 launch corridor_planner mission.launch.py dry_run:=true start_patrol:=false > /tmp/mission13.log 2>&1 &
sleep 6
timeout 25 ros2 topic echo /mission/llm_trace --field data > /tmp/trace13.txt 2>/dev/null &
sleep 2
ros2 topic pub --once /mission/tool_call std_msgs/String "{data: '{\"name\": \"describe_scene\", \"input\": {\"vehicle\": \"v1\"}}'}" > /dev/null 2>&1
sleep 4
grep -o '"png_bytes": [0-9]*' /tmp/trace13.txt | head -1
grep -o '"content_blocks": \[[^]]*\]' /tmp/trace13.txt | head -1
grep -q '"content_blocks": \["image", "text"\]' /tmp/trace13.txt && echo "PROOF 13 PASS (debug path; naming the red box needs the model)" || { echo "PROOF 13 FAIL"; grep -i "error\|died" /tmp/mission13.log | head -3; cat /tmp/trace13.txt | cut -c1-300 | head -3; }
cleanup
echo "=== $(date -Is) done"
