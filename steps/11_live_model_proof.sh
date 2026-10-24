#!/usr/bin/env bash
# Issues #12 #13 #14, model path. Needs ANTHROPIC_API_KEY in ./.env (env.sh sources it). Costs a few cents.
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
[ -n "$ANTHROPIC_API_KEY" ] || { echo "ANTHROPIC_API_KEY is not set; put it in .env"; exit 1; }
LOG=~/deletdis/log/11_live_model_proof.log; exec > >(tee -a "$LOG") 2>&1
echo "=== $(date -Is) live model proof"
cleanup() { pkill -f "ros2 launch corridor" 2>/dev/null; pkill -f "corridor_planner/lib" 2>/dev/null; pkill -f "corridor_nav/lib" 2>/dev/null; pkill -f "gz sim -s" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null; pkill -f robot_state_publisher 2>/dev/null; pkill -f "ros2 topic echo" 2>/dev/null; sleep 1; }
cleanup
# Gazebo backend so describe_scene has a real camera; follower + patrol server on top of it.
ros2 launch corridor_gz sim.launch.py use_ros2_control:=true > /tmp/sim_live.log 2>&1 &
sleep 20
ros2 run corridor_nav waypoint_follower --ros-args -r __ns:=/v1 -p speed_mps:=2.0 -p lookahead_m:=3.0 > /tmp/follower_live.log 2>&1 &
ros2 launch corridor_planner mission.launch.py status_loop:=true > /tmp/mission_live.log 2>&1 &
sleep 8
timeout 150 ros2 topic echo /mission/llm_trace --field data > /tmp/trace_live.txt 2>/dev/null &
timeout 150 ros2 topic echo /mission/reply --field data > /tmp/reply_live.txt 2>/dev/null &
timeout 150 ros2 topic echo /mission/summary_cost --field data > /tmp/cost_live.txt 2>/dev/null &
sleep 2
echo "--- #12 request: send v1 to patrol"
ros2 topic pub --once /mission/request std_msgs/String "{data: 'send v1 to patrol the corridor, one lap'}" > /dev/null 2>&1
for i in $(seq 1 60); do grep -q '"event": "reply"' /tmp/trace_live.txt 2>/dev/null && break; sleep 1; done
grep -o '"name": "[a-z_]*"' /tmp/trace_live.txt | tr '\n' ' '; echo
grep -q '"name": "patrol"' /tmp/trace_live.txt && grep -q '"accepted": true' /tmp/trace_live.txt && echo "PROOF 12 PASS" || echo "PROOF 12 FAIL"
echo "--- #13 request: what does v1 see"
ros2 topic pub --once /mission/request std_msgs/String "{data: 'what does v1 see right now? name the colours of any objects'}" > /dev/null 2>&1
for i in $(seq 1 60); do [ "$(grep -c '"event": "reply"' /tmp/trace_live.txt 2>/dev/null)" -ge 2 ] && break; sleep 1; done
tail -1 /tmp/reply_live.txt | grep -v -- --- | cut -c1-300
grep -q '"name": "describe_scene"' /tmp/trace_live.txt && echo "PROOF 13 PASS if the reply above names what is in front of v1" || echo "PROOF 13 FAIL"
echo "--- #14 status loop cost (two ticks)"
sleep 65
tail -1 /tmp/cost_live.txt | grep -v -- ---
cleanup
echo "=== $(date -Is) done"
