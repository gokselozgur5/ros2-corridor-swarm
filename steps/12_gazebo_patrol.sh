#!/usr/bin/env bash
# End to end on physics: Gazebo + ros2_control diff_drive + follower + patrol action, one lap. Not a PLAN proof, a sanity run.
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
LOG=~/deletdis/log/12_gazebo_patrol.log; exec > >(tee -a "$LOG") 2>&1
echo "=== $(date -Is) gazebo patrol"
cleanup() { pkill -f "ros2 launch corridor" 2>/dev/null; pkill -f "corridor_nav/lib" 2>/dev/null; pkill -f "gz sim -s" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null; pkill -f robot_state_publisher 2>/dev/null; pkill -f "ros2 topic echo" 2>/dev/null; sleep 1; }
cleanup
ros2 launch corridor_gz sim.launch.py use_ros2_control:=true > /tmp/sim_patrol.log 2>&1 &
sleep 20
ros2 run corridor_nav waypoint_follower --ros-args -r __ns:=/v1 -p speed_mps:=3.0 -p lookahead_m:=4.0 -p arrive_tolerance_m:=2.0 -p use_sim_time:=true > /tmp/follower_patrol.log 2>&1 &
ros2 run corridor_nav patrol_server --ros-args -r __ns:=/v1 -p "corridor_geojson:=$(ros2 pkg prefix corridor_nav)/share/corridor_nav/config/corridor_demo.geojson" -p vehicle:=v1 -p inset_m:=40.0 -p use_sim_time:=true > /tmp/patrol_gz.log 2>&1 &
sleep 4
timeout 420 ros2 topic echo /v1/follower/cross_track --field data > /tmp/xte_gz.txt 2>/dev/null &
timeout 420 ros2 action send_goal --feedback /v1/patrol corridor_msgs/action/Patrol "{vehicle: v1, corridor_id: demo, laps: 1, speed_mps: 0.0, clockwise: false}" > /tmp/goal_gz.txt 2>&1
grep -c "lap_progress" /tmp/goal_gz.txt | sed 's/^/feedback messages: /'
grep -A4 "^Result" /tmp/goal_gz.txt
awk 'function abs(v){return v<0?-v:v} /^-?[0-9]/{if(abs($1)>m)m=abs($1); n++} END{printf "cross-track samples=%d max_abs=%.2f m\n", n, m}' /tmp/xte_gz.txt
grep -q "result: done" /tmp/goal_gz.txt && echo "GAZEBO PATROL PASS" || echo "GAZEBO PATROL FAIL"
d=$(grep -o "distance_m: [0-9.]*" /tmp/goal_gz.txt | tail -1 | awk '{print $2}')
grep -q "result: done" /tmp/goal_gz.txt && awk -v d="$d" 'BEGIN{exit !(d>380)}' && echo "GAZEBO PATROL PASS" || echo "GAZEBO PATROL FAIL (distance ${d:-none} m, a lap is about 480 m)"
echo "=== $(date -Is) done"
