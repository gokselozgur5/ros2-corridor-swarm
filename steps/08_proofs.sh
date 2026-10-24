#!/usr/bin/env bash
# Phase 1/2 proofs after steps/07_build_ours.sh. Each block prints PROOF <issue> PASS|FAIL.
# Usage: ./steps/08_proofs.sh [3|4|7|8|9|all]
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
LOG=~/deletdis/log/08_proofs.log; exec > >(tee -a "$LOG") 2>&1
which=${1:-all}
echo "=== $(date -Is) proofs: $which"

cleanup() { pkill -f "ros2 launch corridor" 2>/dev/null; pkill -f "corridor_nav/lib" 2>/dev/null; pkill -f "corridor_sim/lib" 2>/dev/null; pkill -f "corridor_gz" 2>/dev/null; pkill -f "gz sim -s" 2>/dev/null; pkill -f parameter_bridge 2>/dev/null; pkill -f robot_state_publisher 2>/dev/null; pkill -f "ros_gz_sim/create|ros_gz_sim create" 2>/dev/null; pkill -f "ros2 topic echo" 2>/dev/null; rm -f /tmp/status7.txt /tmp/status8.txt /tmp/xte7.txt /tmp/gf8.txt; sleep 1; }

if [[ $which == 3 || $which == all ]]; then
  echo "--- #3 xacro -> URDF -> check_urdf"
  xacro "$(ros2 pkg prefix corridor_description)/share/corridor_description/urdf/ugv.urdf.xacro" sim:=true use_ros2_control:=true > /tmp/ugv.urdf \
    && check_urdf /tmp/ugv.urdf | head -3 && echo "PROOF 3 PASS" || echo "PROOF 3 FAIL"
fi

if [[ $which == 7 || $which == all ]]; then
  echo "--- #7 kinematic follower on a 3-leg square, cross-track error"
  cleanup
  ros2 launch corridor_nav kinematic.launch.py > /tmp/kin7.log 2>&1 &
  sleep 4
  timeout 200 ros2 topic echo /v1/follower/cross_track --field data > /tmp/xte7.txt 2>/dev/null &
  timeout 200 ros2 topic echo /v1/follower/status --field data > /tmp/status7.txt 2>/dev/null &
  sleep 2
  ros2 run corridor_nav send_path --ros-args -r __ns:=/v1 -p "points:=[20.0,20.0,80.0,20.0,80.0,80.0,20.0,80.0]" 2>&1 | grep -oE "sent.*|no subscriber.*"
  for i in $(seq 1 200); do grep -q arrived /tmp/status7.txt 2>/dev/null && break; sleep 1; done; s=$(grep -v -- --- /tmp/status7.txt 2>/dev/null | tail -1)
  echo "status after ${i}s: ${s:-unknown}"
  awk 'function abs(v){return v<0?-v:v} {if(abs($1)>m)m=abs($1); n++} END{printf "cross-track samples=%d max_abs=%.2f m\n", n, m; exit !(n>10 && m<1.0)}' /tmp/xte7.txt \
    && [[ $s == arrived ]] && echo "PROOF 7 PASS" || echo "PROOF 7 FAIL"
  cleanup
fi

if [[ $which == 8 || $which == all ]]; then
  echo "--- #8 geofence: path leaving the corridor -> EXIT event and follower cancelled"
  cleanup
  ros2 launch corridor_nav kinematic.launch.py > /tmp/kin8.log 2>&1 &
  sleep 4
  timeout 40 ros2 topic echo /fleet/geofence_events > /tmp/gf8.txt 2>/dev/null &
  timeout 60 ros2 topic echo /v1/follower/status --field data > /tmp/status8.txt 2>/dev/null &
  sleep 2
  ros2 run corridor_nav send_path --ros-args -r __ns:=/v1 -p "points:=[20.0,20.0,20.0,-60.0]" 2>&1 | grep -oE "sent.*|no subscriber.*"   # south, out of the box
  t0=$(date +%s.%N); ev=""
  for i in $(seq 1 60); do grep -q "^type: 0" /tmp/gf8.txt 2>/dev/null && ev=yes && break; sleep 0.5; done
  sleep 1; s=$(grep -v -- --- /tmp/status8.txt 2>/dev/null | tail -1)
  grep -m1 -B2 -A3 "^type: 0" /tmp/gf8.txt | head -8
  echo "exit event: ${ev:-no}; follower status: ${s:-unknown}"
  [[ $ev == yes && $s == idle ]] && echo "PROOF 8 PASS" || echo "PROOF 8 FAIL"
  cleanup
fi

if [[ $which == 9 || $which == all ]]; then
  echo "--- #9 patrol action: one lap"
  cleanup
  ros2 launch corridor_nav kinematic.launch.py > /tmp/kin9.log 2>&1 &
  sleep 4
  ros2 run corridor_nav patrol_server --ros-args -r __ns:=/v1 -p "corridor_geojson:=$(ros2 pkg prefix corridor_nav)/share/corridor_nav/config/corridor_demo.geojson" -p vehicle:=v1 -p inset_m:=30.0 > /tmp/patrol9.log 2>&1 &
  sleep 3
  timeout 400 ros2 action send_goal --feedback /v1/patrol corridor_msgs/action/Patrol "{vehicle: v1, corridor_id: demo, laps: 1, speed_mps: 0.0, clockwise: false}" > /tmp/goal9.txt 2>&1
  grep -c "lap_progress" /tmp/goal9.txt | sed 's/^/feedback messages: /'
  grep -A4 "^Result" /tmp/goal9.txt
  grep -q "result: done" /tmp/goal9.txt && echo "PROOF 9 PASS" || echo "PROOF 9 FAIL"
  cleanup
fi

if [[ $which == 4 || $which == all ]]; then
  echo "--- #4 Gazebo world, v1 spawned, sensor topics under sim time"
  cleanup
  ros2 launch corridor_gz sim.launch.py > /tmp/sim4.log 2>&1 &
  sleep 15
  pass=yes
  for t in /v1/scan /v1/imu /v1/fix /v1/camera/image /v1/odom; do
    r=$(timeout 8 ros2 topic hz $t 2>&1 | grep -m1 "average rate" | awk '{print $3}')
    echo "$t: ${r:-none} Hz"; [[ -z $r ]] && pass=no
  done
  ros2 topic echo --once /v1/fix --field latitude 2>/dev/null | sed 's/^/fix latitude: /'
  [[ $pass == yes ]] && echo "PROOF 4 PASS" || { echo "PROOF 4 FAIL"; tail -20 /tmp/sim4.log; }
  cleanup
fi
echo "=== $(date -Is) proofs done"
