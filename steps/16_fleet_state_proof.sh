#!/usr/bin/env bash
# ADR-0015 proof: fleet_state publishes /fleet/state for every launched vehicle, mode and goal_id follow
# what the vehicle layer does, and the planner's list_vehicles answers from that topic alone.
# Runs on its own ROS domain so the demo stack on 42 is untouched. Prints PROOF 20 PASS|FAIL.
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
source ./steps/lib.sh
export ROS_DOMAIN_ID=${PROOF_DOMAIN:-77}
LOG=~/deletdis/log/16_fleet_state.log; exec > >(tee -a "$LOG") 2>&1
echo "=== $(date -Is) fleet_state proof on domain $ROS_DOMAIN_ID"
T=/tmp/fleet16; rm -rf $T; mkdir -p $T
cleanup() { kill_ours; sleep 1; }
cleanup
fail=0
check() { if [ "$2" = "$3" ]; then echo "ok   $1: $2"; else echo "FAIL $1: got '$2', want '$3'"; fail=1; fi; }
snapshot() {   # one FleetState as "name mode inside goal_id?" lines
  timeout 10 ros2 topic echo --once /fleet/state corridor_msgs/msg/FleetState 2>/dev/null \
    | awk '/^  name:/{n=$2} /^  mode:/{m=$2} /^  goal_id:/{g=($2==""||$2=="\x27\x27")?"-":"set"} /^  inside_corridor:/{print n, m, $2, g}'   # inside_corridor is the last field of a VehicleState
}

ros2 launch corridor_nav kinematic.launch.py n:=3 foxglove:=false > $T/kin.log 2>&1 &
sleep 8
echo "--- idle fleet"
snapshot | tee $T/idle.txt
check "vehicles listed" "$(awk '{print $1}' $T/idle.txt | tr '\n' ' ')" "v1 v2 v3 "
check "all inside"     "$(awk '{print $3}' $T/idle.txt | sort -u | tr '\n' ' ')" "true "
check "all idle"       "$(awk '{print $2}' $T/idle.txt | sort -u | tr '\n' ' ')" "idle "

echo "--- v2 patrols, v3 gets a raw Path (goto), v1 stays"
timeout 120 ros2 action send_goal /v2/patrol corridor_msgs/action/Patrol "{vehicle: v2, corridor_id: demo, laps: 0, speed_mps: 0.0, clockwise: false}" > $T/goal.txt 2>&1 &
ros2 run corridor_nav send_path --ros-args -r __ns:=/v3 -p "points:=[70.0,20.0,70.0,150.0]" 2>&1 | grep -oE "sent.*|no subscriber.*"   # v3 spawns at (70,20); 130 m north keeps it following well past the snapshot
sleep 4
snapshot | tee $T/busy.txt
check "v1 idle"        "$(awk '$1=="v1"{print $2, $4}' $T/busy.txt)" "idle -"
check "v2 patrol"      "$(awk '$1=="v2"{print $2, $4}' $T/busy.txt)" "patrol set"
check "v3 goto"        "$(awk '$1=="v3"{print $2, $4}' $T/busy.txt)" "goto -"

echo "--- the planner answers list_vehicles from /fleet/state (dry run, no model)"
ros2 run corridor_planner mission_planner --ros-args -p dry_run:=true -p "vehicles:=[v1,v2,v3]" \
  -p "corridor_geojson:=$(ros2 pkg prefix corridor_nav)/share/corridor_nav/config/corridor_demo.geojson" > $T/planner.log 2>&1 &
sleep 4
timeout 15 ros2 topic echo --once /mission/llm_trace std_msgs/msg/String --field data > $T/trace.txt 2>/dev/null &
sleep 1
ros2 topic pub --once /mission/tool_call std_msgs/msg/String "{data: '{\"name\": \"list_vehicles\", \"input\": {}}'}" > /dev/null 2>&1
sleep 3
python3 - "$T/trace.txt" <<'EOF'
import json, sys
raw = open(sys.argv[1]).readline().strip()   # the echo prints the JSON string, then a '---' separator
ev = json.loads(raw)
modes = {v['vehicle']: v['mode'] for v in ev['result']['vehicles']}
print('planner sees', modes)
sys.exit(0 if modes == {'v1': 'idle', 'v2': 'patrol', 'v3': 'goto'} else 1)
EOF
[ $? -eq 0 ] && echo "ok   planner modes" || { echo "FAIL planner modes"; fail=1; }

echo "--- v2 cancelled: back to idle, goal_id cleared"
ros2 topic pub --once /v2/path nav_msgs/msg/Path "{}" > /dev/null 2>&1   # a foreign empty Path preempts the patrol
sleep 4
snapshot | tee $T/after.txt
check "v2 idle again"  "$(awk '$1=="v2"{print $2, $4}' $T/after.txt)" "idle -"

cleanup
[ $fail -eq 0 ] && echo "PROOF 20 PASS" || echo "PROOF 20 FAIL"
echo "=== $(date -Is) done"
