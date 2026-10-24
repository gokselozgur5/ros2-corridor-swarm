#!/usr/bin/env bash
# Issue #16, server side only: cot_bridge connects to the local FreeTAKServer and streams PLI for v1..v3.
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
LOG=~/deletdis/log/13_cot_proof.log; exec > >(tee -a "$LOG") 2>&1
echo "=== $(date -Is) cot proof"
cleanup() { pkill -f "ros2 launch corridor" 2>/dev/null; pkill -f "corridor_nav/lib" 2>/dev/null; pkill -f "corridor_sim/lib" 2>/dev/null; pkill -f "corridor_cot/lib" 2>/dev/null; sleep 1; }
cleanup
ss -ltn | grep -q ":8087" || { echo "no TAK server on 8087"; echo "PROOF 16 FAIL"; exit 1; }
ros2 launch corridor_nav kinematic.launch.py n:=3 > /tmp/kin16.log 2>&1 &
sleep 6
ros2 run corridor_cot cot_bridge --ros-args -p "vehicles:=[v1,v2,v3]" -p host:=127.0.0.1 -p period_s:=1.0 > /tmp/cot16.log 2>&1 &
sleep 8
grep -m1 "connected to TAK server" /tmp/cot16.log || echo "bridge did not connect"
before=$(sudo -n docker logs corridor-fts 2>&1 | wc -l)
sleep 5
sudo -n docker logs corridor-fts 2>&1 | tail -n +$before | grep -i -m3 "corridor-v\|client\|connect" | cut -c1-160
grep -q "connected to TAK server" /tmp/cot16.log && ! grep -q "TAK send failed" /tmp/cot16.log && echo "PROOF 16 PASS (server side; ATAK display needs the emulator)" || { echo "PROOF 16 FAIL"; tail -5 /tmp/cot16.log; }
cleanup
echo "=== $(date -Is) done"
