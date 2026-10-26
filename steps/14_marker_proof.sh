#!/usr/bin/env bash
# Issue #16, reverse direction: a marker sent to the TAK server by a second client reaches /mission/request through cot_bridge.
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
LOG=~/deletdis/log/14_marker_proof.log; exec > >(tee -a "$LOG") 2>&1
echo "=== $(date -Is) marker proof"
cleanup() { pkill -f "ros2 launch corridor" 2>/dev/null; pkill -f "corridor_nav/lib" 2>/dev/null; pkill -f "corridor_sim/lib" 2>/dev/null; pkill -f "corridor_cot/lib" 2>/dev/null; pkill -f "ros2 topic echo" 2>/dev/null; sleep 1; }
cleanup
ros2 launch corridor_nav kinematic.launch.py > /tmp/kin16b.log 2>&1 &
sleep 6
ros2 run corridor_cot cot_bridge --ros-args -p host:=127.0.0.1 -p period_s:=1.0 > /tmp/cot16b.log 2>&1 &
sleep 6
timeout 30 ros2 topic echo /mission/request --field data > /tmp/req16.txt 2>/dev/null &
sleep 2
python3 - <<'PY'
import socket, datetime
now = datetime.datetime.now(datetime.timezone.utc)
f = lambda t: t.strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'
xml = (f'<event version="2.0" uid="atak-marker-1" type="b-m-p-w" how="h-g-i-g-o" time="{f(now)}" start="{f(now)}" '
       f'stale="{f(now + datetime.timedelta(minutes=5))}"><point lat="52.7010" lon="-2.9985" hae="0" ce="9999999" le="9999999"/>'
       '<detail><contact callsign="RALLY1"/></detail></event>')
s = socket.create_connection(('127.0.0.1', 8087), timeout=5)
s.sendall(xml.encode()); s.sendall(xml.encode())
import time; time.sleep(3); s.close()
print('marker sent twice: the bridge forwards the placement once, the repeat at the same spot is dropped (marker_move_m)')
PY
for i in $(seq 1 20); do grep -q "RALLY1" /tmp/req16.txt 2>/dev/null && break; sleep 1; done
grep -m1 "RALLY1" /tmp/req16.txt || { echo "no request seen"; grep -i "marker\|warn\|error" /tmp/cot16b.log | tail -3; }
grep -q "RALLY1" /tmp/req16.txt && echo "PROOF 16 REVERSE PASS (marker relayed by the server reached /mission/request)" || echo "PROOF 16 REVERSE FAIL"
cleanup
echo "=== $(date -Is) done"
