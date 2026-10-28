#!/usr/bin/env bash
# REQ-OPERATOROPS-20 and the #19 fix: kinematic.launch.py starts map_overlay, which publishes the corridor polygon, one inset
# route per vehicle and one heading line per vehicle on /map/overlay (foxglove_msgs/GeoJSON) and /map/markers, each heading
# line 12 m long from the fix and the 3D arrow starting where the fix is in map metres (not at odom x/y). OVERLAY=false runs
# the launch without the node and must fail. Prints PROOF 19 OVERLAY PASS|FAIL and exits 1 on FAIL.
# Limit: the unicycle sim's odom is the map position, so an overlay drawing from odom would pass here too; the fix/odom
# divergence is covered by test_map_overlay.py, this proof covers the launch wiring, the message shapes and the geometry.
cd ~/deletdis/ros2-corridor-swarm
source ./env.sh
source ./steps/lib.sh
LOG=~/deletdis/log/18_overlay_proof.log; exec > >(tee -a "$LOG") 2>&1
echo "=== $(date -Is) overlay proof on domain $ROS_DOMAIN_ID (overlay:=${OVERLAY:-true})"
cleanup() { kill_ours; sleep 1; }
cleanup
trap cleanup EXIT
ros2 launch corridor_nav kinematic.launch.py n:=3 foxglove:=false "overlay:=${OVERLAY:-true}" > /tmp/kin18.log 2>&1 &
sleep 8
timeout 15 ros2 topic echo --once /map/overlay foxglove_msgs/msg/GeoJSON --field geojson > /tmp/overlay18.json 2>/dev/null
timeout 15 ros2 topic echo --once /map/markers visualization_msgs/msg/MarkerArray > /tmp/markers18.yaml 2>/dev/null
for v in v1 v2 v3; do
  timeout 10 ros2 topic echo --once /$v/fix sensor_msgs/msg/NavSatFix 2>/dev/null | grep -E "^(latitude|longitude):" | tr '\n' ' ' | sed "s/^/$v /"; echo
done > /tmp/fix18.txt
python3 - /tmp/overlay18.json /tmp/markers18.yaml /tmp/fix18.txt "$(ros2 pkg prefix corridor_nav)/share/corridor_nav/config/corridor_demo.geojson" <<'PY'
import json, math, re, sys
import yaml
from corridor_nav.geofence import LocalFrame, load_geojson_polygon
geo, markers, fixes = [open(p).read() for p in sys.argv[1:4]]
frame = LocalFrame(*load_geojson_polygon(sys.argv[4])[1][0])   # the launch's corridor: the nodes' map frame origin
raw = geo.strip().splitlines()[0] if geo.strip() else ''
try:
    features = json.loads(raw)['features']
except Exception as exc:
    print(f'FAIL nothing usable on /map/overlay ({type(exc).__name__}: {raw[:60]!r})')
    print('PROOF 19 OVERLAY FAIL')
    sys.exit(1)
fail = []
print('features:', [f['properties']['name'] for f in features])
if sum(f['geometry']['type'] == 'Polygon' for f in features) != 1:
    fail.append('corridor polygon missing')
try:
    marker_list = next(yaml.safe_load_all(markers))['markers']
except Exception as exc:
    print(f'FAIL nothing usable on /map/markers ({type(exc).__name__})')
    print('PROOF 19 OVERLAY FAIL')
    sys.exit(1)
arrows = {m['ns']: m for m in marker_list if m['type'] == 0}   # Marker.ARROW, one per vehicle
k = 111320.0
for v in ('v1', 'v2', 'v3'):
    m = re.search(rf'^{v} latitude: ([-\d.]+) longitude: ([-\d.]+)', fixes, re.M)
    if not m:
        fail.append(f'{v}: no fix read'); continue
    lat, lon = float(m.group(1)), float(m.group(2))
    if not any(f['properties']['name'] == f'{v} route' for f in features):
        fail.append(f'{v}: no route')
    heading = [f for f in features if f['properties']['name'] == f'{v} heading']
    if not heading:
        fail.append(f'{v}: no heading line'); continue
    (lon0, lat0), (lon1, lat1) = heading[0]['geometry']['coordinates']
    length = math.hypot((lat1 - lat0) * k, (lon1 - lon0) * k * math.cos(math.radians(lat)))
    arrow = arrows.get(f'{v}_heading')
    if arrow is None:
        fail.append(f'{v}: no heading arrow on /map/markers'); continue
    (ax, ay), (tx, ty) = [(pt['x'], pt['y']) for pt in arrow['points'][:2]]
    off = math.dist((ax, ay), frame.to_xy(lat, lon))
    print(f'{v}: heading line {length:.1f} m long; arrow starts {off:.2f} m from the fix in map metres, {math.dist((ax, ay), (tx, ty)):.1f} m long')
    if not 10.0 < length < 14.0:
        fail.append(f'{v}: heading line {length:.1f} m, expected 12 (tip not drawn from the fix)')
    if off > 1.0:
        fail.append(f'{v}: arrow starts {off:.1f} m from the fix (odom x/y, not the fix)')
    if not 10.0 < math.dist((ax, ay), (tx, ty)) < 14.0:
        fail.append(f'{v}: arrow {math.dist((ax, ay), (tx, ty)):.1f} m, expected 12')
strips = sum(m['type'] == 4 for m in marker_list)   # Marker.LINE_STRIP: corridor + routes
print(f'markers: {len(arrows)} arrows, {strips} line strips')
if (len(arrows), strips) != (3, 4):
    fail.append(f'markers: {len(arrows)} arrows and {strips} line strips, expected 3 and 4')
for f in fail:
    print('FAIL', f)
print('PROOF 19 OVERLAY PASS' if not fail else 'PROOF 19 OVERLAY FAIL')
sys.exit(1 if fail else 0)
PY
rc=$?
echo "=== $(date -Is) done"
exit $rc
