#!/usr/bin/env bash
# The demo stack (#34, DEMO.md) in one command, tmux session corridor-demo on its own domain:
#   steps/19_demo.sh up          Gazebo v1 (physics, autonomy) + kinematic v2, v3 + fleet nodes + planner + status loop + cot_bridge
#   steps/19_demo.sh up --kinematic   the same with three unicycles and no Gazebo (fallback)
#   steps/19_demo.sh check       every piece alive and publishing; prints DEMO CHECK PASS|FAIL, exit 1 on FAIL
#   steps/19_demo.sh patrol      the whole fleet on patrol until stopped (laps 0), without the planner
#   steps/19_demo.sh down        stops what up started and nothing else (CORRIDOR_DEMO marker), kills the session
# Thor is shared (another stack on domain 42 holds foxglove 8765 and the GPU): everything here runs on ROS_DOMAIN_ID
# ${DEMO_DOMAIN:-77}, GZ_PARTITION=corridor, foxglove on ${DEMO_FOX_PORT:-8766}. The Mac connects to ws://<thor>:8766.
# The planner needs ANTHROPIC_API_KEY in .env; without it the mission window runs dry_run:=true (tools work, no model).
set -eo pipefail
cd "${CORRIDOR_ROOT:-$HOME/deletdis/ros2-corridor-swarm}"
export ROS_DOMAIN_ID="${DEMO_DOMAIN:-77}" GZ_PARTITION=corridor ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source ./env.sh
SESSION=corridor-demo
FOX_PORT="${DEMO_FOX_PORT:-8766}"
FTS_PORT="${DEMO_FTS_PORT:-8087}"
LOGDIR="$HOME/deletdis/log/demo"; mkdir -p "$LOGDIR"
FLEET=v1,v2,v3
cmd="${1:-}"; shift || true

# One tmux window running one command, every process below it marked CORRIDOR_DEMO=<session> so down finds it.
window() {   # name, command
  local name="$1" run="$2"
  tmux new-window -d -t "$SESSION" -n "$name" -c "$PWD"
  tmux send-keys -t "$SESSION:$name" "export CORRIDOR_DEMO=$SESSION ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; source ./env.sh; $run 2>&1 | tee $LOGDIR/$name.log" Enter
}

marked() {   # pids of this user's processes carrying our marker
  local pid
  for pid in $(pgrep -u "$(id -u)"); do
    { tr '\0' '\n' < "/proc/$pid/environ"; } 2>/dev/null | grep -qx "CORRIDOR_DEMO=$SESSION" && echo "$pid"
  done
}

up() {
  local mode=mixed; [ "${1:-}" = "--kinematic" ] && mode=kinematic
  if tmux has-session -t "$SESSION" 2>/dev/null; then echo "session $SESSION exists; run down first"; exit 1; fi
  ss -ltn | grep -q ":$FTS_PORT " || echo "warning: no TAK server on $FTS_PORT, cot_bridge will keep reconnecting"
  ss -ltn | grep -q ":$FOX_PORT " && { echo "port $FOX_PORT is taken; set DEMO_FOX_PORT"; exit 1; }
  local dry=false; [ -n "${ANTHROPIC_API_KEY:-}" ] || { dry=true; echo "warning: no ANTHROPIC_API_KEY, planner in dry_run"; }
  rm -f "$LOGDIR"/*.log
  tmux new-session -d -s "$SESSION" -n work -c "$PWD"
  if [ "$mode" = mixed ]; then
    window gz "ros2 launch corridor_gz sim.launch.py use_ros2_control:=true autonomy:=true foxglove:=false"
    sleep 15   # the model, the controllers and the bridge are up before the fleet nodes start listening
    window kin "ros2 launch corridor_nav kinematic.launch.py first:=2 n:=2 fleet:=$FLEET foxglove_port:=$FOX_PORT"
  else
    window kin "ros2 launch corridor_nav kinematic.launch.py n:=3 foxglove_port:=$FOX_PORT"
  fi
  sleep 6
  window mission "ros2 launch corridor_planner mission.launch.py vehicles:=$FLEET start_patrol:=false fleet_state:=false status_loop:=true dry_run:=$dry"
  window cot "ros2 run corridor_cot cot_bridge --ros-args -p \"vehicles:=[v1,v2,v3]\" -p host:=127.0.0.1 -p port:=$FTS_PORT -p period_s:=1.0"
  echo "$mode" > "$LOGDIR/mode"
  echo "up ($mode): tmux attach -t $SESSION; foxglove ws://$(hostname -I | awk '{print $1}'):$FOX_PORT; domain $ROS_DOMAIN_ID"
}

check() {
  local fail=() mode; mode=$(cat "$LOGDIR/mode" 2>/dev/null || echo mixed)
  tmux has-session -t "$SESSION" 2>/dev/null || { echo "DEMO CHECK FAIL (no tmux session $SESSION)"; exit 1; }
  echo "=== $(date -Is) demo check ($mode) on domain $ROS_DOMAIN_ID"
  for w in kin mission cot $([ "$mode" = mixed ] && echo gz); do
    tmux list-windows -t "$SESSION" -F '#W' | grep -qx "$w" || fail+=("window $w gone")
  done
  local nodes i   # discovery takes a few seconds after up; the planner window is the last to come up
  for i in 1 2 3 4 5 6; do
    nodes=$(timeout 10 ros2 node list 2>/dev/null || true)
    grep -qx /status_loop <<<"$nodes" && grep -qx /cot_bridge <<<"$nodes" && break
    sleep 5
  done
  for n in /fleet_state /geofence_monitor /map_overlay /mission_planner /status_loop /cot_bridge /foxglove_bridge \
           /v1/patrol_server /v2/patrol_server /v3/patrol_server /v1/waypoint_follower /v2/waypoint_follower /v3/waypoint_follower; do
    grep -qx "$n" <<<"$nodes" || fail+=("node $n missing")
  done
  for v in v1 v2 v3; do
    timeout 6 ros2 topic echo --once "/$v/fix" sensor_msgs/msg/NavSatFix 2>/dev/null | grep -q "^latitude:" || fail+=("$v/fix silent")
  done
  local fleet; fleet=$(timeout 6 ros2 topic echo --once /fleet/state corridor_msgs/msg/FleetState 2>/dev/null || true)
  [ -n "$fleet" ] || fail+=("/fleet/state silent")
  for v in v1 v2 v3; do
    grep -q "name: $v" <<<"$fleet" || fail+=("$v not in /fleet/state")
  done
  local unknown; unknown=$(grep -c "mode: unknown" <<<"$fleet" || true)
  [ "${unknown:-0}" -eq 0 ] || fail+=("$unknown vehicle(s) unknown in /fleet/state (stale fix)")
  timeout 8 ros2 topic echo --once /map/overlay foxglove_msgs/msg/GeoJSON 2>/dev/null | grep -q "Polygon" || fail+=("/map/overlay silent")
  ss -ltn | grep -q ":$FOX_PORT " || fail+=("foxglove port $FOX_PORT not listening")
  if [ -f "$LOGDIR/cot.log" ]; then
    local connects; connects=$(grep -c "connected to TAK server" "$LOGDIR/cot.log" || true)
    [ "${connects:-0}" -eq 1 ] || fail+=("cot_bridge connections: ${connects:-0} (want exactly 1)")
    ! grep -q "TAK send failed" "$LOGDIR/cot.log" || fail+=("cot_bridge send failures")
  else fail+=("no cot log"); fi
  if [ "$mode" = mixed ]; then
    local c1 c2
    c1=$(timeout 5 ros2 topic echo --once /clock rosgraph_msgs/msg/Clock --field clock.sec 2>/dev/null | grep -m1 -E '^[0-9]+$' || echo 0)
    sleep 2
    c2=$(timeout 5 ros2 topic echo --once /clock rosgraph_msgs/msg/Clock --field clock.sec 2>/dev/null | grep -m1 -E '^[0-9]+$' || echo 0)
    [ "${c2:-0}" -gt "${c1:-0}" ] || fail+=("/clock not advancing ($c1 -> $c2)")
    timeout 6 ros2 topic echo --once /v1/odom nav_msgs/msg/Odometry 2>/dev/null | grep -q "^header:" || fail+=("v1/odom silent (ros2_control)")
  fi
  grep -q "mission planner:" "$LOGDIR/mission.log" 2>/dev/null || fail+=("planner did not announce itself")
  if [ ${#fail[@]} -eq 0 ]; then echo "DEMO CHECK PASS"; else printf '  - %s\n' "${fail[@]}"; echo "DEMO CHECK FAIL"; exit 1; fi
}

patrol() {
  for v in v1 v2 v3; do
    timeout 10 ros2 action send_goal "/$v/patrol" corridor_msgs/action/Patrol "{vehicle: $v, corridor_id: demo, laps: 0, speed_mps: 0.0, clockwise: false}" 2>&1 | grep -m1 -E "accepted|rejected" | sed "s/^/$v: /" &
  done
  wait
}

down() {
  local pids
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    for w in $(tmux list-windows -t "$SESSION" -F '#W'); do tmux send-keys -t "$SESSION:$w" C-c; done
    sleep 4
  fi
  pids=$(marked)
  if [ -n "$pids" ]; then
    kill -TERM $pids 2>/dev/null || true; sleep 3
    pids=$(marked); [ -n "$pids" ] && kill -KILL $pids 2>/dev/null || true
  fi
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  [ -z "$(marked)" ] && echo "down: nothing of $SESSION left" || { echo "down: still running: $(marked)"; exit 1; }
}

case "$cmd" in
  up) up "$@";; check) check;; patrol) patrol;; down) down;;
  *) sed -n '2,8p' "$0"; exit 2;;
esac
