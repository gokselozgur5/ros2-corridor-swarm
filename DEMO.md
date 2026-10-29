# Demo runbook

Ten minutes, live, on Thor (`192.168.88.55`, user `jensen`, repo `~/deletdis/ros2-corridor-swarm`) with the Mac showing Foxglove.
Everything runs on `ROS_DOMAIN_ID=77`, `GZ_PARTITION=corridor`, Foxglove on port 8766; the other stack on the box (domain 42, port 8765)
is not touched. The planner spends real money: about 1.3 to 2.5 cents per request on Opus, the Haiku status loop about one cent per hour.

## Before the audience arrives (15 min)

```bash
ssh jensen@192.168.88.55
cd ~/deletdis/ros2-corridor-swarm && git log --oneline -1          # the commit you rehearsed with
./steps/19_demo.sh down                                              # a clean start, also when nothing seems to run
./steps/19_demo.sh up                                                # prints "sim clock after N s" then "up (mixed)"; 1 to 3 min
./steps/19_demo.sh check                                             # DEMO CHECK PASS, twice in a row
```

`up` without Gazebo (fallback, 30 s): `./steps/19_demo.sh up --kinematic`. Same scenes, no physics, no camera, so skip scene 5.

On the Mac:

1. Foxglove, Open connection, `ws://192.168.88.55:8766`, Layout, Import from file, `foxglove/corridor.json`.
   The map shows the corridor polygon, three inset routes and three vehicles standing on the south edge.
2. A second terminal for the ATAK side: `ssh -N -L 8087:127.0.0.1:8087 jensen@192.168.88.55` (leave it open), then
   `python3 tools/atak_standin.py watch`: one line per vehicle per second with position, course and speed as FreeTAKServer relays them.
   With a real ATAK/WinTAK client: connect it to the same tunnel (TCP 127.0.0.1:8087, no TLS); V1, V2, V3 appear as friendly vehicles.
3. `tmux attach -t corridor-demo` in a third terminal, window `mission`: the planner's replies and the Haiku status line scroll here.

If `check` fails: `down`, `up`, `check` again. If `/clock` is the failing line, `up` already restarted Gazebo once; use `up --kinematic`.

## Scenes

Say what the audience will see before each command; the stack is honest, nothing is pre-recorded.

### 1. The fleet (1 min)

Foxglove map: corridor, routes, vehicles. v1 is a Gazebo vehicle with physics, lidar and camera (3D panel and camera panel);
v2 and v3 are kinematic unicycles. The status line in the `mission` window (Haiku, every 30 s) says the fleet is idle.

### 2. Patrol (2 min)

Either in plain language through the planner (Foxglove publish panel on `/mission/request`, or the Thor shell):

```bash
ros2 topic pub --once /mission/request std_msgs/msg/String "{data: 'send the whole fleet on patrol until I say stop'}"
```

or directly, without the model: `./steps/19_demo.sh patrol`. The three vehicles move along their routes (speed plot, cross-track
plot for v1), the ATAK watcher's speed column goes from 0.00 to 3.00, the status line changes to "all three on patrol".
The planner's reply (10 to 20 s) is in the `mission` window and on the Foxglove reply panel.

### 3. A question (1 min)

```bash
ros2 topic pub --once /mission/request std_msgs/msg/String "{data: 'where is v3 and what is it doing?'}"
```

Reply in about five seconds: position, speed, distance to the corridor edge. The model reads `/fleet/state`, it does not guess.

### 4. ATAK marker becomes a mission (2 min)

From the Mac (tunnel open), or a real ATAK client dropping a waypoint inside the corridor:

```bash
python3 tools/atak_standin.py marker RALLY1 52.7010 -2.9985
```

Within 20 s the `mission` window shows the request the bridge made of the marker
(`[req N] send the nearest vehicle to marker "RALLY1" at 52.701000, -2.998500`) and the planner's reply naming the vehicle it
chose and why ("v2 was the nearest at about 53 m"). That vehicle leaves its patrol route and drives to the marker on the map.
A second marker while the planner is still working is answered with "busy ..." and asked again when ATAK re-sends it.

### 5. The camera (1 min, Gazebo only)

```bash
ros2 topic pub --once /mission/request std_msgs/msg/String "{data: 'what does v1 see right now?'}"
```

The planner pulls one camera frame from v1 and describes the scene (the red crate in the corridor world).

### 6. Geofence (2 min)

A route that leaves the corridor, sent behind the planner's back:

```bash
ros2 run corridor_nav send_path --ros-args -r __ns:=/v3 -p "points:=[150.0, 100.0, 150.0, 260.0]"
```

v3 drives north; at the edge the geofence monitor logs `v3 left demo by 0.0 m`, publishes the EXIT event (Foxglove geofence panel)
and stops the vehicle (follower status `idle`). Then ask the planner:

```bash
ros2 topic pub --once /mission/request std_msgs/msg/String "{data: 'what happened to v3? bring it back on patrol'}"
```

It reads the geofence events, explains, and sends v3 back on patrol.

### 7. Close (1 min)

The status line summarises the fleet; the `mission` window's `cost:` line shows what the hour cost. `./steps/19_demo.sh down`.

## Fallbacks

| if | then |
|---|---|
| the planner says `the model declined` or `planner error` | repeat the request once; otherwise `./steps/19_demo.sh patrol` and `ros2 action send_goal` show the same behaviour without the model |
| no reply within 30 s | the planner is busy with the previous request (one at a time by design); wait for its reply, then repeat |
| Foxglove shows no map | the bridge is on 8766, not 8765; `ss -ltn \| grep 8766` on Thor, and `check` names the port |
| the ATAK watcher shows `nan` positions | the server relays a GeoChat line first; vehicle lines follow within a second |
| `check` says `/clock not advancing` | the ros_gz bridge missed the clock (discovery race under load): `down`, `up`; second time `up --kinematic` |
| Gazebo is slow (RTF under 0.5) | the other stack on the box is using the GPU; `up --kinematic` |
| the marker does not become a request | the stand-in holds its connection five seconds on purpose; from a real client, drop the marker again a few metres away (`marker_move_m` 5 m dedups re-sends at the same spot) |

## Known limits

- `pub --once` publishes to the first matching subscriber: with a `ros2 topic echo /mission/request` open in another shell the planner can miss the message. Use the Foxglove panel or no extra echo.
- Mixed clocks: v1 runs on sim time, v2 and v3 on wall time. Freshness in `fleet_state` and `cot_bridge` is measured at receipt on each node's own clock, so the mix is harmless; `/fleet/state` stamps are wall time.
- The patrol inset is 30 m + 8 m per vehicle number (v1 30, v2 38, v3 46), the same in every launch, so the overlay routes are the routes the servers drive.
