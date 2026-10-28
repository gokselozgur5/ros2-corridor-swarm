# ros2-corridor-swarm

A ROS 2 Jazzy project on Jetson Thor, fully simulated: a fleet of ground vehicles with real physics in a Gazebo world, patrolling a mission corridor, reporting to ATAK, taking missions in plain language through a Claude-backed planner.

Architecture: [ARCHITECTURE.md](ARCHITECTURE.md). Decisions: [docs/adr](docs/adr/). Requirements (EARS): [docs/requirements.md](docs/requirements.md). Plan with a proof per step: [PLAN.md](PLAN.md). Progress: GitHub milestones and issues.

Vehicles patrol a mission corridor (the KML/GeoJSON polygon from map-writer), stay inside a geofence, and report to ATAK over CoT. Foxglove on the Mac shows the ROS side.

## Environment

ROS 2 Jazzy is a source build at `~/.goske/ros2_jazzy` on Thor. `. ./env.sh` sources it, the workspace overlay and sets `ROS_DOMAIN_ID=42`.

```bash
vcs import ws/src < ws/deps.repos          # foxglove_bridge 0.8.5, rosx_introspection
rosdep install --from-paths ws/src --ignore-src -y --simulate   # must list no ros-jazzy-* package
cd ws && colcon build --symlink-install --packages-up-to foxglove_bridge corridor_sim --cmake-args -DBUILD_TESTING=OFF
```

Run: `ros2 launch corridor_sim day1.launch.py`. Mac: Foxglove app, Open connection, `ws://$THOR_HOST:8765`; a Map panel on `/vehicle/fix`.

Why these choices:

- `ros-foxglove-bridge` main is ROS 1 only now; 0.8.5 is the newest tag with `ros2_foxglove_bridge`. The ROS 2 bridge in foxglove-sdk needs a Rust toolchain.
- `BUILD_TESTING=OFF`: the bridge's smoke test is compiled as C++20 and websocketpp 0.8.2 does not parse under it.
- `rosx_introspection` is cloned because rosdep would otherwise install `ros-jazzy-rosx-introspection` from apt, which would shadow the source build with `/opt/ros`.
- apt packages the build needed: `libwebsocketpp-dev nlohmann-json3-dev rapidjson-dev`.

`steps/` holds the scripts each day was done with; they run inside the `ros2` tmux session (`tmux attach -t ros2`, windows `work` and `run`).

## Status

Phase 1 closed: Gazebo world, live lidar/camera/imu/navsat, ros2_control diff_drive moving the UGV (#1-#5). Phase 2 closed on the kinematic sim: follower, geofence, patrol action, three vehicles (#6-#10). Phase 3: planner tools proven over the debug topic; the model path waits for ANTHROPIC_API_KEY in `.env` (#11-#14 open). Phase 4 groundwork: corridor_cot written and unit-tested, FreeTAKServer container being set up on Thor (#15 #16 open). See PLAN.md and the issues.

## Local settings

Hosts, users and keys are not in the repo. Copy `.env.example` to `.env`, fill it, and `. ./.env` before `env.sh`. `.env` is gitignored.

## Launch files and arguments

| launch | package | arguments | what it starts |
|---|---|---|---|
| `kinematic.launch.py` | corridor_nav | `n` (vehicles, 1), `first` (1), `corridor` (GeoJSON path, also the follower's frame origin), `lookahead_m` (3.0), `speed_mps` (3.0), `foxglove` (true) | unicycle_sim, waypoint_follower, patrol_server per vehicle; one geofence_monitor, one fleet_state (`/fleet/state`) |
| `sim.launch.py` | corridor_gz | `world` (corridor.sdf), `use_ros2_control` (false) | headless gz sim, v1 spawned from the xacro, ros_gz bridge, robot_state_publisher; with ros2_control: joint_state_broadcaster, diff_drive, twist_stamper |
| `display.launch.py` | corridor_description | none | robot_state_publisher on the UGV xacro |
| `mission.launch.py` | corridor_planner | `corridor`, `dry_run` (false), `status_loop` (false), `start_patrol` (true), `fleet_state` (true; false when the vehicle layer already runs one) | patrol_server and fleet_state for v1, mission_planner (Opus), optional status_loop (Haiku) |

Tools: `send_path` (one Path from `points:=[x,y,...]`), `clicked_goal` (`vehicle`, `point_topic`, `corridor_geojson` required: the click becomes a goto from the vehicle's fix), `cot_bridge` (`vehicles`, `host`, `port` 8087, `period_s`, `fix_timeout_s` 5.0: a vehicle whose fix is older is not reported, `marker_move_m` 5.0: a re-sent marker becomes a new request only once it moved this far, `team`). Proof scripts in `steps/`: `08_proofs.sh <issue>`, `10_scene_proof.sh`, `11_live_model_proof.sh` (needs the key), `12_gazebo_patrol.sh`, `13_cot_proof.sh`, `14_marker_proof.sh`, `16_fleet_state_proof.sh`.

## Foxglove

Both launch files start `foxglove_bridge` on port 8765 (`foxglove:=false` to skip). On the Mac: Foxglove, Open connection, `ws://$THOR_HOST:8765`, then Layout, Import from file, `foxglove/corridor.json`: fleet map on the fixes, v1 3D view with lidar and camera, cross-track and speed plot, follower status, geofence events, a publish panel for `/mission/request` with the planner's reply and trace next to it.
