# ros2-corridor-swarm

A ROS 2 Jazzy project on Jetson Thor, fully simulated: a fleet of ground vehicles with real physics in a Gazebo world, patrolling a mission corridor, reporting to ATAK, taking missions in plain language through a Claude-backed planner.

Architecture: [ARCHITECTURE.md](ARCHITECTURE.md). Plan with a proof per step: [PLAN.md](PLAN.md). Progress: GitHub milestones and issues.

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
