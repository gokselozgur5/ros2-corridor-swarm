# ros2-corridor-swarm

A ROS 2 Jazzy project on Jetson Thor, fully simulated: a fleet of ground vehicles with real physics in a Gazebo world, patrolling a mission corridor, reporting to ATAK, taking missions in plain language through a Claude-backed planner.

Architecture: [ARCHITECTURE.md](ARCHITECTURE.md). Decisions: [docs/adr](docs/adr/). Requirements (EARS): [docs/requirements.md](docs/requirements.md). Plan with a proof per step: [PLAN.md](PLAN.md). Progress: GitHub milestones and issues.

Vehicles patrol a mission corridor (the KML/GeoJSON polygon from map-writer), stay inside a geofence, and report to ATAK over CoT. Foxglove on the Mac shows the ROS side.

## Environment

`. ./env.sh` sources ROS 2 Jazzy (the source build at `~/.goske/ros2_jazzy` if present, else apt's `/opt/ros/jazzy`), then the overlay `ws/install` once built, defaults `ROS_DOMAIN_ID` to 42 and exports `./.env` if there is one. A source Jazzy has no `/opt/ros`, so everything the stack needs beyond it comes from `ws/deps.repos`, built into the overlay by the steps below. 06, 07, 09 and 15 `cd` to `~/deletdis/ros2-corridor-swarm` (the clone on Thor), source `env.sh` and repeat the import themselves; logs go to `~/deletdis/log/`.

```bash
. ./env.sh
vcs import ws/src < ws/deps.repos   # 44 repos: foxglove_bridge 0.8.5, rosx_introspection, foxglove-sdk (3); ros_gz with 17 gz vendors and 5 message repos (23); xacro (1); ros2_control, ros2_controllers, gz_ros2_control and 14 they pull in (17)
./steps/05_gz_install.sh            # Gazebo Harmonic from the OSRF apt repo; not a ROS package, nothing lands in /opt/ros
./steps/06_ros_gz_build.sh          # ros_gz_bridge and ros_gz_sim with the gz vendors on top of Harmonic (GZ_VERSION=harmonic); apt libcli11-dev libgflags-dev libdart6.13-all-dev
./steps/09_ros2_control_build.sh    # gz_ros2_control, diff_drive_controller, joint_state_broadcaster; the other controllers get COLCON_IGNORE, tl-expected is built first; apt python3-typeguard python3-jinja2
./steps/15_foxglove_msgs.sh         # before 07: COLCON_IGNORE on the sdk checkout (its ros/src holds a second foxglove_bridge, and colcon stops on a duplicate name once a build selects it, as 07 does through corridor_sim), then foxglove_msgs out of ros/src/foxglove_msgs alone
./steps/07_build_ours.sh            # xacro, foxglove_bridge, corridor_msgs and all our packages; pip anthropic brotli zstandard for the user
```

On a Thor with Jazzy from apt (`/opt/ros/jazzy`), ros_gz, gz_ros2_control, the controllers and foxglove_bridge come from apt as well (`ros-jazzy-*`; `foxglove_msgs` is `ros-jazzy-foxglove-msgs`); nothing is imported from `ws/deps.repos` and the seven `corridor_*` packages build alone: `cd ws && colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF`.

Run: `ros2 launch corridor_nav kinematic.launch.py n:=3` (unicycles, no Gazebo), then a lap: `ros2 action send_goal /v1/patrol corridor_msgs/action/Patrol "{vehicle: v1, corridor_id: demo, laps: 1, speed_mps: 0.0, clockwise: false}"`. Physics instead: `ros2 launch corridor_gz sim.launch.py use_ros2_control:=true` (`steps/12_gazebo_patrol.sh` drives the lap there). Mission layer on top of the kinematic launch: `ros2 launch corridor_planner mission.launch.py start_patrol:=false` (`dry_run:=true` without `ANTHROPIC_API_KEY`). Mac: Foxglove, Open connection, `ws://$THOR_HOST:8765`, Layout, Import from file, `foxglove/corridor.json`.

`env.sh` keeps a `ROS_DOMAIN_ID` already set, so `export ROS_DOMAIN_ID=77` before sourcing moves everything off the demo domain 42; the node tests refuse 42. Thor is shared with another ROS 2 and Gazebo stack on 42, so ours runs there on 77 with `GZ_PARTITION=corridor` exported the same way.

Why these choices:

- `ros-foxglove-bridge` main is ROS 1 only now; 0.8.5 is the newest tag with `ros2_foxglove_bridge`. The ROS 2 bridge moved into foxglove-sdk (`ros/src/foxglove_bridge`): it needed a Rust toolchain when 0.8.5 was picked (ADR-0003), its CMake now fetches a prebuilt C++ SDK, and the pin has not been revisited. Only `foxglove_msgs` is built out of the sdk checkout.
- `BUILD_TESTING=OFF` on every build: the bridge's smoke test is compiled as C++20 and websocketpp 0.8.2 does not parse under it.
- No `ros-jazzy-*` apt package next to the source Jazzy: it would create `/opt/ros` and shadow the build (ADR-0002), which is why all 44 repos are built in the overlay. rosdep wanted `ros-jazzy-rosx-introspection` for the bridge on day 1; 06 still prints `rosdep install --simulate` for ros_gz, and it must name none.
- apt packages the builds need: `libwebsocketpp-dev nlohmann-json3-dev rapidjson-dev` for the bridge (rosdep resolved them on day 1, steps 02b and 02c; no later step installs them) and the ones next to 05, 06 and 09 above. 07 pip-installs `anthropic brotli zstandard` because noble's `python3-brotli` broke the SDK's response decoding.

`steps/` holds the scripts each day was done with; they run inside the `ros2` tmux session (`tmux attach -t ros2`, windows `work` and `run`).

## Status

Phase 1 closed: Gazebo world, live lidar/camera/imu/navsat, ros2_control diff_drive moving the UGV (#1-#5). Phase 2 closed on the kinematic sim: follower, geofence, patrol action, three vehicles (#6-#10). Phase 3: planner tools proven over the debug topic; the model path waits for ANTHROPIC_API_KEY in `.env` (#11-#14 open). Phase 4 groundwork: corridor_cot written and unit-tested, FreeTAKServer container being set up on Thor (#15 #16 open). See PLAN.md and the issues.

## Local settings

Hosts, users and keys are not in the repo. Copy `.env.example` to `.env` and fill it; `env.sh` exports it. `.env` is gitignored.

## Launch files and arguments

| launch | package | arguments | what it starts |
|---|---|---|---|
| `kinematic.launch.py` | corridor_nav | `n` (vehicles, 1), `first` (1), `corridor` (GeoJSON path, also the follower's frame origin), `lookahead_m` (3.0), `speed_mps` (3.0), `foxglove` (true) | unicycle_sim, waypoint_follower, patrol_server per vehicle; one geofence_monitor, one fleet_state (`/fleet/state`) |
| `sim.launch.py` | corridor_gz | `world` (corridor.sdf), `use_ros2_control` (false) | headless gz sim, v1 spawned from the xacro, ros_gz bridge, robot_state_publisher; with ros2_control: joint_state_broadcaster, diff_drive, twist_stamper |
| `display.launch.py` | corridor_description | none | robot_state_publisher on the UGV xacro |
| `mission.launch.py` | corridor_planner | `corridor`, `dry_run` (false), `status_loop` (false), `start_patrol` (true), `fleet_state` (true; false when the vehicle layer already runs one) | patrol_server and fleet_state for v1, mission_planner (Opus), optional status_loop (Haiku) |

Tools: `send_path` (one Path from `points:=[x,y,...]`), `clicked_goal` (`vehicle`, `point_topic`, `corridor_geojson` required: the click becomes a goto from the vehicle's fix), `cot_bridge` (`vehicles`, `host`, `port` 8087, `period_s`, `fix_timeout_s` 5.0: a vehicle whose fix is older is not reported, `marker_move_m` 5.0: a re-sent marker becomes a new request only once it moved this far, `team`). Proof scripts in `steps/`: `08_proofs.sh <issue>`, `10_scene_proof.sh`, `11_live_model_proof.sh` (needs the key), `12_gazebo_patrol.sh`, `13_cot_proof.sh`, `14_marker_proof.sh`, `16_fleet_state_proof.sh`, `17_tests.sh` (the unit and node tests from the built workspace, on `ROS_DOMAIN_ID=77`).

## Foxglove

Both launch files start `foxglove_bridge` on port 8765 (`foxglove:=false` to skip). On the Mac: Foxglove, Open connection, `ws://$THOR_HOST:8765`, then Layout, Import from file, `foxglove/corridor.json`: fleet map on the fixes, v1 3D view with lidar and camera, cross-track and speed plot, follower status, geofence events, a publish panel for `/mission/request` with the planner's reply and trace next to it.
