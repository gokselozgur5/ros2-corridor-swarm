# Architecture

Everything is simulated. The target is a small fleet of ground vehicles that live in a Gazebo world with real physics, patrol a mission corridor, respect a geofence, show up in ATAK, and take missions in plain language through a Claude-backed planner.

```
                 ┌──────────────────────────────────────────────────────────┐
                 │  Mac                                                     │
                 │  Foxglove (ws://thor:8765)   ATAK emulator   terminal    │
                 └───────────▲───────────────────────▲──────────────────────┘
                             │ websocket             │ CoT / TCP 8087
┌────────────────────────────┼───────────────────────┼──────────────────────┐
│ Jetson Thor (ROS 2 Jazzy, source build, ROS_DOMAIN_ID=42)                 │
│                            │                       │                      │
│   foxglove_bridge ─────────┘          cot_bridge ──┘   FreeTAKServer      │
│         ▲                                 ▲             (Docker)          │
│         │ all topics                      │ /fleet/state                  │
│   ┌─────┴─────────────────────────────────┴───────────────────────┐      │
│   │ mission layer                                                 │      │
│   │   mission_planner  (Claude tool use)  ── tools ──┐            │      │
│   │   patrol_server    (action: Patrol)              │            │      │
│   │   geofence_monitor (corridor polygon, GeoJSON)   │            │      │
│   └──────────────────────────────▲───────────────────┼────────────┘      │
│                                  │ /vN/odom /vN/fix  │ /vN/cmd_vel        │
│   ┌──────────────────────────────┴───────────────────▼────────────┐      │
│   │ vehicle layer (per vehicle namespace /v1, /v2, ...)           │      │
│   │   waypoint_follower  (pure pursuit on cmd_vel)                │      │
│   │   ros2_control diff_drive_controller  (wheel motors)          │      │
│   │   sensors: lidar, rgb camera, imu, navsat  (gz plugins)       │      │
│   └──────────────────────────────▲───────────────────────────────┘      │
│                                  │ ros_gz_bridge                         │
│   ┌──────────────────────────────┴───────────────────────────────┐      │
│   │ Gazebo Harmonic (gz-sim), headless, sim time                  │      │
│   │   world: corridor.sdf  (terrain, obstacles, the polygon)      │      │
│   │   model: ugv.urdf.xacro (chassis, 4 wheels, sensor mounts)    │      │
│   └──────────────────────────────────────────────────────────────┘      │
└───────────────────────────────────────────────────────────────────────────┘
```

## Layers

**Simulation.** Gazebo Harmonic runs headless on Thor and owns physics, motors and sensors. Nothing in ROS fakes a sensor; if a topic says lidar, a lidar plugin produced it. `ros_gz_bridge` moves clock, cmd_vel, odometry and sensor topics across. Sim time everywhere (`use_sim_time: true`).

**Vehicle.** One ROS namespace per vehicle. `ros2_control` with `diff_drive_controller` turns `cmd_vel` into wheel joint velocities inside Gazebo, so motors have inertia, slip and limits. `waypoint_follower` is a pure-pursuit controller: it takes a path, publishes `cmd_vel`, reports progress. Nav2 is deliberately not in the first build (it is ~40 source packages on this underlay); it can replace `waypoint_follower` later without touching the mission layer.

**Mission.** `patrol_server` is a ROS action: given a corridor id and a vehicle, it generates the patrol route along the polygon and feeds the follower. `geofence_monitor` watches every vehicle fix against the polygon and raises `/fleet/geofence_events`. `mission_planner` is the Claude node: natural language in, ROS actions out.

**Operator.** Foxglove for the ROS view (map panel on fixes, 3D panel on tf, plots on cmd_vel). ATAK for the tactical view: `cot_bridge` turns `/fleet/state` into Cursor-on-Target events and sends them to FreeTAKServer; ATAK markers come back as waypoints.

## The Claude node

`mission_planner` is a plain Python node using the Anthropic SDK with tool use. Its tools are thin wrappers over ROS:

| tool | ROS side |
|---|---|
| `list_vehicles()` | reads `/fleet/state` |
| `get_vehicle(id)` | fix, odom, battery, current goal |
| `patrol(vehicle, corridor)` | sends the `Patrol` action goal |
| `goto(vehicle, lat, lon)` | sends a single waypoint |
| `stop(vehicle)` | cancels the active goal |
| `describe_scene(vehicle)` | grabs one camera frame, returns it as an image block |
| `geofence_events(since)` | recent breaches |

Rules the node enforces, not the prompt:
- Model `claude-opus-5-5` for planning, `claude-haiku-5-5` for the periodic status summary. Both with `fallbacks: "default"` under beta `server-side-fallback-2026-07-01`.
- `stop_reason` is checked before `content` is read. A `refusal` is logged with its `stop_details.category` and surfaces as a ROS diagnostic, never as a crash.
- Every tool call is logged to `/mission/llm_trace` so Foxglove shows what the planner did and why.
- The planner never publishes `cmd_vel`. It only sends action goals, which the geofence can veto.

## Packages

| package | type | contents |
|---|---|---|
| `corridor_sim` | ament_python | existing `vehicle_sim` (kept as the no-Gazebo fallback), GeoJSON loader, launch files |
| `corridor_description` | ament_cmake | `ugv.urdf.xacro`, meshes, `ros2_control` tags, sensor mounts |
| `corridor_gz` | ament_cmake | `corridor.sdf` world, gz plugins config, bridge yaml, `sim.launch.py` |
| `corridor_nav` | ament_python | `waypoint_follower`, `patrol_server`, `geofence_monitor` |
| `corridor_msgs` | ament_cmake | `Patrol.action`, `FleetState.msg`, `GeofenceEvent.msg` |
| `corridor_cot` | ament_python | `cot_bridge` (pytak) |
| `corridor_planner` | ament_python | `mission_planner` (Anthropic SDK) |

External, cloned into `ws/src` via `ws/deps.repos` and built with colcon: `ros_gz` (branch `jazzy`), `ros2_control`, `ros2_controllers`, `gz_ros2_control`, `foxglove_bridge` (0.8.5, already there). Gazebo Harmonic itself comes from the OSRF apt repo (it is not a ROS package, so it does not shadow the source build).

## Decisions

- **Gazebo, not Isaac Sim.** Isaac needs x86 + RTX; Thor is aarch64. Gazebo Harmonic has arm64 packages for Ubuntu 24.04.
- **Headless.** `gz sim -s` only. The Mac sees the world through Foxglove's 3D panel from tf and sensor topics. GUI on Thor would steal GPU from the people who share it.
- **Physics first, Nav2 later.** The point is motors and sensors behaving like hardware; a pure-pursuit follower is 150 lines and keeps the stack buildable.
- **Claude plans, ROS executes.** The LLM is an operator with a radio, not a controller in the loop. Tick-rate decisions stay in ROS.
- **Keep `vehicle_sim`.** Day 1's kinematic node remains the smoke test when Gazebo is not available.
- **The follower navigates on the GNSS fix and the IMU, not on odometry (#19, 2026-10-25).** Odometry starts wherever the vehicle spawned; the patrol route lives in the corridor frame. `waypoint_follower` converts `/vN/fix` through the same `LocalFrame` as `patrol_server` and `geofence_monitor`, and takes the heading from `/vN/imu` (ENU-referenced; `heading_offset_rad` if a platform's IMU is not). Wheel odometry is not used for navigation at all: on the skid steer its yaw read 119° while the IMU read 7° once the wheels slipped against a crate. The same contract on all three backends (`unicycle_sim` publishes `imu` too), and it is what a real vehicle has. Chosen over a static `map -> odom` tf (correct ROS, more plumbing) and over starting odom at the spawn pose (odom pretending to be map).
- **Skid-steer wheels slide sideways.** With full lateral grip Gazebo's four-wheel vehicle turned at 0.22 rad/s for a 1.0 rad/s command and overshot every corner; `mu2 = 0.15` across the rolling direction gives about 0.75 rad/s. A real skid steer scrubs its tyres the same way; the number is a tuning knob in `ugv.gazebo.xacro`, not physics truth.

## Open questions (2026-10-24, after the first build-out)

Things the first day showed; to be decided together before Phase 4.

1. **Odometry ownership.** Three sources exist: `unicycle_sim`, the gz `OdometryPublisher`, `diff_drive`. They are switched with xacro `if/unless` and launch conditions. Cleaner: one vehicle-layer contract (`/vN/odom`, `/vN/fix`, `/vN/cmd_vel`, `/tf`) and one launch file per backend (kinematic, gz-plugin, ros2_control) with no conditionals inside the model. Decided in part (2026-10-25): the mission layer positions vehicles by `fix`; odom is heading and speed only, so which node publishes it stops mattering for navigation. The launch split is still to do.
2. **Who cancels on a geofence breach.** `geofence_monitor` publishes an empty `Path` to the follower; `patrol_server` notices through the follower status and its own event subscription. Two paths to the same effect. Better: the monitor cancels the active action at the mission layer, and only the mission layer talks to the follower.
3. **`/fleet/state` is in the diagram and nobody publishes it.** `mission_planner` and `status_loop` each subscribe to every vehicle and summarise it themselves. A `fleet_state` node should publish `FleetState` and both should read that.
4. **`goto` publishes a Path.** The planner's rule is "only action goals, never low-level topics"; a Path is low-level. `goto` should be an action like `patrol`.
5. **Nav2 stays out.** The ros_gz and ros2_control source builds (17 vendor packages, DART and OGRE-Next from source, 42 control packages) confirm that the underlay cannot absorb Nav2 cheaply. Pure pursuit is enough for a corridor with few obstacles; obstacle avoidance from the lidar, if wanted, is a small local planner in `corridor_nav`, not Nav2.
