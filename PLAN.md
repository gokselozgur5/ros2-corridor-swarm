# Plan

Phases, not days. Each step has a proof: a command whose output decides whether the step is done. A step is not done because the code exists; it is done when the proof passes on Thor from a clean `colcon build`.

Tracked on GitHub as milestones (one per phase) and issues (one per step).

## Progress

2026-10-24: Phase 1 (#1-#5) and Phase 2 (#6-#10) proven, see the closed issues for the proof outputs. Phase 3 is written; #12 and #13 are proven on the debug path (`steps/08_proofs.sh 12`, `steps/10_scene_proof.sh`), the model path runs with `steps/11_live_model_proof.sh` once `ANTHROPIC_API_KEY` is in `.env`. Found on the way: #19, the Gazebo backend's odom frame is not the map frame the patrol route uses (`steps/12_gazebo_patrol.sh` shows it); decided in the architecture review (ARCHITECTURE.md, open questions).

## Phase 0: kinematic baseline (done, 2026-10-24)

- Workspace, `env.sh`, `foxglove_bridge` 0.8.5 from source, `vehicle_sim` node.
- Proof: `ros2 topic hz /vehicle/fix` ≈ 10 Hz, Foxglove connects from the Mac.

## Phase 1: Gazebo world and a physical vehicle

1. **Gazebo Harmonic on Thor.** OSRF apt repo, `gz-harmonic`, headless only.
   Proof: `gz sim -s -r --iterations 100 empty.sdf` exits 0.
2. **`ros_gz` from source** (branch `jazzy`) in `ws/src`, built against the underlay.
   Proof: `ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` publishes `/clock` from a running sim.
3. **`corridor_description`**: `ugv.urdf.xacro`, differential 4-wheel chassis, lidar, camera, imu, navsat mounts.
   Proof: `check_urdf` passes; `robot_state_publisher` + Foxglove 3D panel shows the model.
4. **`corridor_gz`**: `corridor.sdf` world with the corridor polygon drawn as ground markers and a few obstacles; spawn one vehicle; bridge yaml for clock, odom, tf, cmd_vel, scan, image, imu, navsat.
   Proof: `ros2 topic hz /v1/scan /v1/imu /v1/fix /v1/camera/image` all > 0 with sim time.
5. **`ros2_control` + `gz_ros2_control` + `diff_drive_controller`** from source.
   Proof: `ros2 topic pub /v1/cmd_vel ... linear.x: 1.0` for 5 s moves `/v1/odom` by ≈ 5 m, and the wheel joints report velocity in `/v1/joint_states`.

Exit: one vehicle drives in a physics world under `cmd_vel`, every sensor topic is live, Foxglove shows the 3D scene.

## Phase 2: autonomy

6. **`corridor_msgs`**: `Patrol.action`, `FleetState.msg`, `GeofenceEvent.msg`.
   Proof: `ros2 interface show corridor_msgs/action/Patrol`.
7. **`waypoint_follower`** (pure pursuit): path in, `cmd_vel` out, arrival tolerance parameter.
   Proof: a 3-waypoint square, cross-track error < 1 m logged over the run; a test that feeds odom and asserts the heading command sign.
8. **GeoJSON corridor loader** (reuse map-writer output) and **`geofence_monitor`**.
   Proof: a vehicle commanded outside the polygon produces a `GeofenceEvent` within 1 s and the follower gets a cancel. The test is run with the monitor disabled first and must then fail.
9. **`patrol_server`** action: generates the lap route along the polygon, drives the follower, reports feedback (lap, progress).
   Proof: `ros2 action send_goal /v1/patrol ...` completes a lap; feedback is visible in Foxglove.
10. **Multi-vehicle**: namespaces `/v1 /v2 /v3`, launch argument `n`.
    Proof: three vehicles patrol simultaneously, `ros2 topic hz /clock` stays ≥ 50 Hz (real-time factor ≥ 0.5).

Exit: a fleet patrols the corridor and cannot leave it.

## Phase 3: Claude mission planner

11. **`corridor_planner` skeleton**: Anthropic SDK, tool definitions, `stop_reason` handling, `fallbacks: "default"`, trace topic.
    Proof: a unit test with a stubbed client asserting that a `refusal` response yields a diagnostic and no exception, and that the planner never publishes `cmd_vel`.
12. **Tools wired to ROS**: `list_vehicles`, `get_vehicle`, `patrol`, `goto`, `stop`, `geofence_events`.
    Proof: `ros2 topic pub /mission/request std_msgs/String "data: 'send v2 to patrol the corridor'"` results in an accepted `Patrol` goal; the trace shows the tool call.
13. **`describe_scene`**: one camera frame as an image block.
    Proof: with a red box placed in front of v1 in the world, "what does v1 see" names a red object.
14. **Status loop on Haiku**: every 30 s a one-paragraph fleet summary on `/mission/summary`.
    Proof: cost per hour measured from `usage` and written to the README (target < $0.20/h).

Exit: a sentence typed on the Mac becomes a patrol in Gazebo, and the trace explains it.

## Phase 4: ATAK

15. **FreeTAKServer** in Docker on Thor (our own container, named, never touching others).
    Proof: ATAK in the emulator connects.
16. **`cot_bridge`**: `/fleet/state` → CoT PLI events; ATAK marker → `/mission/request`.
    Proof: three vehicles visible in ATAK moving; dropping a marker sends a vehicle there.

## Phase 5: polish

17. Lifecycle nodes for the mission layer, QoS review, launch arguments documented.
18. Demo video, README rewrite, repo public.

## Order of work

Phase 1 is the long pole: four source packages to build on an unusual underlay. Start it first and let it build while writing Phase 2 code, which is testable against `vehicle_sim` without Gazebo. Phase 3 depends only on the `corridor_msgs` interfaces and can be developed against the kinematic sim too; Gazebo is needed only for step 13.

## Budget

The API credit is $200. Planning calls on Opus 5.5 run at maybe 5k input / 1k output tokens each, about $0.04. The Haiku status loop at 2 calls a minute is under $0.05 an hour. A full day of development and demos is a few dollars; the credit is not the constraint, Phase 1 build time is.
