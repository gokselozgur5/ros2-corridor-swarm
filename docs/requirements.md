# Requirements

EARS-style requirements derived from the code, the launch files and PLAN.md. Each line names what verifies it; `NOT VERIFIED` means nothing checks it yet, `[GAP]` means the code does not satisfy it, `[PLANNED]` means PLAN.md promises it. Decisions behind them: [docs/adr](adr/).

## simulation

Scope: PLAN.md Phase 1 (#1-#5): Gazebo Harmonic world, UGV model and sensors, ros_gz bridge, sim time, ros2_control diff_drive; plus the kinematic fallback in `corridor_sim`. Read from `ws/src/corridor_gz`, `ws/src/corridor_description`, `ws/src/corridor_sim`, `steps/*`.

REQ-SIMULATION-01 (Ubiquitous): The simulation shall run Gazebo Harmonic headless (`gz sim -s -r`) with the `corridor` world from `corridor_gz/worlds/corridor.sdf`, overridable by launch argument `world`. (`sim.launch.py:28,34`; `CLAUDE.md:9`; `ARCHITECTURE.md:86`)
Verified by: `steps/05_gz_install.sh:15` (headless gz exits 0, PLAN #1 proof); `steps/08_proofs.sh:117-130` proof #4 (`sim.launch.py` starts, topics live).

REQ-SIMULATION-02 (Ubiquitous): The `corridor` world shall be a 200 x 200 m box whose SW corner is the world origin, with WGS84/ENU spherical coordinates at 52.7000 N, 3.0000 W, elevation 100 m, heading 0, drawn as four ground-marker box visuals in one static model from (0,0) to (202,200). (`corridor.sdf:2,14-21,40-46`)
Verified by: NOT VERIFIED (no test reads the SDF; `08_proofs.sh:127` prints fix latitude without asserting it).

REQ-SIMULATION-03 (Ubiquitous): The `corridor` world shall load the Physics, UserCommands, SceneBroadcaster, Sensors (ogre2), Imu and NavSat systems, with a 4 ms max step and real-time factor 1.0. (`corridor.sdf:6-12`)
Verified by: `steps/08_proofs.sh` proof #4 indirectly (scan/imu/fix/image rates > 0 need Sensors/Imu/NavSat systems). Physics step and RTF: NOT VERIFIED.

REQ-SIMULATION-04 (Ubiquitous): The world shall contain a static red crate (`crate_front`) at (34,14), 14 m ahead and 6 m right of the v1 spawn, inside the camera's 80 deg view and off the default patrol leg (y = 21.1), plus `crate_1` (60,80), `crate_2` (140,120) and a `barrel` (100,40). (`corridor.sdf:48-62`; commits 6d30a79, 9a3afe6)
Verified by: `steps/10_scene_proof.sh:16-20` (describe_scene returns an image block; naming the red box needs the model, `steps/11_live_model_proof.sh:21-25`).

REQ-SIMULATION-05 (Ubiquitous): The UGV description shall be a 4-wheel differential chassis (body 1.20 x 0.70 x 0.35 m, 60 kg; wheel r 0.20 m, 4 kg) with continuous wheel joints and fixed lidar, camera, imu and gnss mount links, all link/joint names taking a `prefix` argument, frames per REP-103. (`ugv.urdf.xacro:3-20,45-87`)
Verified by: `steps/08_proofs.sh:14-16` proof #3 (`xacro sim:=true use_ros2_control:=true | check_urdf`).

REQ-SIMULATION-06 (Optional): Where the xacro is rendered with `sim:=true`, the model shall carry Gazebo sensors on the mounts: gpu_lidar on `${ns}/scan` (10 Hz, 720 samples, +-pi, 0.3-40 m, gaussian noise stddev 0.01), camera on `${ns}/camera/image` (10 Hz, 640x480 R8G8B8, hfov 1.396, camera_info on `${ns}/camera/camera_info`), imu on `${ns}/imu` (100 Hz, ENU orientation reference), navsat on `${ns}/navsat` (5 Hz). (`ugv.urdf.xacro:89-92`; `ugv.gazebo.xacro:20-64`; commit e1dfeca)
Verified by: `steps/08_proofs.sh:117-130` proof #4 (`ros2 topic hz` on /v1/scan /v1/imu /v1/fix /v1/camera/image /v1/odom > 0). Rates, resolution and noise values themselves: NOT VERIFIED.

REQ-SIMULATION-07 (Ubiquitous): The simulation shall spawn one vehicle `v1` into world `corridor` at (20, 20, 0.5) from `/v1/robot_description`, rendered with `sim:=true ns:=v1 prefix:=v1/`, and run `robot_state_publisher` in namespace `v1` with `use_sim_time: true` and an empty `frame_prefix`. (`sim.launch.py:22-25,35-39`)
Verified by: `steps/08_proofs.sh` proof #4 (topics appear only if the spawn succeeded). Spawn pose: NOT VERIFIED.

REQ-SIMULATION-08 (Ubiquitous): The `parameter_bridge` shall bridge GZ_TO_ROS `/clock` (rosgraph_msgs/Clock), `/v1/scan`, `/v1/camera/image`, `/v1/camera/camera_info`, `/v1/imu`, gz `/v1/navsat` -> ROS `/v1/fix` (NavSatFix), `/v1/odom`, `/v1/tf` (Pose_V -> TFMessage), `/v1/joint_states` (Model -> JointState), and ROS_TO_GZ `/v1/cmd_vel` (Twist). (`bridge_v1.yaml:2-51`; `sim.launch.py:40-41`)
Verified by: `steps/08_proofs.sh` proof #4 (scan, imu, fix, image, odom), proof #5 (joint_states, `08_proofs.sh:110`). `/clock`, `/v1/tf`, `/v1/camera/camera_info`: NOT VERIFIED (PLAN #2 proof at `PLAN.md:23` is manual).

REQ-SIMULATION-09 (Ubiquitous): The `foxglove_bridge` (when launch arg `foxglove` is true), `robot_state_publisher`, `parameter_bridge` and `twist_stamper` nodes started by `sim.launch.py`, and both controllers, shall run with `use_sim_time: true`. The two `controller_manager` spawner nodes are launched without it. (`sim.launch.py:30-33,36,41,43-47,49`; `controllers_v1.yaml:4,12`; `ARCHITECTURE.md:41`)
Verified by: NOT VERIFIED (proof #4 checks topic rates, not that stamps follow `/clock`).

REQ-SIMULATION-10 (Optional): Where `use_ros2_control:=true`, the model shall load `gz_ros2_control::GazeboSimROS2ControlPlugin` under namespace `/v1` with `controllers_v1.yaml` and hardware plugin `gz_ros2_control/GazeboSimSystem`, expose a velocity command interface (-30..30) and position/velocity state interfaces on all four wheel joints, and the launch shall spawn `joint_state_broadcaster` and `diff_drive` (2 wheels per side, separation 0.80 m, radius 0.20 m, 100 Hz manager, 50 Hz odom, cmd_vel timeout 0.5 s, linear 4.0/-2.0 m/s, angular +-2.0 rad/s) against `/v1/controller_manager` with a 60 s timeout, remapping `/v1/diff_drive/odom` to `/v1/odom` via `--controller-ros-args`. (`ugv.urdf.xacro:94-109`; `ugv.gazebo.xacro:65-72`; `controllers_v1.yaml`; `sim.launch.py:43-47`)
Verified by: `steps/08_proofs.sh:101-115` proof #5 (1.0 m/s for 5 s moves odom x by 3.5-6.5 m, joint_states carries wheel velocities); `steps/12_gazebo_patrol.sh:12-23` (one lap > 380 m on diff_drive).

REQ-SIMULATION-11 (State-driven): While `use_ros2_control` is true, the `twist_stamper` node in namespace `v1` shall convert geometry_msgs/Twist on `/v1/cmd_vel` into TwistStamped on `/v1/diff_drive/cmd_vel` stamped with the node clock and `frame_id: v1/base_footprint`, so the follower keeps publishing plain Twist on both backends. (`twist_stamper.py:1-23`; `sim.launch.py:48-49`; commit 5d65cf7)
Verified by: `steps/08_proofs.sh:107` proof #5 (publishes plain Twist on `/v1/cmd_vel`, vehicle moves). No unit test for `twist_stamper` (`ws/src/corridor_nav/test/` has none).

REQ-SIMULATION-12 (State-driven): While `use_ros2_control` is false, the model shall publish odometry and `v1/odom -> v1/base_footprint` tf from the gz `OdometryPublisher` system at 50 Hz on gz `/v1/odom` and `/v1/tf`; while it is true, `diff_drive` shall own odom and tf (`enable_odom_tf: true`, `odom_frame_id: v1/odom`, `base_frame_id: v1/base_footprint`). (`ugv.gazebo.xacro:73-83`; `controllers_v1.yaml:18-20`; commit 2fac34b)
Verified by: `steps/08_proofs.sh` proof #4 (`/v1/odom` rate, gz-plugin path) and proof #5 (odom moves, diff_drive path). The tf side: NOT VERIFIED.

REQ-SIMULATION-13 (Unwanted behaviour) [GAP]: If `use_ros2_control` is false, then the vehicle shall still move under `/v1/cmd_vel`. Today the bridge forwards cmd_vel to gz (`bridge_v1.yaml:47-51`) but `ugv.gazebo.xacro` loads no gz DiffDrive system in the `unless` branch (`ugv.gazebo.xacro:73-83`; no `DiffDrive` system anywhere in `corridor_description` or `corridor_gz`), so the default backend has sensors and odom but no motor; every driving proof uses `use_ros2_control:=true` (`08_proofs.sh:104`, `11_live_model_proof.sh:12`, `12_gazebo_patrol.sh:12`). `ARCHITECTURE.md:98` flags the backend split as unresolved.
Verified by: NOT VERIFIED.

REQ-SIMULATION-14 (Ubiquitous): The gz wheels shall use anisotropic friction (mu 1.0 along `fdir1 = 1 0 0`, mu2 0.15 across) so the skid steer turns at roughly 0.75 rad/s for a 1.0 rad/s command instead of 0.22 rad/s; the ground plane uses mu 0.9. (`ugv.gazebo.xacro:6-19`; `corridor.sdf:33-34`; `ARCHITECTURE.md:92`; commit 5d6bf53)
Verified by: `steps/12_gazebo_patrol.sh:21-23` (lap completes with cross-track printed, not asserted). Turn rate itself: NOT VERIFIED.

REQ-SIMULATION-15 (Ubiquitous): The `JointStatePublisher` system shall publish joint states on gz `${ns}/joint_states` on both backends. (`ugv.gazebo.xacro:84-88`)
Verified by: `steps/08_proofs.sh:110` proof #5 (ros2_control path only).

REQ-SIMULATION-16 (Ubiquitous): `corridor_sim/unicycle_sim` shall integrate a kinematic unicycle from Twist on `cmd_vel` at `rate_hz` (50) and publish `odom`, `fix` (origin 52.70 N, -3.00 E, spherical approximation), `imu` (orientation and yaw rate only) in its namespace, and a `map -> <child_frame_id>` tf (`kinematic.launch.py` sets `child_frame_id` to `vN/base_link`), as the no-Gazebo backend used by `kinematic.launch.py`. (`unicycle_sim.py:18-103`; `kinematic.launch.py:24-26`; `ARCHITECTURE.md:90`; commit a171df1)
Verified by: `steps/08_proofs.sh:65-80` proof #10 (odom rate and speed > 1 m/s on v1..v3); proofs #7, #8, #9 run on it but assert follower/geofence/patrol status only. No unit test for `unicycle_sim` (`corridor_sim` has no `test/`).

REQ-SIMULATION-17 (Unwanted behaviour): If no `cmd_vel` has arrived within `cmd_timeout_s` (0.5 s), then `unicycle_sim` shall set linear and angular velocity to zero; on the Gazebo backend `diff_drive` applies the same 0.5 s `cmd_vel_timeout`. (`unicycle_sim.py:27,56-58`; `controllers_v1.yaml:22`)
Verified by: NOT VERIFIED.

REQ-SIMULATION-18 (Ubiquitous): `corridor_sim/vehicle_sim` shall move one vehicle along a looping lat/lon route at constant `speed_mps` (default 20.0), publishing `/<name>/fix`, `/<name>/odom` and `map -> base_link` tf at `rate_hz` (default 10.0); it is kept as the Phase 0 smoke test (`day1.launch.py`, name `vehicle`). (`vehicle_sim.py:18-112`; `day1.launch.py:12-16`; `PLAN.md:13-16`; `ARCHITECTURE.md:89`)
Verified by: `PLAN.md:16` manual proof (`ros2 topic hz /vehicle/fix` ~ 10 Hz); no script or test.

REQ-SIMULATION-19 (Ubiquitous) [GAP]: With three vehicles patrolling, `/clock` shall stay >= 50 Hz (real-time factor >= 0.5). (`PLAN.md:44`) `PLAN.md:9` and `README.md:32` mark #10 proven, but on the kinematic sim: only `v1` is spawned in Gazebo (`sim.launch.py:37-39`) and proof #10 runs three kinematic vehicles and does not read `/clock` (`08_proofs.sh:65-80`).
Verified by: NOT VERIFIED.

REQ-SIMULATION-20 (Ubiquitous) [PLANNED]: The simulation shall offer one launch file per backend (kinematic, gz-plugin, ros2_control) with no conditionals inside the model. (`ARCHITECTURE.md:98`, "The launch split is still to do")
Verified by: NOT VERIFIED.

Sources: `ws/src/corridor_gz/launch/sim.launch.py`; `ws/src/corridor_gz/worlds/corridor.sdf`; `ws/src/corridor_gz/config/bridge_v1.yaml`; `ws/src/corridor_gz/config/controllers_v1.yaml`; `ws/src/corridor_description/urdf/ugv.urdf.xacro`; `ws/src/corridor_description/urdf/ugv.gazebo.xacro`; `ws/src/corridor_sim/corridor_sim/unicycle_sim.py`; `ws/src/corridor_sim/corridor_sim/vehicle_sim.py`; `ws/src/corridor_sim/launch/day1.launch.py`; `ws/src/corridor_nav/launch/kinematic.launch.py`; `ws/src/corridor_nav/corridor_nav/twist_stamper.py`; `steps/05_gz_install.sh`; `steps/08_proofs.sh`; `steps/10_scene_proof.sh`; `steps/11_live_model_proof.sh`; `steps/12_gazebo_patrol.sh`; `PLAN.md:9,13-31,44`; `ARCHITECTURE.md:41,86,89-92,98`; `README.md:32,42-43`; `CLAUDE.md:9`; commits 6d30a79, 9a3afe6, e1dfeca, 5d65cf7, 2fac34b, 5d6bf53, a171df1.

## vehicle-nav

Scope: `corridor_nav` `waypoint_follower` node, `pure_pursuit` controller, `local_pose`, `follower/status` + `follower/route` reports, `twist_stamper`, and the `cmd_vel` limits that `corridor_nav` itself applies. Repo at commit `60633c8`. No commits made (task is read-only; nothing to commit atomically).

REQ-VEHICLENAV-01 (Ubiquitous) The waypoint_follower shall run inside a vehicle namespace, subscribe `fix` (sensor_msgs/NavSatFix), `imu` (sensor_msgs/Imu) and `path` (nav_msgs/Path), and publish `cmd_vel` (geometry_msgs/Twist), `follower/status` (std_msgs/String), `follower/route` (std_msgs/Header) and `follower/cross_track` (std_msgs/Float32).
Verified by: `ws/src/corridor_nav/test/test_waypoint_follower_node.py:41-52` (rig publishes/subscribes these topics in a private namespace on real rclpy); `steps/08_proofs.sh:24-25` echoes `/v1/follower/cross_track` and `/v1/follower/status`. Source: `waypoint_follower.py:58-66`; namespace from `launch/kinematic.launch.py:27`.

REQ-VEHICLENAV-02 (Ubiquitous) The waypoint_follower shall derive the vehicle's map-frame position from the GNSS fix through the corridor `LocalFrame` whose origin is the first vertex of the `corridor_geojson` polygon, and shall take heading only from the IMU yaw plus `heading_offset_rad`; wheel odometry shall not be used for navigation.
Verified by: `test/test_local_pose.py::test_position_comes_from_the_fix_not_the_heading_source`, `::test_heading_offset_is_applied_and_wrapped`. The node subscribes no odom topic (`waypoint_follower.py:64-66`). Source: `local_pose.py:14-18`, `waypoint_follower.py:47-48,70-76,109`; decision in `ARCHITECTURE.md:90`.

REQ-VEHICLENAV-03 (Event-driven) When a Path with at least two poses arrives on `path`, the waypoint_follower shall replace the current route, reset segment progress to 0, record the Path header stamp as the route id and set status `following`.
Verified by: `test/test_waypoint_follower.py::test_route_report_carries_the_stamp_of_the_path_it_is_about` (stub rclpy) and the same-named test in `test_waypoint_follower_node.py` (real rclpy) for stamp and status. Segment reset to 0: NOT VERIFIED by any assert. Source: `waypoint_follower.py:78-82`.

REQ-VEHICLENAV-04 (Event-driven) When a Path with fewer than two poses arrives on `path`, the waypoint_follower shall set status `idle` and publish a zero Twist on `cmd_vel` (cancel).
Verified by: `test_waypoint_follower.py::test_route_report_carries_the_stamp_of_the_path_it_is_about` and its node twin (empty Path -> `idle`); `steps/08_proofs.sh:35-49` (#8, geofence EXIT -> status `idle`). Zero-Twist publish on cancel itself: NOT VERIFIED by any assert. Source: `waypoint_follower.py:82-84`.

REQ-VEHICLENAV-05 (Ubiquitous) The waypoint_follower shall publish `follower/status` and `follower/route` with TRANSIENT_LOCAL durability, depth 1, and shall publish both at startup and on every `set_status` call (changed or not); `follower/route.stamp` shall be the header stamp of the Path the status is about and `follower/route.frame_id` shall equal the status string.
Verified by: `test_waypoint_follower.py::test_route_report_carries_the_stamp_of_the_path_it_is_about`, `::test_arrival_is_reported_for_the_path_that_arrived` and their node-test twins. Durability: the node-test rig subscribes both topics with TRANSIENT_LOCAL depth 1 (`test_waypoint_follower_node.py:34,51-52`), which would not match a VOLATILE publisher; late-joiner replay of the latched value itself: NOT VERIFIED. Source: `waypoint_follower.py:59-62,68,86-94`; `ARCHITECTURE.md:91`.

REQ-VEHICLENAV-06 (State-driven) While status is `following`, the waypoint_follower shall run the control tick at `rate_hz` (default 20 Hz), and on each tick where both inputs are present and fresh shall publish one Twist and one cross-track sample.
Verified by: `steps/08_proofs.sh:30` (#7 requires >10 cross-track samples); rate value itself NOT VERIFIED. Source: `waypoint_follower.py:42,67,96-120`.

REQ-VEHICLENAV-07 (State-driven) While status is `following` and no fix or no imu message has been received yet, the waypoint_follower shall not publish any Twist and shall log a throttled warning naming the missing input.
Verified by: `test_waypoint_follower.py::test_following_without_a_fix_or_imu_warns_instead_of_silently_doing_nothing`; `test_waypoint_follower_node.py::test_following_without_a_fix_or_imu_warns_and_does_not_drive`. Source: `waypoint_follower.py:99-102`.

REQ-VEHICLENAV-08 (Unwanted behaviour) If the last fix or the last imu message is older than `pose_timeout_s` (default 1.0 s) while following, then the waypoint_follower shall publish a zero Twist and log a throttled warning naming the stale input, and shall not drive on the remaining live input.
Verified by: `test_waypoint_follower.py::test_stale_inputs_names_the_old_ones` (helper only), `::test_stale_imu_holds_even_with_a_live_fix`; `test_waypoint_follower_node.py::test_stale_imu_holds_even_with_a_live_fix` (also asserts resume when the imu returns). Source: `waypoint_follower.py:31-33,43,103-108`.

REQ-VEHICLENAV-09 (Ubiquitous) The pure_pursuit controller shall aim at the first point at `lookahead_m` along the path from the closest segment, and shall compute angular velocity as `speed * 2 sin(heading_error) / lookahead` for |heading_error| <= pi/2.
Verified by: `test/test_pure_pursuit.py::test_target_left_turns_left`, `::test_target_right_turns_right` (sign only; formula value NOT VERIFIED). Source: `pure_pursuit.py:43-63,88-94`.

REQ-VEHICLENAV-10 (Ubiquitous) The pure_pursuit controller shall clamp angular velocity to ±`max_angular` (keyword default 1.5 rad/s; the node does not pass it, so it is not a ROS parameter), and shall command full ±`max_angular` when the target is more than pi/2 behind the heading.
Verified by: `test_pure_pursuit.py::test_target_behind_turns_at_full_rate` (passes `max_angular=1.5` explicitly; the default value itself NOT VERIFIED). Source: `pure_pursuit.py:78,90-95`; `waypoint_follower.py:110-114`.

REQ-VEHICLENAV-11 (Ubiquitous) The pure_pursuit controller shall command linear velocity `speed_mps * max(0.2, cos(heading_error))`, i.e. never less than 20 % of `speed_mps` and never more than `speed_mps` while following.
Verified by: `test_pure_pursuit.py::test_beside_the_path_the_carrot_is_ahead_not_abeam` (linear > 0.3 at speed 1.0); upper bound and the 0.2 floor value NOT VERIFIED directly. Source: `pure_pursuit.py:96`.

REQ-VEHICLENAV-12 (Unwanted behaviour) If the vehicle is farther than `lookahead_m` from the path (before the first vertex or after overshooting a corner), then the pure_pursuit controller shall aim at a carrot `lookahead_m` ahead of the vehicle's projection on the current segment, clamped to that segment's end, never at the route's end.
Verified by: `test_pure_pursuit.py::test_overshot_corner_aims_at_the_current_segment_end`, `::test_before_the_first_vertex_aims_at_the_route_start_not_the_first_leg_end`, `::test_beside_the_path_the_carrot_is_ahead_not_abeam`. Source: `pure_pursuit.py:63,66-75`.

REQ-VEHICLENAV-13 (Ubiquitous) The pure_pursuit controller shall make progress along the route monotonic: the closest segment is searched only within `window` (default 2) segments from the fed-back `min_segment`, so on a closed route the start vertex is on the first segment, not the closing one.
Verified by: `test_pure_pursuit.py::test_closed_route_start_is_the_first_segment_not_the_last`, `::test_progress_does_not_go_backwards`, `::test_start_is_not_arrival`. Source: `pure_pursuit.py:20-40,78,84`; `waypoint_follower.py:114-115`.

REQ-VEHICLENAV-14 (Event-driven) When the vehicle is within `arrive_tolerance_m` of the last path point and the closest segment is the last one, the waypoint_follower shall publish a zero Twist, drop the route, and set status `arrived` with `follower/route.stamp` equal to that route's stamp.
Verified by: `test_pure_pursuit.py::test_arrival_stops`; `test_waypoint_follower.py::test_arrival_is_reported_for_the_path_that_arrived` and node twin; `steps/08_proofs.sh:28-31` (#7 waits for `arrived`). Source: `pure_pursuit.py:85-87`, `waypoint_follower.py:116-123`.

REQ-VEHICLENAV-15 (Ubiquitous) The waypoint_follower shall publish the signed cross-track error on `follower/cross_track` each driving tick, positive when the vehicle is left of the path direction.
Verified by: `test_pure_pursuit.py::test_cross_track_sign`; `steps/08_proofs.sh:30` (#7 asserts max |xte| < 1.0 m on the kinematic square); `steps/12_gazebo_patrol.sh:17,21` records but does not threshold it. Source: `pure_pursuit.py:37-39`, `waypoint_follower.py:120`.

REQ-VEHICLENAV-16 (Event-driven) When the waypoint_follower process exits (KeyboardInterrupt / shutdown), it shall publish a final zero Twist on `cmd_vel` before destroying the node.
Verified by: NOT VERIFIED. Source: `waypoint_follower.py:126-139` (publish at 135).

REQ-VEHICLENAV-17 (Where) Where the Gazebo ros2_control backend is used, the twist_stamper shall convert each Twist on `cmd_vel` to a TwistStamped on `out_topic` (default `diff_drive/cmd_vel`) with the current clock stamp and `frame_id` (default `base_footprint`; `sim.launch.py` sets `v1/base_footprint`), so follower and planner tools keep one plain-Twist contract on all backends.
Verified by: NOT VERIFIED by a unit test; `steps/08_proofs.sh:101-111` (#5, launched with `use_ros2_control:=true`) drives `/v1/cmd_vel` 1.0 m/s for 5 s and checks ~5 m of odom, which exercises the stamper indirectly. Source: `twist_stamper.py:10-24`, `ws/src/corridor_gz/launch/sim.launch.py:48-49`.

REQ-VEHICLENAV-18 (Ubiquitous) The kinematic launch shall start one waypoint_follower per vehicle `v{first}..v{first+n-1}` with `speed_mps` (launch default 3.0), `lookahead_m` (launch default 3.0), `arrive_tolerance_m` 1.5 and the shared `corridor` GeoJSON. The node's own declared defaults (`speed_mps` 2.0, `lookahead_m` 4.0, `arrive_tolerance_m` 1.0) apply only when it is run without the launch.
Verified by: `steps/08_proofs.sh:21,37` (#7/#8 launch `kinematic.launch.py` with defaults); parameter values themselves NOT VERIFIED. Source: `launch/kinematic.launch.py:18-28,40-44`; `waypoint_follower.py:39-41`; `README.md:42`.

REQ-VEHICLENAV-19 (Ubiquitous) [GAP] The corridor_nav layer shall enforce an upper bound on the commanded linear speed independent of the `speed_mps` parameter. Today the only velocity/acceleration limits are in the ros2_control `diff_drive` config (`cmd_vel_timeout: 0.5`, `linear.x.max_velocity: 4.0`, `angular.z.max_velocity: 2.0`, `corridor_gz/config/controllers_v1.yaml:22-28`); `unicycle_sim` applies no limit (`corridor_sim/unicycle_sim.py:49-52`, only a `cmd_timeout_s` zeroing), and `corridor_nav` clamps only angular (REQ-10). Not planned in PLAN.md.
Verified by: NOT VERIFIED.

REQ-VEHICLENAV-20 (Ubiquitous) [GAP][PLANNED] The waypoint_follower shall be replaceable by Nav2 without touching the mission layer (`ARCHITECTURE.md:43,87,102` keep Nav2 out; obstacle avoidance, if wanted, is a small local planner in `corridor_nav`). No code exists for this.
Verified by: NOT VERIFIED.

Sources: `ws/src/corridor_nav/corridor_nav/waypoint_follower.py`, `pure_pursuit.py`, `local_pose.py`, `twist_stamper.py`; `ws/src/corridor_nav/launch/kinematic.launch.py`; `ws/src/corridor_nav/test/test_waypoint_follower.py`, `test_waypoint_follower_node.py`, `test_pure_pursuit.py`, `test_local_pose.py`; `ws/src/corridor_gz/launch/sim.launch.py`, `ws/src/corridor_gz/config/controllers_v1.yaml`; `ws/src/corridor_sim/corridor_sim/unicycle_sim.py`; `steps/08_proofs.sh`, `steps/12_gazebo_patrol.sh`; `ARCHITECTURE.md:43,87,90-91,102`; `README.md:42`; `PLAN.md`; git HEAD `60633c8`.

## mission

REQ-MISSION-01. The `corridor_msgs` package shall provide the interfaces `Patrol.action` (goal: vehicle, corridor_id, laps, speed_mps, clockwise; result: laps_done, distance_m, result; feedback: lap, lap_progress, distance_m, next_waypoint), `GeofenceEvent.msg` (EXIT=0, ENTER=1, WARN=2, signed distance_m, fix), `VehicleState.msg` and `FleetState.msg` (ws/src/corridor_msgs/CMakeLists.txt:12-15, action/Patrol.action, msg/*.msg).
Verified by: PLAN.md:36 proof `ros2 interface show corridor_msgs/action/Patrol`; steps/08_proofs.sh:58 sends a Patrol goal. No dedicated test of the message set; Patrol and GeofenceEvent are exercised indirectly by test_patrol_server.py:24-25 and test_patrol_server_node.py:31-32.

REQ-MISSION-02. The GeoJSON loader (`corridor_nav.geofence.load_geojson_polygon`) shall accept a file path or inline text and return the id and the lat/lon ring of the first Polygon found in a FeatureCollection, Feature or bare Polygon, reading positions in RFC 7946 lon/lat order, dropping altitude and the closing duplicate vertex, and taking the id from `properties.id`, `properties.name`, `feature.id` or `'corridor'` (geofence.py:40-77).
Verified by: ws/src/corridor_nav/test/test_geofence.py::test_geojson_lonlat_order_and_closing_vertex, ::test_geojson_position_with_altitude, ::test_geojson_unclosed_ring_with_altitude_is_accepted, ::test_geojson_closing_vertex_compared_on_lonlat_only.

REQ-MISSION-03. If a GeoJSON ring has fewer than 3 vertices, no area, a non-ring coordinates value, a non-numeric or non-finite position, or the document holds no Polygon, then the loader shall raise ValueError (geofence.py:28-37, 64-78).
Verified by: test_geofence.py::test_geojson_ring_with_two_vertices_is_rejected, ::test_geojson_empty_ring_is_rejected, ::test_geojson_bad_position_is_rejected, ::test_geojson_collinear_ring_is_rejected, ::test_geojson_coordinates_that_are_not_rings_and_huge_integers_are_rejected.

REQ-MISSION-04. The mission layer (`patrol_server`, `geofence_monitor`) shall convert fixes to local ENU metres through a `LocalFrame` whose origin is the polygon's first vertex, and shall compute inside/outside by the even-odd rule with a signed edge distance (positive inside, negative outside) (geofence.py:11-25, 81-111; patrol_server.py:148-150; geofence_monitor.py:27-29).
Verified by: test_geofence.py::test_inside_and_outside, ::test_signed_distance_sign_and_magnitude, ::test_concave_polygon, ::test_local_frame_roundtrip.

REQ-MISSION-05. If the `corridor_geojson` parameter is empty, then `patrol_server` and `geofence_monitor` shall raise ValueError at construction (patrol_server.py:145-147; geofence_monitor.py:24-26).
Verified by: NOT VERIFIED.

REQ-MISSION-06. The `geofence_monitor` shall subscribe to `/<v>/fix` for every name in parameter `vehicles` and, on each fix, publish a `GeofenceEvent` on `/fleet/geofence_events`: EXIT when the vehicle goes from inside to outside (or its first fix is outside; the first fix is checked for EXIT only), ENTER when it comes back inside, WARN once per approach when inside but closer than `warn_margin_m` (default 20.0) to the edge, re-armed when the distance reaches the margin again (geofence_monitor.py:21, 34-78).
Verified by: steps/08_proofs.sh:36-50 (proof #8, EXIT `type: 0` within 30 s). ENTER and WARN: NOT VERIFIED. No unit test imports geofence_monitor.

REQ-MISSION-07. When a vehicle leaves the polygon and parameter `cancel_on_exit` is true (default), the `geofence_monitor` shall publish an empty `nav_msgs/Path` on `/<v>/path` so the follower stops (geofence_monitor.py:22, 74-78).
Verified by: steps/08_proofs.sh:44-50 (follower status must be `idle` after the EXIT event). PLAN.md:40 requires the proof to be run with the monitor disabled first and fail; steps/08 does not do that negative run.

REQ-MISSION-08. The `patrol_server` shall run in a vehicle namespace, expose action `patrol` of type `corridor_msgs/action/Patrol`, and reject a goal whose non-empty `corridor_id` differs from the loaded polygon id or whose non-empty `vehicle` differs from parameter `vehicle` (patrol_server.py:141-170, 187-193).
Verified by: steps/08_proofs.sh:58 and steps/12_gazebo_patrol.sh:18 (accepted goal on /v1/patrol). Rejection branches: NOT VERIFIED.

REQ-MISSION-09. When a Patrol goal is accepted and a fix is known, the `patrol_server` shall publish a `Path` in frame `map` of the polygon vertices moved toward the centroid by `inset_m` (default 15.0), ordered by goal `clockwise`, starting at the vertex nearest the vehicle's fix position and closed back to that vertex (patrol_server.py:42-66, 144, 221, 264-268).
Verified by: test_patrol_route.py::test_inset_moves_vertices_inward, ::test_lap_route_starts_near_vehicle_and_closes, ::test_lap_route_direction; test_patrol_server.py::test_lap_starts_at_the_vertex_nearest_the_fix_not_the_odometry; test_patrol_server_node.py::test_lap_starts_at_the_vertex_nearest_the_fix.

REQ-MISSION-10. If no fix arrives within 5 s of goal acceptance, then the `patrol_server` shall abort the goal with result `error` (patrol_server.py:255-263).
Verified by: NOT VERIFIED.

REQ-MISSION-11. While a goal runs, the `patrol_server` shall publish feedback at least every 0.5 s (and on every route report, geofence event or cancel that wakes the loop) with the current lap, lap_progress 0..1 derived from distance travelled along the fix, cumulative distance_m, and the next route waypoint (patrol_server.py:269-291).
Verified by: test_patrol_server.py::test_feedback_distance_and_next_waypoint_follow_the_fix; steps/08_proofs.sh:59 counts `lap_progress` lines.

REQ-MISSION-12. The `patrol_server` shall stamp every Path it publishes with a unique, increasing, odd nanosecond stamp, and shall count a lap only when `follower/route` reports status `arrived` for that stamp; on a counted lap it re-sends the route, and succeeds with result `done` once `laps_done >= laps` for `laps > 0` (laps = 0 runs until cancelled) (patrol_server.py:88-107, 134-135, 274-278).
Verified by: test_patrol_server.py::test_route_owner_follows_the_stamps, ::test_stamps_are_unique_even_while_the_clock_stands_still, ::test_server_stamps_are_odd_so_a_sim_time_path_from_another_node_never_matches, ::test_own_arrival_completes_the_lap; test_patrol_server_node.py::test_own_arrival_completes_the_lap; steps/08_proofs.sh:61 (`result: done`) and steps/12_gazebo_patrol.sh:23 (`result: done`, distance > 380 m).

REQ-MISSION-13. If the follower reports holding a Path this server did not publish, then the `patrol_server` shall end the running goal as `preempted` without counting a lap and without clearing the follower (patrol_server.py:102-104, 132-133, 295).
Verified by: test_patrol_server.py::test_foreign_arrival_is_not_a_lap_and_does_not_resend_the_route, ::test_foreign_empty_path_ends_the_goal_as_preempted_not_done, ::test_cancel_does_not_clear_a_route_the_server_no_longer_owns; same names in test_patrol_server_node.py.

REQ-MISSION-14. When a `GeofenceEvent` of type EXIT for this server's `vehicle` arrives on `/fleet/geofence_events`, the `patrol_server` shall abort the current goal with result `geofence` and publish an empty Path if the follower still holds (or is about to receive) its route (patrol_server.py:102-104, 130-131, 180-185, 295-296).
Verified by: test_patrol_server.py::test_geofence_exit_clears_the_route_when_the_monitor_does_not, ::test_geofence_exit_is_geofence_even_when_the_monitor_stopped_the_follower_first; same names in test_patrol_server_node.py; test_patrol_server.py::test_verdict_precedence.

REQ-MISSION-15. When a cancel request arrives for the current goal, the `patrol_server` shall publish the empty Path (if it still owns the route) inside the cancel callback before answering the client, and finish the goal as `cancelled` (patrol_server.py:209-216, 293-311).
Verified by: test_patrol_server.py::test_cancel_clears_the_route_inside_the_cancel_callback, ::test_cancel_of_a_goal_that_is_no_longer_current_does_not_clear_the_current_route, ::test_planner_sequence_cancel_then_patrol_leaves_the_new_route_in_place; test_patrol_server_node.py::test_cancel_is_answered_after_the_clear_so_a_successor_route_survives, ::test_planner_sequence_cancel_then_patrol_leaves_the_new_route_in_place.

REQ-MISSION-16. When a new goal is accepted while one runs, the `patrol_server` shall mark the old goal `preempted`, wait up to 5 s for its loop to exit (logging an error and proceeding if it does not), and then start the new goal, so the two do not publish over each other's route (patrol_server.py:195-207).
Verified by: test_patrol_server.py::test_new_goal_preempts_the_running_one_without_clearing_the_follower; test_patrol_server_node.py::test_new_goal_preempts_the_running_one_without_clearing_the_follower.

REQ-MISSION-17. The `kinematic.launch.py` shall start, for each of `n` vehicles numbered from `first`, a `unicycle_sim`, a `waypoint_follower` and a `patrol_server` under namespace `/v<i>` with staggered start_x (20 + 25 m per vehicle number) and inset_m (30 + 8 m per launch index), plus one `geofence_monitor` with `vehicles` set to all names and `warn_margin_m` 15 (ws/src/corridor_nav/launch/kinematic.launch.py:20-33, 40-42).
Verified by: steps/08_proofs.sh:66-80 (proof #10: n:=3, each of /v1 /v2 /v3 patrols with odom speed > 1 m/s). The PLAN.md:44 criterion `/clock >= 50 Hz` is not checked by any steps/*.sh (`grep /clock steps/` empty); unicycle_sim publishes no /clock (ws/src/corridor_sim/corridor_sim/unicycle_sim.py, only `get_clock()` calls).

REQ-MISSION-18. [GAP] The `patrol_server` shall honour the goal field `speed_mps` (0 = vehicle default) when driving the follower (Patrol.action:5). The field is never read by patrol_server.py (no `speed_mps` reference); the follower speed comes only from its own `speed_mps` parameter (waypoint_follower.py:39, 111; kinematic.launch.py:28). The planner always sends 0.0 (corridor_planner/mission_planner.py:170).
Verified by: NOT VERIFIED.

REQ-MISSION-19. [GAP] The mission layer shall publish `corridor_msgs/FleetState` on `/fleet/state` (ARCHITECTURE.md:100 states nobody publishes it; `grep -rn FleetState ws/src` outside corridor_msgs finds no use at all).
Verified by: NOT VERIFIED.

REQ-MISSION-20. [PLANNED] Where the Gazebo backend runs multiple vehicles, the system shall keep `ros2 topic hz /clock` at or above 50 Hz with three vehicles patrolling (PLAN.md:43-44). sim.launch.py spawns only v1 (ws/src/corridor_gz/launch/sim.launch.py:1, 38); kinematic v2/v3 can run beside it via `first:=2 n:=2` (kinematic.launch.py:42).
Verified by: NOT VERIFIED.

Sources: ws/src/corridor_msgs/{CMakeLists.txt,action/Patrol.action,msg/GeofenceEvent.msg,msg/VehicleState.msg,msg/FleetState.msg}; ws/src/corridor_nav/corridor_nav/{geofence.py,patrol_server.py,geofence_monitor.py,waypoint_follower.py}; ws/src/corridor_nav/launch/kinematic.launch.py; ws/src/corridor_nav/test/{test_geofence.py,test_patrol_route.py,test_patrol_server.py,test_patrol_server_node.py}; ws/src/corridor_gz/launch/sim.launch.py; ws/src/corridor_sim/corridor_sim/unicycle_sim.py; ws/src/corridor_planner/corridor_planner/mission_planner.py; steps/08_proofs.sh; steps/12_gazebo_patrol.sh; PLAN.md:35-44; ARCHITECTURE.md:100; git HEAD 60633c8.

## planner

Package `ws/src/corridor_planner` (nodes `mission_planner`, `status_loop`; model side `planner.py`; schemas `tools.py`). Launch: `launch/mission.launch.py`. Line refs are to files under `ws/src/corridor_planner/` (`corridor_planner/*.py`, `test/*.py`, `launch/*.py`).

REQ-PLANNER-01 (Ubiquitous): The mission_planner shall offer the model exactly the tools `list_vehicles`, `get_vehicle`, `patrol`, `goto`, `stop`, `geofence_events`, `describe_scene`, each with a strict JSON input schema (`strict: True`, `additionalProperties: False`) (`tools.py:3-29`, `mission_planner.py:122-124,128`).
Verified by: `test/test_planner.py::test_request_shape_has_fallbacks_and_no_cmd_vel_tool` asserts only that no tool name/description contains `cmd_vel` (`test_planner.py:93`); the exact tool set is NOT VERIFIED by a unit test. `steps/08_proofs.sh` #12 (lines 91-97) and `steps/10_scene_proof.sh:16-20` exercise `patrol`, `list_vehicles`, `describe_scene` over the debug topic.

REQ-PLANNER-02 (Ubiquitous): The mission_planner shall never publish `cmd_vel`; it shall act only through `Patrol` action goals (`patrol`), a two-pose `/vN/path` (`goto`), and goal cancel plus an empty Path (`stop`) (`mission_planner.py:5,110-111,164-237`; ARCHITECTURE.md:67).
Verified by: `test/test_planner.py::test_request_shape_has_fallbacks_and_no_cmd_vel_tool` (no cmd_vel tool offered); no test asserts the absence of a `cmd_vel` publisher on the node. Note: ARCHITECTURE.md:101 (open item 4) records that `goto` publishing a Path violates the "only action goals" rule.

REQ-PLANNER-03 (Ubiquitous): The Planner shall call `client.beta.messages.create` with model `claude-opus-5-5` (node parameter `model`, default `PLANNING_MODEL`), `fallbacks='default'`, beta `server-side-fallback-2026-07-01`, `max_tokens=4096` and `output_config={'effort': <effort>}` from the node parameter `effort` (default `medium`) (`planner.py:12-13,39-40,46-56`; `mission_planner.py:91-92,128-129`).
Verified by: `test/test_planner.py::test_request_shape_has_fallbacks_and_no_cmd_vel_tool` (model, fallbacks, betas; not max_tokens or effort).

REQ-PLANNER-04 (Event-driven): When a `std_msgs/String` arrives on `/mission/request`, the mission_planner shall run `Planner.run` on a daemon worker thread and publish the outcome text on `/mission/reply` (with `dry_run:=true` it replies `dry run, no model: <text>` without a model call) (`mission_planner.py:93,115,118,275-302,306-308`).
Verified by: `steps/11_live_model_proof.sh:21-30` (#12/#13, model path, needs ANTHROPIC_API_KEY); no unit test of `on_request`/`handle_request`.

REQ-PLANNER-05 (State-driven): While a previous request is still being handled, the mission_planner shall reply `busy with the previous request` on `/mission/reply` and not start a second model run (`mission_planner.py:131,276-278,303-304`).
Verified by: NOT VERIFIED.

REQ-PLANNER-06 (Ubiquitous): The Planner shall check `stop_reason` before reading `content` on every response (`planner.py:106-118`).
Verified by: `test/test_planner.py::test_refusal_is_an_outcome_not_an_exception` (refusal response with `content=[]` handled).

REQ-PLANNER-07 (Unwanted behaviour): If a response has `stop_reason == 'refusal'`, then the Planner shall return an `Outcome` with `refused=True` and `category` from `stop_details.category`, append a `refusal` trace event, drop the whole exchange from history, and raise no exception (`planner.py:106-112`).
Verified by: `test/test_planner.py::test_refusal_is_an_outcome_not_an_exception`, `test_refusal_mid_tool_loop_rolls_back_the_whole_exchange`.

REQ-PLANNER-08 (Unwanted behaviour): If the outcome is refused, then the mission_planner shall publish a `DiagnosticStatus` WARN named `mission_planner` with message `refusal: <category|uncategorised>` on `/diagnostics` and reply `the model declined this request (<category|no category>)` (`mission_planner.py:117,294-296,310-317`).
Verified by: NOT VERIFIED (PLAN.md:51 asks for this proof; only the model-side half in `test_planner.py` exists, the diagnostic publish is untested).

REQ-PLANNER-09 (Unwanted behaviour): If the SDK raises during `messages.create` or while answering tool calls, then the Planner shall return an `Outcome` with `error` set, append an `error` trace event, drop the exchange so no `tool_use` is left unanswered, and keep the node alive; the node shall publish a DiagnosticStatus ERROR and reply `planner error: ...` (`planner.py:95-101,125-131`; `mission_planner.py:297-299`).
Verified by: `test/test_planner.py::test_sdk_error_keeps_node_alive`, `test_sdk_error_mid_tool_loop_rolls_back_the_whole_exchange`, `test_rollback_keeps_the_earlier_exchanges`, `test_a_tool_result_that_breaks_the_trace_rolls_back_the_exchange`; diagnostic publish NOT VERIFIED.

REQ-PLANNER-10 (Unwanted behaviour): If a tool callable raises or the model names an unknown tool, then the Planner shall return a `tool_result` with `is_error=True` and an `{'error': ...}` payload instead of aborting the exchange (`planner.py:69-76`).
Verified by: `test/test_planner.py::test_tool_exception_becomes_error_result` (raising tool only; the unknown-tool branch at `planner.py:70-71` is untested).

REQ-PLANNER-11 (Ubiquitous): The Planner shall stop the tool loop after `max_turns` (12) model turns and report `error='max_turns reached'` (`planner.py:41,93,133-134`).
Verified by: NOT VERIFIED.

REQ-PLANNER-12 (Event-driven): When a request completes, the mission_planner shall publish every new trace event (`request`, `tool`, `reply`, `refusal`, `error`) as one JSON `std_msgs/String` per event on `/mission/llm_trace`, with tool events carrying `name`, `input`, `result` summary and `is_error` (`planner.py:84,92,99,110,119,129,134`; `mission_planner.py:116,290-293`).
Verified by: `test/test_planner.py::test_refusal_is_an_outcome_not_an_exception`, `test_max_tokens_with_tool_use_blocks_keeps_no_unanswered_tool_use`, `test_rich_tool_content_passes_blocks_and_trims_trace` (trace list contents); `steps/08_proofs.sh` #12, `steps/10_scene_proof.sh`, `steps/11_live_model_proof.sh` (topic). Trace is published after the run, not per call.

REQ-PLANNER-13 (Event-driven): When a JSON `{"name","input"}` arrives on `/mission/tool_call`, the mission_planner shall run that tool without the model and publish its result on `/mission/llm_trace` as a `tool` event with `source: "debug"` (`mission_planner.py:119-120,261-273`).
Verified by: `steps/08_proofs.sh:91-97` (#12, asserts `"accepted": true` and `"mode": "patrol"` in the trace), `steps/10_scene_proof.sh:16-20` (asserts `content_blocks ["image", "text"]`).

REQ-PLANNER-14 (Ubiquitous): The Planner shall accumulate `usage.input_tokens`/`output_tokens` per request into the `Outcome`, and the mission_planner shall put `tool_calls`, `turns`, `input_tokens`, `output_tokens` as KeyValues on every `/diagnostics` status (`planner.py:102-105`; `mission_planner.py:310-317`).
Verified by: `test/test_planner.py::test_tool_loop_executes_and_feeds_back` asserts `tool_calls` and `turns` only; token sums and the diagnostic KeyValues are NOT VERIFIED.

REQ-PLANNER-15 (Event-driven): When `describe_scene(vehicle)` is called, the mission_planner shall convert the latest `/vN/camera/image` (rgb8/bgr8) to a PNG downscaled by the integer factor `max(1, width // 320)` and return it as a base64 image block plus a text block, with the trace showing only the block types (`camera.py:6-24`; `mission_planner.py:109,246-258`; `planner.py:77-81`).
Verified by: `test/test_planner.py::test_rich_tool_content_passes_blocks_and_trims_trace`; `steps/10_scene_proof.sh` (debug path); `steps/11_live_model_proof.sh:21-25` (#13, model path). `image_to_png` itself has no unit test.

REQ-PLANNER-16 (Unwanted behaviour): If `goto` targets a point outside the corridor polygon or the vehicle has no fix, then the mission_planner shall return `accepted: False` with a reason and cancel nothing (`mission_planner.py:186-191`).
Verified by: `test/test_mission_tools.py::test_goto_outside_the_corridor_is_refused_before_anything_is_stopped`, `test_goto_without_a_fix_is_refused_and_cancels_nothing`.

REQ-PLANNER-17 (Event-driven): When `stop(vehicle)` is called, the mission_planner shall publish an empty Path as soon as the cancel is sent, wait at most `STOP_TIMEOUT_S` (3 s) for the old goal to exit, and report `finished` true/false (`mission_planner.py:36,47-65,220-237`).
Verified by: `test/test_mission_tools.py::test_stop_publishes_the_empty_path_right_after_the_cancel_not_after_the_old_goal_exited`, `test_stop_is_bounded_when_the_server_never_answers`, `test_stop_reports_whether_the_old_goal_finished`; `test/test_mission_planner_node.py::test_stop_returns_after_the_cancelled_goal_has_exited` (needs ROS, on Thor).

REQ-PLANNER-18 (Optional): Where `mission.launch.py` is started with `status_loop:=true`, the status_loop shall every `period_s` (30 s) send one `claude-haiku-5-5` request (`client.messages.create`, `max_tokens=200`, `output_config.effort` low) with the fleet snapshot and publish the text on `/mission/summary` (`status_loop.py:20,29-31,42,52,59-75`; `launch/mission.launch.py:21,28-29`).
Verified by: `steps/11_live_model_proof.sh:15,31-33` (model path only; the proof greps the `cost:` log line, `/mission/summary` is not read); no unit test of `StatusLoop`. [GAP] ARCHITECTURE.md:64 says both models run with `fallbacks: "default"` under the beta; `status_loop.py:61-64` uses the non-beta `client.messages.create` with no `fallbacks`/`betas`.

REQ-PLANNER-19 (Event-driven): When a status tick returns a non-refused response, the status_loop shall publish `{calls, input_tokens, output_tokens, usd, usd_per_hour}` on `/mission/summary_cost` priced at $0.10/$0.50 per 1M tokens and log `cost: calls=...`; `usage` and `calls` are accumulated for every returned response, refused ones included (`status_loop.py:21,43,68-82`).
Verified by: `steps/11_live_model_proof.sh:33` greps the log line; line 19 echoes `/mission/summary_cost` to a file that is never read; no assertion on the value.

REQ-PLANNER-20 (Unwanted behaviour): If the status_loop request raises or returns `stop_reason == 'refusal'`, then the status_loop shall log a warning and skip the tick, never crash (`status_loop.py:65-73`).
Verified by: NOT VERIFIED.

REQ-PLANNER-21 (Ubiquitous) [PLANNED] [GAP]: The README shall record the measured status-loop cost per hour from `usage`, target < $0.20/h (PLAN.md:56-57). README.md contains no cost figure (grep `cost|usd` finds none); PLAN.md:11 says #14 was proven live but the number was not written down.
Verified by: NOT VERIFIED.

REQ-PLANNER-22 (Ubiquitous) [GAP]: The mission_planner's `list_vehicles` shall read `/fleet/state` (ARCHITECTURE.md:55). Code instead subscribes per vehicle to `/vN/fix`, `/vN/odom`, `/vN/follower/status` and summarises itself (`mission_planner.py:104-111,136-158`); ARCHITECTURE.md:100 lists this as open item 3; no node under `ws/src/corridor_*` publishes `/fleet/state` (grep).
Verified by: NOT VERIFIED.

REQ-PLANNER-23 (Unwanted behaviour): If a response ends with `stop_reason != 'tool_use'` but still contains `tool_use` blocks (e.g. cut by `max_tokens`), then the Planner shall not run those tools, append a `reply` trace event with that `stop_reason`, and drop the exchange so no `tool_use` is left unanswered (`planner.py:117-124`).
Verified by: `test/test_planner.py::test_max_tokens_with_tool_use_blocks_keeps_no_unanswered_tool_use`.

Sources: `ws/src/corridor_planner/corridor_planner/planner.py`, `mission_planner.py`, `status_loop.py`, `camera.py`, `tools.py`; `ws/src/corridor_planner/launch/mission.launch.py`; `ws/src/corridor_planner/test/test_planner.py`, `test_mission_tools.py`, `test_mission_planner_node.py`; `steps/08_proofs.sh`, `steps/10_scene_proof.sh`, `steps/11_live_model_proof.sh`; ARCHITECTURE.md:49-67,100-101; PLAN.md:9-11,48-57; README.md (grep `cost|usd`: no match); git HEAD 60633c8.

## atak

Scope: `corridor_cot` (`cot.py`, `cot_bridge.py`), FreeTAKServer container, marker to `/mission/request`, `MarkerGate`. PLAN Phase 4 (#15, #16). Repo root: /Volumes/GOKSSD/claude/ros2-corridor-swarm. Nothing was committed (task is read-only); the "atomic commits" instruction does not apply to this step.

Notes on sources of truth vs code:
- ARCHITECTURE.md:47 and PLAN.md:65 say `cot_bridge` reads `/fleet/state`. The code subscribes `/{v}/fix` (NavSatFix) and `/{v}/odom` (Odometry) per vehicle (cot_bridge.py:37-40). Requirements below follow the code.
- No launch file starts `cot_bridge` (grep over ws/src/*/launch: no hit). It is run by hand / by steps/13 and steps/14 via `ros2 run corridor_cot cot_bridge` (steps/13_cot_proof.sh:15, steps/14_marker_proof.sh:12).
- No script in the repo creates the FreeTAKServer container; steps/13_cot_proof.sh:18 only reads logs of a container named `corridor-fts`.
- `MarkerGate` is a class inside `cot.py` (cot.py:111), not a node.

REQ-ATAK-01 (Ubiquitous) The cot_bridge node shall declare parameters `vehicles` (default `['v1']`), `host` (default `$FTS_HOST` or `127.0.0.1`), `port` (8087), `period_s` (2.0), `fix_timeout_s` (5.0), `marker_move_m` (5.0) and `team` (`Cyan`).
Verified by: NOT VERIFIED (defaults in cot_bridge.py:27-33; README.md:47 documents them; no test asserts the defaults).

REQ-ATAK-02 (Event-driven) When a NavSatFix on `/<vehicle>/fix` arrives with `status.status >= STATUS_FIX`, the cot_bridge shall store lat, lon, hae and the ROS clock time of receipt for that vehicle.
Verified by: test_cot_bridge_node.py::test_fix_without_a_fix_status_is_ignored and ::test_silent_vehicle_gets_no_pli_after_fix_timeout (positive branch via deliver_fix) (cot_bridge.py:54-60).

REQ-ATAK-03 (Unwanted behaviour) If a NavSatFix arrives with `status.status < STATUS_FIX`, then the cot_bridge shall ignore it and send no PLI for that vehicle.
Verified by: test_cot_bridge_node.py::test_fix_without_a_fix_status_is_ignored (cot_bridge.py:58-59).

REQ-ATAK-04 (Event-driven) When an Odometry on `/<vehicle>/odom` arrives, the cot_bridge shall derive compass course (0 = north, clockwise) from the quaternion z,w and speed as `abs(twist.linear.x)`.
Verified by: test_cot.py::test_course_from_quaternion (cot.py:138-141); odom wiring in cot_bridge.py:38-40 NOT VERIFIED.

REQ-ATAK-05 (Ubiquitous) The cot_bridge shall, every `period_s`, build one CoT PLI event per vehicle with a fix: `uid=corridor-<vehicle>`, `callsign=<VEHICLE>`, `type=a-f-G-E-V-C`, `how=m-g`, stale = now + 60 s, lat/lon to 7 decimals, `<__group name=<team>>`, `<track course speed>`, `<takv device=corridor_sim>`.
Verified by: test_cot.py::test_pli_event_shape, ::test_pli_payload_one_event_per_vehicle_with_a_fix, ::test_pli_events_is_one_string_per_live_vehicle_and_the_payload_their_concatenation (cot.py:14-39; timer cot_bridge.py:44).

REQ-ATAK-06 (Unwanted behaviour) If a vehicle's last fix is older than `fix_timeout_s`, then the cot_bridge shall omit that vehicle from the PLI batch so its icon goes stale in ATAK instead of being re-sent as a live position.
Verified by: test_cot.py::test_pli_payload_drops_a_vehicle_whose_fix_went_silent; test_cot_bridge_node.py::test_silent_vehicle_gets_no_pli_after_fix_timeout (cot.py:35; commit 33f5207).

REQ-ATAK-07 (Ubiquitous) The cot_bridge shall send PLI events over plain TCP (no TLS) to `host:port`.
Verified by: steps/13_cot_proof.sh:17 (live server, `connected to TAK server` log) (cot_bridge.py:47-52, 75).

REQ-ATAK-07b (State-driven) While no vehicle has a live fix, the cot_bridge shall send nothing (and open no connection).
Verified by: test_cot_bridge_node.py::test_fix_without_a_fix_status_is_ignored (`sock.sent == []`; the no-connect part NOT VERIFIED) (cot_bridge.py:65-66).

REQ-ATAK-08 (Event-driven) When a TCP connection to the TAK server is (re)established, the cot_bridge shall write exactly one event as its first write (FreeTAKServer registration) and send the full fleet batch from the next tick on.
Verified by: test_cot_bridge_node.py::test_a_new_connection_registers_with_one_event_and_the_fleet_follows_on_the_next_tick; steps/13_cot_proof.sh:58-60 (`bridge connections == 1`) (cot_bridge.py:69-75; commit 1c99eb8).

REQ-ATAK-09 (Unwanted behaviour) If a socket write fails with OSError, then the cot_bridge shall log a throttled warning (10 s), drop the socket and reconnect on the next tick, never raising out of the timer callback.
Verified by: NOT VERIFIED (cot_bridge.py:76-78; steps/13_cot_proof.sh:60 only asserts the absence of `TAK send failed`).

REQ-ATAK-10 (Unwanted behaviour) If the reader receives an empty chunk or an OSError, then the cot_bridge shall drop the socket, clear its buffer and wait for the sender to reconnect.
Verified by: NOT VERIFIED (cot_bridge.py:85-96).

REQ-ATAK-11 (Ubiquitous) The cot_bridge reader shall split the inbound stream on `</event>`, hand each event to `on_event`, and on any exception log `CoT event dropped` and continue with the next event.
Verified by: test_cot_bridge_node.py::test_reader_survives_an_event_that_raises_and_logs_it (cot_bridge.py:97-106; commit 9846f0b).

REQ-ATAK-12 (Event-driven) When an inbound CoT event has type prefix `b-m-p`, `a-u-G` or `a-h-G`, the cot_bridge shall treat it as a marker and extract uid, callsign, lat, lon, type; any other event type, non-XML, or non-`event` root shall be ignored silently.
Verified by: test_cot.py::test_parse_marker_accepts_waypoint_and_rejects_pli, ::test_marker_or_reason_names_why_a_marker_is_dropped (cot.py:70-78, 90-92; non-`event` root branch cot.py:74-75 has no test).

REQ-ATAK-13 (Unwanted behaviour) If a marker's `<point>` is missing, or lat/lon is missing, non-numeric, NaN, infinite or out of [-90,90]/[-180,180], then the cot_bridge shall drop it and log `marker dropped: <reason>`.
Verified by: test_cot.py::test_parse_marker_returns_none_for_unusable_coordinates, ::test_marker_or_reason_names_why_a_marker_is_dropped; test_cot_bridge_node.py::test_reader_ignores_a_marker_without_coordinates_and_goes_on (node level, no request published) (cot.py:47-53, 84-89; cot_bridge.py:110-111; commits b53ac85, adecbc0).

REQ-ATAK-14 (Unwanted behaviour) If a marker's `stale` timestamp has already passed, then the cot_bridge shall drop it with a logged reason; an unreadable `stale` shall be taken as no expiry.
Verified by: test_cot.py::test_parse_marker_drops_a_marker_whose_stale_time_has_passed; test_cot_bridge_node.py::test_stale_marker_is_not_a_request (cot.py:56-63, 81-83; commit 8e0a6c8).

REQ-ATAK-15 (Unwanted behaviour) If an inbound marker's uid starts with `corridor-` (the fleet's own PLI relayed back), then the cot_bridge shall not publish a request for it.
Verified by: NOT VERIFIED (cot_bridge.py:112; own PLI type `a-f-G-E-V-C` is already rejected by REQ-12's type filter, so this guard has no test of its own).

REQ-ATAK-16 (Ubiquitous) The MarkerGate shall admit a uid the first time and again only when its position moved at least `move_m` metres (flat-earth distance) from the last admitted position; a re-send at the same spot, a rename, or jitter under `move_m` shall be rejected.
Verified by: test_cot.py::test_marker_gate_passes_a_placement_once_and_again_only_when_moved; test_cot_bridge_node.py::test_same_marker_twice_is_one_request_and_a_move_is_another; steps/14_marker_proof.sh:24 (marker sent twice) (cot.py:105-108, 111-135; commit 883fde6).

REQ-ATAK-17 (Ubiquitous) The MarkerGate shall remember at most `capacity` (256) uids, forgetting the oldest first, and shall always admit a marker with an empty uid.
Verified by: test_cot.py::test_marker_gate_forgets_the_oldest_uid_beyond_its_capacity_and_never_blocks_a_uidless_marker (cot.py:118-135; commit c81a5b4).

REQ-ATAK-18 (Event-driven) When a marker passes parsing, the `corridor-` filter and the MarkerGate, the cot_bridge shall publish std_msgs/String on `/mission/request` with data `send the nearest vehicle to marker <callsign or uid> at <lat:.6f>, <lon:.6f>` and log `marker -> request`.
Verified by: test_cot_bridge_node.py::test_same_marker_twice_is_one_request_and_a_move_is_another (coordinate text), steps/14_marker_proof.sh:28-30 (end-to-end through FreeTAKServer, `RALLY1` on `/mission/request`) (cot_bridge.py:41, 112-115).

REQ-ATAK-19 (Ubiquitous) [PLANNED] [GAP] The FreeTAKServer shall run in Docker on Thor as a named container (`corridor-fts`) owned by this project, never touching other containers, and an ATAK client in the emulator shall connect to it.
Verified by: NOT VERIFIED (PLAN.md:63-64 #15 open per README.md:32; steps/13_cot_proof.sh:12,18 only checks a listener on 8087 and reads `corridor-fts` logs; no container setup script in the repo).

REQ-ATAK-20 (Ubiquitous) [PLANNED] [GAP] Three vehicles shall be visible and moving in ATAK, and dropping a marker in ATAK shall send a vehicle there.
Verified by: NOT VERIFIED (PLAN.md:65-66 #16 proof; steps/13 proves server-side relay to a second CoT client (steps/13_cot_proof.sh:53-55, 61), steps/14 proves marker to `/mission/request`; the ATAK display and the planner acting on the request are not exercised by any script in this area).

Sources:
- ws/src/corridor_cot/corridor_cot/cot_bridge.py (1-127)
- ws/src/corridor_cot/corridor_cot/cot.py (1-141)
- ws/src/corridor_cot/test/test_cot.py; ws/src/corridor_cot/test/test_cot_bridge_node.py
- steps/13_cot_proof.sh; steps/14_marker_proof.sh
- ARCHITECTURE.md:10-14, 47; PLAN.md:61-66; README.md:32, 47; .env.example:5
- git log: 33f5207, 1c99eb8, 9846f0b, b53ac85, adecbc0, 8e0a6c8, 883fde6, c81a5b4, 980bd92, edcd6fb, cf203a4, eca1a62

## operator-ops

Scope: Foxglove bridge and layout, launch arguments, `.env`/`env.sh` secrets, `steps/` proof scripts, tmux/logs. Read from the launch files, `env.sh`, `steps/lib.sh`, the proof scripts, `foxglove/corridor.json`, `cot_bridge.py`, `mission_planner.py`, `map_overlay.py` and the docs. Note: the repo was read only; nothing was edited or committed (the "atomic commits" reminder has nothing to apply to in this task).

REQ-OPERATOROPS-01 (Ubiquitous). The `kinematic.launch.py`, `sim.launch.py` and `day1.launch.py` launch files shall start `foxglove_bridge` with `port: 8765`, `address: 0.0.0.0` and `send_buffer_limit: 100000000` (`kinematic.launch.py` lines 46-48, `sim.launch.py` lines 31-33, `day1.launch.py` lines 7-11).
Verified by: `steps/04_run.sh` line 31 (`ss -ltnp | grep 8765`, day1 only); kinematic/sim bridge start NOT VERIFIED by any script or test.

REQ-OPERATOROPS-02 (Optional). Where the `foxglove` launch argument is `false` on `kinematic.launch.py` or `sim.launch.py`, the launch shall not start `foxglove_bridge` (`IfCondition(LaunchConfiguration('foxglove'))`, `kinematic.launch.py` lines 45-48, `sim.launch.py` lines 30-33).
Verified by: NOT VERIFIED.

REQ-OPERATOROPS-03 (Ubiquitous). `sim.launch.py` shall start `foxglove_bridge`, `robot_state_publisher` and `parameter_bridge` (and, under `use_ros2_control`, `twist_stamper`) with `use_sim_time: True` (`sim.launch.py` lines 33, 36, 41, 49), per ARCHITECTURE.md line 41 "Sim time everywhere".
Verified by: `steps/08_proofs.sh` block #4 (topic hz under sim time, lines 117-130) covers the bridge yaml path only; the foxglove_bridge `use_sim_time` parameter is NOT VERIFIED.

REQ-OPERATOROPS-04 (Ubiquitous). `kinematic.launch.py` shall declare launch arguments `corridor` (default `corridor_nav/config/corridor_demo.geojson`), `n` (1), `first` (1), `lookahead_m` (3.0), `speed_mps` (3.0), `foxglove` (true), and shall start `unicycle_sim`, `waypoint_follower`, `patrol_server` under namespaces `v{first}..v{first+n-1}` plus one `geofence_monitor` with `vehicles` set to those names (`kinematic.launch.py` lines 13-49; README.md line 42).
Verified by: `steps/08_proofs.sh` block #10 (`n:=3`, lines 65-80) and `steps/13_cot_proof.sh` line 13 for `n`; `first`, `lookahead_m`, `speed_mps` NOT VERIFIED.

REQ-OPERATOROPS-05 (Ubiquitous). `sim.launch.py` shall declare `world` (default `corridor_gz/worlds/corridor.sdf`), `use_ros2_control` (false) and `foxglove` (true); while `use_ros2_control` is `true` it shall additionally spawn `joint_state_broadcaster`, `diff_drive` (odom remapped to `/v1/odom`) and `twist_stamper` (`sim.launch.py` lines 28-50; README.md line 43).
Verified by: `steps/08_proofs.sh` block #5 (`use_ros2_control:=true`, lines 101-115) and block #4 (default); `steps/12_gazebo_patrol.sh` line 12. [GAP] README.md line 43 (launch table) does not list the `foxglove` argument that `sim.launch.py` line 30 declares; README.md line 51 does mention `foxglove:=false` for "both launch files".

REQ-OPERATOROPS-06 (Ubiquitous). `mission.launch.py` shall declare `corridor`, `dry_run` (false), `status_loop` (false), `start_patrol` (true); while `start_patrol` is `false` it shall not start the v1 `patrol_server`, and while `status_loop` is `true` it shall start `status_loop` with `period_s: 30.0` (`mission.launch.py` lines 18-30; README.md line 45).
Verified by: `steps/08_proofs.sh` block #12 line 87 (`dry_run:=true start_patrol:=false`), `steps/10_scene_proof.sh` line 12, `steps/11_live_model_proof.sh` line 15 (`status_loop:=true`).

REQ-OPERATOROPS-07 (State-driven). While `dry_run` is `true`, `mission_planner` shall not construct an Anthropic client and shall serve `/mission/tool_call` requests with `planner=None` (`mission_planner.py` lines 93, 125-129; `on_tool_call` lines 261-273 uses `self.tools` only).
Verified by: `ws/src/corridor_planner/test/test_mission_planner_node.py` line 94 (`dry_run:=true` node fixture) and `steps/08_proofs.sh` block #12 lines 82-99.

REQ-OPERATOROPS-08 (Ubiquitous). `env.sh` shall source the first existing of `~/.goske/ros2_jazzy/install/setup.bash` and `/opt/ros/jazzy/setup.bash`, then the workspace overlay `ws/install/setup.bash` if present, and shall default `ROS_DOMAIN_ID` to `42` without overriding a value already set (`env.sh` lines 2-7; commit d1a5526).
Verified by: NOT VERIFIED (every `steps/*.sh` sources it, none asserts the result).

REQ-OPERATOROPS-09 (Optional). Where `./.env` exists next to `env.sh`, `env.sh` shall source it with `set -a` so that its variables (`THOR_HOST`, `THOR_USER`, `ANTHROPIC_API_KEY`, `FTS_HOST`) are exported to launched nodes; where it is absent, sourcing `env.sh` shall return 0 (`env.sh` line 8; commits b63073d, a08b4b6; `.env.example`).
Verified by: NOT VERIFIED. [GAP] README.md line 36 says `. ./.env` before `env.sh`; `env.sh` line 8 already sources `.env` itself, the doc is stale.

REQ-OPERATOROPS-10 (Ubiquitous). The repository shall not track `.env`, `ws/build/`, `ws/install/`, `ws/log/` or any `ws/src/*` other than `corridor_*` packages (`.gitignore`; commits 284cb14, ed50cd1); `ANTHROPIC_API_KEY` shall reach the planner only through the process environment (`anthropic.Anthropic()` with no key argument, `mission_planner.py` line 127; CLAUDE.md line 19).
Verified by: NOT VERIFIED (no test greps the tree for keys).

REQ-OPERATOROPS-11 (Unwanted behaviour). If `ANTHROPIC_API_KEY` is unset when `steps/11_live_model_proof.sh` runs, then the script shall print `ANTHROPIC_API_KEY is not set; put it in .env` and exit 1 before launching anything (`steps/11_live_model_proof.sh` line 6).
Verified by: the script itself is the check; no automated test.

REQ-OPERATOROPS-12 (Ubiquitous). `cot_bridge` shall default its `host` parameter to `$FTS_HOST` when set and to `127.0.0.1` otherwise, with `port` 8087 (`cot_bridge.py` lines 28-29; README.md line 47).
Verified by: `ws/src/corridor_cot/test/test_cot_bridge_node.py` line 91 passes `host:=127.0.0.1` explicitly; the `FTS_HOST` fallback is NOT VERIFIED.

REQ-OPERATOROPS-13 (Ubiquitous). Each proof script `steps/08_proofs.sh`, `10_scene_proof.sh`, `11_live_model_proof.sh`, `12_gazebo_patrol.sh`, `13_cot_proof.sh`, `14_marker_proof.sh` shall `cd ~/deletdis/ros2-corridor-swarm`, source `env.sh` then `steps/lib.sh`, append its full stdout/stderr to `~/deletdis/log/<NN>_<name>.log` via `exec > >(tee -a "$LOG") 2>&1`, and print a verdict line of the form `PROOF <issue> PASS|FAIL` (variants: `PROOF 16 REVERSE PASS|FAIL` in 14, `GAZEBO PATROL PASS|FAIL` in 12, `RELAY PASS|FAIL` in 13) (`steps/08_proofs.sh` lines 4-7, `10` lines 3-6, `11` lines 3-7, `12` lines 4-7, `13` lines 5-8, `14` lines 3-6; CLAUDE.md line 20). `steps/09_ros2_control_build.sh` and `15_foxglove_msgs.sh` are build scripts, not proofs.
Verified by: the scripts themselves; NOT VERIFIED by any test.

REQ-OPERATOROPS-14 (Ubiquitous). `steps/lib.sh` shall export `CORRIDOR_PROOF=<pid of the proof script>` so every process a proof starts carries that marker, and `kill_ours` shall TERM then (after 3 s) KILL only (a) descendants of the calling script and (b) processes of this user whose `CORRIDOR_PROOF` equals the caller's own or names a script pid no longer in `/proc`, skipping `bash|sh|dash|zsh|tee|sshd|ssh|tmux*`; it shall never kill by executable name (`steps/lib.sh` lines 8-58; commits 40b4451, 8779697, a6540f9; CLAUDE.md line 7 "never stop ... processes ... you did not start").
Verified by: NOT VERIFIED (no test for lib.sh).

REQ-OPERATOROPS-15 (State-driven). While another proof script is still running (its pid exists in `/proc`), `kill_ours` from a different proof shall leave that proof's processes alone (`steps/lib.sh` lines 6-7, 25).
Verified by: NOT VERIFIED.

REQ-OPERATOROPS-16 (Unwanted behaviour). If no TCP listener is on port 8087 when `steps/13_cot_proof.sh` runs, then the script shall print `no TAK server on 8087`, `PROOF 16 FAIL` and exit 1 (`steps/13_cot_proof.sh` line 12); if `docker ps` is not permitted it shall read the `corridor-fts` log through `sudo -n docker logs` (line 18; commit eca1a62).
Verified by: the script itself; no automated test.

REQ-OPERATOROPS-17 (Ubiquitous). `steps/04_run.sh` shall launch `day1.launch.py` in a detached tmux window `ros2:run` with the launch output teed to `~/deletdis/log/day1_launch.log`, replacing any previous `run` window (`steps/04_run.sh` lines 22-24; README.md line 28; CLAUDE.md line 20).
Verified by: `steps/04_run.sh` lines 26-31 (tail of the log, topic list, hz, port 8765). Note: only step 04 uses tmux; steps 08-14 run in the foreground of whatever shell invokes them.

REQ-OPERATOROPS-18 (Unwanted behaviour). If a ROS node test is run with `ROS_DOMAIN_ID=42` (the demo domain set by `env.sh`), then the test module shall exit/fail with a message directing the operator to `ROS_DOMAIN_ID=77`, so tests never publish on the live operator topics (`corridor_nav/test/conftest.py` lines 43-44, `corridor_planner/test/test_mission_planner_node.py` lines 21-22, `corridor_cot/test/test_cot_bridge_node.py` lines 21-22; commits facc91f, 4bab2b5, 1180855).
Verified by: the guards are at module/fixture scope in those three files; no test of the guard itself.

REQ-OPERATOROPS-19 (Ubiquitous). `foxglove/corridor.json` shall provide panels `map!fleet` (`/v1/fix`, `/v2/fix`, `/v3/fix`, `/map/overlay`), `3D!scene` (`/v1/scan`, `/map/markers`, `/v1/camera/image`; follows tf `v1/base_footprint`; publishes `/clicked_point`, `/goal_pose`, `/initialpose`), `Image!cam` (`/v1/camera/image`), `Plot!xte` (`/v1/follower/cross_track.data`, `/v1/odom.twist.twist.linear.x`), `RawMessages!status` (`/v1/follower/status`), `RawMessages!geofence` (`/fleet/geofence_events`), `Publish!request` (`/mission/request`), `RawMessages!reply` (`/mission/reply`), `RawMessages!trace` (`/mission/llm_trace`), `Teleop!v1` (`/v1/cmd_vel`) (`foxglove/corridor.json`; commits 03ffd78, 7de4a0d, d193ae4; README.md line 51).
Verified by: NOT VERIFIED (layout is imported by hand on the Mac).

REQ-OPERATOROPS-20 (Ubiquitous). The `map_overlay` node shall publish the corridor polygon, per-vehicle inset routes and heading lines as `foxglove_msgs/GeoJSON` on `/map/overlay` and `visualization_msgs/MarkerArray` on `/map/markers` at 2 Hz, with `foxglove_msgs` built by `steps/15_foxglove_msgs.sh` from `ws/src/foxglove-sdk/ros/src/foxglove_msgs` only (`map_overlay.py` lines 1-41; `corridor_nav/setup.py` line 31; `steps/15_foxglove_msgs.sh`).
Verified by: `steps/15_foxglove_msgs.sh` line 13 (`ros2 interface show foxglove_msgs/msg/GeoJSON`) for the build; the node is NOT VERIFIED. [GAP] `map_overlay` is in no launch file and not in the README launch/tools table (only `setup.py` line 31 names it); the layout's `/map/overlay` and `/map/markers` panels (REQ-19) are empty unless it is started by hand.

REQ-OPERATOROPS-21 [PLANNED] (Ubiquitous). The project shall document every launch argument and convert the mission-layer nodes to lifecycle nodes with a QoS review (PLAN.md line 70, Phase 5 step 17). README.md lines 38-47 already carry the argument table; lifecycle nodes and QoS review are not in the code.
Verified by: NOT VERIFIED.

Count: 21 requirements (3 [GAP] notes on 05, 09, 20; 1 [PLANNED]).

Sources: `ws/src/corridor_nav/launch/kinematic.launch.py`, `ws/src/corridor_gz/launch/sim.launch.py`, `ws/src/corridor_sim/launch/day1.launch.py`, `ws/src/corridor_planner/launch/mission.launch.py`, `env.sh`, `.gitignore`, `.env.example`, `steps/lib.sh`, `steps/04_run.sh`, `steps/08_proofs.sh`, `steps/10_scene_proof.sh`, `steps/11_live_model_proof.sh`, `steps/12_gazebo_patrol.sh`, `steps/13_cot_proof.sh`, `steps/14_marker_proof.sh`, `steps/15_foxglove_msgs.sh`, `foxglove/corridor.json`, `ws/src/corridor_planner/corridor_planner/mission_planner.py`, `ws/src/corridor_cot/corridor_cot/cot_bridge.py`, `ws/src/corridor_nav/corridor_nav/map_overlay.py`, `ws/src/corridor_nav/setup.py`, `ws/src/corridor_nav/test/conftest.py`, `ws/src/corridor_planner/test/test_mission_planner_node.py`, `ws/src/corridor_cot/test/test_cot_bridge_node.py`, README.md, ARCHITECTURE.md, PLAN.md, CLAUDE.md; commits d1a5526, b63073d, a08b4b6, 284cb14, ed50cd1, 03ffd78, 7de4a0d, d193ae4, facc91f, 4bab2b5, 1180855, 40b4451, 8779697, a6540f9, eca1a62.
