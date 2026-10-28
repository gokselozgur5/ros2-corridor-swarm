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

