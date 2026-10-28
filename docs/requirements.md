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

