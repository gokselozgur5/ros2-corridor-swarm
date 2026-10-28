# ADR-0007: Keep the kinematic unicycle_sim as the no-Gazebo backend; same topic contract on all backends

Status: Accepted (2026-10-24, 558c472 unicycle_sim + 073c895 kinematic.launch.py; contract completed 2026-10-25, a171df1 / 3861f82 / d851be5)

## Context
Gazebo on Thor is a source build: ros_gz, ros2_control, ros2_controllers, gz_ros2_control built with colcon (ARCHITECTURE.md:81); the ros_gz and ros2_control builds amount to 17 vendor packages, DART and OGRE-Next from source and 42 control packages (ARCHITECTURE.md:102, open question 5). Phase 1 is the long pole (PLAN.md:75). Phase 2 (#6-#10) and the Phase 3 tool proof (#12) were proven on the kinematic sim, not on physics (README.md:32; steps/08_proofs.sh:22,38,56,68,111). Three odometry sources exist, switched with xacro `if/unless` and launch conditions (ARCHITECTURE.md:98; ugv.gazebo.xacro:65-83; sim.launch.py:43-49). The follower was redone on #19 to navigate on `fix` + `imu`, so every backend must supply both (ARCHITECTURE.md:90).

## Decision
- `corridor_sim/unicycle_sim` (ws/src/corridor_sim/corridor_sim/unicycle_sim.py, added 558c472 "for testing autonomy without Gazebo") is the no-Gazebo backend. It is a cmd_vel-driven kinematic unicycle: subscribes `cmd_vel` (Twist), integrates x/y/yaw at `rate_hz` (50), zeroes the command after `cmd_timeout_s` (0.5), publishes `odom`, `fix` (origin `origin_lat`/`origin_lon`, equirectangular), `imu` (orientation + angular_velocity.z only) and the `frame_id -> child_frame_id` transform (unicycle_sim.py:21-103).
- The vehicle-layer contract is the same per `/vN` namespace on every backend: `cmd_vel` in; `odom`, `fix`, `imu` out. Gazebo supplies it via `corridor_gz/config/bridge_v1.yaml` (`/v1/fix` <- gz `/v1/navsat`, `/v1/imu`, `/v1/odom`, `/v1/cmd_vel` ROS_TO_GZ); `unicycle_sim` publishes it natively. Transforms differ: the bridge maps gz `/v1/tf` to `/v1/tf` (bridge_v1.yaml:37-41), `unicycle_sim` broadcasts on `/tf` (unicycle_sim.py:44,103). `waypoint_follower` consumes `fix` and `imu`, publishes `cmd_vel` (waypoint_follower.py:58,64-65) and does not read tf or odom.
- `corridor_nav/launch/kinematic.launch.py` (073c895) is the kinematic backend's launch: per vehicle `unicycle_sim` + `waypoint_follower` + `patrol_server`, one `geofence_monitor`; args `corridor`, `n`, `first`, `lookahead_m`, `speed_mps`, `foxglove`. `first:=2 n:=2` runs kinematic vehicles beside a Gazebo v1 (kinematic.launch.py:42).
- Day-1 `vehicle_sim` (fixed looping lat/lon route, no cmd_vel; vehicle_sim.py:1-4,23) stays in the package as the smoke test (setup.py:23, day1.launch.py:13, ARCHITECTURE.md:73,89). ARCHITECTURE.md:73,89 still name `vehicle_sim`, not `unicycle_sim`, as the no-Gazebo fallback; the table has not been updated for this ADR.

## Alternatives considered
- Gazebo only: rejected; Phase 2 and 3 code is written and tested against the kinematic sim while Phase 1 builds (PLAN.md:75, README.md:32).
- Different topic contract per backend (follower reads odom on kinematic, fix/imu on Gazebo): rejected by #19; odom yaw read 119° while the IMU read 7° on the skid steer, and one contract is what a real vehicle has (ARCHITECTURE.md:90).

## Consequences
- Good: `steps/08_proofs.sh` proves #7, #8, #9, #10 and #12 on `kinematic.launch.py` with no Gazebo; a three-vehicle fleet is `n:=3` (08_proofs.sh:68); Phase 3 develops against the kinematic sim, Gazebo is needed only for step 13 (PLAN.md:75).
- Good: a node proven on kinematic moves to Gazebo unchanged; steps/12_gazebo_patrol.sh:14-15 runs the same `waypoint_follower` and `patrol_server` executables against `sim.launch.py use_ros2_control:=true`.
- Bad: kinematic motion hides physics faults. Chasing the Gazebo lap exposed #19 (odom frame vs map frame), two pure-pursuit faults on closed routes (start taken for the closing leg; off-path fallback aiming at the route end) and the skid steer's lateral grip (PLAN.md:11, ARCHITECTURE.md:92). Kinematic proofs are necessary, not sufficient.
- Bad: `unicycle_sim` publishes no `scan`/`camera`/`joint_states` (unicycle_sim.py:41-44); anything using them is Gazebo-only (bridge_v1.yaml:7-21,42-46).
- Open: one launch file per backend (kinematic, gz-plugin, ros2_control) with no conditionals inside the model is "still to do" (ARCHITECTURE.md:98). Today the kinematic backend has `kinematic.launch.py`; both Gazebo backends share `corridor_gz/launch/sim.launch.py` behind `use_ros2_control` (sim.launch.py:29,43-49) and the xacro `if/unless` (ugv.gazebo.xacro:65-83).

## Evidence
- ce1ae24 (2026-10-24) ARCHITECTURE.md:89 "Keep `vehicle_sim`"; :73 corridor_sim row.
- 558c472 (2026-10-24) unicycle_sim added; 073c895 (2026-10-24) kinematic.launch.py; a171df1 (2026-10-25) unicycle_sim publishes `imu`; 3861f82, d851be5 (2026-10-25) follower positions by fix and heads by imu (#19); 0715c46, e26d317 (2026-10-25) ARCHITECTURE decision text, open question 1 partly settled.
- ws/src/corridor_sim/corridor_sim/unicycle_sim.py; ws/src/corridor_sim/corridor_sim/vehicle_sim.py; ws/src/corridor_sim/setup.py:23-24; ws/src/corridor_sim/launch/day1.launch.py:13; ws/src/corridor_nav/launch/kinematic.launch.py; ws/src/corridor_gz/launch/sim.launch.py; ws/src/corridor_gz/config/bridge_v1.yaml; ws/src/corridor_description/urdf/ugv.gazebo.xacro:65-83; ws/src/corridor_nav/corridor_nav/waypoint_follower.py:58-65.
- ARCHITECTURE.md:73,81,89,90,92,98,102; PLAN.md:11,75; README.md:32.
- Proofs: steps/08_proofs.sh (#7, #8, #9, #10, #12 on kinematic.launch.py); steps/12_gazebo_patrol.sh (same nav nodes on Gazebo). Issues #6-#10, #12, #19.

Note: repo was read-only for this step; no commit was made. Commit the ADR as its own atomic commit when adding it to the repo.
