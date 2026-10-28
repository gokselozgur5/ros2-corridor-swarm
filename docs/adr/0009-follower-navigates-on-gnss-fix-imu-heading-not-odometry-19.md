# ADR-0009: Follower navigates on GNSS fix + IMU heading, not odometry (#19)

Status: Accepted (date: 2026-10-25, ARCHITECTURE.md decision at line 90; commits 207c2e8, 3861f82, d851be5, e26d317)

## Context
The patrol route lives in the corridor frame: `LocalFrame` (ENU metres around an origin lat/lon, `geofence.py` lines 3-15), with the origin set to the corridor polygon's first vertex by the nodes that build it (`waypoint_follower.py` lines 47-48). `/vN/odom` starts wherever the vehicle spawned: on Gazebo the spawn is map (20,20) and odom starts at (0,0) (ARCHITECTURE.md line 91; commit f37bdb2 body). `steps/12_gazebo_patrol.sh` exposed this on 2026-10-24 (PLAN.md line 9). Three odometry sources exist (`unicycle_sim`, gz `OdometryPublisher`, `diff_drive`; ARCHITECTURE.md open question 1, line 98). Odom yaw is also untrustworthy: on the skid steer the `diff_drive` odom yaw read 119 deg while the IMU read 7 deg once the wheels spun against a crate, and the follower drove blind (commit d851be5 body). A real vehicle has a GNSS fix and an IMU; it does not have a map-referenced odometry (ARCHITECTURE.md line 90).

## Decision
`waypoint_follower` (package `corridor_nav`, runs in the vehicle namespace) positions itself from `fix` (sensor_msgs/NavSatFix) converted through the same `LocalFrame` as `patrol_server` and `geofence_monitor`, built from the `corridor_geojson` parameter (`waypoint_follower.py` lines 44, 47-48, 64). Heading comes from `imu` (sensor_msgs/Imu) via `yaw_from_quaternion(msg.orientation)` (lines 27-28, 65, 74-76), plus `heading_offset_rad` (default 0.0) for IMUs not referenced to ENU (line 45). Pure function `local_pose.pose_from_fix(frame, lat, lon, heading, heading_offset=0.0)` returns `(x, y, yaw)` in the frame (`local_pose.py` lines 14-18). The follower does not subscribe to odom at all (subscriptions: lines 64-66). `odom_timeout_s` became `pose_timeout_s` (default 1.0 s; commit 3861f82 body, line 43): the follower does not drive until both fix and imu have arrived (lines 99-102) and publishes a zero Twist while either is stale (lines 103-108; commit 10f388b). Contract is the same on all three backends: `unicycle_sim` publishes `imu` with orientation (commit a171df1; `unicycle_sim.py` lines 3, 43, 87-93), the Gazebo IMU sensor is ENU-referenced (`ugv.gazebo.xacro` line 54, commit e1dfeca). Follow-up 2026-10-26: `patrol_server` (commit f37bdb2), the planner's `goto` (commit 0a25ceb) and `clicked_goal` (commit 7dfa1db) position by fix too (ARCHITECTURE.md line 91); `status_loop` takes speed only from odom (commit f96ec33).

## Alternatives considered
- Static `map -> odom` tf per spawn pose: correct ROS, more plumbing per backend; rejected (ARCHITECTURE.md line 90).
- Start odom at the spawn pose: odom pretending to be map; rejected (ARCHITECTURE.md line 90). Neither fixes the slipping-wheel yaw.
- Fuse fix+imu+odom with an EKF (robot_localization): not evaluated; not mentioned anywhere in the repo. ARCHITECTURE.md keeps Nav2 out of the underlay (open question 5, line 102).

## Consequences
- Good: route and vehicle share one frame on kinematic, gz-plugin and ros2_control backends; which node publishes odom no longer matters for navigation (ARCHITECTURE.md line 98). Heading survives wheel slip (commit d851be5). The contract matches a real platform (ARCHITECTURE.md line 90).
- Good: `pose_from_fix` has no ROS dependency and is unit-tested (`test/test_local_pose.py`, 2 tests).
- Bad: follower depends on two live inputs. Between 3861f82 and 10f388b only the fix was stamped, so a stopped imu stream drove pure pursuit on a frozen yaw; since 10f388b a stale `imu` holds the vehicle with a zero Twist (commit 10f388b body; lines 103-108).
- Bad: GNSS-only position inherits the fix's noise and rate; no fusion or smoothing exists. `heading_offset_rad` must be set per platform whose IMU is not ENU-referenced (line 45).
- Bad: `/vN/odom` is still published by three backends; the only remaining consumer in the mission layer is `status_loop` for speed (commit f96ec33). ARCHITECTURE.md line 98 still says "odom is heading and speed only" and `local_pose.py` lines 5, 15 still say the heading is the odometry yaw; both predate d851be5. The launch split (open question 1) remains to do.

## Evidence
- Commits: 207c2e8 (local_pose.pose_from_fix), 3861f82 (position from fix), d851be5 (heading from imu), a171df1 (unicycle_sim imu), e1dfeca (ENU imu in xacro), e26d317 (ARCHITECTURE), 550603c (PLAN note), 10f388b (imu-stale hold), f37bdb2 (patrol_server by fix), 0a25ceb (goto by fix), 7dfa1db (clicked_goal by fix), f96ec33 (status_loop speed from odom only). All dated 2026-10-25 except the last five (2026-10-26); all reachable from HEAD (60633c8).
- Files: `ws/src/corridor_nav/corridor_nav/local_pose.py` (lines 14-18), `ws/src/corridor_nav/corridor_nav/waypoint_follower.py` (lines 3-5, 27-28, 43-48, 64-76, 99-109), `ws/src/corridor_nav/corridor_nav/geofence.py` (lines 3, 11-15), `ws/src/corridor_description/urdf/ugv.gazebo.xacro` line 54, `ws/src/corridor_sim/corridor_sim/unicycle_sim.py` lines 3, 43, 87-93, `ws/src/corridor_nav/test/test_local_pose.py` (2 tests), `ws/src/corridor_nav/corridor_nav/patrol_server.py` lines 4-5, 163.
- Docs: ARCHITECTURE.md lines 90, 91, 98, 102; PLAN.md lines 9, 11.
- Proof: `steps/12_gazebo_patrol.sh` (PLAN.md line 11, "all four fixed with tests seen red first").
- Issue: #19.

## Sources
- ARCHITECTURE.md:90,91,98,102
- PLAN.md:9,11
- ws/src/corridor_nav/corridor_nav/waypoint_follower.py
- ws/src/corridor_nav/corridor_nav/local_pose.py
- ws/src/corridor_nav/corridor_nav/geofence.py
- ws/src/corridor_nav/corridor_nav/patrol_server.py
- ws/src/corridor_nav/test/test_local_pose.py
- ws/src/corridor_sim/corridor_sim/unicycle_sim.py
- ws/src/corridor_description/urdf/ugv.gazebo.xacro:54
- steps/12_gazebo_patrol.sh
- git log: 207c2e8, 3861f82, d851be5, e26d317, a171df1, e1dfeca, 550603c, 10f388b, f37bdb2, 0a25ceb, 7dfa1db, f96ec33, 99e5dab
