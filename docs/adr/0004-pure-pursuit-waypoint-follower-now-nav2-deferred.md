# ADR-0004: Pure-pursuit waypoint follower now, Nav2 deferred

Status: Accepted (date: 2026-10-24, commits ce1ae24 / 5773e99; follower fixes 2026-10-25, #19)

## Context
ROS 2 Jazzy on Thor (aarch64) is a source build at `~/.goske/ros2_jazzy`: 350 packages, no ros_gz, no Nav2, no ros2_control (CLAUDE.md:8). Getting physics in already cost source builds of ros_gz (17 vendor packages, DART and OGRE-Next from source) and ros2_control (42 packages) (ARCHITECTURE.md:102). Nav2 would add ~40 more source packages to the same underlay (ARCHITECTURE.md:43); PLAN.md:75,79 names Phase 1 build time, not API credit, as the real constraint. The target is a corridor with few obstacles and a patrol route along a polygon; the goal of the first build is motors and sensors behaving like hardware, not a planner stack (ARCHITECTURE.md:87). Phase 2 step 7 asks only for "path in, `cmd_vel` out, arrival tolerance parameter" (PLAN.md:37).

## Decision
Per-vehicle `waypoint_follower` (`ws/src/corridor_nav/corridor_nav/waypoint_follower.py`, 139 lines) runs the pure-pursuit controller in `ws/src/corridor_nav/corridor_nav/pure_pursuit.py` (97 lines, no ROS imports; `step(x, y, yaw, path, speed, lookahead, arrive_tolerance, max_angular=1.5, min_segment=0, window=2)`, pure_pursuit.py:78). Node contract in namespace `/vN`: subscribes `fix` (NavSatFix), `imu` (Imu), `path` (nav_msgs/Path); publishes `cmd_vel` (Twist), `follower/status` (String, latched), `follower/route` (Header, latched; stamp = route id, frame_id = status), `follower/cross_track` (Float32) (waypoint_follower.py:58-66). Parameters: `speed_mps` 2.0, `lookahead_m` 4.0, `arrive_tolerance_m` 1.0, `rate_hz` 20.0, `pose_timeout_s` 1.0, `corridor_geojson`, `heading_offset_rad` 0.0 (waypoint_follower.py:39-45). Position comes from `fix` through the shared `LocalFrame`, heading from `imu`; wheel odometry is not used for navigation (#19, ARCHITECTURE.md:90). Nav2 is kept out of the underlay; it may replace `waypoint_follower` later behind the same `path`/`cmd_vel` contract without touching the mission layer (ARCHITECTURE.md:43). Obstacle avoidance, if wanted, is a small lidar-based local planner inside `corridor_nav`, not Nav2 (ARCHITECTURE.md:102).

## Alternatives considered
- **Nav2 from the start.** Rejected: ~40 extra source packages on an underlay that already needed ros_gz and ros2_control from source; cost confirmed by those builds (ARCHITECTURE.md:43,102; open question 5).
- **Static `map -> odom` tf / odom started at the spawn pose** for the follower's position input. Rejected in #19 in favour of fix + IMU: odom yaw read 119° vs IMU 7° after wheel slip against a crate (ARCHITECTURE.md:90).
- **Nav2 from apt binaries.** Ruled out by CLAUDE.md:8: no apt ROS in /opt/ros, it would shadow the source build.

## Consequences
- Good: stack stays buildable; the controller is pure Python with unit tests (`ws/src/corridor_nav/test/test_pure_pursuit.py`, `test_waypoint_follower.py`, `test_waypoint_follower_node.py`); one lap on Gazebo + ros2_control is the proof `steps/12_gazebo_patrol.sh` (PASS when the patrol result is `done` and distance > 380 m, steps/12_gazebo_patrol.sh:3,23; cross-track max is logged, line 21; the proof runs the follower with `speed_mps` 3.0 and `arrive_tolerance_m` 2.0, line 14).
- Good: the mission layer reaches the follower only through `/vN/path`, `follower/route` and `follower/status`: `patrol_server` publishes `path` and reads `follower/route` (patrol_server.py:162,164); `geofence_monitor` publishes an empty Path (ARCHITECTURE.md:99); `mission_planner` and `status_loop` read `follower/status` (mission_planner.py:108, status_loop.py:39). A later swap is contained to that contract.
- Bad: no obstacle avoidance, no costmaps, no recovery behaviours; lidar is bridged (PLAN.md:26) but nothing in `corridor_nav` reads `scan`.
- Bad: closed-route edge cases had to be found by hand on Gazebo (start taken for the closing leg; off-path fallback aiming at route end), fixed in bf34a17, 2c195ad, c8cd2de (PLAN.md:11).
- Bad: several nodes publish `/vN/path`, which forced the route-id echo on `follower/route` and preemption rules (ARCHITECTURE.md:91).

## Evidence
- ce1ae24 (2026-10-24) ARCHITECTURE "Physics first, Nav2 later"; 5773e99 (2026-10-24) open question 5 "Nav2 stays out".
- fe225ae (2026-10-24) `pure_pursuit.step`; 87a2d3a (2026-10-24) `waypoint_follower` node; afb8630 (2026-10-24, #17) latched status; 3861f82, d851be5 (2026-10-25, #19) position from fix, heading from imu; bf34a17, 2c195ad, c8cd2de (2026-10-25, #19) closed-route fixes; 9bada45 (2026-10-26, nav-mission-2) `follower/route`.
- Proof: `steps/12_gazebo_patrol.sh`; PLAN.md:11 (2026-10-25) names it as the #19 proof.
- Issues: #17 (status QoS), #19 (fix + IMU navigation).

## Sources
- CLAUDE.md:8; PLAN.md:11,26,37,75,79; ARCHITECTURE.md:43,87,90,91,99,102
- ws/src/corridor_nav/corridor_nav/waypoint_follower.py (139 lines; :39-45, :58-66); ws/src/corridor_nav/corridor_nav/pure_pursuit.py (97 lines; :78)
- ws/src/corridor_nav/corridor_nav/patrol_server.py:162,164; ws/src/corridor_planner/corridor_planner/mission_planner.py:108; ws/src/corridor_planner/corridor_planner/status_loop.py:39
- ws/src/corridor_nav/test/test_pure_pursuit.py, test_waypoint_follower.py, test_waypoint_follower_node.py
- steps/12_gazebo_patrol.sh:3,14,21,23
- git log: ce1ae24, 5773e99, fe225ae, 87a2d3a, afb8630, 3861f82, d851be5, bf34a17, 2c195ad, c8cd2de, 9bada45
