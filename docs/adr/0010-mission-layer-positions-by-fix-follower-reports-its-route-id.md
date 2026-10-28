# ADR-0010: Mission layer positions by fix; follower reports its route id; one patrol goal at a time; stop semantics
Status: Accepted (date: 2026-10-26, commits 7f40503, 321574b, 377da6a, 5ca5f1b, 8693435)

## Context
On the Gazebo backend the vehicle spawns at map (20,20) while odom starts at (0,0) and drifts (#19, ARCHITECTURE.md:90-91, 98; PLAN.md:9,11), so any node that positions a vehicle by odom plans from the wrong place. Several nodes publish on `/vN/path`: patrol_server, the planner's goto and stop, geofence_monitor, clicked_goal, wasd, send_path (patrol_server.py:11-12). Before the 2026-10-26 review the follower's `arrived` status could not be attributed to a sender, so a foreign Path's arrival could count as a lap, and a cancel could clear a Path another node had just sent (patrol_server.py:12-17; commit 321574b). The planner's own empty Path on a second DDS writer could land after the server's successor route and end the new goal as `preempted` with the vehicle idle (commit e1c3442 body; mission_planner.py:208-210; test_mission_tools.py:169).

## Decision
- `patrol_server`, the planner's `goto` and `clicked_goal` take the vehicle position from `/vN/fix` through the corridor `LocalFrame`, never odom; odom is heading/speed only (ARCHITECTURE.md:91,98; patrol_server.py:3-5, 163, 172-173; mission_planner.py:6, 68-73; clicked_goal.py:4-5, 34-40). `goto` without a fix is refused (mission_planner.py:189-191).
- `waypoint_follower` publishes `follower/route` (std_msgs/Header, latched) with every status: `stamp` = header stamp of the Path it holds (route id), `frame_id` = status (waypoint_follower.py:7-9, 56, 62, 81, 92-94).
- `patrol_server` runs one goal at a time under `goal_lock`: a new goal sets `preempted` on the old run, waits up to 5 s for its loop to exit, then executes (patrol_server.py:159, 195-207). Results: `done | cancelled | geofence | preempted | error` (patrol_server.py:18).
- Every Path the server publishes carries a unique, increasing, odd stamp (`next_stamp`, patrol_server.py:88-92, 218-232), so sim-time stamps from other nodes (even multiples of the clock step) never collide. A lap counts only on an `arrived` report carrying the server's own stamp; a foreign Path ends the goal as `preempted`; a geofence EXIT ends it as `geofence` (patrol_server.py:97-107, 123-136, 180-185; ARCHITECTURE.md:99).
- Cancel/abort clears the follower only while `RouteOwner.owns()` (the follower holds or is about to receive the server's route) and only for the current goal; the clear is published inside the cancel callback, before the client is answered (patrol_server.py:102-104, 209-216, 238-241, 293-296).
- Route reports and geofence events run in a `MutuallyExclusiveCallbackGroup`, in arrival order (patrol_server.py:161, 164-166; commit 5ca5f1b).
- Planner `stop(vehicle)`: request result, send cancel, publish its own empty Path immediately after the cancel is out, then wait bounded `STOP_TIMEOUT_S = 3.0` for the old goal's result; reply carries `cancelled_patrol` and `finished` (mission_planner.py:36, 47-65, 220-237). `patrol`/`goto` cancel the tracked goal without an empty Path of their own and leave mode `stopped` until a successor is accepted (mission_planner.py:207-218; commits e1c3442, 8693435). `goto` re-reads the fix after the stop (mission_planner.py:193).

## Alternatives considered
- Position by odom, as before #19: rejected, odom frame != map frame on Gazebo (PLAN.md:9,11; ARCHITECTURE.md:90).
- Single Path publisher at the mission layer, geofence_monitor cancels the action instead of writing `path`: named as the better design in ARCHITECTURE.md:99, not done; the route id is the compatible fix while many writers remain.

## Consequences
- Good: lap counting, preemption and clearing are attributable; the planner's cancel-then-patrol sequence leaves the new route in place (test_patrol_server.py:311); stop is bounded and reports timeouts (`cancel_timeout`/`result_timeout`, mission_planner.py:55, 61-65).
- Good: Gazebo lap proof passes on physics with `result: done` and > 380 m (steps/12_gazebo_patrol.sh:2-3,23; PASS recorded in merge commit 60633c8, thor .55, 2026-10-27).
- Bad: two paths to the same effect on a geofence breach remain (monitor's empty Path and the server's own event), ARCHITECTURE.md:99. Odd-stamp disambiguation assumes other nodes stamp with `now().to_msg()` under sim time (patrol_server.py:89-91; e.g. mission_planner.py:196). A new goal can block up to 5 s while the old loop exits (patrol_server.py:204-205). The follower's `follower/route` is a side channel every Path writer must understand (waypoint_follower.py:8-9).

## Evidence
- Commits (all 2026-10-26 unless noted): 7f40503 (ARCHITECTURE decision), 99e5dab, e1c3442, 321574b (nav-mission-1), 377da6a/5ca5f1b (nav-mission-2), 8693435, f96ec33, merge 60633c8 (2026-10-27).
- Files: ARCHITECTURE.md:90-91,98-99; PLAN.md:9,11; ws/src/corridor_nav/corridor_nav/patrol_server.py; ws/src/corridor_nav/corridor_nav/waypoint_follower.py; ws/src/corridor_nav/corridor_nav/clicked_goal.py; ws/src/corridor_planner/corridor_planner/mission_planner.py.
- Tests: ws/src/corridor_nav/test/test_patrol_server.py:191-342 (route owner, unique/odd stamps, foreign arrival, cancel clears only owned route, preempt without clearing, cancel-then-patrol sequence); ws/src/corridor_nav/test/test_waypoint_follower.py:94,103; ws/src/corridor_planner/test/test_mission_tools.py:35-223 (stop bounded, empty Path right after cancel, reports finished, goto starts from post-stop fix, patrol/goto publish no empty Path, rejected successor leaves `stopped`).
- Proof: steps/12_gazebo_patrol.sh. Issues: #19.

## Sources
- git log (2026-10-26/27): 7f40503, 99e5dab, e1c3442, 321574b, 377da6a, 5ca5f1b, 8693435, f96ec33, 60633c8
- ARCHITECTURE.md:90-91, 98-99
- PLAN.md:9, 11
- ws/src/corridor_nav/corridor_nav/patrol_server.py:1-19, 88-107, 123-136, 159-169, 172-185, 195-216, 218-241, 293-296
- ws/src/corridor_nav/corridor_nav/waypoint_follower.py:1-11, 56, 62, 78-94
- ws/src/corridor_nav/corridor_nav/clicked_goal.py:4-5, 27-40
- ws/src/corridor_planner/corridor_planner/mission_planner.py:1-8, 36, 47-73, 181-237
- ws/src/corridor_nav/test/test_patrol_server.py:191-342; ws/src/corridor_nav/test/test_waypoint_follower.py:94-103; ws/src/corridor_planner/test/test_mission_tools.py:35-223
- steps/12_gazebo_patrol.sh:2-3, 23
