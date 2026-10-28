# ADR-0017: goto becomes an action like patrol

Status: Proposed (date: 2026-10-28; question raised in ARCHITECTURE.md open question 4, 2026-10-24, commit 5773e99). Note: the repo has no ADR directory or earlier ADRs; the number 0017 is not traceable to anything in the repo.

## Context

ARCHITECTURE.md:67 states "The planner never publishes `cmd_vel`. It only sends action goals, which the geofence can veto", and open question 4 (ARCHITECTURE.md:101) restates the rule as "only action goals, never low-level topics". Yet `goto` publishes a two-pose `nav_msgs/Path` (fix, target) straight onto `/vN/path` (mission_planner.py:181-205, ARCHITECTURE.md:58). `patrol` goes through the `Patrol` action (Patrol.action, patrol_server.py; client at mission_planner.py:111), so the planner holds a goal handle for patrol (mission_planner.py:177) and nothing for goto: after `t_goto` sets `v.mode = 'goto'` (mission_planner.py:204), no result ever comes back and nothing resets `mode` (it is only written at mission_planner.py:178, 204, 215, 236); the planner only sees the follower's free-text status on `/vN/follower/status` (mission_planner.py:108). Besides patrol_server itself, six other nodes write `/vN/path` (planner goto and stop, geofence_monitor, clicked_goal, wasd, send_path; patrol_server.py:11-12; wasd.py and send_path.py in corridor_nav), which is why patrol_server needed route stamps and `follower/route` to tell its own Path from a foreign one (commits 1ab2dae, 377da6a; waypoint_follower.py:62). A geofence EXIT ends a patrol goal as `geofence` (patrol_server.py:131, 180-181), but a goto is stopped only by an empty Path: the monitor's when `cancel_on_exit` is true (default, geofence_monitor.py:22, 77-78) or the planner's own `t_stop` (mission_planner.py:225-235); neither produces a result the planner can read. `clicked_goal` duplicates the goto Path construction (clicked_goal.py:37-46 vs mission_planner.py:194-203) because there is no server to call.

## Decision (proposed, not implemented)

Add `corridor_msgs/action/Goto.action` next to `Patrol.action`: goal `vehicle, corridor_id, latitude, longitude, speed_mps`; result `distance_m, result` with the same vocabulary as Patrol (`done | cancelled | geofence | preempted | error`, Patrol.action:11); feedback `distance_remaining_m, next_waypoint`. Serve it from `patrol_server` (rename to `mission_server`, or a sibling `goto_server` in the same vehicle namespace) so one node owns the follower's route per vehicle: start vertex from `/vN/fix` through the corridor `LocalFrame`, in-corridor check with `signed_distance` (as mission_planner.py:186 does today), unique odd route stamp, arrival logic reused from the patrol path. `mission_planner.t_goto` becomes a mirror of `t_patrol` (mission_planner.py:164-179): `_cancel_tracked`, `send_goal_async`, keep the handle, `mode = 'goto'`; it publishes no Path. `clicked_goal` becomes an action client of the same server instead of a Path publisher. `t_stop` keeps its empty Path only for the no-goal case (today it publishes one in both branches, mission_planner.py:225-235).

## Alternatives considered

- Keep goto as a Path and read `follower/route` in the planner to detect arrival (not evaluated): leaves the planner writing a low-level topic and keeps the stamp-ownership race the patrol_server already had to solve (1ab2dae, 377da6a).
- Fold goto into `Patrol` as a 1-vertex, 1-lap goal: avoids a new .action but abuses `laps`/`clockwise` and makes the trace unreadable; rejected on clarity.
- Nav2 `NavigateToPose`: ruled out by open question 5 (ARCHITECTURE.md:102); the underlay cannot absorb Nav2 cheaply.

## Consequences

Good: the planner obeys its own rule; goto gets a result, so `mode` stops being stale after arrival or a geofence breach; a breach can cancel the action at the mission layer, which is the direction open question 2 (ARCHITECTURE.md:99) already wants; one fewer writer on `/vN/path`; `clicked_goal` loses ~10 duplicated lines. Bad: new interface in `corridor_msgs` and a rebuild (`steps/07_build_ours.sh`); the planner-3 goto tests (test_mission_tools.py:89-200) and `test_patrol_and_goto_publish_no_empty_path_of_their_own` (test_mission_tools.py:206-222) must be rewritten around a fake Goto server; `clicked_goal` gains a dependency on a running server, so a click with no server is dropped unless handled (today it only needs a fix, clicked_goal.py:34-36); launch files must start the server for every vehicle (mission.launch.py:23 starts `patrol_server` for v1 only, behind `start_patrol`; kinematic.launch.py:29 per vehicle).

## Evidence

- ARCHITECTURE.md:58, :67, :99, :101 (open question 4), :102.
- ws/src/corridor_planner/corridor_planner/mission_planner.py:5, :108, :111, :164-205, :207-237.
- ws/src/corridor_planner/corridor_planner/tools.py:14-17.
- ws/src/corridor_nav/corridor_nav/patrol_server.py:1-19, :131, :180-181; clicked_goal.py:1-47; geofence_monitor.py:22, :77-78; waypoint_follower.py:62; wasd.py, send_path.py (exist).
- ws/src/corridor_msgs/action/Patrol.action (created 830aae2 2026-10-24, last changed 70bfce4 2026-10-26).
- Commits 5773e99 (2026-10-24, open questions), 99e5dab (goto Path documented), 0a25ceb / e5ea084 / e1c3442 (goto start vertex and cancel semantics), 1ab2dae / 377da6a (route stamps), b43b989 (one goal at a time), all 2026-10-26.
- Tests: ws/src/corridor_planner/test/test_mission_tools.py:56-67, :89-200, :206-222; ws/src/corridor_nav/test/test_patrol_server.py:239-270.
- Launch: ws/src/corridor_planner/launch/mission.launch.py:23; ws/src/corridor_nav/launch/kinematic.launch.py:29.
- No proof script covers goto end-to-end (no `goto` in steps/*); `steps/12_gazebo_patrol.sh` covers patrol only (README.md:47).

## Sources

ARCHITECTURE.md; ws/src/corridor_planner/corridor_planner/{mission_planner.py,tools.py}; ws/src/corridor_nav/corridor_nav/{patrol_server.py,clicked_goal.py,geofence_monitor.py,waypoint_follower.py,wasd.py,send_path.py}; ws/src/corridor_msgs/action/Patrol.action; ws/src/corridor_planner/test/test_mission_tools.py; ws/src/corridor_nav/test/test_patrol_server.py; ws/src/corridor_planner/launch/mission.launch.py; ws/src/corridor_nav/launch/kinematic.launch.py; steps/; README.md:47; git log (5773e99, 830aae2, 70bfce4, 99e5dab, 0a25ceb, e5ea084, e1c3442, 1ab2dae, 377da6a, b43b989).
