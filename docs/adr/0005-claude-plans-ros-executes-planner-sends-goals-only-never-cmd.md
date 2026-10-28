# ADR-0005: Claude plans, ROS executes: planner sends goals only, never cmd_vel

Status: Accepted (date: 2026-10-24, commit ce1ae24 "docs: architecture, phased plan with proofs, README and CLAUDE.md pointed at them"; code landed same day in 3a0eeff / 780c6e6)

## Context

The fleet is driven at tick rate: `waypoint_follower` runs a 20 Hz timer (`rate_hz` param) and is the only node that publishes `cmd_vel` per vehicle (ws/src/corridor_nav/corridor_nav/waypoint_follower.py:42,58,67); `diff_drive_controller` turns it into wheel joint velocities inside Gazebo (ARCHITECTURE.md:43). A model call can end in a `refusal` or an SDK error (ARCHITECTURE.md:65; mission_planner.py:7). `geofence_monitor` must be able to stop a vehicle regardless of what the planner wanted (geofence_monitor.py:22,73-77). The phase-3 plan therefore fixed the proof up front: "the planner never publishes `cmd_vel`" (PLAN.md:51).

## Decision

`mission_planner` (package `corridor_planner`) is "an operator with a radio, not a controller in the loop. Tick-rate decisions stay in ROS" (ARCHITECTURE.md:88). Concretely:

- Its tool set is exactly `list_vehicles`, `get_vehicle`, `patrol`, `goto`, `stop`, `geofence_events`, `describe_scene` (tools.py:3-29; mission_planner.py:121-123). No tool names or describes `cmd_vel`.
- `patrol` sends a `corridor_msgs/Patrol` action goal to `/<v>/patrol`; `goto` publishes a two-pose `nav_msgs/Path` on `/<v>/path` (fix -> target); `stop` cancels the tracked goal and publishes an empty `Path` (mission_planner.py:1-7,110-111,203,218-227).
- The node's publishers are `/<v>/path`, `/mission/reply`, `/mission/llm_trace`, `/diagnostics` (mission_planner.py:110,115-117). There is no `Twist` publisher.
- `stop_reason` is checked before `content`; a `refusal` becomes a `/diagnostics` entry, not a crash (ARCHITECTURE.md:65; mission_planner.py:7).
- Veto path: on EXIT, `geofence_monitor` with `cancel_on_exit: true` publishes an empty `Path` on the vehicle's `path`, which the follower honours independent of the planner (geofence_monitor.py:22,76-77). The server treats any foreign Path as a preempt (patrol_server.py:8-15).

Note: ARCHITECTURE.md:67 says "only sends action goals"; the code also publishes `Path` for goto/stop (ARCHITECTURE.md:58, commit 99e5dab). Both are setpoints consumed by ROS nodes, not velocity commands; the invariant that holds is "no `cmd_vel`".

## Alternatives considered

- LLM in the control loop (planner emits `cmd_vel`): rejected; the geofence would have to fight the planner at tick rate (ARCHITECTURE.md:88, PLAN.md:51).
- Planner as Nav2 behavior-tree client instead of `patrol_server` + Path: Nav2 deliberately out of the first build; it "can replace `waypoint_follower` later without touching the mission layer" (ARCHITECTURE.md:43,87,102). Not evaluated further.
- Prompt-level rule ("do not drive the vehicle") only: not evaluated; ARCHITECTURE.md:63 states the rules are enforced by the node, not the prompt.

## Consequences

- Good: a refusal, fallback, or SDK error leaves the vehicle under the follower/geofence; worst case is "no new goal". The geofence veto works without planner cooperation.
- Good: the invariant is testable offline with a stub client; `test_request_shape_has_fallbacks_and_no_cmd_vel_tool` fails if any tool mentions `cmd_vel` (test_planner.py:87-93).
- Good: everything the planner did is on `/mission/llm_trace` for Foxglove (ARCHITECTURE.md:66; mission_planner.py:116).
- Bad: the planner cannot do fine manoeuvres (back up, nudge); it only has patrol / goto-one-point / stop. New behaviours require a new ROS action or Path producer, not a prompt change.
- Bad: `path` is a multi-writer topic (planner goto/stop, patrol_server, geofence_monitor, clicked_goal, wasd, send_path; patrol_server.py:8-9); ownership had to be resolved by route stamps on the server/follower side (patrol_server.py:8-15; commits 9bada45, 1ab2dae, 377da6a) and the "no extra `Path()` on cancel" rule on the planner side (mission_planner.py:206-209; commits e1c3442, 8693435).
- Bad: the test only checks the tool schemas, not that the node has no `Twist` publisher; that is enforced by code review (mission_planner.py:110-117), not a test.

## Evidence

- Commits: ce1ae24 (ARCHITECTURE/PLAN, 2026-10-24); 12c0949 (tool schemas), f002666 (Planner loop, refusal as outcome), 780c6e6 (stub-client tests), 3a0eeff (node, issue #12), all 2026-10-24; 9bada45, 1ab2dae, 377da6a (route stamps, corridor_nav); e1c3442, 8693435 (planner cancel semantics); 7f40503, 99e5dab (ARCHITECTURE updates, 2026-10-26).
- Files: ARCHITECTURE.md:43,49-67,87-88,102; PLAN.md:50-51; ws/src/corridor_planner/corridor_planner/mission_planner.py:1-7,110-117,121-123,203,206-209,218-227; ws/src/corridor_planner/corridor_planner/tools.py:3-29; ws/src/corridor_planner/test/test_planner.py:87-93; ws/src/corridor_nav/corridor_nav/waypoint_follower.py:42,58,67; ws/src/corridor_nav/corridor_nav/geofence_monitor.py:22,73-77; ws/src/corridor_nav/corridor_nav/patrol_server.py:1-16.
- Proofs: steps/08_proofs.sh:83-91 (#12, tools wired to ROS via `/mission/tool_call`, `dry_run:=true`); steps/11_live_model_proof.sh:15 (live model, `status_loop:=true`).
- Issues: #11 (skeleton, 713b866), #12 (node, 3a0eeff), #14 (status_loop, be5a71a), #19 (fix vs odom).

## Sources

- ARCHITECTURE.md lines 43, 45, 58, 63, 65-67, 87-88, 102
- PLAN.md lines 48-56
- ws/src/corridor_planner/corridor_planner/mission_planner.py, tools.py
- ws/src/corridor_planner/test/test_planner.py, test_mission_planner_node.py
- ws/src/corridor_nav/corridor_nav/waypoint_follower.py, geofence_monitor.py, patrol_server.py
- steps/08_proofs.sh, steps/11_live_model_proof.sh
- git log (ce1ae24, 12c0949, f002666, 780c6e6, 3a0eeff, 713b866, be5a71a, 9bada45, 1ab2dae, 377da6a, e1c3442, 8693435, 7f40503, 99e5dab)
