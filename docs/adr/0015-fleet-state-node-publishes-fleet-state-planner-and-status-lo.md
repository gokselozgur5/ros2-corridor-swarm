# ADR-0015: fleet_state node publishes /fleet/state; planner and status_loop read it

Status: Accepted (2026-10-28, #20, proof `steps/16_fleet_state_proof.sh`)

## Context
`/fleet/state` is in the ARCHITECTURE diagram as the input of `cot_bridge` and the source of the planner's `list_vehicles()` tool (ARCHITECTURE.md:16, :47, :55), and `corridor_msgs` has carried `FleetState.msg` (`header`, `corridor_id`, `VehicleState[] vehicles`) and `VehicleState.msg` (`header`, `name`, `fix`, `pose`, `velocity`, `battery_pct`, `mode`, `goal_id`, `inside_corridor`) since 2026-10-24 (commits 7906354, cfe1139). Nobody publishes or subscribes to it: outside `corridor_msgs` (`msg/FleetState.msg`, `CMakeLists.txt:12-13`) and the docs, `grep` over `ws/src` and `steps` finds no reference to `FleetState` or `/fleet/state`. Instead three nodes each subscribe to every vehicle's `/vN/fix` and `/vN/odom` and build their own summary: `mission_planner.py:106-108` and `status_loop.py:35-39` (both also `/vN/follower/status`), `cot_bridge.py:37-40` (fix + odom, course from the odom orientation); the first two also subscribe to `/fleet/geofence_events` (`mission_planner.py:113`, `status_loop.py:40`). The planner recomputes `inside_corridor`/`edge_distance_m` from the GeoJSON polygon on every `list_vehicles`/`get_vehicle` call (`mission_planner.py:136-151`), duplicating `geofence_monitor` (`geofence_monitor.py:53-56`). ARCHITECTURE open question 3 (ARCHITECTURE.md:100) names the fix: one `fleet_state` node, both consumers read it.

## Decision

Implemented 2026-10-28 in `corridor_nav/fleet_state.py`. Of the two options for `mode`/`goal_id` below, (b) was taken: `fleet_state` reads `/vN/patrol/_action/status` (action_msgs/GoalStatusArray, latched) for the running goal id, `follower/status` for following/idle/arrived and `/fleet/geofence_events` for a hold; the planner publishes nothing for it and keeps no mode of its own. Heading comes from the IMU, not the odom orientation (same contract as the follower, #19). `period_s` defaults to 0.5 s, `fix_timeout_s` to 5 s.

As proposed:
Add node `fleet_state` (package `corridor_planner`, or `corridor_nav` next to `geofence_monitor`; to settle at implementation). It takes params `vehicles` (list, default `['v1']`, same as the three consumers today) and `corridor_geojson`, subscribes per vehicle to `/vN/fix`, `/vN/odom`, `/vN/follower/status`, and to `/fleet/geofence_events`, and publishes `corridor_msgs/FleetState` on `/fleet/state` on a timer (`period_s`, proposed 1.0 s). Per vehicle it fills `name`, `fix`, `pose` (odom orientation only, for `cot_bridge`'s course; position from the fix through the corridor `LocalFrame`, never odom x/y, following #19 and ARCHITECTURE.md:91), `velocity` (odom twist), `inside_corridor` (signed distance to the polygon, same code as `mission_planner._summary`), `mode`, `goal_id`. `mission_planner` drops its fix/odom/status subscriptions and `_summary`; `t_list`/`t_get` serialise the latest `FleetState`. `status_loop` drops its per-vehicle subscriptions and builds `snapshot()` from the latest `FleetState`. `cot_bridge` migration is in scope of PLAN item 16 (PLAN.md:65), not this ADR.
Open point to resolve before Accepted: `mode` and `goal_id` are today planner-internal state set by tool calls (default `idle`, `mission_planner.py:83`; `patrol`/`goto`/`stopped` at :178, :204, :215, :236; nobody sets the `geofence_hold` value listed in `VehicleState.msg:8`); `fleet_state` cannot observe them from vehicle topics. Options: (a) planner publishes `/mission/goals` (`vehicle`, `mode`, `goal_id`) and `fleet_state` merges it; (b) `fleet_state` derives `mode` from `follower/status` and the `/vN/patrol` action status topic. (a) is the proposal. `battery_pct` stays 0 until a battery source exists (none in `ws/src`; `tools.py:6` advertises battery in `get_vehicle` but `_summary` returns none).

## Alternatives considered
- Keep per-node summarising (status quo). Rejected: three copies of the same subscriptions and two of the corridor test; `/fleet/state` promised in the diagram and PLAN item 16 never materialises.
- `geofence_monitor` publishes `FleetState` itself (it already has fix + polygon, `geofence_monitor.py:34-38`). Not evaluated; mixes the breach-event role with a fleet aggregate and still lacks `mode`/`goal_id`.
- Planner publishes `/fleet/state` as a side effect of `list_vehicles`. Not evaluated; would make the fleet view depend on LLM tool calls and on the planner running.

## Consequences
- Good: one subscription set and one corridor computation; `status_loop` and `cot_bridge` can run without the planner; `FleetState` becomes a real topic Foxglove/`steps` proofs can `ros2 topic echo`.
- Good: the ARCHITECTURE `list_vehicles()` row (line 55) becomes true instead of aspirational.
- Bad: one more node in `ws/src/corridor_planner/launch/mission.launch.py` (planner :26, status_loop :28) and one more hop of latency (timer period) between a fix and the planner's view; `t_get` today answers from the newest fix message (`_summary`, `mission_planner.py:139`), after the change from the newest `FleetState`.
- Bad: `mode`/`goal_id` need a new planner→fleet_state topic (option a), so the planner still publishes a low-level-ish topic; ties into open question 4 (ARCHITECTURE.md:101).
- Tests: the planner node test fakes `/v1/fix` and `/v1/odom` (`test_mission_planner_node.py:50-51, 58-59`; fakes described in af8abbb) and must switch to publishing a `FleetState`; add a `fleet_state` unit test for the corridor flag and a proof step (`steps/08_proofs.sh` family; #12 runs `mission.launch.py` at :87) that echoes `/fleet/state` with N vehicles.

## Evidence
- ARCHITECTURE.md:16, :47, :55, :77, :90-91, :100-101 (diagram, cot_bridge, tools table, packages, fix-not-odom decisions, open questions 3 and 4); PLAN.md:35, :65.
- `ws/src/corridor_msgs/msg/FleetState.msg`, `VehicleState.msg`, `CMakeLists.txt:12-13`; commits 7906354, cfe1139 (2026-10-24).
- `ws/src/corridor_planner/corridor_planner/mission_planner.py:76-83, 104-113, 136-162, 178, 204, 215, 236`; `status_loop.py:29-57`; `tools.py:6`; `ws/src/corridor_cot/corridor_cot/cot_bridge.py:35-40`; `ws/src/corridor_nav/corridor_nav/geofence_monitor.py:34-56`.
- `ws/src/corridor_planner/launch/mission.launch.py:26-28`; `ws/src/corridor_planner/test/test_mission_planner_node.py:50-59`.
- Commit f96ec33 (status_loop takes speed only from odom, no odom x/y); 8693435 (cancelled goal leaves 'stopped'); af8abbb (planner node test fakes).
- No proof script exists yet; `steps/08_proofs.sh` is the place for one.
