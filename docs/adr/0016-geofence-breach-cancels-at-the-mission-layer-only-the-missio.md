# ADR-0016: Geofence breach cancels at the mission layer; only the mission layer talks to the follower

Status: Proposed (2026-10-28; raised as ARCHITECTURE open question 2 on 2026-10-24 in 5773e99, reworded to its current form on 2026-10-26 in 7f40503)

## Context

A geofence EXIT today reaches the follower by two routes. `geofence_monitor` (ws/src/corridor_nav/corridor_nav/geofence_monitor.py:34-38, 74-78) publishes `GeofenceEvent.EXIT` on `/fleet/geofence_events` and, with `cancel_on_exit` (default `True`, line 22), an empty `Path` straight to `/<vehicle>/path`. `patrol_server` (patrol_server.py:166, 180-185) subscribes to the same event and marks the run `breached`, which `verdict()` (lines 123-136) turns into result `geofence`, and `finish()` then clears the follower itself (lines 293-296). Since 2026-10-26 the server also watches `follower/route` (lines 164-165, commit 1ab2dae): the monitor's foreign Path makes `owner.owns()` false, so the same breach can also surface as `preempted` (lines 132-133), depending on which message lands first. 1ab2dae's message records the earlier form of this race ("the geofence result lost to the monitor's empty Path"); the stamp/ownership machinery it introduced (unique stamps, odd stamps in 377da6a, route reports and events in arrival order in 5ca5f1b, clear only for the current goal in 321574b) exists because several nodes write `/<vehicle>/path` (patrol_server.py:11-17). ARCHITECTURE.md:99 names the problem: "Two paths to the same effect." ARCHITECTURE.md:45 already defines the mission layer as `patrol_server`, `geofence_monitor`, `mission_planner`; the planner's rule is "only sends action goals" (ARCHITECTURE.md:67, 101).

## Decision

Proposed, not yet implemented:
- `geofence_monitor` stops publishing on `/<vehicle>/path`; the `path_pubs` publishers (geofence_monitor.py:35-38) and `cancel_on_exit` (line 22) are removed, or `cancel_on_exit` comes to mean "cancel the action" instead of "publish an empty Path".
- On EXIT the monitor keeps publishing `/fleet/geofence_events` (unchanged contract for `mission_planner` line 113, `status_loop` line 40, foxglove/corridor.json) and `patrol_server` remains the only node that stops the follower for a patrol: `on_geofence` -> `breached` -> `finish(run, 'geofence')` -> `clear_if_owned()` (patrol_server.py:180-185, 293-296). No second message reaches the follower from outside the mission layer.
- Follower writers after the change: `patrol_server` (`path`), `mission_planner` `goto` (mission_planner.py:181-203, a two-pose Path, until open question 4 makes `goto` an action) and `stop` (mission_planner.py:220-237, empty Path at 226), and the operator tools `clicked_goal`, `wasd`, `send_path` (patrol_server.py:11-12). `geofence_monitor` is no longer one.
- `verdict()` ordering stays: `breached` (line 130) is checked before `owns()` (line 132), so a breach is reported as `geofence`, never `preempted`.

## Alternatives considered

- **Keep both routes (status quo).** Rejected by ARCHITECTURE.md:99: the result name depends on message order.
- **Monitor sends a cancel request to `/<vehicle>/patrol` as an action client.** Not evaluated. It couples the monitor to the action type and to every future mission action (`goto` as action, open question 4); event + server-side handling needs no new client per action.
- **Monitor only, server ignores events.** Not evaluated. Loses the `geofence` result and leaves a vehicle with no patrol goal and no record why; the planner's `geofence_events` tool (tools.py:21) would be the only trace.

## Consequences

- Good: one writer per concern; the `geofence` result no longer depends on message order; `follower/route` ownership is needed only against operator tools and the planner, not the monitor.
- Good: proof #8 (steps/08_proofs.sh:36-51) still holds for a vehicle under patrol, because the server clears the follower on `geofence` (patrol_server.py:295-296).
- Bad: a vehicle driven by `send_path`/`wasd`/`clicked_goal` with no patrol goal is no longer stopped on EXIT. Proof #8 drives v1 with `send_path`, no patrol goal (08_proofs.sh:43), and passes only on `follower status: idle` (line 50); after the change no node would clear the follower, so the proof needs a patrol goal or a new rule for non-mission drivers.
- Bad: a breach during `mission_planner` `goto` (a raw Path, no action; `on_geofence` only marks a current patrol run, patrol_server.py:182-184) stops nothing until open question 4 is done.

## Evidence

- ARCHITECTURE.md:45, 67, 91, 99, 101 (open questions 2 and 4).
- ws/src/corridor_nav/corridor_nav/geofence_monitor.py:3-4, 22, 34-38, 74-78.
- ws/src/corridor_nav/corridor_nav/patrol_server.py:8-18, 123-136, 161-166, 175-185, 293-296.
- ws/src/corridor_planner/corridor_planner/mission_planner.py:113, 181-203, 207-237.
- ws/src/corridor_planner/corridor_planner/tools.py:21; status_loop.py:40; foxglove/corridor.json.
- steps/08_proofs.sh:36-51 (proof #8).
- Commits: a4bc4fb (2026-10-24, monitor publishes empty Path, #8), 9835222 (2026-10-24, patrol geofence abort, #9), 5773e99 (2026-10-24, open question 2), 1ab2dae, 377da6a, 5ca5f1b, 321574b (2026-10-26 ownership/race fixes), 7f40503 and 99e5dab (2026-10-26 ARCHITECTURE updates).

## Sources

- ARCHITECTURE.md (lines 45, 67, 91, 94-101)
- ws/src/corridor_nav/corridor_nav/geofence_monitor.py
- ws/src/corridor_nav/corridor_nav/patrol_server.py
- ws/src/corridor_planner/corridor_planner/mission_planner.py
- ws/src/corridor_planner/corridor_planner/tools.py
- ws/src/corridor_planner/corridor_planner/status_loop.py
- steps/08_proofs.sh
- foxglove/corridor.json
- `git log` / `git log -L99,99:ARCHITECTURE.md` / `git log -1 --format=%b` for 1ab2dae, 377da6a, 5ca5f1b, 321574b
