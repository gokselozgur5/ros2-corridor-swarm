# ADR-0006: Models: Opus 5.5 planning, Haiku 5.5 status loop, fallbacks default, refusal is an outcome

Status: Accepted (date: 2026-10-24, commits ce1ae24 docs, f002666 planner, be5a71a status loop)

## Context
Phase 3 puts a Claude mission planner between a typed sentence and the fleet (PLAN.md §Phase 3, #11-#14). Two call patterns exist: a rare, tool-using planning call (tools `list_vehicles`, `get_vehicle`, `patrol`, `goto`, `stop`, `describe_scene`, `geofence_events`, ARCHITECTURE.md:53-61) and a periodic one-paragraph fleet summary every 30 s on `/mission/summary` (PLAN.md:56). Budget is a $200 credit; a planning call is estimated at ~5k in / 1k out tokens, about $0.04, and the status loop at 2 calls/min under $0.05/h (PLAN.md:77-79), target < $0.20/h (PLAN.md:57). A policy refusal from the API arrives as HTTP 200 with `stop_reason == "refusal"`, so a node that reads `content` first would crash on a safe answer (planner.py:5-7). The planner must never publish `cmd_vel` (ARCHITECTURE.md:67).

## Decision
- Planning: `corridor_planner.planner.Planner` runs a manual tool-use loop on `PLANNING_MODEL = 'claude-opus-5-5'`, `max_tokens=4096`, `output_config={'effort': ...}` (default `medium`), `betas=['server-side-fallback-2026-07-01']`, `fallbacks='default'` (planner.py:12-13, 39-40, 46-56). The `mission_planner` node exposes params `model` (default `PLANNING_MODEL`) and `effort` (mission_planner.py:91-92, 128-129).
- Status loop: `status_loop` node, `STATUS_MODEL = 'claude-haiku-5-5'`, params `period_s` (30.0) and `model`, `max_tokens=200`, effort `low`; publishes `/mission/summary` and `/mission/summary_cost` (JSON: calls, input_tokens, output_tokens, usd, usd_per_hour at $0.10/$0.50 per 1M) (status_loop.py:20-21, 30-31, 42-43, 59-80).
- Refusal is an outcome, not an exception: `stop_reason` is checked before `content`; on `refusal` the planner sets `Outcome.refused`, records `stop_details.category`, appends a `refusal` trace event and rolls the exchange back from `messages` (planner.py:106-112). The node turns it into a `DiagnosticStatus.WARN` on `/diagnostics` and a reply "the model declined this request (...)" (mission_planner.py:294-296); the status loop logs "status summary refused" and skips the tick (status_loop.py:71-73). SDK errors become `ERROR` diagnostics, never a crash (mission_planner.py:7, 297-299; planner.py:95-101).
- `/mission/tool_call` debug path runs a tool without the model (mission_planner.py:119-120); `dry_run` runs the node with no client (mission_planner.py:93, 125-129, 287-288).
- Key comes from `ANTHROPIC_API_KEY` in the shell, never the repo (CLAUDE.md:19).

## Alternatives considered
- One model for both paths (Opus everywhere): rejected on cost; the summary runs 2/min and needs no tools, Haiku's estimate is under $0.05/h against the $0.20/h target (PLAN.md:56-57, 77-79).
- Treat `refusal` as an exception and let the node restart: rejected; PLAN #11 proof requires "a `refusal` response yields a diagnostic and no exception" (PLAN.md:51), and a restart would drop the conversation history the planner keeps (planner.py:42-44, 58-62).
- Run without server-side fallback (plain `messages.create`): not evaluated as a design choice, but this is in fact what `status_loop` does today (see Consequences).

## Consequences
- Good: a refused or failed planning call is visible in Foxglove (`/diagnostics`, `/mission/llm_trace`) and the vehicles keep whatever goal they had; the planner never emits `cmd_vel` (asserted by `test_request_shape_has_fallbacks_and_no_cmd_vel_tool`, test_planner.py:87-93).
- Good: cost is measured, not guessed: `/mission/summary_cost` and the `cost: calls=... usd_per_hour=...` log line (status_loop.py:82), read by `steps/11_live_model_proof.sh:31-33`.
- Good: model and effort are ROS params (mission_planner.py:91-92, status_loop.py:31), so a cheaper model can be set with `--ros-args -p` without a code change. `mission.launch.py` exposes no launch argument for them (only `dry_run`, `status_loop`; launch/mission.launch.py:15-21).
- Bad: ARCHITECTURE.md:64 and CLAUDE.md:19 say both models use `fallbacks: "default"`, but `status_loop.tick` calls `client.messages.create` with no `betas` and no `fallbacks` (status_loop.py:61-64). Only the planner has it (planner.py:47-56). Docs and code disagree; one must change.
- Bad: PLAN #14's proof says the measured $/h is "written to the README" (PLAN.md:57); README.md has no such number. PLAN.md:11 records #12-#14 as proven live on 2026-10-25; the figure is not in the repo.
- Bad: Haiku pricing is a hard-coded constant (status_loop.py:21); a price change silently skews the reported cost.
- Bad: a refusal mid tool-loop rolls back the whole exchange from `messages`, but tool calls already executed (e.g. a `stop` or `patrol` goal sent) are not undone; the test shows `tool_calls == 1` with `messages == []` after the refusal (planner.py:106-112; test_planner.py:118-130).

## Evidence
- Commits: ce1ae24 (2026-10-24, docs: architecture/plan/README/CLAUDE.md), 713b866 (#11 skeleton), f002666 (Planner on claude-opus-5-5, fallbacks default, refusal as outcome), 780c6e6 (stub-client tests for refusal, tool loop, request shape, SDK errors), 3a0eeff (#12 mission_planner node, diagnostics), be5a71a (#14 status_loop on Haiku, cost per hour), 220d2e5 (2026-10-25, cost log line), d5a19bb (step 11 reads cost from node log), 550603c (PLAN note: #12-#14 live).
- Files: ws/src/corridor_planner/corridor_planner/planner.py, mission_planner.py, status_loop.py; ws/src/corridor_planner/launch/mission.launch.py; ws/src/corridor_planner/test/test_planner.py (`test_refusal_is_an_outcome_not_an_exception`:49, `test_request_shape_has_fallbacks_and_no_cmd_vel_tool`:87, `test_sdk_error_keeps_node_alive`:96, `test_refusal_mid_tool_loop_rolls_back_the_whole_exchange`:118).
- Proof scripts: steps/08_proofs.sh 12 (debug path, :82-95), steps/11_live_model_proof.sh (#12 #13 #14, needs the key).
- PLAN.md steps: #11, #12, #13, #14 (PLAN.md:50-57); GitHub issue contents not checked from the repo.

## Sources
- ARCHITECTURE.md:49-67
- PLAN.md:9-11, 48-57, 77-79
- CLAUDE.md:19
- README.md (grep for cost/usd/per hour: no hit)
- ws/src/corridor_planner/corridor_planner/planner.py:1-135
- ws/src/corridor_planner/corridor_planner/mission_planner.py:1-8, 86-133, 285-304
- ws/src/corridor_planner/corridor_planner/status_loop.py:1-94
- ws/src/corridor_planner/launch/mission.launch.py:2, 15-28
- ws/src/corridor_planner/test/test_planner.py:49, 87-93, 96, 118-130
- steps/08_proofs.sh:82-95; steps/11_live_model_proof.sh:2, 19, 31-33
- git log: ce1ae24 f002666 be5a71a 713b866 780c6e6 3a0eeff (2026-10-24); 220d2e5 d5a19bb 550603c (2026-10-25)
