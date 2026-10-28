# ADR-0013: Hosts, users and keys only from .env, never in the repo

Status: Accepted (date: 2026-10-24, commit 284cb14; env.sh fixes 2026-10-24 a08b4b6 and 2026-10-25 b63073d)

## Context

The README once pointed Foxglove at a literal LAN address (`ws://192.168.88.229:8765`, replaced by `ws://$THOR_HOST:8765` in 284cb14). The project needs four machine- or account-specific values, named in `.env.example`: Thor's LAN address and ssh user (Foxglove, ssh), an Anthropic API key (corridor_planner, Phase 3), and a FreeTAKServer address (Phase 4). The canonical repo is GitHub `origin` and is checked out on both Thor and the Mac mirror (CLAUDE.md "Working rules", line 15), so any literal in it reaches every clone. CLAUDE.md already fixes one half of the rule ("Working rules", line 19): "The key comes from `ANTHROPIC_API_KEY` in the shell, never from the repo."

## Decision

- `.env.example` is committed and lists exactly `THOR_HOST`, `THOR_USER`, `ANTHROPIC_API_KEY`, `FTS_HOST`, each with an empty value and a trailing comment, under the header "copy to .env and fill; never commit .env".
- `.env` is gitignored (`.gitignore` line `.env`, added in 284cb14).
- `env.sh` sources `.env` if present and exports everything it defines (`set -a; . "$_env"; set +a`), so values reach ROS nodes, not only the shell (b63073d: without `set -a`, `ANTHROPIC_API_KEY` never reached the nodes). Sourcing returns 0 when `.env` is absent (a08b4b6).
- Consumers read the environment, never a file in the repo: `corridor_cot/cot_bridge.py:28` declares param `host` with default `os.environ.get('FTS_HOST', '127.0.0.1')` (docstring line 3: "TAK server (FTS_HOST:8087"); `corridor_planner/mission_planner.py:127` and `status_loop.py:46` call `anthropic.Anthropic()` (the SDK reads `ANTHROPIC_API_KEY`); `corridor_planner/launch/mission.launch.py:2` says "ANTHROPIC_API_KEY from the shell"; `steps/11_live_model_proof.sh:6` exits 1 with "ANTHROPIC_API_KEY is not set; put it in .env" when the key is unset; `tools/planner_smoke.py:3` "Run with ANTHROPIC_API_KEY in the environment"; docs reference `$THOR_HOST` only (README.md:19, :51).
- README "Local settings" (README.md:34-36): "Hosts, users and keys are not in the repo. Copy `.env.example` to `.env`, fill it, and `. ./.env` before `env.sh`. `.env` is gitignored."

## Alternatives considered

- Literal values in README/launch files (the pre-284cb14 state): rejected, a LAN address was already committed and a key would follow it into every clone.
- Editing `~/.bashrc` on Thor: rejected by CLAUDE.md "Environment" (line 10: "Do not edit `~/.bashrc`. Put the environment in `./env.sh` and source that."); Thor is "a desktop machine other people also use" (CLAUDE.md line 8).
- ROS params or a committed YAML for hosts: not evaluated in any doc or commit; `cot_bridge` keeps a `host` param but defaults it from `FTS_HOST`, so the env stays the source.

## Consequences

- Good: HEAD holds no address, user or key (`git grep 192\.168 HEAD` is empty); a fresh clone is usable after copying one file; a missing key fails loudly in `steps/11_live_model_proof.sh:6`.
- Good: `cot_bridge` and the planner's debug path run with no `.env`: `127.0.0.1` default (`cot_bridge.py:28`); `steps/08_proofs.sh 12` launches `mission.launch.py dry_run:=true` (line 87) and passes "debug path; the model path needs ANTHROPIC_API_KEY" (line 97).
- Bad: the literal `192.168.88.229` remains in git history (b6839c1, 284cb14); the rule removed it from HEAD only.
- Bad: the rule is enforced only by `.gitignore` and review; nothing scans commits for leaked literals.
- Bad: every `steps/*` script sources `./env.sh`, and without `.env` that is a silent no-op (a08b4b6 makes it return 0); only `steps/11_live_model_proof.sh` checks for the key, the other steps simply run without the values.
- Bad: README "Local settings" still says to `. ./.env` before `env.sh`; since b63073d `env.sh` does that itself with `set -a`, the instruction is redundant.
- Bad: `.env` is one flat file; no per-phase separation, and `FTS_HOST` ("usually the same as THOR_HOST", `.env.example` line 5) is duplicated by hand.

## Evidence

- 284cb14 (2026-10-24) "chore: hosts, users and keys come from .env, not the repo": adds `.env.example` (5 lines), `.gitignore` `.env`, `env.sh` sourcing line, README "Local settings"; README.md:19 literal address becomes `$THOR_HOST`.
- a08b4b6 (2026-10-24) "env.sh: sourcing returns 0 when .env is absent (the trailing test broke && chains)".
- b63073d (2026-10-25) "env.sh: export what .env defines (set -a); sourcing alone left ANTHROPIC_API_KEY out of the nodes' environment".
- d1a5526 later touches `env.sh` (ROS underlay fallback to /opt/ros/jazzy), unrelated to `.env`.
- README.md:32 and PLAN.md:9: the Phase 3 model path "waits for ANTHROPIC_API_KEY in `.env`".

Sources: `.env.example`; `.gitignore`; `env.sh`; `README.md:19,32,34-36,51`; `PLAN.md:9`; `CLAUDE.md:8,10,15,19`; `ws/src/corridor_cot/corridor_cot/cot_bridge.py:3,28`; `ws/src/corridor_planner/corridor_planner/mission_planner.py:127`; `ws/src/corridor_planner/corridor_planner/status_loop.py:46`; `ws/src/corridor_planner/launch/mission.launch.py:2`; `steps/11_live_model_proof.sh:2,4,6`; `steps/08_proofs.sh:87,97`; `steps/lib.sh:1-2`; `tools/planner_smoke.py:3,28`; git: `git show 284cb14 a08b4b6 b63073d d1a5526`, `git log -S192.168.88.229`, `git grep 192\.168 HEAD`, `gh repo view gokselozgur5/ros2-corridor-swarm --json visibility` (PRIVATE).
