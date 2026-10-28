# ADR-0014: Proof per step, atomic commits, every new test seen red by mutation

Status: Accepted (2026-10-24, commits 7febba2 and 57fb805; rules unchanged through the 2026-10-26 review wave merged as 60633c8 on 2026-10-27)

## Context

The project is a ROS 2 Jazzy swarm sim built on a source underlay (`~/.goske/ros2_jazzy`, 350 packages, no Nav2/ros_gz/ros2_control) on a shared Jetson Thor, driven mostly by an agent in a tmux session that Göksel watches (CLAUDE.md "Environment", "Working rules"; 260 of 264 commits carry a `Co-Authored-By: Claude` trailer). PLAN.md has 18 numbered steps under Phase 0 to Phase 5, tracked as one GitHub issue each. "Code exists" is a weak signal on this stack: a stub `rclpy` stand-in, the demo's ROS domain (`ROS_DOMAIN_ID=42`) and launch_testing collecting a whole test directory as one skip all let a run go green without exercising the node (2fe312f, 6bfeebf, d0fc4b8; `ws/src/corridor_nav/test/conftest.py` lines 43-44). The history is 264 commits between 2026-10-24 and 2026-10-27; without a rule, a step would land as one large commit and a test could be green against both the fix and its absence.

## Decision

Three rules in CLAUDE.md "Working rules", first written in 7febba2 and extended in 57fb805:

1. **Proof per step.** Every PLAN.md step (1-16) names a command whose output decides done-ness; "A step is not done because the code exists; it is done when the proof passes on Thor from a clean `colcon build`" (PLAN.md intro). CLAUDE.md: "Commit after every proven step with the proof command and its output in the body. Close the issue with the same output." Proofs are scripts in `steps/` (`08_proofs.sh <issue>`, `10_scene_proof.sh`, `11_live_model_proof.sh`, `12_gazebo_patrol.sh`, `13_cot_proof.sh`, `14_marker_proof.sh`; README.md line 47), run from tmux session `ros2`, logs in `~/deletdis/log/` (CLAUDE.md).
2. **Atomic commits.** "Commits are small: one logical change each, even one file. A doc change, an ignore rule, a config line, a function are separate commits. A step is normally several commits." (57fb805).
3. **Red by mutation.** "A new test must be seen red with the fix reverted before it counts." (7febba2). In practice the commit body names what was reverted or mutated and the red result (ba740ce, 7ebeeea, 040ce54).

## Alternatives considered

- **Step done when the code and its unit tests exist.** Rejected by PLAN.md intro: "A step is not done because the code exists." On this stack tests can be green against a stub `rclpy` or never collected at all (6bfeebf, d0fc4b8).
- **One commit per step / per issue.** Rejected by 57fb805 (subject only, no recorded rationale).
- **Coverage threshold instead of mutation.** Not evaluated; nothing in the repo mentions coverage.

## Consequences

- Good: proofs are reproducible by hand from the commit body (e.g. ba740ce: "Proof, seen red first (12 failed, 5 passed) ... 17 passed"). PLAN.md "Progress" cites proofs rather than claims: #1-#10 proven 2026-10-24, #12-#14 proven with the live model 2026-10-25, #19 and three follower faults "fixed with tests seen red first", `steps/12_gazebo_patrol.sh` as the proof.
- Good: 193 of 264 commits touch a single file, 33 touch two (`git log --shortstat`); review follow-ups merge as small commits (6266a61: two, 980690e: five), each revertable alone.
- Good: the PLAN #8 proof is itself a mutation: "The test is run with the monitor disabled first and must then fail."
- Bad: the rules cost wall time on Thor: a clean `colcon build` per proof, and a red run before each green one. PLAN.md "Budget": "the credit is not the constraint, Phase 1 build time is."
- Bad: proofs that need live resources (`ANTHROPIC_API_KEY` for `11_live_model_proof.sh`, Gazebo for #13, Docker for #15) cannot run everywhere; the test modules carry `REQUIRE_ROS` / demo-domain guards to fail rather than skip (2fe312f, e3b0c82, 3183073, af8abbb).
- Bad: the rules are enforced by convention and commit-body text, not by a hook; `.git/hooks` holds no hooks, there is no `.pre-commit-config.yaml`, and nothing in `steps/` verifies that the red run happened.

## Evidence

- Rules: CLAUDE.md "Working rules" (7febba2 2026-10-24 "docs: README status and CLAUDE.md working rules for the new direction"; 57fb805 2026-10-24 "docs: CLAUDE.md asks for one logical change per commit").
- Proof per step: PLAN.md intro and the "Proof:" line of steps 1-16 (ce1ae24); `steps/08_proofs.sh` .. `14_marker_proof.sh`, `steps/lib.sh`; README.md lines 5 and 47.
- Red-by-mutation in commit bodies: ba740ce, b53ac85, 33f5207, 8e0a6c8, 040ce54, 883fde6, 0a25ceb, 7dfa1db (2026-10-26); 7ebeeea, 6809e24, 3a120bc, c77795c (2026-10-25, PLAN.md 2026-10-25 note: "All four fixed with tests seen red first").
- Guards against false green: 2fe312f, 6bfeebf, e3b0c82, 3183073, d0fc4b8, af8abbb; `ws/src/corridor_nav/test/conftest.py`.
- Issues: #8 (proof must fail with monitor disabled), #10, #12-#14, #19.

## Sources

- CLAUDE.md ("Environment", "Working rules")
- PLAN.md (intro, "Progress", steps 1-18, "Budget")
- README.md (lines 5, 47, "Status")
- steps/ (08, 10-14, lib.sh)
- ws/src/corridor_nav/test/conftest.py
- git log: 7febba2, 57fb805, ce1ae24, 60633c8, ba740ce, b53ac85, 33f5207, 8e0a6c8, 040ce54, 883fde6, 0a25ceb, 7dfa1db, 7ebeeea, 6809e24, 3a120bc, c77795c, 2fe312f, 6bfeebf, d0fc4b8, e3b0c82, 3183073, af8abbb, 6266a61, 980690e; `git log --shortstat`
