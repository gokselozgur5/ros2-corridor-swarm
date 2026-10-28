# ADR-0003: foxglove_bridge 0.8.5 from source with BUILD_TESTING=OFF

Status: Accepted (date: 2026-10-24, commit a8cb568)

## Context

The operator view is Foxglove on the Mac over `ws://thor:8765` (ARCHITECTURE.md:8,47; README.md:19,50). ROS 2 Jazzy on Thor is a source build at `~/.goske/ros2_jazzy` (README.md:11). The project rule is that `rosdep install ... --simulate` must list no `ros-jazzy-*` apt package (README.md:15; steps/02b_bridge_tag.sh:21 and steps/02c_build_bridge.sh:13 abort on one), because an apt ROS package would shadow the source build with `/opt/ros` (README.md:25). `ros-foxglove-bridge` `main` is ROS 1 only now; 0.8.5 is the newest tag that still ships `ros2_foxglove_bridge` (README.md:23; steps/02b_bridge_tag.sh:2,10-14 picks the newest tag with that directory). The ROS 2 bridge in `foxglove-sdk` needs a Rust toolchain (README.md:23). The bridge's smoke test is compiled as C++20 and websocketpp 0.8.2 does not parse under it (README.md:24; steps/03_build.sh:2; commit a8cb568 body).

## Decision

- Clone `foxglove_bridge` at tag `0.8.5` and `rosx_introspection` at `master` into `ws/src` through `ws/deps.repos` (`vcs import ws/src < ws/deps.repos`), as external sources built with colcon (ws/deps.repos:2-9; README.md:14; ARCHITECTURE.md:81; commit a9497b1).
- Build with `colcon build --symlink-install --packages-up-to foxglove_bridge corridor_sim --cmake-args -DBUILD_TESTING=OFF` (README.md:16; commit a8cb568 body, steps/03_build.sh:15). The step that ran it, steps/03_build.sh:7, uses `--packages-select foxglove_bridge corridor_sim --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF`; the earlier attempts steps/02b_bridge_tag.sh:24 and steps/02c_build_bridge.sh:16 built with Release and no `BUILD_TESTING` flag.
- `rosx_introspection` comes from source because rosdep would otherwise install `ros-jazzy-rosx-introspection` from apt (README.md:25; steps/02c_build_bridge.sh:2,22).
- Non-ROS apt deps only: `libwebsocketpp-dev nlohmann-json3-dev rapidjson-dev` (README.md:26; commit a8cb568 body).
- Both launch files start node `foxglove_bridge` (package `foxglove_bridge`, executable `foxglove_bridge`) with `port: 8765`, `address: 0.0.0.0`, `send_buffer_limit: 100000000`; `sim.launch.py` adds `use_sim_time: True`; launch arg `foxglove` (default `true`, `foxglove:=false` skips it) (ws/src/corridor_nav/launch/kinematic.launch.py:45-48; ws/src/corridor_gz/launch/sim.launch.py:30-33; README.md:50).

## Alternatives considered

- `ros-foxglove-bridge` `main`: rejected, ROS 1 only now (README.md:23; steps/02b_bridge_tag.sh:2).
- ROS 2 bridge from `foxglove-sdk`: rejected, needs a Rust toolchain (README.md:23). The sdk checkout (ws/deps.repos:174-177, branch `main`) is used only for `foxglove_msgs`, with `COLCON_IGNORE` on the rest (steps/15_foxglove_msgs.sh:2,8,11; commit d87ee88).
- apt `ros-jazzy-foxglove-bridge` / `ros-jazzy-rosx-introspection`: rejected, any `ros-jazzy-*` apt package would shadow the source ROS build; the step scripts abort on it (README.md:15,25; steps/02b_bridge_tag.sh:21; steps/02c_build_bridge.sh:13).

## Consequences

- Good: no ROS apt packages on Thor (README.md:15; commit a8cb568 body); the bridge is pinned (`0.8.5` in ws/deps.repos:5) and re-imported via `vcs import` (README.md:14); Foxglove connects from the Mac on 8765 (PLAN.md:15-16); README.md:50 documents importing the layout `foxglove/corridor.json`.
- Bad: the bridge's own tests are never built or run (`-DBUILD_TESTING=OFF`, README.md:16,24); a bridge regression would surface only through the proof scripts in `steps/`, not its test suite.
- Bad (inference from README.md:23): pinned to a tag on a repo whose `main` is ROS 1 only, so no upstream ROS 2 bridge fixes without moving to the sdk bridge, which needs Rust.
- `rosx_introspection` floats on `master` (ws/deps.repos:6-9; commit b44f605), unpinned.
- `foxglove_msgs` needed a separate step with the same `-DBUILD_TESTING=OFF` (steps/15_foxglove_msgs.sh:11).

## Evidence

- Commits: a8cb568 (2026-10-24, \"feat: foxglove_bridge 0.8.5 and rosx_introspection from source, corridor_sim with a vehicle_sim node\"; body: \"BUILD_TESTING=OFF because the bridge's smoke_test is compiled as C++20 and websocketpp 0.8.2 does not parse under it\"), 5a1739c (2026-10-24, \"docs: Day 1 done; how to build and run, why the bridge is 0.8.5 without tests; the step scripts\": README.md, steps/01-04), a9497b1 (2026-10-24, \"chore: cloned sources via ws/deps.repos\"), b44f605 (rosx_introspection branch is master, not main), d87ee88 (foxglove_msgs from foxglove-sdk).
- Files: README.md:11-26, 49-50; ARCHITECTURE.md:8,47,81; PLAN.md:15-16; ws/deps.repos:2-9,174-177; steps/02_clone_deps.sh; steps/02b_bridge_tag.sh; steps/02c_build_bridge.sh; steps/03_build.sh; steps/15_foxglove_msgs.sh; ws/src/corridor_nav/launch/kinematic.launch.py:45-48; ws/src/corridor_gz/launch/sim.launch.py:30-33.
- Proof: PLAN.md:16 (`ros2 topic hz /vehicle/fix` ≈ 10 Hz, Foxglove connects from the Mac); steps/03_build.sh:9 prints the first `foxglove_bridge` executable after the build.
- Issues: none reference this decision (gh issue list #1-#19; #3 and #9 mention Foxglove only as the viewer for their proofs).

## Sources

- README.md:11,14-16,19,23-26,50
- ARCHITECTURE.md:8,47,81
- PLAN.md:15-16
- ws/deps.repos:2-9,174-177
- steps/02_clone_deps.sh:2,7,11-13; steps/02b_bridge_tag.sh:2,10-14,21,24; steps/02c_build_bridge.sh:2,13,16,22; steps/03_build.sh:2,7,9,15-16; steps/15_foxglove_msgs.sh:2,8,11
- ws/src/corridor_nav/launch/kinematic.launch.py:45-48; ws/src/corridor_gz/launch/sim.launch.py:30-33
- git log: a8cb568, 5a1739c, a9497b1 (all 2026-10-24), b44f605, d87ee88; gh issue list (#1-#19)
