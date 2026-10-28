# ADR-0002: ROS 2 Jazzy from source, never /opt/ros; extra packages via ws/deps.repos

Status: Accepted (date: 2026-10-24, commits b6839c1, a8cb568, a9497b1; amended 2026-10-27 by d1a5526)

## Context

Thor (JetPack 7, Ubuntu 24.04, aarch64) carries ROS 2 Jazzy as a source build at `~/.goske/ros2_jazzy`: 350 packages (rviz2, turtlesim, demo nodes, tf2) but no ros_gz, no Nav2, no ros2_control (CLAUDE.md:8). Any `ros-jazzy-*` apt package would land in `/opt/ros` and shadow that build (CLAUDE.md:8; README.md:25). The machine is shared and `~/.bashrc` is off limits, so the environment has to come from a repo file (CLAUDE.md:7,10). Day 1 already hit this: `rosdep` wanted `ros-jazzy-rosx-introspection` for foxglove_bridge 0.8.5 (steps/02c_build_bridge.sh:2,22; commit a8cb568; README.md:25). The later Gazebo and control stacks needed 15 gz vendor packages at first (commit 037c450), 17 in the end, plus 42 control packages the underlay lacks (ARCHITECTURE.md:102).

## Decision

- The underlay is the source build only. `env.sh` sources `~/.goske/ros2_jazzy/install/setup.bash`, then `ws/install/setup.bash`, and exports `ROS_DOMAIN_ID=42` (env.sh; README.md:11; ARCHITECTURE.md:12). `steps/01_init.sh:10` writes the same source line.
- No ROS apt packages. The bridge build steps run `rosdep install ... --simulate` and abort if the output names `ros-jazzy` (steps/02b_bridge_tag.sh:19-21, steps/02c_build_bridge.sh:11-13: `ABORT: rosdep wants ROS apt packages`); they install only the apt packages rosdep resolved (`libwebsocketpp-dev nlohmann-json3-dev rapidjson-dev`, README.md:26, commit a8cb568). steps/06_ros_gz_build.sh:11-12 echoes the same simulate output for `ws/src/ros_gz` without an abort. Non-ROS apt libs a build needs are installed directly and named in the commit message (CLAUDE.md:11): `libcli11-dev libgflags-dev libdart6.13-all-dev` (steps/06_ros_gz_build.sh:10, commit d6a40dc), `python3-typeguard python3-jinja2` (steps/09_ros2_control_build.sh:9, commit e75cd4d).
- Every missing ROS package is pinned by branch or tag in `ws/deps.repos` (44 entries: foxglove_bridge 0.8.5, rosx_introspection master, foxglove-sdk main; ros_gz jazzy with 16 gz_*_vendor packages and sdformat_vendor, all jazzy; interface repos actuator_msgs, gps_umd, marine_msgs, vision_msgs, simulation_interfaces, sdformat_urdf; xacro ros2; ros2_control, ros2_controllers, gz_ros2_control, control_msgs, realtime_tools, control_toolbox, ros2_control_cmake jazzy and their deps generate_parameter_library, rsl, cpp_polyfills humble, backward_ros, filters, angles, diagnostics, tl-expected, pal_statistics) and imported with `vcs import ws/src < ws/deps.repos`, then built with colcon in the overlay (steps/06_ros_gz_build.sh:8,14; steps/09_ros2_control_build.sh:8,21-24). The root `.gitignore` ignores `ws/src/*` except `ws/src/corridor_*`, so clones are never committed (commit a9497b1).
- Gazebo Harmonic is the exception: it comes from the OSRF apt repo because it is not a ROS package and does not shadow the source build (CLAUDE.md:9; ARCHITECTURE.md:81; steps/05_gz_install.sh). The gz_*_vendor packages wrap the system Harmonic (commit 037c450); gz_dartsim_vendor wraps the system DART from `libdart6.13-all-dev` (commit d6a40dc).
- Amendment d1a5526 (2026-10-27): `env.sh` tries `~/.goske/ros2_jazzy/install/setup.bash` first and falls back to `/opt/ros/jazzy/setup.bash` only when that file is absent (the second Thor has Jazzy from apt, per the commit body). On the primary Thor the rule stands: never install ROS from apt into `/opt/ros` (CLAUDE.md:8).

## Alternatives considered

- `apt install ros-jazzy-*` for ros_gz, ros2_control, foxglove_bridge: rejected, it would shadow the 350-package source build (CLAUDE.md:8; README.md:25).
- Nav2 in the overlay: deferred. ARCHITECTURE.md:43 puts it at ~40 source packages on this underlay; ARCHITECTURE.md:102 says the ros_gz and ros2_control builds confirmed the underlay cannot absorb it cheaply; pure pursuit in `corridor_nav` instead (ARCHITECTURE.md:87).
- Embedded clones in the repo: this is what a8cb568 did (`ws/src/foxglove_bridge` and `ws/src/rosx_introspection` committed as gitlinks); a9497b1 removed them and replaced them with `deps.repos` + `vcs import` and the ignore rule ("not embedded repos").

## Consequences

- Good: one underlay, no `/opt/ros` ambiguity; every external package is pinned by branch/tag in one file and rebuilt from a clean `colcon build` (CLAUDE.md:16); the `rosdep --simulate` grep in steps/02b and 02c catches a ROS apt dependency before anything installs.
- Good: trimming is possible: steps/09_ros2_control_build.sh:10-18 `COLCON_IGNORE`s controllers other than diff_drive_controller and joint_state_broadcaster, most of diagnostics, and the gz_ros2_control demos.
- Bad: long builds and large dependency trees (17 vendor packages, DART and OGRE-Next, 42 control packages; ARCHITECTURE.md:102); build-order quirks must be hand-fixed (tl-expected built in a first pass because rsl names the apt key `libexpected-dev`, not the package; steps/09_ros2_control_build.sh:20-21; commits 000b3f3, 2b96c50, 8ba7fea).
- Bad: branch drift upstream (ros-foxglove-bridge main is ROS 1 only, pinned to tag 0.8.5, README.md:23; rosx_introspection branch is master not main, b44f605; cpp_polyfills must be humble because main dropped tl_expected, e0662ba).
- Bad: anything not in deps.repos is unavailable; each new dep is a deps.repos commit plus a rebuild (commit history: a77f3ce, 037c450, b3f1d55, 2b96c50, e0662ba).

## Evidence

- Files: CLAUDE.md:7-11, README.md:11-26, ARCHITECTURE.md:12,43,81,87,102, env.sh, ws/deps.repos, .gitignore, steps/01_init.sh:10, steps/02_clone_deps.sh, steps/02b_bridge_tag.sh:19-23, steps/02c_build_bridge.sh:11-15, steps/05_gz_install.sh, steps/06_ros_gz_build.sh:8-15, steps/09_ros2_control_build.sh:8-24.
- Commits (all 2026-10-24 unless noted): b6839c1 (workspace skeleton and env.sh), a8cb568 (bridge 0.8.5 + rosx_introspection from source, embedded), a9497b1 (deps.repos + ignore rule, not embedded repos), a77f3ce / 2d8778e (ros_gz, issue #2), 037c450 (15 gz vendor packages), d6a40dc (system DART), b3f1d55 / 3866c77 (ros2_control, issue #5), 000b3f3, 2b96c50, 8ba7fea (tl-expected ordering), e0662ba (cpp_polyfills humble), b44f605 (rosx_introspection master), e75cd4d (steps/09 apt), d1a5526 (2026-10-27, /opt/ros fallback).
- Proof: `rosdep install --from-paths ws/src --ignore-src -y --simulate` lists no `ros-jazzy-*` package (README.md:15; abort guard in steps/02b, 02c; echoed for ros_gz in steps/06).
- Issues: #2 (ros_gz), #5 (ros2_control), as named in the commit subjects above.
