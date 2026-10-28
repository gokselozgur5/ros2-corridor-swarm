# ADR-0001: Gazebo Harmonic instead of Isaac Sim, headless only

Status: Accepted (2026-10-24, commits ce1ae24 and a55d543)

## Context
The whole project is simulated on one Jetson Thor: JetPack 7, Ubuntu 24.04, aarch64, 14 cores, 122 GB RAM, a desktop machine other people also use (CLAUDE.md:7). ROS 2 Jazzy is a source build at `~/.goske/ros2_jazzy` with no ros_gz, no ros2_control, no Nav2; nothing may be installed into /opt/ros (CLAUDE.md:8). The simulator must own physics, motors and sensors, so no ROS node fakes a sensor (ARCHITECTURE.md:41). The operator view is Foxglove on a Mac over `ws://thor:8765`, not a desktop on Thor (ARCHITECTURE.md:8, README.md:51). GPU on Thor is shared with other people (ARCHITECTURE.md:86).

## Decision
- Simulator: Gazebo Harmonic (`gz-harmonic`) from the OSRF apt repo, installed by `steps/05_gz_install.sh` (issue #1, steps/05_gz_install.sh:2,12). It is not a ROS package and does not shadow the source build (CLAUDE.md:9).
- `ros_gz` (branch `jazzy`, ws/deps.repos) built from source in `ws/src` against the underlay with `GZ_VERSION=harmonic` (steps/06_ros_gz_build.sh:9,15, issue #2).
- Headless only: `corridor_gz/launch/sim.launch.py` runs `gz sim -s -r -v 2 <world>` (sim.launch.py:34); no `gz sim -g`, no GUI on Thor. `corridor.sdf` loads Physics, UserCommands, SceneBroadcaster, Sensors (`ogre2`), Imu, NavSat systems (corridor.sdf:7-12); rendering sensors still need the Sensors system with ogre2 server-side (corridor.sdf:3).
- The Mac sees the world through Foxglove's 3D panel from tf and sensor topics (ARCHITECTURE.md:86); `ros_gz_bridge` `parameter_bridge` bridges `/clock`, `/v1/scan`, `/v1/camera/image`, `/v1/camera/camera_info`, `/v1/imu`, `/v1/fix`, `/v1/odom`, `/v1/tf`, `/v1/joint_states` GZ_TO_ROS and `/v1/cmd_vel` ROS_TO_GZ (`config/bridge_v1.yaml`); `use_sim_time: true` on every node in sim.launch.py (sim.launch.py:33,36,41,49; ARCHITECTURE.md:41).
- Vehicle `v1` spawned by `ros_gz_sim create` from `/v1/robot_description` at (20, 20, 0.5) in world `corridor` (sim.launch.py:37-39).

## Alternatives considered
- **Isaac Sim.** Rejected: needs x86 + RTX; Thor is aarch64 (ARCHITECTURE.md:85).
- **Gazebo with GUI on Thor (`gz sim` without `-s`).** Rejected: would steal GPU from the people who share the machine; Foxglove covers the view (ARCHITECTURE.md:86).
- **Kinematic-only sim (`vehicle_sim` / `unicycle_sim`), no physics engine.** Kept as the smoke test when Gazebo is unavailable, not as the main backend; the point is motors and sensors behaving like hardware (ARCHITECTURE.md:87, 89).
- **Gazebo Classic / Ignition Fortress.** Not mentioned anywhere in the repo; not evaluated.

## Consequences
- Good: arm64 apt packages for Ubuntu 24.04, no build of Gazebo itself (ARCHITECTURE.md:85). Real physics: `diff_drive_controller` under `gz_ros2_control`; proof 5 commands 1 m/s for 5 s and passes when `/v1/odom` x moves between 3.5 and 6.5 m (PLAN.md:28-29; steps/08_proofs.sh:102-113, commit 8f1ccda). Phase 1 closed with "ros2_control diff_drive moving the UGV" (README.md:32). Lidar/camera/imu/navsat come from gz plugins, not ROS fakes (ARCHITECTURE.md:28, 41; README.md:32).
- Good: headless keeps Thor's GPU untouched for other users (ARCHITECTURE.md:86). Install proof: `gz sim -s -r --iterations 100 empty.sdf` exits 0 (PLAN.md:21, steps/05_gz_install.sh:15).
- Bad: `ros_gz` plus 15 `gz_*_vendor` packages (steps/06_ros_gz_build.sh, commit 037c450; apt: `libcli11-dev libgflags-dev libdart6.13-all-dev`, steps/06:10) and `ros2_control`, `ros2_controllers`, `gz_ros2_control` and their dependencies (steps/09_ros2_control_build.sh, issue #5; apt: `python3-typeguard python3-jinja2`, steps/09:9) must be built from source against the source Jazzy underlay (PLAN.md:22, 28; ARCHITECTURE.md:81, 102).
- Bad: no local visual debugging; everything must be exposed as a topic to be seen (Foxglove layout `foxglove/corridor.json`, README.md:51).
- Bad: physics needs tuning that is not ground truth, e.g. `mu2 = 0.15` across the rolling direction for skid-steer turning (ARCHITECTURE.md:92; ws/src/corridor_description/urdf/ugv.gazebo.xacro:12).
- Shared host: other users run Gazebo, `parameter_bridge`, `robot_state_publisher` and `foxglove_bridge` on Thor, so proof cleanup kills by `CORRIDOR_PROOF` pid marker, never by executable name (steps/lib.sh:3-7, commit 40b4451, 2026-10-27).

## Evidence
- ce1ae24 (2026-10-24) docs: ARCHITECTURE.md with the "Gazebo, not Isaac Sim" and "Headless" decisions (ce1ae24:ARCHITECTURE.md:85-86).
- a55d543 (2026-10-24) `steps/05_gz_install.sh`, issue #1; 90633fc (2026-10-24) SIGPIPE fix to the same script. Proof: `gz sim -s -r --iterations 100 empty.sdf` exits 0 (PLAN.md:21).
- a77f3ce, 2d8778e (2026-10-24) `steps/06_ros_gz_build.sh`, issue #2. Proof: `parameter_bridge` publishes `/clock` (PLAN.md:23).
- 9e2361b (2026-10-24) `ws/src/corridor_gz/launch/sim.launch.py`, headless gz sim; current file line 34. `ws/src/corridor_gz/worlds/corridor.sdf:3-12`.
- 3866c77, 8f1ccda (2026-10-24) `steps/09_ros2_control_build.sh`, proof 5 in `steps/08_proofs.sh`, issue #5.
- README.md:32: Phase 1 closed (#1-#5).

## Sources
- CLAUDE.md:7-9
- ARCHITECTURE.md:8, 28, 41, 81, 85-89, 92, 102
- PLAN.md:20-29
- README.md:32, 51
- ws/src/corridor_gz/launch/sim.launch.py:33-49
- ws/src/corridor_gz/worlds/corridor.sdf:3-12
- ws/src/corridor_gz/config/bridge_v1.yaml:1-51
- ws/src/corridor_description/urdf/ugv.gazebo.xacro:7,12
- ws/deps.repos
- steps/05_gz_install.sh, steps/06_ros_gz_build.sh, steps/09_ros2_control_build.sh, steps/08_proofs.sh:102-113, steps/lib.sh:1-8
- git log: ce1ae24, a55d543, 90633fc, a77f3ce, 2d8778e, 037c450, 9e2361b, 3866c77, 8f1ccda (all 2026-10-24), 40b4451 (2026-10-27)
