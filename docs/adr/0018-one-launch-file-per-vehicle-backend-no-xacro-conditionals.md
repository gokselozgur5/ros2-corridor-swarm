# ADR-0018: One launch file per vehicle backend, no xacro conditionals

Status: Proposed (date: 2026-10-28; question opened 2026-10-24 in 5773e99, partly settled 2026-10-25 in 0715c46)

## Context

Three odometry/drive backends exist for one vehicle: `unicycle_sim` (kinematic, package `corridor_sim`, `kinematic.launch.py:24`), the gz `OdometryPublisher` plugin, and `ros2_control` `diff_drive` (ARCHITECTURE.md:98). They are selected by one xacro arg, `use_ros2_control` (default `false`, `ugv.urdf.xacro:5`), through two `xacro:if` blocks (`ugv.urdf.xacro:94-109` adds the `<ros2_control>` system; `ugv.gazebo.xacro:65-72` adds `gz_ros2_control`) and one `xacro:unless` (`ugv.gazebo.xacro:73-83` adds `OdometryPublisher` on gz topics `${ns}/odom`, `${ns}/tf`, 50 Hz). `sim.launch.py` mirrors the same flag with three `IfCondition(use_ros2_control)` nodes: two `controller_manager` spawners (`joint_state_broadcaster`, `diff_drive` with `-r /v1/diff_drive/odom:=/v1/odom`) and `twist_stamper` (`sim.launch.py:43-49`). Of the five xacro args (`ugv.urdf.xacro:4-8`), `sim` gates the whole Gazebo include (`ugv.urdf.xacro:89-92`). The kinematic backend lives in a separate file already, `corridor_nav/launch/kinematic.launch.py`, which takes `n`, `first`, `corridor`, `lookahead_m`, `speed_mps` (README.md:42; `kinematic.launch.py:40-45`). Since 0715c46 and e26d317 the follower navigates on `/vN/fix` and `/vN/imu`; odom is heading/speed only (ARCHITECTURE.md:90, :98), so which node publishes odom no longer matters to navigation, only to launch hygiene.

## Decision (proposed)

- Vehicle-layer contract per namespace `vN`: `/vN/odom`, `/vN/fix`, `/vN/cmd_vel`, `/tf` (ARCHITECTURE.md:98) plus `/vN/imu` (ARCHITECTURE.md:90). Every backend must publish these. Known divergence today: the Gazebo backends emit tf on gz `${ns}/tf` (`ugv.gazebo.xacro:79`; diff_drive `tf_odometry` remapped to `/${ns}/tf` in 2fac34b), bridged to ROS `/v1/tf` (`bridge_v1.yaml:37-41`), while `unicycle_sim` broadcasts a `map -> base_link` transform on `/tf` (`unicycle_sim.py:3,44,103`). The split must settle the tf topic and frame, which this ADR does not.
- Three launch files, one per backend, no shared boolean switch:
  - `corridor_nav/launch/kinematic.launch.py` (exists): `unicycle_sim` per vehicle.
  - `corridor_gz/launch/gz_plugin.launch.py` (rename of today's `sim.launch.py` default path): model with `OdometryPublisher` + `JointStatePublisher` (`ugv.gazebo.xacro:84-88`), no spawners, no `twist_stamper`.
  - `corridor_gz/launch/ros2_control.launch.py`: model with `gz_ros2_control` + `<ros2_control>` block, `joint_state_broadcaster`, `diff_drive` remapped to `/v1/odom`, `twist_stamper` (today's `sim.launch.py:43-49`, unconditional).
- `ugv.urdf.xacro` keeps only the chassis and sensor mounts; the Gazebo and ros2_control blocks move to two includes (`ugv.gazebo.xacro` split into `gz_plugin` and `ros2_control` variants) chosen by the launch file via the xacro file path, not by `xacro:if/unless`. The `use_ros2_control` arg and the three `IfCondition` nodes are deleted. `sim` arg is dropped too; `display.launch.py` already runs the xacro with no args (`display.launch.py:12-13`) and keeps loading the bare chassis.
- Shared pieces (`gz sim -s -r`, `robot_state_publisher`, `ros_gz_sim create`, `parameter_bridge` with `bridge_v1.yaml`, `foxglove_bridge`; `sim.launch.py:31-41`) go into one included launch so the two Gazebo files do not duplicate them.

## Alternatives considered

- **Keep the `use_ros2_control` flag (status quo).** One file, one xacro, but the backend choice is spread over 4 conditional sites in 3 files (`ugv.urdf.xacro:94`, `ugv.gazebo.xacro:65,73`, `sim.launch.py:43-49`); a change to one backend risks the other. ARCHITECTURE.md:98 names this as the thing to replace.
- **Flag only in launch, xacro always emits both plugins.** Rejected: gz would load both `OdometryPublisher` and `diff_drive`, two writers of `/v1/odom` and `/v1/tf`. 2fac34b wrapped `OdometryPublisher` in `xacro:unless` so that "with ros2_control, diff_drive owns odom and tf" (commit message); the commit does not spell out a further rationale.
- **One launch file with a string `backend:=` arg and `OpaqueFunction`.** Not evaluated. Fewer files, but the conditional moves rather than disappears, and the xacro still needs a switch.

## Consequences

- Good: each backend is readable top to bottom; `ros2 launch corridor_gz ros2_control.launch.py` carries no hidden default. Review and proofs (steps/*) can target one file. The model xacro becomes backend-free and reusable on real hardware.
- Good: nothing in `corridor_nav`/`corridor_planner` changes; they already read only the contract topics (ARCHITECTURE.md:90-91).
- Bad: three launch entry points and two Gazebo xacro variants to keep in sync; the shared include is the only guard against drift. README.md:38-45 table, the Foxglove paragraph (README.md:51, "Both launch files start `foxglove_bridge`") and the steps that call `sim.launch.py` need updating: `steps/08_proofs.sh:104,120`, `steps/10_scene_proof.sh:10`, `steps/11_live_model_proof.sh:12`, `steps/12_gazebo_patrol.sh:12`.
- Bad: the rename breaks `ros2 launch corridor_gz sim.launch.py` muscle memory and the step scripts above; keep a one-line deprecation shim or update them in the same change.
- Not done yet: no commit implements this; ARCHITECTURE.md:98 says "The launch split is still to do."

## Evidence

- ARCHITECTURE.md:98 (open question 1), :90-91 (fix/imu navigation), :23 (diagram: `/vN/odom /vN/fix /vN/cmd_vel`).
- ws/src/corridor_description/urdf/ugv.urdf.xacro:4-8, 89-109; ws/src/corridor_description/urdf/ugv.gazebo.xacro:65-88.
- ws/src/corridor_description/launch/display.launch.py:12-13.
- ws/src/corridor_gz/launch/sim.launch.py:20-49; ws/src/corridor_gz/config/bridge_v1.yaml:32-41.
- ws/src/corridor_nav/launch/kinematic.launch.py:24, 40-45; ws/src/corridor_sim/corridor_sim/unicycle_sim.py:3, 41-44, 103; README.md:38-51.
- steps/08_proofs.sh:104,120; steps/10_scene_proof.sh:10; steps/11_live_model_proof.sh:12; steps/12_gazebo_patrol.sh:12.
- Commits: 5773e99 (2026-10-24, open questions), 0715c46, e26d317 (2026-10-25, fix/imu decision), 7f38212 (use_ros2_control arg, issue #5), 2fac34b (OdometryPublisher off under ros2_control), 1d076f7, 5d65cf7 (spawners, twist_stamper), aa6a247 (kinematic n vehicles, issue #10), 42ad4f1 (first:=N beside Gazebo v1).

## Sources

ARCHITECTURE.md; README.md; ws/src/corridor_description/urdf/ugv.urdf.xacro; ws/src/corridor_description/urdf/ugv.gazebo.xacro; ws/src/corridor_description/launch/display.launch.py; ws/src/corridor_gz/launch/sim.launch.py; ws/src/corridor_gz/config/bridge_v1.yaml; ws/src/corridor_nav/launch/kinematic.launch.py; ws/src/corridor_sim/corridor_sim/unicycle_sim.py; steps/08_proofs.sh, 10_scene_proof.sh, 11_live_model_proof.sh, 12_gazebo_patrol.sh; git log (commits listed above).
