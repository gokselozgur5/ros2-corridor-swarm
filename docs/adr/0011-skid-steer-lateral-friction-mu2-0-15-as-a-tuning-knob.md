# ADR-0011: Skid-steer lateral friction mu2=0.15 as a tuning knob

Status: Accepted (2026-10-25, commit 5d6bf53; recorded in ARCHITECTURE.md by e26d317)

## Context

The UGV is a four-wheel differential (skid-steer) vehicle (`ugv.urdf.xacro:3`): two wheels per side on `diff_drive_controller` (`wheels_per_side: 2`, `wheel_separation: 0.80`, `wheel_radius: 0.20`, `controllers_v1.yaml:13-17`), wheels at `wheel_x` 0.40 m fore/aft of centre (`ugv.urdf.xacro:18`; "0.8 m apart", `ugv.gazebo.xacro:6`). With Gazebo's default full lateral grip, a 1.0 rad/s in-place turn command produced 0.22 rad/s and the vehicle overshot every corner of the patrol route (ARCHITECTURE.md:92; commit 5d6bf53 message). The same slip against a crate drove wheel odometry yaw to 119° while the IMU read 7° (ARCHITECTURE.md:90), which is why navigation heading was moved to the IMU under #19. The ground plane in `corridor.sdf:34` has `mu=mu2=0.9`.

## Decision

In `ws/src/corridor_description/urdf/ugv.gazebo.xacro:9-19` a `wheel_friction` macro applies to all four wheel collisions (`front_left`, `front_right`, `rear_left`, `rear_right`):
`<ode><mu>1.0</mu><mu2>0.15</mu2><fdir1>1 0 0</fdir1></ode>`.
`fdir1 = 1 0 0` sets the primary friction direction to the wheel's rolling direction (full grip `mu=1.0`); `mu2=0.15` is the coefficient across the rolling direction, letting the tyres slide sideways (`ugv.gazebo.xacro:6-8`). Result: about 0.75 rad/s for the 1.0 rad/s command (ARCHITECTURE.md:92). The value is declared a tuning knob, not physics truth (ARCHITECTURE.md:92). It applies only when `sim:=true` includes `ugv.gazebo.xacro` (`ugv.urdf.xacro:89-90`). No ROS parameter exposes it; changing it means editing the xacro.

## Alternatives considered

- Keep full grip and raise the commanded angular rate / controller limits: `angular.z.max_velocity` is already 2.0 (`controllers_v1.yaml:26`). Nothing in the repo records this being weighed; the commit went straight to the friction change (5d6bf53).
- Model a true steering axle (Ackermann) or caster wheels: not evaluated; `ugv.urdf.xacro:3` commits to a differential 4-wheel UGV and `diff_drive_controller` is the chosen backend (ARCHITECTURE.md:43).
- Tune the drive plugin instead of surface friction: not evaluated; odometry/tf ownership was settled on ros2_control `diff_drive` (commit 2fac34b).

## Consequences

- Good: in-place yaw rate goes 0.22 -> ~0.75 rad/s (ARCHITECTURE.md:92); the Gazebo lap proof `steps/12_gazebo_patrol.sh` passes on `result: done` and distance > 380 m (lines 2-3, 23), which is the #19 proof PLAN.md:11 names.
- Good: matches how a real skid steer scrubs tyres (ARCHITECTURE.md:92; `ugv.gazebo.xacro:6-8`).
- Bad: 0.15 is empirical for this Gazebo world, the `<ode>` friction block and this mass/geometry (`body_m` 60, `wheel_m` 4, `ugv.urdf.xacro:14,17`); a different wheel mass, ground `mu`, or physics engine shifts the achieved rate. No test in the repo references `mu2`, `fdir1` or the yaw rate (grep over `ws/src`, `steps`), although PLAN.md:11 groups this fix with ones "seen red first".
- Bad: lower lateral grip means more sideways drift under lateral loads, so wheel odometry is even less trustworthy; navigation already ignores it (ARCHITECTURE.md:90).
- Bad: the knob lives only in xacro; the kinematic backends `unicycle_sim`/`vehicle_sim` (ARCHITECTURE.md:89-90; `ws/src/corridor_sim/corridor_sim/`) have no friction model, so turn behaviour differs across backends.

## Evidence

- Commit 5d6bf53 (2026-10-25): "corridor_description: low lateral wheel friction (mu2 0.15 along fdir1) so the skid steer can turn (#19)"; message records 0.22 rad/s for 1.0 commanded.
- Commit e26d317 (2026-10-25): ARCHITECTURE decision entry "Skid-steer wheels slide sideways" (ARCHITECTURE.md:92).
- `ws/src/corridor_description/urdf/ugv.gazebo.xacro:6-19` (comment and macro).
- `ws/src/corridor_description/urdf/ugv.urdf.xacro:3,14-18,89-90` (vehicle type, masses/geometry, sim include).
- `ws/src/corridor_gz/config/controllers_v1.yaml:13-28` (wheel geometry, angular limits).
- `ws/src/corridor_gz/worlds/corridor.sdf:34` (ground mu 0.9).
- Proof: `steps/12_gazebo_patrol.sh` (PASS criterion lines 2-3, 23); PLAN.md:11.
- Issue #19.

## Sources

- ARCHITECTURE.md:43, 89, 90, 92
- PLAN.md:11
- ws/src/corridor_description/urdf/ugv.gazebo.xacro:3-19
- ws/src/corridor_description/urdf/ugv.urdf.xacro:3, 14-18, 89-90
- ws/src/corridor_gz/config/controllers_v1.yaml:13-28
- ws/src/corridor_gz/worlds/corridor.sdf:6-7, 34
- steps/12_gazebo_patrol.sh:2-3, 23
- git: 5d6bf53, e26d317, 2fac34b
