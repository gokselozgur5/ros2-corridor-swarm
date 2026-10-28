# Architecture decision records

One file per decision, numbered, never edited after acceptance; a change is a new ADR that supersedes. Status Proposed = ARCHITECTURE.md open question, to be decided before it is coded.

- [ADR-0001: Gazebo Harmonic instead of Isaac Sim, headless only](0001-gazebo-harmonic-instead-of-isaac-sim-headless-only.md) (Accepted)
- [ADR-0002: ROS 2 Jazzy from source, never /opt/ros; extra packages via ws/deps.repos](0002-ros-2-jazzy-from-source-never-opt-ros-extra-packages-via-ws-.md) (Accepted)
- [ADR-0003: foxglove_bridge 0.8.5 from source with BUILD_TESTING=OFF](0003-foxglove-bridge-0-8-5-from-source-with-build-testing-off.md) (Accepted)
- [ADR-0004: Pure-pursuit waypoint follower now, Nav2 deferred](0004-pure-pursuit-waypoint-follower-now-nav2-deferred.md) (Accepted)
- [ADR-0005: Claude plans, ROS executes: planner sends goals only, never cmd_vel](0005-claude-plans-ros-executes-planner-sends-goals-only-never-cmd.md) (Accepted)
- [ADR-0006: Models: Opus 5.5 planning, Haiku 5.5 status loop, fallbacks default, refusal is an outcome](0006-models-opus-5-5-planning-haiku-5-5-status-loop-fallbacks-def.md) (Accepted)
- [ADR-0007: Keep the kinematic unicycle_sim as the no-Gazebo backend; same topic contract on all backends](0007-keep-the-kinematic-unicycle-sim-as-the-no-gazebo-backend-sam.md) (Accepted)
- [ADR-0008: ros2_control diff_drive with twist_stamper (TwistStamped) for the Gazebo vehicle](0008-ros2-control-diff-drive-with-twist-stamper-twiststamped-for-.md) (Accepted)
- [ADR-0009: Follower navigates on GNSS fix + IMU heading, not odometry (#19)](0009-follower-navigates-on-gnss-fix-imu-heading-not-odometry-19.md) (Accepted)
- [ADR-0010: Mission layer positions by fix; follower reports its route id; one patrol goal at a time; stop semantics](0010-mission-layer-positions-by-fix-follower-reports-its-route-id.md) (Accepted)
- [ADR-0011: Skid-steer lateral friction mu2=0.15 as a tuning knob](0011-skid-steer-lateral-friction-mu2-0-15-as-a-tuning-knob.md) (Accepted)
- [ADR-0012: CoT over raw TCP without pytak; FreeTAKServer arm64 image in a named Docker container](0012-cot-over-raw-tcp-without-pytak-freetakserver-arm64-image-in-.md) (Accepted)
- [ADR-0013: Hosts, users and keys only from .env, never in the repo](0013-hosts-users-and-keys-only-from-env-never-in-the-repo.md) (Accepted)
- [ADR-0014: Proof per step, atomic commits, every new test seen red by mutation](0014-proof-per-step-atomic-commits-every-new-test-seen-red-by-mut.md) (Accepted)
- [ADR-0015: fleet_state node publishes /fleet/state; planner and status_loop read it](0015-fleet-state-node-publishes-fleet-state-planner-and-status-lo.md) (Proposed)
- [ADR-0016: Geofence breach cancels at the mission layer; only the mission layer talks to the follower](0016-geofence-breach-cancels-at-the-mission-layer-only-the-missio.md) (Proposed)
- [ADR-0017: goto becomes an action like patrol](0017-goto-becomes-an-action-like-patrol.md) (Proposed)
- [ADR-0018: One launch file per vehicle backend, no xacro conditionals](0018-one-launch-file-per-vehicle-backend-no-xacro-conditionals.md) (Proposed)
