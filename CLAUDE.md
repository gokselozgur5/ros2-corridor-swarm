# ros2-corridor-swarm on Thor

One-week ROS 2 Jazzy project, fully simulated. The plan is in README.md. Göksel watches this tmux session; keep going without asking unless a step is destructive or needs his decision.

## Environment

- Jetson Thor: JetPack 7, Ubuntu 24.04, aarch64, 14 cores, 122 GB RAM, about 86 GB free on disk. A desktop machine other people also use: never reboot, never stop or restart processes, containers or services you did not start.
- ROS 2 Jazzy is a SOURCE build at `~/.goske/ros2_jazzy` (350 packages: rviz2, turtlesim, demo nodes, tf2; no ros_gz, no Nav2, no foxglove_bridge). Activate it with `source ~/.goske/ros2_jazzy/install/setup.bash`. Never install ROS from apt into /opt/ros; it would shadow the source build. Extra ROS packages are cloned into `ws/src` and built with colcon.
- Do not edit `~/.bashrc`. Put the environment in `./env.sh` (sources the ROS build and `ws/install/setup.bash`) and source that.
- Work only under `~/deletdis/ros2-corridor-swarm`. Install apt packages only when a build needs them, and name them in the commit message.
- `gz` (Gazebo) is not installed. Day 1 does not need it.

## Day 1

1. `git init` here; commit after every step with the command you ran and its output in the message body.
2. `ws/src/` plus `env.sh`.
3. foxglove_bridge from source: `git clone https://github.com/foxglove/ros-foxglove-bridge.git ws/src/foxglove_bridge` (branch main), resolve its dependencies (rosdep if initialised, else the apt packages it names), `colcon build --packages-select foxglove_bridge`.
4. Package `corridor_sim` (ament_python) with a `vehicle_sim` node: one vehicle moving along a hard-coded 3-waypoint route near 52 N, 2 W at 20 m/s, publishing `sensor_msgs/NavSatFix`, `nav_msgs/Odometry` and a `map -> base_link` tf at 10 Hz. Parameters for speed and the route.
5. `launch/day1.launch.py` starting foxglove_bridge (port 8765, address 0.0.0.0) and vehicle_sim.
6. Prove it: `ros2 topic hz /vehicle/fix`, one `ros2 topic echo --once`, `ss -ltnp | grep 8765`. Leave the launch running in tmux window `run` so Göksel can connect Foxglove from his Mac to `ws://192.168.88.229:8765`.

Report what works and what does not, with the exact commands.
