# ros2-corridor-swarm on Thor

ROS 2 Jazzy project, fully simulated. Architecture in ARCHITECTURE.md, plan with a proof per step in PLAN.md, progress as GitHub issues (`gh issue list`). Göksel watches the tmux session; keep going without asking unless a step is destructive or needs his decision.

## Environment

- Jetson Thor: JetPack 7, Ubuntu 24.04, aarch64, 14 cores, 122 GB RAM, about 72 GB free on disk. A desktop machine other people also use: never reboot, never stop or restart processes, containers or services you did not start.
- ROS 2 Jazzy is a SOURCE build at `~/.goske/ros2_jazzy` (350 packages: rviz2, turtlesim, demo nodes, tf2; no ros_gz, no Nav2, no ros2_control). Activate it with `source ~/.goske/ros2_jazzy/install/setup.bash`. Never install ROS from apt into /opt/ros; it would shadow the source build. Extra ROS packages are cloned into `ws/src` via `ws/deps.repos` and built with colcon.
- Gazebo Harmonic comes from the OSRF apt repo. It is not a ROS package and does not shadow the source build. Headless only (`gz sim -s`).
- Do not edit `~/.bashrc`. Put the environment in `./env.sh` and source that.
- Work only under `~/deletdis/ros2-corridor-swarm`. Install apt packages only when a build needs them, and name them in the commit message.

## Working rules

- Git: canonical repo is GitHub `origin`, branch `main`. Thor's checkout and the Mac mirror both push there.
- A step is done when its proof in PLAN.md passes from a clean `colcon build`. Commit after every proven step with the proof command and its output in the body. Close the issue with the same output.
- A new test must be seen red with the fix reverted before it counts.
- Claude API: `claude-opus-5-5` for planning, `claude-haiku-5-5` for the status loop, both with `fallbacks: "default"` under beta `server-side-fallback-2026-07-01`. Check `stop_reason` before reading `content`. The key comes from `ANTHROPIC_API_KEY` in the shell, never from the repo.
- Steps run as scripts in `steps/` from the tmux session `ros2` (windows `work` and `run`); logs in `~/deletdis/log/`.
