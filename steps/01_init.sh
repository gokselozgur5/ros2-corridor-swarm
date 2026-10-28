#!/usr/bin/env bash
# Day 1 step 1: git repo, env.sh, empty workspace.
set -eo pipefail
cd "${CORRIDOR_ROOT:-$HOME/deletdis/ros2-corridor-swarm}"
[ -d .git ] || git init -q
git config user.name "Göksel Özgür" && git config user.email "gokselozgur@novit.ai"
mkdir -p ws/src
cat > env.sh <<'ENV'
# source this file: . env.sh
source "$HOME/.goske/ros2_jazzy/install/setup.bash"
_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws"
[ -f "$_ws/install/setup.bash" ] && source "$_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ENV
printf 'ws/build/\nws/install/\nws/log/\n__pycache__/\n' > .gitignore
. ./env.sh
echo "ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
ros2 pkg list | wc -l | xargs echo "packages visible:"
git add -A && git commit -q -m "chore: workspace skeleton and env.sh" && git log --oneline -1
