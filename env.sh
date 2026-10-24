# source this file: . env.sh
source "$HOME/.goske/ros2_jazzy/install/setup.bash"
_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws"
[ -f "$_ws/install/setup.bash" ] && source "$_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
[ -f "$(dirname "${BASH_SOURCE[0]}")/.env" ] && . "$(dirname "${BASH_SOURCE[0]}")/.env"
