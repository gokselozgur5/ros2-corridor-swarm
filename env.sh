# source this file: . env.sh
for _ros in "$HOME/.goske/ros2_jazzy/install/setup.bash" /opt/ros/jazzy/setup.bash; do
  if [ -f "$_ros" ]; then source "$_ros"; break; fi
done
_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ws"
[ -f "$_ws/install/setup.bash" ] && source "$_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
_env="$(dirname "${BASH_SOURCE[0]}")/.env"; if [ -f "$_env" ]; then set -a; . "$_env"; set +a; fi; unset _env _ws _ros
