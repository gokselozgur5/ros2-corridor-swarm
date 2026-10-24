# One vehicle as the mission layer sees it.
std_msgs/Header header
string name                      # namespace without the slash, e.g. v1
sensor_msgs/NavSatFix fix
geometry_msgs/Pose pose          # in the map frame
geometry_msgs/Twist velocity     # body frame
float32 battery_pct              # 0..100, simulated
string mode                      # idle | patrol | goto | stopped | geofence_hold
string goal_id                   # active action goal, empty if none
bool inside_corridor
