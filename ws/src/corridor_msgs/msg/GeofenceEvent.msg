std_msgs/Header header
string vehicle
string corridor_id
uint8 EXIT=0        # left the polygon
uint8 ENTER=1       # came back
uint8 WARN=2        # within warn_margin_m of the edge
uint8 type
float32 distance_m  # signed distance to the polygon edge, negative outside
sensor_msgs/NavSatFix fix
