"""map_overlay draws each vehicle where its fix says it is (#19): the heading line and the 3D arrow start
at the fix's map position. Wheel-odometry x/y begin at the spawn (0,0 on the ros2_control backend) and
drift with slip, so a tip computed from them lands far from the dot drawn at the fix.
Runs without ROS (conftest stand-ins) and with it (real messages); nothing here needs a live node."""
import json
import math
from types import SimpleNamespace as NS

from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from corridor_nav.geofence import LocalFrame
from corridor_nav.map_overlay import MapOverlay

FRAME = LocalFrame(52.7, -3.0)


def overlay(odom_xy, yaw, fix_xy, length=12.0):
    """A MapOverlay without Node.__init__: the state publish() reads, fake publishers that capture."""
    node = MapOverlay.__new__(MapOverlay)
    node.frame, node.corridor_id = FRAME, 'demo'
    node.poly = [(0.0, 0.0), (200.0, 0.0), (200.0, 200.0)]
    node.vehicles, node.pose = ['v1'], {}
    node.routes = {'v1': [(10.0, 10.0), (100.0, 10.0), (100.0, 100.0)]}
    node.get_parameter = lambda _name: NS(value=length)
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: Header().stamp))
    out = {}
    node.geo_pub = NS(publish=lambda m: out.__setitem__('geo', m))
    node.marker_pub = NS(publish=lambda m: out.__setitem__('markers', m))
    odom = Odometry()
    odom.pose.pose.position.x, odom.pose.pose.position.y = odom_xy
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    node.on_odom('v1', odom)
    lat, lon = FRAME.to_latlon(*fix_xy)
    node.pose['v1'].update(lat=lat, lon=lon)
    node.publish()
    return out


def test_heading_line_and_arrow_start_at_the_fix_not_at_odom():
    out = overlay(odom_xy=(0.0, 0.0), yaw=0.0, fix_xy=(20.0, 20.0))
    features = json.loads(out['geo'].geojson)['features']
    line = next(f for f in features if f['properties']['name'] == 'v1 heading')['geometry']['coordinates']
    start, tip = [FRAME.to_xy(lat, lon) for lon, lat in line]
    assert math.dist(start, (20.0, 20.0)) < 0.03 and math.dist(tip, (32.0, 20.0)) < 0.03, (start, tip)
    arrow = next(m for m in out['markers'].markers if m.ns == 'v1_heading')
    assert (round(arrow.points[0].x, 3), round(arrow.points[0].y, 3)) == (20.0, 20.0)
    assert (round(arrow.points[1].x, 3), round(arrow.points[1].y, 3)) == (32.0, 20.0)
