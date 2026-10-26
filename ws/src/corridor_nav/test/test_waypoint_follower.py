"""waypoint_follower on the stub rclpy from conftest: callbacks and the control tick are called directly,
the stub clock is advanced by hand. The same scenarios run against real rclpy in test_waypoint_follower_node.py."""
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu, NavSatFix

from corridor_nav.waypoint_follower import WaypointFollower, stale_inputs

GEOJSON = '''{"type":"FeatureCollection","features":[{"type":"Feature","properties":{"id":"demo"},
"geometry":{"type":"Polygon","coordinates":[[[-3.0000,52.7000],[-2.9970,52.7000],[-2.9970,52.7018],[-3.0000,52.7018],[-3.0000,52.7000]]]}}]}'''


def make_follower(node_params, **params):
    node_params(corridor_geojson=GEOJSON, pose_timeout_s=1.0, **params)
    return WaypointFollower()


def path(*points, sec=0, nanosec=0):
    msg = Path()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
    for x, y in points:
        ps = PoseStamped()
        ps.pose.position.x, ps.pose.position.y = x, y
        msg.poses.append(ps)
    return msg


def fix_at(node, x, y):
    lat, lon = node.frame.to_latlon(x, y)
    node.on_fix(NavSatFix(latitude=lat, longitude=lon))


def imu_east(node):
    node.on_imu(Imu(orientation=Quaternion(w=1.0)))


def cmds(node):
    return node.publishers['cmd_vel'].sent


def warnings(node):
    return [text for level, text in node.logger.lines if level == 'warn']


# ----- nav-follow-3: the heading needs a watchdog too, and silence is not a plan -------------------

def test_stale_inputs_names_the_old_ones():
    now = 10_000_000_000
    assert stale_inputs(now, {'fix': now - 100_000_000, 'imu': now - 2_000_000_000}, 1.0) == ['imu']
    assert stale_inputs(now, {'fix': now - 3_000_000_000, 'imu': now - 2_000_000_000}, 1.0) == ['fix', 'imu']
    assert stale_inputs(now, {'fix': now, 'imu': now - 900_000_000}, 1.0) == []


def test_stale_imu_holds_even_with_a_live_fix(node_params):
    f = make_follower(node_params)
    f.on_path(path((20.0, 20.0), (100.0, 20.0)))
    fix_at(f, 20.0, 20.0)
    imu_east(f)
    f.tick()
    assert cmds(f)[-1].linear.x > 0.0   # driving on fresh inputs
    f.clock.advance(2.0)
    fix_at(f, 25.0, 20.0)   # fixes keep coming, the imu stopped
    f.tick()
    assert cmds(f)[-1].linear.x == 0.0 and cmds(f)[-1].angular.z == 0.0
    assert any('imu' in w and 'stale' in w for w in warnings(f))


def test_following_without_a_fix_or_imu_warns_instead_of_silently_doing_nothing(node_params):
    f = make_follower(node_params)
    f.on_path(path((20.0, 20.0), (100.0, 20.0)))
    f.tick()
    assert f.status == 'following'
    assert cmds(f) == []
    assert any('fix' in w for w in warnings(f)), warnings(f)
