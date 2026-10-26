"""waypoint_follower against real rclpy (Thor only): the follower and a rig node under one executor in a
private namespace per test; the rig feeds path, fix and imu and collects cmd_vel and the status topics.

    source ws/install/setup.bash && ROS_DOMAIN_ID=77 python3 -m pytest -q ws/src/corridor_nav/test -m ros
"""
import itertools
import threading
import time

import pytest

from conftest import ROS, until

if not ROS:
    pytest.skip('needs ROS 2: run on Thor', allow_module_level=True)

from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import Imu, NavSatFix  # noqa: E402
from std_msgs.msg import Header, String  # noqa: E402

from corridor_nav.waypoint_follower import WaypointFollower  # noqa: E402

pytestmark = pytest.mark.ros

GEOJSON = '''{"type":"FeatureCollection","features":[{"type":"Feature","properties":{"id":"demo"},
"geometry":{"type":"Polygon","coordinates":[[[-3.0000,52.7000],[-2.9970,52.7000],[-2.9970,52.7018],[-3.0000,52.7018],[-3.0000,52.7000]]]}}]}'''
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
TIMEOUT_S = 0.5
namespaces = itertools.count(1)


class Rig:
    def __init__(self, ns):
        self.follower = WaypointFollower(namespace=ns, parameter_overrides=[
            Parameter('corridor_geojson', value=GEOJSON), Parameter('pose_timeout_s', value=TIMEOUT_S)])
        self.node = Node('rig', namespace=ns)
        self.path_pub = self.node.create_publisher(Path, 'path', 10)
        self.fix_pub = self.node.create_publisher(NavSatFix, 'fix', 10)
        self.imu_pub = self.node.create_publisher(Imu, 'imu', 10)
        self.cmds = []
        self.routes = []
        self.statuses = []
        self.node.create_subscription(Twist, 'cmd_vel', self.cmds.append, 10)
        self.node.create_subscription(Header, 'follower/route', self.routes.append, LATCHED)
        self.node.create_subscription(String, 'follower/status', lambda m: self.statuses.append(m.data), LATCHED)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.follower)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        assert until(lambda: self.path_pub.get_subscription_count() > 0 and self.fix_pub.get_subscription_count() > 0
                     and self.imu_pub.get_subscription_count() > 0, timeout=5.0), 'follower not discovered'

    def send_path(self, points, sec=0, nanosec=0):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
        for x, y in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def feed(self, seconds, x, y, fix=True, imu=True):
        """Publish the fix for map (x, y) and/or an east-facing imu at 20 Hz; returns the commands received meanwhile."""
        lat, lon = self.follower.frame.to_latlon(x, y)
        n = len(self.cmds)
        end = time.time() + seconds
        while time.time() < end:
            if fix:
                self.fix_pub.publish(NavSatFix(latitude=lat, longitude=lon))
            if imu:
                self.imu_pub.publish(Imu())   # identity orientation: yaw 0, east
            time.sleep(0.05)
        return self.cmds[n:]

    def close(self):
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(5.0)
        self.follower.destroy_node()
        self.node.destroy_node()


@pytest.fixture
def rig(ros):
    r = Rig(f'/wf{next(namespaces)}')
    yield r
    r.close()


def moving(cmds):
    return [c for c in cmds if c.linear.x > 0.0]


def stopped(cmds):
    return [c for c in cmds if c.linear.x == 0.0 and c.angular.z == 0.0]


# ----- nav-follow-3: the heading needs a watchdog too, and silence is not a plan -------------------

def test_stale_imu_holds_even_with_a_live_fix(rig, capfd):
    rig.send_path([(20.0, 20.0), (100.0, 20.0)])
    assert moving(rig.feed(1.0, 20.0, 20.0)), 'not driving on fresh fix and imu'
    late = rig.feed(3 * TIMEOUT_S, 25.0, 20.0, imu=False)[-5:]
    assert late and len(stopped(late)) == len(late), f'kept driving on a frozen heading: {late}'
    assert 'imu stale' in capfd.readouterr().err
    assert moving(rig.feed(1.0, 25.0, 20.0)[-5:]), 'did not resume when the imu came back'


def test_following_without_a_fix_or_imu_warns_and_does_not_drive(rig, capfd):
    rig.send_path([(20.0, 20.0), (100.0, 20.0)])
    assert until(lambda: 'following' in rig.statuses, timeout=3.0)
    time.sleep(1.0)
    assert rig.cmds == []
    assert 'no fix' in capfd.readouterr().err


# ----- nav-mission-2: a status the owner of a Path can attribute to its own route ------------------

def reports(rig):
    return [(r.stamp.sec, r.stamp.nanosec, r.frame_id) for r in rig.routes]


def test_route_report_carries_the_stamp_of_the_path_it_is_about(rig):
    rig.send_path([(20.0, 20.0), (100.0, 20.0)], sec=5, nanosec=7)
    assert until(lambda: (5, 7, 'following') in reports(rig), timeout=3.0), reports(rig)
    rig.send_path([], sec=9)   # someone else's empty Path
    assert until(lambda: (9, 0, 'idle') in reports(rig), timeout=3.0), reports(rig)
    assert rig.statuses[-1] == 'idle'


def test_arrival_is_reported_for_the_path_that_arrived(rig):
    rig.send_path([(20.0, 20.0), (22.0, 20.0)], sec=5, nanosec=7)
    rig.feed(1.0, 21.5, 20.0)   # within the 1 m arrive tolerance of the end
    assert until(lambda: (5, 7, 'arrived') in reports(rig), timeout=3.0), reports(rig)
    assert rig.statuses[-1] == 'arrived'
