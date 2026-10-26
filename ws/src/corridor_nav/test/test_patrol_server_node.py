"""patrol_server against real rclpy (Thor only): an executor spins the server, a fake follower and a client
node in a private namespace per test. Needs the built workspace for corridor_msgs:

    source ws/install/setup.bash && ROS_DOMAIN_ID=77 python3 -m pytest -q ws/src/corridor_nav/test -m ros
"""
import itertools
import math
import threading

import pytest

from conftest import ROS, until

if not ROS:
    pytest.skip('needs ROS 2: run on Thor', allow_module_level=True)

from action_msgs.msg import GoalStatus  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from std_msgs.msg import Header, String  # noqa: E402

from corridor_msgs.action import Patrol  # noqa: E402
from corridor_nav.patrol_server import PatrolServer, inset_polygon  # noqa: E402

pytestmark = pytest.mark.ros

GEOJSON = '''{"type":"FeatureCollection","features":[{"type":"Feature","properties":{"id":"demo"},
"geometry":{"type":"Polygon","coordinates":[[[-3.0000,52.7000],[-2.9970,52.7000],[-2.9970,52.7018],[-3.0000,52.7018],[-3.0000,52.7000]]]}}]}'''
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
namespaces = itertools.count(1)


class FakeFollower(Node):
    """Stands in for waypoint_follower: keeps every Path it gets and reports a status for the one it holds."""

    def __init__(self, ns):
        super().__init__('fake_follower', namespace=ns)
        self.paths = []
        self.held = Header()
        self.status_pub = self.create_publisher(String, 'follower/status', LATCHED)
        self.route_pub = self.create_publisher(Header, 'follower/route', LATCHED)
        self.create_subscription(Path, 'path', self.on_path, 10)

    def on_path(self, msg):
        self.paths.append(msg)
        self.held = msg.header
        self.report('following' if len(msg.poses) >= 2 else 'idle')

    def report(self, status, header=None):
        self.route_pub.publish(Header(stamp=(header or self.held).stamp, frame_id=status))
        self.status_pub.publish(String(data=status))


class Rig:
    """Server, fake follower and a client node under one executor in a private namespace."""

    def __init__(self, ns, inset=30.0):
        self.server = PatrolServer(namespace=ns, parameter_overrides=[
            Parameter('corridor_geojson', value=GEOJSON), Parameter('vehicle', value='v1'), Parameter('inset_m', value=inset)])
        self.follower = FakeFollower(ns)
        self.node = Node('rig', namespace=ns)
        self.client = ActionClient(self.node, Patrol, 'patrol')
        self.fix_pub = self.node.create_publisher(NavSatFix, 'fix', 10)
        self.path_pub = self.node.create_publisher(Path, 'path', 10)
        self.goals = []
        self.executor = MultiThreadedExecutor(num_threads=6)
        for n in (self.server, self.follower, self.node):
            self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        assert self.client.wait_for_server(timeout_sec=5.0), 'patrol server not discovered'

    def place(self, x, y):
        """Publish the fix for map point (x, y) until the server has it."""
        lat, lon = self.server.frame.to_latlon(x, y)

        def arrived():
            self.fix_pub.publish(NavSatFix(latitude=lat, longitude=lon))
            return self.server.pose is not None and math.dist(self.server.pose, (x, y)) < 0.5
        assert until(arrived, timeout=5.0, step=0.05), 'server never got the fix'

    def goal(self, laps=1, clockwise=False):
        fut = self.client.send_goal_async(Patrol.Goal(vehicle='v1', corridor_id='demo', laps=laps, clockwise=clockwise))
        assert until(fut.done, timeout=5.0), 'goal not answered'
        gh = fut.result()
        assert gh.accepted
        self.goals.append(gh)
        return gh

    def result(self, gh, timeout=5.0):
        fut = gh.get_result_async()
        assert until(fut.done, timeout=timeout), 'no result'
        return fut.result()

    def cancel(self, gh):
        fut = gh.cancel_goal_async()
        assert until(fut.done, timeout=5.0), 'cancel not answered'
        return fut.result()

    def routes(self):
        return [p for p in self.follower.paths if len(p.poses) >= 2]

    def close(self):
        for gh in self.goals:
            self.cancel(gh)
            fut = gh.get_result_async()
            until(fut.done, timeout=5.0)
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(5.0)
        for n in (self.server, self.follower, self.node):
            n.destroy_node()


@pytest.fixture
def rig(ros):
    r = Rig(f'/pt{next(namespaces)}')
    yield r
    r.close()


def vertex(path, i):
    return path.poses[i].pose.position.x, path.poses[i].pose.position.y


# ----- nav-mission-3: position from the fix -----------------------------------------------------

def test_lap_starts_at_the_vertex_nearest_the_fix(rig):
    ne = rig.server.poly[2]
    here = (ne[0] - 5.0, ne[1] - 5.0)
    rig.place(*here)
    gh = rig.goal(laps=1)
    assert until(rig.routes, timeout=5.0), 'no route reached the follower'
    nearest = min(inset_polygon(rig.server.poly, 30.0), key=lambda v: math.dist(v, here))
    assert vertex(rig.routes()[0], 0) == pytest.approx(nearest)
    rig.cancel(gh)
    assert rig.result(gh).status == GoalStatus.STATUS_CANCELED
