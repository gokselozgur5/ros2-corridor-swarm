"""patrol_server against real rclpy (Thor only): an executor spins the server, a fake follower and a client
node in a private namespace per test. Needs the built workspace for corridor_msgs:

    source ws/install/setup.bash && ROS_DOMAIN_ID=77 python3 -m pytest -q ws/src/corridor_nav/test -m ros
"""
import itertools
import math
import threading
import time

import pytest

from conftest import ROS, until

if not ROS:
    if __import__('os').environ.get('REQUIRE_ROS'):
        pytest.exit('REQUIRE_ROS is set but rclpy is absent: source the ROS workspace first', returncode=3)
    pytest.skip('needs ROS 2: run on Thor', allow_module_level=True)

from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from std_msgs.msg import Header, String  # noqa: E402

from corridor_msgs.action import Patrol  # noqa: E402
from corridor_msgs.msg import GeofenceEvent  # noqa: E402
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
        self.event_pub = self.node.create_publisher(GeofenceEvent, '/fleet/geofence_events', 10)
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

    def empties(self):
        return [p for p in self.follower.paths if not p.poses]

    def foreign_path(self, points, sec=123):
        """A Path from another publisher (the planner's goto, or its stop when empty)."""
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp.sec = sec
        for x, y in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        n = len(self.follower.paths)
        self.path_pub.publish(msg)
        assert until(lambda: len(self.follower.paths) > n, timeout=3.0), 'foreign Path not delivered'

    def breach(self):
        assert until(lambda: self.event_pub.get_subscription_count() > 0, timeout=3.0), 'server not subscribed to events'
        self.event_pub.publish(GeofenceEvent(vehicle='v1', corridor_id='demo', type=GeofenceEvent.EXIT, distance_m=-3.0))

    def patrolling(self, laps=1):
        """A goal under way: route delivered to the follower, which reports following it."""
        self.place(20.0, 20.0)
        gh = self.goal(laps=laps)
        assert until(self.routes, timeout=5.0), 'no route reached the follower'
        return gh

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


# ----- nav-mission-2: only a status about the server's own route means anything -----------------

def test_own_arrival_completes_the_lap(rig):
    gh = rig.patrolling(laps=1)
    rig.follower.report('arrived')
    res = rig.result(gh)
    assert res.status == GoalStatus.STATUS_SUCCEEDED
    assert (res.result.result, res.result.laps_done) == ('done', 1)


def test_foreign_arrival_is_not_a_lap_and_does_not_resend_the_route(rig):
    gh = rig.patrolling(laps=2)
    rig.foreign_path([(20.0, 20.0), (60.0, 60.0)])   # the planner's goto took the follower
    rig.follower.report('arrived')                      # the goto arrives
    res = rig.result(gh)
    assert (res.result.result, res.result.laps_done) == ('preempted', 0)
    time.sleep(0.7)
    assert len(rig.routes()) == 2, 'patrol route re-sent over the goto'


def test_foreign_empty_path_ends_the_goal_as_preempted_not_done(rig):
    gh = rig.patrolling(laps=1)
    rig.foreign_path([], sec=0)   # wasd, clicked_goal or a planner stop on a goal it does not track
    res = rig.result(gh)
    assert res.status == GoalStatus.STATUS_ABORTED and res.result.result == 'preempted'


def test_geofence_exit_clears_the_route_when_the_monitor_does_not(rig):
    gh = rig.patrolling(laps=0)
    rig.breach()   # cancel_on_exit false: no empty Path from the monitor
    res = rig.result(gh)
    assert res.status == GoalStatus.STATUS_ABORTED and res.result.result == 'geofence'
    assert until(rig.empties, timeout=3.0), 'the vehicle keeps driving the lap outside the corridor'


def test_geofence_exit_is_geofence_even_when_the_monitor_stopped_the_follower_first(rig):
    gh = rig.patrolling(laps=0)
    rig.breach()
    rig.foreign_path([], sec=0)   # the monitor's own empty Path, right after its event
    assert rig.result(gh).result.result == 'geofence'


def test_cancel_does_not_clear_a_route_the_server_no_longer_owns(rig):
    gh = rig.patrolling(laps=0)
    rig.foreign_path([(20.0, 20.0), (60.0, 60.0)])   # a goto replaced the patrol route
    rig.cancel(gh)
    rig.result(gh)
    time.sleep(0.7)
    assert rig.empties() == [], 'cancel killed the goto with an empty Path'


# ----- nav-mission-1: one goal at a time, and the clear goes out before the cancel is answered --------

def pose_counts(rig):
    return [len(p.poses) for p in rig.follower.paths]


def test_new_goal_preempts_the_running_one_without_clearing_the_follower(rig):
    a = rig.patrolling(laps=0)
    rig.goal(laps=0)
    res = rig.result(a)
    assert res.status == GoalStatus.STATUS_ABORTED and res.result.result == 'preempted'
    assert until(lambda: len(rig.routes()) == 2, timeout=5.0), 'the new goal published no route'
    time.sleep(0.7)
    assert pose_counts(rig) == [5, 5], 'an empty Path went out around the new route'


def test_cancel_is_answered_after_the_clear_so_a_successor_route_survives(rig):
    a = rig.patrolling(laps=0)
    rig.cancel(a)                                     # the planner's stop, awaited
    rig.foreign_path([(20.0, 20.0), (60.0, 60.0)])    # its goto, sent on the answer
    res = rig.result(a)
    assert res.status == GoalStatus.STATUS_CANCELED and res.result.result == 'cancelled'
    time.sleep(0.7)
    assert pose_counts(rig) == [5, 0, 2], 'the goto did not survive the cancel'


def test_planner_sequence_cancel_then_patrol_leaves_the_new_route_in_place(rig):
    a = rig.patrolling(laps=0)
    rig.cancel(a)
    rig.goal(laps=0)
    assert rig.result(a).status == GoalStatus.STATUS_CANCELED
    assert until(lambda: len(rig.routes()) == 2, timeout=5.0), 'the new goal published no route'
    time.sleep(0.7)
    assert pose_counts(rig) == [5, 0, 5], 'the new patrol route was overwritten'
