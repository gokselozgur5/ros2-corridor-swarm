"""patrol_server on the stub rclpy from conftest: callbacks are called directly, the execute loop runs in a
thread behind a fake goal handle, and a fake follower answers on the topics the server subscribes to.
The same scenarios run against real rclpy in test_patrol_server_node.py (Thor)."""
import math
import threading
import types

import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, String

from conftest import until
from corridor_msgs.action import Patrol
from corridor_nav.patrol_server import PatrolServer, inset_polygon

GEOJSON = '''{"type":"FeatureCollection","features":[{"type":"Feature","properties":{"id":"demo"},
"geometry":{"type":"Polygon","coordinates":[[[-3.0000,52.7000],[-2.9970,52.7000],[-2.9970,52.7018],[-3.0000,52.7018],[-3.0000,52.7000]]]}}]}'''


class FakeHandle:
    """The slice of rclpy's ServerGoalHandle the server touches; execute() runs the callback in a thread
    the way the executor does."""
    count = 0

    def __init__(self, srv, laps=1, clockwise=False):
        FakeHandle.count += 1
        self.srv = srv
        self.goal_id = types.SimpleNamespace(uuid=FakeHandle.count.to_bytes(16, 'big'))
        self.request = Patrol.Goal(vehicle='v1', corridor_id='demo', laps=laps, clockwise=clockwise)
        self.cancel_requested = False
        self.state = 'executing'
        self.feedback = []
        self.result = None
        self.error = None
        self.thread = None

    @property
    def is_cancel_requested(self):
        return self.cancel_requested

    def execute(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            self.result = self.srv.server.execute_callback(self)
        except Exception as exc:   # rclpy would log it and abort the goal
            self.error = exc
            self.state = 'aborted'

    def succeed(self):
        self.state = 'succeeded'

    def abort(self):
        self.state = 'aborted'

    def canceled(self):
        if not self.cancel_requested:
            raise RuntimeError('canceled() before the goal reached CANCELING')
        self.state = 'canceled'

    def publish_feedback(self, fb):
        self.feedback.append(fb)

    def finished(self):
        return self.thread is not None and not self.thread.is_alive()


def make_server(node_params, inset=30.0):
    node_params(corridor_geojson=GEOJSON, vehicle='v1', inset_m=inset)
    return PatrolServer()


def accept(srv, handle):
    """What rclpy does once on_goal accepts: handle_accepted_callback, by default handle.execute()."""
    (srv.server.handle_accepted_callback or (lambda h: h.execute()))(handle)


def fix_at(srv, x, y):
    lat, lon = srv.frame.to_latlon(x, y)
    return NavSatFix(latitude=lat, longitude=lon)


def odom_at(x, y):
    odom = Odometry()
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    return odom


def routes(srv):
    return [p for p in srv.publishers['path'].sent if len(p.poses) >= 2]


def report(srv, status, stamp=None):
    """The follower reports `status` for the Path stamped `stamp` (default: the server's last Path)."""
    if stamp is None:
        stamp = srv.publishers['path'].sent[-1].header.stamp
    if 'follower/route' in srv.subscriptions:
        srv.subscriptions['follower/route'](Header(stamp=stamp, frame_id=status))
    if 'follower/status' in srv.subscriptions:
        srv.subscriptions['follower/status'](String(data=status))


def stop(srv, handle):
    """Cancel the goal the way rclpy does and wait for its loop to exit."""
    if not handle.finished():
        srv.server.cancel_callback(handle)
        handle.cancel_requested = True
    assert until(handle.finished, timeout=3.0), 'execute loop did not exit'
    assert handle.error is None, handle.error


def first_vertex(path):
    return path.poses[0].pose.position.x, path.poses[0].pose.position.y


# ----- nav-mission-3: position from the fix -----------------------------------------------------

def test_lap_starts_at_the_vertex_nearest_the_fix_not_the_odometry(node_params):
    srv = make_server(node_params)
    # ros2_control odometry starts at its spawn origin; the fix puts the vehicle in the NE corner of the corridor.
    if 'odom' in srv.subscriptions:
        srv.subscriptions['odom'](odom_at(0.0, 0.0))
    ne = srv.poly[2]
    here = (ne[0] - 5.0, ne[1] - 5.0)
    srv.subscriptions['fix'](fix_at(srv, *here))
    handle = FakeHandle(srv, laps=1)
    accept(srv, handle)
    assert until(lambda: routes(srv)), 'no route published'
    nearest = min(inset_polygon(srv.poly, 30.0), key=lambda v: math.dist(v, here))
    assert first_vertex(routes(srv)[0]) == pytest.approx(nearest)
    stop(srv, handle)


def test_feedback_distance_and_next_waypoint_follow_the_fix(node_params):
    srv = make_server(node_params)
    srv.subscriptions['fix'](fix_at(srv, 20.0, 20.0))
    handle = FakeHandle(srv, laps=1)
    accept(srv, handle)
    assert until(lambda: routes(srv))
    route = [(ps.pose.position.x, ps.pose.position.y) for ps in routes(srv)[0].poses]
    report(srv, 'following')
    # 10 m along the first leg from the start vertex: distance grows, the next waypoint is the leg's end.
    sx, sy = route[0]
    ex, ey = route[1]
    leg = math.dist(route[0], route[1])
    srv.subscriptions['fix'](fix_at(srv, sx + 10.0 * (ex - sx) / leg, sy + 10.0 * (ey - sy) / leg))
    fb = until(lambda: next((f for f in handle.feedback if f.distance_m > 1.0), None))
    assert fb, 'feedback never reflected the fix'
    assert (fb.next_waypoint.x, fb.next_waypoint.y) == pytest.approx(route[1])
    stop(srv, handle)
