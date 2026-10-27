"""patrol_server on the stub rclpy from conftest: callbacks are called directly, the execute loop runs in a
thread behind a fake goal handle, and a fake follower answers on the topics the server subscribes to.
The same scenarios run against real rclpy in test_patrol_server_node.py (Thor)."""
import pytest
import rclpy

# On the real stack these scenarios run in test_patrol_server_node.py. A module-level pytest.skip would do,
# but launch_testing's collection hook imports every module first and a Skipped raised there skips the
# whole test directory, node tests included; a marker leaves the other modules alone.
pytestmark = pytest.mark.skipif(hasattr(rclpy, '__file__'),
                                reason='stub-rclpy tests: test_patrol_server_node.py covers the real stack')

import math
import threading
import time
import types

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, String

from conftest import until
from corridor_msgs.action import Patrol
from corridor_msgs.msg import GeofenceEvent
from corridor_nav.patrol_server import PatrolServer, RouteOwner, Run, inset_polygon, verdict

FOREIGN = Time(sec=123, nanosec=0)   # a Path stamped by some other publisher

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


def empties(srv):
    return [p for p in srv.publishers['path'].sent if not p.poses]


def patrolling(node_params, laps=1):
    """A server with a goal under way: route published and the follower following it."""
    srv = make_server(node_params)
    srv.subscriptions['fix'](fix_at(srv, 20.0, 20.0))
    handle = FakeHandle(srv, laps=laps)
    accept(srv, handle)
    assert until(lambda: routes(srv)), 'no route published'
    report(srv, 'following')
    return srv, handle


def breach(srv):
    srv.subscriptions['/fleet/geofence_events'](GeofenceEvent(vehicle='v1', corridor_id='demo', type=GeofenceEvent.EXIT, distance_m=-3.0))


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


# ----- nav-mission-2: only a status about the server's own route means anything -----------------

def test_route_owner_follows_the_stamps():
    o = RouteOwner()
    assert not o.owns()
    s1 = o.next_stamp(5_000)
    o.published(s1, route=True)
    assert o.owns()                      # in flight: ours until the follower says otherwise
    o.report(s1, 'following')
    assert o.owns() and not o.arrived()
    o.report(s1, 'arrived')
    assert o.arrived()
    o.report(123, 'following')           # someone else's Path took the follower
    assert not o.owns() and not o.arrived()
    o.published(o.next_stamp(5_000), route=False)   # a clear is nothing to own
    assert not o.owns()


def test_stamps_are_unique_even_while_the_clock_stands_still():
    o = RouteOwner()
    a = o.next_stamp(0)
    o.published(a, True)
    b = o.next_stamp(0)
    o.published(b, True)
    assert (a, b, o.next_stamp(10**12)) == (1, 3, 10**12 + 1)   # unique, increasing, and odd


def test_verdict_precedence():
    o = RouteOwner()
    s = o.next_stamp(1)
    o.published(s, True)
    o.report(s, 'following')
    run = Run(handle=None, laps=1)
    assert verdict(run, o, False) is None
    o.report(s, 'arrived')
    assert verdict(run, o, False) == 'lap'
    o.report(0, 'idle')
    assert verdict(run, o, False) == 'preempted'
    run.breached = True
    assert verdict(run, o, False) == 'geofence'
    assert verdict(run, o, True) == 'cancelled'


def test_own_arrival_completes_the_lap(node_params):
    srv, handle = patrolling(node_params, laps=1)
    report(srv, 'arrived')
    assert until(handle.finished)
    assert handle.state == 'succeeded' and handle.result.result == 'done' and handle.result.laps_done == 1


def test_foreign_arrival_is_not_a_lap_and_does_not_resend_the_route(node_params):
    srv, handle = patrolling(node_params, laps=2)
    # A goto Path from the planner took the follower; the goto arriving is not a patrol lap.
    report(srv, 'following', stamp=FOREIGN)
    report(srv, 'arrived', stamp=FOREIGN)
    assert until(handle.finished), 'goal still running after another node took the follower'
    assert handle.result.laps_done == 0 and handle.result.result != 'done'
    assert len(routes(srv)) == 1, 'patrol route re-sent over the goto'


def test_foreign_empty_path_ends_the_goal_as_preempted_not_done(node_params):
    srv, handle = patrolling(node_params, laps=1)
    report(srv, 'idle', stamp=Time())   # wasd, clicked_goal or a planner stop on a goal it does not track
    assert until(handle.finished)
    assert handle.state == 'aborted' and handle.result.result == 'preempted'


def test_geofence_exit_clears_the_route_when_the_monitor_does_not(node_params):
    srv, handle = patrolling(node_params, laps=0)
    breach(srv)   # cancel_on_exit false: no empty Path from the monitor
    assert until(handle.finished)
    assert handle.state == 'aborted' and handle.result.result == 'geofence'
    assert empties(srv), 'the vehicle keeps driving the lap outside the corridor'


def test_geofence_exit_is_geofence_even_when_the_monitor_stopped_the_follower_first(node_params):
    srv, handle = patrolling(node_params, laps=0)
    breach(srv)
    report(srv, 'idle', stamp=Time())   # the monitor's own empty Path, published right after its event
    assert until(handle.finished)
    assert handle.result.result == 'geofence'


def test_cancel_does_not_clear_a_route_the_server_no_longer_owns(node_params):
    srv, handle = patrolling(node_params, laps=0)
    report(srv, 'following', stamp=FOREIGN)   # a goto replaced the patrol route
    stop(srv, handle)
    assert empties(srv) == [], 'cancel killed the goto with an empty Path'


# ----- nav-mission-1: one goal at a time, and the clear goes out before the cancel is answered --------

def test_new_goal_preempts_the_running_one_without_clearing_the_follower(node_params):
    srv, a = patrolling(node_params, laps=0)
    b = FakeHandle(srv, laps=0)
    accept(srv, b)
    assert until(a.finished), 'the first goal keeps running beside the second'
    assert a.state == 'aborted' and a.result.result == 'preempted'
    assert until(lambda: len(routes(srv)) == 2), 'the new goal published no route'
    report(srv, 'following')
    time.sleep(0.6)
    assert empties(srv) == [] and not b.finished()
    stop(srv, b)


def test_cancel_clears_the_route_inside_the_cancel_callback(node_params):
    # rclpy answers the client after cancel_callback returns, and the planner publishes its next Path right
    # after that answer: the empty Path must already be out, from the callback's own thread.
    srv, a = patrolling(node_params, laps=0)
    published = []
    original = srv.path_pub.publish
    srv.path_pub.publish = lambda msg: (published.append((msg, threading.current_thread())), original(msg))
    srv.server.cancel_callback(a)
    cleared = [thread for msg, thread in published if not msg.poses]
    assert cleared == [threading.current_thread()], 'the clear is not out when the cancel is answered'
    a.cancel_requested = True
    report(srv, 'following', stamp=FOREIGN)   # the planner's goto, published on the cancel answer
    assert until(a.finished)
    assert a.state == 'canceled' and a.result.result == 'cancelled'
    assert len(empties(srv)) == 1, 'a second empty Path went out after the goto'


def test_planner_sequence_cancel_then_patrol_leaves_the_new_route_in_place(node_params):
    srv, a = patrolling(node_params, laps=0)
    srv.server.cancel_callback(a)   # t_stop, awaited by the planner
    a.cancel_requested = True
    b = FakeHandle(srv, laps=0)     # t_patrol's next goal
    accept(srv, b)
    assert until(lambda: len(routes(srv)) == 2)
    report(srv, 'following')
    assert until(a.finished)
    assert a.state == 'canceled'
    time.sleep(0.6)
    assert srv.publishers['path'].sent[-1] is routes(srv)[1], 'something was published after the new route'
    assert not b.finished()
    stop(srv, b)


# ----- final review: RouteOwner is server-wide, so a clear for a goal that is no longer current must not erase
# the current goal's route (the preempt-timeout and queued-cancel paths)

def test_cancel_of_a_goal_that_is_no_longer_current_does_not_clear_the_current_route(node_params):
    from corridor_nav.patrol_server import goal_key
    srv, a = patrolling(node_params, laps=0)
    b = FakeHandle(srv)
    srv.runs[goal_key(b)] = Run(b, 0)
    srv.current = srv.runs[goal_key(b)]          # b took over (as after the 5 s preempt timeout) ...
    srv.send_route([(0.0, 0.0), (10.0, 0.0)])     # ... and published its route
    before = len(empties(srv))
    srv.server.cancel_callback(a)                 # a's cancel arrives late
    assert len(empties(srv)) == before, "a's cancel erased b's route"


def test_server_stamps_are_odd_so_a_sim_time_path_from_another_node_never_matches():
    owner = RouteOwner()
    s1 = owner.next_stamp(1_000_000)            # a /clock tick: even, what now().to_msg() gives every node
    assert s1 % 2 == 1 and s1 >= 1_000_000
    owner.published(s1, True)
    s2 = owner.next_stamp(1_000_000)
    assert s2 % 2 == 1 and s2 > s1
