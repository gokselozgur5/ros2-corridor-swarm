"""The planner's tool methods against fake ROS objects (ros_fakes.py). Runs without ROS through the
stand-ins in conftest.py, and against the real message classes on Thor."""
import math
import pathlib
import time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from corridor_nav.geofence import LocalFrame, load_geojson_polygon
from corridor_planner import mission_planner as mp
from corridor_planner.mission_planner import MissionPlanner, Vehicle
from ros_fakes import FakeActionClient, FakeClock, FakeGoalHandle, FakeLogger, FakePublisher

GEOJSON = pathlib.Path(__file__).resolve().parents[2] / 'corridor_nav' / 'config' / 'corridor_demo.geojson'


def planner(log):
    """A MissionPlanner without rclpy: no __init__, the attributes the tools use set by hand."""
    node = MissionPlanner.__new__(MissionPlanner)
    node.corridor_id, ring = load_geojson_polygon(str(GEOJSON))
    node.frame = LocalFrame(*ring[0])
    node.poly = [node.frame.to_xy(lat, lon) for lat, lon in ring]
    node.vehicles = {'v1': Vehicle()}
    node.path_pubs = {'v1': FakePublisher(log)}
    node.patrol_clients = {'v1': FakeActionClient(log)}
    node.get_clock = lambda: FakeClock()
    node.logger = FakeLogger()
    node.get_logger = lambda: node.logger
    return node


# ----- planner-2: stop waits for the old goal to exit before a successor is sent --------------------

def test_stop_returns_only_after_the_old_goal_has_exited():
    log = []
    node = planner(log)
    old = FakeGoalHandle(log, exit_delay_s=0.2)
    node.vehicles['v1'].goal = old
    out = node.t_stop('v1')
    assert out == {'stopped': True, 'vehicle': 'v1', 'cancelled_patrol': True}
    assert old.result_future.done(), f'returned before the old goal exited: {log}'
    assert old.result_requests == 1
    assert node.vehicles['v1'].goal is None


def test_patrol_sends_the_new_goal_after_the_old_one_exited():
    log = []
    node = planner(log)
    node.vehicles['v1'].goal = FakeGoalHandle(log, exit_delay_s=0.2)
    out = node.t_patrol('v1', 0, False)
    assert out['accepted'] is True
    assert 'old_result' in log and log.index('old_result') < log.index('send_goal'), log


def test_goto_publishes_the_path_after_the_old_goal_exited():
    log = []
    node = planner(log)
    v = node.vehicles['v1']
    v.goal = FakeGoalHandle(log, exit_delay_s=0.2)
    v.fix = NavSatFix(latitude=52.7001, longitude=-2.9997)
    v.odom = Odometry()
    lat, lon = node.frame.to_latlon(100.0, 60.0)
    out = node.t_goto('v1', lat, lon)
    assert out['accepted'] is True
    assert 'old_result' in log and log.index('old_result') < log.index('path'), log


def test_stop_is_bounded_when_the_server_never_answers(monkeypatch):
    monkeypatch.setattr(mp, 'STOP_TIMEOUT_S', 0.3)
    log = []
    node = planner(log)
    node.vehicles['v1'].goal = FakeGoalHandle(log, exit_delay_s=None, ack_delay_s=None)
    t0 = time.monotonic()
    out = node.t_stop('v1')
    elapsed = time.monotonic() - t0
    assert 0.3 <= elapsed < 1.5, elapsed
    assert out['cancelled_patrol'] is True and log[-1] == 'path_empty'
    assert any(level == 'warn' for level, _ in node.logger.messages)


def test_stop_goal_reports_which_wait_timed_out():
    log = []
    assert mp.stop_goal(FakeGoalHandle(log, exit_delay_s=0.05), timeout_s=1.0) == 'finished'
    assert mp.stop_goal(FakeGoalHandle(log, exit_delay_s=None, ack_delay_s=None), timeout_s=0.2) == 'cancel_timeout'
    assert mp.stop_goal(FakeGoalHandle(log, exit_delay_s=None), timeout_s=0.2) == 'result_timeout'


# ----- planner-3: the goto Path starts at the fix in the corridor frame, never at odom ------------

def fix_at(frame, x, y):
    lat, lon = frame.to_latlon(x, y)
    return NavSatFix(latitude=lat, longitude=lon)


def test_start_xy_is_the_fix_in_the_corridor_frame():
    frame = LocalFrame(52.7, -3.0)
    x, y = mp.start_xy(frame, fix_at(frame, 20.0, 70.0))
    assert math.isclose(x, 20.0, abs_tol=1e-6) and math.isclose(y, 70.0, abs_tol=1e-6)
    assert mp.start_xy(frame, None) is None


def test_goto_path_starts_at_the_fix_not_at_odom():
    # Gazebo: v1 spawns at map (20, 20) while its odom starts at (0, 0).
    log = []
    node = planner(log)
    v = node.vehicles['v1']
    v.fix = fix_at(node.frame, 20.0, 20.0)
    v.odom = Odometry()
    lat, lon = node.frame.to_latlon(100.0, 20.0)
    out = node.t_goto('v1', lat, lon)
    assert out['accepted'] is True and (out['x'], out['y']) == (100.0, 20.0)
    path = node.path_pubs['v1'].published[-1]
    assert path.header.frame_id == 'map' and len(path.poses) == 2
    start, goal = path.poses[0].pose.position, path.poses[1].pose.position
    assert math.isclose(start.x, 20.0, abs_tol=1e-6) and math.isclose(start.y, 20.0, abs_tol=1e-6), (start.x, start.y)
    assert math.isclose(goal.x, 100.0, abs_tol=1e-6) and math.isclose(goal.y, 20.0, abs_tol=1e-6)
    assert v.mode == 'goto'


def test_goto_without_a_fix_is_refused_and_cancels_nothing():
    log = []
    node = planner(log)
    v = node.vehicles['v1']
    v.odom = Odometry()
    v.goal = FakeGoalHandle(log)
    v.mode = 'patrol'
    lat, lon = node.frame.to_latlon(100.0, 20.0)
    out = node.t_goto('v1', lat, lon)
    assert out['accepted'] is False and 'fix' in out['reason'], out
    assert log == [] and node.path_pubs['v1'].published == []
    assert v.goal is not None and v.mode == 'patrol'


def test_goto_outside_the_corridor_is_refused_before_anything_is_stopped():
    log = []
    node = planner(log)
    v = node.vehicles['v1']
    v.fix = fix_at(node.frame, 20.0, 20.0)
    v.goal = FakeGoalHandle(log)
    lat, lon = node.frame.to_latlon(20.0, -60.0)
    out = node.t_goto('v1', lat, lon)
    assert out['accepted'] is False and log == [] and v.goal is not None


def test_summary_xy_come_from_the_fix():
    node = planner([])
    v = node.vehicles['v1']
    v.fix = fix_at(node.frame, 20.0, 20.0)
    v.odom = Odometry()
    v.odom.twist.twist.linear.x = 1.5
    d = node._summary('v1')
    assert (d['x'], d['y']) == (20.0, 20.0), d
    assert d['speed_mps'] == 1.5 and d['inside_corridor'] is True and d['edge_distance_m'] == 20.0


def test_summary_without_a_corridor_has_no_xy():
    node = planner([])
    node.frame = node.poly = None
    v = node.vehicles['v1']
    v.fix = NavSatFix(latitude=52.7001, longitude=-2.9997)
    v.odom = Odometry()
    d = node._summary('v1')
    assert 'x' not in d and 'y' not in d and d['lat'] == 52.7001
