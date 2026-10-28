"""The planner's tool methods against fake ROS objects (ros_fakes.py). Runs without ROS through the
stand-ins in conftest.py, and against the real message classes on Thor."""
import math
import pathlib
import threading
import time

import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from corridor_msgs.msg import FleetState, VehicleState
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
    assert out == {'stopped': True, 'vehicle': 'v1', 'cancelled_patrol': True, 'finished': True}
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
    v.state = VehicleState(name='v1', mode='idle')
    v.state.velocity.linear.x = 1.5
    d = node._summary('v1')
    assert (d['x'], d['y']) == (20.0, 20.0), d
    assert d['speed_mps'] == 1.5 and d['inside_corridor'] is True and d['edge_distance_m'] == 20.0


def fleet(node, **vehicles):
    """A FleetState as corridor_nav's fleet_state publishes it: name -> (x, y, mode) in the corridor frame,
    or None for a vehicle whose fix fleet_state does not have (mode 'unknown', fix zeroed)."""
    msg = FleetState(corridor_id=node.corridor_id)
    for name, spec in vehicles.items():
        st = VehicleState(name=name)
        if spec is None:
            st.mode = 'unknown'
        else:
            x, y, st.mode = spec
            st.fix = fix_at(node.frame, x, y)
            st.pose.position.x, st.pose.position.y = x, y
            st.inside_corridor = True
        msg.vehicles.append(st)
    return msg


# ----- ADR-0015: the planner reads /fleet/state ------------------------------------------------------

def test_fleet_state_is_kept_for_the_vehicles_the_planner_knows_only():
    node = planner([])
    node.vehicles['v2'] = Vehicle()
    node.on_fleet(fleet(node, v1=(20.0, 70.0, 'goto'), v9=(1.0, 1.0, 'idle')))
    assert node.vehicles['v1'].state.mode == 'goto'
    assert node.vehicles['v2'].state is None
    assert 'v9' not in node.vehicles                  # a name outside the vehicles parameter is not adopted


def test_summary_mode_is_the_observed_one_not_the_commanded_one():
    log = []
    node = planner(log)
    node.on_fleet(fleet(node, v1=(20.0, 20.0, 'idle')))
    assert node.t_patrol('v1', 0, False)['accepted'] is True
    assert node._summary('v1')['mode'] == 'idle'          # until fleet_state sees the goal running
    node.on_fleet(fleet(node, v1=(20.0, 20.0, 'patrol')))
    assert node._summary('v1')['mode'] == 'patrol'


def test_summary_carries_the_goal_id_fleet_state_reports():
    node = planner([])
    node.vehicles['v1'].state = VehicleState(name='v1', mode='patrol', goal_id='ab')
    assert node._summary('v1')['goal_id'] == 'ab'


def test_fleet_state_sets_the_fix_goto_starts_from():
    node = planner([])
    node.on_fleet(fleet(node, v1=(20.0, 70.0, 'idle')))
    assert mp.start_xy(node.frame, node.vehicles['v1'].fix) == pytest.approx((20.0, 70.0), abs=1e-6)


def test_a_vehicle_the_fleet_reports_unknown_loses_its_fix_and_goto_refuses():
    log = []
    node = planner(log)
    node.on_fleet(fleet(node, v1=(20.0, 20.0, 'idle')))
    node.on_fleet(fleet(node, v1=None))
    assert node.vehicles['v1'].fix is None
    lat, lon = node.frame.to_latlon(100.0, 20.0)
    assert node.t_goto('v1', lat, lon)['accepted'] is False and log == []


def test_summary_without_a_corridor_has_no_xy():
    node = planner([])
    node.frame = node.poly = None
    v = node.vehicles['v1']
    v.fix = NavSatFix(latitude=52.7001, longitude=-2.9997)
    v.odom = Odometry()
    d = node._summary('v1')
    assert 'x' not in d and 'y' not in d and d['lat'] == 52.7001


# ----- review follow-ups: the follower stops at once, the tool says whether the old goal finished, goto starts where the vehicle is after the stop

def test_stop_publishes_the_empty_path_right_after_the_cancel_not_after_the_old_goal_exited():
    log = []
    node = planner(log)
    node.vehicles['v1'].goal = FakeGoalHandle(log, exit_delay_s=0.2)
    node.t_stop('v1')
    assert log.index('cancel') < log.index('path_empty') < log.index('old_result'), log


def test_stop_reports_whether_the_old_goal_finished(monkeypatch):
    monkeypatch.setattr(mp, 'STOP_TIMEOUT_S', 0.2)
    log = []
    node = planner(log)
    node.vehicles['v1'].goal = FakeGoalHandle(log, exit_delay_s=0.05)
    assert node.t_stop('v1')['finished'] is True
    node.vehicles['v1'].goal = FakeGoalHandle(log, exit_delay_s=None)
    assert node.t_stop('v1')['finished'] is False
    assert node.t_stop('v1')['finished'] is True   # nothing to cancel


def test_goto_starts_where_the_vehicle_is_after_the_stop_not_before():
    import threading
    log = []
    node = planner(log)
    v = node.vehicles['v1']
    v.fix = fix_at(node.frame, 20.0, 20.0)
    v.goal = FakeGoalHandle(log, exit_delay_s=0.2)
    threading.Timer(0.05, lambda: setattr(v, 'fix', fix_at(node.frame, 23.0, 20.0))).start()   # it keeps driving while the old goal exits
    lat, lon = node.frame.to_latlon(100.0, 20.0)
    assert node.t_goto('v1', lat, lon)['accepted'] is True
    start = node.path_pubs['v1'].published[-1].poses[0].pose.position
    assert math.isclose(start.x, 23.0, abs_tol=1e-6), start.x


# ----- final review: patrol and goto cancel the tracked goal without an empty Path of their own; the successor
# route replaces what the follower holds, and two DDS writers racing (planner's Path() vs the server's route)
# could otherwise leave the new goal 'preempted' with the vehicle idle

def test_patrol_and_goto_publish_no_empty_path_of_their_own():
    log = []
    node = planner(log)
    assert node.t_patrol('v1', 0, False)['accepted'] is True
    assert 'path_empty' not in log, log                      # nothing tracked: the server's route replaces the follower's Path
    node.vehicles['v1'].goal = FakeGoalHandle(log, exit_delay_s=0.05)
    assert node.t_patrol('v1', 0, True)['accepted'] is True
    assert 'path_empty' not in log, log                      # tracked goal: the server clears an owned route in its cancel callback
    v = node.vehicles['v1']
    v.goal = FakeGoalHandle(log, exit_delay_s=0.05)
    v.fix = fix_at(node.frame, 20.0, 20.0)
    lat, lon = node.frame.to_latlon(100.0, 20.0)
    assert node.t_goto('v1', lat, lon)['accepted'] is True
    assert 'path_empty' not in log and log.count('path') == 1, log
    assert node.t_stop('v1')['stopped'] is True and log[-1] == 'path_empty'   # only the operator's stop clears the follower itself


def test_a_rejected_successor_does_not_leave_the_vehicle_reported_as_patrolling():
    log = []
    node = planner(log)
    v = node.vehicles['v1']
    v.goal = FakeGoalHandle(log, exit_delay_s=0.05)
    v.mode = 'patrol'
    node.patrol_clients['v1'].accepted = False
    out = node.t_patrol('v1', 0, False)
    assert out['accepted'] is False
    assert v.goal is None and v.mode == 'stopped', (v.goal, v.mode)


# ----- busy_marker (Aether's review, 2026-10-28) --------------------------------------------------------

def test_busy_reply_names_the_request_it_dropped():
    """cot_bridge matches this reply to the marker it sent and lets ATAK's re-send through; a bare 'busy'
    left that placement suppressed until the marker moved."""
    log = []
    node = planner(log)
    node.busy = threading.Lock()
    node.busy.acquire()
    node.reply_pub = FakePublisher(log, name='reply')
    node.on_request(String(data='send the nearest vehicle to marker RALLY at 52.701000, -2.998500'))
    assert [m.data for m in node.reply_pub.published] == [
        'busy with the previous request, dropped: send the nearest vehicle to marker RALLY at 52.701000, -2.998500']
