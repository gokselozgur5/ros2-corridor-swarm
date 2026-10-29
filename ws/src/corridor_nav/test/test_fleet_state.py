"""fleet_state: the pure derivations and the node on the stub rclpy from conftest (callbacks and the
timer tick called directly, the stub clock advanced by hand)."""
import pytest
import rclpy

pytestmark = pytest.mark.skipif(hasattr(rclpy, '__file__'),
                                reason='stub-rclpy tests: the node is proven live by steps/16_fleet_state_proof.sh')

from action_msgs.msg import GoalStatus, GoalStatusArray  # noqa: E402
from geometry_msgs.msg import Quaternion  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Imu, NavSatFix  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from corridor_msgs.msg import GeofenceEvent  # noqa: E402
from corridor_nav.fleet_state import FleetStateNode, active_goal_id, mode_of  # noqa: E402

GEOJSON = '''{"type":"FeatureCollection","features":[{"type":"Feature","properties":{"id":"demo"},
"geometry":{"type":"Polygon","coordinates":[[[-3.0000,52.7000],[-2.9970,52.7000],[-2.9970,52.7018],[-3.0000,52.7018],[-3.0000,52.7000]]]}}]}'''


# ----- derivations ---------------------------------------------------------------------------

def test_mode_geofence_hold_beats_a_running_patrol():
    assert mode_of('abc', 'following', breached=True) == 'geofence_hold'


def test_mode_patrol_when_a_goal_is_active():
    assert mode_of('abc', 'following', breached=False) == 'patrol'


def test_mode_goto_when_the_follower_follows_without_a_patrol_goal():
    assert mode_of('', 'following', breached=False) == 'goto'


def test_mode_idle_after_arrival_or_cancel():
    assert mode_of('', 'arrived', breached=False) == 'idle'
    assert mode_of('', 'idle', breached=False) == 'idle'


def test_mode_unknown_before_the_follower_reports():
    assert mode_of('', '', breached=False) == 'unknown'


def status(uuid_byte, code):
    st = GoalStatus()
    st.goal_info.goal_id.uuid = bytes([uuid_byte]) + bytes(15)
    st.status = code
    return st


def test_active_goal_id_skips_finished_goals_and_takes_the_running_one():
    arr = GoalStatusArray()
    arr.status_list = [status(1, GoalStatus.STATUS_SUCCEEDED), status(2, GoalStatus.STATUS_EXECUTING)]
    assert active_goal_id(arr) == '02' + '00' * 15


def test_active_goal_id_is_empty_when_every_goal_is_over():
    arr = GoalStatusArray()
    arr.status_list = [status(1, GoalStatus.STATUS_CANCELED), status(2, GoalStatus.STATUS_ABORTED)]
    assert active_goal_id(arr) == ''


# ----- node ----------------------------------------------------------------------------------

def make_node(node_params, **params):
    node_params(**{'corridor_geojson': GEOJSON, 'vehicles': ['v1', 'v2'], 'period_s': 0.5, 'fix_timeout_s': 5.0, **params})
    return FleetStateNode()


def send_fix(node, vehicle, x, y):
    lat, lon = node.frame.to_latlon(x, y)
    node.subscriptions[f'/{vehicle}/fix'](NavSatFix(latitude=lat, longitude=lon))


def last(node):
    node.tick()
    return node.publishers['/fleet/state'].sent[-1]


def test_every_vehicle_is_listed_and_positioned_by_its_fix(node_params):
    node = make_node(node_params)
    send_fix(node, 'v1', 20.0, 70.0)
    send_fix(node, 'v2', 150.0, 10.0)
    msg = last(node)
    assert msg.corridor_id == 'demo'
    assert [v.name for v in msg.vehicles] == ['v1', 'v2']
    assert (round(msg.vehicles[0].pose.position.x), round(msg.vehicles[0].pose.position.y)) == (20, 70)
    assert (round(msg.vehicles[1].pose.position.x), round(msg.vehicles[1].pose.position.y)) == (150, 10)


def test_inside_corridor_follows_the_polygon(node_params):
    node = make_node(node_params)
    send_fix(node, 'v1', 20.0, 70.0)
    send_fix(node, 'v2', -30.0, 70.0)
    msg = last(node)
    assert msg.vehicles[0].inside_corridor is True
    assert msg.vehicles[1].inside_corridor is False


def test_heading_comes_from_the_imu_and_speed_from_odom(node_params):
    node = make_node(node_params)
    send_fix(node, 'v1', 20.0, 70.0)
    node.subscriptions['/v1/imu'](Imu(orientation=Quaternion(z=0.7071, w=0.7071)))
    odom = Odometry()
    odom.twist.twist.linear.x = 2.5
    odom.pose.pose.position.x = 999.0   # odom x/y must never reach the state (#19)
    node.subscriptions['/v1/odom'](odom)
    v1 = last(node).vehicles[0]
    assert round(v1.pose.orientation.z, 4) == 0.7071
    assert v1.velocity.linear.x == 2.5
    assert round(v1.pose.position.x) == 20


def test_mode_and_goal_id_follow_the_patrol_action_status(node_params):
    node = make_node(node_params)
    send_fix(node, 'v1', 20.0, 70.0)
    node.subscriptions['/v1/follower/status'](String(data='following'))
    assert last(node).vehicles[0].mode == 'goto'
    arr = GoalStatusArray()
    arr.status_list = [status(7, GoalStatus.STATUS_EXECUTING)]
    node.subscriptions['/v1/patrol/_action/status'](arr)
    v1 = last(node).vehicles[0]
    assert v1.mode == 'patrol'
    assert v1.goal_id == '07' + '00' * 15
    arr.status_list = [status(7, GoalStatus.STATUS_SUCCEEDED)]
    node.subscriptions['/v1/patrol/_action/status'](arr)
    node.subscriptions['/v1/follower/status'](String(data='arrived'))
    v1 = last(node).vehicles[0]
    assert (v1.mode, v1.goal_id) == ('idle', '')


def test_geofence_exit_holds_the_vehicle_until_it_re_enters(node_params):
    node = make_node(node_params)
    send_fix(node, 'v1', 20.0, 70.0)
    node.subscriptions['/v1/follower/status'](String(data='following'))
    node.subscriptions['/fleet/geofence_events'](GeofenceEvent(vehicle='v1', type=GeofenceEvent.EXIT))
    assert last(node).vehicles[0].mode == 'geofence_hold'
    assert last(node).vehicles[1].mode == 'unknown'   # v2 never sent a fix
    node.subscriptions['/fleet/geofence_events'](GeofenceEvent(vehicle='v1', type=GeofenceEvent.ENTER))
    assert last(node).vehicles[0].mode == 'goto'


def test_a_stale_fix_leaves_the_vehicle_listed_but_unknown(node_params):
    node = make_node(node_params)
    send_fix(node, 'v1', 20.0, 70.0)
    node.subscriptions['/v1/follower/status'](String(data='idle'))
    assert last(node).vehicles[0].mode == 'idle'
    node.clock.advance(6.0)
    v1 = last(node).vehicles[0]
    assert v1.name == 'v1'
    assert v1.mode == 'unknown'
    assert v1.fix.latitude == 0.0
    assert v1.inside_corridor is False


def test_timer_runs_at_period_s(node_params):
    node = make_node(node_params, period_s=0.25)
    assert [p for p, _ in node.timers] == [0.25]


def test_vehicle_state_is_stamped_with_the_vehicles_own_fix_clock(node_params):
    # A Gazebo vehicle runs on sim time while fleet_state runs on wall time; consumers age that vehicle's data
    # (the planner's camera frame) against this stamp, so it must be the vehicle's clock, not the node's (#37).
    node = make_node(node_params)
    lat, lon = node.frame.to_latlon(20.0, 70.0)
    fix = NavSatFix(latitude=lat, longitude=lon)
    fix.header.stamp.sec, fix.header.stamp.nanosec = 100, 500
    node.subscriptions['/v1/fix'](fix)
    msg = last(node)
    assert (msg.vehicles[0].header.stamp.sec, msg.vehicles[0].header.stamp.nanosec) == (100, 500)
    assert msg.vehicles[1].mode == 'unknown'   # no fix: stamped by the node, nothing to age against
