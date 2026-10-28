"""fleet_state: the pure derivations and the node on the stub rclpy from conftest (callbacks and the
timer tick called directly, the stub clock advanced by hand)."""
import pytest
import rclpy

pytestmark = pytest.mark.skipif(hasattr(rclpy, '__file__'),
                                reason='stub-rclpy tests: the node is proven live by steps/16_fleet_state_proof.sh')

from action_msgs.msg import GoalStatus, GoalStatusArray  # noqa: E402

from corridor_nav.fleet_state import active_goal_id, mode_of  # noqa: E402

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


