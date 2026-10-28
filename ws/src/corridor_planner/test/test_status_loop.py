"""status_loop's fleet snapshot from a FleetState (ADR-0015). Runs on the conftest stand-ins without ROS."""
from corridor_msgs.msg import FleetState, VehicleState
from sensor_msgs.msg import NavSatFix

from corridor_planner.status_loop import fleet_snapshot


def vehicle(name, mode='idle', lat=52.7001, lon=-2.9997, vx=0.0, vy=0.0, inside=True):
    st = VehicleState(name=name, mode=mode, fix=NavSatFix(latitude=lat, longitude=lon), inside_corridor=inside)
    st.velocity.linear.x, st.velocity.linear.y = vx, vy
    return st


def test_snapshot_reports_position_speed_mode_and_corridor_flag():
    msg = FleetState(vehicles=[vehicle('v1', mode='patrol', vx=3.0, vy=4.0, inside=False)])
    assert fleet_snapshot(msg, ['v1']) == {'v1': {'lat': 52.7001, 'lon': -2.9997, 'speed': 5.0, 'mode': 'patrol', 'inside': False}}


def test_snapshot_keeps_only_the_vehicles_the_loop_reports():
    msg = FleetState(vehicles=[vehicle('v1'), vehicle('v7')])
    assert list(fleet_snapshot(msg, ['v1'])) == ['v1']


def test_an_unknown_vehicle_has_no_position_in_the_snapshot():
    msg = FleetState(vehicles=[vehicle('v1', mode='unknown', lat=1.0, lon=1.0)])
    assert fleet_snapshot(msg, ['v1']) == {'v1': {'mode': 'unknown'}}
