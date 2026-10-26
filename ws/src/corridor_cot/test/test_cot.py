import xml.etree.ElementTree as ET
from datetime import datetime, timezone

from corridor_cot.cot import MarkerGate, course_from_quaternion, parse_marker, pli_event, pli_payload


def test_pli_event_shape():
    now = datetime(2026, 10, 24, 15, 0, 0, tzinfo=timezone.utc)
    xml = pli_event('v1', 'V1', 52.7, -3.0, course_deg=90.0, speed_mps=3.0, now=now)
    ev = ET.fromstring(xml)
    assert ev.get('uid') == 'v1' and ev.get('type') == 'a-f-G-E-V-C'
    assert ev.get('time') == '2026-10-24T15:00:00.000Z' and ev.get('stale') == '2026-10-24T15:01:00.000Z'
    pt = ev.find('point')
    assert pt.get('lat') == '52.7000000' and pt.get('lon') == '-3.0000000'
    assert ev.find('detail/contact').get('callsign') == 'V1'
    assert ev.find('detail/track').get('speed') == '3.00'


def test_pli_payload_one_event_per_vehicle_with_a_fix():
    state = {'v1': {'lat': 52.7, 'lon': -3.0, 'hae': 10.0, 'course': 45.0, 'speed': 2.0}, 'v2': {'course': 10.0}}
    events = [ET.fromstring(x + '</event>') for x in pli_payload(state, team='Red').split('</event>') if x]
    assert [e.get('uid') for e in events] == ['corridor-v1']
    assert events[0].find('detail/contact').get('callsign') == 'V1'
    assert events[0].find('detail/__group').get('name') == 'Red'
    assert events[0].find('detail/track').get('course') == '45.0'
    assert pli_payload({'v1': {}}) == ''


def test_pli_payload_drops_a_vehicle_whose_fix_went_silent():
    # cot-2: v2's last fix was at t=100; after fix_timeout_s with nothing new it must not be re-sent as a live PLI
    state = {'v1': {'lat': 52.7, 'lon': -3.0, 't': 109.0}, 'v2': {'lat': 52.8, 'lon': -3.1, 't': 100.0}}
    live = pli_payload(state, t=110.0, fix_timeout_s=5.0)
    assert 'corridor-v1' in live and 'corridor-v2' not in live
    both = pli_payload(state, t=104.0, fix_timeout_s=5.0)
    assert 'corridor-v1' in both and 'corridor-v2' in both


def test_parse_marker_accepts_waypoint_and_rejects_pli():
    marker = ('<event version="2.0" uid="m1" type="b-m-p-w" how="h-g-i-g-o" time="t" start="t" stale="t">'
              '<point lat="52.701" lon="-2.999" hae="0" ce="9999999" le="9999999"/>'
              '<detail><contact callsign="WP1"/></detail></event>')
    m = parse_marker(marker)
    assert m == {'uid': 'm1', 'callsign': 'WP1', 'lat': 52.701, 'lon': -2.999, 'type': 'b-m-p-w'}
    assert parse_marker(pli_event('v1', 'V1', 52.7, -3.0)) is None
    assert parse_marker('<not xml') is None


def _marker(point, uid='m', stale=None):
    attrs = f' stale="{stale}"' if stale else ''
    return f'<event version="2.0" uid="{uid}" type="b-m-p-w"{attrs}>{point}<detail><contact callsign="X"/></detail></event>'


def test_parse_marker_returns_none_for_unusable_coordinates():
    # cot-1: a matching-type event with a broken <point> must be ignored, never raise out to the reader thread
    assert parse_marker(_marker('<point lon="-2.9"/>')) is None                 # lat missing: float(None)
    assert parse_marker(_marker('<point lat="abc" lon="-2.9"/>')) is None       # float('abc')
    assert parse_marker(_marker('<point lat="nan" lon="-2.9"/>')) is None
    assert parse_marker(_marker('<point lat="52.7" lon="1e400"/>')) is None     # inf
    assert parse_marker(_marker('<point lat="999" lon="-2.9"/>')) is None
    assert parse_marker(_marker('<point lat="52.7" lon="-181"/>')) is None
    assert parse_marker(_marker('<point lat="-90" lon="180"/>'))['lat'] == -90.0


def test_parse_marker_drops_a_marker_whose_stale_time_has_passed():
    # cot-4: a stale marker (one way a client or a replaying server signals an expired placement) must not dispatch a vehicle
    now = datetime(2026, 10, 24, 15, 0, 0, tzinfo=timezone.utc)
    point = '<point lat="52.7" lon="-2.9"/>'
    assert parse_marker(_marker(point, stale='2026-10-24T14:59:59.000Z'), now=now) is None
    assert parse_marker(_marker(point, stale='2026-10-24T14:59:59Z'), now=now) is None
    assert parse_marker(_marker(point, stale='2026-10-24T15:00:01.000Z'), now=now)['uid'] == 'm'
    assert parse_marker(_marker(point, stale='garbage'), now=now)['uid'] == 'm'  # unreadable stale: no expiry known
    assert parse_marker(_marker(point), now=now)['uid'] == 'm'


def test_marker_gate_passes_a_placement_once_and_again_only_when_moved():
    # cot-4: ATAK re-sends the same uid on rename, refresh or relay; only a new placement or a real move is a request
    gate = MarkerGate(move_m=5.0)
    m = {'uid': 'atak-1', 'callsign': 'RALLY1', 'lat': 52.7010, 'lon': -2.9985, 'type': 'b-m-p-w'}
    north = 1.0 / 111320.0  # degrees of latitude per metre
    assert gate.admit(m)
    assert not gate.admit(m)                                    # identical re-send
    assert not gate.admit(dict(m, callsign='RALLY'))            # renamed after the auto callsign, same spot
    assert not gate.admit(dict(m, lat=m['lat'] + 2 * north))    # 2 m: jitter, not a move
    assert gate.admit(dict(m, lat=m['lat'] + 20 * north))       # 20 m north: moved, dispatch again
    assert not gate.admit(dict(m, lat=m['lat'] + 20 * north))   # the new spot is now the known one
    assert gate.admit(dict(m, uid='atak-2'))                    # a second marker on the same spot is its own placement


def test_course_from_quaternion():
    # yaw 0 (east) -> course 90; yaw +90 (north) -> course 0
    assert abs(course_from_quaternion(0.0, 1.0) - 90.0) < 1e-6
    assert abs(course_from_quaternion(0.7071068, 0.7071068) - 0.0) < 1e-4


# ----- review follow-up: a dropped marker says why, so the bridge can log it instead of staying silent

def test_marker_or_reason_names_why_a_marker_is_dropped():
    from corridor_cot.cot import marker_or_reason
    now = datetime(2026, 10, 26, 12, 0, 0, tzinfo=timezone.utc)
    good = '<event version="2.0" uid="m1" type="b-m-p-w" stale="2026-10-26T12:05:00.000Z"><point lat="52.7" lon="-2.9"/></event>'
    marker, reason = marker_or_reason(good, now=now)
    assert marker['uid'] == 'm1' and reason is None
    assert marker_or_reason('<event version="2.0" uid="x" type="a-f-G-U-C"><point lat="52.7" lon="-2.9"/></event>', now=now) == (None, None)
    _, reason = marker_or_reason('<event version="2.0" uid="m2" type="b-m-p-w"><point lon="-2.9"/></event>', now=now)
    assert reason and 'm2' in reason and 'lat' in reason, reason
    _, reason = marker_or_reason('<event version="2.0" uid="m3" type="b-m-p-w" stale="2026-10-26T11:59:00.000Z"><point lat="52.7" lon="-2.9"/></event>', now=now)
    assert reason and 'm3' in reason and 'stale' in reason and '60 s' in reason, reason
    _, reason = marker_or_reason('<event version="2.0" uid="m4" type="b-m-p-w"></event>', now=now)
    assert reason and 'point' in reason, reason
