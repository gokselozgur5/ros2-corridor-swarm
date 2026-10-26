import xml.etree.ElementTree as ET
from datetime import datetime, timezone

from corridor_cot.cot import course_from_quaternion, parse_marker, pli_event, pli_payload


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


def test_course_from_quaternion():
    # yaw 0 (east) -> course 90; yaw +90 (north) -> course 0
    assert abs(course_from_quaternion(0.0, 1.0) - 90.0) < 1e-6
    assert abs(course_from_quaternion(0.7071068, 0.7071068) - 0.0) < 1e-4
