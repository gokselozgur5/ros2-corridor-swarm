import xml.etree.ElementTree as ET
from datetime import datetime, timezone

from corridor_cot.cot import course_from_quaternion, parse_marker, pli_event


def test_pli_event_shape():
    now = datetime(2026, 10, 24, 15, 0, 0, tzinfo=timezone.utc)
    xml = pli_event('v1', 'V1', 52.7, -3.0, course_deg=90.0, speed_mps=3.0, now=now)
    ev = ET.fromstring(xml)
    assert ev.get('uid') == 'v1' and ev.get('type') == 'a-f-G-E-V-C'
    assert ev.get('time') == '2026-10-24T15:00:00.00Z' and ev.get('stale') == '2026-10-24T15:01:00.00Z'
    pt = ev.find('point')
    assert pt.get('lat') == '52.7000000' and pt.get('lon') == '-3.0000000'
    assert ev.find('detail/contact').get('callsign') == 'V1'
    assert ev.find('detail/track').get('speed') == '3.00'


def test_parse_marker_accepts_waypoint_and_rejects_pli():
    marker = ('<event version="2.0" uid="m1" type="b-m-p-w" how="h-g-i-g-o" time="t" start="t" stale="t">'
              '<point lat="52.701" lon="-2.999" hae="0" ce="9999999" le="9999999"/>'
              '<detail><contact callsign="WP1"/></detail></event>')
    m = parse_marker(marker)
    assert m == {'uid': 'm1', 'callsign': 'WP1', 'lat': 52.701, 'lon': -2.999, 'type': 'b-m-p-w'}
    assert parse_marker(pli_event('v1', 'V1', 52.7, -3.0)) is None
    assert parse_marker('<not xml') is None


def test_course_from_quaternion():
    # yaw 0 (east) -> course 90; yaw +90 (north) -> course 0
    assert abs(course_from_quaternion(0.0, 1.0) - 90.0) < 1e-6
    assert abs(course_from_quaternion(0.7071068, 0.7071068) - 0.0) < 1e-4
