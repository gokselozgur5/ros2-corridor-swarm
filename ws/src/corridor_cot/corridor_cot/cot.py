"""Cursor-on-Target XML, built and parsed without pytak so it is unit-testable and dependency-free."""
import math
import xml.etree.ElementTree as ET
from datetime import datetime, timedelta, timezone

ISO = '%Y-%m-%dT%H:%M:%S.%fZ'


def _stamp(t):
    return t.strftime(ISO)[:-4] + 'Z'


def pli_event(uid, callsign, lat, lon, hae=0.0, course_deg=0.0, speed_mps=0.0, cot_type='a-f-G-E-V-C',
              team='Cyan', role='Team Member', stale_s=60, now=None):
    """Position report for one vehicle. cot_type a-f-G-E-V-C: friendly ground equipment, civilian vehicle."""
    now = now or datetime.now(timezone.utc)
    ev = ET.Element('event', version='2.0', uid=uid, type=cot_type, how='m-g',
                    time=_stamp(now), start=_stamp(now), stale=_stamp(now + timedelta(seconds=stale_s)))
    ET.SubElement(ev, 'point', lat=f'{lat:.7f}', lon=f'{lon:.7f}', hae=f'{hae:.1f}', ce='5.0', le='5.0')
    detail = ET.SubElement(ev, 'detail')
    ET.SubElement(detail, 'contact', callsign=callsign)
    ET.SubElement(detail, '__group', name=team, role=role)
    ET.SubElement(detail, 'track', course=f'{course_deg:.1f}', speed=f'{speed_mps:.2f}')
    ET.SubElement(detail, 'takv', device='corridor_sim', platform='ROS 2', version='0.1')
    return ET.tostring(ev, encoding='unicode')


def pli_payload(state, t=0.0, fix_timeout_s=float('inf'), team='Cyan', now=None):
    """One PLI per vehicle in state ({name: {lat, lon, hae, course, speed, t}}) whose fix is younger than
    fix_timeout_s at time t, concatenated. A vehicle that stops reporting gets no further events, so its
    last one goes stale on the TAK side instead of being re-sent as a fresh machine-GPS position."""
    payload = ''
    for v, st in state.items():
        if 'lat' not in st or t - st.get('t', t) > fix_timeout_s:
            continue
        payload += pli_event(uid=f'corridor-{v}', callsign=v.upper(), lat=st['lat'], lon=st['lon'], hae=st.get('hae', 0.0),
                             course_deg=st.get('course', 0.0), speed_mps=st.get('speed', 0.0), team=team, now=now)
    return payload


def _coordinate(text, limit):
    """Attribute text -> float within [-limit, limit], or None when missing, non-numeric, nan or inf."""
    try:
        x = float(text)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) and abs(x) <= limit else None


def parse_marker(xml_text):
    """A TAK marker (type a-*-G or b-m-p-*) -> dict(uid, callsign, lat, lon, type) or None for other events.

    None also for a marker whose <point> has no usable lat/lon: the 8087 stream is unauthenticated and the
    reader thread must not die on one bad event."""
    try:
        ev = ET.fromstring(xml_text)
    except ET.ParseError:
        return None
    if ev.tag != 'event':
        return None
    cot_type = ev.get('type', '')
    if not (cot_type.startswith('b-m-p') or cot_type.startswith('a-u-G') or cot_type.startswith('a-h-G')):
        return None
    pt = ev.find('point')
    if pt is None:
        return None
    lat, lon = _coordinate(pt.get('lat'), 90.0), _coordinate(pt.get('lon'), 180.0)
    if lat is None or lon is None:
        return None
    contact = ev.find('detail/contact')
    return {'uid': ev.get('uid', ''), 'callsign': contact.get('callsign', '') if contact is not None else '',
            'lat': lat, 'lon': lon, 'type': cot_type}


def course_from_quaternion(z, w):
    """ENU yaw (quaternion z,w) -> compass course in degrees, 0 = north, clockwise."""
    yaw = math.degrees(2.0 * math.atan2(z, w))
    return (90.0 - yaw) % 360.0
