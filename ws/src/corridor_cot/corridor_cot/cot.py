"""Cursor-on-Target XML, built and parsed without pytak so it is unit-testable and dependency-free."""
import math
import re
import xml.etree.ElementTree as ET
from collections import OrderedDict
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


def pli_events(state, t=0.0, fix_timeout_s=float('inf'), team='Cyan', now=None):
    """One PLI per vehicle in state ({name: {lat, lon, hae, course, speed, t}}) whose fix is younger than
    fix_timeout_s at time t, as a list of event strings. A vehicle that stops reporting gets no further events,
    so its last one goes stale on the TAK side instead of being re-sent as a fresh machine-GPS position."""
    events = []
    for v, st in state.items():
        if 'lat' not in st or t - st.get('t', t) > fix_timeout_s:
            continue
        events.append(pli_event(uid=f'corridor-{v}', callsign=v.upper(), lat=st['lat'], lon=st['lon'], hae=st.get('hae', 0.0),
                                course_deg=st.get('course', 0.0), speed_mps=st.get('speed', 0.0), team=team, now=now))
    return events


def pli_payload(state, t=0.0, fix_timeout_s=float('inf'), team='Cyan', now=None):
    """The events of pli_events concatenated: one write for an established TAK connection."""
    return ''.join(pli_events(state, t, fix_timeout_s, team, now))


def _coordinate(text, limit):
    """Attribute text -> float within [-limit, limit], or None when missing, non-numeric, nan or inf."""
    try:
        x = float(text)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) and abs(x) <= limit else None


def _parse_stamp(text):
    """CoT timestamp -> aware datetime, or None when missing or in a form we do not read."""
    for fmt in (ISO, '%Y-%m-%dT%H:%M:%SZ'):
        try:
            return datetime.strptime(text, fmt).replace(tzinfo=timezone.utc)
        except (TypeError, ValueError):
            pass
    return None


def marker_or_reason(xml_text, now=None):
    """(marker, None) for a usable TAK marker, (None, reason) for a marker that is dropped, (None, None)
    for any other event. The reason is for the bridge's log: the 8087 stream is unauthenticated and a
    silently vanishing marker is indistinguishable from a dead bridge."""
    try:
        ev = ET.fromstring(xml_text)
    except ET.ParseError:
        return None, None
    if ev.tag != 'event':
        return None, None
    cot_type = ev.get('type', '')
    if not (cot_type.startswith('b-m-p') or cot_type.startswith('a-u-G') or cot_type.startswith('a-h-G')):
        return None, None
    uid = ev.get('uid', '')
    now = now or datetime.now(timezone.utc)
    stale = _parse_stamp(ev.get('stale'))
    if stale is not None and stale < now:
        return None, f'marker {uid!r} stale since {ev.get("stale")} ({(now - stale).total_seconds():.0f} s ago)'
    pt = ev.find('point')
    if pt is None:
        return None, f'marker {uid!r} has no point'
    lat, lon = _coordinate(pt.get('lat'), 90.0), _coordinate(pt.get('lon'), 180.0)
    if lat is None or lon is None:
        return None, f'marker {uid!r} has no usable lat/lon: lat={pt.get("lat")!r} lon={pt.get("lon")!r}'
    contact = ev.find('detail/contact')
    return {'uid': uid, 'callsign': contact.get('callsign', '') if contact is not None else '',
            'lat': lat, 'lon': lon, 'type': cot_type}, None


def parse_marker(xml_text, now=None):
    """A TAK marker (type a-*-G or b-m-p-*) -> dict(uid, callsign, lat, lon, type) or None for other events.

    None also for a marker whose <point> has no usable lat/lon (the 8087 stream is unauthenticated and the
    reader thread must not die on one bad event) and for one whose stale time has passed at `now` (a replayed
    or expired placement must not dispatch a vehicle); an unreadable stale is taken as no expiry.
    marker_or_reason says why a marker was dropped."""
    return marker_or_reason(xml_text, now)[0]


_NOT_LABEL = re.compile(r'[^A-Za-z0-9 _.-]')
LABEL_MAX = 24


def label(marker):
    """The marker's display name as the planner may see it: the callsign, else the uid, else 'marker', kept to
    [A-Za-z0-9 _.-], single spaces and at most LABEL_MAX characters. The 8087 stream is unauthenticated and the
    text ends up in the model's user turn, so whatever someone typed into ATAK is a name to show, never wording
    to follow; the planner's SYSTEM prompt says the same about the quoted label."""
    for raw in (marker.get('callsign'), marker.get('uid')):
        text = ' '.join(_NOT_LABEL.sub('', raw or '').split())[:LABEL_MAX].strip()
        if text:
            return text
    return 'marker'


def request_text(marker):
    """The /mission/request text for a marker. The label is quoted and cannot contain a quote, so the planner
    can tell the name from the order; mission_planner echoes this text back in its busy reply, so the bridge
    keys its pending requests on it."""
    return f'send the nearest vehicle to marker "{label(marker)}" at {marker["lat"]:.6f}, {marker["lon"]:.6f}'


def _metres(a, b):
    """Flat-earth distance between two (lat, lon) pairs; good to a few percent over the tens of metres it is used for."""
    k = 111320.0
    return math.hypot((b[0] - a[0]) * k, (b[1] - a[1]) * k * math.cos(math.radians((a[0] + b[0]) / 2.0)))


class MarkerGate:
    """Decides which marker events become mission requests: a uid passes the first time and again only once it
    has moved by `move_m`. ATAK re-sends the same uid on a rename after the auto callsign, on a refresh and
    through a second client; each of those cost a planning run or was dropped by the planner's busy guard.
    Remembers at most `capacity` uids (the stream is unauthenticated: unique uids must not grow memory without
    bound); a marker without a uid has nothing to deduplicate on and always passes."""

    def __init__(self, move_m=5.0, capacity=256):
        self.move_m = move_m
        self.capacity = capacity
        self.sent = OrderedDict()

    def admit(self, marker):
        uid = marker['uid']
        if not uid:
            return True
        here = (marker['lat'], marker['lon'])
        last = self.sent.get(uid)
        if last is not None and _metres(last, here) < self.move_m:
            return False
        self.sent[uid] = here
        self.sent.move_to_end(uid)
        while len(self.sent) > self.capacity:
            self.sent.popitem(last=False)
        return True

    def forget(self, uid):
        """The placement was not acted on (the planner was busy): the next event for this uid passes again."""
        self.sent.pop(uid, None)


def course_from_quaternion(z, w):
    """ENU yaw (quaternion z,w) -> compass course in degrees, 0 = north, clockwise."""
    yaw = math.degrees(2.0 * math.atan2(z, w))
    return (90.0 - yaw) % 360.0
