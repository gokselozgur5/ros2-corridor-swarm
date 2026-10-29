"""tools/atak_standin.py speaks the CoT the bridge and the server speak (#34): its marker is one cot_bridge accepts with
the label and position intact, and its watch parser reads the PLI cot_bridge emits."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
sys.path.insert(0, os.path.join(HERE, '..', 'ws', 'src', 'corridor_cot'))

import atak_standin  # noqa: E402
from corridor_cot.cot import parse_marker, pli_event, request_text  # noqa: E402


def test_marker_event_is_a_placement_the_bridge_turns_into_a_request():
    xml = atak_standin.event('marker-1', 'b-m-p-w', 'RALLY1', 52.7010, -2.9985, 'h-g-i-g-o')
    marker = parse_marker(xml)
    assert marker is not None
    assert (marker['lat'], marker['lon']) == (52.701, -2.9985)
    assert request_text(marker) == 'send the nearest vehicle to marker "RALLY1" at 52.701000, -2.998500'


def test_hello_is_a_friendly_unit_not_a_marker():
    assert parse_marker(atak_standin.hello('atak-standin-1')) is None


def test_watch_regexes_read_the_bridges_pli():
    xml = pli_event('corridor-v2', 'v2', 52.7009, -2.9985, course_deg=87.5, speed_mps=3.0).encode()
    head = xml.split(b'>', 1)[0]
    attrs = dict((k.decode(), v.decode()) for k, v in atak_standin.ATTR.findall(head))
    assert attrs['uid'] == 'corridor-v2' and attrs['type'] == 'a-f-G-E-V-C'
    point = atak_standin.fields(atak_standin.POINT.search(xml))
    assert (float(point['lat']), float(point['lon'])) == (52.7009, -2.9985)
    assert atak_standin.CALLSIGN.search(xml).group(1) == b'v2'
    track = atak_standin.fields(atak_standin.TRACK.search(xml))
    assert (track['course'], track['speed']) == ('87.5', '3.00')


def test_watch_fields_do_not_depend_on_the_servers_attribute_order():
    reordered = b'<point hae="0.0" lon="-2.9985000" ce="5.0" lat="52.7009000" le="5.0"/>'
    point = atak_standin.fields(atak_standin.POINT.search(reordered))
    assert (float(point['lat']), float(point['lon'])) == (52.7009, -2.9985)
