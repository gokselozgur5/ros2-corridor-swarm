"""ATAK stand-in for the demo (#34): a plain CoT TCP client against FreeTAKServer, standard library only.

    python3 tools/atak_standin.py watch [--host H] [--port 8087]          # the fleet as the server relays it, one line per update
    python3 tools/atak_standin.py marker LABEL LAT LON [--host H] [--port 8087]   # drop a marker, the way an ATAK user would

FreeTAKServer on Thor listens on 127.0.0.1:8087 only, so from the Mac: ssh -N -L 8087:127.0.0.1:8087 jensen@<thor> first.
FTS 1.9.9.2 reads a connection's first message as the client registration, so both commands open with a PLI of their own.
"""
import argparse
import datetime
import re
import socket
import sys
import time
import uuid


def stamp(t):
    return t.strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'


def event(uid, cot_type, callsign, lat, lon, how, stale_min=5, detail=''):
    now = datetime.datetime.now(datetime.timezone.utc)
    return (f'<event version="2.0" uid="{uid}" type="{cot_type}" how="{how}" time="{stamp(now)}" start="{stamp(now)}" '
            f'stale="{stamp(now + datetime.timedelta(minutes=stale_min))}">'
            f'<point lat="{lat:.6f}" lon="{lon:.6f}" hae="0" ce="9999999" le="9999999"/>'
            f'<detail><contact callsign="{callsign}"/>{detail}</detail></event>')


def hello(uid):   # the stand-in's own position: an operator at the corridor's south-west corner
    return event(uid, 'a-f-G-U-C', 'OPERATOR', 52.7000, -3.0000, 'h-e', detail='<__group name="Cyan" role="Team Member"/>')


def connect(host, port, uid):
    s = socket.create_connection((host, port), timeout=5)
    s.sendall(hello(uid).encode())
    return s


ATTR = re.compile(rb'\b(uid|type|time|stale)="([^"]*)"')
POINT = re.compile(rb'<point\b[^>]*>')
CALLSIGN = re.compile(rb'<contact\b[^>]*\bcallsign="([^"]*)"')
TRACK = re.compile(rb'<track\b[^>]*>')
FIELD = re.compile(rb'\b(\w+)="([^"]*)"')   # the server re-serialises events, attribute order is not the bridge's


def fields(tag_match):
    return dict((k.decode(), v.decode()) for k, v in FIELD.findall(tag_match.group(0))) if tag_match else {}


def watch(args):
    uid = f'atak-standin-{uuid.uuid4().hex[:8]}'
    s = connect(args.host, args.port, uid)
    print(f'connected to {args.host}:{args.port} as {uid}; the fleet should appear within two seconds (Ctrl-C to stop)')
    print(f'{"time":>12}  {"callsign":<14}{"type":<14}{"lat":>11}{"lon":>11}{"course":>7}{"speed":>6}')
    buf = b''
    last = {}
    try:
        while True:
            chunk = s.recv(65536)
            if not chunk:
                print('server closed the connection')
                return 1
            buf += chunk
            while b'</event>' in buf:
                xml, buf = buf.split(b'</event>', 1)
                attrs = dict((k.decode(), v.decode()) for k, v in ATTR.findall(xml.split(b'>', 1)[0]))
                if attrs.get('uid') == uid:
                    continue
                if attrs.get('type', '').startswith('b-t-f'):   # GeoChat, not a position
                    continue
                point, track = fields(POINT.search(xml)), fields(TRACK.search(xml))
                name = CALLSIGN.search(xml)
                lat, lon = float(point.get('lat', 'nan')), float(point.get('lon', 'nan'))
                course, speed = track.get('course', ''), track.get('speed', '')
                key = attrs.get('uid', '?')
                moved = last.get(key) != (lat, lon)
                last[key] = (lat, lon)
                print(f'{time.strftime("%H:%M:%S"):>12}  {(name.group(1).decode() if name else key):<14}{attrs.get("type", ""):<14}'
                      f'{lat:>11.6f}{lon:>11.6f}{course[:6]:>7}{speed[:5]:>6}{"" if moved else "  (same spot)"}')
    except KeyboardInterrupt:
        return 0
    finally:
        s.close()


def marker(args):
    uid = f'atak-standin-{uuid.uuid4().hex[:8]}'
    s = connect(args.host, args.port, uid)
    time.sleep(0.5)   # registration first, the marker as the second message
    xml = event(f'marker-{uuid.uuid4().hex[:8]}', 'b-m-p-w', args.label, args.lat, args.lon, 'h-g-i-g-o')
    s.sendall(xml.encode())
    time.sleep(5.0)   # FTS relays a client's events while it is connected: closing within a second loses the marker
    s.close()
    print(f'marker "{args.label}" at {args.lat:.6f}, {args.lon:.6f} sent to {args.host}:{args.port}; '
          f'cot_bridge turns it into: send the nearest vehicle to marker "{args.label}" at {args.lat:.6f}, {args.lon:.6f}')
    return 0


def main(argv=None):
    sys.stdout.reconfigure(line_buffering=True)   # watch is often piped into tee or head
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8087)
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('watch')
    m = sub.add_parser('marker')
    m.add_argument('label')
    m.add_argument('lat', type=float)
    m.add_argument('lon', type=float)
    args = ap.parse_args(argv)
    return watch(args) if args.cmd == 'watch' else marker(args)


if __name__ == '__main__':
    sys.exit(main())
