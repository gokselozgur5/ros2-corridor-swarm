"""cot_bridge node tests. NEEDS ROS: rclpy and the message packages, so they run on Thor
(`ROS_DOMAIN_ID=77 REQUIRE_ROS=1 python3 -m pytest -q -rs ws/src/corridor_cot/test/test_cot_bridge_node.py`,
on a private domain because the tests publish /v1/fix) and skip where ROS is absent unless REQUIRE_ROS is set. The TAK socket is replaced by a fake that hands the reader thread its chunks and records
what send_pli writes; the request publisher and the logger are replaced by recorders."""
import datetime
import threading
import time

import os

import pytest

if os.environ.get('REQUIRE_ROS'):
    import rclpy   # an ImportError here is the failure REQUIRE_ROS asks for
else:
    rclpy = pytest.importorskip('rclpy')
if not hasattr(rclpy, '__file__'):
    if os.environ.get('REQUIRE_ROS'):
        pytest.exit('REQUIRE_ROS is set but rclpy is a stand-in: source the ROS workspace first', returncode=3)
    pytest.skip('a stand-in rclpy from another package: the real stack runs these on Thor', allow_module_level=True)
if os.environ.get('ROS_DOMAIN_ID', '0') == '42':
    pytest.exit('ROS_DOMAIN_ID is the demo domain 42 and these tests publish /v1/fix: run them with ROS_DOMAIN_ID=77', returncode=3)

from sensor_msgs.msg import NavSatFix, NavSatStatus  # noqa: E402

from corridor_cot import cot_bridge  # noqa: E402


class FakeSock:
    """recv hands out the chunks one per call, then blocks until the test ends; sendall records PLI batches."""

    def __init__(self, chunks=()):
        self.chunks = list(chunks)
        self.done = threading.Event()
        self.entered = threading.Event()   # the reader thread is inside recv
        self.closed = False
        self.sent = []

    def recv(self, n):
        self.entered.set()
        if self.chunks:
            return self.chunks.pop(0)
        self.done.wait()
        return b''

    def sendall(self, data):
        self.sent.append(data.decode())

    def shutdown(self, how):
        pass

    def close(self):
        self.closed = True


class BrokenSock(FakeSock):
    """A connection the server has dropped: reads block, writes fail."""

    def sendall(self, data):
        raise OSError('connection reset by peer')


class Recorder:
    """Stands in for the /mission/request publisher (publish) and for the node logger (info/warn/...)."""

    def __init__(self):
        self.lines = []

    def publish(self, msg):
        self.lines.append(msg.data)

    def debug(self, text, **kw):
        self.lines.append(('debug', text))

    def info(self, text, **kw):
        self.lines.append(('info', text))

    def warn(self, text, **kw):
        self.lines.append(('warn', text))

    warning = warn

    def error(self, text, **kw):
        self.lines.append(('error', text))

    fatal = error


def marker_xml(uid, lat, lon, callsign, stale_min=5):
    now = datetime.datetime.now(datetime.timezone.utc)
    f = lambda t: t.strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3] + 'Z'  # noqa: E731
    return (f'<event version="2.0" uid="{uid}" type="b-m-p-w" how="h-g-i-g-o" time="{f(now)}" start="{f(now)}" '
            f'stale="{f(now + datetime.timedelta(minutes=stale_min))}"><point lat="{lat}" lon="{lon}" hae="0" ce="9999999" '
            f'le="9999999"/><detail><contact callsign="{callsign}"/></detail></event>').encode()


def wait_for(cond, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while not cond() and time.monotonic() < deadline:
        time.sleep(0.05)
    return cond()


@pytest.fixture
def bridge():
    # period_s large: PLI goes out only when a test calls send_pli itself; port 1 so a stray connect fails fast
    rclpy.init(args=['--ros-args', '-p', 'period_s:=1000.0', '-p', 'fix_timeout_s:=0.5', '-p', 'host:=127.0.0.1', '-p', 'port:=1'])
    node = cot_bridge.CotBridge()
    node.requests, node.log, node.fakes = Recorder(), Recorder(), []
    node.request_pub = node.requests
    node.get_logger = lambda: node.log
    yield node
    for fake in node.fakes:
        fake.done.set()
    node.destroy_node()
    rclpy.shutdown()


def feed(node, chunks):
    """Give the reader thread a socket to read; it polls self.sock once a second while it is None."""
    fake = FakeSock(chunks)
    node.fakes.append(fake)
    with node.lock:
        node.sock = fake
    return fake


def deliver_fix(node, status, lat=52.7, lon=-3.0, timeout_s=5.0):
    """Publish /v1/fix from a helper node and spin the bridge until it has (or, for NO_FIX, has not) recorded it."""
    helper = rclpy.create_node('fix_source')
    pub = helper.create_publisher(NavSatFix, '/v1/fix', 10)
    msg = NavSatFix()
    msg.latitude, msg.longitude, msg.status.status = lat, lon, status
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.state['v1'].get('lat') == lat:
                return True
        return False
    finally:
        helper.destroy_node()


# ----- cot-1: the reader thread survives a bad event and logs it ----------------------------------

def test_reader_survives_an_event_that_raises_and_logs_it(bridge, monkeypatch):
    real = cot_bridge.marker_or_reason

    def exploding(xml):
        if 'boom' in xml:
            raise RuntimeError('boom')
        return real(xml)

    monkeypatch.setattr(cot_bridge, 'marker_or_reason', exploding)
    feed(bridge, [marker_xml('boom', 52.701, -2.9985, 'BAD'), marker_xml('atak-1', 52.701, -2.9985, 'RALLY1')])
    assert wait_for(lambda: len(bridge.requests.lines) == 1), bridge.log.lines
    assert 'RALLY1' in bridge.requests.lines[0]
    assert any(level == 'warn' and 'boom' in text for level, text in bridge.log.lines), bridge.log.lines


def test_reader_ignores_a_marker_without_coordinates_and_goes_on(bridge):
    broken = b'<event version="2.0" uid="m" type="b-m-p-w"><point lon="-2.9"/></event>'
    feed(bridge, [broken, marker_xml('atak-2', 52.701, -2.9985, 'RALLY2')])
    assert wait_for(lambda: len(bridge.requests.lines) == 1), bridge.log.lines
    assert 'RALLY2' in bridge.requests.lines[0]


# ----- a new connection registers with one event; the fleet follows on the next tick ------------------
# FreeTAKServer 1.9.9.2 parses a connection's first message, up to its </event>, as the client's registration and
# drops the client when more follows in that chunk: a three-vehicle batch on a fresh socket reconnected once a second.

def test_a_new_connection_registers_with_one_event_and_the_fleet_follows_on_the_next_tick(bridge, monkeypatch):
    now = bridge.now_s()
    bridge.state = {v: {'lat': 52.7, 'lon': -3.0 + i * 1e-3, 't': now} for i, v in enumerate(['v1', 'v2', 'v3'])}
    fake = FakeSock()
    bridge.fakes.append(fake)
    monkeypatch.setattr(bridge, 'connect', lambda: fake)
    with bridge.lock:
        bridge.sock = None
    bridge.send_pli()
    assert [s.count('<event') for s in fake.sent] == [1], fake.sent
    assert 'uid="corridor-v1"' in fake.sent[0]
    bridge.send_pli()
    assert [s.count('<event') for s in fake.sent] == [1, 3], fake.sent
    assert all(f'uid="corridor-{v}"' in fake.sent[1] for v in ('v1', 'v2', 'v3'))


# ----- cot-2: a vehicle that stops reporting is not re-sent as a fresh PLI ---------------------------

def test_silent_vehicle_gets_no_pli_after_fix_timeout(bridge):
    sock = feed(bridge, [])
    assert deliver_fix(bridge, NavSatStatus.STATUS_FIX)
    bridge.send_pli()
    assert len(sock.sent) == 1 and 'uid="corridor-v1"' in sock.sent[0]
    time.sleep(1.0)  # fix_timeout_s is 0.5 in the fixture and nothing new arrived
    bridge.send_pli()
    assert len(sock.sent) == 1, sock.sent[1:]
    assert deliver_fix(bridge, NavSatStatus.STATUS_FIX, lat=52.71)
    bridge.send_pli()
    assert len(sock.sent) == 2 and 'lat="52.7100000"' in sock.sent[1]


def test_fix_without_a_fix_status_is_ignored(bridge):
    sock = feed(bridge, [])
    assert not deliver_fix(bridge, NavSatStatus.STATUS_NO_FIX, timeout_s=1.5)
    assert 'lat' not in bridge.state['v1']
    bridge.send_pli()
    assert sock.sent == []
    assert deliver_fix(bridge, NavSatStatus.STATUS_FIX)  # the same path delivers once the status says fix


# ----- cot-4: one request per placement ------------------------------------------------------------

def test_same_marker_twice_is_one_request_and_a_move_is_another(bridge):
    same = marker_xml('atak-1', 52.7010, -2.9985, 'RALLY1')
    moved = marker_xml('atak-1', 52.7020, -2.9985, 'RALLY1')  # about 110 m north
    feed(bridge, [same, same, marker_xml('atak-1', 52.7010, -2.9985, 'RALLY'), moved])
    assert wait_for(lambda: len(bridge.requests.lines) == 2), bridge.requests.lines
    time.sleep(0.5)
    assert len(bridge.requests.lines) == 2, bridge.requests.lines
    assert '52.701000' in bridge.requests.lines[0] and '52.702000' in bridge.requests.lines[1]


def test_stale_marker_is_not_a_request(bridge):
    feed(bridge, [marker_xml('atak-old', 52.7010, -2.9985, 'OLD', stale_min=-5), marker_xml('atak-3', 52.7010, -2.9985, 'RALLY3')])
    assert wait_for(lambda: len(bridge.requests.lines) == 1), bridge.log.lines
    assert 'RALLY3' in bridge.requests.lines[0]


# ----- cot_generation (Aether's review, 2026-10-28) --------------------------------------------------------

def test_reader_of_a_replaced_connection_leaves_the_new_one_alone(bridge, monkeypatch):
    """send_pli drops a connection whose write failed and connects again on the next tick, while the reader
    thread may still be blocked in recv on the old one. When that recv finally ends, only the old connection
    may be forgotten: the replacement, which has already registered with the server, stays."""
    old = BrokenSock()
    bridge.fakes.append(old)
    with bridge.lock:
        bridge.sock = old
    assert old.entered.wait(3.0), 'reader did not start reading the old connection'
    bridge.state['v1'].update(lat=52.7, lon=-3.0, t=bridge.now_s())
    bridge.send_pli()                                   # the write fails: connection dropped
    assert bridge.sock is None
    new = FakeSock()
    bridge.fakes.append(new)
    monkeypatch.setattr(bridge, 'connect', lambda: new)
    bridge.send_pli()                                   # reconnected and registered
    assert bridge.sock is new and len(new.sent) == 1
    old.done.set()                                      # the server side of the old connection closes now
    assert new.entered.wait(3.0), 'reader did not move on to the new connection'
    assert bridge.sock is new, 'the reader cleared the replacement connection on the old one\'s EOF'
    assert old.closed, 'the dropped connection was not closed'
