"""cot_bridge node tests. NEEDS ROS: rclpy and the message packages, so they run on Thor
(`source env.sh && python3 -m pytest -q ws/src/corridor_cot/test/test_cot_bridge_node.py`) and skip where
ROS is absent. The TAK socket is replaced by a fake that hands the reader thread its chunks and records
what send_pli writes; the request publisher and the logger are replaced by recorders."""
import datetime
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from corridor_cot import cot_bridge  # noqa: E402


class FakeSock:
    """recv hands out the chunks one per call, then blocks until the test ends; sendall records PLI batches."""

    def __init__(self, chunks=()):
        self.chunks = list(chunks)
        self.done = threading.Event()
        self.sent = []

    def recv(self, n):
        if self.chunks:
            return self.chunks.pop(0)
        self.done.wait()
        return b''

    def sendall(self, data):
        self.sent.append(data.decode())


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
    rclpy.init(args=['--ros-args', '-p', 'period_s:=1000.0', '-p', 'host:=127.0.0.1', '-p', 'port:=1'])
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


# ----- cot-1: the reader thread survives a bad event and logs it ----------------------------------

def test_reader_survives_an_event_that_raises_and_logs_it(bridge, monkeypatch):
    real = cot_bridge.parse_marker

    def exploding(xml):
        if 'boom' in xml:
            raise RuntimeError('boom')
        return real(xml)

    monkeypatch.setattr(cot_bridge, 'parse_marker', exploding)
    feed(bridge, [marker_xml('boom', 52.701, -2.9985, 'BAD'), marker_xml('atak-1', 52.701, -2.9985, 'RALLY1')])
    assert wait_for(lambda: len(bridge.requests.lines) == 1), bridge.log.lines
    assert 'RALLY1' in bridge.requests.lines[0]
    assert any(level == 'warn' and 'boom' in text for level, text in bridge.log.lines), bridge.log.lines


def test_reader_ignores_a_marker_without_coordinates_and_goes_on(bridge):
    broken = b'<event version="2.0" uid="m" type="b-m-p-w"><point lon="-2.9"/></event>'
    feed(bridge, [broken, marker_xml('atak-2', 52.701, -2.9985, 'RALLY2')])
    assert wait_for(lambda: len(bridge.requests.lines) == 1), bridge.log.lines
    assert 'RALLY2' in bridge.requests.lines[0]
