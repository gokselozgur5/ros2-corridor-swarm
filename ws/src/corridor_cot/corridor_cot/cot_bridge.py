"""CoT bridge (issue #16).

Every vehicle's fix/odom -> one PLI event per `period_s` over TCP to the TAK server (FTS_HOST:8087,
plain CoT, no TLS); a vehicle whose fix is older than `fix_timeout_s` is left out, so its icon goes stale
in ATAK. Events from the server that are markers -> std_msgs/String on /mission/request (cot.request_text:
'send the nearest vehicle to marker "<label>" at lat, lon', the label a sanitised callsign or uid, since the
stream is unauthenticated and the text becomes the model's user turn), so the planner can act on them; one
request per placement (a uid re-sent at the same spot is a rename or a refresh, not a new order), none for a
marker already stale. Reconnects on any socket error; a (re)connection's first write is a single event, which
FreeTAKServer takes as the client's registration, and the fleet follows on the next tick. The reader thread
forgets only the connection it was reading, so a replacement the sender made survives the old one's EOF.
Each request carries a tag ('[req N] ') that mission_planner echoes in its busy reply; the gate forgets the
placement the tag names, so ATAK's re-send of that marker is asked again (two markers may share one text).
Never raises out of a callback."""
import os
import socket
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from corridor_cot.cot import MarkerGate, course_from_quaternion, marker_or_reason, pli_events, request_text


def _discard(sock):
    """Release a connection the bridge no longer uses; shutdown wakes a reader blocked in recv on it.
    A stand-in with neither method (a test's fake that only reads and writes) is forgotten all the same."""
    if sock is None:
        return
    for name, args in (('shutdown', (socket.SHUT_RDWR,)), ('close', ())):
        try:
            getattr(sock, name)(*args)
        except Exception:
            pass


class CotBridge(Node):
    def __init__(self):
        super().__init__('cot_bridge')
        self.declare_parameter('vehicles', ['v1'])
        self.declare_parameter('host', os.environ.get('FTS_HOST', '127.0.0.1'))
        self.declare_parameter('port', 8087)
        self.declare_parameter('period_s', 2.0)
        self.declare_parameter('fix_timeout_s', 5.0)
        self.declare_parameter('marker_move_m', 5.0)
        self.declare_parameter('team', 'Cyan')
        self.gate = MarkerGate(self.get_parameter('marker_move_m').value)
        self.state = {v: {} for v in self.get_parameter('vehicles').value}
        for v in self.state:
            self.create_subscription(NavSatFix, f'/{v}/fix', lambda m, v=v: self.on_fix(v, m), 10)
            self.create_subscription(Odometry, f'/{v}/odom', lambda m, v=v: self.state[v].update(
                course=course_from_quaternion(m.pose.pose.orientation.z, m.pose.pose.orientation.w),
                speed=abs(m.twist.twist.linear.x)), 10)
        self.request_pub = self.create_publisher(String, '/mission/request', 10)
        self.create_subscription(String, '/mission/reply', self.on_reply, 10)
        self.gate_lock = threading.Lock()   # the reader thread admits, the executor resolves busy replies
        self.sock = None
        self.lock = threading.Lock()
        self.create_timer(self.get_parameter('period_s').value, self.send_pli)
        threading.Thread(target=self.reader, daemon=True).start()

    def connect(self):
        host, port = self.get_parameter('host').value, self.get_parameter('port').value
        s = socket.create_connection((host, port), timeout=5.0)
        s.settimeout(None)
        self.get_logger().info(f'connected to TAK server {host}:{port}')
        return s

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_fix(self, v, m):
        if m.status.status < NavSatStatus.STATUS_FIX:
            return
        self.state[v].update(lat=m.latitude, lon=m.longitude, hae=m.altitude, t=self.now_s())

    def send_pli(self):
        events = pli_events(self.state, t=self.now_s(), fix_timeout_s=self.get_parameter('fix_timeout_s').value,
                            team=self.get_parameter('team').value)
        if not events:
            return
        with self.lock:
            try:
                if self.sock is None:
                    self.sock = self.connect()
                    # FreeTAKServer reads a new connection's first message up to its </event> as the client's
                    # registration and drops the client when more follows in the same chunk (1.9.9.2 closed the
                    # socket on a three-vehicle batch, once a second). One event now; batches are fine afterwards.
                    events = events[:1]
                self.sock.sendall(''.join(events).encode())
            except OSError as exc:
                self.get_logger().warn(f'TAK send failed: {exc}', throttle_duration_sec=10.0)
                dead, self.sock = self.sock, None
                _discard(dead)

    def reader(self):
        buf = b''
        while rclpy.ok():
            with self.lock:
                s = self.sock
            if s is None:
                threading.Event().wait(1.0)
                continue
            try:
                chunk = s.recv(65536)
            except OSError:
                chunk = b''
            if not chunk:
                with self.lock:
                    if self.sock is s:          # the sender may already have replaced this connection
                        self.sock = None
                _discard(s)
                buf = b''
                continue
            buf += chunk
            while b'</event>' in buf:
                xml, buf = buf.split(b'</event>', 1)
                start = xml.rfind(b'<event')
                if start < 0:
                    continue
                try:
                    self.on_event(xml[start:].decode(errors='replace') + '</event>')
                except Exception as exc:
                    self.get_logger().warn(f'CoT event dropped: {exc!r}')

    def on_event(self, xml):
        marker, reason = marker_or_reason(xml)
        if reason:
            self.get_logger().warn(f'marker dropped: {reason}')
        if not marker or marker['uid'].startswith('corridor-'):
            return
        with self.gate_lock:
            tag = self.gate.admit(marker)
        if not tag:
            return
        text = f'{tag} {request_text(marker)}'
        self.request_pub.publish(String(data=text))
        self.get_logger().info(f'marker -> request: {text}')

    def on_reply(self, msg):
        """mission_planner drops a request while a plan runs and echoes it, tag and all: the gate forgets that
        placement, so the marker's re-send is asked again."""
        with self.gate_lock:
            uid = self.gate.dropped(msg.data)
        if uid:
            self.get_logger().warn(f'planner busy, marker {uid} will be requested again when ATAK re-sends it')


def main():
    rclpy.init()
    node = CotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
