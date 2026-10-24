"""ROS side of the mission planner.

/mission/request (std_msgs/String) -> Planner.run -> /mission/reply (String) and /mission/llm_trace
(String, one JSON object per event). Tools read the vehicles' fix/odom/follower status, send Patrol
goals, publish a one-point Path for goto, cancel for stop. The planner never publishes cmd_vel.
Refusals and SDK errors become diagnostics on /diagnostics, not crashes.
"""
import collections
import json
import math
import os
import threading

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from corridor_msgs.action import Patrol
from corridor_msgs.msg import GeofenceEvent
from corridor_nav.geofence import LocalFrame, load_geojson_polygon, signed_distance
from corridor_planner.planner import PLANNING_MODEL, Planner
from corridor_planner.tools import SCHEMAS

EVENT_NAMES = {GeofenceEvent.EXIT: 'exit', GeofenceEvent.ENTER: 'enter', GeofenceEvent.WARN: 'warn'}


class Vehicle:
    def __init__(self):
        self.fix = None
        self.odom = None
        self.status = 'unknown'
        self.goal = None       # ClientGoalHandle of the active patrol
        self.mode = 'idle'


class MissionPlanner(Node):
    def __init__(self, client=None):
        super().__init__('mission_planner')
        self.declare_parameter('vehicles', ['v1'])
        self.declare_parameter('corridor_geojson', '')
        self.declare_parameter('model', PLANNING_MODEL)
        self.declare_parameter('effort', 'medium')
        self.declare_parameter('dry_run', False)   # no API: tools still work, request echoes

        path = self.get_parameter('corridor_geojson').value
        self.corridor_id, ring = load_geojson_polygon(path) if path else ('unknown', None)
        self.frame = LocalFrame(*ring[0]) if ring else None
        self.poly = [self.frame.to_xy(lat, lon) for lat, lon in ring] if ring else None

        cb = ReentrantCallbackGroup()
        self.vehicles = {}
        self.path_pubs = {}
        self.patrol_clients = {}
        for v in self.get_parameter('vehicles').value:
            self.vehicles[v] = Vehicle()
            self.create_subscription(NavSatFix, f'/{v}/fix', lambda m, v=v: setattr(self.vehicles[v], 'fix', m), 10, callback_group=cb)
            self.create_subscription(Odometry, f'/{v}/odom', lambda m, v=v: setattr(self.vehicles[v], 'odom', m), 10, callback_group=cb)
            self.create_subscription(String, f'/{v}/follower/status', lambda m, v=v: setattr(self.vehicles[v], 'status', m.data), 10, callback_group=cb)
            self.path_pubs[v] = self.create_publisher(Path, f'/{v}/path', 10)
            self.patrol_clients[v] = ActionClient(self, Patrol, f'/{v}/patrol', callback_group=cb)
        self.events = collections.deque(maxlen=200)
        self.create_subscription(GeofenceEvent, '/fleet/geofence_events', lambda e: self.events.appendleft(e), 20, callback_group=cb)

        self.reply_pub = self.create_publisher(String, '/mission/reply', 10)
        self.trace_pub = self.create_publisher(String, '/mission/llm_trace', 50)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(String, '/mission/request', self.on_request, 10, callback_group=cb)
        # Debug path that skips the model: {"name": "patrol", "input": {...}} runs the tool and publishes its result on the trace.
        self.create_subscription(String, '/mission/tool_call', self.on_tool_call, 10, callback_group=cb)

        tools = {'list_vehicles': self.t_list, 'get_vehicle': self.t_get, 'patrol': self.t_patrol,
                 'goto': self.t_goto, 'stop': self.t_stop, 'geofence_events': self.t_events}
        if client is None and not self.get_parameter('dry_run').value:
            import anthropic
            client = anthropic.Anthropic()
        self.planner = Planner(client, tools=tools, schemas=SCHEMAS,
                               model=self.get_parameter('model').value, effort=self.get_parameter('effort').value) if client else None
        self.tools = tools
        self.busy = threading.Lock()
        self.get_logger().info(f'mission planner: {list(self.vehicles)} on corridor {self.corridor_id}, '
                               f'model {self.get_parameter("model").value}, dry_run={client is None}')

    # ----- tools -----------------------------------------------------------------------------
    def _summary(self, name):
        v = self.vehicles[name]
        d = {'vehicle': name, 'mode': v.mode, 'follower': v.status}
        if v.fix is not None:
            d['lat'] = round(v.fix.latitude, 6)
            d['lon'] = round(v.fix.longitude, 6)
            if self.poly:
                x, y = self.frame.to_xy(v.fix.latitude, v.fix.longitude)
                sd = signed_distance(x, y, self.poly)
                d['inside_corridor'] = sd >= 0
                d['edge_distance_m'] = round(sd, 1)
        if v.odom is not None:
            p = v.odom.pose.pose.position
            d['x'] = round(p.x, 1)
            d['y'] = round(p.y, 1)
            d['speed_mps'] = round(math.hypot(v.odom.twist.twist.linear.x, v.odom.twist.twist.linear.y), 2)
        return d

    def _check(self, name):
        if name not in self.vehicles:
            raise ValueError(f'unknown vehicle {name}; known: {list(self.vehicles)}')

    def t_list(self):
        return {'corridor_id': self.corridor_id, 'vehicles': [self._summary(n) for n in self.vehicles]}

    def t_get(self, vehicle):
        self._check(vehicle)
        return self._summary(vehicle)

    def t_patrol(self, vehicle, laps, clockwise):
        self._check(vehicle)
        client = self.patrol_clients[vehicle]
        if not client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError(f'no patrol server for {vehicle}')
        self.t_stop(vehicle)
        goal = Patrol.Goal(vehicle=vehicle, corridor_id=self.corridor_id, laps=int(laps), speed_mps=0.0, clockwise=bool(clockwise))
        fut = client.send_goal_async(goal)
        done = threading.Event()
        fut.add_done_callback(lambda f: done.set())
        if not done.wait(3.0):
            raise RuntimeError('patrol goal not acknowledged in 3 s')
        handle = fut.result()
        if not handle.accepted:
            return {'accepted': False, 'reason': 'rejected by patrol server'}
        self.vehicles[vehicle].goal = handle
        self.vehicles[vehicle].mode = 'patrol'
        return {'accepted': True, 'vehicle': vehicle, 'laps': laps, 'clockwise': clockwise}

    def t_goto(self, vehicle, lat, lon):
        self._check(vehicle)
        if self.frame is None:
            raise RuntimeError('no corridor loaded, cannot convert lat/lon')
        x, y = self.frame.to_xy(float(lat), float(lon))
        if self.poly and signed_distance(x, y, self.poly) < 0:
            return {'accepted': False, 'reason': 'target is outside the corridor'}
        self.t_stop(vehicle)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        v = self.vehicles[vehicle]
        if v.odom is not None:
            start = PoseStamped(header=path.header)
            start.pose = v.odom.pose.pose
            path.poses.append(start)
        goal = PoseStamped(header=path.header)
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        path.poses.append(goal)
        self.path_pubs[vehicle].publish(path)
        v.mode = 'goto'
        return {'accepted': True, 'vehicle': vehicle, 'x': round(x, 1), 'y': round(y, 1)}

    def t_stop(self, vehicle):
        self._check(vehicle)
        v = self.vehicles[vehicle]
        cancelled = False
        if v.goal is not None:
            v.goal.cancel_goal_async()
            v.goal = None
            cancelled = True
        self.path_pubs[vehicle].publish(Path())
        v.mode = 'stopped'
        return {'stopped': True, 'vehicle': vehicle, 'cancelled_patrol': cancelled}

    def t_events(self, limit):
        out = []
        for ev in list(self.events)[:max(1, int(limit))]:
            out.append({'vehicle': ev.vehicle, 'type': EVENT_NAMES.get(ev.type, str(ev.type)),
                        'distance_m': round(ev.distance_m, 1), 'stamp': ev.header.stamp.sec})
        return out

    # ----- request handling ------------------------------------------------------------------
    def on_tool_call(self, msg):
        call = {}
        try:
            call = json.loads(msg.data)
            result, is_error = self.tools[call['name']](**call.get('input', {})), False
        except Exception as exc:
            result, is_error = {'error': f'{type(exc).__name__}: {exc}'}, True
        ev = {'event': 'tool', 'name': call.get('name'), 'input': call.get('input'),
              'result': result, 'is_error': is_error, 'source': 'debug'}
        self.trace_pub.publish(String(data=json.dumps(ev, default=str)))
        self.get_logger().info(f'debug tool call: {ev}')

    def on_request(self, msg):
        if not self.busy.acquire(blocking=False):
            self.reply('busy with the previous request')
            return
        try:
            threading.Thread(target=self.handle_request, args=(msg.data,), daemon=True).start()
        except Exception:
            self.busy.release()
            raise

    def handle_request(self, text):
        try:
            if self.planner is None:
                self.reply(f'dry run, no model: {text}')
                return
            n0 = len(self.planner.trace)
            out = self.planner.run(text)
            for ev in self.planner.trace[n0:]:
                self.trace_pub.publish(String(data=json.dumps(ev, default=str)))
            if out.refused:
                self.diag(DiagnosticStatus.WARN, f'refusal: {out.category or "uncategorised"}', out)
                self.reply(f'the model declined this request ({out.category or "no category"})')
            elif out.error:
                self.diag(DiagnosticStatus.ERROR, out.error, out)
                self.reply(f'planner error: {out.error}')
            else:
                self.diag(DiagnosticStatus.OK, 'ok', out)
                self.reply(out.text or '(no reply text)')
        finally:
            self.busy.release()

    def reply(self, text):
        self.reply_pub.publish(String(data=text))
        self.get_logger().info(f'reply: {text}')

    def diag(self, level, message, out):
        st = DiagnosticStatus(level=level, name='mission_planner', message=message, hardware_id=self.get_parameter('model').value)
        st.values = [KeyValue(key='tool_calls', value=str(out.tool_calls)), KeyValue(key='turns', value=str(out.turns)),
                     KeyValue(key='input_tokens', value=str(out.input_tokens)), KeyValue(key='output_tokens', value=str(out.output_tokens))]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)


def main():
    rclpy.init()
    node = MissionPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
