"""Periodic fleet summary on Haiku: every `period_s` one short paragraph on /mission/summary.

Reads the same vehicle topics as mission_planner but calls no tools; a single
claude-haiku-5-5 request with the fleet snapshot in the prompt. Usage is accumulated and
published on /mission/summary_cost (std_msgs/String JSON) so the hourly cost can be measured
(issue #14). Refusals and SDK errors are logged and skipped, never fatal.
"""
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from corridor_msgs.msg import GeofenceEvent

STATUS_MODEL = 'claude-haiku-5-5'
PRICE_IN, PRICE_OUT = 0.10, 0.50          # $ per 1M tokens, Haiku 5.5 up to 100K prompt
SYSTEM = ('You write the one-paragraph status line of a small simulated vehicle fleet patrolling a '
          'corridor. Be concrete: which vehicles move, where, anything abnormal. Under 60 words.')


class StatusLoop(Node):
    def __init__(self, client=None):
        super().__init__('status_loop')
        self.declare_parameter('vehicles', ['v1'])
        self.declare_parameter('period_s', 30.0)
        self.declare_parameter('model', STATUS_MODEL)
        self.state = {v: {} for v in self.get_parameter('vehicles').value}
        self.events = []
        for v in self.state:
            self.create_subscription(NavSatFix, f'/{v}/fix', lambda m, v=v: self.state[v].update(lat=round(m.latitude, 5), lon=round(m.longitude, 5)), 10)
            # odom gives speed only: its position starts at the spawn and drifts, the fleet is positioned by fix (#19)
            self.create_subscription(Odometry, f'/{v}/odom', lambda m, v=v: self.state[v].update(
                speed=round(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y), 2)), 10)
            self.create_subscription(String, f'/{v}/follower/status', lambda m, v=v: self.state[v].update(follower=m.data),
                                     QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))  # latched by the follower
        self.create_subscription(GeofenceEvent, '/fleet/geofence_events',
                                 lambda e: self.events.append({'vehicle': e.vehicle, 'type': int(e.type), 'd': round(e.distance_m, 1)}), 10)
        self.summary_pub = self.create_publisher(String, '/mission/summary', 10)
        self.cost_pub = self.create_publisher(String, '/mission/summary_cost', 10)
        if client is None:
            import anthropic
            client = anthropic.Anthropic()
        self.client = client
        self.in_tokens = 0
        self.out_tokens = 0
        self.calls = 0
        self.t0 = time.time()
        self.create_timer(self.get_parameter('period_s').value, self.tick)

    def snapshot(self):
        snap = {'t': round(time.time() - self.t0), 'vehicles': self.state, 'recent_geofence_events': self.events[-5:]}
        self.events = self.events[-5:]
        return json.dumps(snap)

    def tick(self):
        try:
            resp = self.client.messages.create(
                model=self.get_parameter('model').value, max_tokens=200, system=SYSTEM,
                output_config={'effort': 'low'},
                messages=[{'role': 'user', 'content': f'Fleet snapshot: {self.snapshot()}'}])
        except Exception as exc:
            self.get_logger().warn(f'status call failed: {type(exc).__name__}: {exc}')
            return
        self.calls += 1
        self.in_tokens += resp.usage.input_tokens
        self.out_tokens += resp.usage.output_tokens
        if resp.stop_reason == 'refusal':
            self.get_logger().warn('status summary refused')
            return
        text = ' '.join(b.text for b in resp.content if getattr(b, 'type', '') == 'text').strip()
        self.summary_pub.publish(String(data=text))
        hours = max(1e-6, (time.time() - self.t0) / 3600.0)
        cost = (self.in_tokens * PRICE_IN + self.out_tokens * PRICE_OUT) / 1e6
        self.cost_pub.publish(String(data=json.dumps({
            'calls': self.calls, 'input_tokens': self.in_tokens, 'output_tokens': self.out_tokens,
            'usd': round(cost, 5), 'usd_per_hour': round(cost / hours, 4)})))
        self.get_logger().info(f'summary: {text}')
        self.get_logger().info(f'cost: calls={self.calls} in={self.in_tokens} out={self.out_tokens} usd={cost:.5f} usd_per_hour={cost / hours:.4f}')


def main():
    rclpy.init()
    node = StatusLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
