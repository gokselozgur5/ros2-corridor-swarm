"""Patrol action server: laps along the corridor polygon, driven through the waypoint follower.

Runs in a vehicle namespace. Goal -> a Path on `path` (polygon vertices inset by `inset_m`,
starting at the vertex nearest the vehicle); progress from `odom` and `follower/status`.
Cancel or a geofence EXIT for this vehicle stops the lap.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from corridor_msgs.action import Patrol
from corridor_msgs.msg import GeofenceEvent
from corridor_nav.geofence import LocalFrame, load_geojson_polygon


def inset_polygon(poly, inset):
    """Move each vertex toward the centroid by `inset` metres (good enough for convex corridors)."""
    cx = sum(p[0] for p in poly) / len(poly)
    cy = sum(p[1] for p in poly) / len(poly)
    out = []
    for x, y in poly:
        d = math.hypot(x - cx, y - cy)
        if d <= inset:
            out.append((cx, cy))
        else:
            out.append((x + (cx - x) * inset / d, y + (cy - y) * inset / d))
    return out


def lap_route(poly, start_xy, clockwise):
    """Vertices in patrol order starting at the one nearest start_xy, closed back to the start."""
    ring = list(poly)
    # Shoelace: positive area means counter-clockwise.
    area = sum(ring[i][0] * ring[(i + 1) % len(ring)][1] - ring[(i + 1) % len(ring)][0] * ring[i][1]
               for i in range(len(ring)))
    if (area > 0) == clockwise:
        ring.reverse()
    k = min(range(len(ring)), key=lambda i: math.dist(ring[i], start_xy))
    ring = ring[k:] + ring[:k]
    return ring + [ring[0]]


class PatrolServer(Node):
    def __init__(self):
        super().__init__('patrol_server')
        self.declare_parameter('corridor_geojson', '')
        self.declare_parameter('vehicle', 'v1')
        self.declare_parameter('inset_m', 15.0)
        path = self.get_parameter('corridor_geojson').value
        if not path:
            raise ValueError('corridor_geojson parameter is required')
        self.corridor_id, ring = load_geojson_polygon(path)
        self.frame = LocalFrame(*ring[0])
        self.poly = [self.frame.to_xy(lat, lon) for lat, lon in ring]
        self.vehicle = self.get_parameter('vehicle').value

        self.pose = None
        self.status = 'idle'
        self.breached = False
        self.lock = threading.Lock()
        cb = ReentrantCallbackGroup()
        self.path_pub = self.create_publisher(Path, 'path', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20, callback_group=cb)
        self.create_subscription(String, 'follower/status', self.on_status, 10, callback_group=cb)
        self.create_subscription(GeofenceEvent, '/fleet/geofence_events', self.on_geofence, 10, callback_group=cb)
        self.server = ActionServer(self, Patrol, 'patrol', execute_callback=self.execute,
                                   goal_callback=self.on_goal, cancel_callback=self.on_cancel,
                                   callback_group=cb)
        self.get_logger().info(f'{self.vehicle}: patrol server on corridor {self.corridor_id}')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def on_status(self, msg):
        self.status = msg.data

    def on_geofence(self, ev):
        if ev.vehicle == self.vehicle and ev.type == GeofenceEvent.EXIT:
            self.breached = True

    def on_goal(self, goal):
        if goal.corridor_id and goal.corridor_id != self.corridor_id:
            self.get_logger().warn(f'unknown corridor {goal.corridor_id}')
            return GoalResponse.REJECT
        if goal.vehicle and goal.vehicle != self.vehicle:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, _):
        return CancelResponse.ACCEPT

    def send_route(self, route):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in route:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def execute(self, handle):
        goal = handle.request
        result = Patrol.Result()
        deadline = time.time() + 5.0
        while self.pose is None and time.time() < deadline:
            time.sleep(0.1)
        if self.pose is None:
            result.result = 'error'
            handle.abort()
            return result
        inset = inset_polygon(self.poly, self.get_parameter('inset_m').value)
        route = lap_route(inset, self.pose, goal.clockwise)
        lap_len = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
        self.breached = False
        laps_done = 0
        distance = 0.0
        last = self.pose
        while goal.laps == 0 or laps_done < goal.laps:
            self.status = 'following'
            self.send_route(route)
            time.sleep(0.5)
            while self.status == 'following':
                if handle.is_cancel_requested:
                    self.send_route([])
                    handle.canceled()
                    result.laps_done = laps_done
                    result.distance_m = distance
                    result.result = 'cancelled'
                    return result
                if self.breached:
                    handle.abort()
                    result.laps_done = laps_done
                    result.distance_m = distance
                    result.result = 'geofence'
                    return result
                if self.pose is not None:
                    distance += math.dist(last, self.pose)
                    last = self.pose
                fb = Patrol.Feedback()
                fb.lap = laps_done + 1
                fb.lap_progress = float(min(1.0, (distance - laps_done * lap_len) / lap_len)) if lap_len else 0.0
                fb.distance_m = distance
                nxt = min(route, key=lambda p: -math.dist(p, last))  # placeholder farthest; refined below
                fb.next_waypoint = Point(x=nxt[0], y=nxt[1])
                handle.publish_feedback(fb)
                time.sleep(0.5)
            if self.status != 'arrived':
                break
            laps_done += 1
        handle.succeed()
        result.laps_done = laps_done
        result.distance_m = distance
        result.result = 'done'
        return result


def main():
    rclpy.init()
    node = PatrolServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
