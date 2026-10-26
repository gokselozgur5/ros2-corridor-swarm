"""Patrol action server: laps along the corridor polygon, driven through the waypoint follower.

Runs in a vehicle namespace. Goal -> a Path on `path` (polygon vertices inset by `inset_m`,
starting at the vertex nearest the vehicle); the vehicle's position comes from `fix` through the
corridor's LocalFrame, like the follower's (#19: odometry starts at the spawn and drifts). Cancel or
a geofence EXIT for this vehicle stops the lap.

One goal at a time: a new goal preempts the running one (result 'preempted') and publishes its route
only once the old loop has exited, so the two never write the follower's path over each other.

Several nodes publish on `path` (the planner's goto and stop, geofence_monitor, clicked_goal, wasd,
send_path). Every Path this server publishes carries a unique header stamp, and the follower echoes
the stamp of the Path it holds with each status on `follower/route`. So only an arrival on this
server's own route counts as a lap, a Path from anyone else ends the goal as 'preempted', and a
cancel or abort clears the follower only while it still holds this server's route, never a Path
another node sent since. A cancel's clear goes out inside the cancel callback, before the client is
answered, so a Path the client sends on that answer lands after it.
Results: done | cancelled | geofence | preempted | error.
"""
import math
import threading
import time
from dataclasses import dataclass, field

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

from corridor_msgs.action import Patrol
from corridor_msgs.msg import GeofenceEvent
from corridor_nav.geofence import LocalFrame, load_geojson_polygon
from corridor_nav.pure_pursuit import closest_segment


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


def stamp_ns(stamp):
    """builtin_interfaces/Time -> int nanoseconds."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def goal_key(handle):
    return bytes(handle.goal_id.uuid)


class RouteOwner:
    """Which Path the follower holds and whether this server published it. No ROS here."""

    def __init__(self):
        self.stamp = 0          # ns, the last Path this server published
        self.route = False      # that Path was a route (>= 2 poses), not a clear
        self.pending = False    # published, not yet reported back by the follower
        self.held = None        # ns, the Path the follower last reported on
        self.status = None      # the follower's status for `held`

    def next_stamp(self, now_ns):
        """Unique and increasing, so two Paths never share a stamp even while the clock stands still."""
        return max(now_ns, self.stamp + 1, 1)

    def published(self, stamp, route):
        self.stamp, self.route, self.pending = stamp, route, True

    def report(self, stamp, status):
        self.held, self.status = stamp, status
        if stamp == self.stamp:
            self.pending = False

    def owns(self):
        """The follower holds, or is about to receive, a route of this server's; no other Path replaced it."""
        return self.route and (self.pending or self.held == self.stamp)

    def arrived(self):
        return self.route and self.held == self.stamp and self.status == 'arrived'


@dataclass
class Run:
    """One accepted goal and what has happened to it."""
    handle: object
    laps: int
    laps_done: int = 0
    distance: float = 0.0
    cancelled: bool = False
    preempted: bool = False     # a newer goal takes over
    breached: bool = False
    finished: threading.Event = field(default_factory=threading.Event)


def verdict(run, owner, cancel_requested):
    """What the execute loop does next: 'lap' when the server's own route arrived, a result name to
    finish with, or None to keep waiting."""
    if run.cancelled or cancel_requested:
        return 'cancelled'
    if run.preempted:
        return 'preempted'
    if run.breached:
        return 'geofence'
    if not owner.owns():
        return 'preempted'
    if owner.arrived():
        return 'lap'
    return None


class PatrolServer(Node):
    def __init__(self, **node_kwargs):
        super().__init__('patrol_server', **node_kwargs)
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
        self.owner = RouteOwner()
        self.lock = threading.Lock()      # owner
        self.wake = threading.Event()     # the execute loop re-evaluates on every report, event or cancel
        self.runs = {}                    # goal id -> Run, from acceptance to the end of its loop
        self.current = None               # the Run whose loop is running or about to
        self.goal_lock = threading.Lock()   # one goal takes over at a time
        cb = ReentrantCallbackGroup()
        self.path_pub = self.create_publisher(Path, 'path', 10)
        self.create_subscription(NavSatFix, 'fix', self.on_fix, 20, callback_group=cb)
        self.create_subscription(Header, 'follower/route', self.on_route,
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=cb)
        self.create_subscription(GeofenceEvent, '/fleet/geofence_events', self.on_geofence, 10, callback_group=cb)
        self.server = ActionServer(self, Patrol, 'patrol', execute_callback=self.execute,
                                   goal_callback=self.on_goal, cancel_callback=self.on_cancel,
                                   handle_accepted_callback=self.on_accepted, callback_group=cb)
        self.get_logger().info(f'{self.vehicle}: patrol server on corridor {self.corridor_id}')

    def on_fix(self, msg):
        self.pose = self.frame.to_xy(msg.latitude, msg.longitude)

    def on_route(self, hdr):
        with self.lock:
            self.owner.report(stamp_ns(hdr.stamp), hdr.frame_id)
        self.wake.set()

    def on_geofence(self, ev):
        if ev.vehicle == self.vehicle and ev.type == GeofenceEvent.EXIT:
            run = self.current
            if run is not None:
                run.breached = True
                self.wake.set()

    def on_goal(self, goal):
        if goal.corridor_id and goal.corridor_id != self.corridor_id:
            self.get_logger().warn(f'unknown corridor {goal.corridor_id}')
            return GoalResponse.REJECT
        if goal.vehicle and goal.vehicle != self.vehicle:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_accepted(self, handle):
        run = Run(handle, handle.request.laps)
        self.runs[goal_key(handle)] = run
        with self.goal_lock:
            old = self.current
            if old is not None:
                self.get_logger().info('new goal preempts the running one')
                old.preempted = True
                self.wake.set()
                if not old.finished.wait(5.0):
                    self.get_logger().error('the running goal did not stop in 5 s')
            self.current = run
            handle.execute()

    def on_cancel(self, handle):
        run = self.runs.get(goal_key(handle))
        if run is not None:
            run.cancelled = True
            self.clear_if_owned()   # before the client is answered: a Path it sends on the answer lands after this
            self.wake.set()
        return CancelResponse.ACCEPT

    def publish(self, route):
        """A Path with a stamp of this server's own; called with the lock held."""
        path = Path()
        path.header.frame_id = 'map'
        stamp = self.owner.next_stamp(self.get_clock().now().nanoseconds)
        path.header.stamp.sec, path.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        for x, y in route:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.owner.published(stamp, len(route) >= 2)
        self.path_pub.publish(path)

    def send_route(self, route):
        with self.lock:
            self.publish(route)

    def clear_if_owned(self):
        with self.lock:
            if self.owner.owns():
                self.publish([])

    def execute(self, handle):
        run = self.runs[goal_key(handle)]
        try:
            return self.patrol(run)
        finally:
            self.runs.pop(goal_key(handle), None)
            if self.current is run:
                self.current = None
            run.finished.set()

    def patrol(self, run):
        handle = run.handle
        deadline = time.time() + 5.0
        while self.pose is None and time.time() < deadline and not (run.cancelled or run.preempted or handle.is_cancel_requested):
            time.sleep(0.1)
        if run.cancelled or handle.is_cancel_requested:
            return self.finish(run, 'cancelled')
        if run.preempted:
            return self.finish(run, 'preempted')
        if self.pose is None:
            return self.finish(run, 'error')
        inset = inset_polygon(self.poly, self.get_parameter('inset_m').value)
        route = lap_route(inset, self.pose, handle.request.clockwise)
        lap_len = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
        last = self.pose
        self.send_route(route)
        while True:
            self.wake.wait(0.5)
            self.wake.clear()
            with self.lock:
                what = verdict(run, self.owner, handle.is_cancel_requested)
            if what == 'lap':
                run.laps_done += 1
                if run.laps and run.laps_done >= run.laps:
                    return self.finish(run, 'done')
                self.send_route(route)
                continue
            if what is not None:
                return self.finish(run, what)
            run.distance += math.dist(last, self.pose)
            last = self.pose
            fb = Patrol.Feedback()
            fb.lap = run.laps_done + 1
            fb.lap_progress = float(min(1.0, (run.distance - run.laps_done * lap_len) / lap_len)) if lap_len else 0.0
            fb.distance_m = run.distance
            seg, _ = closest_segment(last[0], last[1], route)
            nxt = route[min(seg + 1, len(route) - 1)]
            fb.next_waypoint = Point(x=nxt[0], y=nxt[1])
            handle.publish_feedback(fb)

    def finish(self, run, name):
        handle = run.handle
        if name in ('cancelled', 'geofence', 'error'):
            self.clear_if_owned()
        if name == 'cancelled':
            # rclpy moves the goal to CANCELING right after the cancel callback; canceled() is legal only from there.
            deadline = time.time() + 1.0
            while not handle.is_cancel_requested and time.time() < deadline:
                time.sleep(0.005)
        result = Patrol.Result()
        result.laps_done = run.laps_done
        result.distance_m = run.distance
        result.result = name
        if name == 'done':
            handle.succeed()
        elif name == 'cancelled' and handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        self.get_logger().info(f'patrol {name}: {run.laps_done} laps, {run.distance:.0f} m')
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
