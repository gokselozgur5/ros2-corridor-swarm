"""Watches every vehicle fix against the corridor polygon.

Publishes corridor_msgs/GeofenceEvent on /fleet/geofence_events (EXIT, ENTER, WARN) and,
on EXIT, an empty Path on <vehicle>/path so the follower stops. Vehicles are discovered
from parameter `vehicles` (e.g. ['v1','v2']); their fixes are read from /<v>/fix.
"""
import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from corridor_msgs.msg import GeofenceEvent
from corridor_nav.geofence import LocalFrame, load_geojson_polygon, signed_distance


class GeofenceMonitor(Node):
    def __init__(self):
        super().__init__('geofence_monitor')
        self.declare_parameter('corridor_geojson', '')
        self.declare_parameter('vehicles', ['v1'])
        self.declare_parameter('warn_margin_m', 20.0)
        self.declare_parameter('cancel_on_exit', True)

        path = self.get_parameter('corridor_geojson').value
        if not path:
            raise ValueError('corridor_geojson parameter is required')
        self.corridor_id, ring = load_geojson_polygon(path)
        self.frame = LocalFrame(*ring[0])
        self.poly = [self.frame.to_xy(lat, lon) for lat, lon in ring]
        self.warn = self.get_parameter('warn_margin_m').value
        self.inside = {}
        self.warned = {}

        self.event_pub = self.create_publisher(GeofenceEvent, '/fleet/geofence_events', 10)
        self.path_pubs = {}
        for v in self.get_parameter('vehicles').value:
            self.create_subscription(NavSatFix, f'/{v}/fix', lambda m, v=v: self.on_fix(v, m), 10)
            self.path_pubs[v] = self.create_publisher(Path, f'/{v}/path', 10)
        self.get_logger().info(f'corridor {self.corridor_id}: {len(self.poly)} vertices, '
                               f'{len(self.path_pubs)} vehicles, warn {self.warn} m')

    def emit(self, vehicle, kind, d, fix):
        ev = GeofenceEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.header.frame_id = 'map'
        ev.vehicle = vehicle
        ev.corridor_id = self.corridor_id
        ev.type = kind
        ev.distance_m = float(d)
        ev.fix = fix
        self.event_pub.publish(ev)

    def on_fix(self, vehicle, fix):
        x, y = self.frame.to_xy(fix.latitude, fix.longitude)
        d = signed_distance(x, y, self.poly)
        inside = d >= 0.0
        was = self.inside.get(vehicle)
        self.inside[vehicle] = inside
        if was is None:
            if not inside:
                self.leave(vehicle, d, fix)
            return
        if was and not inside:
            self.leave(vehicle, d, fix)
        elif not was and inside:
            self.emit(vehicle, GeofenceEvent.ENTER, d, fix)
            self.get_logger().info(f'{vehicle} back inside {self.corridor_id}')
        elif inside and d < self.warn and not self.warned.get(vehicle):
            self.warned[vehicle] = True
            self.emit(vehicle, GeofenceEvent.WARN, d, fix)
        elif inside and d >= self.warn:
            self.warned[vehicle] = False

    def leave(self, vehicle, d, fix):
        self.emit(vehicle, GeofenceEvent.EXIT, d, fix)
        self.get_logger().warn(f'{vehicle} left {self.corridor_id} by {-d:.1f} m')
        if self.get_parameter('cancel_on_exit').value and vehicle in self.path_pubs:
            self.path_pubs[vehicle].publish(Path())


def main():
    rclpy.init()
    node = GeofenceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
