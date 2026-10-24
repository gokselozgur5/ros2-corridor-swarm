"""What the Foxglove map cannot draw from raw fixes: the corridor polygon, each vehicle's patrol route,
and a heading line per vehicle. Published as foxglove_msgs/GeoJSON on /map/overlay (Map panel) and as
visualization_msgs/MarkerArray on /map/markers (3D panel), both at 2 Hz."""
import json
import math

import rclpy
from foxglove_msgs.msg import GeoJSON
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from corridor_nav.geofence import LocalFrame, load_geojson_polygon
from corridor_nav.patrol_server import inset_polygon

COLORS = {'v1': ('#ff8c00', (1.0, 0.55, 0.0)), 'v2': ('#00bfff', (0.0, 0.75, 1.0)), 'v3': ('#7cfc00', (0.49, 0.99, 0.0))}


class MapOverlay(Node):
    def __init__(self):
        super().__init__('map_overlay')
        self.declare_parameter('corridor_geojson', '')
        self.declare_parameter('vehicles', ['v1'])
        self.declare_parameter('insets_m', [30.0])
        self.declare_parameter('heading_len_m', 12.0)
        self.corridor_id, ring = load_geojson_polygon(self.get_parameter('corridor_geojson').value)
        self.frame = LocalFrame(*ring[0])
        self.poly = [self.frame.to_xy(lat, lon) for lat, lon in ring]
        self.vehicles = list(self.get_parameter('vehicles').value)
        insets = list(self.get_parameter('insets_m').value)
        self.routes = {v: inset_polygon(self.poly, insets[i] if i < len(insets) else insets[-1]) for i, v in enumerate(self.vehicles)}
        self.pose = {}
        for v in self.vehicles:
            self.create_subscription(Odometry, f'/{v}/odom', lambda m, v=v: self.on_odom(v, m), 10)
            self.create_subscription(NavSatFix, f'/{v}/fix', lambda m, v=v: self.pose.setdefault(v, {}).update(lat=m.latitude, lon=m.longitude), 10)
        self.geo_pub = self.create_publisher(GeoJSON, '/map/overlay', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/map/markers', 1)
        self.create_timer(0.5, self.publish)

    def on_odom(self, v, m):
        q = m.pose.pose.orientation
        self.pose.setdefault(v, {}).update(x=m.pose.pose.position.x, y=m.pose.pose.position.y,
                                           yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def lonlat(self, x, y):
        lat, lon = self.frame.to_latlon(x, y)
        return [round(lon, 7), round(lat, 7)]

    def publish(self):
        features = [{'type': 'Feature', 'properties': {'name': self.corridor_id, 'style': {'color': '#ff3b30', 'weight': 3, 'fill': False}},
                     'geometry': {'type': 'Polygon', 'coordinates': [[self.lonlat(*p) for p in self.poly] + [self.lonlat(*self.poly[0])]]}}]
        markers = MarkerArray()
        markers.markers.append(self.line('corridor', 0, self.poly + [self.poly[0]], (1.0, 0.23, 0.19), 0.6))
        for i, v in enumerate(self.vehicles):
            hexc, rgb = COLORS.get(v, ('#ffffff', (1.0, 1.0, 1.0)))
            route = self.routes[v]
            features.append({'type': 'Feature', 'properties': {'name': f'{v} route', 'style': {'color': hexc, 'weight': 2, 'dashArray': '6 6'}},
                             'geometry': {'type': 'LineString', 'coordinates': [self.lonlat(*p) for p in route] + [self.lonlat(*route[0])]}})
            markers.markers.append(self.line(f'{v}_route', 10 + i, route + [route[0]], rgb, 0.3))
            p = self.pose.get(v, {})
            if 'yaw' in p and 'lat' in p:
                length = self.get_parameter('heading_len_m').value
                tip = (p['x'] + length * math.cos(p['yaw']), p['y'] + length * math.sin(p['yaw']))
                features.append({'type': 'Feature', 'properties': {'name': f'{v} heading', 'style': {'color': hexc, 'weight': 4}},
                                 'geometry': {'type': 'LineString', 'coordinates': [[p['lon'], p['lat']], self.lonlat(*tip)]}})
                features.append({'type': 'Feature', 'properties': {'name': v.upper(), 'style': {'color': hexc, 'radius': 6, 'fillOpacity': 0.9}},
                                 'geometry': {'type': 'Point', 'coordinates': [p['lon'], p['lat']]}})
                arrow = Marker()
                arrow.header.frame_id = 'map'
                arrow.header.stamp = self.get_clock().now().to_msg()
                arrow.ns = f'{v}_heading'
                arrow.id = 20 + i
                arrow.type = Marker.ARROW
                arrow.points = [Point(x=p['x'], y=p['y'], z=0.5), Point(x=tip[0], y=tip[1], z=0.5)]
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.5, 1.2, 1.0
                arrow.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)
                markers.markers.append(arrow)
        msg = GeoJSON()
        msg.geojson = json.dumps({'type': 'FeatureCollection', 'features': features})
        self.geo_pub.publish(msg)
        self.marker_pub.publish(markers)

    def line(self, ns, mid, pts, rgb, width):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.scale.x = width
        m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)
        m.points = [Point(x=x, y=y, z=0.1) for x, y in pts]
        return m


def main():
    rclpy.init()
    node = MapOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
