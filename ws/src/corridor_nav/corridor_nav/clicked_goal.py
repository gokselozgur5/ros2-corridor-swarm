"""Foxglove 3D panel 'publish point' -> goto for one vehicle.

/clicked_point (geometry_msgs/PointStamped, map frame) becomes a two-pose Path on <vehicle>/path:
the vehicle's fix in the corridor frame (`corridor_geojson`, the follower's frame; odom is not the
map, issue #19), then the clicked point. The vehicle is a parameter (default v1). No fix, no Path."""
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from corridor_nav.geofence import LocalFrame, load_geojson_polygon


class ClickedGoal(Node):
    def __init__(self):
        super().__init__('clicked_goal')
        self.declare_parameter('vehicle', 'v1')
        self.declare_parameter('point_topic', '/clicked_point')
        self.declare_parameter('corridor_geojson', '')
        geojson = self.get_parameter('corridor_geojson').value
        if not geojson:
            raise ValueError('corridor_geojson parameter is required')
        _, ring = load_geojson_polygon(geojson)
        self.frame = LocalFrame(*ring[0])
        v = self.get_parameter('vehicle').value
        self.fix = None
        self.create_subscription(NavSatFix, f'/{v}/fix', lambda m: setattr(self, 'fix', (m.latitude, m.longitude)), 10)
        self.create_subscription(PointStamped, self.get_parameter('point_topic').value, self.on_point, 10)
        self.path_pub = self.create_publisher(Path, f'/{v}/path', 10)
        self.get_logger().info(f'clicks on {self.get_parameter("point_topic").value} drive {v}')

    def on_point(self, pt):
        if self.fix is None:
            self.get_logger().warn('no fix yet, click ignored')
            return
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in (self.frame.to_xy(*self.fix), (pt.point.x, pt.point.y)):
            ps = PoseStamped(header=path.header)
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.get_logger().info(f'goto {pt.point.x:.1f}, {pt.point.y:.1f}')


def main():
    rclpy.init()
    node = ClickedGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
