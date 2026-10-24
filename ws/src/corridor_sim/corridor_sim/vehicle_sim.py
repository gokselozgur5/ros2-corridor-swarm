"""One simulated vehicle following a lat/lon route at constant speed.

Publishes sensor_msgs/NavSatFix, nav_msgs/Odometry (map frame, local ENU around
the first route point) and the map -> base_link transform. The route loops.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster

EARTH_RADIUS_M = 6371008.8


class VehicleSim(Node):
    def __init__(self):
        super().__init__('vehicle_sim')
        self.declare_parameter('name', 'vehicle')
        # Flat list: lat, lon, lat, lon, ... (a leg of the UK route from the map-writer demo).
        self.declare_parameter('route', [52.70, -3.00, 52.60, -2.60, 52.40, -2.00])
        self.declare_parameter('speed_mps', 20.0)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')

        self.name = self.get_parameter('name').value
        flat = list(self.get_parameter('route').value)
        if len(flat) < 4 or len(flat) % 2:
            raise ValueError('route needs at least two lat,lon pairs')
        self.route = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        self.speed = float(self.get_parameter('speed_mps').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.origin = self.route[0]
        self.cos_lat0 = math.cos(math.radians(self.origin[0]))
        self.points = [self.to_enu(p) for p in self.route]
        self.leg_lengths = [math.dist(a, b) for a, b in zip(self.points, self.points[1:])]
        self.total = sum(self.leg_lengths)
        self.distance = 0.0

        self.fix_pub = self.create_publisher(NavSatFix, f'/{self.name}/fix', 10)
        self.odom_pub = self.create_publisher(Odometry, f'/{self.name}/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.period = 1.0 / float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(self.period, self.step)
        self.get_logger().info(
            f'{self.name}: {len(self.route)} waypoints, {self.total / 1000:.1f} km loop at {self.speed} m/s')

    def to_enu(self, latlon):
        lat, lon = latlon
        x = math.radians(lon - self.origin[1]) * EARTH_RADIUS_M * self.cos_lat0
        y = math.radians(lat - self.origin[0]) * EARTH_RADIUS_M
        return (x, y)

    def to_latlon(self, x, y):
        lat = self.origin[0] + math.degrees(y / EARTH_RADIUS_M)
        lon = self.origin[1] + math.degrees(x / (EARTH_RADIUS_M * self.cos_lat0))
        return lat, lon

    def position(self, distance):
        """Point and heading at `distance` along the looping polyline."""
        d = distance % self.total if self.total > 0 else 0.0
        for (a, b), length in zip(zip(self.points, self.points[1:]), self.leg_lengths):
            if d <= length or length == 0:
                t = d / length if length else 0.0
                x = a[0] + (b[0] - a[0]) * t
                y = a[1] + (b[1] - a[1]) * t
                return x, y, math.atan2(b[1] - a[1], b[0] - a[0])
            d -= length
        a, b = self.points[-2], self.points[-1]
        return b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0])

    def step(self):
        self.distance += self.speed * self.period
        x, y, yaw = self.position(self.distance)
        lat, lon = self.to_latlon(x, y)
        now = self.get_clock().now().to_msg()

        fix = NavSatFix()
        fix.header.stamp = now
        fix.header.frame_id = self.child_frame_id
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = 100.0
        self.fix_pub.publish(fix)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        odom.twist.twist.linear.x = self.speed
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.child_frame_id
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = odom.pose.pose.orientation.z
        tf.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
