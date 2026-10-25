"""Pure-pursuit follower: nav_msgs/Path in, geometry_msgs/Twist out.

Runs inside a vehicle namespace. Position comes from `fix` (NavSatFix) through the
corridor's LocalFrame, so the path and the vehicle share the map frame on every backend;
`imu` supplies the heading (wheel odometry yaw drifts as soon as a skid-steer slips, issue #19). Subscribes `path`, publishes `cmd_vel` and
`follower/status` (std_msgs/String: idle | following | arrived). An empty Path cancels.
The controller itself is corridor_nav.pure_pursuit, tested without ROS.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String

from corridor_nav.geofence import LocalFrame, load_geojson_polygon
from corridor_nav.local_pose import pose_from_fix
from corridor_nav.pure_pursuit import step


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.declare_parameter('speed_mps', 2.0)
        self.declare_parameter('lookahead_m', 4.0)
        self.declare_parameter('arrive_tolerance_m', 1.0)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('pose_timeout_s', 1.0)
        self.declare_parameter('corridor_geojson', '')
        self.declare_parameter('heading_offset_rad', 0.0)   # added to the imu yaw; 0 when the imu reports ENU (east = 0)

        _, ring = load_geojson_polygon(self.get_parameter('corridor_geojson').value)
        self.frame = LocalFrame(*ring[0])
        self.path = []
        self.fix = None
        self.fix_stamp = None
        self.heading = None
        self.status = 'idle'

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Latched: patrol_server and the planner may start after us and need the current status, not the next change.
        self.status_pub = self.create_publisher(String, 'follower/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.xte_pub = self.create_publisher(Float32, 'follower/cross_track', 10)
        self.create_subscription(NavSatFix, 'fix', self.on_fix, 20)
        self.create_subscription(Imu, 'imu', self.on_imu, 20)
        self.create_subscription(Path, 'path', self.on_path, 10)
        self.timer = self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)
        self.status_pub.publish(String(data=self.status))

    def on_fix(self, msg):
        self.fix = (msg.latitude, msg.longitude)
        self.fix_stamp = self.get_clock().now()

    def on_imu(self, msg):
        self.heading = yaw_from_quaternion(msg.orientation)

    def on_path(self, msg):
        self.path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.set_status('following' if len(self.path) >= 2 else 'idle')
        if len(self.path) < 2:
            self.cmd_pub.publish(Twist())

    def set_status(self, status):
        if status != self.status:
            self.status = status
            self.get_logger().info(f'status: {status}')
        self.status_pub.publish(String(data=status))

    def tick(self):
        if self.status != 'following':
            return
        if self.fix is None or self.heading is None:
            return
        age = (self.get_clock().now() - self.fix_stamp).nanoseconds * 1e-9
        if age > self.get_parameter('pose_timeout_s').value:
            self.cmd_pub.publish(Twist())
            self.get_logger().warn('fix stale, holding', throttle_duration_sec=5.0)
            return
        x, y, yaw = pose_from_fix(self.frame, *self.fix, self.heading, self.get_parameter('heading_offset_rad').value)
        cmd = step(x, y, yaw, self.path,
                   self.get_parameter('speed_mps').value,
                   self.get_parameter('lookahead_m').value,
                   self.get_parameter('arrive_tolerance_m').value)
        twist = Twist()
        twist.linear.x = cmd.linear
        twist.angular.z = cmd.angular
        self.cmd_pub.publish(twist)
        self.xte_pub.publish(Float32(data=float(cmd.cross_track)))
        if cmd.done:
            self.path = []
            self.set_status('arrived')


def main():
    rclpy.init()
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass   # context already shut down on SIGINT
        node.destroy_node()
        rclpy.try_shutdown()
