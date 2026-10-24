"""Pure-pursuit follower: nav_msgs/Path in, geometry_msgs/Twist out.

Runs inside a vehicle namespace. Subscribes `odom` and `path`, publishes `cmd_vel`
and `follower/status` (std_msgs/String: idle | following | arrived). An empty Path
cancels. The controller itself is corridor_nav.pure_pursuit, tested without ROS.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32, String

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
        self.declare_parameter('odom_timeout_s', 1.0)

        self.path = []
        self.pose = None
        self.pose_stamp = None
        self.status = 'idle'

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Latched: patrol_server and the planner may start after us and need the current status, not the next change.
        self.status_pub = self.create_publisher(String, 'follower/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.xte_pub = self.create_publisher(Float32, 'follower/cross_track', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(Path, 'path', self.on_path, 10)
        self.timer = self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)
        self.status_pub.publish(String(data=self.status))

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.pose_stamp = self.get_clock().now()

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
        if self.pose is None:
            return
        age = (self.get_clock().now() - self.pose_stamp).nanoseconds * 1e-9
        if age > self.get_parameter('odom_timeout_s').value:
            self.cmd_pub.publish(Twist())
            self.get_logger().warn('odom stale, holding', throttle_duration_sec=5.0)
            return
        x, y, yaw = self.pose
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
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
