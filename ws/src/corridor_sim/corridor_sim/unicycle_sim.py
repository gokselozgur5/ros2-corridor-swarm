"""Kinematic unicycle driven by cmd_vel, for testing the autonomy layer without Gazebo.

Publishes odom, fix, imu (orientation only) and the map -> base_link tf like vehicle_sim, but the motion comes
from geometry_msgs/Twist on `cmd_vel` instead of a fixed route. Runs in a vehicle namespace.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster

EARTH_RADIUS_M = 6371008.8


class UnicycleSim(Node):
    def __init__(self):
        super().__init__('unicycle_sim')
        self.declare_parameter('origin_lat', 52.70)
        self.declare_parameter('origin_lon', -3.00)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')

        self.lat0 = self.get_parameter('origin_lat').value
        self.lon0 = self.get_parameter('origin_lon').value
        self.cos_lat0 = math.cos(math.radians(self.lat0))
        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.yaw = self.get_parameter('start_yaw').value
        self.v = 0.0
        self.w = 0.0
        self.last_cmd = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.dt = 1.0 / self.get_parameter('rate_hz').value
        self.create_timer(self.dt, self.step)

    def on_cmd(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        if self.last_cmd is None or (now - self.last_cmd).nanoseconds * 1e-9 > self.get_parameter('cmd_timeout_s').value:
            self.v = 0.0
            self.w = 0.0
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw = math.atan2(math.sin(self.yaw + self.w * self.dt), math.cos(self.yaw + self.w * self.dt))
        stamp = now.to_msg()
        frame = self.get_parameter('frame_id').value
        child = self.get_parameter('child_frame_id').value
        qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = frame
        odom.child_frame_id = child
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = child
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.latitude = self.lat0 + math.degrees(self.y / EARTH_RADIUS_M)
        fix.longitude = self.lon0 + math.degrees(self.x / (EARTH_RADIUS_M * self.cos_lat0))
        self.fix_pub.publish(fix)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = child
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.angular_velocity.z = self.w
        self.imu_pub.publish(imu)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = frame
        t.child_frame_id = child
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = UnicycleSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
