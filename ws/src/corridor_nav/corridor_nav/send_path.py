"""Publish a nav_msgs/Path once: ros2 run corridor_nav send_path --ros-args -r __ns:=/v1 -p points:=[x1,y1,x2,y2,...]

Waits for a subscriber (the follower) before publishing, otherwise the message is sent into the
void during discovery and the follower never sees it."""
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class SendPath(Node):
    def __init__(self):
        super().__init__('send_path')
        self.declare_parameter('points', [0.0, 0.0, 10.0, 0.0])
        self.declare_parameter('wait_s', 5.0)
        self.pub = self.create_publisher(Path, 'path', 10)

    def send(self):
        flat = list(self.get_parameter('points').value)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for i in range(0, len(flat) - 1, 2):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(flat[i])
            ps.pose.position.y = float(flat[i + 1])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        deadline = time.time() + self.get_parameter('wait_s').value
        while self.pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pub.get_subscription_count() == 0:
            self.get_logger().error('no subscriber on path')
            return False
        self.pub.publish(path)
        rclpy.spin_once(self, timeout_sec=0.3)
        self.get_logger().info(f'sent {len(path.poses)} poses to {self.pub.get_subscription_count()} subscriber(s)')
        return True


def main():
    rclpy.init()
    node = SendPath()
    ok = node.send()
    node.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit(0 if ok else 1)
