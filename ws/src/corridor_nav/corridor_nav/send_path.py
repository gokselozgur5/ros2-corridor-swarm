"""Publish a nav_msgs/Path once: ros2 run corridor_nav send_path --ros-args -r __ns:=/v1 -p points:=[x1,y1,x2,y2,...]"""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy


class SendPath(Node):
    def __init__(self):
        super().__init__('send_path')
        self.declare_parameter('points', [0.0, 0.0, 10.0, 0.0])
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
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Path, 'path', qos)
        self.pub.publish(path)
        self.get_logger().info(f'sent {len(path.poses)} poses')


def main():
    rclpy.init()
    node = SendPath()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.try_shutdown()
