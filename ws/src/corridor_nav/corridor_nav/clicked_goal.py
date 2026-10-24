"""Foxglove 3D panel 'publish point' -> goto for one vehicle.

/clicked_point (geometry_msgs/PointStamped, map frame) becomes a two-pose Path on <vehicle>/path:
current odom pose, then the clicked point. The vehicle is a parameter (default v1)."""
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class ClickedGoal(Node):
    def __init__(self):
        super().__init__('clicked_goal')
        self.declare_parameter('vehicle', 'v1')
        self.declare_parameter('point_topic', '/clicked_point')
        v = self.get_parameter('vehicle').value
        self.odom = None
        self.create_subscription(Odometry, f'/{v}/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(PointStamped, self.get_parameter('point_topic').value, self.on_point, 10)
        self.path_pub = self.create_publisher(Path, f'/{v}/path', 10)
        self.get_logger().info(f'clicks on {self.get_parameter("point_topic").value} drive {v}')

    def on_point(self, pt):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        if self.odom is not None:
            start = PoseStamped(header=path.header)
            start.pose = self.odom.pose.pose
            path.poses.append(start)
        goal = PoseStamped(header=path.header)
        goal.pose.position.x = pt.point.x
        goal.pose.position.y = pt.point.y
        goal.pose.orientation.w = 1.0
        path.poses.append(goal)
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
