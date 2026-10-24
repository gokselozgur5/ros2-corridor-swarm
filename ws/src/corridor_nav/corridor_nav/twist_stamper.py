"""geometry_msgs/Twist on `cmd_vel` -> geometry_msgs/TwistStamped on `diff_drive/cmd_vel`.

The jazzy diff_drive_controller takes only TwistStamped; the follower and the planner-side tools
keep publishing plain Twist so the kinematic and Gazebo backends share one contract."""
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('out_topic', 'diff_drive/cmd_vel')
        self.pub = self.create_publisher(TwistStamped, self.get_parameter('out_topic').value, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)

    def on_twist(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.get_parameter('frame_id').value
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
