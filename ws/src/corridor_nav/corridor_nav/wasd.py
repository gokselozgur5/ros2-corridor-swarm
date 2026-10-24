"""Keyboard teleop: ros2 run corridor_nav wasd --ros-args -r __ns:=/v1

w/s  forward / reverse (steps of step_mps, held by repeating the key)
a/d  turn left / right
space  stop, q quit. Any key not pressed for hold_s seconds -> stop.
On start it publishes an empty Path so the follower goes idle and does not fight the keys.
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node


class Wasd(Node):
    def __init__(self):
        super().__init__('wasd')
        self.declare_parameter('step_mps', 0.5)
        self.declare_parameter('max_mps', 4.0)
        self.declare_parameter('turn_rps', 1.0)
        self.declare_parameter('hold_s', 0.6)
        self.declare_parameter('rate_hz', 20.0)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'path', 10)
        self.v = 0.0
        self.w = 0.0
        self.last_key = self.get_clock().now()
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)
        self.create_timer(0.5, self.silence_follower)
        self.silenced = 0

    def silence_follower(self):
        if self.silenced < 4:
            self.path_pub.publish(Path())
            self.silenced += 1

    def key(self, k):
        step = self.get_parameter('step_mps').value
        vmax = self.get_parameter('max_mps').value
        turn = self.get_parameter('turn_rps').value
        if k == 'w':
            self.v = min(vmax, self.v + step)
        elif k == 's':
            self.v = max(-vmax / 2, self.v - step)
        elif k == 'a':
            self.w = turn
        elif k == 'd':
            self.w = -turn
        elif k == ' ':
            self.v = 0.0
            self.w = 0.0
        self.last_key = self.get_clock().now()

    def tick(self):
        age = (self.get_clock().now() - self.last_key).nanoseconds * 1e-9
        if age > self.get_parameter('hold_s').value:
            self.w = 0.0
            if age > 3 * self.get_parameter('hold_s').value:
                self.v = 0.0
        t = Twist()
        t.linear.x = self.v
        t.angular.z = self.w
        self.cmd_pub.publish(t)
        sys.stdout.write(f'\r v={self.v:+.1f} m/s  w={self.w:+.1f} rad/s   (w/s a/d space q)   ')
        sys.stdout.flush()


def main():
    rclpy.init()
    node = Wasd()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd) if sys.stdin.isatty() else None
    if old:
        tty.setcbreak(fd)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1)
                if k == 'q' or k == '':
                    break
                node.key(k)
    except KeyboardInterrupt:
        pass
    finally:
        if old:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.cmd_pub.publish(Twist())
        print()
        node.destroy_node()
        rclpy.try_shutdown()
