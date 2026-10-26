"""corridor_nav.clicked_goal's Path construction against a fake publisher (planner-3). Lives here
because corridor_planner owns the goto contract; runs without ROS through conftest.py's stand-ins."""
import math

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

from corridor_nav.clicked_goal import ClickedGoal
from corridor_nav.geofence import LocalFrame
from ros_fakes import FakeClock, FakeLogger, FakePublisher

FRAME = LocalFrame(52.7, -3.0)


def node():
    n = ClickedGoal.__new__(ClickedGoal)
    n.frame = FRAME
    n.fix = None
    n.odom = None
    n.path_pub = FakePublisher()
    n.get_clock = lambda: FakeClock()
    n.get_logger = lambda: FakeLogger()
    return n


def click(x, y):
    pt = PointStamped()
    pt.header.frame_id = 'map'
    pt.point.x = x
    pt.point.y = y
    return pt


def test_clicked_path_starts_at_the_fix_not_at_odom():
    n = node()
    n.fix = FRAME.to_latlon(20.0, 20.0)
    n.odom = Odometry()   # Gazebo: odom (0, 0) at the spawn, map (20, 20)
    n.on_point(click(100.0, 20.0))
    path = n.path_pub.published[-1]
    assert len(path.poses) == 2
    start = path.poses[0].pose.position
    assert math.isclose(start.x, 20.0, abs_tol=1e-6) and math.isclose(start.y, 20.0, abs_tol=1e-6), (start.x, start.y)
    assert (path.poses[1].pose.position.x, path.poses[1].pose.position.y) == (100.0, 20.0)


def test_click_without_a_fix_publishes_nothing():
    n = node()
    n.on_point(click(100.0, 20.0))
    assert n.path_pub.published == []
