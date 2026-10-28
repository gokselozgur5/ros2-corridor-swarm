"""mission_planner as a real node (planner-2, planner-3). NEEDS ROS: rclpy and a built corridor_msgs.

Run on Thor after `. ./env.sh`, on a private domain (the fake v1 publishes on live topics):
    ROS_DOMAIN_ID=77 REQUIRE_ROS=1 python3 -m pytest -q -rs ws/src/corridor_planner/test/test_mission_planner_node.py
Skipped where conftest.py had to stand in for rclpy, unless REQUIRE_ROS is set. A fake v1 publishes the
FleetState corridor_nav's fleet_state would (fix at map 20,20, speed 0; the planner reads no odom, ADR-0015)
and runs a Patrol server that models the old 0.5 s-poll cancel handling, the slowest the planner must tolerate; the merged corridor_nav server clears
an owned route inside its cancel callback, so the real pair is faster than this fake."""
import math
import pathlib
import threading
import time

import pytest
import rclpy

if getattr(rclpy, 'STUB', False):
    if __import__('os').environ.get('REQUIRE_ROS'):
        pytest.exit('REQUIRE_ROS is set but rclpy is a stand-in: source the ROS workspace first', returncode=3)
    pytest.skip('needs ROS (run on Thor)', allow_module_level=True)
if __import__('os').environ.get('ROS_DOMAIN_ID', '0') == '42':
    pytest.exit('ROS_DOMAIN_ID is the demo domain 42 and this test publishes a fake v1 on live topics: run with ROS_DOMAIN_ID=77', returncode=3)

from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from rclpy.action import ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402

from corridor_msgs.action import Patrol  # noqa: E402
from corridor_msgs.msg import FleetState, VehicleState  # noqa: E402
from corridor_nav.geofence import LocalFrame  # noqa: E402
from corridor_planner.mission_planner import MissionPlanner  # noqa: E402

GEOJSON = pathlib.Path(__file__).resolve().parents[2] / 'corridor_nav' / 'config' / 'corridor_demo.geojson'
FRAME = LocalFrame(52.7, -3.0)
POLL_S = 0.5


class FakeVehicle(Node):
    def __init__(self):
        super().__init__('fake_v1')
        cb = ReentrantCallbackGroup()
        self.paths = []          # (number of poses, x of the first pose) in arrival order on /v1/path
        self.goals = 0
        self.exited = None       # monotonic time the last cancelled goal's execute returned
        self.path_pub = self.create_publisher(Path, '/v1/path', 10)
        self.create_subscription(Path, '/v1/path', self.on_path, 10, callback_group=cb)
        self.fleet_pub = self.create_publisher(FleetState, '/fleet/state', 10)
        self.server = ActionServer(self, Patrol, '/v1/patrol', execute_callback=self.execute,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)
        self.create_timer(0.1, self.publish_pose, callback_group=cb)

    def publish_pose(self):
        lat, lon = FRAME.to_latlon(20.0, 20.0)
        st = VehicleState(name='v1', mode='idle', fix=NavSatFix(latitude=lat, longitude=lon), inside_corridor=True)
        st.pose.position.x, st.pose.position.y = 20.0, 20.0
        self.fleet_pub.publish(FleetState(corridor_id='demo', vehicles=[st]))

    def on_path(self, msg):
        self.paths.append((len(msg.poses), msg.poses[0].pose.position.x if msg.poses else None))

    def send_route(self, n):
        path = Path()
        path.header.frame_id = 'map'
        for x, y in ([(float(n), 0.0), (float(n), 100.0)] if n else []):
            ps = PoseStamped(header=path.header)
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def execute(self, handle):
        self.goals += 1
        self.send_route(self.goals)
        result = Patrol.Result()
        while rclpy.ok():
            time.sleep(POLL_S)
            if handle.is_cancel_requested:
                self.send_route(0)
                handle.canceled()
                result.result = 'cancelled'
                self.exited = time.monotonic()
                return result
        handle.abort()
        result.result = 'error'
        return result


@pytest.fixture
def ros():
    rclpy.init(args=['--ros-args', '-p', f'corridor_geojson:={GEOJSON}', '-p', 'dry_run:=true'])
    vehicle = FakeVehicle()
    planner = MissionPlanner()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(vehicle)
    executor.add_node(planner)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    deadline = time.monotonic() + 5.0
    while planner.vehicles['v1'].fix is None and time.monotonic() < deadline:
        time.sleep(0.05)
    assert planner.vehicles['v1'].fix is not None, 'no fleet state reached the planner'
    yield vehicle, planner
    if planner.vehicles['v1'].goal is not None:
        planner.t_stop('v1')
    executor.shutdown(timeout_sec=2.0)
    vehicle.destroy_node()
    planner.destroy_node()
    rclpy.shutdown()
    spinner.join(timeout=2.0)


# ----- planner-2 ------------------------------------------------------------------------------------

def test_stop_returns_after_the_cancelled_goal_has_exited(ros):
    vehicle, planner = ros
    assert planner.t_patrol('v1', 0, True)['accepted'] is True
    time.sleep(POLL_S)
    out = planner.t_stop('v1')
    assert out['cancelled_patrol'] is True
    assert vehicle.exited is not None and vehicle.exited <= time.monotonic(), 'stop returned while the goal was still running'


def test_restart_patrol_leaves_the_new_route_in_force(ros):
    vehicle, planner = ros
    assert planner.t_patrol('v1', 0, True)['accepted'] is True
    time.sleep(2 * POLL_S)
    n = len(vehicle.paths)
    assert planner.t_patrol('v1', 0, False)['accepted'] is True
    time.sleep(3 * POLL_S)   # any empty Path trailing from the old goal's poll has landed by now
    after = vehicle.paths[n:]
    assert (2, 2.0) in after, after
    assert after[-1] == (2, 2.0), f'the old goal erased the new route: {after}'


# ----- planner-3 ------------------------------------------------------------------------------------

def test_goto_path_starts_at_the_fix_not_at_odom(ros):
    vehicle, planner = ros
    lat, lon = FRAME.to_latlon(100.0, 20.0)
    out = planner.t_goto('v1', lat, lon)
    assert out['accepted'] is True and (out['x'], out['y']) == (100.0, 20.0)
    deadline = time.monotonic() + 2.0
    while (not vehicle.paths or vehicle.paths[-1][0] != 2) and time.monotonic() < deadline:
        time.sleep(0.05)
    poses, x0 = vehicle.paths[-1]
    assert poses == 2
    assert math.isclose(x0, 20.0, abs_tol=0.01), f'first vertex at x={x0}, odom would say 0.0'


def test_summary_xy_come_from_the_fix(ros):
    vehicle, planner = ros
    d = planner.t_get('v1')
    assert (d['x'], d['y']) == (20.0, 20.0), d
    assert d['speed_mps'] == 0.0 and d['inside_corridor'] is True and d['mode'] == 'idle'
