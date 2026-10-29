"""One fleet view: corridor_msgs/FleetState on /fleet/state every `period_s` (ADR-0015).

Per vehicle (parameter `vehicles`) it reads /<v>/fix, /<v>/imu, /<v>/odom, /<v>/follower/status and the
patrol action's status topic, plus /fleet/geofence_events, and fills one VehicleState: position from the
fix through the corridor LocalFrame (never odom x/y, #19), heading from the IMU, speed from odom,
inside_corridor from the polygon, mode and goal_id derived from what the vehicle layer reports. Nothing
here is told by the planner; a vehicle driven by an operator tool shows the same as one driven by Claude.
A vehicle whose fix is older than `fix_timeout_s` is still listed, with `mode` 'unknown' and no fix.
"""
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String

from corridor_msgs.msg import FleetState, GeofenceEvent, VehicleState
from corridor_nav.geofence import LocalFrame, load_geojson_polygon, signed_distance

ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)


def active_goal_id(status_array):
    """Hex id of the goal the action server is running, '' when none is accepted, executing or canceling."""
    for st in status_array.status_list:
        if st.status in ACTIVE:
            return bytes(st.goal_info.goal_id.uuid).hex()
    return ''


def mode_of(patrol_goal_id, follower_status, breached):
    """idle | patrol | goto | geofence_hold | unknown from what the vehicle layer reports.

    A geofence EXIT without an ENTER since holds the vehicle whatever else is going on; a running patrol
    goal is 'patrol'; a follower that follows a route nobody's patrol owns is a goto (planner, clicked
    goal, send_path alike); otherwise idle. 'unknown' only when the follower never reported."""
    if breached:
        return 'geofence_hold'
    if patrol_goal_id:
        return 'patrol'
    if follower_status == 'following':
        return 'goto'
    if follower_status in ('idle', 'arrived'):
        return 'idle'
    return 'unknown'


class Vehicle:
    def __init__(self):
        self.fix = None
        self.fix_ns = None
        self.imu = None
        self.odom = None
        self.follower = ''
        self.goal_id = ''
        self.breached = False


class FleetStateNode(Node):
    def __init__(self):
        super().__init__('fleet_state')
        self.declare_parameter('corridor_geojson', '')
        self.declare_parameter('vehicles', ['v1'])
        self.declare_parameter('period_s', 0.5)
        self.declare_parameter('fix_timeout_s', 5.0)

        path = self.get_parameter('corridor_geojson').value
        if not path:
            raise ValueError('corridor_geojson parameter is required')
        self.corridor_id, ring = load_geojson_polygon(path)
        self.frame = LocalFrame(*ring[0])
        self.poly = [self.frame.to_xy(lat, lon) for lat, lon in ring]
        self.fix_timeout_s = self.get_parameter('fix_timeout_s').value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.vehicles = {}
        for v in self.get_parameter('vehicles').value:
            self.vehicles[v] = Vehicle()
            self.create_subscription(NavSatFix, f'/{v}/fix', lambda m, v=v: self.on_fix(v, m), 20)
            self.create_subscription(Imu, f'/{v}/imu', lambda m, v=v: setattr(self.vehicles[v], 'imu', m), 20)
            self.create_subscription(Odometry, f'/{v}/odom', lambda m, v=v: setattr(self.vehicles[v], 'odom', m), 20)
            self.create_subscription(String, f'/{v}/follower/status', lambda m, v=v: setattr(self.vehicles[v], 'follower', m.data), latched)
            self.create_subscription(GoalStatusArray, f'/{v}/patrol/_action/status',
                                     lambda m, v=v: setattr(self.vehicles[v], 'goal_id', active_goal_id(m)), latched)
        self.create_subscription(GeofenceEvent, '/fleet/geofence_events', self.on_geofence, 20)
        self.pub = self.create_publisher(FleetState, '/fleet/state', 10)
        self.create_timer(self.get_parameter('period_s').value, self.tick)
        self.get_logger().info(f'fleet_state: {list(self.vehicles)} on corridor {self.corridor_id}')

    def on_fix(self, vehicle, msg):
        v = self.vehicles[vehicle]
        v.fix = msg
        v.fix_ns = self.get_clock().now().nanoseconds

    def on_geofence(self, ev):
        if ev.vehicle not in self.vehicles:
            return
        if ev.type == GeofenceEvent.EXIT:
            self.vehicles[ev.vehicle].breached = True
        elif ev.type == GeofenceEvent.ENTER:
            self.vehicles[ev.vehicle].breached = False

    def fix_is_fresh(self, v, now_ns):
        return v.fix is not None and (now_ns - v.fix_ns) * 1e-9 <= self.fix_timeout_s

    def vehicle_state(self, name, now_ns):
        v = self.vehicles[name]
        st = VehicleState()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'map'
        st.name = name
        st.goal_id = v.goal_id
        if not self.fix_is_fresh(v, now_ns):
            st.mode = 'unknown'
            return st
        # The vehicle's own clock: a Gazebo vehicle stamps on sim time while this node runs on wall time, and
        # consumers age that vehicle's other data (the planner's camera frame) against this stamp (#37).
        st.header.stamp = v.fix.header.stamp
        st.fix = v.fix
        x, y = self.frame.to_xy(v.fix.latitude, v.fix.longitude)
        st.pose.position.x, st.pose.position.y = x, y
        if v.imu is not None:
            st.pose.orientation = v.imu.orientation
        if v.odom is not None:
            st.velocity = v.odom.twist.twist
        st.inside_corridor = signed_distance(x, y, self.poly) >= 0.0
        st.mode = mode_of(v.goal_id, v.follower, v.breached)
        return st

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        msg = FleetState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.corridor_id = self.corridor_id
        msg.vehicles = [self.vehicle_state(name, now_ns) for name in self.vehicles]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FleetStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
