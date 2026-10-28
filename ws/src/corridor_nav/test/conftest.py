"""Test setup for corridor_nav.

Without ROS 2 (the Mac), patrol_server and waypoint_follower cannot import rclpy or the message packages.
When the real modules are missing, the stubs below take their place so the nodes' decision logic runs here:
message classes with ROS-like defaults and a Node that records publishers, subscriptions, timers and logs.
Tests marked `ros` need the real stack (rclpy, corridor_msgs built with colcon) and are skipped here; the
orchestrator runs them on Thor.
"""
import os
import sys
import time
import types

import pytest

try:
    import rclpy  # noqa: F401
    ROS = hasattr(rclpy, '__file__')   # a stand-in installed by another package's conftest has no file
except ImportError:
    ROS = False


def pytest_configure(config):
    config.addinivalue_line('markers', 'ros: needs ROS 2 (rclpy, corridor_msgs); skipped without it, run on Thor')


def pytest_collection_modifyitems(config, items):
    if ROS or os.environ.get('REQUIRE_ROS'):   # with REQUIRE_ROS the ros fixture fails instead of skipping
        return
    skip = pytest.mark.skip(reason='needs ROS 2: run on Thor')
    for item in items:
        if 'ros' in item.keywords:
            item.add_marker(skip)


@pytest.fixture(scope='session')
def ros():
    """One rclpy context for the whole session (the Thor node tests)."""
    if not ROS:
        if os.environ.get('REQUIRE_ROS'):
            pytest.fail('REQUIRE_ROS is set but rclpy is absent: source the ROS workspace first')
        pytest.skip('needs ROS 2')
    if os.environ.get('ROS_DOMAIN_ID', '0') == '42':
        pytest.fail('ROS_DOMAIN_ID is the demo domain 42: the node tests publish geofence events and Paths for v1 on live topics, run them with ROS_DOMAIN_ID=77')
    import rclpy
    rclpy.init(args=[])   # not pytest's argv; the domain comes from ROS_DOMAIN_ID
    yield
    rclpy.shutdown()


def until(predicate, timeout=2.0, step=0.01):
    """Poll `predicate` until it is truthy; returns its last value (falsy on timeout)."""
    end = time.time() + timeout
    value = predicate()
    while not value and time.time() < end:
        time.sleep(step)
        value = predicate()
    return value


# ----- stubs, installed when ROS is absent; foxglove_msgs alone wherever that package is missing -----

def message(cls_name, **fields):
    """A message class with ROS-like defaults: nested messages constructed, numbers 0, strings empty."""
    def __init__(self, **kw):
        for f, factory in fields.items():
            setattr(self, f, kw.pop(f) if f in kw else factory())
        if kw:
            raise TypeError(f'{cls_name}: unknown fields {list(kw)}')

    def __eq__(self, other):
        return type(other) is type(self) and vars(self) == vars(other)

    def __repr__(self):
        return f'{cls_name}({", ".join(f"{k}={v!r}" for k, v in vars(self).items())})'

    return type(cls_name, (), {'__init__': __init__, '__eq__': __eq__, '__repr__': __repr__, '__hash__': None})


class FakeTime:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        return FakeTime(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return Time(sec=self.nanoseconds // 1_000_000_000, nanosec=self.nanoseconds % 1_000_000_000)


class FakeClock:
    def __init__(self):
        self.ns = 1_000_000_000   # 1 s in, so stamps are never 0

    def now(self):
        return FakeTime(self.ns)

    def advance(self, seconds):
        self.ns += int(seconds * 1e9)


class FakeLogger:
    def __init__(self):
        self.lines = []   # (level, text)

    def _log(self, level, text):
        self.lines.append((level, text))

    def debug(self, text, **kw):
        self._log('debug', text)

    def info(self, text, **kw):
        self._log('info', text)

    def warn(self, text, **kw):
        self._log('warn', text)

    warning = warn

    def error(self, text, **kw):
        self._log('error', text)


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)

    def get_subscription_count(self):
        return 1


class FakeParameter:
    def __init__(self, value):
        self.value = value


class Parameter:
    def __init__(self, name, type_=None, value=None):
        self.name = name
        self.value = value


PARAMETER_OVERRIDES = {}   # name -> value for nodes built without overrides; see the node_params fixture


class Node:
    def __init__(self, name, namespace=None, parameter_overrides=None, **kw):
        self.name = name
        self._params = dict(PARAMETER_OVERRIDES)
        for p in parameter_overrides or []:
            self._params[p.name] = p.value
        self.publishers = {}      # topic -> FakePublisher
        self.subscriptions = {}   # topic -> callback
        self.timers = []          # (period_s, callback)
        self.clock = FakeClock()
        self.logger = FakeLogger()

    def declare_parameter(self, name, value=None):
        self._params.setdefault(name, value)

    def get_parameter(self, name):
        return FakeParameter(self._params[name])

    def create_publisher(self, msg_type, topic, qos, **kw):
        self.publishers[topic] = FakePublisher(topic)
        return self.publishers[topic]

    def create_subscription(self, msg_type, topic, callback, qos, **kw):
        self.subscriptions[topic] = callback

    def create_timer(self, period, callback, **kw):
        self.timers.append((period, callback))

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return self.logger

    def destroy_node(self):
        pass


class ActionServer:
    def __init__(self, node, action_type, name, execute_callback=None, goal_callback=None, cancel_callback=None,
                 handle_accepted_callback=None, **kw):
        self.node = node
        self.name = name
        self.execute_callback = execute_callback
        self.goal_callback = goal_callback
        self.cancel_callback = cancel_callback
        self.handle_accepted_callback = handle_accepted_callback


class GoalResponse:
    REJECT = 1
    ACCEPT = 2


class CancelResponse:
    REJECT = 1
    ACCEPT = 2


class DurabilityPolicy:
    TRANSIENT_LOCAL = 1
    VOLATILE = 2


class QoSProfile:
    def __init__(self, **kw):
        self.__dict__.update(kw)


Time = message('Time', sec=int, nanosec=int)
Header = message('Header', stamp=Time, frame_id=str)
Point = message('Point', x=float, y=float, z=float)
Quaternion = message('Quaternion', x=float, y=float, z=float, w=lambda: 1.0)
Vector3 = message('Vector3', x=float, y=float, z=float)
Pose = message('Pose', position=Point, orientation=Quaternion)
PoseStamped = message('PoseStamped', header=Header, pose=Pose)
Twist = message('Twist', linear=Vector3, angular=Vector3)
Path = message('Path', header=Header, poses=list)
Odometry = message('Odometry', header=Header, child_frame_id=str,
                   pose=message('PoseWithCovariance', pose=Pose), twist=message('TwistWithCovariance', twist=Twist))
String = message('String', data=str)
Float32 = message('Float32', data=float)
NavSatStatus = message('NavSatStatus', status=int, service=int)
NavSatFix = message('NavSatFix', header=Header, status=NavSatStatus, latitude=float, longitude=float, altitude=float)
Imu = message('Imu', header=Header, orientation=Quaternion, angular_velocity=Vector3, linear_acceleration=Vector3)
GeofenceEvent = message('GeofenceEvent', header=Header, vehicle=str, corridor_id=str, type=int, distance_m=float, fix=NavSatFix)
GeofenceEvent.EXIT, GeofenceEvent.ENTER, GeofenceEvent.WARN = 0, 1, 2
VehicleState = message('VehicleState', header=Header, name=str, fix=NavSatFix, pose=Pose, velocity=Twist,
                       battery_pct=float, mode=str, goal_id=str, inside_corridor=bool)
FleetState = message('FleetState', header=Header, corridor_id=str, vehicles=list)
UUID = message('UUID', uuid=lambda: bytes(16))
GoalInfo = message('GoalInfo', goal_id=UUID, stamp=Time)
GoalStatus = message('GoalStatus', goal_info=GoalInfo, status=int)
(GoalStatus.STATUS_UNKNOWN, GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING,
 GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED) = range(7)
GoalStatusArray = message('GoalStatusArray', status_list=list)
ColorRGBA = message('ColorRGBA', r=float, g=float, b=float, a=float)
Marker = message('Marker', header=Header, ns=str, id=int, type=int, points=list, scale=Vector3, color=ColorRGBA)
Marker.ARROW, Marker.LINE_STRIP = 0, 4
MarkerArray = message('MarkerArray', markers=list)
GeoJSON = message('GeoJSON', geojson=str)


class Patrol:
    Goal = message('Patrol_Goal', vehicle=str, corridor_id=str, laps=int, speed_mps=float, clockwise=bool)
    Result = message('Patrol_Result', laps_done=int, distance_m=float, result=str)
    Feedback = message('Patrol_Feedback', lap=int, lap_progress=float, distance_m=float, next_waypoint=Point)


def module(name, **attrs):
    mod = types.ModuleType(name)
    mod.__dict__.update(attrs)
    mod.__stub__ = True
    sys.modules[name] = mod
    return mod


def install_stubs():
    def noop(*args, **kw):
        return None

    rclpy_mod = module('rclpy', init=noop, spin=noop, spin_once=noop, shutdown=noop, try_shutdown=noop, ok=lambda: True)
    rclpy_mod.node = module('rclpy.node', Node=Node)
    rclpy_mod.parameter = module('rclpy.parameter', Parameter=Parameter)
    rclpy_mod.action = module('rclpy.action', ActionServer=ActionServer, GoalResponse=GoalResponse, CancelResponse=CancelResponse)
    rclpy_mod.callback_groups = module('rclpy.callback_groups', ReentrantCallbackGroup=type('ReentrantCallbackGroup', (), {}),
                                       MutuallyExclusiveCallbackGroup=type('MutuallyExclusiveCallbackGroup', (), {}))
    rclpy_mod.executors = module('rclpy.executors', MultiThreadedExecutor=type('MultiThreadedExecutor', (), {}))
    rclpy_mod.qos = module('rclpy.qos', QoSProfile=QoSProfile, DurabilityPolicy=DurabilityPolicy)
    module('geometry_msgs').msg = module('geometry_msgs.msg', Point=Point, Pose=Pose, PoseStamped=PoseStamped,
                                         Quaternion=Quaternion, Twist=Twist, Vector3=Vector3)
    module('nav_msgs').msg = module('nav_msgs.msg', Odometry=Odometry, Path=Path)
    module('std_msgs').msg = module('std_msgs.msg', Header=Header, String=String, Float32=Float32, ColorRGBA=ColorRGBA)
    module('visualization_msgs').msg = module('visualization_msgs.msg', Marker=Marker, MarkerArray=MarkerArray)
    module('foxglove_msgs').msg = module('foxglove_msgs.msg', GeoJSON=GeoJSON)
    module('sensor_msgs').msg = module('sensor_msgs.msg', Imu=Imu, NavSatFix=NavSatFix, NavSatStatus=NavSatStatus)
    module('builtin_interfaces').msg = module('builtin_interfaces.msg', Time=Time)
    corridor_msgs = module('corridor_msgs')
    corridor_msgs.msg = module('corridor_msgs.msg', GeofenceEvent=GeofenceEvent, FleetState=FleetState, VehicleState=VehicleState)
    module('action_msgs').msg = module('action_msgs.msg', GoalStatus=GoalStatus, GoalStatusArray=GoalStatusArray, GoalInfo=GoalInfo)
    module('unique_identifier_msgs').msg = module('unique_identifier_msgs.msg', UUID=UUID)
    corridor_msgs.action = module('corridor_msgs.action', Patrol=Patrol)


if not ROS:
    install_stubs()
else:
    try:
        import foxglove_msgs.msg  # noqa: F401
    except ImportError:
        # Thor has rclpy but not ros-jazzy-foxglove-msgs; map_overlay imports it at module level and the
        # launch_testing plugin turns one module's import error into an error for the whole directory.
        # GeoJSON is an envelope with one string field; the stand-in keeps map_overlay's tests running.
        module('foxglove_msgs').msg = module('foxglove_msgs.msg', GeoJSON=GeoJSON)


@pytest.fixture
def node_params():
    """Parameters for a stub node constructed without overrides, read when the node declares them."""
    def set_params(**params):
        PARAMETER_OVERRIDES.clear()
        PARAMETER_OVERRIDES.update(params)
    yield set_params
    PARAMETER_OVERRIDES.clear()
