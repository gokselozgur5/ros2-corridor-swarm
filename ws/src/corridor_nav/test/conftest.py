"""Test setup for corridor_nav.

Without ROS 2 (the Mac), patrol_server and waypoint_follower cannot import rclpy or the message packages.
When the real modules are missing, the stubs below take their place so the nodes' decision logic runs here:
message classes with ROS-like defaults and a Node that records publishers, subscriptions, timers and logs.
Tests marked `ros` need the real stack (rclpy, corridor_msgs built with colcon) and are skipped here; the
orchestrator runs them on Thor.
"""
import sys
import time
import types

import pytest

try:
    import rclpy  # noqa: F401
    ROS = True
except ImportError:
    ROS = False


def pytest_configure(config):
    config.addinivalue_line('markers', 'ros: needs ROS 2 (rclpy, corridor_msgs); skipped without it, run on Thor')


def pytest_collection_modifyitems(config, items):
    if ROS:
        return
    skip = pytest.mark.skip(reason='needs ROS 2: run on Thor')
    for item in items:
        if 'ros' in item.keywords:
            item.add_marker(skip)


@pytest.fixture(scope='session')
def ros():
    """One rclpy context for the whole session (the Thor node tests)."""
    if not ROS:
        pytest.skip('needs ROS 2')
    import os
    if os.environ.get('ROS_DOMAIN_ID', '0') == '42':
        pytest.skip('the node tests publish geofence events and Paths for v1 on live topics: run with a private domain, e.g. ROS_DOMAIN_ID=77, never on the demo domain 42')
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


# ----- stubs, installed only when ROS is absent -------------------------------------------------

def message(name, **fields):
    """A message class with ROS-like defaults: nested messages constructed, numbers 0, strings empty."""
    def __init__(self, **kw):
        for f, factory in fields.items():
            setattr(self, f, kw.pop(f) if f in kw else factory())
        if kw:
            raise TypeError(f'{name}: unknown fields {list(kw)}')

    def __eq__(self, other):
        return type(other) is type(self) and vars(self) == vars(other)

    def __repr__(self):
        return f'{name}({", ".join(f"{k}={v!r}" for k, v in vars(self).items())})'

    return type(name, (), {'__init__': __init__, '__eq__': __eq__, '__repr__': __repr__, '__hash__': None})


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


class Patrol:
    Goal = message('Patrol_Goal', vehicle=str, corridor_id=str, laps=int, speed_mps=float, clockwise=bool)
    Result = message('Patrol_Result', laps_done=int, distance_m=float, result=str)
    Feedback = message('Patrol_Feedback', lap=int, lap_progress=float, distance_m=float, next_waypoint=Point)


def install_stubs():
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        mod.__stub__ = True
        sys.modules[name] = mod
        return mod

    def noop(*args, **kw):
        return None

    rclpy_mod = module('rclpy', init=noop, spin=noop, spin_once=noop, shutdown=noop, try_shutdown=noop, ok=lambda: True)
    rclpy_mod.node = module('rclpy.node', Node=Node)
    rclpy_mod.parameter = module('rclpy.parameter', Parameter=Parameter)
    rclpy_mod.action = module('rclpy.action', ActionServer=ActionServer, GoalResponse=GoalResponse, CancelResponse=CancelResponse)
    rclpy_mod.callback_groups = module('rclpy.callback_groups', ReentrantCallbackGroup=type('ReentrantCallbackGroup', (), {}))
    rclpy_mod.executors = module('rclpy.executors', MultiThreadedExecutor=type('MultiThreadedExecutor', (), {}))
    rclpy_mod.qos = module('rclpy.qos', QoSProfile=QoSProfile, DurabilityPolicy=DurabilityPolicy)
    module('geometry_msgs').msg = module('geometry_msgs.msg', Point=Point, Pose=Pose, PoseStamped=PoseStamped,
                                         Quaternion=Quaternion, Twist=Twist, Vector3=Vector3)
    module('nav_msgs').msg = module('nav_msgs.msg', Odometry=Odometry, Path=Path)
    module('std_msgs').msg = module('std_msgs.msg', Header=Header, String=String, Float32=Float32)
    module('sensor_msgs').msg = module('sensor_msgs.msg', Imu=Imu, NavSatFix=NavSatFix, NavSatStatus=NavSatStatus)
    module('builtin_interfaces').msg = module('builtin_interfaces.msg', Time=Time)
    corridor_msgs = module('corridor_msgs')
    corridor_msgs.msg = module('corridor_msgs.msg', GeofenceEvent=GeofenceEvent)
    corridor_msgs.action = module('corridor_msgs.action', Patrol=Patrol)


if not ROS:
    install_stubs()


@pytest.fixture
def node_params():
    """Parameters for a stub node constructed without overrides, read when the node declares them."""
    def set_params(**params):
        PARAMETER_OVERRIDES.clear()
        PARAMETER_OVERRIDES.update(params)
    yield set_params
    PARAMETER_OVERRIDES.clear()
