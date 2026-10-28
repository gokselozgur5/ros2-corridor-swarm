"""Lets the node modules import without ROS.

On Thor (rclpy present) this does nothing and the tests run against the real message classes. On a
machine without ROS it installs minimal stand-ins for rclpy and the message packages that
mission_planner and clicked_goal import, so their tool methods can be exercised with fake handles
and publishers (see ros_fakes.py). Node-level tests check `rclpy.STUB` and skip themselves here."""
import sys
import types

try:
    import rclpy  # noqa: F401
    _REAL_ROS = hasattr(rclpy, '__file__')   # a stand-in installed by another package's conftest has no file
except ImportError:
    _REAL_ROS = False

if not _REAL_ROS:
    class _Msg:
        FIELDS = {}

        def __init__(self, **kw):
            for name, default in self.FIELDS.items():
                setattr(self, name, default() if callable(default) else default)
            for name, value in kw.items():
                if name not in self.FIELDS:
                    raise TypeError(f'{type(self).__name__} has no field {name}')
                setattr(self, name, value)

    def _msg(cls_name, **fields):
        return type(cls_name, (_Msg,), {'FIELDS': fields})

    Time = _msg('Time', sec=0, nanosec=0)
    builtin = types.ModuleType('builtin_interfaces.msg')
    builtin.Time = Time
    Header = _msg('Header', stamp=Time, frame_id='')
    Point = _msg('Point', x=0.0, y=0.0, z=0.0)
    Quaternion = _msg('Quaternion', x=0.0, y=0.0, z=0.0, w=1.0)
    Pose = _msg('Pose', position=Point, orientation=Quaternion)
    Vector3 = _msg('Vector3', x=0.0, y=0.0, z=0.0)
    Twist = _msg('Twist', linear=Vector3, angular=Vector3)
    PoseWithCovariance = _msg('PoseWithCovariance', pose=Pose)
    TwistWithCovariance = _msg('TwistWithCovariance', twist=Twist)

    geometry = types.ModuleType('geometry_msgs.msg')
    geometry.Point, geometry.Pose, geometry.Quaternion, geometry.Twist, geometry.Vector3 = Point, Pose, Quaternion, Twist, Vector3
    geometry.PoseStamped = _msg('PoseStamped', header=Header, pose=Pose)
    geometry.PointStamped = _msg('PointStamped', header=Header, point=Point)
    nav = types.ModuleType('nav_msgs.msg')
    nav.Path = _msg('Path', header=Header, poses=list)
    nav.Odometry = _msg('Odometry', header=Header, child_frame_id='', pose=PoseWithCovariance, twist=TwistWithCovariance)
    sensor = types.ModuleType('sensor_msgs.msg')
    sensor.NavSatFix = _msg('NavSatFix', header=Header, latitude=0.0, longitude=0.0, altitude=0.0)
    sensor.Image = _msg('Image', header=Header, height=0, width=0, encoding='', step=0, data=bytes)
    sensor.Imu = _msg('Imu', header=Header, orientation=Quaternion)
    std = types.ModuleType('std_msgs.msg')
    std.String = _msg('String', data='')
    std.Float32 = _msg('Float32', data=0.0)
    std.Header = Header
    diag = types.ModuleType('diagnostic_msgs.msg')
    diag.KeyValue = _msg('KeyValue', key='', value='')
    diag.DiagnosticStatus = _msg('DiagnosticStatus', level=b'\x00', name='', message='', hardware_id='', values=list)
    diag.DiagnosticStatus.OK, diag.DiagnosticStatus.WARN, diag.DiagnosticStatus.ERROR = b'\x00', b'\x01', b'\x02'
    diag.DiagnosticArray = _msg('DiagnosticArray', header=Header, status=list)
    corridor_msg = types.ModuleType('corridor_msgs.msg')
    corridor_msg.GeofenceEvent = _msg('GeofenceEvent', header=Header, vehicle='', corridor_id='', type=0, distance_m=0.0, fix=sensor.NavSatFix)
    corridor_msg.GeofenceEvent.EXIT, corridor_msg.GeofenceEvent.ENTER, corridor_msg.GeofenceEvent.WARN = 0, 1, 2
    corridor_action = types.ModuleType('corridor_msgs.action')
    corridor_action.Patrol = type('Patrol', (), {
        'Goal': _msg('Patrol_Goal', vehicle='', corridor_id='', laps=0, speed_mps=0.0, clockwise=False),
        'Result': _msg('Patrol_Result', laps_done=0, distance_m=0.0, result=''),
        'Feedback': _msg('Patrol_Feedback', lap=0, lap_progress=0.0, distance_m=0.0, next_waypoint=Point)})

    rclpy = types.ModuleType('rclpy')
    rclpy.STUB = True
    rclpy.init = lambda *a, **k: None
    rclpy.try_shutdown = lambda: None
    rclpy.node = types.ModuleType('rclpy.node')
    rclpy.node.Node = type('Node', (), {})
    rclpy.action = types.ModuleType('rclpy.action')
    rclpy.action.ActionClient = type('ActionClient', (), {})
    rclpy.callback_groups = types.ModuleType('rclpy.callback_groups')
    rclpy.callback_groups.ReentrantCallbackGroup = type('ReentrantCallbackGroup', (), {})
    rclpy.executors = types.ModuleType('rclpy.executors')
    rclpy.executors.MultiThreadedExecutor = type('MultiThreadedExecutor', (), {})
    rclpy.qos = types.ModuleType('rclpy.qos')
    rclpy.qos.DurabilityPolicy = type('DurabilityPolicy', (), {'TRANSIENT_LOCAL': 1, 'VOLATILE': 2})
    rclpy.qos.QoSProfile = type('QoSProfile', (), {'__init__': lambda self, **kw: self.__dict__.update(kw)})

    for pkg, msg in (('builtin_interfaces', builtin), ('geometry_msgs', geometry), ('nav_msgs', nav),
                     ('sensor_msgs', sensor), ('std_msgs', std), ('diagnostic_msgs', diag)):
        sys.modules[pkg] = types.ModuleType(pkg)
        sys.modules[pkg].msg = msg
        sys.modules[f'{pkg}.msg'] = msg
    sys.modules['corridor_msgs'] = types.ModuleType('corridor_msgs')
    sys.modules['corridor_msgs'].msg, sys.modules['corridor_msgs'].action = corridor_msg, corridor_action
    sys.modules['corridor_msgs.msg'], sys.modules['corridor_msgs.action'] = corridor_msg, corridor_action
    for name in ('rclpy', 'rclpy.node', 'rclpy.action', 'rclpy.callback_groups', 'rclpy.executors', 'rclpy.qos'):
        sys.modules[name] = rclpy if name == 'rclpy' else getattr(rclpy, name.split('.')[1])
