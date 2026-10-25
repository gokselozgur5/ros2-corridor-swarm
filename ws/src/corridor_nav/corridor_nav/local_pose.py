"""Map-frame pose from a GNSS fix and a heading. No ROS here so it is unit-testable.

Position comes from the fix through the corridor LocalFrame (the same origin patrol_server
and geofence_monitor use), so every backend (kinematic, gz plugin, ros2_control) places the
vehicle where the corridor polygon says it is. Odometry only contributes the heading; its
position starts wherever the vehicle spawned and is not the map (issue #19)."""
import math


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose_from_fix(frame, lat, lon, heading, heading_offset=0.0):
    """(x, y, yaw) in the frame's ENU metres; heading is the odometry yaw, offset corrects
    a spawn that did not face east."""
    x, y = frame.to_xy(lat, lon)
    return x, y, wrap_angle(heading + heading_offset)
