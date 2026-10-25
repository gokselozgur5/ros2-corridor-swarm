import math

from corridor_nav.geofence import LocalFrame
from corridor_nav.local_pose import pose_from_fix

FRAME = LocalFrame(52.7, -3.0)


def test_position_comes_from_the_fix_not_the_heading_source():
    lat, lon = FRAME.to_latlon(20.0, 70.0)
    x, y, yaw = pose_from_fix(FRAME, lat, lon, heading=0.0)
    assert math.isclose(x, 20.0, abs_tol=1e-6) and math.isclose(y, 70.0, abs_tol=1e-6)
    assert yaw == 0.0


def test_heading_offset_is_applied_and_wrapped():
    lat, lon = FRAME.to_latlon(0.0, 0.0)
    _, _, yaw = pose_from_fix(FRAME, lat, lon, heading=3.0, heading_offset=1.0)
    assert math.isclose(yaw, 4.0 - 2 * math.pi)
