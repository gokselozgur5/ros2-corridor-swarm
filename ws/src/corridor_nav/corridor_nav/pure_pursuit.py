"""Pure pursuit on a 2D polyline. No ROS here so it is unit-testable."""
import math
from dataclasses import dataclass


@dataclass
class Command:
    linear: float
    angular: float
    target_index: int
    cross_track: float
    done: bool


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def closest_segment(x, y, path):
    """Index i of the segment path[i]->path[i+1] nearest to (x, y), and the signed cross-track error.

    Positive error means the vehicle is left of the path direction."""
    best = (float('inf'), 0, 0.0)
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2))
        px, py = ax + t * dx, ay + t * dy
        dist = math.hypot(x - px, y - py)
        if dist < best[0]:
            cross = dx * (y - ay) - dy * (x - ax)
            sign = 1.0 if cross > 0 else -1.0
            best = (dist, i, sign * dist)
    return best[1], best[2]


def lookahead_point(x, y, path, start_index, lookahead):
    """First point at `lookahead` distance along the path from the closest segment; the last point if none."""
    for i in range(start_index, len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        fx, fy = ax - x, ay - y
        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - lookahead * lookahead
        disc = b * b - 4 * a * c
        if a == 0 or disc < 0:
            continue
        t = (-b + math.sqrt(disc)) / (2 * a)
        if 0.0 <= t <= 1.0:
            return (ax + t * dx, ay + t * dy), i
    return path[-1], len(path) - 2


def step(x, y, yaw, path, speed, lookahead, arrive_tolerance, max_angular=1.5):
    """One control step. `path` is a list of (x, y) in the same frame as the pose."""
    if len(path) < 2:
        return Command(0.0, 0.0, 0, 0.0, True)
    seg, cross = closest_segment(x, y, path)
    gx, gy = path[-1]
    if math.hypot(gx - x, gy - y) <= arrive_tolerance and seg >= len(path) - 2:
        return Command(0.0, 0.0, len(path) - 1, cross, True)
    (tx, ty), idx = lookahead_point(x, y, path, seg, lookahead)
    heading_error = wrap(math.atan2(ty - y, tx - x) - yaw)
    # curvature = 2 sin(alpha) / L ; angular = v * curvature
    angular = speed * 2.0 * math.sin(heading_error) / lookahead
    angular = max(-max_angular, min(max_angular, angular))
    linear = speed * max(0.2, math.cos(heading_error))
    return Command(linear, angular, idx + 1, cross, False)
