import math

from corridor_nav.pure_pursuit import closest_segment, step

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)]


def test_target_left_turns_left():
    # On the first leg heading +x, the path turns +y at (10,0): angular must be positive.
    cmd = step(9.0, 0.0, 0.0, SQUARE, speed=1.0, lookahead=2.0, arrive_tolerance=0.5)
    assert cmd.angular > 0.1
    assert not cmd.done


def test_target_right_turns_right():
    # Mirror: path turning to -y; angular must be negative.
    path = [(0.0, 0.0), (10.0, 0.0), (10.0, -10.0)]
    cmd = step(9.0, 0.0, 0.0, path, speed=1.0, lookahead=2.0, arrive_tolerance=0.5)
    assert cmd.angular < -0.1


def test_cross_track_sign():
    # Vehicle left of a +x leg -> positive, right -> negative.
    _, left = closest_segment(5.0, 1.0, SQUARE[:2])
    _, right = closest_segment(5.0, -1.0, SQUARE[:2])
    assert left > 0 and right < 0
    assert math.isclose(abs(left), 1.0)


def test_arrival_stops():
    # Open path: arrival is the far end, not the shared start/end of the closed square.
    cmd = step(0.1, 10.1, 0.0, SQUARE[:4], speed=1.0, lookahead=2.0, arrive_tolerance=0.5)
    assert cmd.done and cmd.linear == 0.0 and cmd.angular == 0.0


def test_start_is_not_arrival():
    # Start and end of the square coincide; at the start we must not report done.
    cmd = step(0.0, 0.0, 0.0, SQUARE[:3], speed=1.0, lookahead=2.0, arrive_tolerance=0.5)
    assert not cmd.done
