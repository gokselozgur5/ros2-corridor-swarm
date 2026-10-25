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
    # Open path, the follower has already progressed to the last segment.
    cmd = step(0.1, 10.1, 0.0, SQUARE[:4], speed=1.0, lookahead=2.0, arrive_tolerance=0.5, min_segment=2)
    assert cmd.done and cmd.linear == 0.0 and cmd.angular == 0.0


def test_start_is_not_arrival():
    # Start and end of the square coincide; at the start we must not report done.
    cmd = step(0.0, 0.0, 0.0, SQUARE[:3], speed=1.0, lookahead=2.0, arrive_tolerance=0.5)
    assert not cmd.done


def test_closed_route_start_is_the_first_segment_not_the_last():
    # Closed square; the vehicle stands just outside the first vertex, nearer to the closing
    # segment D->A than to A->B. It has not driven the lap, so it must not be "arrived".
    square = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0), (0.0, 0.0)]
    cmd = step(-1.0, 2.0, 0.0, square, 2.0, 4.0, 3.0)
    assert not cmd.done
    assert cmd.segment == 0


def test_progress_does_not_go_backwards():
    square = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0), (0.0, 0.0)]
    # On the last leg, 2 m from the end, having reported segment 3: now it is a lap.
    cmd = step(1.0, 1.5, -math.pi / 2, square, 2.0, 4.0, 3.0, min_segment=3)
    assert cmd.done
