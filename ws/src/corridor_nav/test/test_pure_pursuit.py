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


def test_overshot_corner_aims_at_the_current_segment_end():
    # Past corner B by 34 m heading east, on segment B->C (min_segment 1), farther than the lookahead
    # from the path. The target must be C, up and behind-left, not the route's end at A straight behind.
    square = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0), (0.0, 0.0)]
    cmd = step(134.0, 0.0, 0.0, square, 3.0, 4.0, 2.0, min_segment=1)
    assert cmd.target_index == 2
    assert cmd.angular > 1.0


def test_target_behind_turns_at_full_rate():
    # Target 170 deg to the left: sin(170 deg) is small but the turn must not be.
    cmd = step(0.0, 0.0, 0.0, [(-10.0, 1.76), (-20.0, 3.5)], 1.0, 2.0, 0.5, max_angular=1.5)
    assert cmd.angular == 1.5


def test_before_the_first_vertex_aims_at_the_route_start_not_the_first_leg_end():
    # Spawn at (20, 20), route starts at (28, 28) and its first leg runs east to (173, 28). Off the path by
    # more than the lookahead: the carrot is just past the start vertex, up-right, not 150 m east.
    route = [(28.0, 28.0), (173.0, 28.0), (173.0, 173.0), (28.0, 173.0), (28.0, 28.0)]
    cmd = step(20.0, 20.0, 0.0, route, 3.0, 4.0, 2.0)
    assert cmd.angular > 0.5


def test_beside_the_path_the_carrot_is_ahead_not_abeam():
    # 6 m left of a straight leg (beyond the 4 m lookahead), heading along it: steer right and keep
    # moving, instead of turning in place toward the point directly abeam.
    cmd = step(50.0, 6.0, 0.0, [(0.0, 0.0), (100.0, 0.0)], 1.0, 4.0, 1.0)
    assert cmd.angular < 0
    assert cmd.linear > 0.3
