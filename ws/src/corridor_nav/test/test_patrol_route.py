import math

from corridor_nav.patrol_server import inset_polygon, lap_route

SQUARE = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]


def test_inset_moves_vertices_inward():
    inner = inset_polygon(SQUARE, 10.0)
    for (x, y) in inner:
        assert 0 < x < 100 and 0 < y < 100
    # Corner moved along the diagonal by 10 m.
    assert math.isclose(math.dist(inner[0], (0.0, 0.0)), 10.0)


def test_lap_route_starts_near_vehicle_and_closes():
    route = lap_route(SQUARE, (95.0, 5.0), clockwise=False)
    assert route[0] == (100.0, 0.0)
    assert route[-1] == route[0]
    assert len(route) == 5


def test_lap_route_direction():
    ccw = lap_route(SQUARE, (0.0, 0.0), clockwise=False)
    cw = lap_route(SQUARE, (0.0, 0.0), clockwise=True)
    assert ccw[1] == (100.0, 0.0)   # east first when counter-clockwise from the SW corner
    assert cw[1] == (0.0, 100.0)    # north first when clockwise
