import math

import pytest

from corridor_nav.geofence import LocalFrame, load_geojson_polygon, point_in_polygon, signed_distance

SQUARE = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]
GEOJSON = '''{"type":"FeatureCollection","features":[{"type":"Feature","properties":{"id":"c1"},
"geometry":{"type":"Polygon","coordinates":[[[-3.0,52.7],[-2.6,52.6],[-2.0,52.4],[-3.0,52.7]]]}}]}'''


def polygon(ring):
    return '{"type":"Polygon","coordinates":[%s]}' % ring


def test_inside_and_outside():
    assert point_in_polygon(50.0, 50.0, SQUARE)
    assert not point_in_polygon(150.0, 50.0, SQUARE)
    assert not point_in_polygon(50.0, -1.0, SQUARE)


def test_signed_distance_sign_and_magnitude():
    assert math.isclose(signed_distance(50.0, 10.0, SQUARE), 10.0)
    assert math.isclose(signed_distance(50.0, -10.0, SQUARE), -10.0)
    assert math.isclose(signed_distance(120.0, 120.0, SQUARE), -math.hypot(20.0, 20.0))


def test_concave_polygon():
    # U shape: the notch (50, 80) is outside although it is inside the bounding box.
    u = [(0, 0), (100, 0), (100, 100), (70, 100), (70, 30), (30, 30), (30, 100), (0, 100)]
    assert not point_in_polygon(50.0, 80.0, u)
    assert point_in_polygon(15.0, 80.0, u)


def test_geojson_lonlat_order_and_closing_vertex():
    cid, ring = load_geojson_polygon(GEOJSON)
    assert cid == 'c1'
    assert len(ring) == 3
    assert ring[0] == (52.7, -3.0)  # (lat, lon)


# nav-mission-5: RFC 7946 positions may carry an altitude; KML->GeoJSON exports always do.
def test_geojson_position_with_altitude():
    _, ring = load_geojson_polygon(polygon('[[-3.0,52.7,0],[-2.997,52.7,0],[-2.997,52.7018,0],[-3.0,52.7,0]]'))
    assert ring == [(52.7, -3.0), (52.7, -2.997), (52.7018, -2.997)]


def test_geojson_closing_vertex_compared_on_lonlat_only():
    # Same lon/lat first and last, different altitude: still the closing duplicate.
    _, ring = load_geojson_polygon(polygon('[[-3.0,52.7,0],[-2.997,52.7,0],[-2.997,52.7018,0],[-3.0,52.7,5]]'))
    assert len(ring) == 3


def test_geojson_unclosed_ring_with_altitude_is_accepted():
    _, ring = load_geojson_polygon(polygon('[[-3.0,52.7,1],[-2.997,52.7,1],[-2.997,52.7018,1]]'))
    assert ring == [(52.7, -3.0), (52.7, -2.997), (52.7018, -2.997)]


# nav-mission-5: with 2 vertices point_in_polygon is always False, every fix would be an EXIT.
def test_geojson_ring_with_two_vertices_is_rejected():
    with pytest.raises(ValueError, match='vertices'):
        load_geojson_polygon(polygon('[[-1,-1],[1,1],[-1,-1]]'))
    with pytest.raises(ValueError, match='vertices'):
        load_geojson_polygon(polygon('[[-1,-1],[1,1]]'))


def test_geojson_empty_ring_is_rejected():
    with pytest.raises(ValueError):
        load_geojson_polygon('{"type":"Polygon","coordinates":[]}')
    with pytest.raises(ValueError):
        load_geojson_polygon('{"type":"Polygon","coordinates":[[]]}')


@pytest.mark.parametrize('bad', ['[-2.997,NaN]', '[Infinity,52.7]', '["-2.997","52.7"]', '[true,52.7]', '[-2.997]', 'null'])
def test_geojson_bad_position_is_rejected(bad):
    with pytest.raises(ValueError, match='position 1'):
        load_geojson_polygon(polygon('[[-3.0,52.7],%s,[-2.997,52.7018],[-3.0,52.7018],[-3.0,52.7]]' % bad))


def test_geojson_collinear_ring_is_rejected():
    with pytest.raises(ValueError, match='area'):
        load_geojson_polygon(polygon('[[0,0],[1,1],[2,2],[0,0]]'))


def test_local_frame_roundtrip():
    f = LocalFrame(52.7, -3.0)
    x, y = f.to_xy(52.6, -2.6)
    lat, lon = f.to_latlon(x, y)
    assert math.isclose(lat, 52.6, abs_tol=1e-9) and math.isclose(lon, -2.6, abs_tol=1e-9)
    assert y < 0 and x > 0  # south and east of the origin
