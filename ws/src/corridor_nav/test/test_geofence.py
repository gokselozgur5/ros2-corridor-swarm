import math

from corridor_nav.geofence import LocalFrame, load_geojson_polygon, point_in_polygon, signed_distance

SQUARE = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]
GEOJSON = '''{"type":"FeatureCollection","features":[{"type":"Feature","properties":{"id":"c1"},
"geometry":{"type":"Polygon","coordinates":[[[-3.0,52.7],[-2.6,52.6],[-2.0,52.4],[-3.0,52.7]]]}}]}'''


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


def test_local_frame_roundtrip():
    f = LocalFrame(52.7, -3.0)
    x, y = f.to_xy(52.6, -2.6)
    lat, lon = f.to_latlon(x, y)
    assert math.isclose(lat, 52.6, abs_tol=1e-9) and math.isclose(lon, -2.6, abs_tol=1e-9)
    assert y < 0 and x > 0  # south and east of the origin
