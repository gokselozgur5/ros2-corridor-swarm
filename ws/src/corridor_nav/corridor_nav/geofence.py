"""Corridor polygon geometry. No ROS here so it is unit-testable.

Coordinates are local ENU metres around an origin lat/lon, same convention as
corridor_sim.vehicle_sim. GeoJSON comes in as lon/lat (RFC 7946 order)."""
import json
import math

EARTH_RADIUS_M = 6371008.8


class LocalFrame:
    def __init__(self, origin_lat, origin_lon):
        self.lat0 = origin_lat
        self.lon0 = origin_lon
        self.cos_lat0 = math.cos(math.radians(origin_lat))

    def to_xy(self, lat, lon):
        x = math.radians(lon - self.lon0) * EARTH_RADIUS_M * self.cos_lat0
        y = math.radians(lat - self.lat0) * EARTH_RADIUS_M
        return x, y

    def to_latlon(self, x, y):
        lat = self.lat0 + math.degrees(y / EARTH_RADIUS_M)
        lon = self.lon0 + math.degrees(x / (EARTH_RADIUS_M * self.cos_lat0))
        return lat, lon


def load_geojson_polygon(path_or_text):
    """First Polygon (or first Polygon feature) in a GeoJSON document; returns (id, [(lat, lon), ...]).

    The closing duplicate vertex is dropped."""
    text = path_or_text
    if not text.lstrip().startswith('{'):
        with open(path_or_text) as f:
            text = f.read()
    doc = json.loads(text)
    candidates = []
    if doc.get('type') == 'FeatureCollection':
        candidates = doc['features']
    elif doc.get('type') == 'Feature':
        candidates = [doc]
    elif doc.get('type') == 'Polygon':
        candidates = [{'geometry': doc, 'properties': {}}]
    for feat in candidates:
        geom = feat.get('geometry') or {}
        if geom.get('type') != 'Polygon':
            continue
        ring = geom['coordinates'][0]
        if len(ring) > 1 and ring[0] == ring[-1]:
            ring = ring[:-1]
        props = feat.get('properties') or {}
        cid = str(props.get('id') or props.get('name') or feat.get('id') or 'corridor')
        return cid, [(lat, lon) for lon, lat in ring]
    raise ValueError('no Polygon in GeoJSON')


def point_in_polygon(x, y, poly):
    """Even-odd rule. poly is [(x, y), ...] without the closing vertex."""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xi = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xi:
                inside = not inside
    return inside


def distance_to_edge(x, y, poly):
    best = float('inf')
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def signed_distance(x, y, poly):
    """Positive inside, negative outside, metres to the nearest edge."""
    d = distance_to_edge(x, y, poly)
    return d if point_in_polygon(x, y, poly) else -d
