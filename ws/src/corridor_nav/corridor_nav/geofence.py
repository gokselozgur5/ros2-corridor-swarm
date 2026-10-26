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


def _lonlat(cid, i, pos):
    """lon, lat of an RFC 7946 position [lon, lat(, alt)]; the altitude is ignored."""
    try:
        ok = isinstance(pos, (list, tuple)) and len(pos) >= 2 and all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in pos[:2])
    except OverflowError:   # an integer too large for a float
        ok = False
    if not ok:
        raise ValueError(f'polygon {cid!r}: position {i} is not [lon, lat] of finite numbers: {pos!r}')
    return pos[0], pos[1]


def load_geojson_polygon(path_or_text):
    """First Polygon (or first Polygon feature) in a GeoJSON document; returns (id, [(lat, lon), ...]).

    Positions may carry an altitude (RFC 7946), it is dropped. The closing duplicate vertex is
    dropped; an unclosed ring is accepted. ValueError on fewer than 3 vertices or no area."""
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
        props = feat.get('properties') or {}
        cid = str(props.get('id') or props.get('name') or feat.get('id') or 'corridor')
        rings = geom.get('coordinates') or []
        if not isinstance(rings, list) or (rings and not isinstance(rings[0], list)):
            raise ValueError(f'polygon {cid!r}: coordinates must be a list of rings, got {type(rings).__name__}')
        ring = rings[0] if rings else []
        ring = [_lonlat(cid, i, pos) for i, pos in enumerate(ring)]
        if len(ring) > 1 and ring[0] == ring[-1]:
            ring = ring[:-1]
        if len(ring) < 3:
            raise ValueError(f'polygon {cid!r}: ring has {len(ring)} vertices, need at least 3')
        lon0, lat0 = ring[0]
        area2 = sum((a[0] - lon0) * (b[1] - lat0) - (b[0] - lon0) * (a[1] - lat0)
                    for a, b in zip(ring, ring[1:] + ring[:1]))
        if abs(area2) < 1e-12:   # deg^2, about 0.01 m^2: float noise of collinear vertices, never a corridor
            raise ValueError(f'polygon {cid!r}: ring has no area (collinear or repeated vertices)')
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
