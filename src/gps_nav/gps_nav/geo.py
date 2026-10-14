"""WGS84 geodetic coordinates to a fixed local ENU tangent frame."""
import math

A = 6378137.0
F = 1 / 298.257223563
E2 = F * (2 - F)


def validate(lat, lon):
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and
               math.isfinite(v) for v in (lat, lon)):
        raise ValueError('coordinates must be finite numbers')
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError('latitude or longitude outside valid range')


def ecef(lat, lon, altitude=0.0):
    validate(lat, lon)
    if not math.isfinite(altitude):
        raise ValueError('altitude must be finite')
    lat, lon = math.radians(lat), math.radians(lon)
    sinlat, coslat = math.sin(lat), math.cos(lat)
    n = A / math.sqrt(1 - E2 * sinlat * sinlat)
    return ((n + altitude) * coslat * math.cos(lon),
            (n + altitude) * coslat * math.sin(lon),
            (n * (1 - E2) + altitude) * sinlat)


class ENU:
    def __init__(self, lat, lon):
        validate(lat, lon)
        self.lat, self.lon = lat, lon
        self.xyz0 = ecef(lat, lon)
        self.phi, self.lam = math.radians(lat), math.radians(lon)

    def to_local(self, lat, lon):
        x, y, z = ecef(lat, lon)
        dx, dy, dz = x - self.xyz0[0], y - self.xyz0[1], z - self.xyz0[2]
        s, c = math.sin(self.phi), math.cos(self.phi)
        sl, cl = math.sin(self.lam), math.cos(self.lam)
        return (-sl * dx + cl * dy,
                -s * cl * dx - s * sl * dy + c * dz)


def nearby_geodetic(lat, lon, east_m, north_m):
    """Short-distance WGS84 inverse for scripted metre-scale demo goals."""
    validate(lat, lon)
    if not all(math.isfinite(v) for v in (east_m, north_m)):
        raise ValueError('local offsets must be finite')
    phi = math.radians(lat)
    denominator = 1 - E2 * math.sin(phi) ** 2
    meridional = A * (1 - E2) / denominator ** 1.5
    prime_vertical = A / math.sqrt(denominator)
    if abs(math.cos(phi)) < 1e-9 and east_m:
        raise ValueError('East offset is undefined at the pole')
    new_lat = lat + math.degrees(north_m / meridional)
    new_lon = lon + math.degrees(east_m / (prime_vertical * math.cos(phi))) if east_m else lon
    if not -90 <= new_lat <= 90:
        raise ValueError('offset crosses a pole')
    return new_lat, (new_lon + 180) % 360 - 180
