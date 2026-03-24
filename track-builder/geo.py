from __future__ import annotations

import math

import numpy as np

WGS84_A = 6_378_137.0
WGS84_F = 1 / 298.257223563
WGS84_B = WGS84_A * (1 - WGS84_F)
WGS84_E2 = 1 - (WGS84_B / WGS84_A) ** 2


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)

    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1 - WGS84_E2) + alt_m) * sin_lat

    return np.array([x, y, z], dtype=np.float64)


def ecef_to_lla(x: float, y: float, z: float) -> tuple[float, float, float]:
    lon = math.atan2(y, x)
    p = math.sqrt(x ** 2 + y ** 2)
    lat = math.atan2(z, p * (1 - WGS84_E2))

    for _ in range(10):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)
        lat_new = math.atan2(z + WGS84_E2 * n * sin_lat, p)
        if abs(lat_new - lat) < 1e-12:
            break
        lat = lat_new

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)

    if abs(cos_lat) > 1e-10:
        alt = p / cos_lat - n
    else:
        alt = abs(z) / abs(sin_lat) - n * (1 - WGS84_E2)

    return math.degrees(lat), math.degrees(lon), alt
