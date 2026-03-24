"""Convert between WGS84 geodetic coordinates and ECEF coordinates."""

from __future__ import annotations

import functools
import math

import numpy as np

# Define the core WGS84 ellipsoid constants.
WGS84_A = 6_378_137.0  # semi-major axis (m)
WGS84_F = 1 / 298.257223563  # flattening
WGS84_B = WGS84_A * (1 - WGS84_F)  # semi-minor axis (m)
WGS84_E2 = 1 - (WGS84_B / WGS84_A) ** 2  # first eccentricity squared


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """Convert WGS84 latitude, longitude, and altitude to ECEF meters."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    # Compute the prime-vertical radius of curvature.
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)

    x = (N + alt_m) * cos_lat * cos_lon
    y = (N + alt_m) * cos_lat * sin_lon
    z = (N * (1 - WGS84_E2) + alt_m) * sin_lat

    return np.array([x, y, z], dtype=np.float64)


@functools.lru_cache(maxsize=512)
def sensor_lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """Cache fixed sensor WGS84-to-ECEF conversions for reuse."""
    return lla_to_ecef(lat_deg, lon_deg, alt_m)


def ecef_to_lla(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert ECEF meters to WGS84 latitude, longitude, and altitude."""
    lon = math.atan2(y, x)

    p = math.sqrt(x ** 2 + y ** 2)

    # Seed the latitude iteration with Bowring's method.
    lat = math.atan2(z, p * (1 - WGS84_E2))

    for _ in range(10):
        sin_lat = math.sin(lat)
        N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)
        lat_new = math.atan2(z + WGS84_E2 * N * sin_lat, p)
        if abs(lat_new - lat) < 1e-12:
            break
        lat = lat_new

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)

    if abs(cos_lat) > 1e-10:
        alt = p / cos_lat - N
    else:
        alt = abs(z) / abs(sin_lat) - N * (1 - WGS84_E2)

    return math.degrees(lat), math.degrees(lon), alt


def ft_to_m(feet: float) -> float:
    """Convert feet to meters."""
    return feet * 0.3048


def m_to_ft(meters: float) -> float:
    """Convert meters to feet."""
    return meters / 0.3048
