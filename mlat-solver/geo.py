"""Geodetic coordinate conversions for MLAT solver.

Converts between WGS84 (lat, lon, alt) and ECEF (x, y, z) coordinate
systems. The MLAT solver works in ECEF meters internally, then converts
results back to WGS84 for output.

References:
  - WGS84 ellipsoid parameters (NIMA TR8350.2)
  - MLAT_Verified_Combined_Reference.md Part 3.3 (Layer 4 spec)
"""

from __future__ import annotations

import math

import numpy as np

# WGS84 ellipsoid constants
WGS84_A = 6_378_137.0  # semi-major axis (m)
WGS84_F = 1 / 298.257223563  # flattening
WGS84_B = WGS84_A * (1 - WGS84_F)  # semi-minor axis (m)
WGS84_E2 = 1 - (WGS84_B / WGS84_A) ** 2  # first eccentricity squared


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """Convert WGS84 (lat, lon, alt) to ECEF (x, y, z) in meters.

    Args:
        lat_deg: Latitude in degrees.
        lon_deg: Longitude in degrees.
        alt_m: Altitude above WGS84 ellipsoid in meters.

    Returns:
        numpy array [x, y, z] in meters.
    """
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    # Radius of curvature in the prime vertical
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat ** 2)

    x = (N + alt_m) * cos_lat * cos_lon
    y = (N + alt_m) * cos_lat * sin_lon
    z = (N * (1 - WGS84_E2) + alt_m) * sin_lat

    return np.array([x, y, z], dtype=np.float64)


def ecef_to_lla(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert ECEF (x, y, z) in meters to WGS84 (lat, lon, alt).

    Uses Bowring's iterative method (converges in 2-3 iterations).

    Args:
        x, y, z: ECEF coordinates in meters.

    Returns:
        Tuple of (latitude_deg, longitude_deg, altitude_m).
    """
    lon = math.atan2(y, x)

    p = math.sqrt(x ** 2 + y ** 2)

    # Initial estimate using Bowring's method
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
