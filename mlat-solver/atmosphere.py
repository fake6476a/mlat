"""Atmospheric refraction velocity correction for MLAT solver.

Implements Markochev's atmospheric model from the OpenSky competition
(1st place Round 2, DOI: 10.3390/engproc2021013012).

The speed of radio waves through the atmosphere is slightly less than
the vacuum speed of light due to refractivity. This correction improves
MLAT accuracy by ~0.1m per receiver pair.

References:
  - MLAT_Verified_Combined_Reference.md Part 13.2 (Markochev model)
  - MLAT_Verified_Combined_Reference.md Part 22 (Key formulas)
  - ITU-R P.453 (refractivity at sea level)
"""

from __future__ import annotations

import numpy as np

# Speed of light in vacuum (m/s)
C_VACUUM = 299_792_458.0

# Speed of light in air at sea level (m/s)
# From mutability/mlat-server: constants.Cair
C_AIR = 299_702_547.0

# Refractivity at sea level (from ITU-R P.453)
A0 = 315e-6

# Exponential decay constant (1/m)
B = 0.1361e-3


def effective_velocity(h_sensor: float, h_aircraft: float) -> float:
    """Compute effective radio wave velocity between sensor and aircraft.

    Uses Markochev's atmospheric refraction model which accounts for
    the varying refractivity of air at different altitudes.

    Args:
        h_sensor: Sensor altitude in meters above sea level.
        h_aircraft: Aircraft altitude in meters above sea level.

    Returns:
        Effective propagation velocity in m/s.
    """
    dh = h_aircraft - h_sensor
    if abs(dh) < 1.0:
        # Same altitude: use refractivity at sensor height
        return C_VACUUM / (1 + A0 * np.exp(-B * h_sensor))

    correction = A0 / (B * dh) * (np.exp(-B * h_sensor) - np.exp(-B * h_aircraft))
    return C_VACUUM / (1 + correction)


def propagation_time(distance_m: float, h_sensor: float, h_aircraft: float) -> float:
    """Compute propagation time accounting for atmospheric refraction.

    Args:
        distance_m: Straight-line distance in meters.
        h_sensor: Sensor altitude in meters.
        h_aircraft: Aircraft altitude in meters.

    Returns:
        Propagation time in seconds.
    """
    v = effective_velocity(h_sensor, h_aircraft)
    return distance_m / v
