"""Apply Markochev atmospheric refraction corrections for MLAT propagation."""

from __future__ import annotations

import numpy as np

# Define the vacuum speed of light in m/s.
C_VACUUM = 299_792_458.0

# Define the sea-level air propagation speed in m/s.
C_AIR = 299_702_547.0

# Define the sea-level refractivity constant.
A0 = 315e-6

# Define the refractivity decay constant in 1/m.
B = 0.1361e-3


def effective_velocity(h_sensor: float, h_aircraft: float) -> float:
    """Return the effective radio-wave velocity between a sensor and aircraft."""
    dh = h_aircraft - h_sensor
    if abs(dh) < 1.0:
        # Use the local refractivity when both endpoints are at nearly the same altitude.
        return C_VACUUM / (1 + A0 * np.exp(-B * h_sensor))

    correction = A0 / (B * dh) * (np.exp(-B * h_sensor) - np.exp(-B * h_aircraft))
    return C_VACUUM / (1 + correction)


def propagation_time(distance_m: float, h_sensor: float, h_aircraft: float) -> float:
    """Return the refraction-corrected propagation time for a radio path."""
    v = effective_velocity(h_sensor, h_aircraft)
    return distance_m / v
