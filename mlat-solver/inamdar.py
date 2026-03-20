"""Inamdar algebraic TDOA solutions for MLAT initialization.

Implements the exact algebraic TDOA solutions from:
  Inamdar N.K. (2025), "Time Difference of Arrival Source Localization:
  Exact Linear Solutions for the General 3D Problem", arXiv:2501.01076v2.

Key properties:
  - 5-sensor case: no sign ambiguity, unique solution
  - 4-sensor case: one sign ambiguity (resolved with altitude constraint)
  - No iteration needed, purely algebraic
  - Used as initialization for iterative refinement (Frisch TOA)

References:
  - MLAT_Verified_Combined_Reference.md Part 19 (Inamdar solutions)
  - MLAT_Verified_Combined_Reference.md Part 20 (Solver pipeline)
  - arXiv:2501.01076v2
"""

from __future__ import annotations

import numpy as np

from atmosphere import C_VACUUM


def inamdar_5sensor(
    sensors: np.ndarray,
    arrival_times: np.ndarray,
    c: float = C_VACUUM,
) -> np.ndarray | None:
    """Exact algebraic TDOA solution for 5+ sensors.

    For 5 sensors, forms a 3x3 linear system A*r_S = b and solves
    directly via matrix inversion. No iteration needed.

    Args:
        sensors: Sensor positions in ECEF, shape (N, 3) where N >= 5.
        arrival_times: Arrival times in seconds, shape (N,).
        c: Speed of propagation in m/s.

    Returns:
        Estimated aircraft position in ECEF [x, y, z], or None if failed.
    """
    if len(sensors) < 5:
        return None

    # Use first sensor as reference
    ref = sensors[0]
    t_ref = arrival_times[0]

    # Relative positions and TDOA range differences
    r = sensors[1:] - ref  # shape (N-1, 3)
    delta = (arrival_times[1:] - t_ref) * c  # range differences

    # Need at least 4 non-reference sensors for 5-sensor case
    # Build the 3x3 system using sensor pairs (1,2), (1,3), (1,4)
    # from the non-reference set (indices 0,1,2,3 in r/delta)
    if len(r) < 4:
        return None

    A = np.zeros((3, 3))
    b_vec = np.zeros(3)

    # Use indices j=0,1,2 and k=3 (or cycling through pairs)
    # Following the Inamdar formulation: for each row, pick two
    # non-reference sensors and form the ratio
    pairs = [(0, 1), (0, 2), (0, 3)]

    for row, (k, j) in enumerate(pairs):
        if abs(delta[j]) < 1e-12:
            return None
        ratio = delta[k] / delta[j]
        A[row, :] = 2 * (r[k] - ratio * r[j])
        b_vec[row] = (
            -(delta[k] ** 2 - ratio * delta[j] ** 2)
            + (np.dot(r[k], r[k]) - ratio * np.dot(r[j], r[j]))
        )

    try:
        # Check matrix condition
        if np.linalg.cond(A) > 1e12:
            return None
        r_s = np.linalg.solve(A, b_vec)
    except np.linalg.LinAlgError:
        return None

    position = r_s + ref

    # Sanity check: position should be within reasonable range of sensors
    max_range = 500_000.0  # 500 km
    for s in sensors:
        if np.linalg.norm(position - s) > max_range:
            return None

    return position


def inamdar_4sensor_altitude(
    sensors: np.ndarray,
    arrival_times: np.ndarray,
    altitude_m: float,
    c: float = C_VACUUM,
) -> np.ndarray | None:
    """Algebraic TDOA solution for 4 sensors with known altitude.

    With 4 sensors, the system yields a quadratic equation with two
    solutions. The known altitude is used to disambiguate and select
    the correct solution.

    Uses the Inamdar ratio-based formulation adapted for 4 sensors:
    builds a 2x3 overdetermined system from 2 sensor pairs, then
    combines with altitude constraint for a full 3D solution.

    Args:
        sensors: Sensor positions in ECEF, shape (4, 3).
        arrival_times: Arrival times in seconds, shape (4,).
        altitude_m: Known aircraft altitude in meters (from DF4).
        c: Speed of propagation in m/s.

    Returns:
        Estimated aircraft position in ECEF [x, y, z], or None if failed.
    """
    if len(sensors) < 4:
        return None

    # Try using the 5-sensor method if we have enough sensors
    # by treating it as an overdetermined system
    if len(sensors) >= 5:
        return inamdar_5sensor(sensors, arrival_times, c)

    # Use first sensor as reference
    ref = sensors[0]
    t_ref = arrival_times[0]

    r = sensors[1:] - ref  # shape (3, 3)
    delta = (arrival_times[1:] - t_ref) * c  # range differences

    # Build the Inamdar ratio-based system for pairs of non-reference sensors
    # With 3 non-reference sensors, we can form 3 pairs: (0,1), (0,2), (1,2)
    pairs = [(0, 1), (0, 2), (1, 2)]
    rows_A = []
    rows_b = []

    for k, j in pairs:
        if abs(delta[j]) < 1e-12:
            continue
        ratio = delta[k] / delta[j]
        row_A = 2 * (r[k] - ratio * r[j])
        row_b = (
            -(delta[k] ** 2 - ratio * delta[j] ** 2)
            + (np.dot(r[k], r[k]) - ratio * np.dot(r[j], r[j]))
        )
        rows_A.append(row_A)
        rows_b.append(row_b)

    if len(rows_A) < 2:
        return None

    A = np.array(rows_A)
    b_vec = np.array(rows_b)

    try:
        # Use least squares for overdetermined system
        result, _, _, _ = np.linalg.lstsq(A, b_vec, rcond=None)
        position = result + ref
    except np.linalg.LinAlgError:
        return None

    # Project to the correct altitude
    from geo import ecef_to_lla, lla_to_ecef

    lat, lon, _ = ecef_to_lla(position[0], position[1], position[2])
    position = lla_to_ecef(lat, lon, altitude_m)

    # Sanity check
    max_range = 500_000.0
    for s in sensors:
        if np.linalg.norm(position - s) > max_range:
            return None

    return position


def centroid_init(
    sensors: np.ndarray,
    altitude_m: float | None = None,
) -> np.ndarray:
    """Fallback initialization using sensor centroid.

    Projects the centroid to the correct altitude if known.

    Args:
        sensors: Sensor positions in ECEF, shape (N, 3).
        altitude_m: Known aircraft altitude in meters, or None.

    Returns:
        Initial position estimate in ECEF [x, y, z].
    """
    centroid = np.mean(sensors, axis=0)

    if altitude_m is not None:
        from geo import ecef_to_lla, lla_to_ecef

        lat, lon, _ = ecef_to_lla(centroid[0], centroid[1], centroid[2])
        centroid = lla_to_ecef(lat, lon, altitude_m)

    return centroid
