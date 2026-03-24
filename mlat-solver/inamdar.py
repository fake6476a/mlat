"""Provide Inamdar algebraic TDOA initializers for MLAT solving."""

from __future__ import annotations

import numpy as np

from atmosphere import C_VACUUM


def inamdar_5sensor(
    sensors: np.ndarray,
    arrival_times: np.ndarray,
    c: float = C_VACUUM,
) -> np.ndarray | None:
    """Return the exact Inamdar initializer for five or more sensors."""
    if len(sensors) < 5:
        return None

    # Use the first sensor as the reference receiver.
    ref = sensors[0]
    t_ref = arrival_times[0]

    # Build relative positions and TDOA range differences.
    r = sensors[1:] - ref  # shape (N-1, 3)
    delta = (arrival_times[1:] - t_ref) * c  # range differences

    # Require four non-reference sensors so the 3x3 system is fully determined.
    if len(r) < 4:
        return None

    A = np.zeros((3, 3))
    b_vec = np.zeros(3)

    # Use fixed non-reference sensor pairs to build the Inamdar ratio system.
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
        # Reject ill-conditioned systems before solving.
        if np.linalg.cond(A) > 1e12:
            return None
        r_s = np.linalg.solve(A, b_vec)
    except np.linalg.LinAlgError:
        return None

    position = r_s + ref

    # Reject positions that fall implausibly far from every sensor.
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
    """Return the altitude-constrained Inamdar initializer for four sensors."""
    if len(sensors) < 4:
        return None

    # Reuse the 5-sensor path when extra receivers are available.
    if len(sensors) >= 5:
        return inamdar_5sensor(sensors, arrival_times, c)

    # Use the first sensor as the reference receiver.
    ref = sensors[0]
    t_ref = arrival_times[0]

    r = sensors[1:] - ref  # shape (3, 3)
    delta = (arrival_times[1:] - t_ref) * c  # range differences

    # Build the ratio system from the three non-reference sensor pairs.
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
        # Solve the overdetermined system in the least-squares sense.
        result, _, _, _ = np.linalg.lstsq(A, b_vec, rcond=None)
        position = result + ref
    except np.linalg.LinAlgError:
        return None

    # Project the solution onto the known altitude.
    from geo import ecef_to_lla, lla_to_ecef

    lat, lon, _ = ecef_to_lla(position[0], position[1], position[2])
    position = lla_to_ecef(lat, lon, altitude_m)

    # Reject positions that fall implausibly far from every sensor.
    max_range = 500_000.0
    for s in sensors:
        if np.linalg.norm(position - s) > max_range:
            return None

    return position


def centroid_init(
    sensors: np.ndarray,
    altitude_m: float | None = None,
) -> np.ndarray:
    """Return a centroid-based fallback initializer in ECEF coordinates."""
    centroid = np.mean(sensors, axis=0)

    if altitude_m is not None:
        from geo import ecef_to_lla, lla_to_ecef

        lat, lon, _ = ecef_to_lla(centroid[0], centroid[1], centroid[2])
        centroid = lla_to_ecef(lat, lon, altitude_m)

    return centroid
