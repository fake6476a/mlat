"""GDOP (Geometric Dilution of Precision) computation for MLAT.

Pre-computes GDOP before calling the solver. Groups with poor sensor
geometry (high GDOP) are skipped because they will produce inaccurate
solutions regardless of the solver quality.

References:
  - MLAT_Verified_Combined_Reference.md Part 4.3
  - MLAT_Verified_Combined_Reference.md Part 22 (Key formulas)
  - No dedicated GDOP-for-MLAT Python library exists (Part 7, gap #2)
"""

from __future__ import annotations

import numpy as np


def compute_gdop(aircraft_pos: np.ndarray, sensor_positions: np.ndarray) -> float:
    """Compute Geometric Dilution of Precision.

    GDOP measures how sensor geometry affects position accuracy.
    Lower GDOP = better geometry = more accurate solutions.

    Args:
        aircraft_pos: Estimated aircraft position [x, y, z] in ECEF meters.
        sensor_positions: Array of sensor positions, shape (N, 3) in ECEF meters.

    Returns:
        GDOP value. Lower is better. Infinity if computation fails.
    """
    n_sensors = len(sensor_positions)
    if n_sensors < 3:
        return float("inf")

    # Build geometry matrix H: unit vectors from aircraft to each sensor,
    # plus a column of ones for the time offset
    H = []
    for s in sensor_positions:
        r = np.linalg.norm(aircraft_pos - s)
        if r < 1.0:
            return float("inf")
        H.append((aircraft_pos - s) / r)

    H = np.array(H)
    # Append column of ones for clock offset dimension
    H = np.column_stack([H, np.ones(n_sensors)])

    try:
        cov = np.linalg.inv(H.T @ H)
        trace_val = np.trace(cov)
        if trace_val < 0 or not np.isfinite(trace_val):
            return float("inf")
        return float(np.sqrt(trace_val))
    except np.linalg.LinAlgError:
        return float("inf")


def compute_gdop_2d(aircraft_pos: np.ndarray, sensor_positions: np.ndarray) -> float:
    """Compute 2D (horizontal-only) GDOP for 3-sensor + altitude groups.

    When altitude is known, the vertical DOF is fixed. The effective geometry
    matrix only needs horizontal unit vectors, giving a meaningful GDOP for
    3-sensor cases that would have a singular 4x4 H^T H matrix.

    Args:
        aircraft_pos: Aircraft position [x, y, z] in ECEF meters.
        sensor_positions: Sensor positions, shape (N, 3) in ECEF meters.

    Returns:
        2D GDOP value. Lower is better. Infinity if computation fails.
    """
    from geo import ecef_to_lla

    n_sensors = len(sensor_positions)
    if n_sensors < 2:
        return float("inf")

    # Convert to local ENU frame to extract horizontal components
    lat, lon, _ = ecef_to_lla(aircraft_pos[0], aircraft_pos[1], aircraft_pos[2])
    lat_r = np.radians(lat)
    lon_r = np.radians(lon)

    # ENU rotation: East and North unit vectors
    east = np.array([-np.sin(lon_r), np.cos(lon_r), 0.0])
    north = np.array([
        -np.sin(lat_r) * np.cos(lon_r),
        -np.sin(lat_r) * np.sin(lon_r),
        np.cos(lat_r),
    ])

    H_rows = []
    for s in sensor_positions:
        diff = aircraft_pos - s
        r = np.linalg.norm(diff)
        if r < 1.0:
            return float("inf")
        unit = diff / r
        H_rows.append([float(np.dot(unit, east)), float(np.dot(unit, north))])

    H2 = np.array(H_rows)  # shape (N, 2)

    try:
        cov = np.linalg.inv(H2.T @ H2)
        trace_val = np.trace(cov)
        if trace_val < 0 or not np.isfinite(trace_val):
            return float("inf")
        return float(np.sqrt(trace_val))
    except np.linalg.LinAlgError:
        return float("inf")


def pre_check_gdop(
    sensor_positions: np.ndarray,
    centroid: np.ndarray,
    threshold: float = 20.0,
    position_prior: np.ndarray | None = None,
) -> bool:
    """Quick GDOP pre-check.

    Uses position_prior as GDOP evaluation point when available (more
    accurate), falling back to sensor centroid.

    Args:
        sensor_positions: Array of sensor positions, shape (N, 3) in ECEF.
        centroid: Centroid of sensor positions in ECEF.
        threshold: Maximum acceptable GDOP value.
        position_prior: Optional prior position from cache for better accuracy.

    Returns:
        True if geometry is acceptable (GDOP <= threshold).
    """
    eval_pos = position_prior if position_prior is not None else centroid
    gdop = compute_gdop(eval_pos, sensor_positions)
    return gdop <= threshold
