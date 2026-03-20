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


def pre_check_gdop(
    sensor_positions: np.ndarray, centroid: np.ndarray, threshold: float = 20.0
) -> bool:
    """Quick GDOP pre-check using sensor centroid as rough aircraft position.

    Args:
        sensor_positions: Array of sensor positions, shape (N, 3) in ECEF.
        centroid: Centroid of sensor positions in ECEF.
        threshold: Maximum acceptable GDOP value.

    Returns:
        True if geometry is acceptable (GDOP <= threshold).
    """
    gdop = compute_gdop(centroid, sensor_positions)
    return gdop <= threshold
