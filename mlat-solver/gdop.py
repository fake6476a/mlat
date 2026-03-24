"""Compute MLAT GDOP values for geometry-based gating."""

from __future__ import annotations

import numpy as np


def compute_gdop(aircraft_pos: np.ndarray, sensor_positions: np.ndarray) -> float:
    """Return the 3D GDOP at an estimated aircraft position."""
    n_sensors = len(sensor_positions)
    if n_sensors < 3:
        return float("inf")

    # Build the geometry matrix from sensor line-of-sight vectors plus the clock term.
    H = []
    for s in sensor_positions:
        r = np.linalg.norm(aircraft_pos - s)
        if r < 1.0:
            return float("inf")
        H.append((aircraft_pos - s) / r)

    H = np.array(H)
    # Append the transmission-time offset column.
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
    """Return the horizontal GDOP for altitude-constrained MLAT geometry."""
    from geo import ecef_to_lla

    n_sensors = len(sensor_positions)
    if n_sensors < 2:
        return float("inf")

    # Convert to a local ENU frame so only horizontal geometry contributes.
    lat, lon, _ = ecef_to_lla(aircraft_pos[0], aircraft_pos[1], aircraft_pos[2])
    lat_r = np.radians(lat)
    lon_r = np.radians(lon)

    # Build the local East and North basis vectors.
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
    """Return whether the geometry passes a quick GDOP threshold check."""
    eval_pos = position_prior if position_prior is not None else centroid
    gdop = compute_gdop(eval_pos, sensor_positions)
    return gdop <= threshold
