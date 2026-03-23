"""Frisch TOA formulation for iterative MLAT refinement.

Implements the TOA approach from:
  Frisch & Hanebeck (2025), "Why You Shouldn't Use TDOA for Multilateration",
  MFI 2025. Code reference: github.com/KIT-ISAS/MFI2025_MLAT-TOA

Key insight: Instead of forming pairwise TDOA equations (which loses
information and introduces correlated noise), keep all N TOA equations
and eliminate the unknown transmission time analytically:

    t0_hat(x) = weighted_mean(t_i - ||x - s_i|| / c)

This reduces the problem from 4D (x, y, z, t0) to 3D (x, y, z),
making it simpler and faster than TDOA.

Comparison with mutability/mlat-server (Part 13.1):
  - mutability: [x, y, z, offset] (4D), scipy finds t0 numerically
  - Frisch: [x, y, z] (3D), t0 eliminated analytically

References:
  - MLAT_Verified_Combined_Reference.md Part 18 (Frisch formulation)
  - MLAT_Verified_Combined_Reference.md Part 22 (Key formulas)
  - isas.iar.kit.edu/pdf/MFI25_Frisch.pdf
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares

from atmosphere import C_VACUUM, effective_velocity


def _timing_residuals(
    x: np.ndarray,
    sensors: np.ndarray,
    arrival_times: np.ndarray,
    sensor_alts_m: np.ndarray,
    aircraft_alt_m: float | None,
    c: float = C_VACUUM,
) -> np.ndarray:
    N = len(sensors)
    ranges = np.array([np.linalg.norm(x - s) for s in sensors])

    if aircraft_alt_m is not None:
        velocities = np.array([
            effective_velocity(h_s, aircraft_alt_m) for h_s in sensor_alts_m
        ])
    else:
        velocities = np.full(N, c)

    t0_hat = np.mean(arrival_times - ranges / velocities)
    return (arrival_times - t0_hat) * velocities - ranges


def frisch_residual(
    x: np.ndarray,
    sensors: np.ndarray,
    arrival_times: np.ndarray,
    sensor_alts_m: np.ndarray,
    aircraft_alt_m: float | None,
    track_prediction_ecef: np.ndarray | None = None,
    c: float = C_VACUUM,
    prediction_weight: float = 8.0,
) -> np.ndarray:
    """Compute TOA residuals with analytical t0 elimination.

    The Frisch formulation eliminates t0 in closed form:
        t0_hat = mean(t_i - ||x - s_i|| / c_i)
        residual_i = (t_i - t0_hat) * c - ||x - s_i||

    Uses atmospheric refraction-corrected velocity per sensor pair
    (Markochev model) when aircraft altitude is known.

    Args:
        x: Current position estimate [x, y, z] in ECEF meters.
        sensors: Sensor positions, shape (N, 3) in ECEF meters.
        arrival_times: Arrival times in seconds, shape (N,).
        sensor_alts_m: Sensor altitudes in meters, shape (N,).
        aircraft_alt_m: Aircraft altitude in meters, or None.
        c: Default speed of propagation in m/s.

    Returns:
        Residuals in distance units (meters), shape (N,).
    """
    residuals = _timing_residuals(
        x,
        sensors,
        arrival_times,
        sensor_alts_m,
        aircraft_alt_m,
        c,
    )

    # Optional: altitude constraint as a pseudo-sensor
    if aircraft_alt_m is not None:
        from geo import ecef_to_lla
        _, _, current_alt = ecef_to_lla(x[0], x[1], x[2])
        # Strong weight to force altitude adherence (critical for 2-sensor)
        alt_residual = (current_alt - aircraft_alt_m) * 10.0
        residuals = np.append(residuals, alt_residual)

    # Optional: prediction anchor for underdetermined 2-sensor fallback
    if track_prediction_ecef is not None:
        # Per-axis soft constraint forcing the solver to pick the intersection
        # closest to the EKF prediction. Weight = 5.0 provides a strong anchor
        # for under-determined 2-sensor systems while still allowing geometry
        # to shift the solution when TDOA data clearly supports it.
        # Using per-axis residuals (3 values) instead of scalar distance
        # gives the optimizer better gradient information.
        pred_residuals = (x - track_prediction_ecef) * prediction_weight
        residuals = np.append(residuals, pred_residuals)

    return residuals


def solve_toa(
    sensors: np.ndarray,
    arrival_times: np.ndarray,
    sensor_alts_m: np.ndarray,
    x0: np.ndarray,
    altitude_m: float | None = None,
    track_prediction_ecef: np.ndarray | None = None,
    c: float = C_VACUUM,
    max_nfev: int = 1000,
    prediction_weight: float = 8.0,
) -> dict | None:
    """Iterative MLAT solve using Frisch TOA formulation.

    Uses scipy.optimize.least_squares with:
      - Trust Region Reflective (trf) method
      - Huber loss for outlier robustness (Part 21, Approach 1)
      - f_scale=100.0 (Huber transition at 100m residual)

    Args:
        sensors: Sensor positions, shape (N, 3) in ECEF meters.
        arrival_times: Arrival times in seconds, shape (N,).
        sensor_alts_m: Sensor altitudes in meters, shape (N,).
        x0: Initial position estimate in ECEF [x, y, z].
        altitude_m: Known aircraft altitude in meters, or None.
        c: Speed of propagation in m/s.
        max_nfev: Maximum function evaluations.

    Returns:
        Dict with {position, residual_m, t0_s, success} or None if failed.
    """
    try:
        # Full 3D solve in ECEF coordinates
        result = least_squares(
            frisch_residual,
            x0,
            args=(sensors, arrival_times, sensor_alts_m, altitude_m, track_prediction_ecef, c, prediction_weight),
            method="trf",
            loss="soft_l1",
            f_scale=500.0,  # Moderate outlier transition — balances convergence from far init vs accuracy
            max_nfev=max_nfev,
        )

        if not result.success and result.status <= 0:
            return None

        position = result.x.copy()

        # If altitude is known, correct the solved altitude to match.
        # This implements the "2.5D multilateration" concept from Nik
        # (MLAT_Verified_Combined_Reference.md Part 2.3, tip #2):
        # fix the altitude dimension and use the solved lat/lon.
        if altitude_m is not None:
            from geo import ecef_to_lla, lla_to_ecef

            lat, lon, _ = ecef_to_lla(position[0], position[1], position[2])
            position = lla_to_ecef(lat, lon, altitude_m)

        # Compute final residual (RMS of all residual terms including constraints)
        final_residuals = frisch_residual(
            position, sensors, arrival_times, sensor_alts_m, altitude_m, track_prediction_ecef, c, prediction_weight
        )
        timing_residuals = _timing_residuals(
            position,
            sensors,
            arrival_times,
            sensor_alts_m,
            altitude_m,
            c,
        )
        residual_m = float(np.sqrt(np.mean(timing_residuals ** 2)))
        objective_residual_m = float(np.sqrt(np.mean(final_residuals ** 2)))

        # Compute estimated transmission time
        ranges = np.array([np.linalg.norm(position - s) for s in sensors])
        if altitude_m is not None:
            velocities = np.array([
                effective_velocity(h_s, altitude_m) for h_s in sensor_alts_m
            ])
        else:
            velocities = np.full(len(sensors), c)
        t0_s = float(np.mean(arrival_times - ranges / velocities))

        return {
            "position": position,
            "residual_m": residual_m,
            "objective_residual_m": objective_residual_m,
            "t0_s": t0_s,
            "cost": float(result.cost),
            "nfev": result.nfev,
            "success": True,
        }

    except Exception:
        return None


def solve_constrained_3sensor(
    sensors: np.ndarray,
    arrival_times: np.ndarray,
    sensor_alts_m: np.ndarray,
    altitude_m: float,
    x0: np.ndarray,
    c: float = C_VACUUM,
) -> dict | None:
    """TDOA solve for exactly 3 sensors with known altitude.

    With 3 sensors + altitude constraint, the system is determined.
    Uses the full 3D Frisch TOA formulation with altitude correction.

    This is the standard approach for 3-sensor MLAT as described in
    FR24's operational system (Part 9) and Osypiuk 2025 (Part 17).

    Args:
        sensors: Sensor positions, shape (3, 3) in ECEF meters.
        arrival_times: Arrival times, shape (3,).
        sensor_alts_m: Sensor altitudes in meters, shape (3,).
        altitude_m: Known aircraft altitude in meters.
        x0: Initial position estimate in ECEF.
        c: Speed of propagation in m/s.

    Returns:
        Dict with {position, residual_m, t0_s, success} or None.
    """
    return solve_toa(
        sensors=sensors,
        arrival_times=arrival_times,
        sensor_alts_m=sensor_alts_m,
        x0=x0,
        altitude_m=altitude_m,
        c=c,
        max_nfev=50,
    )
