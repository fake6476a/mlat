"""MLAT Solver pipeline — routes correlation groups through the solver chain.

Implements the full solver pipeline from Part 20 of the reference:

1. Route by sensor count:
   - 5+ sensors → Inamdar exact algebraic (no iteration)
   - 4 sensors + altitude → Inamdar with altitude disambiguation
   - 3 sensors + altitude → Constrained TDOA (determined system)
   - 2 sensors → Cannot solve (prediction-aided requires Layer 5 tracker)
   - 0-1 sensors → Cannot solve

2. Iterative refinement:
   - Frisch TOA formulation with scipy.optimize.least_squares
   - loss='huber' for outlier robustness
   - Atmospheric refraction velocity (Markochev model)
   - Initial guess = algebraic solution from step 1

3. Validation:
   - Position within MAX_RANGE of all sensors
   - GDOP computation → reject if > threshold
   - Altitude consistency check
   - Offset (t0) non-negative and reasonable

References:
  - MLAT_Verified_Combined_Reference.md Part 20 (Solver pipeline)
  - MLAT_Verified_Combined_Reference.md Part 13.1 (mutability analysis)
  - MLAT_Verified_Combined_Reference.md Part 18 (Frisch formulation)
  - MLAT_Verified_Combined_Reference.md Part 19 (Inamdar solutions)
  - MLAT_Verified_Combined_Reference.md Part 21 (Outlier rejection)
"""

from __future__ import annotations

import numpy as np

from atmosphere import C_VACUUM
from frisch import solve_constrained_3sensor, solve_toa
from gdop import compute_gdop, compute_gdop_2d
from geo import ecef_to_lla, ft_to_m, lla_to_ecef, m_to_ft
from inamdar import centroid_init, inamdar_4sensor_altitude, inamdar_5sensor

# Maximum range from any sensor to accept a solution (500 km)
MAX_RANGE_M = 500_000.0

# Maximum acceptable GDOP value (reference doc Part 4.3: "Skip if GDOP > 10-20")
# Using upper bound of range since Cornwall sensors are nearly coplanar
# (Part 28), which naturally inflates vertical GDOP.
MAX_GDOP = 20.0
MAX_GDOP_2D = 10.0  # Tighter for 2D (altitude known), horizontal-only GDOP

# Maximum acceptable residual (meters)
MAX_RESIDUAL_M = 10_000.0
MAX_RESIDUAL_PRIOR_M = 200.0  # Tuned: 3000m balances solve count vs accuracy

# Maximum distance from prior position for prediction-aided solves (meters)
MAX_PRIOR_DRIFT_M = 30_000.0  # 30 km — reject if solution drifts too far from prior
MAX_PRIOR_DRIFT_2SENSOR_M = 5_000.0

# Maximum acceptable t0 offset (seconds) — should be small positive value
MAX_T0_OFFSET_S = 1.0

# Minimum number of sensors to attempt a solve
MIN_SENSORS_NO_ALT = 4  # without altitude (was 5, lowered per Frisch TOA formulation)
MIN_SENSORS_WITH_ALT = 3  # with altitude constraint
MIN_SENSORS_WITH_PRIOR = 2  # with altitude + cached position prior


class SolveResult:
    """Result of a single MLAT solve attempt."""

    __slots__ = (
        "icao", "lat", "lon", "alt_ft", "residual_m", "quality_residual_m", "gdop",
        "num_sensors", "solve_method", "timestamp_s", "timestamp_ns",
        "df_type", "squawk", "raw_msg", "t0_s",
    )

    def __init__(
        self,
        icao: str,
        lat: float,
        lon: float,
        alt_ft: float,
        residual_m: float,
        quality_residual_m: float,
        gdop: float,
        num_sensors: int,
        solve_method: str,
        timestamp_s: int,
        timestamp_ns: int,
        df_type: int,
        squawk: str | None,
        raw_msg: str,
        t0_s: float,
    ) -> None:
        self.icao = icao
        self.lat = lat
        self.lon = lon
        self.alt_ft = alt_ft
        self.residual_m = residual_m
        self.quality_residual_m = quality_residual_m
        self.gdop = gdop
        self.num_sensors = num_sensors
        self.solve_method = solve_method
        self.timestamp_s = timestamp_s
        self.timestamp_ns = timestamp_ns
        self.df_type = df_type
        self.squawk = squawk
        self.raw_msg = raw_msg
        self.t0_s = t0_s

    def to_dict(self) -> dict:
        return {
            "icao": self.icao,
            "lat": round(self.lat, 6),
            "lon": round(self.lon, 6),
            "alt_ft": round(self.alt_ft, 0),
            "residual_m": round(self.residual_m, 2),
            "quality_residual_m": round(self.quality_residual_m, 2),
            "gdop": round(self.gdop, 2),
            "num_sensors": self.num_sensors,
            "solve_method": self.solve_method,
            "timestamp_s": self.timestamp_s,
            "timestamp_ns": self.timestamp_ns,
            "df_type": self.df_type,
            "squawk": self.squawk,
            "raw_msg": self.raw_msg,
            "t0_s": round(self.t0_s, 9),
        }


def _per_sensor_residuals(
    position: np.ndarray,
    sensors: np.ndarray,
    arrival_times: np.ndarray,
    sensor_alts_m: np.ndarray,
    altitude_m: float | None,
) -> np.ndarray:
    """Compute per-sensor TOA residuals in meters for outlier detection.

    Uses the Frisch analytical t0 elimination to compute how much each
    sensor's observed arrival time deviates from the model.
    """
    from atmosphere import C_VACUUM, effective_velocity

    N = len(sensors)
    ranges = np.array([np.linalg.norm(position - s) for s in sensors])

    if altitude_m is not None:
        velocities = np.array([
            effective_velocity(h_s, altitude_m) for h_s in sensor_alts_m
        ])
    else:
        velocities = np.full(N, C_VACUUM)

    t0_hat = np.mean(arrival_times - ranges / velocities)
    residuals = np.abs((arrival_times - t0_hat) * velocities - ranges)
    return residuals


# Threshold for per-sensor residual to be considered an outlier (meters)
OUTLIER_SENSOR_THRESHOLD_M = 3_000.0


def _solve_with_outlier_rejection(
    sensor_positions: np.ndarray,
    arrival_times: np.ndarray,
    sensor_alts_m: np.ndarray,
    altitude_m: float | None,
    position_prior_ecef: np.ndarray | None,
    min_sensors: int,
) -> tuple[dict | None, np.ndarray, np.ndarray, np.ndarray, str]:
    """Solve with iterative outlier sensor rejection.

    For groups with more sensors than the minimum, solve, check per-sensor
    residuals, remove the worst outlier sensor, and re-solve. Repeat until
    no outliers remain or we hit the minimum sensor count.

    Returns:
        Tuple of (result_dict, used_sensors, used_times, used_alts, method).
    """
    sensors = sensor_positions.copy()
    times = arrival_times.copy()
    alts = sensor_alts_m.copy()
    n = len(sensors)

    best_result = None
    best_method = "unknown"

    for iteration in range(n - min_sensors + 1):
        cur_n = len(sensors)

        # Step 1: Initialize
        x0 = None
        method = "unknown"

        if cur_n >= 5:
            # Try multiple reference sensors for Inamdar
            best_init = None
            for ref_shift in range(min(cur_n, 4)):
                shifted_sensors = np.roll(sensors, -ref_shift, axis=0)
                shifted_times = np.roll(times, -ref_shift)
                candidate = inamdar_5sensor(shifted_sensors, shifted_times)
                if candidate is not None:
                    best_init = candidate
                    break
            x0 = best_init
            if x0 is not None:
                method = "inamdar_5sensor"

        if x0 is None and cur_n >= 4 and altitude_m is not None:
            x0 = inamdar_4sensor_altitude(sensors[:4], times[:4], altitude_m)
            if x0 is not None:
                method = "inamdar_4sensor_alt"

        if x0 is None and position_prior_ecef is not None:
            x0 = position_prior_ecef.copy()
            method = "prior_aided"

        if x0 is None:
            x0 = centroid_init(sensors, altitude_m)
            method = "centroid_init"

        # Step 2: Solve — adaptive prediction weight
        pw = 8.0 if cur_n <= 2 else 3.0
        if cur_n == 2 and altitude_m is not None and position_prior_ecef is not None:
            result = solve_toa(
                sensors=sensors, arrival_times=times,
                sensor_alts_m=alts, x0=x0,
                altitude_m=altitude_m,
                track_prediction_ecef=position_prior_ecef,
                prediction_weight=pw,
            )
            if result is not None:
                method = "prior_2sensor"
        elif cur_n == 3 and altitude_m is not None:
            result = solve_constrained_3sensor(
                sensors=sensors, arrival_times=times,
                sensor_alts_m=alts, altitude_m=altitude_m, x0=x0,
            )
            if result is not None and method in ("centroid_init", "prior_aided", "grid_search"):
                method = "constrained_3sensor"
        else:
            result = solve_toa(
                sensors=sensors, arrival_times=times,
                sensor_alts_m=alts, x0=x0,
                altitude_m=altitude_m,
                track_prediction_ecef=None,
                prediction_weight=pw,
            )
            if result is not None and method in ("centroid_init", "prior_aided", "grid_search"):
                method = "frisch_toa"

        if result is None:
            break

        best_result = result
        best_method = method

        # Step 3: Check if we can/should reject an outlier
        if cur_n <= min_sensors:
            break  # Can't remove any more sensors

        per_sensor = _per_sensor_residuals(
            result["position"], sensors, times, alts, altitude_m
        )
        worst_idx = int(np.argmax(per_sensor))
        worst_residual = per_sensor[worst_idx]

        if worst_residual < OUTLIER_SENSOR_THRESHOLD_M:
            break  # No outliers, solution is good

        # Remove worst sensor and retry
        mask = np.ones(cur_n, dtype=bool)
        mask[worst_idx] = False
        sensors = sensors[mask]
        times = times[mask]
        alts = alts[mask]

    return best_result, sensors, times, alts, best_method


def solve_group(
    group: dict,
    position_prior_ecef: np.ndarray | None = None,
) -> SolveResult | None:
    """Solve a single correlation group from Layer 3.

    Routes the group through the solver chain based on sensor count
    and available altitude information. For groups with 5+ sensors,
    applies iterative outlier sensor rejection.

    Args:
        group: Correlation group dict from Layer 3 with fields:
            icao, df_type, altitude_ft, squawk, raw_msg,
            num_sensors, receptions[]
        position_prior_ecef: Optional cached position from previous solve
            for this ICAO. Used for prediction-aided solving with fewer
            sensors and as a better initial guess.

    Returns:
        Tuple of (SolveResult, None) if successful, or (None, failure_reason_str).
    """
    icao = group.get("icao", "")
    df_type = group.get("df_type", 0)
    altitude_ft = group.get("altitude_ft")
    squawk = group.get("squawk")
    raw_msg = group.get("raw_msg", "")
    receptions = group.get("receptions", [])

    n_sensors = len(receptions)
    if n_sensors < 2:
        return None, "too_few_sensors"

    # Convert altitude from feet to meters if available
    altitude_m: float | None = None
    if altitude_ft is not None:
        altitude_m = ft_to_m(altitude_ft)

    # Check minimum sensor requirements
    if altitude_m is not None and position_prior_ecef is not None:
        if n_sensors < MIN_SENSORS_WITH_PRIOR:
            return None, "too_few_sensors"
    elif altitude_m is not None:
        if n_sensors < MIN_SENSORS_WITH_ALT:
            return None, "too_few_sensors"
    else:
        if n_sensors < MIN_SENSORS_NO_ALT:
            return None, "too_few_sensors"

    # Extract sensor positions (ECEF) and arrival times
    sensor_positions = np.zeros((n_sensors, 3))
    sensor_alts_m = np.zeros(n_sensors)
    arrival_times = np.zeros(n_sensors)

    # Compute reference timestamp (earliest reception) to prevent float64 precision loss!
    # If timestamp_s is large (e.g. UNIX epoch), adding nanoseconds as float will truncate.
    # We must make it relative first before casting to float.
    ref_idx = min(range(n_sensors), key=lambda i: receptions[i]["timestamp_s"] * 1000000000 + receptions[i]["timestamp_ns"])
    ref_timestamp_s = receptions[ref_idx]["timestamp_s"]
    ref_timestamp_ns = receptions[ref_idx]["timestamp_ns"]

    for i, rec in enumerate(receptions):
        sensor_positions[i] = lla_to_ecef(
            rec["lat"], rec["lon"], rec["alt"]
        )
        sensor_alts_m[i] = rec["alt"]
        # Make timestamp relative BEFORE converting to float to save precision
        dt_s = rec["timestamp_s"] - ref_timestamp_s
        dt_ns = rec["timestamp_ns"] - ref_timestamp_ns
        arrival_times[i] = dt_s + dt_ns * 1e-9

    # Determine minimum sensors for outlier rejection
    if altitude_m is not None and position_prior_ecef is not None:
        min_sensors_needed = MIN_SENSORS_WITH_PRIOR
    elif altitude_m is not None:
        min_sensors_needed = MIN_SENSORS_WITH_ALT
    else:
        min_sensors_needed = MIN_SENSORS_NO_ALT

    # Solve with outlier rejection for 4+ sensor groups
    # Don't allow dropping below 3 sensors — 2-sensor via outlier path has poor P95
    if n_sensors >= 4:
        outlier_min = max(3, min_sensors_needed)
        result, used_sensors, used_times, used_alts, solve_method = \
            _solve_with_outlier_rejection(
                sensor_positions, arrival_times, sensor_alts_m,
                altitude_m, position_prior_ecef, outlier_min,
            )
        n_used = len(used_sensors)
        if n_used < n_sensors:
            solve_method += f"_drop{n_sensors - n_used}"
    else:
        # Standard solve path for 2-3 sensor groups
        x0 = None
        solve_method = "unknown"

        if position_prior_ecef is not None:
            x0 = position_prior_ecef.copy()
            solve_method = "prior_aided"

        if x0 is None:
            x0 = centroid_init(sensor_positions, altitude_m)
            solve_method = "centroid_init"

        # Adaptive prediction weight: strong for 2-sensor, weaker for 3+
        pw = 8.0 if n_sensors <= 2 else 3.0
        if n_sensors == 2 and altitude_m is not None and position_prior_ecef is not None:
            result = solve_toa(
                sensors=sensor_positions,
                arrival_times=arrival_times,
                sensor_alts_m=sensor_alts_m,
                x0=x0,
                altitude_m=altitude_m,
                track_prediction_ecef=position_prior_ecef,
                prediction_weight=pw,
            )
            if result is not None:
                solve_method = "prior_2sensor"
        elif n_sensors == 3 and altitude_m is not None:
            result = solve_constrained_3sensor(
                sensors=sensor_positions,
                arrival_times=arrival_times,
                sensor_alts_m=sensor_alts_m,
                altitude_m=altitude_m,
                x0=x0,
            )
            if result is not None and solve_method in ("centroid_init", "prior_aided", "grid_search"):
                solve_method = "constrained_3sensor"
        else:
            result = solve_toa(
                sensors=sensor_positions,
                arrival_times=arrival_times,
                sensor_alts_m=sensor_alts_m,
                x0=x0,
                altitude_m=altitude_m,
                track_prediction_ecef=None,
                prediction_weight=pw,
            )
            if result is not None and solve_method in ("centroid_init", "prior_aided", "grid_search"):
                solve_method = "frisch_toa"
        n_used = n_sensors
        used_sensors = sensor_positions

    if result is None:
        return None, "solver_diverged"

    position = result["position"]
    residual_m = result["residual_m"]
    prior_offset_m = 0.0
    if position_prior_ecef is not None:
        prior_offset_m = float(np.linalg.norm(position - position_prior_ecef))
    if n_sensors == 2 and position_prior_ecef is not None:
        quality_residual_m = max(residual_m, prior_offset_m)
    else:
        quality_residual_m = residual_m
    t0_s = result["t0_s"]

    # Step 3: Validation

    # 3a. Range check — position must be within MAX_RANGE of all sensors
    for s in used_sensors:
        if np.linalg.norm(position - s) > MAX_RANGE_M:
            return None, "range_exceeded"

    # 3b. Residual check — tighter thresholds for better-determined systems
    if n_sensors == 2 and position_prior_ecef is not None:
        effective_max_residual = MAX_RESIDUAL_PRIOR_M
    elif n_used == 3 and altitude_m is not None:
        effective_max_residual = 2_000.0  # 2km — 3-sensor+alt is well-determined
    elif n_used >= 4:
        effective_max_residual = 5_000.0  # 5km — overdetermined
    else:
        effective_max_residual = MAX_RESIDUAL_M
    if residual_m > effective_max_residual:
        return None, "residual_exceeded"

    # 3b2. Prior drift check — prediction-aided solutions must stay near the prior
    if position_prior_ecef is not None and n_sensors <= 3:
        max_prior_drift = MAX_PRIOR_DRIFT_2SENSOR_M if n_sensors == 2 else MAX_PRIOR_DRIFT_M
        if prior_offset_m > max_prior_drift:
            return None, "prior_drift_exceeded"

    # 3c. GDOP computation (Part 4.3)
    # A full 3D+Time GDOP requires a 4x4 matrix inversion which is singular for < 4 sensors.
    if n_used >= 4:
        gdop = compute_gdop(position, used_sensors)
        if gdop > MAX_GDOP:
            return None, "gdop_exceeded"
    elif n_used == 3 and altitude_m is not None:
        # 2D horizontal GDOP for 3-sensor + altitude cases (Finding 7 / R2)
        gdop = compute_gdop_2d(position, used_sensors)
        if gdop > MAX_GDOP_2D:
            return None, "gdop_2d_exceeded"
    else:
        gdop = 0.0  # GDOP is not applicable for 2-sensor determined systems

    # Convert result back to WGS84
    lat, lon, alt_m = ecef_to_lla(position[0], position[1], position[2])

    # Use known altitude if available, otherwise use solved altitude
    if altitude_ft is not None:
        final_alt_ft = float(altitude_ft)
    else:
        final_alt_ft = m_to_ft(alt_m)

    return SolveResult(
        icao=icao,
        lat=lat,
        lon=lon,
        alt_ft=final_alt_ft,
        residual_m=residual_m,
        quality_residual_m=quality_residual_m,
        gdop=gdop,
        num_sensors=n_used,
        solve_method=solve_method,
        timestamp_s=ref_timestamp_s,
        timestamp_ns=ref_timestamp_ns,
        df_type=df_type,
        squawk=squawk,
        raw_msg=raw_msg,
        t0_s=t0_s,
    ), None
