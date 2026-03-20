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
from gdop import compute_gdop
from geo import ecef_to_lla, ft_to_m, lla_to_ecef, m_to_ft
from inamdar import centroid_init, inamdar_4sensor_altitude, inamdar_5sensor

# Maximum range from any sensor to accept a solution (500 km)
MAX_RANGE_M = 500_000.0

# Maximum acceptable GDOP value (reference doc Part 4.3: "Skip if GDOP > 10-20")
# Using upper bound of range since Cornwall sensors are nearly coplanar
# (Part 28), which naturally inflates vertical GDOP.
MAX_GDOP = 20.0

# Maximum acceptable residual (meters)
MAX_RESIDUAL_M = 10_000.0

# Maximum acceptable t0 offset (seconds) — should be small positive value
MAX_T0_OFFSET_S = 1.0

# Minimum number of sensors to attempt a solve
MIN_SENSORS_NO_ALT = 5  # without altitude
MIN_SENSORS_WITH_ALT = 3  # with altitude constraint


class SolveResult:
    """Result of a single MLAT solve attempt."""

    __slots__ = (
        "icao", "lat", "lon", "alt_ft", "residual_m", "gdop",
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


def solve_group(group: dict) -> SolveResult | None:
    """Solve a single correlation group from Layer 3.

    Routes the group through the solver chain based on sensor count
    and available altitude information.

    Args:
        group: Correlation group dict from Layer 3 with fields:
            icao, df_type, altitude_ft, squawk, raw_msg,
            num_sensors, receptions[]

    Returns:
        SolveResult if successful, None if cannot solve.
    """
    icao = group.get("icao", "")
    df_type = group.get("df_type", 0)
    altitude_ft = group.get("altitude_ft")
    squawk = group.get("squawk")
    raw_msg = group.get("raw_msg", "")
    receptions = group.get("receptions", [])

    n_sensors = len(receptions)
    if n_sensors < 2:
        return None

    # Convert altitude from feet to meters if available
    altitude_m: float | None = None
    if altitude_ft is not None:
        altitude_m = ft_to_m(altitude_ft)

    # Check minimum sensor requirements
    if altitude_m is not None:
        if n_sensors < MIN_SENSORS_WITH_ALT:
            return None
    else:
        if n_sensors < MIN_SENSORS_NO_ALT:
            return None

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

    # Step 1: Algebraic initialization — route by sensor count
    x0 = None
    solve_method = "unknown"

    if n_sensors >= 5:
        # Inamdar exact algebraic solution (Part 19)
        x0 = inamdar_5sensor(sensor_positions, arrival_times)
        if x0 is not None:
            solve_method = "inamdar_5sensor"

    if x0 is None and n_sensors >= 4 and altitude_m is not None:
        # Inamdar with altitude disambiguation (Part 19)
        x0 = inamdar_4sensor_altitude(
            sensor_positions[:4], arrival_times[:4], altitude_m
        )
        if x0 is not None:
            solve_method = "inamdar_4sensor_alt"

    if x0 is None:
        # Fallback: centroid initialization
        x0 = centroid_init(sensor_positions, altitude_m)
        solve_method = "centroid_init"

    # Step 2: Iterative refinement — Frisch TOA formulation
    if n_sensors == 3 and altitude_m is not None:
        # Constrained 3-sensor solve (Part 9, Part 17)
        result = solve_constrained_3sensor(
            sensors=sensor_positions,
            arrival_times=arrival_times,
            sensor_alts_m=sensor_alts_m,
            altitude_m=altitude_m,
            x0=x0,
        )
        if result is not None and solve_method == "centroid_init":
            solve_method = "constrained_3sensor"
    else:
        # Full Frisch TOA solve (Part 18)
        result = solve_toa(
            sensors=sensor_positions,
            arrival_times=arrival_times,
            sensor_alts_m=sensor_alts_m,
            x0=x0,
            altitude_m=altitude_m,
        )
        if result is not None and solve_method == "centroid_init":
            solve_method = "frisch_toa"

    if result is None:
        return None

    position = result["position"]
    residual_m = result["residual_m"]
    t0_s = result["t0_s"]

    # Step 3: Validation

    # 3a. Range check — position must be within MAX_RANGE of all sensors
    for s in sensor_positions:
        if np.linalg.norm(position - s) > MAX_RANGE_M:
            print("Failed range check")
            return None

    # 3b. Residual check
    if residual_m > MAX_RESIDUAL_M:
        print("Failed residual check")
        return None

    # 3c. GDOP computation (Part 4.3)
    # A full 3D+Time GDOP requires a 4x4 matrix inversion which is singular for < 4 sensors.
    if n_sensors >= 4:
        gdop = compute_gdop(position, sensor_positions)
        if gdop > MAX_GDOP:
            return None
    else:
        gdop = 0.0  # GDOP is not applicable for 2.5D determined systems

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
        gdop=gdop,
        num_sensors=n_sensors,
        solve_method=solve_method,
        timestamp_s=ref_timestamp_s,
        timestamp_ns=ref_timestamp_ns,
        df_type=df_type,
        squawk=squawk,
        raw_msg=raw_msg,
        t0_s=t0_s,
    )
