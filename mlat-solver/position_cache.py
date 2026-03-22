"""Per-ICAO position cache for prediction-aided MLAT solving.

Caches the last known solved position for each aircraft (by ICAO address).
This enables:
  - Better initial guesses for the iterative solver (faster convergence)
  - 3-sensor solving without altitude (using cached position as prior)
  - 2-sensor + altitude solving (using cached position to constrain)

Inspired by mutability/mlat-server mlattrack.py:
    last_result_position = ac.last_result_position
    r = solver.solve(cluster, altitude, altitude_error,
                     last_result_position if last_result_position else ...)

References:
    - mutability/mlat-server mlattrack.py (last_result_position pattern)
    - LAYER4_IMPROVEMENT_PLAN.md Phase 3 & 4
"""

from __future__ import annotations

import time

import numpy as np


# Maximum age of a cached position before it's considered stale (seconds)
MAX_CACHE_AGE = 120.0

# Maximum number of cached aircraft
MAX_CACHE_SIZE = 5000

# Maximum plausible aircraft speed (m/s) — ~Mach 3 (~1029 m/s)
# Commercial aircraft rarely exceed Mach 0.9 (~300 m/s), military jets ~Mach 2.5
# Using Mach 3 as generous upper bound to avoid false rejections
MAX_AIRCRAFT_SPEED_MPS = 1030.0

# Maximum plausible acceleration (m/s^2) — generous bound for turns/climbs
MAX_AIRCRAFT_ACCEL_MPS2 = 50.0



class CachedPosition:
    """A single cached aircraft position."""

    __slots__ = (
        "ecef", "lat", "lon", "alt_m",
        "timestamp", "velocity_ecef", "residual_m",
        "solve_count", "last_mono",
    )

    def __init__(
        self,
        ecef: np.ndarray,
        lat: float,
        lon: float,
        alt_m: float,
        timestamp: float,
        velocity_ecef: np.ndarray | None = None,
        residual_m: float = 0.0,
    ) -> None:
        self.ecef = ecef
        self.lat = lat
        self.lon = lon
        self.alt_m = alt_m
        self.timestamp = timestamp  # message timestamp (timestamp_s + timestamp_ns*1e-9)
        self.velocity_ecef = velocity_ecef  # estimated velocity for prediction
        self.residual_m = residual_m  # solve residual for uncertainty-aware gating
        self.solve_count = 1
        self.last_mono = time.monotonic()

    def predict(self, target_timestamp: float) -> np.ndarray:
        """Predict position at a future timestamp using linear extrapolation.

        Args:
            target_timestamp: Target message timestamp in seconds.

        Returns:
            Predicted ECEF position.
        """
        dt = target_timestamp - self.timestamp
        if self.velocity_ecef is not None and 0 < dt < 60.0:
            return self.ecef + self.velocity_ecef * dt
        return self.ecef.copy()

    def is_physically_consistent(
        self, new_ecef: np.ndarray, new_timestamp: float,
        new_residual_m: float = 0.0,
    ) -> bool:
        """Check if a proposed position is physically reachable.

        Implements nokologo's suggestion: reject solutions that imply
        impossible aircraft movements (e.g., point A to B at speed of light).

        Q7 enhancement: accounts for position uncertainty (residual) from
        both the cached and new positions so that high-residual solves don't
        appear as Mach-3 jumps.

        Args:
            new_ecef: Proposed new position in ECEF.
            new_timestamp: Timestamp of the proposed position.
            new_residual_m: Residual of the new solve (meters), 0 for CPR.

        Returns:
            True if the movement is physically plausible.
        """
        dt = abs(new_timestamp - self.timestamp)
        if dt < 0.001:
            return True  # Same timestamp, can't check

        distance = float(np.linalg.norm(new_ecef - self.ecef))

        # Q7: Subtract position uncertainty (2-sigma) from distance before
        # computing implied speed. Both the cached and new positions have
        # error budgets that inflate apparent distance.
        position_slack = 2.0 * (self.residual_m + new_residual_m)
        effective_distance = max(0.0, distance - position_slack)
        implied_speed = effective_distance / dt

        if implied_speed > MAX_AIRCRAFT_SPEED_MPS:
            return False

        # If we have velocity, also check acceleration
        if self.velocity_ecef is not None and dt > 0.1:
            predicted = self.ecef + self.velocity_ecef * (new_timestamp - self.timestamp)
            prediction_error = float(np.linalg.norm(new_ecef - predicted))
            # Maximum deviation from constant-velocity prediction
            # Using kinematic: d = 0.5 * a * t^2
            max_deviation = 0.5 * MAX_AIRCRAFT_ACCEL_MPS2 * dt * dt
            # Q7: add position uncertainty to the tolerance
            if prediction_error > max(max_deviation + position_slack, 5000.0):
                return False

        return True


class PositionCache:
    """Cache of last-known aircraft positions, keyed by ICAO address.

    Used to provide initial guesses for the solver and enable
    prediction-aided solving for under-determined systems (2-3 sensors).
    """

    def __init__(self) -> None:
        self.cache: dict[str, CachedPosition] = {}
        self.hits = 0
        self.misses = 0

    def get(self, icao: str, target_timestamp: float | None = None) -> CachedPosition | None:
        """Get a cached position for an aircraft.

        Args:
            icao: ICAO hex address.
            target_timestamp: If provided, check staleness against this timestamp.

        Returns:
            CachedPosition or None if not found/stale.
        """
        cached = self.cache.get(icao)
        if cached is None:
            self.misses += 1
            return None

        # Check staleness
        if target_timestamp is not None:
            age = abs(target_timestamp - cached.timestamp)
            if age > MAX_CACHE_AGE:
                self.misses += 1
                return None

        self.hits += 1
        return cached

    def put(
        self,
        icao: str,
        ecef: np.ndarray,
        lat: float,
        lon: float,
        alt_m: float,
        timestamp: float,
        residual_m: float = 0.0,
    ) -> None:
        """Store or update a cached position.

        Also computes velocity estimate from the previous position.

        Args:
            icao: ICAO hex address.
            ecef: Solved position in ECEF.
            lat, lon, alt_m: Solved position in WGS84.
            timestamp: Message timestamp (timestamp_s + timestamp_ns*1e-9).
            residual_m: Solve residual in meters (0 for CPR-seeded positions).
        """
        velocity = None
        prev = self.cache.get(icao)
        if prev is not None:
            dt = timestamp - prev.timestamp
            if 0.1 < dt < 60.0:
                velocity = (ecef - prev.ecef) / dt

        entry = CachedPosition(
            ecef=ecef.copy(),
            lat=lat, lon=lon, alt_m=alt_m,
            timestamp=timestamp,
            velocity_ecef=velocity,
            residual_m=residual_m,
        )
        if prev is not None:
            entry.solve_count = prev.solve_count + 1

        self.cache[icao] = entry

        # Prune if too large
        if len(self.cache) > MAX_CACHE_SIZE:
            self._prune()

    def update_velocity_from_adsb(
        self,
        icao: str,
        ew_mps: float,
        ns_mps: float,
        vrate_mps: float,
        timestamp: float,
    ) -> None:
        """Update cached velocity using ADS-B velocity data (TC 19).

        ADS-B velocity is more accurate than position-derived velocity
        because it comes directly from the aircraft's navigation system.

        Args:
            icao: ICAO hex address.
            ew_mps: East-West velocity in m/s (east positive).
            ns_mps: North-South velocity in m/s (north positive).
            vrate_mps: Vertical rate in m/s (positive = climb).
            timestamp: Message timestamp.
        """
        cached = self.cache.get(icao)
        if cached is None:
            return

        # Convert ENU velocity to ECEF velocity
        # Approximate: at the cached lat/lon, compute rotation
        import math
        lat_r = math.radians(cached.lat)
        lon_r = math.radians(cached.lon)

        sin_lat = math.sin(lat_r)
        cos_lat = math.cos(lat_r)
        sin_lon = math.sin(lon_r)
        cos_lon = math.cos(lon_r)

        # ENU to ECEF rotation:
        # vx = -sin_lon * ve - sin_lat * cos_lon * vn + cos_lat * cos_lon * vu
        # vy =  cos_lon * ve - sin_lat * sin_lon * vn + cos_lat * sin_lon * vu
        # vz =                  cos_lat * vn           + sin_lat * vu
        ve = ew_mps
        vn = ns_mps
        vu = vrate_mps

        vx = -sin_lon * ve - sin_lat * cos_lon * vn + cos_lat * cos_lon * vu
        vy = cos_lon * ve - sin_lat * sin_lon * vn + cos_lat * sin_lon * vu
        vz = cos_lat * vn + sin_lat * vu

        cached.velocity_ecef = np.array([vx, vy, vz], dtype=np.float64)
        cached.timestamp = timestamp
        cached.last_mono = time.monotonic()

    def _prune(self) -> None:
        """Remove the oldest half of entries."""
        entries = sorted(
            self.cache.items(),
            key=lambda x: x[1].last_mono,
        )
        to_remove = len(entries) // 2
        for i in range(to_remove):
            del self.cache[entries[i][0]]

    def stats_dict(self) -> dict:
        """Return cache statistics."""
        return {
            "cached_aircraft": len(self.cache),
            "hits": self.hits,
            "misses": self.misses,
            "hit_rate": (
                f"{self.hits / (self.hits + self.misses) * 100:.1f}%"
                if (self.hits + self.misses) > 0 else "0.0%"
            ),
            "multi_solve": sum(
                1 for c in self.cache.values() if c.solve_count > 1
            ),
        }
