"""Cache per-ICAO positions for prior-aided and prediction-aided MLAT solves."""

from __future__ import annotations

import time

import numpy as np


# Limit cached-position age in seconds.
MAX_CACHE_AGE = 120.0

# Limit the number of cached aircraft.
MAX_CACHE_SIZE = 5000

# Use a generous Mach-3 upper bound to avoid false physical-consistency rejections.
MAX_AIRCRAFT_SPEED_MPS = 1030.0

# Use a generous acceleration bound for turns and climbs.
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
        """Predict the ECEF position at a future timestamp by linear extrapolation."""
        dt = target_timestamp - self.timestamp
        if self.velocity_ecef is not None and 0 < dt < 60.0:
            return self.ecef + self.velocity_ecef * dt
        return self.ecef.copy()

    def is_physically_consistent(
        self, new_ecef: np.ndarray, new_timestamp: float,
        new_residual_m: float = 0.0,
    ) -> bool:
        """Return whether a proposed position is physically reachable from the cached one."""
        dt = abs(new_timestamp - self.timestamp)
        if dt < 0.001:
            return True  # Same timestamp, can't check

        distance = float(np.linalg.norm(new_ecef - self.ecef))

        # Subtract combined position uncertainty before computing implied speed.
        position_slack = 2.0 * (self.residual_m + new_residual_m)
        effective_distance = max(0.0, distance - position_slack)
        implied_speed = effective_distance / dt

        if implied_speed > MAX_AIRCRAFT_SPEED_MPS:
            return False

        # Check acceleration as well when a velocity estimate is available.
        if self.velocity_ecef is not None and dt > 0.1:
            predicted = self.ecef + self.velocity_ecef * (new_timestamp - self.timestamp)
            prediction_error = float(np.linalg.norm(new_ecef - predicted))
            # Use `d = 0.5 * a * t^2` as the maximum constant-acceleration deviation.
            max_deviation = 0.5 * MAX_AIRCRAFT_ACCEL_MPS2 * dt * dt
            # Add position uncertainty to the tolerance.
            if prediction_error > max(max_deviation + position_slack, 5000.0):
                return False

        return True


class PositionCache:
    """Cache last-known aircraft positions keyed by ICAO address."""

    def __init__(self) -> None:
        self.cache: dict[str, CachedPosition] = {}
        self.hits = 0
        self.misses = 0

    def get(self, icao: str, target_timestamp: float | None = None) -> CachedPosition | None:
        """Return a cached aircraft position unless it is missing or stale."""
        cached = self.cache.get(icao)
        if cached is None:
            self.misses += 1
            return None

        # Reject entries that are too old for the target timestamp.
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
        """Store or update a cached aircraft position and its derived velocity."""
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

        # Prune the cache when it grows beyond the configured limit.
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
        """Update cached velocity from ADS-B TC19 velocity data."""
        cached = self.cache.get(icao)
        if cached is None:
            return

        # Convert ENU velocity to ECEF velocity at the cached location.
        import math
        lat_r = math.radians(cached.lat)
        lon_r = math.radians(cached.lon)

        sin_lat = math.sin(lat_r)
        cos_lat = math.cos(lat_r)
        sin_lon = math.sin(lon_r)
        cos_lon = math.cos(lon_r)

        # Rotate ENU velocity components into the ECEF frame.
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
