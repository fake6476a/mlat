"""Estimate pairwise receiver clock offsets from ADS-B references for MLAT."""

from __future__ import annotations

import heapq
import sys
import time
from collections import defaultdict

import numpy as np

from atmosphere import C_AIR
from geo import sensor_lla_to_ecef

# Minimum sync points before a pairing is considered valid
MIN_SYNC_POINTS = 3

# Maximum age of a sync observation before it's pruned (seconds)
MAX_SYNC_AGE = 300.0

# Outlier rejection threshold (multiples of current stddev)
OUTLIER_SIGMA = 3.5

# Reset the filter after too many consecutive outliers so clock steps can re-lock.
MAX_CONSECUTIVE_OUTLIERS = 5

# Maximum allowed clock offset magnitude (seconds) — reject if larger
MAX_CLOCK_OFFSET = 0.5

# Define the Kalman measurement-noise variance in seconds squared.
_R = 25e-12
# Define the drift process-noise spectral density used by the Kalman filter.
_Q_DRIFT = 1e-18


def _log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


class ClockPairing:
    """Track relative clock offset and drift between one receiver pair."""

    __slots__ = (
        "sensor_a", "sensor_b",
        "n",
        "offset", "drift", "variance",
        "p00", "p01", "p11",
        "valid", "last_update",
        "_consecutive_outliers",
    )

    def __init__(self, sensor_a: int, sensor_b: int) -> None:
        self.sensor_a = sensor_a
        self.sensor_b = sensor_b
        self.n = 0
        # State vector: [offset, drift]
        self.offset = 0.0
        self.drift = 0.0
        # Covariance matrix P (symmetric 2x2, stored as 3 elements)
        self.p00 = 2.5e-9     # offset variance — ~50μs initial uncertainty
        self.p01 = 0.0        # offset-drift covariance
        self.p11 = 1e-12      # drift variance — ~1 PPM initial uncertainty
        # Derived
        self.variance = 2.5e-9  # = p00, exposed for compatibility
        self.valid = False
        self.last_update = 0.0
        self._consecutive_outliers = 0

    def update(self, measured_offset: float, now: float) -> bool:
        """Update the offset and drift estimate from one sync observation."""
        dt = now - self.last_update if self.last_update > 0 else 0.0

        if self.n == 0:
            # First observation — initialize state
            self.offset = measured_offset
            self.drift = 0.0
            self.p00 = _R * 100  # High initial uncertainty (~50μs)
            self.p01 = 0.0
            self.p11 = 1e-12     # ~1 PPM drift uncertainty
            self.variance = self.p00
            self.n = 1
            self.last_update = now
            return True

        # Predict the next offset from the current drift.
        self.offset += self.drift * dt

        # Predict covariance with the constant-velocity Kalman model.
        old_p00 = self.p00
        old_p01 = self.p01
        old_p11 = self.p11

        self.p00 = old_p00 + 2.0 * dt * old_p01 + dt * dt * old_p11 + _Q_DRIFT * dt * dt * dt / 3.0
        self.p01 = old_p01 + dt * old_p11 + _Q_DRIFT * dt * dt / 2.0
        self.p11 = old_p11 + _Q_DRIFT * dt

        r = _R

        # --- Outlier rejection ---
        innovation = measured_offset - self.offset
        S = self.p00 + r  # Innovation variance
        if self.valid and self.n >= MIN_SYNC_POINTS:
            if abs(innovation) > OUTLIER_SIGMA * (S ** 0.5):
                self._consecutive_outliers += 1
                # Reset after repeated outliers because the clock has likely stepped.
                if self._consecutive_outliers >= MAX_CONSECUTIVE_OUTLIERS:
                    self.n = 0
                    self.offset = measured_offset
                    self.drift = 0.0
                    self.p00 = _R * 100
                    self.p01 = 0.0
                    self.p11 = 1e-12
                    self.variance = self.p00
                    self.valid = False
                    self.n = 1
                    self.last_update = now
                    self._consecutive_outliers = 0
                    return True
                # Reject but keep the predicted state
                self.variance = self.p00
                self.last_update = now
                return False

        # Update the filter with the scalar observation model `H = [1, 0]`.
        k0 = self.p00 / S
        k1 = self.p01 / S

        self.offset += k0 * innovation
        self.drift += k1 * innovation

        # Apply the Joseph-form covariance update for numerical stability.
        a = 1.0 - k0
        # new_p00 = a² * p00 + k0² * R
        new_p00 = a * a * self.p00 + k0 * k0 * r
        # new_p01 = a * (-k1 * p00 + p01) + k0 * k1 * R
        new_p01 = a * (-k1 * self.p00 + self.p01) + k0 * k1 * r
        # new_p11 = k1² * p00 - 2*k1*p01 + p11 + k1² * R
        new_p11 = k1 * k1 * self.p00 - 2.0 * k1 * self.p01 + self.p11 + k1 * k1 * r

        self.p00 = new_p00
        self.p01 = new_p01
        self.p11 = new_p11

        self.variance = self.p00
        self.n += 1
        self.valid = self.n >= MIN_SYNC_POINTS
        self.last_update = now
        self._consecutive_outliers = 0

        return True

    def predict(self, now: float) -> float:
        """Predict the current clock offset at the requested time."""
        if not self.valid:
            return self.offset
        dt = now - self.last_update
        return self.offset + self.drift * dt


class ClockCalibrator:
    """Manage clock calibration across all receiver pairs in the current network."""

    def __init__(self) -> None:
        # Map of (sensor_a, sensor_b) -> ClockPairing where sensor_a < sensor_b
        self.pairings: dict[tuple[int, int], ClockPairing] = {}

        # Cache of recently seen ADS-B positions: icao -> (ecef, timestamp_mono)
        self.adsb_cache: dict[str, tuple[np.ndarray, float]] = {}

        # Stats
        self.sync_points_total = 0
        self.sync_points_accepted = 0
        self.groups_corrected = 0
        self.calibrated_pairs = 0

    def _get_pairing(self, s_a: int, s_b: int) -> ClockPairing:
        """Get or create a clock pairing for two sensors."""
        key = (min(s_a, s_b), max(s_a, s_b))
        if key not in self.pairings:
            self.pairings[key] = ClockPairing(key[0], key[1])
        return self.pairings[key]

    def process_adsb_reference(
        self,
        aircraft_ecef: np.ndarray,
        receptions: list[dict],
        now: float,
    ) -> None:
        """Use a known ADS-B position to update pairwise receiver clock offsets."""
        n = len(receptions)
        if n < 2:
            return

        # Compute propagation-delay-corrected timestamps for each receiver.
        corrected = []
        for rec in receptions:
            sensor_ecef = sensor_lla_to_ecef(rec["lat"], rec["lon"], rec["alt"])
            distance = np.linalg.norm(aircraft_ecef - sensor_ecef)
            delay = distance / C_AIR  # propagation delay in seconds
            t_raw = rec["timestamp_s"] + rec["timestamp_ns"] * 1e-9
            t_corrected = t_raw - delay  # should be ≈ transmission time
            corrected.append((rec["sensor_id"], t_corrected))

        # Convert corrected timestamp differences into pairwise clock offsets.
        for i in range(n):
            for j in range(i + 1, n):
                s_a, tc_a = corrected[i]
                s_b, tc_b = corrected[j]
                # A positive offset means sensor `b` is behind sensor `a`.
                measured_offset = tc_a - tc_b

                self.sync_points_total += 1

                pairing = self._get_pairing(s_a, s_b)
                # Ensure consistent sign: always compute as (lower_id - higher_id)
                if s_a > s_b:
                    measured_offset = -measured_offset

                if pairing.update(measured_offset, now):
                    self.sync_points_accepted += 1

        # Update calibrated pair count
        self.calibrated_pairs = sum(
            1 for p in self.pairings.values() if p.valid
        )

    def correct_timestamps(
        self,
        receptions: list[dict],
        now: float,
    ) -> list[dict]:
        """Normalize reception timestamps with MST-based clock corrections."""
        if len(receptions) < 2:
            return receptions

        sensors = [r["sensor_id"] for r in receptions]
        n = len(sensors)
        sensor_idx = {s: i for i, s in enumerate(sensors)}

        # Build adjacency edges weighted by clock-offset variance.
        edges: list[tuple[float, int, int, float]] = []  # (variance, i, j, offset)
        for i in range(n):
            for j in range(i + 1, n):
                s_a, s_b = sensors[i], sensors[j]
                key = (min(s_a, s_b), max(s_a, s_b))
                pairing = self.pairings.get(key)
                if pairing is None or not pairing.valid:
                    continue
                offset = pairing.predict(now)
                variance = pairing.variance if pairing.variance > 0 else 1e-20
                # Convert the stored pair offset into a correction from `j` to `i`.
                if s_a == key[0]:
                    # Add the direct offset when `i` is the lower-ID sensor.
                    correction_i_to_j = offset
                else:
                    # Negate the offset when `i` is the higher-ID sensor.
                    correction_i_to_j = -offset
                edges.append((variance, i, j, correction_i_to_j))

        if not edges:
            # Fall back to raw timestamps when no valid pairings exist.
            return receptions

        # Start Prim's algorithm from the best-connected sensor.
        edge_count = [0] * n
        adj: dict[int, list[tuple[float, int, float]]] = {i: [] for i in range(n)}
        for var, i, j, corr in edges:
            adj[i].append((var, j, corr))
            adj[j].append((var, i, -corr))
            edge_count[i] += 1
            edge_count[j] += 1

        # Pick the reference node with the most valid edges.
        ref_node = max(range(n), key=lambda x: edge_count[x])

        # Build the minimum spanning tree with Prim's algorithm.
        visited = set()
        # Store the cumulative correction that brings each sensor into the reference timebase.
        correction = [0.0] * n
        heap: list[tuple[float, int, int, float]] = []
        visited.add(ref_node)

        for var, neighbor, corr in adj[ref_node]:
            heapq.heappush(heap, (var, ref_node, neighbor, corr))

        while heap and len(visited) < n:
            var, src, dst, corr = heapq.heappop(heap)
            if dst in visited:
                continue
            visited.add(dst)
            # Accumulate the correction along the MST path.
            correction[dst] = correction[src] + corr
            for next_var, next_node, next_corr in adj[dst]:
                if next_node not in visited:
                    heapq.heappush(heap, (next_var, dst, next_node, next_corr))

        # Apply corrections while leaving the reference node unchanged.
        for i in range(n):
            if i == ref_node or i not in visited:
                continue
            corr = correction[i]
            if abs(corr) > MAX_CLOCK_OFFSET:
                continue  # Safety: skip absurdly large corrections

            correction_ns = int(round(corr * 1e9))
            receptions[i]["timestamp_ns"] += correction_ns

            # Renormalize nanoseconds after the correction.
            while receptions[i]["timestamp_ns"] >= 1_000_000_000:
                receptions[i]["timestamp_ns"] -= 1_000_000_000
                receptions[i]["timestamp_s"] += 1
            while receptions[i]["timestamp_ns"] < 0:
                receptions[i]["timestamp_ns"] += 1_000_000_000
                receptions[i]["timestamp_s"] -= 1

        self.groups_corrected += 1
        return receptions

    def is_calibrated(self, receptions: list[dict]) -> bool:
        """Return whether every receiver pair in the group has a valid calibration."""
        sensors = [r["sensor_id"] for r in receptions]
        ref = sensors[0]
        for s in sensors[1:]:
            key = (min(ref, s), max(ref, s))
            pairing = self.pairings.get(key)
            if pairing is None or not pairing.valid:
                return False
        return True

    def has_any_calibration(self, receptions: list[dict]) -> bool:
        """Check if we have at least one valid pairing for this group."""
        sensors = [r["sensor_id"] for r in receptions]
        for i in range(len(sensors)):
            for j in range(i + 1, len(sensors)):
                key = (min(sensors[i], sensors[j]), max(sensors[i], sensors[j]))
                pairing = self.pairings.get(key)
                if pairing is not None and pairing.valid:
                    return True
        return False

    def stats_dict(self) -> dict:
        """Return calibration statistics."""
        valid_pairs = [(k, p) for k, p in self.pairings.items() if p.valid]
        return {
            "total_pairings": len(self.pairings),
            "valid_pairings": len(valid_pairs),
            "sync_points_total": self.sync_points_total,
            "sync_points_accepted": self.sync_points_accepted,
            "groups_corrected": self.groups_corrected,
            "pair_details": {
                f"{k[0]}-{k[1]}": {
                    "offset_us": round(p.offset * 1e6, 1),
                    "drift_ppm": round(p.drift * 1e6, 3),
                    "variance_us2": round(p.variance * 1e12, 1),
                    "n": p.n,
                }
                for k, p in valid_pairs[:10]  # limit output
            },
        }
