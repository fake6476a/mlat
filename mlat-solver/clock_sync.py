"""Clock synchronization for MLAT solver using ADS-B reference beacons.

Implements pairwise receiver clock offset estimation inspired by
mutability/mlat-server (clocktrack.py, clocksync.py, clocknorm.py).

Core idea: ADS-B DF17 messages contain aircraft positions (CPR-encoded).
When two receivers both see the same Mode-S message, the expected
propagation delay to each receiver can be computed from the known
aircraft position. The difference between measured timestamps minus
expected delays gives the relative clock offset between receivers.

For non-DF17 messages (where aircraft position is unknown), we apply
the learned clock offsets to correct timestamps before solving.

Architecture:
    1. ClockPairing: tracks offset & drift between one pair of receivers
    2. ClockCalibrator: manages all pairings, processes groups, corrects timestamps

References:
    - mutability/mlat-server clocktrack.py, clocksync.py, clocknorm.py
    - Markochev S., Engineering Proceedings 2021, 13(1):12
"""

from __future__ import annotations

import heapq
import sys
import time
from collections import defaultdict

import numpy as np

from atmosphere import C_AIR
from geo import lla_to_ecef

# Minimum sync points before a pairing is considered valid
MIN_SYNC_POINTS = 3

# Maximum age of a sync observation before it's pruned (seconds)
MAX_SYNC_AGE = 300.0

# Outlier rejection threshold (multiples of current stddev)
OUTLIER_SIGMA = 3.5

# R3: Maximum consecutive outlier rejections before resetting the filter.
# Handles clock steps (e.g., GPS lock changes) that would otherwise cause
# permanent rejection. Inspired by mutability's 5-outlier counter.
MAX_CONSECUTIVE_OUTLIERS = 5

# Maximum allowed clock offset magnitude (seconds) — reject if larger
MAX_CLOCK_OFFSET = 0.5

# Kalman filter tuning parameters
# Measurement noise variance (seconds²) — SDR timestamp jitter ~5μs → (5e-6)² = 25e-12
_R = 25e-12
# Drift process noise spectral density (s/s²/Hz)
# With message-timestamp dt (real seconds), this must be small enough that
# Q contribution (q*dt³/3 for p00) stays reasonable over typical dt=1-10s.
# SDR crystal Allan deviation ~1e-9 at τ=1s → PSD ~1e-18
_Q_DRIFT = 1e-18


def _log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


class ClockPairing:
    """Tracks the relative clock offset and drift between two receivers.

    Uses a 2-state Kalman filter with state vector [offset, drift]:
      - offset: clock difference in seconds (t_a_corrected - t_b_corrected)
      - drift: rate of change of offset in seconds/second (PPM * 1e-6)

    The Kalman filter provides:
      - Optimal weighting of observations vs predictions
      - Formal uncertainty via covariance matrix P
      - Natural handling of measurement gaps (covariance grows during gaps)
      - Built-in warm-up detection (high P[0,0] = high uncertainty)

    Replaces the previous PI controller (R4 from MLAT_Accuracy_Analysis.md).
    """

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
        """Add a new sync observation and update the offset/drift estimate.

        Args:
            measured_offset: (t_a - delay_a) - (t_b - delay_b) in seconds.
                A positive value means sensor_b's clock is ahead of sensor_a.
            now: message timestamp (seconds) for dt computation and aging.

        Returns:
            True if the observation was accepted, False if rejected as outlier.
        """
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

        # --- Predict step ---
        # State prediction: offset += drift * dt
        self.offset += self.drift * dt

        # Covariance prediction: P = F P F' + Q
        # F = [[1, dt], [0, 1]]
        # Q = q_drift * [[dt³/3, dt²/2], [dt²/2, dt]]  (constant-velocity model)
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
                # R3: After too many consecutive outliers, the clock has likely
                # stepped. Reset the filter to re-acquire from scratch.
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

        # --- Update step ---
        # Kalman gain: K = P H' / S, where H = [1, 0]
        k0 = self.p00 / S
        k1 = self.p01 / S

        self.offset += k0 * innovation
        self.drift += k1 * innovation

        # Covariance update: Joseph form P = (I-KH)P(I-KH)' + KRK'
        # For H=[1,0], K=[k0,k1]': (I-KH) = [[1-k0, 0], [-k1, 1]]
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
        """Predict the current clock offset at time 'now'.

        Returns:
            Predicted offset in seconds.
        """
        if not self.valid:
            return self.offset
        dt = now - self.last_update
        return self.offset + self.drift * dt


class ClockCalibrator:
    """Manages clock synchronization across all receiver pairs.

    Uses ADS-B DF17 messages with known positions as reference beacons
    to calibrate pairwise clock offsets. Then applies corrections to
    correlation groups before they are passed to the solver.

    This is equivalent to mutability/mlat-server's ClockTracker +
    clocknorm.normalize() pipeline.
    """

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
        """Use a DF17 message with known position as a sync reference.

        For each pair of receivers that saw this message, compute the
        expected propagation delay and derive the clock offset.

        Args:
            aircraft_ecef: Known aircraft position in ECEF [x, y, z].
            receptions: List of reception dicts with sensor_id, lat, lon, alt,
                        timestamp_s, timestamp_ns.
            now: Monotonic time for aging.
        """
        n = len(receptions)
        if n < 2:
            return

        # Compute reception-delay-corrected timestamps for each receiver
        corrected = []
        for rec in receptions:
            sensor_ecef = lla_to_ecef(rec["lat"], rec["lon"], rec["alt"])
            distance = np.linalg.norm(aircraft_ecef - sensor_ecef)
            delay = distance / C_AIR  # propagation delay in seconds
            t_raw = rec["timestamp_s"] + rec["timestamp_ns"] * 1e-9
            t_corrected = t_raw - delay  # should be ≈ transmission time
            corrected.append((rec["sensor_id"], t_corrected))

        # For each pair of receivers, the difference in corrected times
        # is the clock offset
        for i in range(n):
            for j in range(i + 1, n):
                s_a, tc_a = corrected[i]
                s_b, tc_b = corrected[j]
                # offset = tc_a - tc_b
                # if positive, sensor_b's clock is behind sensor_a
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
        """Apply clock corrections using MST-based normalization.

        Builds a graph of sensor pairs weighted by clock offset variance,
        finds the minimum spanning tree (Prim's algorithm), picks the
        best-connected node as reference, and normalizes all timestamps
        through the optimal-variance path. This is equivalent to
        mutability/mlat-server's clocknorm.normalize() pipeline.

        Args:
            receptions: List of reception dicts (will be modified in place).
            now: Monotonic time for predicting current offsets.

        Returns:
            The corrected receptions list (same objects, modified in place).
        """
        if len(receptions) < 2:
            return receptions

        sensors = [r["sensor_id"] for r in receptions]
        n = len(sensors)
        sensor_idx = {s: i for i, s in enumerate(sensors)}

        # Build adjacency: edges weighted by clock offset variance
        # Lower variance = better calibration = preferred path
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
                # offset = clock_{key[0]} - clock_{key[1]}
                # To bring j to i's timebase: add (clock_i - clock_j) to j
                if s_a == key[0]:
                    # sensors[i] == key[0], sensors[j] == key[1]
                    # offset = clock_i - clock_j → add offset to j
                    correction_i_to_j = offset
                else:
                    # sensors[i] == key[1], sensors[j] == key[0]
                    # offset = clock_j - clock_i → add -offset to j
                    correction_i_to_j = -offset
                edges.append((variance, i, j, correction_i_to_j))

        if not edges:
            # No valid pairings — fall back to no correction
            return receptions

        # Prim's MST: start from the node with the most valid edges (best-connected)
        edge_count = [0] * n
        adj: dict[int, list[tuple[float, int, float]]] = {i: [] for i in range(n)}
        for var, i, j, corr in edges:
            adj[i].append((var, j, corr))
            adj[j].append((var, i, -corr))
            edge_count[i] += 1
            edge_count[j] += 1

        # Pick reference node: most edges (best connected)
        ref_node = max(range(n), key=lambda x: edge_count[x])

        # Prim's algorithm to build MST
        visited = set()
        # correction[i] = cumulative correction to apply to sensor i
        # to bring it into ref_node's timebase
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
            # Cumulative correction: ref → ... → src → dst
            correction[dst] = correction[src] + corr
            for next_var, next_node, next_corr in adj[dst]:
                if next_node not in visited:
                    heapq.heappush(heap, (next_var, dst, next_node, next_corr))

        # Apply corrections (ref_node gets 0 correction)
        for i in range(n):
            if i == ref_node or i not in visited:
                continue
            corr = correction[i]
            if abs(corr) > MAX_CLOCK_OFFSET:
                continue  # Safety: skip absurdly large corrections

            correction_ns = int(round(corr * 1e9))
            receptions[i]["timestamp_ns"] += correction_ns

            # Handle overflow/underflow of nanoseconds
            while receptions[i]["timestamp_ns"] >= 1_000_000_000:
                receptions[i]["timestamp_ns"] -= 1_000_000_000
                receptions[i]["timestamp_s"] += 1
            while receptions[i]["timestamp_ns"] < 0:
                receptions[i]["timestamp_ns"] += 1_000_000_000
                receptions[i]["timestamp_s"] -= 1

        self.groups_corrected += 1
        return receptions

    def is_calibrated(self, receptions: list[dict]) -> bool:
        """Check if we have valid clock pairings for all receiver pairs in a group.

        Args:
            receptions: List of reception dicts.

        Returns:
            True if all pairwise offsets are calibrated.
        """
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
