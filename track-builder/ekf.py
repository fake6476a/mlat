"""Extended Kalman Filter for aircraft track building.

Implements a constant-velocity EKF with state [x, y, z, vx, vy, vz]
for continuous flight track association and filtering.

The EKF maintains per-aircraft state estimates and covariances,
predicting forward between measurements and updating with new
MLAT position fixes from Layer 4.

Innovation gating (Mahalanobis distance) rejects outlier measurements
that are inconsistent with the predicted track state.

References:
  - MLAT_Verified_Combined_Reference.md Part 22 (Custom EKF)
  - MLAT_Verified_Combined_Reference.md Part 21 (Innovation gating)
  - MLAT_Verified_Combined_Reference.md Part 4.2 (2-sensor semi-MLAT)
"""

from __future__ import annotations

import numpy as np


# Chi-squared threshold for 3-DOF at 99.7% confidence (3-sigma gate)
# scipy.stats.chi2.ppf(0.997, df=3) ≈ 14.16
CHI2_GATE_3DOF = 14.16

# Maximum time gap (seconds) before resetting the filter
MAX_PREDICT_GAP_S = 60.0

# Process noise acceleration standard deviation (m/s^2)
# Aircraft typically have accelerations < 5 m/s^2 in level flight
PROCESS_NOISE_ACCEL = 5.0

# Measurement noise standard deviation (meters)
# Based on typical MLAT residuals (median ~145m from reference stats)
MEASUREMENT_NOISE_M = 200.0


class AircraftEKF:
    """Extended Kalman Filter for a single aircraft track.

    State vector: [x, y, z, vx, vy, vz] in ECEF meters.
    Measurement: [x, y, z] position from MLAT solver.

    Uses a constant-velocity motion model with configurable
    process noise and measurement noise.
    """

    __slots__ = (
        "x", "P", "last_timestamp_s", "updates", "innovations",
        "_process_accel", "_meas_noise",
    )

    def __init__(
        self,
        position: np.ndarray,
        timestamp_s: float,
        process_accel: float = PROCESS_NOISE_ACCEL,
        meas_noise: float = MEASUREMENT_NOISE_M,
    ) -> None:
        """Initialize EKF with first position measurement.

        Args:
            position: Initial [x, y, z] in ECEF meters.
            timestamp_s: Measurement timestamp in seconds.
            process_accel: Process noise acceleration (m/s^2).
            meas_noise: Measurement noise std dev (meters).
        """
        # State: [x, y, z, vx, vy, vz]
        self.x = np.zeros(6)
        self.x[:3] = position

        # Covariance: large initial uncertainty for velocity
        self.P = np.eye(6)
        self.P[0, 0] = self.P[1, 1] = self.P[2, 2] = meas_noise ** 2
        self.P[3, 3] = self.P[4, 4] = self.P[5, 5] = 1e6  # unknown velocity

        self.last_timestamp_s = timestamp_s
        self.updates = 1
        self.innovations: list[float] = []

        self._process_accel = process_accel
        self._meas_noise = meas_noise

    def predict(self, timestamp_s: float) -> np.ndarray:
        """Predict state forward to the given timestamp.

        Uses constant-velocity model: x_new = x + v * dt.

        Args:
            timestamp_s: Target time in seconds.

        Returns:
            Predicted position [x, y, z].
        """
        dt = timestamp_s - self.last_timestamp_s
        if dt <= 0:
            return self.x[:3].copy()

        if dt > MAX_PREDICT_GAP_S:
            # Gap too large; keep state but inflate covariance
            self.P += np.eye(6) * 1e6
            self.last_timestamp_s = timestamp_s
            return self.x[:3].copy()

        # State transition matrix F (constant velocity)
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt

        # Process noise Q using discrete white noise acceleration model
        q = self._process_accel ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt / 2.0
        dt4 = dt2 * dt2 / 4.0

        Q = np.zeros((6, 6))
        for i in range(3):
            Q[i, i] = dt4 * q       # position variance
            Q[i, i + 3] = dt3 * q   # position-velocity covariance
            Q[i + 3, i] = dt3 * q   # velocity-position covariance
            Q[i + 3, i + 3] = dt2 * q  # velocity variance

        # Predict
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.last_timestamp_s = timestamp_s

        return self.x[:3].copy()

    def update(self, measurement: np.ndarray, timestamp_s: float) -> float:
        """Update state with a new position measurement.

        Performs innovation gating (Mahalanobis distance check) and
        rejects measurements that are too far from the prediction.

        Args:
            measurement: Position [x, y, z] in ECEF meters.
            timestamp_s: Measurement timestamp in seconds.

        Returns:
            Mahalanobis distance of the innovation. Returns -1.0 if
            the measurement was rejected by the gate.
        """
        # First predict to the measurement time
        self.predict(timestamp_s)

        # Observation matrix H: we observe position only
        H = np.zeros((3, 6))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0

        # Measurement noise covariance R
        R = np.eye(3) * (self._meas_noise ** 2)

        # Innovation (residual)
        y = measurement - H @ self.x

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Mahalanobis distance for innovation gating (Part 21, Approach 3)
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return -1.0

        mahalanobis_sq = float(y.T @ S_inv @ y)

        # Gate check: reject if innovation is too large
        if mahalanobis_sq > CHI2_GATE_3DOF:
            return -1.0

        mahalanobis = float(np.sqrt(mahalanobis_sq))

        # Kalman gain
        K = self.P @ H.T @ S_inv

        # State update
        self.x = self.x + K @ y

        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        self.updates += 1
        self.innovations.append(mahalanobis)

        return mahalanobis

    @property
    def position(self) -> np.ndarray:
        """Current estimated position [x, y, z] in ECEF."""
        return self.x[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        """Current estimated velocity [vx, vy, vz] in ECEF m/s."""
        return self.x[3:].copy()

    @property
    def speed_mps(self) -> float:
        """Current estimated speed in m/s."""
        return float(np.linalg.norm(self.x[3:]))

    @property
    def median_innovation(self) -> float:
        """Median Mahalanobis distance of accepted innovations."""
        if not self.innovations:
            return 0.0
        sorted_inn = sorted(self.innovations)
        return sorted_inn[len(sorted_inn) // 2]
