"""Implement the constant-velocity EKF used for aircraft track building."""

from __future__ import annotations

import numpy as np


# Use the 99.7% chi-squared gate for 3-DOF innovation rejection.
CHI2_GATE_3DOF = 14.16

# Reset or inflate the filter after long measurement gaps.
MAX_PREDICT_GAP_S = 60.0

# Use a 5 m/s² process-noise acceleration scale for typical aircraft motion.
PROCESS_NOISE_ACCEL = 5.0

# Keep the default measurement-noise scale at 200 m.
MEASUREMENT_NOISE_M = 200.0


class AircraftEKF:
    """Track one aircraft with a constant-velocity ECEF Kalman filter."""

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
        """Initialize the filter from the first position measurement."""
        # Store the `[x, y, z, vx, vy, vz]` state vector.
        self.x = np.zeros(6)
        self.x[:3] = position

        # Start with large velocity uncertainty.
        self.P = np.eye(6)
        self.P[0, 0] = self.P[1, 1] = self.P[2, 2] = meas_noise ** 2
        self.P[3, 3] = self.P[4, 4] = self.P[5, 5] = 1e6  # unknown velocity

        self.last_timestamp_s = timestamp_s
        self.updates = 1
        self.innovations: list[float] = []

        self._process_accel = process_accel
        self._meas_noise = meas_noise

    def predict(self, timestamp_s: float) -> np.ndarray:
        """Predict the state forward to the requested timestamp."""
        dt = timestamp_s - self.last_timestamp_s
        if dt <= 0:
            return self.x[:3].copy()

        if dt > MAX_PREDICT_GAP_S:
            # Inflate covariance instead of trusting a stale prediction gap.
            self.P += np.eye(6) * 1e6
            self.last_timestamp_s = timestamp_s
            return self.x[:3].copy()

        # Build the constant-velocity state-transition matrix.
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt

        # Build the discrete white-noise acceleration covariance.
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

        # Predict the next state and covariance.
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.last_timestamp_s = timestamp_s

        return self.x[:3].copy()

    def update(self, measurement: np.ndarray, timestamp_s: float,
               measurement_noise_m: float | None = None) -> float:
        """Update the filter with a position measurement and innovation gate."""
        # Predict to the measurement time first.
        self.predict(timestamp_s)

        # Observe position directly with the linear measurement matrix.
        H = np.zeros((3, 6))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0

        # Build the measurement-noise covariance.
        noise = measurement_noise_m if measurement_noise_m is not None else self._meas_noise
        R = np.eye(3) * (noise ** 2)

        # Compute the innovation residual.
        y = measurement - H @ self.x

        # Compute the innovation covariance.
        S = H @ self.P @ H.T + R

        # Compute the Mahalanobis distance for innovation gating.
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return -1.0

        mahalanobis_sq = float(y.T @ S_inv @ y)

        # Reject innovations that exceed the chi-squared gate.
        if mahalanobis_sq > CHI2_GATE_3DOF:
            return -1.0

        mahalanobis = float(np.sqrt(mahalanobis_sq))

        # Compute the Kalman gain.
        K = self.P @ H.T @ S_inv

        # Apply the state update.
        self.x = self.x + K @ y

        # Update covariance with the Joseph-stabilized form.
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
