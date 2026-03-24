"""Implement the Frisch TOA formulation for iterative MLAT refinement."""

from __future__ import annotations

import numpy as np
from numba import njit
from scipy.optimize import least_squares

from atmosphere import C_VACUUM, effective_velocity
from geo import ecef_to_lla, lla_to_ecef


@njit(cache=True)
def _timing_state_core(
    x: np.ndarray,
    sensors: np.ndarray,
    arrival_times: np.ndarray,
    velocities: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    n = sensors.shape[0]
    residuals = np.empty(n, dtype=np.float64)
    ranges = np.empty(n, dtype=np.float64)
    units = np.empty((n, 3), dtype=np.float64)
    t0_sum = 0.0

    for i in range(n):
        dx0 = x[0] - sensors[i, 0]
        dx1 = x[1] - sensors[i, 1]
        dx2 = x[2] - sensors[i, 2]
        r = np.sqrt(dx0 * dx0 + dx1 * dx1 + dx2 * dx2)
        if r < 1e-12:
            units[i, 0] = 0.0
            units[i, 1] = 0.0
            units[i, 2] = 0.0
            r = 1e-12
        else:
            inv_r = 1.0 / r
            units[i, 0] = dx0 * inv_r
            units[i, 1] = dx1 * inv_r
            units[i, 2] = dx2 * inv_r
        ranges[i] = r
        t0_sum += arrival_times[i] - r / velocities[i]

    t0_hat = t0_sum / n

    for i in range(n):
        residuals[i] = (arrival_times[i] - t0_hat) * velocities[i] - ranges[i]

    return residuals, ranges, units, t0_hat


@njit(cache=True)
def _timing_jacobian_core(
    units: np.ndarray,
    velocities: np.ndarray,
) -> np.ndarray:
    n = units.shape[0]
    sum_u_over_v = np.zeros(3, dtype=np.float64)

    for i in range(n):
        inv_v = 1.0 / velocities[i]
        sum_u_over_v[0] += units[i, 0] * inv_v
        sum_u_over_v[1] += units[i, 1] * inv_v
        sum_u_over_v[2] += units[i, 2] * inv_v

    jac = np.empty((n, 3), dtype=np.float64)
    inv_n = 1.0 / n

    for i in range(n):
        scale = velocities[i] * inv_n
        jac[i, 0] = scale * sum_u_over_v[0] - units[i, 0]
        jac[i, 1] = scale * sum_u_over_v[1] - units[i, 1]
        jac[i, 2] = scale * sum_u_over_v[2] - units[i, 2]

    return jac


def _compute_velocities(
    sensor_alts_m: np.ndarray,
    aircraft_alt_m: float | None,
    c: float = C_VACUUM,
) -> np.ndarray:
    if aircraft_alt_m is None:
        return np.full(len(sensor_alts_m), c, dtype=np.float64)

    velocities = np.empty(len(sensor_alts_m), dtype=np.float64)
    for i, h_s in enumerate(sensor_alts_m):
        velocities[i] = effective_velocity(float(h_s), aircraft_alt_m)
    return velocities


def _timing_residuals_from_velocities(
    x: np.ndarray,
    sensors: np.ndarray,
    arrival_times: np.ndarray,
    velocities: np.ndarray,
) -> np.ndarray:
    residuals, _, _, _ = _timing_state_core(x, sensors, arrival_times, velocities)
    return residuals


def _altitude_residual(
    x: np.ndarray,
    aircraft_alt_m: float,
) -> float:
    _, _, current_alt = ecef_to_lla(x[0], x[1], x[2])
    return (current_alt - aircraft_alt_m) * 10.0


def _altitude_residual_and_gradient(
    x: np.ndarray,
    aircraft_alt_m: float,
) -> tuple[float, np.ndarray]:
    alt_residual = _altitude_residual(x, aircraft_alt_m)
    grad = np.empty(3, dtype=np.float64)
    step_scale = np.sqrt(np.finfo(np.float64).eps)

    for axis in range(3):
        step = step_scale * max(1.0, abs(float(x[axis])))
        x_step = x.copy()
        x_step[axis] += step
        grad[axis] = (_altitude_residual(x_step, aircraft_alt_m) - alt_residual) / step

    return alt_residual, grad


def _build_objective_residuals(
    timing_residuals: np.ndarray,
    x: np.ndarray,
    aircraft_alt_m: float | None,
    track_prediction_ecef: np.ndarray | None,
    prediction_weight: float,
    altitude_residual: float | None = None,
) -> np.ndarray:
    extra = 0
    if aircraft_alt_m is not None:
        extra += 1
    if track_prediction_ecef is not None:
        extra += 3
    if extra == 0:
        return timing_residuals

    residuals = np.empty(len(timing_residuals) + extra, dtype=np.float64)
    residuals[:len(timing_residuals)] = timing_residuals
    idx = len(timing_residuals)

    if aircraft_alt_m is not None:
        if altitude_residual is None:
            altitude_residual = _altitude_residual(x, aircraft_alt_m)
        residuals[idx] = altitude_residual
        idx += 1

    if track_prediction_ecef is not None:
        residuals[idx:idx + 3] = (x - track_prediction_ecef) * prediction_weight

    return residuals


def _timing_residuals(
    x: np.ndarray,
    sensors: np.ndarray,
    arrival_times: np.ndarray,
    sensor_alts_m: np.ndarray,
    aircraft_alt_m: float | None,
    c: float = C_VACUUM,
) -> np.ndarray:
    velocities = _compute_velocities(sensor_alts_m, aircraft_alt_m, c)
    return _timing_residuals_from_velocities(x, sensors, arrival_times, velocities)


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
    """Compute Frisch TOA residuals with analytical elimination of transmission time."""
    velocities = _compute_velocities(sensor_alts_m, aircraft_alt_m, c)
    timing_residuals = _timing_residuals_from_velocities(
        x,
        sensors,
        arrival_times,
        velocities,
    )
    return _build_objective_residuals(
        timing_residuals,
        x,
        aircraft_alt_m,
        track_prediction_ecef,
        prediction_weight,
    )


class _FrischEvaluator:
    __slots__ = (
        "sensors",
        "arrival_times",
        "aircraft_alt_m",
        "track_prediction_ecef",
        "prediction_weight",
        "velocities",
        "_last_x",
        "_last_timing_residuals",
        "_last_ranges",
        "_last_units",
        "_last_t0_hat",
        "_last_altitude_residual",
    )

    def __init__(
        self,
        sensors: np.ndarray,
        arrival_times: np.ndarray,
        sensor_alts_m: np.ndarray,
        aircraft_alt_m: float | None,
        track_prediction_ecef: np.ndarray | None,
        c: float,
        prediction_weight: float,
    ) -> None:
        self.sensors = sensors
        self.arrival_times = arrival_times
        self.aircraft_alt_m = aircraft_alt_m
        self.track_prediction_ecef = track_prediction_ecef
        self.prediction_weight = prediction_weight
        self.velocities = _compute_velocities(sensor_alts_m, aircraft_alt_m, c)
        self._last_x: np.ndarray | None = None
        self._last_timing_residuals: np.ndarray | None = None
        self._last_ranges: np.ndarray | None = None
        self._last_units: np.ndarray | None = None
        self._last_t0_hat = 0.0
        self._last_altitude_residual: float | None = None

    def _ensure_state(self, x: np.ndarray) -> None:
        if self._last_x is not None and np.array_equal(x, self._last_x):
            return

        residuals, ranges, units, t0_hat = _timing_state_core(
            x,
            self.sensors,
            self.arrival_times,
            self.velocities,
        )
        self._last_x = x.copy()
        self._last_timing_residuals = residuals
        self._last_ranges = ranges
        self._last_units = units
        self._last_t0_hat = t0_hat
        self._last_altitude_residual = None

    def residual(self, x: np.ndarray) -> np.ndarray:
        self._ensure_state(x)

        altitude_residual = None
        if self.aircraft_alt_m is not None:
            if self._last_altitude_residual is None:
                self._last_altitude_residual = _altitude_residual(self._last_x, self.aircraft_alt_m)
            altitude_residual = self._last_altitude_residual

        return _build_objective_residuals(
            self._last_timing_residuals,
            self._last_x,
            self.aircraft_alt_m,
            self.track_prediction_ecef,
            self.prediction_weight,
            altitude_residual,
        )

    def jacobian(self, x: np.ndarray) -> np.ndarray:
        self._ensure_state(x)
        timing_jac = _timing_jacobian_core(self._last_units, self.velocities)

        extra = 0
        if self.aircraft_alt_m is not None:
            extra += 1
        if self.track_prediction_ecef is not None:
            extra += 3
        if extra == 0:
            return timing_jac

        jac = np.empty((timing_jac.shape[0] + extra, 3), dtype=np.float64)
        jac[:timing_jac.shape[0], :] = timing_jac
        idx = timing_jac.shape[0]

        if self.aircraft_alt_m is not None:
            alt_residual, alt_grad = _altitude_residual_and_gradient(self._last_x, self.aircraft_alt_m)
            self._last_altitude_residual = alt_residual
            jac[idx, :] = alt_grad
            idx += 1

        if self.track_prediction_ecef is not None:
            jac[idx:idx + 3, :] = 0.0
            jac[idx, 0] = self.prediction_weight
            jac[idx + 1, 1] = self.prediction_weight
            jac[idx + 2, 2] = self.prediction_weight

        return jac

    def timing_residuals(self, x: np.ndarray) -> np.ndarray:
        self._ensure_state(x)
        return self._last_timing_residuals.copy()

    def objective_residuals(self, x: np.ndarray) -> np.ndarray:
        return self.residual(x)

    def t0_s(self, x: np.ndarray) -> float:
        self._ensure_state(x)
        return float(self._last_t0_hat)


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
    """Solve an MLAT position iteratively with the Frisch TOA formulation."""
    try:
        sensors = np.asarray(sensors, dtype=np.float64)
        arrival_times = np.asarray(arrival_times, dtype=np.float64)
        sensor_alts_m = np.asarray(sensor_alts_m, dtype=np.float64)
        x0 = np.asarray(x0, dtype=np.float64)
        if track_prediction_ecef is not None:
            track_prediction_ecef = np.asarray(track_prediction_ecef, dtype=np.float64)

        evaluator = _FrischEvaluator(
            sensors=sensors,
            arrival_times=arrival_times,
            sensor_alts_m=sensor_alts_m,
            aircraft_alt_m=altitude_m,
            track_prediction_ecef=track_prediction_ecef,
            c=c,
            prediction_weight=prediction_weight,
        )

        # Full 3D solve in ECEF coordinates
        result = least_squares(
            evaluator.residual,
            x0,
            jac=evaluator.jacobian,
            method="trf",
            loss="soft_l1",
            f_scale=500.0,  # Moderate outlier transition — balances convergence from far init vs accuracy
            max_nfev=max_nfev,
        )

        if not result.success and result.status <= 0:
            return None

        position = result.x.copy()

        # Reproject the solution onto the known altitude in the usual 2.5D MLAT way.
        if altitude_m is not None:
            lat, lon, _ = ecef_to_lla(position[0], position[1], position[2])
            position = lla_to_ecef(lat, lon, altitude_m)

        # Compute final residual (RMS of all residual terms including constraints)
        final_residuals = evaluator.objective_residuals(position)
        timing_residuals = evaluator.timing_residuals(position)
        residual_m = float(np.sqrt(np.mean(timing_residuals ** 2)))
        objective_residual_m = float(np.sqrt(np.mean(final_residuals ** 2)))

        # Compute estimated transmission time
        t0_s = evaluator.t0_s(position)

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
    """Solve the exactly determined 3-sensor-plus-altitude MLAT case."""
    return solve_toa(
        sensors=sensors,
        arrival_times=arrival_times,
        sensor_alts_m=sensor_alts_m,
        x0=x0,
        altitude_m=altitude_m,
        c=c,
        max_nfev=50,
    )
