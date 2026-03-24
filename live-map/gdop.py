from __future__ import annotations

import numpy as np


def compute_gdop(aircraft_pos: np.ndarray, sensor_positions: np.ndarray) -> float:
    n_sensors = len(sensor_positions)
    if n_sensors < 3:
        return float("inf")

    h_rows = []
    for sensor in sensor_positions:
        r = np.linalg.norm(aircraft_pos - sensor)
        if r < 1.0:
            return float("inf")
        h_rows.append((aircraft_pos - sensor) / r)

    h = np.array(h_rows)
    h = np.column_stack([h, np.ones(n_sensors)])

    try:
        cov = np.linalg.inv(h.T @ h)
        trace_val = np.trace(cov)
        if trace_val < 0 or not np.isfinite(trace_val):
            return float("inf")
        return float(np.sqrt(trace_val))
    except np.linalg.LinAlgError:
        return float("inf")
