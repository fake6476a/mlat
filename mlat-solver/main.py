#!/usr/bin/env python3
"""Layer 4: MLAT Solver (Enhanced)

Reads Layer 3 JSONL correlation groups from stdin, solves each group
to compute aircraft position using TDOA/TOA multilateration, and
outputs position fixes as JSONL to stdout.

Enhanced pipeline (implements LAYER4_IMPROVEMENT_PLAN.md):

0. Clock Calibration (Phase 1):
   - DF17 messages with ADS-B positions used as reference beacons
   - Pairwise receiver clock offsets tracked via PI controller
   - All timestamps corrected before solving

1. Route by sensor count:
   - 5+ sensors → Inamdar exact algebraic (no iteration)
   - 4 sensors (with or without altitude) → Frisch TOA (Phase 2)
   - 3 sensors + altitude → Constrained TDOA
   - 2 sensors + altitude + cached position → Prediction-aided (Phase 4)
   - 0-1 sensors → Cannot solve

2. Iterative refinement:
   - Frisch TOA formulation with scipy.optimize.least_squares
   - loss='soft_l1' for outlier robustness
   - Atmospheric refraction velocity (Markochev model)
   - Position cache provides better initial guesses (Phase 3)

3. Validation:
   - Position within MAX_RANGE of all sensors
   - GDOP computation → reject if > threshold
   - Residual check

Usage:
    Full pipeline (Layer 1 → 2 → 3 → 4):
    ./data-pipe/mlat-pipe | python3 modes-decoder/main.py | \\
        python3 correlation-engine/main.py | python3 mlat-solver/main.py

    Or replay from saved Layer 3 output:
    cat correlated_data.jsonl | python3 mlat-solver/main.py

All logs go to stderr to keep stdout as a clean data stream.
"""

import copy
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from adsb_decoder import (
    CPRBuffer,
    extract_df17_position_fields,
    extract_df17_velocity,
    position_to_ecef,
)
from clock_sync import ClockCalibrator
from geo import ft_to_m, lla_to_ecef
from position_cache import PositionCache
from solver import solve_group

C_VACUUM = 299792458.0
STATS_INTERVAL = 30  # seconds


# Stats counters
class Stats:
    """Solver statistics counters."""

    def __init__(self) -> None:
        self.groups_received = 0
        self.groups_solved = 0
        self.groups_failed = 0
        self.groups_skipped_sensors = 0
        self.parse_errors = 0
        # Per-method counters
        self.method_counts: dict[str, int] = {}
        # Q2: Failure reason histogram
        self.failure_reasons: dict[str, int] = {}
        # Residual tracking
        self.residuals: list[float] = []
        # Clock sync validation (rolling 100 samples per sensor)
        self.sensor_residuals_m: dict[int, list[float]] = {}

    def record_solve(self, method: str, residual_m: float) -> None:
        self.groups_solved += 1
        self.method_counts[method] = self.method_counts.get(method, 0) + 1
        self.residuals.append(residual_m)

    def record_sensor_residual(self, sensor_id: int, residual_m: float) -> None:
        if sensor_id not in self.sensor_residuals_m:
            self.sensor_residuals_m[sensor_id] = []
        self.sensor_residuals_m[sensor_id].append(residual_m)
        if len(self.sensor_residuals_m[sensor_id]) > 100:
            self.sensor_residuals_m[sensor_id].pop(0)

    def to_dict(self) -> dict:
        result: dict = {
            "groups_received": self.groups_received,
            "groups_solved": self.groups_solved,
            "groups_failed": self.groups_failed,
            "groups_skipped_sensors": self.groups_skipped_sensors,
            "parse_errors": self.parse_errors,
            "solve_rate": (
                f"{self.groups_solved / self.groups_received * 100:.1f}%"
                if self.groups_received > 0
                else "0.0%"
            ),
            "methods": self.method_counts,
            "failure_reasons": self.failure_reasons,
        }
        if self.residuals:
            sorted_r = sorted(self.residuals)
            n = len(sorted_r)
            result["median_residual_m"] = round(sorted_r[n // 2], 2)
            result["p95_residual_m"] = round(
                sorted_r[int(n * 0.95)] if n > 20 else sorted_r[-1], 2
            )
        if self.sensor_residuals_m:
            # Report the mean residual bias per sensor as a clock sync indicator
            result["clock_sync_bias_m"] = {
                str(sid): round(sum(res) / len(res), 2)
                for sid, res in self.sensor_residuals_m.items()
                if len(res) >= 10
            }
        return result


def log(msg: str) -> None:
    """Log to stderr (keeps stdout clean for data)."""
    print(msg, file=sys.stderr, flush=True)


# =============================================================
# Location Override System
# =============================================================
# The Neuron sellers report slightly-off positions (privacy).
# The challenge provides location-overrides.txt with corrected
# lat/lon/alt for each sensor. We match sensor_id → override
# by geographic proximity and replace positions before solving.

_OVERRIDE_MATCH_THRESHOLD_DEG = 0.5  # ~55km — generous to catch offset sensors

def _load_location_overrides() -> list[dict]:
    """Load location overrides from the JSON file."""
    # Try multiple paths
    for path in [
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data-pipe", "location-overrides.txt"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "location-override(2).json"),
    ]:
        path = os.path.normpath(path)
        if os.path.exists(path):
            with open(path) as f:
                overrides = json.load(f)
            log(f"Loaded {len(overrides)} location overrides from {path}")
            return overrides
    log("WARNING: No location-overrides file found!")
    return []


class SensorOverrideMap:
    """Maps sensor_id (int64) to corrected positions from location-overrides."""

    def __init__(self, overrides: list[dict]) -> None:
        self._overrides = overrides
        self._sensor_map: dict[int, dict] = {}  # sensor_id → override entry
        self._rejected: set[int] = set()  # sensor_ids confirmed non-Cornwall
        self.matched_count = 0

    def lookup(self, sensor_id: int, stream_lat: float, stream_lon: float) -> dict | None:
        """Return the override entry for this sensor_id, or None if non-Cornwall."""
        if sensor_id in self._sensor_map:
            return self._sensor_map[sensor_id]
        if sensor_id in self._rejected:
            return None

        # Try to match by position proximity
        best = None
        best_dist = _OVERRIDE_MATCH_THRESHOLD_DEG
        for o in self._overrides:
            dist = ((stream_lat - o["lat"]) ** 2 + (stream_lon - o["lon"]) ** 2) ** 0.5
            if dist < best_dist:
                best_dist = dist
                best = o
        if best is not None:
            self._sensor_map[sensor_id] = best
            self.matched_count += 1
            log(f"Matched sensor {sensor_id} → {best['name']} (dist={best_dist:.6f}°)")
            return best
        else:
            self._rejected.add(sensor_id)
            return None

    def is_cornwall(self, sensor_id: int) -> bool:
        return sensor_id in self._sensor_map

    def stats_dict(self) -> dict:
        return {
            "matched_sensors": self.matched_count,
            "rejected_sensors": len(self._rejected),
            "sensor_names": {str(sid): o["name"] for sid, o in self._sensor_map.items()},
        }


def _apply_overrides(receptions: list[dict], override_map: SensorOverrideMap) -> list[dict]:
    """Apply location overrides to receptions and filter non-Cornwall sensors.

    Returns only receptions from known Cornwall sensors with corrected positions.
    """
    corrected = []
    for rec in receptions:
        sid = rec.get("sensor_id")
        if sid is None:
            continue
        override = override_map.lookup(sid, rec.get("lat", 0), rec.get("lon", 0))
        if override is not None:
            rec = dict(rec)  # shallow copy to avoid mutating original
            rec["lat"] = override["lat"]
            rec["lon"] = override["lon"]
            rec["alt"] = override["alt"]
            corrected.append(rec)
    return corrected


def _extract_df17_altitude(raw_msg: str) -> int | None:
    """Extract altitude from a DF17 TC 9-18 airborne position message.

    The ME field carries a 12-bit altitude code that the original decoder
    misses (it only extracts altitude from DF0/4/16/20 via pms.altcode).
    This function extracts it directly from the raw hex bytes.

    Args:
        raw_msg: 28-char hex DF17 message.

    Returns:
        Altitude in feet, or None.
    """
    frame = extract_df17_position_fields(raw_msg)
    if frame is not None:
        return frame.alt_ft
    return None


def _process_group(
    group: dict,
    clock_cal: ClockCalibrator,
    pos_cache: PositionCache,
    cpr_buffer: CPRBuffer,
    stats: Stats,
    counters: dict,
    now: float,
    override_map: SensorOverrideMap | None = None,
) -> None:
    """Process a single correlation group through the full pipeline."""
    receptions = group.get("receptions", [])

    # Apply location overrides and filter to Cornwall-only sensors
    if override_map is not None:
        receptions = _apply_overrides(receptions, override_map)
        if receptions:
            group = dict(group)
            group["receptions"] = receptions

    n_sensors = len(receptions)
    if n_sensors < 2:
        stats.groups_skipped_sensors += 1
        return

    icao = group.get("icao", "")
    raw_msg = group.get("raw_msg", "")
    msg_timestamp = None
    if receptions:
        msg_timestamp = (
            receptions[0]["timestamp_s"]
            + receptions[0]["timestamp_ns"] * 1e-9
        )

    # =============================================================
    # Phase 5: ADS-B CPR Decode — free cache seeding
    # =============================================================
    if group.get("df_type") == 17 and len(raw_msg) == 28:
        cpr_frame = extract_df17_position_fields(raw_msg)
        if cpr_frame is not None and msg_timestamp is not None:
            cpr_frame.timestamp = msg_timestamp
            decoded = cpr_buffer.add_frame(icao, cpr_frame)
            if decoded is not None:
                alt_m = ft_to_m(decoded.alt_ft) if decoded.alt_ft is not None else None
                if alt_m is None:
                    cached_entry = pos_cache.get(icao, msg_timestamp)
                    if cached_entry is not None:
                        alt_m = cached_entry.alt_m

                if alt_m is not None:
                    decoded_ecef = position_to_ecef(
                        decoded.lat, decoded.lon, decoded.alt_ft
                    )
                else:
                    decoded_ecef = position_to_ecef(
                        decoded.lat, decoded.lon, None
                    )
                    alt_m = 10000.0

                pos_cache.put(
                    icao=icao, ecef=decoded_ecef,
                    lat=decoded.lat, lon=decoded.lon,
                    alt_m=alt_m, timestamp=msg_timestamp,
                )
                counters["cpr_seeds"] += 1

                clock_cal.process_adsb_reference(
                    aircraft_ecef=decoded_ecef,
                    receptions=receptions, now=now,
                )

    # Phase 5b: ADS-B Velocity Decode (TC 19)
    if group.get("df_type") == 17 and len(raw_msg) == 28 and msg_timestamp is not None:
        vel = extract_df17_velocity(raw_msg)
        if vel is not None:
            KTS_TO_MPS = 0.514444
            pos_cache.update_velocity_from_adsb(
                icao=icao,
                ew_mps=vel.ew_knots * KTS_TO_MPS,
                ns_mps=vel.ns_knots * KTS_TO_MPS,
                vrate_mps=vel.vrate_fpm * 0.00508,
                timestamp=msg_timestamp,
            )

    # =============================================================
    # Phase 6: Altitude augmentation
    # =============================================================
    effective_alt_ft = group.get("altitude_ft")

    if effective_alt_ft is None and group.get("df_type") == 17:
        df17_alt = _extract_df17_altitude(raw_msg)
        if df17_alt is not None:
            effective_alt_ft = df17_alt
            counters["df17_alt_extracted"] += 1

    if effective_alt_ft is None and msg_timestamp is not None:
        cached_for_alt = pos_cache.get(icao, msg_timestamp)
        if cached_for_alt is not None and cached_for_alt.alt_m is not None:
            effective_alt_ft = round(cached_for_alt.alt_m / 0.3048)
            counters["cached_alt_used"] += 1

    working_group = group
    if effective_alt_ft != group.get("altitude_ft"):
        working_group = dict(group)
        working_group["altitude_ft"] = effective_alt_ft

    # =============================================================
    # Phase 1: Clock Calibration — apply corrections
    # =============================================================
    corrected_receptions = copy.deepcopy(receptions)
    if clock_cal.has_any_calibration(corrected_receptions):
        clock_cal.correct_timestamps(corrected_receptions, now)
        corrected_group = dict(working_group)
        corrected_group["receptions"] = corrected_receptions
    else:
        corrected_group = working_group

    # =============================================================
    # Phase 3: Position cache lookup
    # =============================================================
    cached = pos_cache.get(icao, msg_timestamp)
    position_prior = cached.predict(msg_timestamp) if (cached and msg_timestamp) else None

    # =============================================================
    # Phase 4: 2-sensor gate — require altitude + cached prior
    # =============================================================
    MAX_PAIR_VARIANCE_US2 = 50.0
    if n_sensors == 2 and effective_alt_ft is not None:
        if position_prior is None:
            output = {"unsolved_group": group}
            print(json.dumps(output, separators=(",", ":")), flush=True)
            stats.groups_skipped_sensors += 1
            stats.failure_reasons["no_prior_2sensor"] = stats.failure_reasons.get("no_prior_2sensor", 0) + 1
            return

        if len(corrected_receptions) == 2:
            s_a = corrected_receptions[0].get("sensor_id")
            s_b = corrected_receptions[1].get("sensor_id")
            if s_a is not None and s_b is not None:
                pair_key = (min(s_a, s_b), max(s_a, s_b))
                pairing = clock_cal.pairings.get(pair_key)
                if pairing is not None and pairing.valid:
                    pair_var_us2 = pairing.variance * 1e12
                    if pair_var_us2 > MAX_PAIR_VARIANCE_US2:
                        output = {"unsolved_group": group}
                        print(json.dumps(output, separators=(",", ":")), flush=True)
                        stats.groups_skipped_sensors += 1
                        stats.failure_reasons["pair_variance_exceeded"] = stats.failure_reasons.get("pair_variance_exceeded", 0) + 1
                        return

    # =============================================================
    # Solve the correlation group
    # =============================================================
    result, fail_reason = solve_group(corrected_group, position_prior_ecef=position_prior)

    if result is not None:
        aircraft_ecef = lla_to_ecef(
            result.lat, result.lon, result.alt_ft * 0.3048
        )
        if cached is not None and msg_timestamp is not None:
            if not cached.is_physically_consistent(
                aircraft_ecef, msg_timestamp,
                new_residual_m=result.residual_m,
            ):
                stats.groups_failed += 1
                stats.failure_reasons["physically_inconsistent"] = stats.failure_reasons.get("physically_inconsistent", 0) + 1
                return

        output = result.to_dict()
        print(json.dumps(output, separators=(",", ":")), flush=True)
        stats.record_solve(result.solve_method, result.residual_m)
        if msg_timestamp is not None:
            pos_cache.put(
                icao=icao, ecef=aircraft_ecef,
                lat=result.lat, lon=result.lon,
                alt_m=result.alt_ft * 0.3048,
                timestamp=msg_timestamp,
                residual_m=result.residual_m,
            )

        # Q9: Only feed solver positions back to clock cal if low residual.
        # Prevents high-residual solves from contaminating calibration.
        # CPR-decoded positions (line 210) remain the primary trusted source.
        # Sweep showed <200m is optimal: recovers sync points while filtering junk.
        # R10: Additionally require that at least one clock pair in this group
        # is already converged. This prevents early noisy solves from corrupting
        # freshly initializing pairs — only calibrated pairs get refinement.
        if result.residual_m < 200.0 and clock_cal.has_any_calibration(receptions):
            clock_cal.process_adsb_reference(
                aircraft_ecef=aircraft_ecef,
                receptions=receptions, now=now,
            )

        for rec in receptions:
            s_id = rec.get("sensor_id")
            if s_id is not None:
                s_pos = lla_to_ecef(rec["lat"], rec["lon"], rec["alt"])
                dt_s = rec["timestamp_s"] - result.timestamp_s
                dt_ns = rec["timestamp_ns"] - result.timestamp_ns
                arr_time = dt_s + dt_ns * 1e-9
                dist = float(np.linalg.norm(aircraft_ecef - s_pos))
                expected_arr = dist / C_VACUUM
                actual_arr = arr_time - result.t0_s
                residual_m = (actual_arr - expected_arr) * C_VACUUM
                stats.record_sensor_residual(s_id, residual_m)
    else:
        stats.groups_failed += 1
        if fail_reason:
            stats.failure_reasons[fail_reason] = stats.failure_reasons.get(fail_reason, 0) + 1

        # For 2-sensor failures with altitude, still pass downstream
        if n_sensors == 2 and effective_alt_ft is not None:
            output = {"unsolved_group": group}
            print(json.dumps(output, separators=(",", ":")), flush=True)


def main() -> None:
    log("=== MLAT Solver (Layer 4) — Enhanced with ADS-B CPR Seeding ===")
    log("Solver: Frisch TOA formulation with Inamdar algebraic initialization")

    # Load location overrides for Cornwall sensors
    overrides = _load_location_overrides()
    override_map = SensorOverrideMap(overrides) if overrides else None

    stats = Stats()
    clock_cal = ClockCalibrator()
    pos_cache = PositionCache()
    cpr_buffer = CPRBuffer()

    counters = {"cpr_seeds": 0, "df17_alt_extracted": 0, "cached_alt_used": 0}

    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                group = json.loads(line)
            except (json.JSONDecodeError, TypeError):
                stats.parse_errors += 1
                continue

            stats.groups_received += 1

            # Use message timestamp for clock sync (Q1 fix: time.monotonic()
            # gives ~ms dt in batch replay; message timestamps give real seconds)
            _receptions = group.get("receptions", [])
            if _receptions:
                _msg_ts = (
                    _receptions[0]["timestamp_s"]
                    + _receptions[0]["timestamp_ns"] * 1e-9
                )
            else:
                _msg_ts = time.monotonic()

            _process_group(
                group, clock_cal, pos_cache, cpr_buffer,
                stats, counters, _msg_ts, override_map,
            )

    except KeyboardInterrupt:
        log("Interrupted by user")
    except BrokenPipeError:
        pass
    finally:
        final_stats = stats.to_dict()
        final_stats["clock_cal"] = clock_cal.stats_dict()
        final_stats["pos_cache"] = pos_cache.stats_dict()
        final_stats["cpr_buffer"] = cpr_buffer.stats_dict()
        final_stats["cpr_cache_seeds"] = counters["cpr_seeds"]
        final_stats["df17_alt_extracted"] = counters["df17_alt_extracted"]
        final_stats["cached_alt_used"] = counters["cached_alt_used"]
        if override_map is not None:
            final_stats["location_overrides"] = override_map.stats_dict()
        log("Shutting down. Final stats:")
        log(f"[stats] {json.dumps(final_stats, indent=2)}")
        log(f"Clock calibration: {clock_cal.calibrated_pairs} valid pairings")
        log(f"Position cache: {len(pos_cache.cache)} aircraft tracked")
        log(f"CPR cache seeds: {counters['cpr_seeds']} (global={cpr_buffer.global_decodes}, local={cpr_buffer.local_decodes})")


if __name__ == "__main__":
    main()
