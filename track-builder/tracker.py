"""Manage MLAT fixes and fold them into continuous EKF-backed aircraft tracks."""

from __future__ import annotations

import math
import time

import numpy as np

from ekf import AircraftEKF
from geo import ecef_to_lla, lla_to_ecef

# Limit the time a stale track can survive without updates.
MAX_TRACK_AGE_S = 300.0

# Require at least two updates before treating a track as established.
MIN_TRACK_UPDATES = 2

# Keep at most this many historical positions per track.
MAX_TRACK_HISTORY = 100

# Define velocity conversion factors.
MPS_TO_KTS = 1.9438444924406  # meters/second to knots
MPS_TO_FPM = 196.85039370079  # meters/second to feet/minute


class TrackState:
    """Complete state of an aircraft track."""

    __slots__ = (
        "icao", "ekf", "positions", "timestamps",
        "last_update_time", "creation_time",
        "last_alt_ft", "last_df_type", "last_squawk",
    )

    def __init__(
        self,
        icao: str,
        position_ecef: np.ndarray,
        timestamp_s: float,
        alt_ft: float,
        df_type: int,
        squawk: str | None,
    ) -> None:
        self.icao = icao
        self.ekf = AircraftEKF(position_ecef, timestamp_s)
        self.positions: list[tuple[float, float, float, float]] = []  # (lat, lon, alt_ft, ts)
        self.timestamps: list[float] = []
        self.last_update_time = time.monotonic()
        self.creation_time = time.monotonic()
        self.last_alt_ft = alt_ft
        self.last_df_type = df_type
        self.last_squawk = squawk

        # Store initial position
        lat, lon, _ = ecef_to_lla(
            position_ecef[0], position_ecef[1], position_ecef[2]
        )
        self.positions.append((lat, lon, alt_ft, timestamp_s))
        self.timestamps.append(timestamp_s)

    def update(
        self,
        position_ecef: np.ndarray,
        timestamp_s: float,
        alt_ft: float,
        df_type: int,
        squawk: str | None,
        measurement_noise_m: float | None = None,
    ) -> float:
        """Update the track with a new position fix and return its gate distance."""
        mahal = self.ekf.update(position_ecef, timestamp_s, measurement_noise_m)

        if mahal >= 0:
            # Persist the accepted measurement and derived metadata.
            self.last_update_time = time.monotonic()
            self.last_alt_ft = alt_ft
            self.last_df_type = df_type
            if squawk is not None:
                self.last_squawk = squawk

            lat, lon, _ = ecef_to_lla(
                self.ekf.position[0], self.ekf.position[1], self.ekf.position[2]
            )
            self.positions.append((lat, lon, alt_ft, timestamp_s))
            self.timestamps.append(timestamp_s)

            # Trim the stored track history to the configured limit.
            if len(self.positions) > MAX_TRACK_HISTORY:
                self.positions = self.positions[-MAX_TRACK_HISTORY:]
                self.timestamps = self.timestamps[-MAX_TRACK_HISTORY:]

        return mahal

    @property
    def age_s(self) -> float:
        """Time since last update in seconds (wall clock)."""
        return time.monotonic() - self.last_update_time

    @property
    def is_established(self) -> bool:
        """Track has enough updates to be considered reliable."""
        return self.ekf.updates >= MIN_TRACK_UPDATES

    @property
    def heading_deg(self) -> float:
        """Return the estimated heading in degrees from the local ENU velocity."""
        vel = self.ekf.velocity
        speed = np.linalg.norm(vel)
        if speed < 1.0:
            return 0.0

        # Convert the current ECEF position to geodetic coordinates.
        pos = self.ekf.position
        lat, lon, _ = ecef_to_lla(pos[0], pos[1], pos[2])
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)

        # Build the local ENU basis vectors.
        sin_lat = math.sin(lat_r)
        cos_lat = math.cos(lat_r)
        sin_lon = math.sin(lon_r)
        cos_lon = math.cos(lon_r)

        # East component
        ve = -sin_lon * vel[0] + cos_lon * vel[1]
        # North component
        vn = -sin_lat * cos_lon * vel[0] - sin_lat * sin_lon * vel[1] + cos_lat * vel[2]

        heading = math.degrees(math.atan2(ve, vn)) % 360.0
        return heading

    @property
    def ground_speed_kts(self) -> float:
        """Return the estimated ground speed in knots from the ENU velocity."""
        vel = self.ekf.velocity
        pos = self.ekf.position
        lat, lon, _ = ecef_to_lla(pos[0], pos[1], pos[2])
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)

        sin_lat = math.sin(lat_r)
        cos_lat = math.cos(lat_r)
        sin_lon = math.sin(lon_r)
        cos_lon = math.cos(lon_r)

        ve = -sin_lon * vel[0] + cos_lon * vel[1]
        vn = -sin_lat * cos_lon * vel[0] - sin_lat * sin_lon * vel[1] + cos_lat * vel[2]

        ground_speed_mps = math.sqrt(ve * ve + vn * vn)
        return ground_speed_mps * MPS_TO_KTS

    @property
    def vertical_rate_fpm(self) -> float:
        """Return the estimated vertical rate in feet per minute."""
        vel = self.ekf.velocity
        pos = self.ekf.position
        lat, lon, _ = ecef_to_lla(pos[0], pos[1], pos[2])
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)

        sin_lat = math.sin(lat_r)
        cos_lat = math.cos(lat_r)
        sin_lon = math.sin(lon_r)
        cos_lon = math.cos(lon_r)

        # Use the local Up component of the ENU velocity.
        vu = cos_lat * cos_lon * vel[0] + cos_lat * sin_lon * vel[1] + sin_lat * vel[2]
        return vu * MPS_TO_FPM

    def to_output_dict(self, fix: dict) -> dict:
        """Build the Layer 5 output dict from EKF state and the input fix."""
        pos = self.ekf.position
        lat, lon, _ = ecef_to_lla(pos[0], pos[1], pos[2])

        # Convert ECEF covariance into local ENU coordinates for 2D visualization.
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)
        sin_lat = math.sin(lat_r)
        cos_lat = math.cos(lat_r)
        sin_lon = math.sin(lon_r)
        cos_lon = math.cos(lon_r)
        R_enu = np.array([
            [-sin_lon, cos_lon, 0],
            [-sin_lat*cos_lon, -sin_lat*sin_lon, cos_lat],
            [cos_lat*cos_lon, cos_lat*sin_lon, sin_lat]
        ])
        P_enu = R_enu @ self.ekf.P[:3, :3] @ R_enu.T

        return {
            "icao": self.icao,
            "lat": round(lat, 6),
            "lon": round(lon, 6),
            "alt_ft": round(self.last_alt_ft, 0),
            "heading_deg": round(self.heading_deg, 1),
            "speed_kts": round(self.ground_speed_kts, 1),
            "vrate_fpm": round(self.vertical_rate_fpm, 0),
            "track_quality": self.ekf.updates,
            "positions_count": len(self.positions),
            "residual_m": fix.get("residual_m", 0.0),
            "quality_residual_m": fix.get("quality_residual_m", fix.get("residual_m", 0.0)),
            "gdop": fix.get("gdop", 0.0),
            "num_sensors": fix.get("num_sensors", 0),
            "solve_method": fix.get("solve_method", ""),
            "timestamp_s": fix.get("timestamp_s", 0),
            "timestamp_ns": fix.get("timestamp_ns", 0),
            "df_type": self.last_df_type,
            "squawk": self.last_squawk,
            "raw_msg": fix.get("raw_msg", ""),
            "t0_s": fix.get("t0_s", 0.0),
            "cov_matrix": [
                [float(P_enu[0, 0]), float(P_enu[0, 1])],
                [float(P_enu[1, 0]), float(P_enu[1, 1])]
            ],
        }


class TrackManager:
    """Manage active aircraft tracks and associate incoming MLAT fixes by ICAO."""

    def __init__(self) -> None:
        # Store active tracks keyed by ICAO hex address.
        self._tracks: dict[str, TrackState] = {}

        # Track basic Layer 5 statistics.
        self.fixes_received = 0
        self.fixes_accepted = 0
        self.fixes_rejected = 0
        self.tracks_created = 0
        self.tracks_pruned = 0

    def process_fix(self, fix: dict) -> dict | None:
        """Process one Layer 4 fix and return an enriched track update if accepted."""
        self.fixes_received += 1

        if "unsolved_group" in fix:
            return self._solve_prediction_aided(fix["unsolved_group"])

        icao = fix.get("icao", "")
        if not icao:
            return None

        lat = fix.get("lat")
        lon = fix.get("lon")
        alt_ft = fix.get("alt_ft")
        timestamp_s = fix.get("timestamp_s", 0)
        timestamp_ns = fix.get("timestamp_ns", 0)
        df_type = fix.get("df_type", 0)
        squawk = fix.get("squawk")

        if lat is None or lon is None:
            return None

        # Convert the reported altitude for ECEF computation.
        alt_m = alt_ft * 0.3048 if alt_ft is not None else 0.0
        if alt_ft is None:
            alt_ft = 0.0

        # Convert the measurement into ECEF for EKF processing.
        position_ecef = lla_to_ecef(lat, lon, alt_m)

        # Combine the timestamp fields into floating-point seconds.
        ts = float(timestamp_s) + float(timestamp_ns) * 1e-9

        # Derive measurement noise from solver quality and geometry.
        residual_m = fix.get("quality_residual_m", fix.get("residual_m", 0.0))
        gdop = fix.get("gdop", 0.0)
        if residual_m > 0 and gdop > 0:
            meas_noise = min(2000.0, max(50.0, residual_m * gdop))
        elif residual_m > 0:
            meas_noise = min(2000.0, max(50.0, residual_m * 2.0))
        else:
            meas_noise = None  # fall back to the EKF default

        track = self._tracks.get(icao)

        if track is None:
            # Create a new track when this ICAO is first seen.
            track = TrackState(
                icao=icao,
                position_ecef=position_ecef,
                timestamp_s=ts,
                alt_ft=alt_ft,
                df_type=df_type,
                squawk=squawk,
            )
            self._tracks[icao] = track
            self.tracks_created += 1
            self.fixes_accepted += 1
            return track.to_output_dict(fix)

        # Update an existing track with the new measurement.
        mahal = track.update(
            position_ecef=position_ecef,
            timestamp_s=ts,
            alt_ft=alt_ft,
            df_type=df_type,
            squawk=squawk,
            measurement_noise_m=meas_noise,
        )

        if mahal < 0:
            # Drop fixes rejected by the innovation gate.
            self.fixes_rejected += 1
            return None

        self.fixes_accepted += 1
        return track.to_output_dict(fix)

    def _solve_prediction_aided(self, group: dict) -> dict | None:
        """Attempt to solve a 2-sensor group using EKF track prediction."""
        icao = group.get("icao", "")
        if not icao:
            return None

        track = self._tracks.get(icao)
        if track is None or not track.is_established:
            return None  # Cannot predict-aid without an established track

        receptions = group.get("receptions", [])
        if len(receptions) != 2:
            return None

        altitude_ft = group.get("altitude_ft")
        if altitude_ft is None:
            return None
        altitude_m = float(altitude_ft) * 0.3048

        # Build the 2-sensor geometry and relative arrival times.
        sensor_positions = np.zeros((2, 3))
        sensor_alts_m = np.zeros(2)
        arrival_times = np.zeros(2)

        ref_idx = min(range(2), key=lambda i: receptions[i]["timestamp_s"] * 1000000000 + receptions[i]["timestamp_ns"])
        ref_timestamp_s = receptions[ref_idx]["timestamp_s"]
        ref_timestamp_ns = receptions[ref_idx]["timestamp_ns"]

        for i, rec in enumerate(receptions):
            sensor_positions[i] = lla_to_ecef(rec["lat"], rec["lon"], rec["alt"])
            sensor_alts_m[i] = rec["alt"]
            dt_s = rec["timestamp_s"] - ref_timestamp_s
            dt_ns = rec["timestamp_ns"] - ref_timestamp_ns
            arrival_times[i] = dt_s + dt_ns * 1e-9

        # Predict the track position at the target timestamp.
        target_ts = float(ref_timestamp_s) + float(ref_timestamp_ns) * 1e-9
        predicted_ecef = track.ekf.predict(target_ts)

        # Scale the prediction anchor from EKF covariance quality.
        pred_uncertainty = float(np.sqrt(np.trace(track.ekf.P[:3, :3])))
        pw = max(1.0, min(12.0, 500.0 / max(pred_uncertainty, 1.0)))

        from frisch import solve_toa
        result = solve_toa(
            sensors=sensor_positions,
            arrival_times=arrival_times,
            sensor_alts_m=sensor_alts_m,
            x0=predicted_ecef,
            altitude_m=altitude_m,
            track_prediction_ecef=predicted_ecef,
            prediction_weight=pw,
        )

        if result is None:
            return None

        # Reformat the prediction-aided result as a normal Layer 4 fix.
        position = result["position"]
        lat, lon, _ = ecef_to_lla(position[0], position[1], position[2])

        solved_fix = {
            "icao": icao,
            "lat": lat,
            "lon": lon,
            "alt_ft": altitude_ft,
            "residual_m": result["residual_m"],
            "quality_residual_m": result.get("objective_residual_m", result["residual_m"]),
            "gdop": 0.0,
            "num_sensors": 2,
            "solve_method": "prediction_aided_2sensor",
            "timestamp_s": ref_timestamp_s,
            "timestamp_ns": ref_timestamp_ns,
            "df_type": group.get("df_type", 0),
            "squawk": group.get("squawk"),
            "raw_msg": group.get("raw_msg", ""),
            "t0_s": result["t0_s"],
        }
        
        # Avoid double-counting the unsolved input while processing the solved fix.
        self.fixes_received -= 1
        return self.process_fix(solved_fix)

    def prune_stale(self) -> int:
        """Remove stale tracks and return the number pruned."""
        stale_keys = [
            icao for icao, track in self._tracks.items()
            if track.age_s > MAX_TRACK_AGE_S
        ]
        for icao in stale_keys:
            del self._tracks[icao]
        self.tracks_pruned += len(stale_keys)
        return len(stale_keys)

    @property
    def active_track_count(self) -> int:
        return len(self._tracks)

    @property
    def established_track_count(self) -> int:
        return sum(1 for t in self._tracks.values() if t.is_established)

    def get_all_tracks(self) -> list[dict]:
        """Return summaries of all active established tracks for status reporting."""
        result = []
        for icao, track in self._tracks.items():
            if track.is_established and len(track.positions) > 0:
                last_pos = track.positions[-1]
                result.append({
                    "icao": icao,
                    "lat": round(last_pos[0], 6),
                    "lon": round(last_pos[1], 6),
                    "alt_ft": round(last_pos[2], 0),
                    "heading_deg": round(track.heading_deg, 1),
                    "speed_kts": round(track.ground_speed_kts, 1),
                    "vrate_fpm": round(track.vertical_rate_fpm, 0),
                    "track_quality": track.ekf.updates,
                    "positions": [
                        {"lat": round(p[0], 6), "lon": round(p[1], 6)}
                        for p in track.positions[-20:]  # Last 20 positions for trail
                    ],
                })
        return result

    def stats(self) -> dict:
        return {
            "fixes_received": self.fixes_received,
            "fixes_accepted": self.fixes_accepted,
            "fixes_rejected": self.fixes_rejected,
            "tracks_created": self.tracks_created,
            "tracks_pruned": self.tracks_pruned,
            "active_tracks": self.active_track_count,
            "established_tracks": self.established_track_count,
            "accept_rate": (
                f"{self.fixes_accepted / self.fixes_received * 100:.1f}%"
                if self.fixes_received > 0
                else "0.0%"
            ),
        }
