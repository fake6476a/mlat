"""Track manager for associating MLAT fixes to continuous flight tracks.

Maintains a dictionary of per-ICAO aircraft tracks, each backed by an
EKF instance. Handles track creation, update, and pruning of stale tracks.

Computes derived quantities (heading, ground speed, vertical rate) from
the EKF velocity state for downstream consumers (Layer 6 Live Map).

References:
  - MLAT_Verified_Combined_Reference.md Part 3.3 (Layer 5 spec)
  - MLAT_Verified_Combined_Reference.md Part 22 (EKF formulas)
  - MLAT_Verified_Combined_Reference.md Part 21 (Innovation gating)
"""

from __future__ import annotations

import math
import time

import numpy as np

from ekf import AircraftEKF

# Add parent directory for geo module access
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mlat-solver"))

from geo import ecef_to_lla, lla_to_ecef

# Maximum track age (seconds) before pruning
MAX_TRACK_AGE_S = 300.0

# Minimum updates for a track to be considered established
MIN_TRACK_UPDATES = 2

# Maximum number of track positions to retain in history
MAX_TRACK_HISTORY = 100

# Conversion factors
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
    ) -> float:
        """Update track with a new position fix.

        Args:
            position_ecef: Position [x, y, z] in ECEF meters.
            timestamp_s: Measurement timestamp in seconds.
            alt_ft: Altitude in feet.
            df_type: Downlink format type.
            squawk: Squawk code.

        Returns:
            Mahalanobis distance (-1.0 if rejected by gate).
        """
        mahal = self.ekf.update(position_ecef, timestamp_s)

        if mahal >= 0:
            # Measurement accepted
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

            # Trim history to MAX_TRACK_HISTORY
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
        """Estimated heading in degrees (0=N, 90=E, 180=S, 270=W).

        Computed from the EKF velocity vector projected onto the
        local ENU (East-North-Up) frame at the current position.
        """
        vel = self.ekf.velocity
        speed = np.linalg.norm(vel)
        if speed < 1.0:
            return 0.0

        # Get current position in geodetic coordinates
        pos = self.ekf.position
        lat, lon, _ = ecef_to_lla(pos[0], pos[1], pos[2])
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)

        # ECEF to ENU rotation
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
        """Estimated ground speed in knots.

        Computed from horizontal components of the EKF velocity
        projected to the local ENU frame.
        """
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
        """Estimated vertical rate in feet per minute.

        Computed from the vertical (Up) component of the EKF velocity
        projected to the local ENU frame.
        """
        vel = self.ekf.velocity
        pos = self.ekf.position
        lat, lon, _ = ecef_to_lla(pos[0], pos[1], pos[2])
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)

        sin_lat = math.sin(lat_r)
        cos_lat = math.cos(lat_r)
        sin_lon = math.sin(lon_r)
        cos_lon = math.cos(lon_r)

        # Up component
        vu = cos_lat * cos_lon * vel[0] + cos_lat * sin_lon * vel[1] + sin_lat * vel[2]
        return vu * MPS_TO_FPM

    def to_output_dict(self, fix: dict) -> dict:
        """Build output dict combining EKF state with original fix data.

        Args:
            fix: Original position fix dict from Layer 4.

        Returns:
            Enriched output dict with track state.
        """
        pos = self.ekf.position
        lat, lon, _ = ecef_to_lla(pos[0], pos[1], pos[2])

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
            "gdop": fix.get("gdop", 0.0),
            "num_sensors": fix.get("num_sensors", 0),
            "solve_method": fix.get("solve_method", ""),
            "timestamp_s": fix.get("timestamp_s", 0),
            "timestamp_ns": fix.get("timestamp_ns", 0),
            "df_type": self.last_df_type,
            "squawk": self.last_squawk,
            "raw_msg": fix.get("raw_msg", ""),
            "t0_s": fix.get("t0_s", 0.0),
        }


class TrackManager:
    """Manages all active aircraft tracks.

    Associates incoming MLAT position fixes with existing tracks
    by ICAO address, creates new tracks for unknown aircraft, and
    prunes stale tracks that have not received updates.
    """

    def __init__(self) -> None:
        # Active tracks keyed by ICAO hex address
        self._tracks: dict[str, TrackState] = {}

        # Stats counters
        self.fixes_received = 0
        self.fixes_accepted = 0
        self.fixes_rejected = 0
        self.tracks_created = 0
        self.tracks_pruned = 0

    def process_fix(self, fix: dict) -> dict | None:
        """Process a single position fix from Layer 4.

        Args:
            fix: Position fix dict from Layer 4 with fields:
                icao, lat, lon, alt_ft, residual_m, gdop,
                num_sensors, solve_method, timestamp_s, timestamp_ns,
                df_type, squawk, raw_msg, t0_s

        Returns:
            Enriched track output dict, or None if rejected.
        """
        self.fixes_received += 1

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

        # Convert altitude for ECEF computation
        alt_m = alt_ft * 0.3048 if alt_ft is not None else 0.0
        if alt_ft is None:
            alt_ft = 0.0

        # Convert to ECEF for EKF processing
        position_ecef = lla_to_ecef(lat, lon, alt_m)

        # Compute timestamp as float seconds
        ts = float(timestamp_s) + float(timestamp_ns) * 1e-9

        track = self._tracks.get(icao)

        if track is None:
            # Create new track
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

        # Update existing track
        mahal = track.update(
            position_ecef=position_ecef,
            timestamp_s=ts,
            alt_ft=alt_ft,
            df_type=df_type,
            squawk=squawk,
        )

        if mahal < 0:
            # Rejected by innovation gate
            self.fixes_rejected += 1
            return None

        self.fixes_accepted += 1
        return track.to_output_dict(fix)

    def prune_stale(self) -> int:
        """Remove tracks that have not received updates recently.

        Returns:
            Number of tracks pruned.
        """
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
        """Get summary of all active tracks for status reporting."""
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
