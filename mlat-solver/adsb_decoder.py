"""Decode ADS-B CPR positions and TC19 velocities for MLAT cache seeding."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from geo import lla_to_ecef

# Limit the even/odd CPR pairing window in seconds.
CPR_PAIR_WINDOW = 10.0

# Use the standard ADS-B CPR longitude-zone constant.
NZ = 15


@dataclass
class CPRFrame:
    """A single CPR-encoded position frame."""
    f_bit: int          # 0=even, 1=odd
    lat_cpr: int        # 17-bit encoded latitude
    lon_cpr: int        # 17-bit encoded longitude
    alt_ft: int | None  # barometric altitude in feet (from ME field)
    timestamp: float    # message timestamp (timestamp_s + timestamp_ns*1e-9)


@dataclass
class DecodedVelocity:
    """Decoded ADS-B ground velocity."""
    ew_knots: float     # East-West velocity component (knots, east positive)
    ns_knots: float     # North-South velocity component (knots, north positive)
    speed_knots: float  # Ground speed magnitude
    heading_deg: float  # Track angle (degrees, 0=north, clockwise)
    vrate_fpm: int      # Vertical rate (feet per minute, positive=climb)


@dataclass
class ADSBPosition:
    """A fully decoded ADS-B position."""
    lat: float
    lon: float
    alt_ft: int | None
    timestamp: float
    icao: str


class CPRBuffer:
    """Buffer per-ICAO CPR frames and attempt global or local position decodes."""

    def __init__(self) -> None:
        # Store the latest even and odd CPR frames by ICAO.
        self._frames: dict[str, dict[str, CPRFrame]] = {}
        # Store the latest decoded `(lat, lon)` reference for local CPR.
        self._references: dict[str, tuple[float, float]] = {}
        # Track CPR decode statistics.
        self.global_decodes = 0
        self.local_decodes = 0
        self.frames_received = 0

    def add_frame(self, icao: str, frame: CPRFrame) -> ADSBPosition | None:
        """Add one CPR frame and return a decoded ADS-B position when possible."""
        self.frames_received += 1

        if icao not in self._frames:
            self._frames[icao] = {}

        key = "even" if frame.f_bit == 0 else "odd"
        self._frames[icao][key] = frame

        # Try a global decode once both even and odd frames are available.
        buf = self._frames[icao]
        if "even" in buf and "odd" in buf:
            even = buf["even"]
            odd = buf["odd"]
            dt = abs(even.timestamp - odd.timestamp)
            if dt < CPR_PAIR_WINDOW:
                # Use the most recent frame to select the final CPR latitude branch.
                most_recent_is_even = even.timestamp >= odd.timestamp
                result = cpr_global_decode(
                    even.lat_cpr, even.lon_cpr,
                    odd.lat_cpr, odd.lon_cpr,
                    most_recent_is_even,
                )
                if result is not None:
                    lat, lon = result
                    self._references[icao] = (lat, lon)
                    self.global_decodes += 1
                    return ADSBPosition(
                        lat=lat, lon=lon,
                        alt_ft=frame.alt_ft,
                        timestamp=frame.timestamp,
                        icao=icao,
                    )

        # Fall back to local decode when a recent reference position exists.
        ref = self._references.get(icao)
        if ref is not None:
            result = cpr_local_decode(
                frame.lat_cpr, frame.lon_cpr,
                frame.f_bit, ref[0], ref[1],
            )
            if result is not None:
                lat, lon = result
                self._references[icao] = (lat, lon)
                self.local_decodes += 1
                return ADSBPosition(
                    lat=lat, lon=lon,
                    alt_ft=frame.alt_ft,
                    timestamp=frame.timestamp,
                    icao=icao,
                )

        return None

    def stats_dict(self) -> dict:
        """Return buffer statistics."""
        return {
            "frames_received": self.frames_received,
            "global_decodes": self.global_decodes,
            "local_decodes": self.local_decodes,
            "tracked_icaos": len(self._frames),
        }


# CPR decode helpers.

def _cpr_nl(lat: float) -> int:
    """Return the CPR longitude-zone count for a given latitude."""
    if abs(lat) >= 87.0:
        return 1
    try:
        nz = NZ
        cos_lat = math.cos(math.radians(abs(lat)))
        nl = math.floor(
            2.0 * math.pi
            / math.acos(1.0 - (1.0 - math.cos(math.pi / (2.0 * nz))) / (cos_lat * cos_lat))
        )
        return max(nl, 1)
    except (ValueError, ZeroDivisionError):
        return 1


def cpr_global_decode(
    lat_cpr_even: int,
    lon_cpr_even: int,
    lat_cpr_odd: int,
    lon_cpr_odd: int,
    most_recent_is_even: bool = True,
) -> tuple[float, float] | None:
    """Decode a global CPR latitude and longitude from an even/odd frame pair."""
    nz = NZ
    d_lat_even = 360.0 / (4 * nz)       # 6.0 degrees
    d_lat_odd = 360.0 / (4 * nz - 1)    # ~6.1017 degrees

    # Normalize the CPR fields to the `[0, 1)` interval.
    lat_even = lat_cpr_even / 131072.0
    lon_even = lon_cpr_even / 131072.0
    lat_odd = lat_cpr_odd / 131072.0
    lon_odd = lon_cpr_odd / 131072.0

    # Compute the shared latitude index.
    j = math.floor(59.0 * lat_even - 60.0 * lat_odd + 0.5)

    # Recover candidate latitudes for the even and odd frames.
    lat_e = d_lat_even * ((j % 60) + lat_even)
    lat_o = d_lat_odd * ((j % 59) + lat_odd)

    # Wrap latitudes into the signed range.
    if lat_e >= 270.0:
        lat_e -= 360.0
    if lat_o >= 270.0:
        lat_o -= 360.0

    # Reject pairs that straddle different latitude-zone counts.
    nl_e = _cpr_nl(lat_e)
    nl_o = _cpr_nl(lat_o)
    if nl_e != nl_o:
        return None

    # Select the latitude branch from the most recent frame.
    if most_recent_is_even:
        lat = lat_e
        nl = nl_e
        ni = max(nl, 1)
    else:
        lat = lat_o
        nl = nl_o
        ni = max(nl - 1, 1)

    # Recover the longitude from the selected frame branch.
    m = math.floor(lon_even * (nl - 1) - lon_odd * nl + 0.5)
    if most_recent_is_even:
        lon = (360.0 / ni) * ((m % ni) + lon_even)
    else:
        lon = (360.0 / ni) * ((m % ni) + lon_odd)

    if lon >= 180.0:
        lon -= 360.0

    # Reject impossible latitudes.
    if lat < -90.0 or lat > 90.0:
        return None

    return (lat, lon)


def cpr_local_decode(
    lat_cpr: int,
    lon_cpr: int,
    f_bit: int,
    ref_lat: float,
    ref_lon: float,
) -> tuple[float, float] | None:
    """Decode a local CPR latitude and longitude from one frame and a reference."""
    nz = NZ
    if f_bit == 0:
        d_lat = 360.0 / (4 * nz)
    else:
        d_lat = 360.0 / (4 * nz - 1)

    lat_cpr_norm = lat_cpr / 131072.0
    j = math.floor(ref_lat / d_lat) + math.floor(
        0.5 + (ref_lat % d_lat) / d_lat - lat_cpr_norm
    )
    lat = d_lat * (j + lat_cpr_norm)

    # Reject latitudes that drift too far from the reference.
    if abs(lat - ref_lat) > 5.0:
        return None

    nl = _cpr_nl(lat)
    if f_bit == 0:
        ni = max(nl, 1)
    else:
        ni = max(nl - 1, 1)

    d_lon = 360.0 / ni
    lon_cpr_norm = lon_cpr / 131072.0
    m = math.floor(ref_lon / d_lon) + math.floor(
        0.5 + (ref_lon % d_lon) / d_lon - lon_cpr_norm
    )
    lon = d_lon * (m + lon_cpr_norm)

    if lon >= 180.0:
        lon -= 360.0
    if lon < -180.0:
        lon += 360.0

    # Reject longitudes that drift too far from the reference.
    if abs(lon - ref_lon) > 5.0:
        return None

    return (lat, lon)


# DF17 field extraction helpers.

def extract_df17_position_fields(raw_msg: str) -> CPRFrame | None:
    """Extract CPR position fields from a DF17 airborne-position message."""
    if len(raw_msg) != 28:
        return None

    try:
        # Reject non-DF17 messages.
        first_byte = int(raw_msg[0:2], 16)
        df = first_byte >> 3
        if df != 17:
            return None

        # Decode the 7-byte ME field.
        me = bytes.fromhex(raw_msg[8:22])

        # Reject non-position type codes.
        tc = me[0] >> 3
        if not (9 <= tc <= 18):
            return None

        # Extract the 12-bit altitude code from the ME field.
        alt_code = ((me[0] & 0x07) << 9) | (me[1] << 1) | (me[2] >> 7)
        alt_ft = _decode_altitude(alt_code)

        # Extract the even/odd CPR format bit.
        f_bit = (me[2] >> 2) & 1

        # Extract the 17-bit CPR latitude.
        lat_cpr = ((me[2] & 0x03) << 15) | (me[3] << 7) | (me[4] >> 1)

        # Extract the 17-bit CPR longitude.
        lon_cpr = ((me[4] & 0x01) << 16) | (me[5] << 8) | me[6]

        return CPRFrame(
            f_bit=f_bit,
            lat_cpr=lat_cpr,
            lon_cpr=lon_cpr,
            alt_ft=alt_ft,
            timestamp=0.0,  # caller sets this
        )
    except (ValueError, IndexError):
        return None


def _decode_altitude(alt_code: int) -> int | None:
    """Decode a 12-bit DF17 altitude code into feet when the Q-bit is set."""
    # Extract the Q-bit from the 12-bit altitude code.
    q_bit = (alt_code >> 4) & 1

    if q_bit == 1:
        # Remove the Q-bit and reconstruct the 11-bit altitude value.
        n = ((alt_code & 0xFE0) >> 1) | (alt_code & 0x00F)
        alt_ft = n * 25 - 1000
        if alt_ft < -1000 or alt_ft > 100000:
            return None
        return alt_ft
    else:
        # Skip rare Gillham-coded values for now.
        return None


def extract_df17_velocity(raw_msg: str) -> DecodedVelocity | None:
    """Extract subtype-1 DF17 TC19 airborne velocity data."""
    if len(raw_msg) != 28:
        return None

    try:
        first_byte = int(raw_msg[0:2], 16)
        df = first_byte >> 3
        if df != 17:
            return None

        me = bytes.fromhex(raw_msg[8:22])
        tc = me[0] >> 3
        if tc != 19:
            return None

        st = me[0] & 0x07
        if st != 1:
            return None  # Only decode subtype 1 ground-speed messages.

        # Extract east-west direction and speed.
        ew_dir = (me[1] >> 2) & 1
        ew_vel = ((me[1] & 0x03) << 8) | me[2]

        # Extract north-south direction and speed.
        ns_dir = (me[3] >> 7) & 1
        ns_vel = ((me[3] & 0x7F) << 3) | (me[4] >> 5)

        if ew_vel == 0 or ns_vel == 0:
            return None  # Velocity is not available.

        # Convert the signed horizontal speeds to knots.
        ew_knots = float(ew_vel - 1)
        ns_knots = float(ns_vel - 1)
        if ew_dir:
            ew_knots = -ew_knots  # west
        if ns_dir:
            ns_knots = -ns_knots  # south

        speed = math.sqrt(ew_knots ** 2 + ns_knots ** 2)
        heading = math.degrees(math.atan2(ew_knots, ns_knots)) % 360.0

        # Extract the vertical rate.
        vr_sign = (me[4] >> 4) & 1
        vr_raw = ((me[4] & 0x0F) << 5) | (me[5] >> 3)
        if vr_raw == 0:
            vrate_fpm = 0
        else:
            vrate_fpm = (vr_raw - 1) * 64
            if vr_sign:
                vrate_fpm = -vrate_fpm

        return DecodedVelocity(
            ew_knots=ew_knots,
            ns_knots=ns_knots,
            speed_knots=speed,
            heading_deg=heading,
            vrate_fpm=vrate_fpm,
        )
    except (ValueError, IndexError):
        return None


def position_to_ecef(lat: float, lon: float, alt_ft: int | None) -> np.ndarray:
    """Convert a decoded ADS-B position into ECEF coordinates."""
    alt_m = alt_ft * 0.3048 if alt_ft is not None else 10000.0
    return lla_to_ecef(lat, lon, alt_m)
