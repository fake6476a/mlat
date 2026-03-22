"""ADS-B position and velocity decoder for MLAT cache seeding.

Decodes CPR-encoded latitude/longitude from DF17 TC 9-18 airborne position
messages, and extracts altitude from the ME field. Also decodes ground
speed and heading from DF17 TC 19 velocity messages.

CPR (Compact Position Reporting) global decoding requires pairing an even
frame (F=0) with an odd frame (F=1) from the same ICAO within ~10 seconds.
Local decoding uses a single frame plus a reference position within ~180nm.

References:
  - The 1090MHz Riddle (mode-s.org/decode) by Junzi Sun, Chapter 6
  - ICAO Doc 9871 (Mode S Technical Provisions)
  - RTCA DO-260B (ADS-B message formats)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from geo import lla_to_ecef

# CPR pairing window: max time between even and odd frames (seconds)
CPR_PAIR_WINDOW = 10.0

# Number of longitude zones (standard ADS-B NZ=15)
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
    """Per-ICAO buffer for CPR even/odd frame pairing.

    Stores the most recent even and odd frames for each ICAO,
    attempting global CPR decode when a pair is available.
    """

    def __init__(self) -> None:
        # icao -> {'even': CPRFrame, 'odd': CPRFrame}
        self._frames: dict[str, dict[str, CPRFrame]] = {}
        # icao -> (lat, lon) last decoded reference for local CPR
        self._references: dict[str, tuple[float, float]] = {}
        # Stats
        self.global_decodes = 0
        self.local_decodes = 0
        self.frames_received = 0

    def add_frame(self, icao: str, frame: CPRFrame) -> ADSBPosition | None:
        """Add a CPR frame and attempt decode.

        Tries global decode first (even+odd pair). If no pair available
        but a reference position exists, tries local decode.

        Args:
            icao: ICAO hex address.
            frame: CPR frame extracted from DF17 TC 9-18 message.

        Returns:
            Decoded position or None.
        """
        self.frames_received += 1

        if icao not in self._frames:
            self._frames[icao] = {}

        key = "even" if frame.f_bit == 0 else "odd"
        self._frames[icao][key] = frame

        # Try global decode if we have both frames
        buf = self._frames[icao]
        if "even" in buf and "odd" in buf:
            even = buf["even"]
            odd = buf["odd"]
            dt = abs(even.timestamp - odd.timestamp)
            if dt < CPR_PAIR_WINDOW:
                # Use most recent frame to determine which latitude to use
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

        # Try local decode if we have a reference position
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


# ---------------------------------------------------------------------------
# CPR decode functions
# ---------------------------------------------------------------------------

def _cpr_nl(lat: float) -> int:
    """Compute the number of longitude zones for a given latitude.

    NL(lat) is the number of equal-longitude zones at latitude that
    have the same width as the zones at the equator.
    """
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
    """Global CPR decode from an even+odd frame pair.

    Args:
        lat_cpr_even: 17-bit CPR latitude from even frame.
        lon_cpr_even: 17-bit CPR longitude from even frame.
        lat_cpr_odd: 17-bit CPR latitude from odd frame.
        lon_cpr_odd: 17-bit CPR longitude from odd frame.
        most_recent_is_even: If True, use even frame for final position.

    Returns:
        (latitude, longitude) in degrees, or None if decode fails.
    """
    nz = NZ
    d_lat_even = 360.0 / (4 * nz)       # 6.0 degrees
    d_lat_odd = 360.0 / (4 * nz - 1)    # ~6.1017 degrees

    # Normalize CPR values to [0, 1)
    lat_even = lat_cpr_even / 131072.0
    lon_even = lon_cpr_even / 131072.0
    lat_odd = lat_cpr_odd / 131072.0
    lon_odd = lon_cpr_odd / 131072.0

    # Compute latitude index
    j = math.floor(59.0 * lat_even - 60.0 * lat_odd + 0.5)

    # Compute latitudes for both frames
    lat_e = d_lat_even * ((j % 60) + lat_even)
    lat_o = d_lat_odd * ((j % 59) + lat_odd)

    # Adjust for southern hemisphere
    if lat_e >= 270.0:
        lat_e -= 360.0
    if lat_o >= 270.0:
        lat_o -= 360.0

    # Check NL consistency (latitude zone boundary check)
    nl_e = _cpr_nl(lat_e)
    nl_o = _cpr_nl(lat_o)
    if nl_e != nl_o:
        return None

    # Select latitude based on most recent frame
    if most_recent_is_even:
        lat = lat_e
        nl = nl_e
        ni = max(nl, 1)
    else:
        lat = lat_o
        nl = nl_o
        ni = max(nl - 1, 1)

    # Compute longitude
    m = math.floor(lon_even * (nl - 1) - lon_odd * nl + 0.5)
    if most_recent_is_even:
        lon = (360.0 / ni) * ((m % ni) + lon_even)
    else:
        lon = (360.0 / ni) * ((m % ni) + lon_odd)

    if lon >= 180.0:
        lon -= 360.0

    # Sanity check: reasonable latitude range
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
    """Local CPR decode using a reference position.

    Requires reference position within ~180nm (~333km) of the aircraft.

    Args:
        lat_cpr: 17-bit CPR latitude.
        lon_cpr: 17-bit CPR longitude.
        f_bit: CPR format bit (0=even, 1=odd).
        ref_lat: Reference latitude in degrees.
        ref_lon: Reference longitude in degrees.

    Returns:
        (latitude, longitude) in degrees, or None if decode fails.
    """
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

    # Sanity: decoded lat should be within ~5 degrees of reference
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

    # Sanity: decoded lon should be within ~5 degrees of reference
    if abs(lon - ref_lon) > 5.0:
        return None

    return (lat, lon)


# ---------------------------------------------------------------------------
# DF17 field extraction from raw hex messages
# ---------------------------------------------------------------------------

def extract_df17_position_fields(raw_msg: str) -> CPRFrame | None:
    """Extract CPR position fields from a DF17 TC 9-18 message.

    Parses the ME (Message Extended squitter) field to extract:
    - Altitude (12-bit Gillham/Q-bit encoded)
    - CPR format bit (even/odd)
    - CPR latitude (17-bit)
    - CPR longitude (17-bit)

    ME field layout for TC 9-18 (airborne position):
        TC(5) | SS(2) | SAF(1) | ALT(12) | T(1) | F(1) | LAT-CPR(17) | LON-CPR(17)

    Args:
        raw_msg: 28-char hex string (DF17 message).

    Returns:
        CPRFrame or None if extraction fails.
    """
    if len(raw_msg) != 28:
        return None

    try:
        # Check DF=17
        first_byte = int(raw_msg[0:2], 16)
        df = first_byte >> 3
        if df != 17:
            return None

        # ME field is bytes 4-10 (hex chars 8-21), 7 bytes
        me = bytes.fromhex(raw_msg[8:22])

        # Type code = first 5 bits of ME
        tc = me[0] >> 3
        if not (9 <= tc <= 18):
            return None

        # Altitude: bits 8-19 of ME (12 bits)
        alt_code = ((me[0] & 0x07) << 9) | (me[1] << 1) | (me[2] >> 7)
        alt_ft = _decode_altitude(alt_code)

        # CPR format bit: bit 21 of ME
        f_bit = (me[2] >> 2) & 1

        # CPR latitude: bits 22-38 of ME (17 bits)
        lat_cpr = ((me[2] & 0x03) << 15) | (me[3] << 7) | (me[4] >> 1)

        # CPR longitude: bits 39-55 of ME (17 bits)
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
    """Decode 12-bit altitude code from DF17 TC 9-18 ME field.

    The Q-bit (bit 4 of the 12-bit code) determines encoding:
    - Q=1: 25ft resolution (standard)
    - Q=0: 100ft Gillham code (rare, not decoded here)

    Args:
        alt_code: 12-bit altitude code.

    Returns:
        Altitude in feet, or None if cannot decode.
    """
    # Q-bit is bit 4 (0-indexed from MSB) of the 12-bit code
    # Bit layout: N1 N2 N3 N4 Q N5 N6 N7 N8 N9 N10 N11
    q_bit = (alt_code >> 4) & 1

    if q_bit == 1:
        # Remove Q-bit and reconstruct the 11-bit value
        n = ((alt_code & 0xFE0) >> 1) | (alt_code & 0x00F)
        alt_ft = n * 25 - 1000
        if alt_ft < -1000 or alt_ft > 100000:
            return None
        return alt_ft
    else:
        # Gillham code (100ft increments) — rare, skip for now
        return None


def extract_df17_velocity(raw_msg: str) -> DecodedVelocity | None:
    """Extract velocity from a DF17 TC 19 airborne velocity message.

    Only decodes subtype 1 (ground speed, subsonic).

    ME field layout for TC 19 ST 1:
        TC(5) | ST(3) | IC(1) | IFR(1) | NACv(3) |
        EWdir(1) | EWvel(10) | NSdir(1) | NSvel(10) |
        VrSrc(1) | VrSign(1) | Vr(9) | ...

    Args:
        raw_msg: 28-char hex string (DF17 message).

    Returns:
        DecodedVelocity or None.
    """
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
            return None  # Only decode subtype 1 (ground speed, subsonic)

        # EW direction and velocity
        ew_dir = (me[1] >> 2) & 1
        ew_vel = ((me[1] & 0x03) << 8) | me[2]

        # NS direction and velocity
        ns_dir = (me[3] >> 7) & 1
        ns_vel = ((me[3] & 0x7F) << 3) | (me[4] >> 5)

        if ew_vel == 0 or ns_vel == 0:
            return None  # velocity not available

        # Convert to signed knots
        ew_knots = float(ew_vel - 1)
        ns_knots = float(ns_vel - 1)
        if ew_dir:
            ew_knots = -ew_knots  # west
        if ns_dir:
            ns_knots = -ns_knots  # south

        speed = math.sqrt(ew_knots ** 2 + ns_knots ** 2)
        heading = math.degrees(math.atan2(ew_knots, ns_knots)) % 360.0

        # Vertical rate
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
    """Convert a decoded ADS-B position to ECEF coordinates.

    Args:
        lat: Latitude in degrees.
        lon: Longitude in degrees.
        alt_ft: Altitude in feet, or None (defaults to 10000m / ~32808ft).

    Returns:
        ECEF position as numpy array [x, y, z].
    """
    alt_m = alt_ft * 0.3048 if alt_ft is not None else 10000.0
    return lla_to_ecef(lat, lon, alt_m)
