"""Mode-S message decoder using pyModeS.

Decodes raw Mode-S hex messages into structured fields:
  - ICAO 24-bit aircraft address
  - Downlink Format (DF) type
  - Barometric altitude (from DF0, DF4, DF16, DF20)
  - Squawk/identity code (from DF5, DF21)

References:
  - The 1090MHz Riddle (mode-s.org/decode) by Junzi Sun
  - pyModeS API docs (mode-s.org/pymodes/api/)
  - ICAO Doc 9871 (Mode S Technical Provisions)
"""

import pyModeS as pms

# Valid hex message lengths:
#   14 hex chars = 56 bits (short: DF0, DF4, DF5, DF11)
#   28 hex chars = 112 bits (long: DF16, DF17, DF18, DF19, DF20, DF21)
VALID_MSG_LENGTHS = (14, 28)

# DF types that carry altitude in the AC (Altitude Code) field
DF_WITH_ALTITUDE = {0, 4, 16, 20}

# DF types that carry identity/squawk in the ID field
DF_WITH_SQUAWK = {5, 21}

# DF types where ICAO must be recovered via AP XOR CRC
DF_ICAO_PARITY = {0, 4, 5, 16, 20, 21}

# All DF types we can meaningfully decode
KNOWN_DF_TYPES = {0, 4, 5, 11, 16, 17, 18, 20, 21}


def is_valid_hex(msg: str) -> bool:
    """Check if a string is valid hexadecimal."""
    try:
        int(msg, 16)
        return True
    except (ValueError, TypeError):
        return False


def decode_message(raw_msg: str) -> dict | None:
    """Decode a raw Mode-S hex message string.

    Args:
        raw_msg: Hex-encoded Mode-S message (14 or 28 hex characters).

    Returns:
        Dictionary with decoded fields, or None if the message is invalid.
        Fields:
            icao: str - 6-char hex ICAO aircraft address (uppercase)
            df_type: int - Downlink Format number
            altitude_ft: int | None - Barometric altitude in feet (DF0/4/16/20)
            squawk: str | None - 4-digit squawk code (DF5/21)
            raw_msg: str - Original hex message (uppercase)
    """
    if not raw_msg or not isinstance(raw_msg, str):
        return None

    msg = raw_msg.strip().upper()

    if len(msg) not in VALID_MSG_LENGTHS:
        return None

    if not is_valid_hex(msg):
        return None

    # Extract downlink format
    try:
        df = pms.df(msg)
    except Exception:
        return None

    if df not in KNOWN_DF_TYPES:
        return None

    # Extract ICAO address
    # pyModeS.icao() handles both clear-text (DF11/17/18) and
    # parity-recovery (DF0/4/5/16/20/21) automatically
    try:
        icao = pms.icao(msg)
    except Exception:
        return None

    if not icao or len(icao) != 6:
        return None

    # For parity-recovered ICAO (DF0/4/5/16/20/21), validate with CRC
    # The CRC remainder after XOR should yield a plausible ICAO
    if df in DF_ICAO_PARITY:
        # For parity-recovered ICAO, verify it's non-zero
        # pyModeS.icao() already does AP XOR CRC(payload) extraction
        if icao == "000000":
            return None

    # Build result
    result = {
        "icao": icao.upper(),
        "df_type": df,
        "altitude_ft": None,
        "squawk": None,
        "raw_msg": msg,
    }

    # Decode altitude for DF types that carry it
    if df in DF_WITH_ALTITUDE:
        try:
            alt = pms.altcode(msg)
            if alt is not None:
                result["altitude_ft"] = int(alt)
        except Exception:
            pass

    # Decode squawk/identity for DF types that carry it
    if df in DF_WITH_SQUAWK:
        try:
            squawk = pms.idcode(msg)
            if squawk is not None:
                result["squawk"] = str(squawk)
        except Exception:
            pass

    return result
