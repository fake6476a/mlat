"""Decode raw Mode-S hex messages into structured fields with pyModeS."""

import pyModeS as pms

# Accept short 56-bit and long 112-bit Mode-S hex messages.
VALID_MSG_LENGTHS = (14, 28)

# Track DF types that carry altitude in the AC field.
DF_WITH_ALTITUDE = {0, 4, 16, 20}

# Track DF types that carry identity or squawk in the ID field.
DF_WITH_SQUAWK = {5, 21}

# Track DF types whose ICAO is recovered through AP XOR CRC.
DF_ICAO_PARITY = {0, 4, 5, 16, 20, 21}

# List the DF types this decoder handles.
KNOWN_DF_TYPES = {0, 4, 5, 11, 16, 17, 18, 20, 21}


def is_valid_hex(msg: str) -> bool:
    """Check if a string is valid hexadecimal."""
    try:
        int(msg, 16)
        return True
    except (ValueError, TypeError):
        return False


def decode_message(raw_msg: str) -> dict | None:
    """Decode one raw Mode-S hex message and return the extracted fields."""
    if not raw_msg or not isinstance(raw_msg, str):
        return None

    msg = raw_msg.strip().upper()

    if len(msg) not in VALID_MSG_LENGTHS:
        return None

    if not is_valid_hex(msg):
        return None

    # Decode the downlink format.
    try:
        df = pms.df(msg)
    except Exception:
        return None

    if df not in KNOWN_DF_TYPES:
        return None

    # Decode the ICAO address, including parity-recovered formats.
    try:
        icao = pms.icao(msg)
    except Exception:
        return None

    if not icao or len(icao) != 6:
        return None

    # Reject the all-zero parity-recovered ICAO placeholder.
    if df in DF_ICAO_PARITY:
        if icao == "000000":
            return None

    # Build the decoded result payload.
    result = {
        "icao": icao.upper(),
        "df_type": df,
        "altitude_ft": None,
        "squawk": None,
        "raw_msg": msg,
    }

    # Decode altitude for DF types that expose it directly.
    if df in DF_WITH_ALTITUDE:
        try:
            alt = pms.altcode(msg)
            if alt is not None:
                result["altitude_ft"] = int(alt)
        except Exception:
            pass

    # Decode DF17 airborne-position altitude from the ADS-B ME field when needed.
    if df == 17 and len(msg) == 28 and result["altitude_ft"] is None:
        try:
            tc = pms.adsb.typecode(msg)
            if tc is not None and 9 <= tc <= 18:
                alt = pms.adsb.altitude(msg)
                if alt is not None:
                    result["altitude_ft"] = int(alt)
        except Exception:
            pass

    # Decode the squawk or identity field when present.
    if df in DF_WITH_SQUAWK:
        try:
            squawk = pms.idcode(msg)
            if squawk is not None:
                result["squawk"] = str(squawk)
        except Exception:
            pass

    return result
