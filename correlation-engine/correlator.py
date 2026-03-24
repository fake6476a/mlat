"""Group decoded Mode-S receptions into time-windowed correlation sets."""

from __future__ import annotations


# Use a 2 ms correlation window in nanoseconds to cover propagation and sync error.
DEFAULT_WINDOW_NS = 2_000_000

# Emit groups with at least two receptions so 2-sensor MLAT remains possible.
DEFAULT_MIN_RECEPTIONS = 2


class Reception:
    """A single sensor reception of a Mode-S message."""

    __slots__ = ("sensor_id", "lat", "lon", "alt", "timestamp_s", "timestamp_ns")

    def __init__(
        self,
        sensor_id: int,
        lat: float,
        lon: float,
        alt: float,
        timestamp_s: int,
        timestamp_ns: int,
    ) -> None:
        self.sensor_id = sensor_id
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.timestamp_s = timestamp_s
        self.timestamp_ns = timestamp_ns

    @property
    def abs_time_ns(self) -> int:
        """Absolute timestamp in nanoseconds since midnight."""
        return self.timestamp_s * 1_000_000_000 + self.timestamp_ns

    def to_dict(self) -> dict:
        return {
            "sensor_id": self.sensor_id,
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "timestamp_s": self.timestamp_s,
            "timestamp_ns": self.timestamp_ns,
        }


class CorrelationGroup:
    """A group of receptions of the same transmission by different sensors."""

    __slots__ = ("icao", "df_type", "altitude_ft", "squawk", "raw_msg", "receptions")

    def __init__(
        self,
        icao: str,
        df_type: int,
        altitude_ft: int | None,
        squawk: str | None,
        raw_msg: str,
    ) -> None:
        self.icao = icao
        self.df_type = df_type
        self.altitude_ft = altitude_ft
        self.squawk = squawk
        self.raw_msg = raw_msg
        self.receptions: list[Reception] = []

    @property
    def first_time_ns(self) -> int:
        """Earliest reception timestamp in the group."""
        return self.receptions[0].abs_time_ns

    @property
    def num_sensors(self) -> int:
        return len(self.receptions)

    def has_sensor(self, sensor_id: int) -> bool:
        """Check if this sensor already contributed to this group."""
        return any(r.sensor_id == sensor_id for r in self.receptions)

    def to_dict(self) -> dict:
        return {
            "icao": self.icao,
            "df_type": self.df_type,
            "altitude_ft": self.altitude_ft,
            "squawk": self.squawk,
            "raw_msg": self.raw_msg,
            "num_sensors": self.num_sensors,
            "receptions": [r.to_dict() for r in self.receptions],
        }


class Correlator:
    """Correlate decoded Mode-S packets by `(icao, raw_msg)` within a time window."""

    def __init__(
        self,
        window_ns: int = DEFAULT_WINDOW_NS,
        min_receptions: int = DEFAULT_MIN_RECEPTIONS,
    ) -> None:
        self.window_ns = window_ns
        self.min_receptions = min_receptions

        # Store active groups keyed by `(icao, raw_msg)`.
        self._buffer: dict[tuple[str, str], CorrelationGroup] = {}

        # Track correlation statistics.
        self.groups_formed = 0
        self.groups_emitted = 0
        self.groups_dropped = 0
        self.messages_processed = 0

    def process(self, packet: dict) -> list[dict]:
        """Process one decoded packet and return any completed correlation groups."""
        self.messages_processed += 1

        icao = packet.get("icao")
        raw_msg = packet.get("raw_msg")
        sensor_id = packet.get("sensor_id")

        if not icao or not raw_msg or sensor_id is None:
            return []

        reception = Reception(
            sensor_id=sensor_id,
            lat=packet.get("lat") or 0.0,
            lon=packet.get("lon") or 0.0,
            alt=packet.get("alt") or 0.0,
            timestamp_s=packet.get("timestamp_s") or 0,
            timestamp_ns=packet.get("timestamp_ns") or 0,
        )

        current_time_ns = reception.abs_time_ns

        # Flush groups whose time window has already expired.
        emitted = self._flush_expired(current_time_ns)

        # Look up or create the group for this `(icao, raw_msg)` key.
        key = (icao, raw_msg)
        group = self._buffer.get(key)

        if group is not None:
            # Accept the reception only if it still falls inside the group window.
            if current_time_ns - group.first_time_ns <= self.window_ns:
                # Ignore duplicate sensors within the same group.
                if not group.has_sensor(sensor_id):
                    group.receptions.append(reception)
            else:
                # Emit the expired group for this key and start a fresh one.
                emitted.extend(self._emit_group(key))
                self._start_new_group(key, packet, reception)
        else:
            self._start_new_group(key, packet, reception)

        return emitted

    def flush_all(self) -> list[dict]:
        """Flush all remaining groups (call at end of stream)."""
        emitted: list[dict] = []
        for key in list(self._buffer.keys()):
            emitted.extend(self._emit_group(key))
        return emitted

    def _start_new_group(
        self, key: tuple[str, str], packet: dict, reception: Reception
    ) -> None:
        """Create a new correlation group."""
        group = CorrelationGroup(
            icao=packet["icao"],
            df_type=packet["df_type"],
            altitude_ft=packet.get("altitude_ft"),
            squawk=packet.get("squawk"),
            raw_msg=packet["raw_msg"],
        )
        group.receptions.append(reception)
        self._buffer[key] = group
        self.groups_formed += 1

    def _flush_expired(self, current_time_ns: int) -> list[dict]:
        """Emit all groups whose time window has expired."""
        emitted: list[dict] = []
        expired_keys: list[tuple[str, str]] = []

        for key, group in self._buffer.items():
            if current_time_ns - group.first_time_ns > self.window_ns:
                expired_keys.append(key)

        for key in expired_keys:
            emitted.extend(self._emit_group(key))

        return emitted

    def _emit_group(self, key: tuple[str, str]) -> list[dict]:
        """Emit a group if it meets the minimum reception threshold."""
        group = self._buffer.pop(key, None)
        if group is None:
            return []

        if group.num_sensors >= self.min_receptions:
            self.groups_emitted += 1
            return [group.to_dict()]
        else:
            self.groups_dropped += 1
            return []

    def stats(self) -> dict:
        return {
            "messages_processed": self.messages_processed,
            "groups_formed": self.groups_formed,
            "groups_emitted": self.groups_emitted,
            "groups_dropped": self.groups_dropped,
            "buffer_size": len(self._buffer),
        }
