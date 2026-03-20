"""Correlation engine for grouping Mode-S receptions.

Groups decoded Mode-S messages from multiple sensors that received the
SAME transmission from the SAME aircraft at nearly the SAME time.

Correlation key: (ICAO address, raw message content)
  - Same ICAO = same aircraft
  - Same raw_msg = same transmission (identical radio frame)

Time window: configurable, default 2ms (2,000,000 ns)
  - Airsquitter sensors have 30ns GPS sync
  - Max propagation delay across 50km baseline ≈ 167µs
  - 2ms provides comfortable margin

References:
  - MLAT_Verified_Combined_Reference.md Part 3.3 (Layer 3 spec)
  - Workshop tips: "Group messages from different sensors"
  - Previous run stats: 156K groups formed, 101K dropped (< 3 receptions)
"""

from __future__ import annotations


# Default correlation window: 2ms in nanoseconds
# Max propagation delay for 50km sensor baseline ≈ 167µs
# GPS sync precision: 30ns (Jetvision Airsquitter)
DEFAULT_WINDOW_NS = 2_000_000

# Minimum receptions to emit a group
# 2 = enables 2-sensor semi-multilateration (differentiating feature)
# 3 = standard MLAT minimum
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
    """Time-windowed correlation engine using Python dicts.

    Groups decoded Mode-S packets by (ICAO, raw_msg) key within a
    configurable time window. Emits groups when the window expires.
    """

    def __init__(
        self,
        window_ns: int = DEFAULT_WINDOW_NS,
        min_receptions: int = DEFAULT_MIN_RECEPTIONS,
    ) -> None:
        self.window_ns = window_ns
        self.min_receptions = min_receptions

        # Active groups: keyed by (icao, raw_msg) -> CorrelationGroup
        self._buffer: dict[tuple[str, str], CorrelationGroup] = {}

        # Stats
        self.groups_formed = 0
        self.groups_emitted = 0
        self.groups_dropped = 0
        self.messages_processed = 0

    def process(self, packet: dict) -> list[dict]:
        """Process a decoded packet from Layer 2.

        Args:
            packet: Decoded JSONL dict with fields:
                icao, df_type, altitude_ft, squawk, raw_msg,
                sensor_id, lat, lon, alt, timestamp_s, timestamp_ns

        Returns:
            List of completed correlation groups (may be empty).
        """
        self.messages_processed += 1

        icao = packet.get("icao")
        raw_msg = packet.get("raw_msg")
        sensor_id = packet.get("sensor_id")

        if not icao or not raw_msg or sensor_id is None:
            return []

        reception = Reception(
            sensor_id=sensor_id,
            lat=packet.get("lat", 0.0),
            lon=packet.get("lon", 0.0),
            alt=packet.get("alt", 0.0),
            timestamp_s=packet.get("timestamp_s", 0),
            timestamp_ns=packet.get("timestamp_ns", 0),
        )

        current_time_ns = reception.abs_time_ns

        # First, flush any expired groups
        emitted = self._flush_expired(current_time_ns)

        # Look up or create group for this (icao, raw_msg)
        key = (icao, raw_msg)
        group = self._buffer.get(key)

        if group is not None:
            # Check if this reception is within the time window
            if current_time_ns - group.first_time_ns <= self.window_ns:
                # Skip duplicate sensor in same group
                if not group.has_sensor(sensor_id):
                    group.receptions.append(reception)
            else:
                # Window expired for this specific key; emit old, start new
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
