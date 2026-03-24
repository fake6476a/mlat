#!/usr/bin/env python3
"""Read Layer 1 JSONL, decode Mode-S frames, and emit enriched JSONL."""

import json
import os
import sys
import time

# Make sibling imports work regardless of the current working directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from decoder import decode_message

# Track decoder statistics.
stats = {
    "received": 0,
    "decoded": 0,
    "failed_parse": 0,
    "failed_decode": 0,
}

# Track counts per DF type.
df_counts: dict[int, int] = {}

STATS_INTERVAL = 30  # seconds


def log(msg: str) -> None:
    """Log to stderr (keeps stdout clean for data)."""
    print(msg, file=sys.stderr, flush=True)


def print_stats() -> None:
    """Print decoding statistics to stderr."""
    total = stats["received"]
    decoded = stats["decoded"]
    rate = (decoded / total * 100) if total > 0 else 0.0
    log(
        f"[stats] received={total} decoded={decoded} "
        f"({rate:.1f}%) failed_parse={stats['failed_parse']} "
        f"failed_decode={stats['failed_decode']} df_types={dict(df_counts)}"
    )


def process_line(line: str) -> None:
    """Process a single JSONL line from Layer 1."""
    stats["received"] += 1

    # Parse the input JSON.
    try:
        packet = json.loads(line)
    except (json.JSONDecodeError, TypeError):
        stats["failed_parse"] += 1
        return

    raw_msg = packet.get("raw_msg")
    if not raw_msg:
        stats["failed_decode"] += 1
        return

    # Decode the Mode-S message.
    decoded = decode_message(raw_msg)
    if decoded is None:
        stats["failed_decode"] += 1
        return

    stats["decoded"] += 1

    # Update the DF type histogram.
    df = decoded["df_type"]
    df_counts[df] = df_counts.get(df, 0) + 1

    # Combine decoded fields with the original sensor metadata.
    output = {
        "icao": decoded["icao"],
        "df_type": decoded["df_type"],
        "altitude_ft": decoded["altitude_ft"],
        "squawk": decoded["squawk"],
        "raw_msg": decoded["raw_msg"],
        "sensor_id": packet.get("sensor_id"),
        "lat": packet.get("lat"),
        "lon": packet.get("lon"),
        "alt": packet.get("alt"),
        "timestamp_s": packet.get("timestamp_s"),
        "timestamp_ns": packet.get("timestamp_ns"),
    }

    # Emit the decoded packet as JSONL.
    print(json.dumps(output, separators=(",", ":")), flush=True)


def main() -> None:
    log("=== MLAT Mode-S Decoder (Layer 2) ===")
    log("Reading JSONL from stdin, outputting decoded JSONL to stdout")
    log(f"Stats logged every {STATS_INTERVAL}s to stderr")

    last_stats_time = time.monotonic()

    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue

            process_line(line)

            # Print periodic stats.
            now = time.monotonic()
            if now - last_stats_time >= STATS_INTERVAL:
                print_stats()
                last_stats_time = now

    except KeyboardInterrupt:
        log("Interrupted by user")
    except BrokenPipeError:
        pass
    finally:
        log("Shutting down. Final stats:")
        print_stats()


if __name__ == "__main__":
    main()
