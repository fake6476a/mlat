#!/usr/bin/env python3
"""Layer 2: Mode-S Decoder

Reads Layer 1 JSONL from stdin, decodes each Mode-S message using pyModeS,
and outputs enriched JSONL to stdout.

Usage:
    ./data-pipe/mlat-pipe | python3 modes-decoder/main.py

Input (JSONL from Layer 1):
    {"sensor_id":1234,"lat":50.1,"lon":-5.7,"alt":100.0,
     "timestamp_s":43200,"timestamp_ns":500000000,"raw_msg":"2000171806a983"}

Output (JSONL):
    {"icao":"4CA7E8","df_type":4,"altitude_ft":36000,"squawk":null,
     "raw_msg":"2000171806A983",
     "sensor_id":1234,"lat":50.1,"lon":-5.7,"alt":100.0,
     "timestamp_s":43200,"timestamp_ns":500000000}

All logs go to stderr to keep stdout as a clean data stream.
"""

import json
import os
import sys
import time

# Ensure imports work regardless of CWD
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from decoder import decode_message

# Stats counters
stats = {
    "received": 0,
    "decoded": 0,
    "failed_parse": 0,
    "failed_decode": 0,
}

# Per-DF-type counters
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

    # Parse input JSON
    try:
        packet = json.loads(line)
    except (json.JSONDecodeError, TypeError):
        stats["failed_parse"] += 1
        return

    raw_msg = packet.get("raw_msg")
    if not raw_msg:
        stats["failed_decode"] += 1
        return

    # Decode the Mode-S message
    decoded = decode_message(raw_msg)
    if decoded is None:
        stats["failed_decode"] += 1
        return

    stats["decoded"] += 1

    # Track DF type distribution
    df = decoded["df_type"]
    df_counts[df] = df_counts.get(df, 0) + 1

    # Build output: decoded fields + sensor metadata from Layer 1
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

    # Write to stdout as JSONL
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

            # Periodic stats
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
