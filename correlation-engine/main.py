#!/usr/bin/env python3
"""Layer 3: Correlation Engine

Reads Layer 2 JSONL from stdin, groups receptions of the same Mode-S
transmission by multiple sensors using time-windowed correlation, and
outputs correlation groups as JSONL to stdout.

Usage:
    python3 modes-decoder/main.py < data.jsonl | python3 correlation-engine/main.py

    Or full pipeline:
    ./data-pipe/mlat-pipe | python3 modes-decoder/main.py | python3 correlation-engine/main.py

Input (JSONL from Layer 2):
    {"icao":"4CA7E8","df_type":4,"altitude_ft":36000,"squawk":null,
     "raw_msg":"2000171806A983","sensor_id":1001,"lat":50.1,"lon":-5.7,
     "alt":100.0,"timestamp_s":43200,"timestamp_ns":500000000}

Output (JSONL):
    {"icao":"4CA7E8","df_type":4,"altitude_ft":36000,"squawk":null,
     "raw_msg":"2000171806A983","num_sensors":3,
     "receptions":[
       {"sensor_id":1001,"lat":50.1,"lon":-5.7,"alt":100.0,
        "timestamp_s":43200,"timestamp_ns":500000000},
       {"sensor_id":1002,"lat":50.2,"lon":-5.6,"alt":105.0,
        "timestamp_s":43200,"timestamp_ns":500000050},
       {"sensor_id":1003,"lat":50.3,"lon":-5.5,"alt":110.0,
        "timestamp_s":43200,"timestamp_ns":500000100}
     ]}

All logs go to stderr to keep stdout as a clean data stream.
"""

import json
import os
import sys
import time

# Ensure imports work regardless of CWD
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from correlator import Correlator

STATS_INTERVAL = 30  # seconds


def log(msg: str) -> None:
    """Log to stderr (keeps stdout clean for data)."""
    print(msg, file=sys.stderr, flush=True)


def main() -> None:
    # Configuration via environment variables
    window_ms = float(os.environ.get("MLAT_WINDOW_MS", "2.0"))
    min_receptions = int(os.environ.get("MLAT_MIN_RECEPTIONS", "2"))

    window_ns = int(window_ms * 1_000_000)

    log("=== MLAT Correlation Engine (Layer 3) ===")
    log(f"Window: {window_ms}ms ({window_ns} ns)")
    log(f"Min receptions per group: {min_receptions}")
    log(f"Stats logged every {STATS_INTERVAL}s to stderr")

    correlator = Correlator(window_ns=window_ns, min_receptions=min_receptions)

    parse_errors = 0
    last_stats_time = time.monotonic()

    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue

            # Parse input JSON
            try:
                packet = json.loads(line)
            except (json.JSONDecodeError, TypeError):
                parse_errors += 1
                continue

            # Process through correlator
            emitted = correlator.process(packet)

            # Output completed groups
            for group in emitted:
                print(json.dumps(group, separators=(",", ":")), flush=True)

            # Periodic stats
            now = time.monotonic()
            if now - last_stats_time >= STATS_INTERVAL:
                stats = correlator.stats()
                log(f"[stats] {stats} parse_errors={parse_errors}")
                last_stats_time = now

    except KeyboardInterrupt:
        log("Interrupted by user")
    except BrokenPipeError:
        pass
    finally:
        # Flush remaining groups at end of stream
        remaining = correlator.flush_all()
        for group in remaining:
            print(json.dumps(group, separators=(",", ":")), flush=True)

        stats = correlator.stats()
        log("Shutting down. Final stats:")
        log(f"[stats] {stats} parse_errors={parse_errors}")


if __name__ == "__main__":
    main()
