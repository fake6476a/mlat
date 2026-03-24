#!/usr/bin/env python3
"""Read Layer 2 JSONL, group correlated receptions, and emit Layer 3 JSONL."""

import json
import os
import sys
import time

# Make sibling imports work regardless of the current working directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from correlator import Correlator

STATS_INTERVAL = 30  # seconds


def log(msg: str) -> None:
    """Log to stderr (keeps stdout clean for data)."""
    print(msg, file=sys.stderr, flush=True)


def main() -> None:
    # Read runtime configuration from environment variables.
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

            # Parse the incoming JSON packet.
            try:
                packet = json.loads(line)
            except (json.JSONDecodeError, TypeError):
                parse_errors += 1
                continue

            # Process the packet through the correlator.
            emitted = correlator.process(packet)

            # Emit completed groups as JSONL.
            for group in emitted:
                print(json.dumps(group, separators=(",", ":")), flush=True)

            # Print periodic stats.
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
        # Flush any remaining groups at end of stream.
        remaining = correlator.flush_all()
        for group in remaining:
            print(json.dumps(group, separators=(",", ":")), flush=True)

        stats = correlator.stats()
        log("Shutting down. Final stats:")
        log(f"[stats] {stats} parse_errors={parse_errors}")


if __name__ == "__main__":
    main()
