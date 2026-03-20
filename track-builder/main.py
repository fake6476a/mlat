#!/usr/bin/env python3
"""Layer 5: Track Builder & Filter

Reads Layer 4 JSONL position fixes from stdin, associates them with
continuous per-aircraft tracks using an Extended Kalman Filter (EKF),
filters bad data via innovation gating, and outputs enriched track
updates as JSONL to stdout.

Track Builder pipeline:
1. Receive position fix from Layer 4
2. Look up existing track by ICAO address (or create new track)
3. EKF predict to measurement time
4. Innovation gating: reject if Mahalanobis distance > threshold
5. EKF update with accepted measurement
6. Compute derived quantities (heading, speed, vertical rate)
7. Output enriched track update

Usage:
    Full pipeline (Layer 1 → 2 → 3 → 4 → 5):
    ./data-pipe/mlat-pipe | python3 modes-decoder/main.py | \\
        python3 correlation-engine/main.py | python3 mlat-solver/main.py | \\
        python3 track-builder/main.py

    Or replay from saved Layer 4 output:
    cat solved_fixes.jsonl | python3 track-builder/main.py

Input (JSONL from Layer 4):
    {"icao":"4CA7E8","lat":50.15,"lon":-5.65,"alt_ft":36000,
     "residual_m":145.23,"gdop":3.45,"num_sensors":3,
     "solve_method":"constrained_3sensor",
     "timestamp_s":43200,"timestamp_ns":500000000,
     "df_type":4,"squawk":null,"raw_msg":"2000171806A983",
     "t0_s":43200.000499833}

Output (JSONL):
    {"icao":"4CA7E8","lat":50.15,"lon":-5.65,"alt_ft":36000,
     "heading_deg":45.2,"speed_kts":450.3,"vrate_fpm":0,
     "track_quality":5,"positions_count":5,
     "residual_m":145.23,"gdop":3.45,"num_sensors":3,
     "solve_method":"constrained_3sensor",
     "timestamp_s":43200,"timestamp_ns":500000000,
     "df_type":4,"squawk":null,"raw_msg":"2000171806A983",
     "t0_s":43200.000499833}

All logs go to stderr to keep stdout as a clean data stream.
"""

import json
import os
import sys
import time

# Ensure imports work regardless of CWD
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tracker import TrackManager

STATS_INTERVAL = 30  # seconds
PRUNE_INTERVAL = 60  # seconds


def log(msg: str) -> None:
    """Log to stderr (keeps stdout clean for data)."""
    print(msg, file=sys.stderr, flush=True)


def main() -> None:
    log("=== MLAT Track Builder (Layer 5) ===")
    log("Reading JSONL position fixes from stdin")
    log("EKF: 6-state constant-velocity [x, y, z, vx, vy, vz]")
    log("Innovation gate: Chi-squared 3-DOF at 99.7% confidence")
    log(f"Stats logged every {STATS_INTERVAL}s to stderr")
    log(f"Stale track pruning every {PRUNE_INTERVAL}s")

    manager = TrackManager()
    parse_errors = 0
    last_stats_time = time.monotonic()
    last_prune_time = time.monotonic()

    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue

            # Parse input JSON
            try:
                fix = json.loads(line)
            except (json.JSONDecodeError, TypeError):
                parse_errors += 1
                continue

            # Process through track manager
            result = manager.process_fix(fix)

            if result is not None:
                # Output enriched track update
                print(json.dumps(result, separators=(",", ":")), flush=True)

            now = time.monotonic()

            # Periodic stale track pruning
            if now - last_prune_time >= PRUNE_INTERVAL:
                pruned = manager.prune_stale()
                if pruned > 0:
                    log(f"[prune] Removed {pruned} stale tracks")
                last_prune_time = now

            # Periodic stats
            if now - last_stats_time >= STATS_INTERVAL:
                stats = manager.stats()
                stats["parse_errors"] = parse_errors
                log(f"[stats] {json.dumps(stats)}")
                last_stats_time = now

    except KeyboardInterrupt:
        log("Interrupted by user")
    except BrokenPipeError:
        pass
    finally:
        log("Shutting down. Final stats:")
        stats = manager.stats()
        stats["parse_errors"] = parse_errors
        log(f"[stats] {json.dumps(stats, indent=2)}")


if __name__ == "__main__":
    main()
