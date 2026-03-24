#!/usr/bin/env python3
"""Read Layer 4 fixes, update EKF tracks, and emit Layer 5 JSONL."""

import json
import os
import sys
import time

# Make sibling imports work regardless of the current working directory.
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

            # Parse the incoming fix.
            try:
                fix = json.loads(line)
            except (json.JSONDecodeError, TypeError):
                parse_errors += 1
                continue

            # Process the fix through the track manager.
            result = manager.process_fix(fix)

            if result is not None:
                # Emit the enriched track update.
                print(json.dumps(result, separators=(",", ":")), flush=True)

            now = time.monotonic()

            # Prune stale tracks periodically.
            if now - last_prune_time >= PRUNE_INTERVAL:
                pruned = manager.prune_stale()
                if pruned > 0:
                    log(f"[prune] Removed {pruned} stale tracks")
                last_prune_time = now

            # Emit periodic stats.
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
