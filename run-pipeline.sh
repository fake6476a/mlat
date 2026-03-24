#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR/data-pipe"

PORT=$(grep '^port=' "$ROOT_DIR/buyer-env" | cut -d= -f2)
if [ -z "$PORT" ]; then
    echo "ERROR: port not found in buyer-env" >&2
    exit 1
fi

./mlat-pipe \
    --port "$PORT" \
    --buyer-or-seller buyer \
    --mode peer \
    --list-of-sellers-source env \
    --envFile "$ROOT_DIR/buyer-env" \
    2>pipe.log \
| python3 ../modes-decoder/main.py 2>../decoder.log \
| python3 ../correlation-engine/main.py 2>../correlator.log \
> correlation_groups.jsonl
