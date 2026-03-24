#!/bin/bash
# Run the Layer 1→3 MLAT pipeline and capture correlation groups.

set -euo pipefail

cd "$(dirname "$0")/data-pipe"

# Read the port from the buyer env file because the SDK needs it on the CLI.
PORT=$(grep '^port=' .buyer-env | cut -d= -f2)
if [ -z "$PORT" ]; then
    echo "ERROR: port not found in .buyer-env" >&2
    exit 1
fi

echo "Starting pipeline with port=$PORT" >&2

./mlat-pipe \
    --port "$PORT" \
    --buyer-or-seller buyer \
    --mode peer \
    --list-of-sellers-source env \
    --envFile .buyer-env \
    2>pipe.log \
| python3 ../modes-decoder/main.py 2>../decoder.log \
| python3 ../correlation-engine/main.py 2>../correlator.log \
> correlation_groups.jsonl
