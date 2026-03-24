# Native C++ MLAT Pipeline

This README documents only the native Layer 4 and Layer 5 workflow, the live and recorded commands that are still part of that working stack, and the solver optimization journey that led to the current native C++ path.

## Scope

Run every command in this README from the repository root.

Current working stack:

- Layer 1: Go (`data-pipe`)
- Layer 2: Python (`modes-decoder`)
- Layer 3: Python (`correlation-engine`)
- Layer 4: Native C++ (`native-l45/build/mlat-solver-native`)
- Layer 5: Native C++ (`native-l45/build/track-builder-native`)
- Layer 6: Python (`live-map/server.py`)

This README does not use the removed Python Layer 4 solver.

## Private Local Files

Keep these files local and out of Git:

- `buyer-env`
- `data-pipe/location-overrides.txt`
- `data-pipe/correlation_groups_1h.jsonl`

`buyer-env` must contain your Neuron buyer credentials, seller list, and the buyer `port` value.

The native solver looks for `data-pipe/location-overrides.txt`, so the commands below assume the current working directory is the repo root.

## Build

### Python environment

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install \
  -r modes-decoder/requirements.txt \
  -r live-map/requirements.txt
```

### Go Layer 1 binary

```bash
go build -o data-pipe/mlat-pipe ./data-pipe
```

### Native Layer 4 and Layer 5 binaries

```bash
cmake -S native-l45 -B native-l45/build -DCMAKE_BUILD_TYPE=Release
cmake --build native-l45/build -j
```

## Live Layer 3 Capture

### Capture to `data-pipe/correlation_groups.jsonl`

```bash
bash run-pipeline.sh
```

This writes:

- `data-pipe/correlation_groups.jsonl`
- `data-pipe/pipe.log`
- `decoder.log`
- `correlator.log`

### Record one hour of live Layer 3 data

```bash
timeout 3600 bash run-pipeline.sh || test $? -eq 124
mv data-pipe/correlation_groups.jsonl data-pipe/correlation_groups_1h.jsonl
```

## Recorded Replay With Native Layer 4

```bash
cat data-pipe/correlation_groups_1h.jsonl \
| ./native-l45/build/mlat-solver-native 2>solver_native_1h.log \
> layer4_native_1h.jsonl
```

This writes:

- `layer4_native_1h.jsonl`
- `solver_native_1h.log`

## Recorded Replay With Native Layer 4 And Layer 5

```bash
cat layer4_native_1h.jsonl \
| ./native-l45/build/track-builder-native 2>tracker_native_1h.log \
> layer5_native_1h.jsonl
```

This writes:

- `layer5_native_1h.jsonl`
- `tracker_native_1h.log`

## Recorded Replay Through Layer 6

```bash
cat layer5_native_1h.jsonl \
| python3 live-map/server.py --host 127.0.0.1 --port 8080 2>livemap_native_1h.log
```

Open `http://127.0.0.1:8080`.

This writes:

- `livemap_native_1h.log`

## Full Live Native Chain

```bash
PORT=$(grep '^port=' buyer-env | cut -d= -f2)
./data-pipe/mlat-pipe \
  --port "$PORT" \
  --buyer-or-seller buyer \
  --mode peer \
  --list-of-sellers-source env \
  --envFile "$PWD/buyer-env" \
  2>data-pipe/pipe_live_native.log \
| python3 modes-decoder/main.py 2>decoder_live_native.log \
| python3 correlation-engine/main.py 2>correlator_live_native.log \
| ./native-l45/build/mlat-solver-native 2>solver_live_native.log \
| ./native-l45/build/track-builder-native 2>tracker_live_native.log \
| python3 live-map/server.py --host 127.0.0.1 --port 8080 2>livemap_live_native.log
```

Open `http://127.0.0.1:8080`.

This writes:

- `data-pipe/pipe_live_native.log`
- `decoder_live_native.log`
- `correlator_live_native.log`
- `solver_live_native.log`
- `tracker_live_native.log`
- `livemap_live_native.log`

## Optimization Journey

The solver path improved in stages before the native rewrite became the default run path.

User-provided historical progression:

- Early 30-minute capture at roughly `110k` groups: median residual around `50 m`, p95 around `200 m`, solves in the `20s` range.
- Later 42-minute run at roughly `121k` groups: median residual around `39 m`, p95 around `150 m`, solves in the `30s` range.
- Later tuned run on the same `121k`-group scale: median residual around `25 m`, p95 around `150 m`, solves in about `40s`.

Exact saved benchmark points used for the current baseline:

| Stage | Dataset | Solves | Solve rate | Median residual | p95 residual | Runtime |
|---|---:|---:|---:|---:|---:|---:|
| Python Layer 4 baseline | `data-pipe/correlation_groups_30min.jsonl` | `36,293 / 77,722` | `46.7%` | `18.19 m` | `133.04 m` | `3m37.741s` |
| Native C++ Layer 4 parity / optimized baseline | `data-pipe/correlation_groups_30min.jsonl` | `36,694-36,723 / 77,722` | about `47.2%` | about `16.5 m` | about `129-130 m` | about `1m08s` |

Additional saved native notes:

- The current native path preserved Layer 5 parity against the Python tracker on the same Layer 4 output.
- An optional `NATIVE_L45_DELEGATE_2SENSOR_MIN_SOLVES` tuning path reached about `49.9s` end-to-end on the 30-minute replay, but it reduced standalone Layer 4 solve rate, so it is not the default documented mode.
- A later 1-hour benchmark run showed the native Layer 4 path at about `11.3x` faster than the Python Layer 4 path while keeping solve quality essentially aligned.

## Notes

- `buyer-env`, `data-pipe/location-overrides.txt`, and the recorded JSONL captures should stay local.
- `bash run-pipeline.sh` now reads the root `buyer-env` directly; the old `data-pipe/.buyer-env` link is no longer required.
- The live-map server pushes snapshots every `1.0s` by default unless `MLAT_UPDATE_INTERVAL` is changed.
