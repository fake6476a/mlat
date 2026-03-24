# Native C++ MLAT Pipeline — Benchmarking & Usage

A 6-layer multilateration pipeline that takes raw ADS-B sensor feeds and produces real-time aircraft tracks on a live map. Layers 4 and 5 are native C++ for maximum throughput.

## Pipeline Architecture

```
Layer 1  Go data-pipe          Raw sensor feed ingestion
Layer 2  Python modes-decoder  ADS-B frame decoding
Layer 3  Python correlation    TOA correlation grouping
Layer 4  C++ mlat-solver       Multilateration position solving
Layer 5  C++ track-builder     EKF-based track filtering
Layer 6  Python live-map       Browser-based live map
```

All layers are connected via Unix pipes (stdin/stdout JSONL). Layers 4+5 run in parallel and fully utilise both CPU cores.

Run every command in this README from the repository root.

## Pipeline Flow

```
                          ┌─────────────────────────────────────┐
                          │        Raw ADS-B Sensor Feed        │
                          └──────────────┬──────────────────────┘
                                         │
                          ┌──────────────▼──────────────────────┐
                          │  Layer 1 — Go data-pipe             │
                          │  Ingest raw sensor data via Neuron  │
                          └──────────────┬──────────────────────┘
                                         │  stdout JSONL
                          ┌──────────────▼──────────────────────┐
                          │  Layer 2 — Python modes-decoder     │
                          │  Decode ADS-B frames (DF17, etc.)   │
                          └──────────────┬──────────────────────┘
                                         │  stdout JSONL
                          ┌──────────────▼──────────────────────┐
                          │  Layer 3 — Python correlation       │
                          │  Group receptions by TOA into       │
                          │  correlation groups                 │
                          └──────────────┬──────────────────────┘
                                         │  stdout JSONL (groups)
                          ┌──────────────▼──────────────────────┐
                          │  Layer 4 — C++ mlat-solver          │
                          │  TOA multilateration solving        │
                          │  (LM optimizer, clock cal, GDOP)    │
                          └──────────────┬──────────────────────┘
                                         │  stdout JSONL (fixes + unsolved)
                          ┌──────────────▼──────────────────────┐
                          │  Layer 5 — C++ track-builder        │
                          │  EKF track filtering & prediction   │
                          │  (Chi² gating, adaptive noise)      │
                          └──────────────┬──────────────────────┘
                                         │  stdout JSONL (tracks)
                          ┌──────────────▼──────────────────────┐
                          │  Layer 6 — Python live-map          │
                          │  Browser-based live aircraft map    │
                          └─────────────────────────────────────┘
```

---

## 1. Build

### 1.1 Python environment

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install \
  -r modes-decoder/requirements.txt \
  -r live-map/requirements.txt
```

### 1.2 Go Layer 1 binary

```bash
go build -o data-pipe/mlat-pipe ./data-pipe
```

### 1.3 Native Layer 4 + Layer 5 binaries

```bash
cmake -S native-l45 -B native-l45/build -DCMAKE_BUILD_TYPE=Release
cmake --build native-l45/build -j
```

---

## 2. Location Overrides

The file `data-pipe/location-overrides.txt` maps sensor public keys to known geographic positions (lat, lon, alt) and human-readable names. The native solver loads this file automatically on startup.

Location overrides improve solve quality significantly (see [Benchmark Results](#5-benchmark-results--1-hour-dataset)).

To update the sensor list, edit `data-pipe/location-overrides.txt`. The format is a JSON array:

```json
[
  {
    "public_key": "021a29e7...",
    "lat": 50.09916,
    "lon": -5.55674,
    "alt": 153.8,
    "name": "ne073 Penzance"
  }
]
```

---

## 3. Running With Real Live Data

You need a `buyer-env` file (not checked in) with your Neuron buyer credentials, seller list, and the buyer `port` value.

### 3.1 Capture Layer 3 correlation groups (live)

```bash
bash run-pipeline.sh
```

This runs Layers 1-3 and writes correlation groups to `data-pipe/correlation_groups.jsonl`.

Output logs: `data-pipe/pipe.log`, `decoder.log`, `correlator.log`.

### 3.2 Record a fixed duration of live data

Record 1 hour of Layer 3 data for later replay:

```bash
timeout 3600 bash run-pipeline.sh || test $? -eq 124
mv data-pipe/correlation_groups.jsonl data-pipe/correlation_groups_1h.jsonl
```

Adjust `3600` (seconds) for shorter or longer captures.

### 3.3 Full live pipeline (Layers 1-6)

Run all layers end-to-end with live sensor data and open the live map:

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

Open `http://127.0.0.1:8080` in a browser.

---

## 4. Running on Captured / Recorded Data

Use a previously recorded JSONL file (e.g. `data-pipe/correlation_groups_1h.jsonl`) to replay through the native solver and tracker without needing live sensors.

### 4.1 Layer 4 only — MLAT solver

```bash
cat data-pipe/correlation_groups_1h.jsonl \
| ./native-l45/build/mlat-solver-native 2>solver_native_1h.log \
> layer4_native_1h.jsonl
```

Output: `layer4_native_1h.jsonl` (solved positions + passthrough), `solver_native_1h.log` (JSON stats on stderr).

### 4.2 Layer 4 + Layer 5 — solver + track builder

```bash
cat data-pipe/correlation_groups_1h.jsonl \
| ./native-l45/build/mlat-solver-native 2>solver_native_1h.log \
| ./native-l45/build/track-builder-native 2>tracker_native_1h.log \
> layer5_native_1h.jsonl
```

Output: `layer5_native_1h.jsonl` (EKF-filtered tracks), `tracker_native_1h.log` (JSON stats on stderr).

### 4.3 Full replay through Layer 6 (live map)

```bash
cat data-pipe/correlation_groups_1h.jsonl \
| ./native-l45/build/mlat-solver-native 2>solver_native_1h.log \
| ./native-l45/build/track-builder-native 2>tracker_native_1h.log \
| python3 live-map/server.py --host 127.0.0.1 --port 8080 2>livemap_native_1h.log
```

Open `http://127.0.0.1:8080`.

### 4.4 Timed benchmark run (5 iterations)

```bash
for i in 1 2 3 4 5; do
  /usr/bin/time -v bash -c \
    'cat data-pipe/correlation_groups_1h.jsonl \
     | ./native-l45/build/mlat-solver-native 2>/dev/null \
     | ./native-l45/build/track-builder-native 2>/dev/null \
     > /dev/null' 2>&1 | grep -E "wall clock|Maximum resident"
done
```

### 4.5 Reading benchmark stats from logs

Both the solver and tracker write a JSON summary to stderr at the end of each run. Key fields:

**Solver log** (`solver_native_1h.log`):
- `groups_received`, `groups_solved`, `solve_rate`
- `median_residual_m`, `p95_residual_m`
- `location_overrides.matched_sensors`

**Tracker log** (`tracker_native_1h.log`):
- `fixes_received`, `fixes_accepted`, `accept_rate`
- `tracks_created`, `tracks_established`

---

## 5. Benchmark Results — 1-Hour Dataset

**Dataset:** `data-pipe/correlation_groups_1h.jsonl` — 165,917 correlation groups (57 MB)
**System:** AMD EPYC (2 cores), GCC 11.4.0, Release + LTO, `-O3 -march=native`
**Location overrides:** 9 sensors matched from `data-pipe/location-overrides.txt`

### 5.1 Performance

| Component | Wall Time (avg of 5 runs) | Throughput |
|---|---:|---:|
| Full pipeline (L4+L5 piped) | 3.77 s | ~44,000 groups/s |
| **Realtime factor** | | **~955x** |

1 hour of captured data processes in under 4 seconds.

### 5.2 Solver Quality (Layer 4)

| Metric | Value |
|---|---:|
| Groups received | 165,917 |
| Groups solved | 84,558 |
| Solve rate | 51% |
| Median residual | 18.95 m |
| p95 residual | 140.9 m |

Solve method breakdown: 98.9% prior_2sensor, 1.0% constrained_3sensor, 0.1% other.

### 5.3 Track Builder Quality (Layer 5)

| Metric | Value |
|---|---:|
| Fixes received | 154,301 |
| Fixes accepted | 43,246 |
| Fixes rejected (EKF gate) | 49,656 |
| Rejected (2-sensor quality) | 34,858 |
| Accept rate | 28% |
| Unique aircraft tracked | 120 |
| Established tracks | 120 |

### 5.4 Optimization History

| Stage | Dataset | Solve Rate | Median Residual | p95 Residual | Runtime |
|---|---|---:|---:|---:|---:|
| Python L4 baseline | 30-min (77k groups) | 46.7% | 18.19 m | 133.04 m | 3m38s |
| Native C++ L4 | 30-min (77k groups) | 47.2% | 16.5 m | 129-130 m | 1m08s |
| Native C++ L4+L5 | 1-hour (166k groups) | 51% | 18.95 m | 140.9 m | 3.77s |

---

## 6. Private Local Files

Keep these files local and out of Git:

- `buyer-env` — Neuron buyer credentials, seller list, and buyer port
- `data-pipe/correlation_groups_1h.jsonl` — recorded captures (large)

## 7. Notes

- `bash run-pipeline.sh` reads the root `buyer-env` directly; the old `data-pipe/.buyer-env` symlink is no longer required.
- The live-map server pushes snapshots every `1.0s` by default unless `MLAT_UPDATE_INTERVAL` is set.
- The native solver auto-detects `data-pipe/location-overrides.txt` on startup. If the file is missing, the solver runs without overrides.
