# MLAT - Multilateration Pipeline

Real-time and replayable multilateration pipeline for Mode-S and ADS-B data from Neuron's 4DSky network. The repository is split into six layers that exchange JSONL over stdin and stdout, so you can capture upstream data once and replay downstream stages repeatedly.

## Repository Layout

| Layer | Directory | Language | Role |
|---|---|---|---|
| 1 | `data-pipe` | Go | Buyer node that receives raw seller packets from the Neuron network |
| 2 | `modes-decoder` | Python | Decodes raw Mode-S hex into ICAO, DF, altitude, and squawk |
| 3 | `correlation-engine` | Python | Groups same-aircraft same-frame receptions across sensors |
| 4 | `mlat-solver` | Python | Applies overrides, clock correction, cache seeding, and MLAT solving |
| 5 | `track-builder` | Python | Builds EKF-smoothed tracks and prediction-aids some 2-sensor groups |
| 6 | `live-map` | Python + browser JS | FastAPI/WebSocket backend with MapLibre GL JS + Deck.gl frontend |

## Current Behavior

- `./run-pipeline.sh` runs only Layers 1 → 3 and writes `data-pipe/correlation_groups.jsonl`.
- Layer 4 consumes Layer 3 groups, solves what it can, and also forwards some unresolved 2-sensor altitude groups as `unsolved_group` records for Layer 5.
- Layer 5 smooths solved fixes, derives kinematics, and can recover some unresolved 2-sensor cases using established-track prediction.
- Layer 6 is a FastAPI server plus a static HTML and JavaScript frontend. It is not React.
- All stages keep `stdout` machine-readable and write logs and stats to `stderr`.

## Requirements

- **Go 1.23+** for `data-pipe` (`go.mod` currently targets `go 1.23.2`)
- **Python 3.10+** for Layers 2–6
- **Neuron buyer credentials and seller public keys**
- **A machine reachable by sellers** or equivalent forwarding and tunnel setup for the buyer node
- **Optional QUIC buffer tuning** if your kernel limits are too low:

```bash
sudo sysctl -w net.core.rmem_max=7500000
sudo sysctl -w net.core.wmem_max=7500000
```

## Install Dependencies

### Go

```bash
go build -o data-pipe/mlat-pipe ./data-pipe
```

### Python

The safest install is the combined one, because `track-builder` and `live-map` import shared modules from `mlat-solver`:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install \
  -r modes-decoder/requirements.txt \
  -r mlat-solver/requirements.txt \
  -r track-builder/requirements.txt \
  -r live-map/requirements.txt
```

Current per-directory requirements:

| Directory | Packages |
|---|---|
| `modes-decoder` | `pyModeS` |
| `mlat-solver` | `numpy`, `scipy`, `numba` |
| `track-builder` | `numpy`, `scipy` |
| `live-map` | `fastapi`, `uvicorn`, `websockets` |

## Configure Credentials and Sensor Overrides

The repo currently keeps buyer credentials in the root-level `buyer-env` file. Replace it with your own values and keep the `data-pipe` links pointed at it:

```bash
ln -sf ../buyer-env data-pipe/.buyer-env
ln -sf ../buyer-env data-pipe/.env
```

Expected keys:

```text
eth_rpc_url=...
hedera_evm_id=...
hedera_id=...
location={"lat":...,"lon":...,"alt":...}
mirror_api_url=...
private_key=...
smart_contract_address=...
list_of_sellers=<comma-separated seller public keys>
port=61339
buyer_or_seller=buyer
```

The repo already includes `data-pipe/location-overrides.txt`, which is the current Cornwall sensor override map used by the solver. Replace it if you are running against a different deployment. Each entry is a JSON object with `public_key`, `lat`, `lon`, `alt`, and `name`.

> **Critical SDK gotcha:** the Neuron SDK parses some flags during `init()`, before environment files are loaded, so `--port`, `--buyer-or-seller`, `--mode`, `--list-of-sellers-source`, and `--envFile` must be supplied on the command line. `run-pipeline.sh` handles that for Layers 1 → 3.

## Run the Pipeline

### Capture Layer 3 correlation groups

```bash
./run-pipeline.sh
```

This builds the live stream through Layers 1 → 3 and writes:

- `data-pipe/correlation_groups.jsonl`
- `data-pipe/pipe.log`
- `decoder.log`
- `correlator.log`

### Replay Layers 4 → 6 from captured Layer 3 data

Run just the solver:

```bash
cat data-pipe/correlation_groups.jsonl | python3 mlat-solver/main.py 2>solver.log > layer4.jsonl
```

Build tracks from Layer 4 output:

```bash
cat layer4.jsonl | python3 track-builder/main.py 2>tracker.log > layer5.jsonl
```

Serve the live map from replayed data:

```bash
cat layer4.jsonl | python3 track-builder/main.py 2>tracker.log | python3 live-map/server.py 2>livemap.log
```

Then open `http://localhost:8080`.

### Manual Layer 1 → 3 command

If you do not want to use `run-pipeline.sh`, the equivalent command is:

```bash
cd data-pipe
./mlat-pipe \
    --port 61339 \
    --buyer-or-seller buyer \
    --mode peer \
    --list-of-sellers-source env \
    --envFile .buyer-env \
    2>pipe.log \
| python3 ../modes-decoder/main.py 2>../decoder.log \
| python3 ../correlation-engine/main.py 2>../correlator.log \
> correlation_groups.jsonl
```

## Layer 1: Data Pipe

`data-pipe/main.go` runs the Neuron buyer node, accepts seller libp2p streams, parses the binary wire format, and emits one JSON packet per received Mode-S frame.

Output fields:

| Field | Description |
|---|---|
| `sensor_id` | Seller sensor identifier from the packet header |
| `lat`, `lon`, `alt` | Seller-reported sensor position |
| `timestamp_s`, `timestamp_ns` | Reception timestamp split into seconds and nanoseconds |
| `raw_msg` | Raw Mode-S message as uppercase hex |

Example:

```json
{"sensor_id":1234,"lat":50.12993,"lon":-5.5137,"alt":56.3,"timestamp_s":43200,"timestamp_ns":500000000,"raw_msg":"8DA4C2B658B9..."}
```

## Layer 2: Mode-S Decoder

`modes-decoder/main.py` reads Layer 1 JSONL and enriches it with decoded Mode-S fields using `pyModeS`.

Current decoder behavior:

- Accepts both 56-bit and 112-bit Mode-S messages.
- Decodes ICAO, DF type, squawk, and altitude when available.
- Extracts altitude for DF0, DF4, DF16, and DF20 via the AC field.
- Extracts airborne-position altitude for DF17 type codes 9–18 from the ADS-B ME field.
- Logs rolling stats to `stderr` every 30 seconds.

Currently handled DF types are 0, 4, 5, 11, 16, 17, 18, 20, and 21.

Output fields:

| Field | Description |
|---|---|
| `icao` | Aircraft ICAO address |
| `df_type` | Mode-S downlink format |
| `altitude_ft` | Decoded altitude when present |
| `squawk` | Decoded squawk for identity replies |
| `raw_msg` | Original raw message |
| `sensor_id`, `lat`, `lon`, `alt`, `timestamp_s`, `timestamp_ns` | Passed through from Layer 1 |

Example:

```json
{"icao":"4CA7E8","df_type":17,"altitude_ft":36000,"squawk":null,"raw_msg":"8D4CA7E858B9...","sensor_id":1001,"lat":50.1,"lon":-5.7,"alt":100.0,"timestamp_s":43200,"timestamp_ns":500000000}
```

## Layer 3: Correlation Engine

`correlation-engine/main.py` groups receptions that are from the same aircraft and the same radio frame and that arrive within a configured time window.

Current behavior:

- Correlation key: `(icao, raw_msg)`
- Default window: `MLAT_WINDOW_MS=2.0`
- Default minimum group size: `MLAT_MIN_RECEPTIONS=2`
- Flushes complete groups as JSONL and prints stats to `stderr` every 30 seconds

Environment variables:

| Variable | Default | Description |
|---|---|---|
| `MLAT_WINDOW_MS` | `2.0` | Correlation window in milliseconds |
| `MLAT_MIN_RECEPTIONS` | `2` | Minimum sensors required before emitting a group |

Output shape:

```json
{"icao":"4CA7E8","df_type":17,"altitude_ft":36000,"squawk":null,"raw_msg":"8D4CA7E858B9...","num_sensors":3,"receptions":[{"sensor_id":1001,"lat":50.1,"lon":-5.7,"alt":100.0,"timestamp_s":43200,"timestamp_ns":500000000},{"sensor_id":1002,"lat":50.2,"lon":-5.6,"alt":105.0,"timestamp_s":43200,"timestamp_ns":500000050},{"sensor_id":1003,"lat":50.3,"lon":-5.5,"alt":110.0,"timestamp_s":43200,"timestamp_ns":500000100}]}
```

## Layer 4: MLAT Solver

`mlat-solver/main.py` is the highest-leverage stage in the repo. It does more than geometric solving:

- Loads `data-pipe/location-overrides.txt` and replaces seller-reported coordinates with corrected sensor positions.
- Uses ADS-B DF17 position and velocity messages to seed a per-ICAO cache.
- Maintains receiver-pair clock calibration using ADS-B references.
- Routes groups by sensor count into the appropriate solver path.
- Emits solved fixes as normal Layer 4 records.
- Also emits `{"unsolved_group": ...}` records for some 2-sensor altitude groups so Layer 5 can try prediction-aided recovery.

Current solve routing:

| Case | Current path |
|---|---|
| 5+ sensors | `inamdar_5sensor` init plus iterative refinement |
| 4 sensors + altitude | `inamdar_4sensor_alt` |
| 3 sensors + altitude | `constrained_3sensor` |
| 2 sensors + altitude + prior | Prior-aided `solve_toa` |
| 2 sensors + altitude without usable prior | Forwarded as `unsolved_group` |

Current validation gates in `solver.py` include:

- Sensor range limit: 500 km
- GDOP limit: 20.0
- General residual limit: 10,000 m
- Prior-aided 2-sensor residual limit: 200 m quality gate
- Prior drift limits for prediction-aided cases

Solved output fields:

| Field | Description |
|---|---|
| `icao`, `lat`, `lon`, `alt_ft` | Solved aircraft state |
| `residual_m` | Solver residual reported for the solve |
| `quality_residual_m` | Quality metric used downstream for filtering and EKF measurement noise |
| `gdop` | Geometry score for the solve |
| `num_sensors` | Sensors used in the solve |
| `solve_method` | Method label such as `inamdar_5sensor` or `constrained_3sensor` |
| `timestamp_s`, `timestamp_ns` | Reference reception timestamp |
| `df_type`, `squawk`, `raw_msg`, `t0_s` | Metadata carried through from the correlation group |

Solved example:

```json
{"icao":"4CA7E8","lat":50.150000,"lon":-5.500000,"alt_ft":36000.0,"residual_m":24.8,"quality_residual_m":24.8,"gdop":4.5,"num_sensors":5,"solve_method":"inamdar_5sensor","timestamp_s":43200,"timestamp_ns":47198,"df_type":17,"squawk":null,"raw_msg":"8D4CA7E858B9...","t0_s":43199.999999988}
```

Forwarded 2-sensor example:

```json
{"unsolved_group":{"icao":"4CA7E8","num_sensors":2,"receptions":[...]}}
```

### Module Structure

| Module | Description | Reference |
|--------|-------------|-----------|
| `geo.py` | WGS84 ↔ ECEF coordinate conversions | — |
| `atmosphere.py` | Markochev atmospheric refraction model | Part 13.2 |
| `gdop.py` | GDOP computation |
| `inamdar.py` | Algebraic initializers for altitude-constrained cases |
| `frisch.py` | Frisch TOA refinement and constrained 3-sensor solve |
| `clock_sync.py` | Receiver-pair offset and drift calibration |
| `adsb_decoder.py` | DF17 CPR position and velocity helpers |
| `position_cache.py` | Per-ICAO cache for position and velocity priors |
| `solver.py` | Routing, validation, and solve packaging |
| `main.py` | Layer 4 stdin and stdout entrypoint |

## Layer 5: Track Builder

`track-builder/main.py` consumes the mixed Layer 4 stream, updates EKF-backed per-ICAO tracks, and emits smoothed track updates.

Current behavior:

- Maintains one 6-state constant-velocity EKF per ICAO.
- Accepts normal solved Layer 4 fixes.
- Intercepts `unsolved_group` records and attempts `prediction_aided_2sensor` recovery when an established track exists.
- Derives EKF measurement noise from `quality_residual_m * gdop`, clamped into a bounded range.
- Prunes stale tracks every 60 seconds and removes tracks older than 300 seconds.

Derived fields in each emitted track update:

| Field | Description |
|---|---|
| `heading_deg` | Ground-track heading |
| `speed_kts` | Ground speed |
| `vrate_fpm` | Vertical rate |
| `track_quality` | Number of accepted EKF updates |
| `positions_count` | Stored point count for the track |
| `cov_matrix` | 2x2 local ENU covariance matrix used by the live map |

Example output:

```json
{"icao":"4CA7E8","lat":50.159600,"lon":-5.640400,"alt_ft":36000,"heading_deg":32.7,"speed_kts":450.2,"vrate_fpm":-200,"track_quality":12,"positions_count":20,"residual_m":24.8,"quality_residual_m":24.8,"gdop":3.2,"num_sensors":4,"solve_method":"inamdar_4sensor_alt","timestamp_s":43201,"timestamp_ns":500000000,"df_type":17,"squawk":null,"raw_msg":"8D4CA7E858B9...","t0_s":43201.000499833,"cov_matrix":[[1200.0,15.0],[15.0,900.0]]}
```

## Layer 6: Live Map

`live-map/server.py` serves the browser UI and reads Layer 5 JSONL from `stdin` in a background thread.

Current stack:

| Component | Current implementation |
|---|---|
| Backend | FastAPI + `uvicorn` |
| Frontend | Static `live-map/static/index.html` |
| Map | MapLibre GL JS |
| Rendering overlays | Deck.gl |
| Push transport | WebSocket |

Current server behavior:

- Stores the latest track per ICAO.
- Keeps the last 50 trail points for each aircraft.
- Broadcasts snapshot-style WebSocket updates every `MLAT_UPDATE_INTERVAL` seconds.
- Prunes stale aircraft after 300 seconds.
- Serves a precomputed Cornwall GDOP overlay from `/api/gdop_grid`.

Environment variables:

| Variable | Default | Description |
|---|---|---|
| `MLAT_MAP_HOST` | `0.0.0.0` | Default bind address |
| `MLAT_MAP_PORT` | `8080` | Default bind port |
| `MLAT_UPDATE_INTERVAL` | `1.0` | Seconds between WebSocket snapshot pushes |

CLI flags:

```bash
python3 live-map/server.py --host 0.0.0.0 --port 8080
```

There is currently no `--demo` mode in the code.

Endpoints:

| Endpoint | Description |
|---|---|
| `GET /` | Main live map page |
| `GET /api/aircraft` | Current snapshot of all aircraft |
| `GET /api/gdop_grid` | Cornwall GDOP overlay grid |
| `WS /ws` | Periodic snapshot stream |

Snapshot payload shape:

```json
{
  "type": "update",
  "timestamp": 1711234567.89,
  "aircraft": [
    {
      "icao": "4CA7E8",
      "lat": 50.15,
      "lon": -5.65,
      "alt_ft": 36000,
      "heading_deg": 32.7,
      "speed_kts": 450.2,
      "vrate_fpm": -200,
      "track_quality": 12,
      "cov_matrix": [[1200.0, 15.0], [15.0, 900.0]],
      "trail": [[-5.70, 50.10], [-5.68, 50.12], [-5.65, 50.15]]
    }
  ],
  "count": 1
}
```
