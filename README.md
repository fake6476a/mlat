# MLAT - Multilateration System

Aircraft localization using Time Difference of Arrival (TDOA) from Neuron's 4DSky sensor network.

Built for the Hedera Hello Future Apex Hackathon.

## Architecture

| Layer | Component | Language | Description |
|-------|-----------|----------|-------------|
| 1 | Data Pipe | Go | Connects to Neuron 4DSky network, receives raw Mode-S packets |
| 2 | Mode-S Decoder | Python | Decodes raw bytes using pyModeS: ICAO, altitude, DF type |
| 3 | Correlation Engine | Python | Groups messages from 3+ sensors by ICAO + time window |
| 4 | MLAT Solver | Python | Frisch TOA formulation with Inamdar algebraic init |
| 5 | Track Builder | Python | EKF for continuous flight track association |
| 6 | Live Map | React | MapLibre GL JS + Deck.gl real-time visualization |

## Layer 1: Data Pipe

The Go data pipe connects to the Neuron 4DSky network as a buyer node using the `neuron-go-hedera-sdk`. It receives live Mode-S packets from distributed sensors via libp2p QUIC streams and outputs structured JSON to stdout.

### Build

```bash
cd data-pipe
go build -o mlat-pipe ./...
```

### Run

```bash
cd data-pipe
./mlat-pipe --buyer-or-seller buyer --port 12345
```

Requires a `.env` file with buyer credentials in the `data-pipe/` directory.

### Output Format

One JSON object per line (JSONL):

```json
{"sensor_id":1234,"lat":28.61,"lon":77.21,"alt":216.0,"timestamp_s":43200,"timestamp_ns":500000000,"raw_msg":"8da4c2b658b9..."}
```

## Layer 2: Mode-S Decoder

Decodes raw Mode-S hex messages from Layer 1 using [pyModeS](https://github.com/junzis/pyModeS). Extracts the ICAO aircraft address, Downlink Format type, barometric altitude, and squawk code.

### Supported Downlink Formats

| DF | Name | Fields Extracted |
|----|------|-----------------|
| 0 | Short Air-Air Surveillance | ICAO (via parity), altitude |
| 4 | Surveillance Altitude Reply | ICAO (via parity), altitude |
| 5 | Surveillance Identity Reply | ICAO (via parity), squawk |
| 11 | All-Call Reply | ICAO (clear text) |
| 16 | Long Air-Air Surveillance | ICAO (via parity), altitude |
| 17 | ADS-B Extended Squitter | ICAO (clear text) |
| 18 | TIS-B / ADS-R | ICAO (clear text) |
| 20 | Comm-B Altitude Reply | ICAO (via parity), altitude |
| 21 | Comm-B Identity Reply | ICAO (via parity), squawk |

For DF4/5/20/21, the 24-bit ICAO address is recovered via `AP XOR CRC(payload)` as specified in ICAO Doc 9871. pyModeS handles this automatically.

### Install

```bash
pip install -r modes-decoder/requirements.txt
```

### Run

Pipe Layer 1 output directly into Layer 2:

```bash
cd data-pipe && ./mlat-pipe --buyer-or-seller buyer --port 12345 | python3 ../modes-decoder/main.py
```

Or replay from a saved file:

```bash
cat saved_data.jsonl | python3 modes-decoder/main.py
```

### Output Format

One JSON object per line (JSONL):

```json
{"icao":"4CA7E8","df_type":4,"altitude_ft":36000,"squawk":null,"raw_msg":"2000171806A983","sensor_id":1001,"lat":50.1,"lon":-5.7,"alt":100.0,"timestamp_s":43200,"timestamp_ns":500000000}
```

| Field | Type | Description |
|-------|------|-------------|
| `icao` | string | 6-char hex ICAO aircraft address |
| `df_type` | int | Downlink Format number |
| `altitude_ft` | int\|null | Barometric altitude in feet (DF0/4/16/20 only) |
| `squawk` | string\|null | 4-digit squawk code (DF5/21 only) |
| `raw_msg` | string | Original hex message (uppercase) |
| `sensor_id` | int | Sensor that received this message |
| `lat` | float | Sensor latitude |
| `lon` | float | Sensor longitude |
| `alt` | float | Sensor altitude |
| `timestamp_s` | int | Reception time (seconds since midnight) |
| `timestamp_ns` | int | Reception time (nanosecond component) |

Logs and statistics are written to stderr every 30 seconds.

## Layer 3: Correlation Engine

Groups decoded Mode-S messages from multiple sensors that received the **same transmission** from the **same aircraft** at nearly the **same time**. Uses Python dicts with time-windowed correlation.

### How It Works

1. Messages are keyed by `(ICAO, raw_msg)` — same aircraft, same radio frame
2. Receptions arriving within a configurable time window (default 2ms) are grouped together
3. Groups are emitted when the window expires
4. Groups below the minimum reception count are dropped

**Time window rationale:** Airsquitter sensors have 30ns GPS sync. Max propagation delay across a 50km baseline ≈ 167µs. The default 2ms window provides comfortable margin.

### Configuration

Environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `MLAT_WINDOW_MS` | `2.0` | Correlation window in milliseconds |
| `MLAT_MIN_RECEPTIONS` | `2` | Minimum sensors per group (2 enables semi-MLAT) |

### Run

Full pipeline (Layer 1 → 2 → 3):

```bash
./data-pipe/mlat-pipe | python3 modes-decoder/main.py | python3 correlation-engine/main.py
```

Or replay from saved Layer 2 output:

```bash
cat decoded_data.jsonl | python3 correlation-engine/main.py
```

### Output Format

One JSON object per line (JSONL):

```json
{"icao":"4CA7E8","df_type":4,"altitude_ft":36000,"squawk":null,"raw_msg":"2000171806A983","num_sensors":3,"receptions":[{"sensor_id":1001,"lat":50.1,"lon":-5.7,"alt":100.0,"timestamp_s":43200,"timestamp_ns":500000000},{"sensor_id":1002,"lat":50.2,"lon":-5.6,"alt":105.0,"timestamp_s":43200,"timestamp_ns":500000050},{"sensor_id":1003,"lat":50.3,"lon":-5.5,"alt":110.0,"timestamp_s":43200,"timestamp_ns":500000100}]}
```

| Field | Type | Description |
|-------|------|-------------|
| `icao` | string | 6-char hex ICAO aircraft address |
| `df_type` | int | Downlink Format number |
| `altitude_ft` | int\|null | Barometric altitude in feet |
| `squawk` | string\|null | 4-digit squawk code |
| `raw_msg` | string | Original hex message |
| `num_sensors` | int | Number of sensors in this group |
| `receptions` | array | List of per-sensor receptions |

Each reception contains: `sensor_id`, `lat`, `lon`, `alt`, `timestamp_s`, `timestamp_ns`.

Logs and statistics are written to stderr every 30 seconds.

## Layer 4: MLAT Solver

Computes aircraft positions from correlation groups using Time Difference of Arrival (TDOA). Implements the Frisch TOA formulation with Inamdar algebraic initialization, Markochev atmospheric refraction correction, and Huber-loss outlier robustness.

### Solver Pipeline (Part 20)

1. **Route by sensor count:**
   - 5+ sensors → Inamdar exact algebraic solution (no iteration needed)
   - 4 sensors + altitude → Inamdar with altitude disambiguation
   - 3 sensors + altitude → Constrained TDOA (determined system)
   - 2 sensors → Skip (prediction-aided requires Layer 5 tracker)
   - 0-1 sensors → Cannot solve

2. **Iterative refinement:** Frisch TOA formulation (`scipy.optimize.least_squares`)
   - `method='trf'` (Trust Region Reflective)
   - `loss='huber'` for outlier robustness (Part 21)
   - Atmospheric refraction-corrected velocity per sensor pair (Markochev model)

3. **Validation:**
   - Position within 500 km of all sensors
   - GDOP ≤ 20 (Part 4.3)
   - Residual ≤ 10,000 m
   - Altitude consistency check

### Key Innovation: Frisch TOA (Part 18)

Instead of forming pairwise TDOA equations (which loses information and introduces correlated noise), the solver keeps all N TOA equations and eliminates the unknown transmission time analytically:

```
t0_hat(x) = mean(t_i - ||x - s_i|| / c_i)
```

This reduces the problem from 4D (x, y, z, t0) to 3D (x, y, z).

### Install

```bash
pip install -r mlat-solver/requirements.txt
```

### Run

Full pipeline (Layer 1 → 2 → 3 → 4):

```bash
./data-pipe/mlat-pipe | python3 modes-decoder/main.py | python3 correlation-engine/main.py | python3 mlat-solver/main.py
```

Or replay from saved Layer 3 output:

```bash
cat correlated_groups.jsonl | python3 mlat-solver/main.py
```

### Output Format

One JSON object per line (JSONL):

```json
{"icao":"4CA7E8","lat":50.15,"lon":-5.5,"alt_ft":36000.0,"residual_m":1.31,"gdop":4.5,"num_sensors":5,"solve_method":"inamdar_5sensor","timestamp_s":43200,"timestamp_ns":47198,"df_type":4,"squawk":null,"raw_msg":"2000171806A983","t0_s":43199.999999988}
```

| Field | Type | Description |
|-------|------|-------------|
| `icao` | string | 6-char hex ICAO aircraft address |
| `lat` | float | Solved latitude (WGS84) |
| `lon` | float | Solved longitude (WGS84) |
| `alt_ft` | float | Altitude in feet (from Mode-S or solved) |
| `residual_m` | float | RMS residual in meters |
| `gdop` | float | Geometric Dilution of Precision |
| `num_sensors` | int | Number of sensors used |
| `solve_method` | string | Algorithm used for initialization |
| `timestamp_s` | int | Reference reception time (seconds) |
| `timestamp_ns` | int | Reference reception time (nanoseconds) |
| `df_type` | int | Downlink Format number |
| `squawk` | string\|null | 4-digit squawk code |
| `raw_msg` | string | Original hex message |
| `t0_s` | float | Estimated transmission time (seconds) |

Logs and statistics are written to stderr every 30 seconds.

### Module Structure

| Module | Description | Reference |
|--------|-------------|-----------|
| `geo.py` | WGS84 ↔ ECEF coordinate conversions | — |
| `atmosphere.py` | Markochev atmospheric refraction model | Part 13.2 |
| `gdop.py` | GDOP computation | Part 4.3, Part 22 |
| `inamdar.py` | Algebraic TDOA initialization | Part 19 |
| `frisch.py` | Frisch TOA iterative refinement | Part 18, Part 22 |
| `solver.py` | Full pipeline routing + validation | Part 20 |
| `main.py` | stdin/stdout JSONL entry point | — |

## Layer 5: Track Builder

Filters and smooths MLAT position fixes into continuous aircraft tracks using a custom Extended Kalman Filter (EKF). Associates incoming fixes by ICAO address and maintains per-aircraft track state with position history and derived kinematics.

### Extended Kalman Filter (Part 22)

- **State vector:** `[x, y, z, vx, vy, vz]` — position + velocity in ECEF meters
- **Motion model:** Constant-velocity with process noise acceleration σ = 5.0 m/s²
- **Measurement model:** Direct position observation with σ = 200 m per axis
- **Innovation gating:** Mahalanobis distance check — rejects fixes with χ²₃ > 14.16 (99.7% confidence, 3-DOF)
- **Max prediction gap:** 60 seconds — track is reset if gap exceeds this

### Track Lifecycle

1. **Create:** New track initialized on first fix for an ICAO address
2. **Update:** Subsequent fixes run EKF predict → gate → update cycle
3. **Establish:** Track considered reliable after ≥ 2 accepted updates
4. **Prune:** Tracks with no updates for > 300 seconds are removed

### Derived Quantities

Computed from the EKF velocity state at each update:

| Quantity | Unit | Description |
|----------|------|-------------|
| `heading_deg` | degrees | Ground track heading (0° = North, clockwise) |
| `speed_kts` | knots | Ground speed |
| `vrate_fpm` | ft/min | Vertical rate |

### Install

```bash
pip install -r track-builder/requirements.txt
```

### Run

Full pipeline (Layer 1 → 2 → 3 → 4 → 5):

```bash
./data-pipe/mlat-pipe | python3 modes-decoder/main.py | python3 correlation-engine/main.py | python3 mlat-solver/main.py | python3 track-builder/main.py
```

Or replay from saved Layer 4 output:

```bash
cat solved_positions.jsonl | python3 track-builder/main.py
```

### Output Format

One JSON object per line (JSONL):

```json
{"icao":"4CA7E8","lat":50.1596,"lon":-5.6404,"alt_ft":36000,"heading_deg":32.7,"speed_kts":2383.7,"vrate_fpm":23.0,"track_quality":2,"positions_count":2,"residual_m":120.5,"gdop":3.2,"num_sensors":4,"solve_method":"inamdar_4sensor_alt","timestamp_s":43201,"timestamp_ns":500000000,"df_type":4,"squawk":null,"raw_msg":"2000171806A984","t0_s":43201.000499833}
```

| Field | Type | Description |
|-------|------|-------------|
| `icao` | string | 6-char hex ICAO aircraft address |
| `lat` | float | EKF-smoothed latitude (WGS84) |
| `lon` | float | EKF-smoothed longitude (WGS84) |
| `alt_ft` | float | Altitude in feet |
| `heading_deg` | float | Ground track heading (degrees, 0=North) |
| `speed_kts` | float | Ground speed (knots) |
| `vrate_fpm` | float | Vertical rate (feet/min) |
| `track_quality` | int | Number of accepted EKF updates for this track |
| `positions_count` | int | Total positions in track history |
| All Layer 4 fields | — | Passed through from the input fix |

Logs and statistics are written to stderr every 30 seconds.

### Module Structure

| Module | Description | Reference |
|--------|-------------|-----------|
| `ekf.py` | Custom 6-state Extended Kalman Filter | Part 22 |
| `tracker.py` | Per-ICAO track association and management | Part 3.2 |
| `main.py` | stdin/stdout JSONL entry point | — |

## Layer 6: Live Map

Real-time web visualization of aircraft tracks using MapLibre GL JS + Deck.gl. A FastAPI backend reads Layer 5 output and broadcasts track state to connected browsers via WebSocket.

### Technology Stack (Part 5.2, Part 14)

| Component | Library | Version |
|-----------|---------|---------|
| Backend | FastAPI + uvicorn | 0.135.1 |
| Map renderer | MapLibre GL JS | 5.20.2 |
| Data layers | Deck.gl | 9.2.11 |
| Real-time transport | WebSocket | — |

### Architecture

```
Layer 5 (JSONL stdout) → stdin → FastAPI Backend → WebSocket → Browser
                                      ↓
                              REST /api/aircraft (polling fallback)
```

### Visualization Layers

| Deck.gl Layer | Purpose |
|---------------|---------|
| `ScatterplotLayer` | Aircraft positions colored by altitude |
| `TextLayer` | ICAO address labels |
| `PathLayer` | Flight trail history (last 50 positions) |
| `ScatterplotLayer` | Sensor positions (Cornwall network) |

### Altitude Color Coding

| Color | Altitude Range |
|-------|---------------|
| Green | < 10,000 ft |
| Cyan | 10,000 – 25,000 ft |
| Purple | 25,000 – 35,000 ft |
| Orange | > 35,000 ft |

### Configuration

Environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `MLAT_MAP_HOST` | `0.0.0.0` | Server bind address |
| `MLAT_MAP_PORT` | `8080` | Server port |
| `MLAT_UPDATE_INTERVAL` | `1.0` | WebSocket push interval (seconds) |

### Install

```bash
pip install -r live-map/requirements.txt
```

### Run

Full pipeline (Layer 1 → 2 → 3 → 4 → 5 → 6):

```bash
./data-pipe/mlat-pipe | python3 modes-decoder/main.py | python3 correlation-engine/main.py | python3 mlat-solver/main.py | python3 track-builder/main.py | python3 live-map/server.py
```

Or run with synthetic demo data (no pipeline required):

```bash
python3 live-map/server.py --demo
```

Then open `http://localhost:8080` in a browser.

### Endpoints

| Endpoint | Protocol | Description |
|----------|----------|-------------|
| `GET /` | HTTP | Main map page |
| `GET /api/aircraft` | HTTP | Current aircraft state (JSON) |
| `WS /ws` | WebSocket | Real-time aircraft updates |

### WebSocket Message Format

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
      "trail": [[-5.70, 50.10], [-5.68, 50.12], [-5.65, 50.15]]
    }
  ],
  "count": 1
}
```
