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
