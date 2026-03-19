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
