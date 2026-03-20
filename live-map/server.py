#!/usr/bin/env python3
"""Layer 6: Live Map — FastAPI WebSocket backend.

Reads Layer 5 JSONL track updates from stdin and broadcasts them
to connected WebSocket clients for real-time map visualization.

Architecture (from MLAT_Verified_Combined_Reference.md Part 14):
    Sensor Data -> FastAPI Backend -> WebSocket -> Browser (MapLibre + Deck.gl)

The server maintains the latest state of all active aircraft tracks
and pushes updates to connected clients at configurable intervals.

Usage:
    Full pipeline (Layer 1 → 2 → 3 → 4 → 5 → 6):
    ./data-pipe/mlat-pipe | python3 modes-decoder/main.py | \\
        python3 correlation-engine/main.py | python3 mlat-solver/main.py | \\
        python3 track-builder/main.py | python3 live-map/server.py

    Or replay from saved Layer 5 output:
    cat track_data.jsonl | python3 live-map/server.py

    Or run standalone for demo/testing:
    python3 live-map/server.py --demo

References:
  - MLAT_Verified_Combined_Reference.md Part 3.3 (Layer 6 spec)
  - MLAT_Verified_Combined_Reference.md Part 5.2 (Stack: FastAPI + MapLibre + Deck.gl)
  - MLAT_Verified_Combined_Reference.md Part 14 (Deck.gl guidance)
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import random
import sys
import threading
import time
from pathlib import Path

import uvicorn
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

# Server configuration
HOST = os.environ.get("MLAT_MAP_HOST", "0.0.0.0")
PORT = int(os.environ.get("MLAT_MAP_PORT", "8080"))
UPDATE_INTERVAL_S = float(os.environ.get("MLAT_UPDATE_INTERVAL", "1.0"))

PRUNE_INTERVAL_S = 60.0
MAX_TRACK_AGE_S = 300.0


@asynccontextmanager
async def lifespan(application: FastAPI):
    """Manage background tasks for the application lifecycle."""
    task = asyncio.create_task(_periodic_prune())
    yield
    task.cancel()


async def _periodic_prune() -> None:
    """Prune stale aircraft from the store every PRUNE_INTERVAL_S seconds."""
    while True:
        await asyncio.sleep(PRUNE_INTERVAL_S)
        pruned = store.prune_stale(MAX_TRACK_AGE_S)
        if pruned > 0:
            log(f"Pruned {pruned} stale aircraft from store")


app = FastAPI(title="MLAT Live Map", version="1.0.0", lifespan=lifespan)

# Mount static files for the frontend
static_dir = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")


# ── Shared state ───────────────────────────────────────────────────────

class AircraftStore:
    """Thread-safe store for current aircraft track states.

    Updated by the stdin reader thread, read by the WebSocket broadcaster.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        # Current state of each aircraft keyed by ICAO
        self._aircraft: dict[str, dict] = {}
        # Track position history keyed by ICAO
        self._trails: dict[str, list[list[float]]] = {}
        self._last_update = 0.0
        self.messages_received = 0

    def update(self, track: dict) -> None:
        """Update or insert an aircraft track."""
        icao = track.get("icao", "")
        if not icao:
            return

        with self._lock:
            self._aircraft[icao] = track
            self._last_update = time.time()
            self.messages_received += 1

            # Maintain trail history (last 50 positions)
            lat = track.get("lat")
            lon = track.get("lon")
            if lat is not None and lon is not None:
                if icao not in self._trails:
                    self._trails[icao] = []
                trail = self._trails[icao]
                trail.append([lon, lat])
                if len(trail) > 50:
                    self._trails[icao] = trail[-50:]

    def get_snapshot(self) -> dict:
        """Get current state of all aircraft for WebSocket broadcast."""
        with self._lock:
            aircraft_list = []
            for icao, data in self._aircraft.items():
                entry = dict(data)
                entry["trail"] = self._trails.get(icao, [])
                aircraft_list.append(entry)

            return {
                "type": "update",
                "timestamp": time.time(),
                "aircraft": aircraft_list,
                "count": len(aircraft_list),
            }

    def prune_stale(self, max_age_s: float = 300.0) -> int:
        """Remove aircraft not updated recently."""
        now = time.time()
        with self._lock:
            stale = [
                icao for icao, data in self._aircraft.items()
                if now - data.get("_update_time", 0) > max_age_s
            ]
            for icao in stale:
                del self._aircraft[icao]
                self._trails.pop(icao, None)
            return len(stale)


store = AircraftStore()


# ── WebSocket management ──────────────────────────────────────────────

class ConnectionManager:
    """Manages active WebSocket connections."""

    def __init__(self) -> None:
        self.active: list[WebSocket] = []

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self.active.append(ws)

    def disconnect(self, ws: WebSocket) -> None:
        if ws in self.active:
            self.active.remove(ws)

    async def broadcast(self, message: str) -> None:
        disconnected: list[WebSocket] = []
        for ws in self.active:
            try:
                await ws.send_text(message)
            except Exception:
                disconnected.append(ws)
        for ws in disconnected:
            self.disconnect(ws)


manager = ConnectionManager()


# ── Routes ────────────────────────────────────────────────────────────

@app.get("/", response_class=HTMLResponse)
async def index() -> HTMLResponse:
    """Serve the main map page."""
    index_path = static_dir / "index.html"
    return HTMLResponse(content=index_path.read_text())


@app.get("/api/aircraft")
async def get_aircraft() -> dict:
    """REST endpoint for current aircraft state (polling fallback)."""
    return store.get_snapshot()


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    """WebSocket endpoint for real-time aircraft updates.

    Sends the current aircraft snapshot to the client at regular
    intervals (default 1 second, configurable via MLAT_UPDATE_INTERVAL).
    """
    await manager.connect(ws)
    try:
        # Send initial snapshot immediately
        snapshot = store.get_snapshot()
        await ws.send_text(json.dumps(snapshot))

        # Keep connection alive and send periodic updates
        while True:
            await asyncio.sleep(UPDATE_INTERVAL_S)
            snapshot = store.get_snapshot()
            await ws.send_text(json.dumps(snapshot))
    except WebSocketDisconnect:
        manager.disconnect(ws)
    except Exception:
        manager.disconnect(ws)


# ── Stdin reader thread ───────────────────────────────────────────────

def stdin_reader() -> None:
    """Read JSONL track updates from stdin in a background thread.

    This runs in a separate thread so the FastAPI event loop
    remains responsive for WebSocket connections.
    """
    log("Stdin reader started — waiting for Layer 5 JSONL input")
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                track = json.loads(line)
                track["_update_time"] = time.time()
                store.update(track)
            except (json.JSONDecodeError, TypeError):
                continue
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    log("Stdin reader finished")


# ── Demo mode ─────────────────────────────────────────────────────────

def demo_data_generator() -> None:
    """Generate synthetic aircraft data for demo/testing.

    Simulates multiple aircraft flying over the Cornwall sensor area
    with realistic headings and speeds.
    """
    log("Demo mode: generating synthetic aircraft over Cornwall")

    # Simulated aircraft with initial positions near Cornwall
    aircraft = [
        {"icao": "4CA7E8", "lat": 50.15, "lon": -5.50, "alt_ft": 36000,
         "heading": 45, "speed_kts": 450},
        {"icao": "4CA100", "lat": 50.30, "lon": -5.20, "alt_ft": 28000,
         "heading": 180, "speed_kts": 380},
        {"icao": "4CA201", "lat": 49.90, "lon": -5.80, "alt_ft": 41000,
         "heading": 90, "speed_kts": 500},
        {"icao": "4CA302", "lat": 50.40, "lon": -5.00, "alt_ft": 32000,
         "heading": 270, "speed_kts": 420},
        {"icao": "4CA403", "lat": 50.05, "lon": -5.35, "alt_ft": 25000,
         "heading": 135, "speed_kts": 350},
    ]

    timestamp_s = 43200  # Start at noon

    while True:
        for ac in aircraft:
            # Update position based on heading and speed
            speed_deg_per_s = ac["speed_kts"] / 3600.0 / 60.0  # rough deg/s
            heading_rad = math.radians(ac["heading"])
            ac["lat"] += speed_deg_per_s * math.cos(heading_rad)
            ac["lon"] += speed_deg_per_s * math.sin(heading_rad)

            # Add small random variations
            ac["heading"] += random.uniform(-2, 2)
            ac["alt_ft"] += random.uniform(-100, 100)

            # Wrap around if too far from Cornwall
            if abs(ac["lat"] - 50.15) > 2.0 or abs(ac["lon"] + 5.5) > 3.0:
                ac["lat"] = 50.15 + random.uniform(-0.5, 0.5)
                ac["lon"] = -5.50 + random.uniform(-0.5, 0.5)

            track = {
                "icao": ac["icao"],
                "lat": round(ac["lat"], 6),
                "lon": round(ac["lon"], 6),
                "alt_ft": round(ac["alt_ft"], 0),
                "heading_deg": round(ac["heading"] % 360, 1),
                "speed_kts": round(ac["speed_kts"] + random.uniform(-10, 10), 1),
                "vrate_fpm": round(random.uniform(-500, 500), 0),
                "track_quality": random.randint(3, 15),
                "positions_count": random.randint(5, 50),
                "residual_m": round(random.uniform(50, 300), 2),
                "gdop": round(random.uniform(2, 8), 2),
                "num_sensors": random.randint(3, 7),
                "solve_method": random.choice([
                    "inamdar_5sensor", "inamdar_4sensor_alt",
                    "constrained_3sensor", "frisch_toa",
                ]),
                "timestamp_s": timestamp_s,
                "timestamp_ns": random.randint(0, 999999999),
                "df_type": 4,
                "squawk": None,
                "raw_msg": "DEMO",
                "t0_s": float(timestamp_s),
                "_update_time": time.time(),
            }
            store.update(track)

        timestamp_s += 1
        time.sleep(1.0)


# ── Logging ───────────────────────────────────────────────────────────

def log(msg: str) -> None:
    """Log to stderr."""
    print(msg, file=sys.stderr, flush=True)


# ── Main ──────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="MLAT Live Map Server (Layer 6)")
    parser.add_argument(
        "--demo", action="store_true",
        help="Run with synthetic demo data (no stdin required)",
    )
    parser.add_argument(
        "--host", default=HOST,
        help=f"Server host (default: {HOST})",
    )
    parser.add_argument(
        "--port", type=int, default=PORT,
        help=f"Server port (default: {PORT})",
    )
    args = parser.parse_args()

    log("=== MLAT Live Map (Layer 6) ===")
    log(f"Backend: FastAPI v0.135.1 + WebSocket")
    log(f"Frontend: MapLibre GL JS v5.20.2 + Deck.gl v9.2.11")
    log(f"Update interval: {UPDATE_INTERVAL_S}s")
    log(f"Server: http://{args.host}:{args.port}")

    if args.demo:
        # Start demo data generator in background thread
        demo_thread = threading.Thread(target=demo_data_generator, daemon=True)
        demo_thread.start()
        log("Demo data generator started")
    else:
        # Start stdin reader in background thread
        reader_thread = threading.Thread(target=stdin_reader, daemon=True)
        reader_thread.start()

    # Start FastAPI server
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
