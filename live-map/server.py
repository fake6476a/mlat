#!/usr/bin/env python3
"""Serve the Layer 6 live map over FastAPI and WebSockets from Layer 5 JSONL."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
import threading
import time
from collections import deque
from contextlib import asynccontextmanager
from pathlib import Path

import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, Response
from fastapi.staticfiles import StaticFiles
from gdop import compute_gdop
from geo import lla_to_ecef

_gdop_grid_cache = None

HOST = os.environ.get("MLAT_MAP_HOST", "0.0.0.0")
PORT = int(os.environ.get("MLAT_MAP_PORT", "8080"))
UPDATE_INTERVAL_S = float(os.environ.get("MLAT_UPDATE_INTERVAL", "1.0"))

PRUNE_INTERVAL_S = 60.0
MAX_TRACK_AGE_S = 300.0

LOCATION_OVERRIDES_PATH = (
    Path(__file__).parent.parent / "data-pipe" / "location-overrides.txt"
)

_sensors_cache: list[dict] | None = None


def load_sensors() -> list[dict]:
    """Load sensor positions from the location-overrides file."""
    global _sensors_cache
    if _sensors_cache is not None:
        return _sensors_cache

    sensors: list[dict] = []
    if LOCATION_OVERRIDES_PATH.exists():
        try:
            raw = json.loads(LOCATION_OVERRIDES_PATH.read_text())
            for entry in raw:
                sensors.append({
                    "name": entry.get("name", "Unknown"),
                    "lat": entry["lat"],
                    "lon": entry["lon"],
                    "alt": entry.get("alt", 0),
                })
        except (json.JSONDecodeError, KeyError):
            pass

    if not sensors:
        sensors = [
            {"lon": -5.7, "lat": 50.1, "alt": 0, "name": "Sensor 1"},
            {"lon": -5.6, "lat": 50.2, "alt": 0, "name": "Sensor 2"},
            {"lon": -5.5, "lat": 50.3, "alt": 0, "name": "Sensor 3"},
            {"lon": -5.4, "lat": 50.15, "alt": 0, "name": "Sensor 4"},
            {"lon": -5.3, "lat": 50.25, "alt": 0, "name": "Sensor 5"},
            {"lon": -5.1, "lat": 50.35, "alt": 0, "name": "Sensor 6"},
            {"lon": -5.0, "lat": 50.1, "alt": 0, "name": "Sensor 7"},
            {"lon": -6.3, "lat": 49.92, "alt": 0, "name": "Sensor 8"},
            {"lon": -6.35, "lat": 49.95, "alt": 0, "name": "Sensor 9"},
        ]
    _sensors_cache = sensors
    return sensors


class AircraftStore:
    """Thread-safe store for current aircraft track state."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._aircraft: dict[str, dict] = {}
        self._trails: dict[str, list[list[float]]] = {}
        self._last_update = 0.0
        self.messages_received = 0
        self._stats_window: deque[float] = deque()
        self._residuals: deque[float] = deque(maxlen=500)

    def update(self, track: dict) -> None:
        icao = track.get("icao", "")
        if not icao:
            return

        now = time.time()
        with self._lock:
            self._aircraft[icao] = track
            self._last_update = now
            self.messages_received += 1
            self._stats_window.append(now)

            residual = track.get("residual_m")
            if residual is not None:
                self._residuals.append(residual)

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
        now = time.time()
        with self._lock:
            aircraft_list = []
            for icao, data in self._aircraft.items():
                entry = dict(data)
                entry["trail"] = self._trails.get(icao, [])
                update_time = data.get("_update_time", now)
                entry["age_s"] = round(now - update_time, 1)
                aircraft_list.append(entry)

            return {
                "type": "update",
                "timestamp": now,
                "aircraft": aircraft_list,
                "count": len(aircraft_list),
            }

    def get_stats(self) -> dict:
        now = time.time()
        with self._lock:
            while self._stats_window and now - self._stats_window[0] > 60:
                self._stats_window.popleft()
            msg_per_min = len(self._stats_window)
            msg_per_sec = round(msg_per_min / 60.0, 1) if msg_per_min else 0

            residuals = list(self._residuals)
            median_residual = 0.0
            p95_residual = 0.0
            if residuals:
                sorted_r = sorted(residuals)
                median_residual = round(sorted_r[len(sorted_r) // 2], 1)
                p95_idx = min(int(len(sorted_r) * 0.95), len(sorted_r) - 1)
                p95_residual = round(sorted_r[p95_idx], 1)

            return {
                "total_messages": self.messages_received,
                "active_tracks": len(self._aircraft),
                "messages_per_sec": msg_per_sec,
                "median_residual_m": median_residual,
                "p95_residual_m": p95_residual,
                "last_update": self._last_update,
            }

    def prune_stale(self, max_age_s: float = 300.0) -> int:
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


class ReplayBuffer:
    """Buffer for recorded data replay with playback controls."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.records: list[dict] = []
        self.position = 0
        self.paused = False
        self.speed = 1.0
        self.loaded = False

    def load_all(self, lines: list[str]) -> None:
        with self._lock:
            for line in lines:
                line = line.strip()
                if not line:
                    continue
                try:
                    self.records.append(json.loads(line))
                except (json.JSONDecodeError, TypeError):
                    continue
            self.loaded = True

    @property
    def total(self) -> int:
        return len(self.records)

    def get_status(self) -> dict:
        with self._lock:
            return {
                "mode": "replay" if self.loaded else "live",
                "total_records": len(self.records),
                "position": self.position,
                "paused": self.paused,
                "speed": self.speed,
                "progress": (
                    round(self.position / len(self.records) * 100, 1)
                    if self.records else 0
                ),
            }

    def set_paused(self, paused: bool) -> None:
        with self._lock:
            self.paused = paused

    def set_speed(self, speed: float) -> None:
        with self._lock:
            self.speed = max(0.1, min(100.0, speed))

    def seek(self, position: int) -> None:
        with self._lock:
            self.position = max(0, min(position, len(self.records) - 1))
            store._aircraft.clear()
            store._trails.clear()


replay_buffer = ReplayBuffer()


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


@asynccontextmanager
async def lifespan(application: FastAPI):
    prune_task = asyncio.create_task(_periodic_prune())
    replay_task = None
    if replay_buffer.loaded:
        replay_task = asyncio.create_task(_replay_feeder())
    yield
    prune_task.cancel()
    if replay_task:
        replay_task.cancel()


async def _periodic_prune() -> None:
    while True:
        await asyncio.sleep(PRUNE_INTERVAL_S)
        pruned = store.prune_stale(MAX_TRACK_AGE_S)
        if pruned > 0:
            log(f"Pruned {pruned} stale aircraft from store")


async def _replay_feeder() -> None:
    """Feed buffered replay data into the store at controlled speed."""
    log(f"Replay feeder started with {replay_buffer.total} records")
    while replay_buffer.position < replay_buffer.total:
        if replay_buffer.paused:
            await asyncio.sleep(0.1)
            continue

        with replay_buffer._lock:
            if replay_buffer.position >= len(replay_buffer.records):
                break
            record = replay_buffer.records[replay_buffer.position]
            replay_buffer.position += 1

        record["_update_time"] = time.time()
        store.update(record)

        delay = UPDATE_INTERVAL_S / replay_buffer.speed / 10
        await asyncio.sleep(max(0.001, delay))

    log("Replay feeder finished")


app = FastAPI(title="MLAT Live Map", version="2.0.0", lifespan=lifespan)

static_dir = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")


@app.get("/", response_class=HTMLResponse)
async def index() -> HTMLResponse:
    index_path = static_dir / "index.html"
    return HTMLResponse(content=index_path.read_text())


@app.get("/favicon.ico")
async def favicon() -> Response:
    return Response(status_code=204)


@app.get("/api/aircraft")
async def get_aircraft() -> dict:
    return store.get_snapshot()


@app.get("/api/sensors")
async def get_sensors() -> dict:
    sensors = load_sensors()
    return {"sensors": sensors, "count": len(sensors)}


@app.get("/api/stats")
async def get_stats() -> dict:
    stats = store.get_stats()
    stats["replay"] = replay_buffer.get_status()
    return stats


@app.get("/api/gdop_grid")
async def get_gdop_grid() -> dict:
    global _gdop_grid_cache
    if _gdop_grid_cache is not None:
        return _gdop_grid_cache

    sensors = load_sensors()
    sensor_ecef = np.array([
        lla_to_ecef(s["lat"], s["lon"], s.get("alt", 100.0))
        for s in sensors
    ])

    grid = []
    lat_min, lat_max = 49.8, 50.8
    lon_min, lon_max = -6.5, -4.5
    steps = 20

    alt_m = 30000 * 0.3048

    for i in range(steps):
        lat = lat_min + (lat_max - lat_min) * i / (steps - 1)
        for j in range(steps):
            lon = lon_min + (lon_max - lon_min) * j / (steps - 1)
            pos_ecef = lla_to_ecef(lat, lon, alt_m)
            gdop = compute_gdop(pos_ecef, sensor_ecef)
            grid.append({
                "lat": round(lat, 4),
                "lon": round(lon, 4),
                "gdop": round(float(gdop), 2)
            })

    _gdop_grid_cache = {"grid": grid}
    return _gdop_grid_cache


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    await manager.connect(ws)
    try:
        snapshot = store.get_snapshot()
        snapshot["replay"] = replay_buffer.get_status()
        await ws.send_text(json.dumps(snapshot))

        while True:
            await asyncio.sleep(UPDATE_INTERVAL_S)
            snapshot = store.get_snapshot()
            snapshot["replay"] = replay_buffer.get_status()
            await ws.send_text(json.dumps(snapshot))
    except WebSocketDisconnect:
        manager.disconnect(ws)
    except Exception:
        manager.disconnect(ws)


@app.websocket("/ws/control")
async def control_endpoint(ws: WebSocket) -> None:
    """WebSocket endpoint for playback control commands."""
    await ws.accept()
    try:
        while True:
            data = await ws.receive_text()
            try:
                cmd = json.loads(data)
                action = cmd.get("action", "")
                if action == "pause":
                    replay_buffer.set_paused(True)
                elif action == "resume":
                    replay_buffer.set_paused(False)
                elif action == "toggle_pause":
                    replay_buffer.set_paused(not replay_buffer.paused)
                elif action == "set_speed":
                    replay_buffer.set_speed(cmd.get("speed", 1.0))
                elif action == "seek":
                    replay_buffer.seek(cmd.get("position", 0))
                elif action == "restart":
                    replay_buffer.seek(0)
                    replay_buffer.set_paused(False)

                await ws.send_text(json.dumps(replay_buffer.get_status()))
            except (json.JSONDecodeError, TypeError):
                pass
    except WebSocketDisconnect:
        pass
    except Exception:
        pass


def stdin_reader() -> None:
    """Read Layer 5 JSONL updates from stdin in a background thread."""
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


def stdin_replay_loader() -> None:
    """Read all stdin into replay buffer for controlled playback."""
    log("Loading replay data from stdin...")
    lines: list[str] = []
    try:
        for line in sys.stdin:
            lines.append(line)
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    replay_buffer.load_all(lines)
    log(f"Loaded {replay_buffer.total} records for replay")


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="MLAT Live Map Server (Layer 6)")
    parser.add_argument(
        "--host", default=HOST,
        help=f"Server host (default: {HOST})",
    )
    parser.add_argument(
        "--port", type=int, default=PORT,
        help=f"Server port (default: {PORT})",
    )
    parser.add_argument(
        "--replay", action="store_true",
        help="Enable replay mode: buffer all stdin data and allow playback controls",
    )
    args = parser.parse_args()

    log("=== MLAT Live Map (Layer 6) v2.0 ===")
    log(f"Backend: FastAPI + WebSocket")
    log(f"Update interval: {UPDATE_INTERVAL_S}s")
    log(f"Mode: {'replay' if args.replay else 'live'}")
    log(f"Server: http://{args.host}:{args.port}")

    sensors = load_sensors()
    log(f"Loaded {len(sensors)} sensor positions")

    if args.replay:
        stdin_replay_loader()
    else:
        reader_thread = threading.Thread(target=stdin_reader, daemon=True)
        reader_thread.start()

    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
