/**
 * main.js — Entry point for the MLAT Live Map frontend.
 */

import { state } from './state.js';
import { initMap, getMap } from './map.js';
import { buildLayers } from './layers.js';
import { connect, setCallbacks } from './websocket.js';
import {
    initUI, showTooltip, showDetailPanel, hideDetailPanel,
    updateStatus, updateStatsBar, updatePlaybackUI, updateSidebar,
    setOnLayerChange,
} from './ui.js';

let deckOverlay = null;

async function refreshLayers() {
    const layers = await buildLayers(
        onHover,
        onClick,
    );
    if (deckOverlay) {
        deckOverlay.setProps({ layers });
    }
}

function onHover(info) {
    showTooltip(info);
}

function onClick(info) {
    if (info && info.object) {
        const d = info.object;
        state.selectedIcao = d.icao;
        showDetailPanel(d);
        updateSidebar();
    } else {
        state.selectedIcao = null;
        hideDetailPanel();
    }
    refreshLayers();
}

function handleUpdate(msg) {
    state.updateCount++;
    const aircraft = msg.aircraft || [];

    state.aircraftData = aircraft.map(ac => ({
        icao: ac.icao,
        lat: ac.lat,
        lon: ac.lon,
        alt_ft: ac.alt_ft,
        heading_deg: ac.heading_deg,
        speed_kts: ac.speed_kts,
        vrate_fpm: ac.vrate_fpm,
        track_quality: ac.track_quality,
        num_sensors: ac.num_sensors,
        gdop: ac.gdop,
        residual_m: ac.residual_m,
        solve_method: ac.solve_method,
        mlat_mode: ac.mlat_mode,
        position_source: ac.position_source,
        uses_broadcast_position: ac.uses_broadcast_position,
        uses_track_prior: ac.uses_track_prior,
        baro_altitude_used: ac.baro_altitude_used,
        clock_reference_source: ac.clock_reference_source,
        cov_matrix: ac.cov_matrix,
        squawk: ac.squawk,
        age_s: ac.age_s || 0,
    }));

    state.trailData = aircraft
        .filter(ac => ac.trail && ac.trail.length >= 2)
        .map(ac => ({
            icao: ac.icao,
            alt_ft: ac.alt_ft,
            path: ac.trail,
            age_s: ac.age_s || 0,
        }));

    if (msg.replay) {
        state.replay = { ...state.replay, ...msg.replay };
    }

    // Update selected detail panel if an aircraft is selected
    if (state.selectedIcao) {
        const sel = state.aircraftData.find(d => d.icao === state.selectedIcao);
        if (sel) showDetailPanel(sel);
    }

    updateStatsBar();
    updatePlaybackUI();
    updateSidebar();
    refreshLayers();
}

async function loadSensors() {
    try {
        const res = await fetch('/api/sensors');
        const data = await res.json();
        state.sensorData = data.sensors || [];
    } catch (e) {
        console.error('Failed to load sensors:', e);
    }
}

async function loadGdopGrid() {
    try {
        const res = await fetch('/api/gdop_grid');
        const data = await res.json();
        state.gdopGrid = data.grid || [];
    } catch (e) {
        console.error('Failed to load GDOP grid:', e);
    }
}

async function loadStats() {
    try {
        const res = await fetch('/api/stats');
        const data = await res.json();
        state.stats = {
            total_messages: data.total_messages || 0,
            active_tracks: data.active_tracks || 0,
            messages_per_sec: data.messages_per_sec || 0,
            median_residual_m: data.median_residual_m || 0,
            p95_residual_m: data.p95_residual_m || 0,
        };
        if (data.replay) {
            state.replay = { ...state.replay, ...data.replay };
        }
        updateStatsBar();
        updatePlaybackUI();
    } catch (e) {
        // ignore
    }
}

// Periodic stats refresh
setInterval(loadStats, 5000);

// Initialize everything when the map loads
const map = initMap();

map.on('load', async () => {
    // Create Deck.gl overlay
    deckOverlay = new deck.MapboxOverlay({
        interleaved: false,
        layers: [],
    });
    map.addControl(deckOverlay);

    initUI();
    setOnLayerChange(refreshLayers);

    // Load data in parallel
    await Promise.all([loadSensors(), loadGdopGrid(), loadStats()]);

    // Connect WebSocket
    setCallbacks({
        onUpdate: handleUpdate,
        onStatusChange: updateStatus,
    });
    connect();

    // Initial layer render
    refreshLayers();
});
