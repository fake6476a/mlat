/**
 * state.js — Shared application state for the MLAT Live Map.
 */

export const state = {
    aircraftData: [],
    trailData: [],
    sensorData: [],
    gdopGrid: [],
    updateCount: 0,
    selectedIcao: null,

    // Layer visibility toggles
    layers: {
        trails: true,
        ellipses: true,
        sensors: true,
        gdop: false,
        labels: true,
    },

    // Altitude filter (feet)
    altitudeFilter: {
        min: 0,
        max: 60000,
    },

    // Replay state (from server)
    replay: {
        mode: 'live',
        total_records: 0,
        position: 0,
        paused: false,
        speed: 1.0,
        progress: 0,
    },

    // Pipeline stats
    stats: {
        total_messages: 0,
        active_tracks: 0,
        messages_per_sec: 0,
        median_residual_m: 0,
        p95_residual_m: 0,
    },
};

// Emergency squawk codes
export const EMERGENCY_SQUAWKS = {
    '7500': { label: 'HIJACK', color: [255, 0, 0, 255] },
    '7600': { label: 'RADIO FAIL', color: [255, 165, 0, 255] },
    '7700': { label: 'EMERGENCY', color: [255, 0, 0, 255] },
};

export function altitudeColor(altFt, opacity = 220) {
    if (altFt == null) return [128, 128, 128, opacity];
    if (altFt < 10000)  return [0, 255, 136, opacity];
    if (altFt < 25000)  return [0, 204, 255, opacity];
    if (altFt < 35000)  return [170, 102, 255, opacity];
    return [255, 102, 68, opacity];
}

export function altitudeColorHex(altFt) {
    if (altFt == null) return '#808080';
    if (altFt < 10000)  return '#00ff88';
    if (altFt < 25000)  return '#00ccff';
    if (altFt < 35000)  return '#aa66ff';
    return '#ff6644';
}

export function isEmergency(squawk) {
    return squawk && EMERGENCY_SQUAWKS[squawk];
}
