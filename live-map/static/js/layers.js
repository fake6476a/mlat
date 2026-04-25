/**
 * layers.js — Deck.gl layer builders for aircraft, trails, sensors, GDOP.
 */

import { state, altitudeColor, isEmergency, EMERGENCY_SQUAWKS } from './state.js';

// Aircraft SVG icon as data URL (pointing north/up)
const AIRCRAFT_SVG = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64" width="64" height="64">
  <path d="M32 4 L28 24 L8 32 L12 36 L28 34 L28 48 L20 54 L24 58 L32 54 L40 58 L44 54 L36 48 L36 34 L52 36 L56 32 L36 24 Z" fill="white"/>
</svg>`;
const AIRCRAFT_ICON_URL = 'data:image/svg+xml;base64,' + btoa(AIRCRAFT_SVG);

// Sensor SVG icon
const SENSOR_SVG = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 32 32" width="32" height="32">
  <circle cx="16" cy="16" r="10" fill="#ffc800" stroke="white" stroke-width="2"/>
  <circle cx="16" cy="16" r="4" fill="#ff8800"/>
</svg>`;
const SENSOR_ICON_URL = 'data:image/svg+xml;base64,' + btoa(SENSOR_SVG);

let iconImageLoaded = false;
let aircraftIconImage = null;
let sensorIconImage = null;

function loadIcons() {
    if (iconImageLoaded) return Promise.resolve();
    return new Promise((resolve) => {
        let loaded = 0;
        const check = () => { if (++loaded === 2) { iconImageLoaded = true; resolve(); } };

        aircraftIconImage = new Image();
        aircraftIconImage.onload = check;
        aircraftIconImage.src = AIRCRAFT_ICON_URL;

        sensorIconImage = new Image();
        sensorIconImage.onload = check;
        sensorIconImage.src = SENSOR_ICON_URL;
    });
}

function computeEllipseVertices(center, covMatrix, segments = 32) {
    if (!covMatrix) return [];
    const [lon, lat] = center;
    const Pxx = covMatrix[0][0];
    const Pxy = covMatrix[0][1];
    const Pyy = covMatrix[1][1];

    const trace = Pxx + Pyy;
    const det = Pxx * Pyy - Pxy * Pxy;
    const lambda1 = (trace + Math.sqrt(Math.max(0, trace * trace - 4 * det))) / 2;
    const lambda2 = (trace - Math.sqrt(Math.max(0, trace * trace - 4 * det))) / 2;

    const theta = 0.5 * Math.atan2(2 * Pxy, Pxx - Pyy);
    const scale1 = 3.03 * Math.sqrt(Math.max(0, lambda1));
    const scale2 = 3.03 * Math.sqrt(Math.max(0, lambda2));

    const metersPerDegLat = 111320;
    const metersPerDegLon = 40075000 * Math.cos(lat * Math.PI / 180) / 360;

    const vertices = [];
    for (let i = 0; i < segments; i++) {
        const angle = (i * 2 * Math.PI) / segments;
        const x = scale1 * Math.cos(angle);
        const y = scale2 * Math.sin(angle);

        const dx = x * Math.cos(theta) - y * Math.sin(theta);
        const dy = x * Math.sin(theta) + y * Math.cos(theta);

        vertices.push([
            lon + dx / metersPerDegLon,
            lat + dy / metersPerDegLat
        ]);
    }
    return vertices;
}

function staleOpacity(age_s) {
    if (age_s == null || age_s < 30) return 1.0;
    if (age_s > 240) return 0.15;
    return 1.0 - (age_s - 30) / (240 - 30) * 0.85;
}

function filterByAltitude(data) {
    const { min, max } = state.altitudeFilter;
    if (min === 0 && max >= 60000) return data;
    return data.filter(d => {
        const alt = d.alt_ft;
        if (alt == null) return true;
        return alt >= min && alt <= max;
    });
}

export async function buildLayers(onHover, onClick) {
    await loadIcons();
    const layers = [];
    const filtered = filterByAltitude(state.aircraftData);

    // Trail layer
    if (state.layers.trails && state.trailData.length > 0) {
        const filteredTrails = filterByAltitude(state.trailData);
        layers.push(new deck.PathLayer({
            id: 'trails',
            data: filteredTrails,
            getPath: d => d.path,
            getColor: d => {
                const c = altitudeColor(d.alt_ft);
                const opacity = Math.round(80 * staleOpacity(d.age_s));
                return [c[0], c[1], c[2], opacity];
            },
            getWidth: 2,
            widthMinPixels: 1,
            widthMaxPixels: 3,
            capRounded: true,
            jointRounded: true,
            pickable: false,
        }));
    }

    // Uncertainty ellipses
    if (state.layers.ellipses && filtered.length > 0) {
        layers.push(new deck.PolygonLayer({
            id: 'uncertainty-ellipses',
            data: filtered.filter(d => d.cov_matrix && d.cov_matrix[0][0] < 1e5),
            getPolygon: d => computeEllipseVertices([d.lon, d.lat], d.cov_matrix),
            getFillColor: d => {
                const em = isEmergency(d.squawk);
                if (em) return [255, 0, 0, 30];
                return [255, 60, 60, 40];
            },
            getLineColor: d => {
                const em = isEmergency(d.squawk);
                if (em) return [255, 0, 0, 200];
                return [255, 60, 60, 150];
            },
            lineWidthMinPixels: 1,
            stroked: true,
            filled: true,
            pickable: false,
        }));
    }

    // GDOP heatmap overlay
    if (state.layers.gdop && state.gdopGrid.length > 0) {
        layers.push(new deck.ScatterplotLayer({
            id: 'gdop-grid',
            data: state.gdopGrid.filter(d => d.gdop < 100),
            getPosition: d => [d.lon, d.lat],
            getRadius: 3000,
            getFillColor: d => {
                const g = d.gdop;
                if (g < 5) return [0, 200, 0, 60];
                if (g < 10) return [255, 255, 0, 60];
                if (g < 20) return [255, 140, 0, 60];
                return [255, 0, 0, 60];
            },
            radiusMinPixels: 8,
            radiusMaxPixels: 40,
            pickable: false,
        }));
    }

    // Aircraft icons
    if (filtered.length > 0) {
        layers.push(new deck.IconLayer({
            id: 'aircraft',
            data: filtered,
            getPosition: d => [d.lon, d.lat],
            getIcon: () => 'aircraft',
            iconAtlas: aircraftIconImage,
            iconMapping: {
                aircraft: { x: 0, y: 0, width: 64, height: 64, mask: true },
            },
            getSize: d => {
                if (state.selectedIcao && d.icao === state.selectedIcao) return 36;
                return isEmergency(d.squawk) ? 30 : 24;
            },
            getAngle: d => -(d.heading_deg || 0),
            getColor: d => {
                const em = isEmergency(d.squawk);
                if (em) return EMERGENCY_SQUAWKS[d.squawk].color;
                const c = altitudeColor(d.alt_ft);
                const alpha = Math.round(c[3] * staleOpacity(d.age_s));
                return [c[0], c[1], c[2], alpha];
            },
            sizeMinPixels: 16,
            sizeMaxPixels: 48,
            pickable: true,
            autoHighlight: true,
            highlightColor: [255, 255, 255, 80],
            onHover: onHover,
            onClick: onClick,
            updateTriggers: {
                getSize: [state.selectedIcao],
                getColor: [state.aircraftData.map(d => d.age_s)],
            },
        }));

        // ICAO labels
        if (state.layers.labels) {
            layers.push(new deck.TextLayer({
                id: 'labels',
                data: filtered,
                getPosition: d => [d.lon, d.lat],
                getText: d => {
                    const em = isEmergency(d.squawk);
                    if (em) return `${d.icao} ${EMERGENCY_SQUAWKS[d.squawk].label}`;
                    if (d.squawk) return `${d.icao} [${d.squawk}]`;
                    return d.icao;
                },
                getSize: 11,
                getColor: d => {
                    const em = isEmergency(d.squawk);
                    if (em) return [255, 80, 80, 255];
                    return [255, 255, 255, Math.round(200 * staleOpacity(d.age_s))];
                },
                getAngle: 0,
                getTextAnchor: 'start',
                getAlignmentBaseline: 'bottom',
                getPixelOffset: [14, -14],
                fontFamily: 'monospace',
                fontWeight: 700,
                pickable: false,
            }));
        }
    }

    // Sensor markers
    if (state.layers.sensors && state.sensorData.length > 0) {
        layers.push(new deck.IconLayer({
            id: 'sensors',
            data: state.sensorData,
            getPosition: d => [d.lon, d.lat],
            getIcon: () => 'sensor',
            iconAtlas: sensorIconImage,
            iconMapping: {
                sensor: { x: 0, y: 0, width: 32, height: 32, mask: false },
            },
            getSize: 24,
            sizeMinPixels: 12,
            sizeMaxPixels: 32,
            pickable: true,
            onHover: (info) => {
                if (info && info.object) {
                    onHover({
                        ...info,
                        _isSensor: true,
                        object: info.object,
                    });
                } else {
                    onHover(info);
                }
            },
        }));

        // Sensor labels
        layers.push(new deck.TextLayer({
            id: 'sensor-labels',
            data: state.sensorData,
            getPosition: d => [d.lon, d.lat],
            getText: d => d.name || '',
            getSize: 10,
            getColor: [255, 200, 0, 180],
            getTextAnchor: 'start',
            getAlignmentBaseline: 'bottom',
            getPixelOffset: [14, -8],
            fontFamily: 'monospace',
            fontWeight: 600,
            pickable: false,
        }));
    }

    return layers;
}
