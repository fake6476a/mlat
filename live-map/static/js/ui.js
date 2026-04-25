/**
 * ui.js — UI controls, sidebar, tooltips, playback controls.
 */

import { state, altitudeColorHex, isEmergency, EMERGENCY_SQUAWKS } from './state.js';
import { sendControl } from './websocket.js';
import { flyTo } from './map.js';

let sidebarOpen = false;
let sortField = 'icao';
let sortAsc = true;
let onLayerChangeCallback = null;

export function setOnLayerChange(cb) {
    onLayerChangeCallback = cb;
}

function notifyLayerChange() {
    if (onLayerChangeCallback) onLayerChangeCallback();
}

export function initUI() {
    initLayerToggles();
    initAltitudeSlider();
    initPlaybackControls();
    initSidebarToggle();

    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape') {
            state.selectedIcao = null;
            hideDetailPanel();
        }
    });
}

// --- Layer Toggles ---

function initLayerToggles() {
    const toggles = document.querySelectorAll('.layer-toggle');
    toggles.forEach(toggle => {
        const layer = toggle.dataset.layer;
        if (layer && state.layers[layer] !== undefined) {
            toggle.checked = state.layers[layer];
            toggle.addEventListener('change', () => {
                state.layers[layer] = toggle.checked;
                notifyLayerChange();
            });
        }
    });
}

// --- Altitude Slider ---

function initAltitudeSlider() {
    const minSlider = document.getElementById('alt-min');
    const maxSlider = document.getElementById('alt-max');
    const minLabel = document.getElementById('alt-min-val');
    const maxLabel = document.getElementById('alt-max-val');

    if (!minSlider || !maxSlider) return;

    function update() {
        let min = parseInt(minSlider.value);
        let max = parseInt(maxSlider.value);
        if (min > max) { min = max; minSlider.value = min; }
        state.altitudeFilter.min = min;
        state.altitudeFilter.max = max;
        minLabel.textContent = formatAlt(min);
        maxLabel.textContent = formatAlt(max);
    }

    minSlider.addEventListener('input', () => { update(); notifyLayerChange(); });
    maxSlider.addEventListener('input', () => { update(); notifyLayerChange(); });
    update();
}

function formatAlt(ft) {
    if (ft >= 1000) return `${(ft / 1000).toFixed(0)}k`;
    return `${ft}`;
}

// --- Playback Controls ---

function initPlaybackControls() {
    const playPause = document.getElementById('play-pause-btn');
    const restartBtn = document.getElementById('restart-btn');
    const speedSelect = document.getElementById('speed-select');
    const seekSlider = document.getElementById('seek-slider');

    if (playPause) {
        playPause.addEventListener('click', () => {
            sendControl({ action: 'toggle_pause' });
        });
    }

    if (restartBtn) {
        restartBtn.addEventListener('click', () => {
            sendControl({ action: 'restart' });
        });
    }

    if (speedSelect) {
        speedSelect.addEventListener('change', () => {
            sendControl({ action: 'set_speed', speed: parseFloat(speedSelect.value) });
        });
    }

    if (seekSlider) {
        seekSlider.addEventListener('input', () => {
            sendControl({ action: 'seek', position: parseInt(seekSlider.value) });
        });
    }
}

export function updatePlaybackUI() {
    const panel = document.getElementById('playback-panel');
    if (!panel) return;

    const r = state.replay;
    if (r.mode !== 'replay') {
        panel.style.display = 'none';
        return;
    }
    panel.style.display = 'block';

    const playPause = document.getElementById('play-pause-btn');
    if (playPause) {
        playPause.textContent = r.paused ? '\u25B6' : '\u23F8';
        playPause.title = r.paused ? 'Resume' : 'Pause';
    }

    const seekSlider = document.getElementById('seek-slider');
    if (seekSlider) {
        seekSlider.max = r.total_records;
        seekSlider.value = r.position;
    }

    const progressLabel = document.getElementById('replay-progress');
    if (progressLabel) {
        progressLabel.textContent = `${r.progress}% (${r.position}/${r.total_records})`;
    }
}

// --- Sidebar (Aircraft List) ---

function initSidebarToggle() {
    const toggleBtn = document.getElementById('sidebar-toggle');
    if (toggleBtn) {
        toggleBtn.addEventListener('click', () => {
            sidebarOpen = !sidebarOpen;
            const sidebar = document.getElementById('aircraft-sidebar');
            if (sidebar) {
                sidebar.classList.toggle('open', sidebarOpen);
            }
            toggleBtn.textContent = sidebarOpen ? '\u00BB' : '\u00AB';
            toggleBtn.title = sidebarOpen ? 'Close aircraft list' : 'Open aircraft list';
        });
    }

    document.querySelectorAll('.sort-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const field = btn.dataset.sort;
            if (sortField === field) {
                sortAsc = !sortAsc;
            } else {
                sortField = field;
                sortAsc = true;
            }
            updateSidebar();
        });
    });
}

export function updateSidebar() {
    const tbody = document.getElementById('aircraft-tbody');
    if (!tbody) return;

    let data = [...state.aircraftData];

    // Apply search filter
    const search = document.getElementById('aircraft-search');
    if (search && search.value) {
        const q = search.value.toUpperCase();
        data = data.filter(d => d.icao.includes(q) || (d.squawk && d.squawk.includes(q)));
    }

    // Sort
    data.sort((a, b) => {
        let va = a[sortField] ?? '';
        let vb = b[sortField] ?? '';
        if (typeof va === 'number' && typeof vb === 'number') {
            return sortAsc ? va - vb : vb - va;
        }
        va = String(va);
        vb = String(vb);
        return sortAsc ? va.localeCompare(vb) : vb.localeCompare(va);
    });

    tbody.innerHTML = data.map(d => {
        const em = isEmergency(d.squawk);
        const rowClass = em ? 'emergency-row' : (d.icao === state.selectedIcao ? 'selected-row' : '');
        const altColor = altitudeColorHex(d.alt_ft);
        return `<tr class="${rowClass}" data-icao="${d.icao}" onclick="window.__selectAircraft('${d.icao}', ${d.lon}, ${d.lat})">
            <td style="color:${altColor}">${d.icao}</td>
            <td>${d.squawk || '—'}</td>
            <td>${d.alt_ft != null ? Math.round(d.alt_ft).toLocaleString() : '—'}</td>
            <td>${d.speed_kts != null ? Math.round(d.speed_kts) : '—'}</td>
            <td>${d.heading_deg != null ? Math.round(d.heading_deg) + '\u00B0' : '—'}</td>
            <td>${d.track_quality || '—'}</td>
        </tr>`;
    }).join('');
}

// Global click handler for sidebar rows
window.__selectAircraft = (icao, lon, lat) => {
    state.selectedIcao = icao;
    flyTo(lon, lat);
    showDetailPanel(state.aircraftData.find(d => d.icao === icao));
    updateSidebar();
};

// --- Tooltip ---

export function showTooltip(info) {
    const tooltip = document.getElementById('tooltip');
    if (!tooltip) return;

    if (!info || !info.object) {
        tooltip.style.display = 'none';
        return;
    }

    if (info._isSensor) {
        const d = info.object;
        tooltip.innerHTML = `
            <div class="tt-icao">\uD83D\uDCE1 ${d.name || 'Sensor'}</div>
            <div class="tt-row"><span class="tt-label">Lat</span><span class="tt-value">${d.lat.toFixed(5)}</span></div>
            <div class="tt-row"><span class="tt-label">Lon</span><span class="tt-value">${d.lon.toFixed(5)}</span></div>
            <div class="tt-row"><span class="tt-label">Alt</span><span class="tt-value">${d.alt ? d.alt.toFixed(0) + ' m' : '—'}</span></div>
        `;
    } else {
        const d = info.object;
        const em = isEmergency(d.squawk);
        const emBadge = em ? `<span class="emergency-badge">${EMERGENCY_SQUAWKS[d.squawk].label}</span>` : '';
        tooltip.innerHTML = `
            <div class="tt-icao">${d.icao} ${emBadge}</div>
            ${d.squawk ? `<div class="tt-row"><span class="tt-label">Squawk</span><span class="tt-value ${em ? 'emergency-text' : ''}">${d.squawk}</span></div>` : ''}
            <div class="tt-row"><span class="tt-label">Altitude</span><span class="tt-value">${d.alt_ft != null ? Math.round(d.alt_ft).toLocaleString() + ' ft' : '—'}</span></div>
            <div class="tt-row"><span class="tt-label">Speed</span><span class="tt-value">${d.speed_kts != null ? Math.round(d.speed_kts) + ' kts' : '—'}</span></div>
            <div class="tt-row"><span class="tt-label">Heading</span><span class="tt-value">${d.heading_deg != null ? Math.round(d.heading_deg) + '\u00B0' : '—'}</span></div>
            <div class="tt-row"><span class="tt-label">V/Rate</span><span class="tt-value">${d.vrate_fpm != null ? Math.round(d.vrate_fpm) + ' fpm' : '—'}</span></div>
            <div class="tt-row"><span class="tt-label">Sensors</span><span class="tt-value">${d.num_sensors || '—'}</span></div>
            <div class="tt-row"><span class="tt-label">GDOP</span><span class="tt-value">${d.gdop != null ? d.gdop.toFixed(1) : '—'}</span></div>
            <div class="tt-row"><span class="tt-label">Residual</span><span class="tt-value">${d.residual_m != null ? d.residual_m.toFixed(0) + ' m' : '—'}</span></div>
            <div class="tt-row"><span class="tt-label">Quality</span><span class="tt-value">${d.track_quality || '—'}</span></div>
            <div class="tt-row"><span class="tt-label">Method</span><span class="tt-value">${d.solve_method || '—'}</span></div>
            <div class="tt-row"><span class="tt-label">Mode</span><span class="tt-value">${d.mlat_mode || '—'}</span></div>
            <div class="tt-row"><span class="tt-label">Source</span><span class="tt-value">${d.position_source || '—'}</span></div>
            <div class="tt-row"><span class="tt-label">Broadcast pos</span><span class="tt-value">${d.uses_broadcast_position === true ? 'yes' : 'no'}</span></div>
        `;
    }

    tooltip.style.display = 'block';
    tooltip.style.left = (info.x + 12) + 'px';
    tooltip.style.top = (info.y + 12) + 'px';
}

// --- Detail Panel (click-to-select) ---

export function showDetailPanel(d) {
    const panel = document.getElementById('detail-panel');
    if (!panel || !d) return;

    const em = isEmergency(d.squawk);
    const emBadge = em ? `<div class="detail-emergency">${EMERGENCY_SQUAWKS[d.squawk].label}</div>` : '';

    panel.innerHTML = `
        <div class="detail-header">
            <h2>${d.icao}</h2>
            ${emBadge}
            <button class="detail-close" onclick="window.__deselectAircraft()">\u2715</button>
        </div>
        ${d.squawk ? `<div class="detail-row"><span class="detail-label">Squawk</span><span class="detail-value ${em ? 'emergency-text' : ''}">${d.squawk}</span></div>` : ''}
        <div class="detail-row"><span class="detail-label">Altitude</span><span class="detail-value">${d.alt_ft != null ? Math.round(d.alt_ft).toLocaleString() + ' ft' : '—'}</span></div>
        <div class="detail-row"><span class="detail-label">Speed</span><span class="detail-value">${d.speed_kts != null ? Math.round(d.speed_kts) + ' kts' : '—'}</span></div>
        <div class="detail-row"><span class="detail-label">Heading</span><span class="detail-value">${d.heading_deg != null ? Math.round(d.heading_deg) + '\u00B0' : '—'}</span></div>
        <div class="detail-row"><span class="detail-label">Vert Rate</span><span class="detail-value">${d.vrate_fpm != null ? Math.round(d.vrate_fpm) + ' fpm' : '—'}</span></div>
        <hr class="detail-divider">
        <div class="detail-row"><span class="detail-label">Sensors</span><span class="detail-value">${d.num_sensors || '—'}</span></div>
        <div class="detail-row"><span class="detail-label">GDOP</span><span class="detail-value">${d.gdop != null ? d.gdop.toFixed(2) : '—'}</span></div>
        <div class="detail-row"><span class="detail-label">Residual</span><span class="detail-value">${d.residual_m != null ? d.residual_m.toFixed(1) + ' m' : '—'}</span></div>
        <div class="detail-row"><span class="detail-label">Quality</span><span class="detail-value">${d.track_quality || '—'}</span></div>
        <div class="detail-row"><span class="detail-label">Method</span><span class="detail-value">${d.solve_method || '—'}</span></div>
        <div class="detail-row"><span class="detail-label">MLAT Mode</span><span class="detail-value">${d.mlat_mode || '—'}</span></div>
        <div class="detail-row"><span class="detail-label">Position Source</span><span class="detail-value">${d.position_source || '—'}</span></div>
        <div class="detail-row"><span class="detail-label">Broadcast Position</span><span class="detail-value">${d.uses_broadcast_position === true ? 'yes' : 'no'}</span></div>
        <div class="detail-row"><span class="detail-label">Age</span><span class="detail-value">${d.age_s != null ? d.age_s.toFixed(0) + 's' : '—'}</span></div>
        <div class="detail-row"><span class="detail-label">Position</span><span class="detail-value">${d.lat != null ? d.lat.toFixed(5) + ', ' + d.lon.toFixed(5) : '—'}</span></div>
    `;
    panel.style.display = 'block';
}

export function hideDetailPanel() {
    const panel = document.getElementById('detail-panel');
    if (panel) panel.style.display = 'none';
}

window.__deselectAircraft = () => {
    state.selectedIcao = null;
    hideDetailPanel();
};

// --- Status Updates ---

export function updateStatus(wsState) {
    const indicator = document.getElementById('ws-indicator');
    const label = document.getElementById('ws-state');
    if (!indicator || !label) return;

    indicator.className = 'ws-status';
    switch (wsState) {
        case 'connected':
            indicator.classList.add('ws-connected');
            label.textContent = 'Connected';
            break;
        case 'disconnected':
            indicator.classList.add('ws-disconnected');
            label.textContent = 'Disconnected';
            break;
        case 'connecting':
            indicator.classList.add('ws-connecting');
            label.textContent = 'Connecting\u2026';
            break;
    }
}

export function updateStatsBar() {
    const s = state.stats;
    const el = (id) => document.getElementById(id);

    const msgSec = el('stat-msg-sec');
    if (msgSec) msgSec.textContent = `${s.messages_per_sec}/s`;

    const medRes = el('stat-med-res');
    if (medRes) medRes.textContent = `${s.median_residual_m} m`;

    const p95Res = el('stat-p95-res');
    if (p95Res) p95Res.textContent = `${s.p95_residual_m} m`;

    const acCount = el('aircraft-count');
    if (acCount) acCount.textContent = state.aircraftData.length;

    const updCount = el('update-count');
    if (updCount) updCount.textContent = state.updateCount;

    const lastUpd = el('last-update');
    if (lastUpd) lastUpd.textContent = new Date().toLocaleTimeString();
}
