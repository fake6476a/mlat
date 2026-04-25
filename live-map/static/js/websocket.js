/**
 * websocket.js — WebSocket connection management.
 */

import { state } from './state.js';

let ws = null;
let controlWs = null;
let reconnectTimer = null;
let onUpdateCallback = null;
let onStatusChangeCallback = null;

export function setCallbacks({ onUpdate, onStatusChange }) {
    onUpdateCallback = onUpdate;
    onStatusChangeCallback = onStatusChange;
}

export function connect() {
    const wsUrl = `ws://${window.location.host}/ws`;
    setStatus('connecting');

    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        setStatus('connected');
        if (reconnectTimer) {
            clearTimeout(reconnectTimer);
            reconnectTimer = null;
        }
        connectControl();
    };

    ws.onmessage = (event) => {
        try {
            const msg = JSON.parse(event.data);
            if (msg.type === 'update' && onUpdateCallback) {
                onUpdateCallback(msg);
            }
        } catch (e) {
            console.error('Parse error:', e);
        }
    };

    ws.onclose = () => {
        setStatus('disconnected');
        scheduleReconnect();
    };

    ws.onerror = () => {
        setStatus('disconnected');
    };
}

function connectControl() {
    const controlUrl = `ws://${window.location.host}/ws/control`;
    controlWs = new WebSocket(controlUrl);

    controlWs.onmessage = (event) => {
        try {
            const status = JSON.parse(event.data);
            state.replay = { ...state.replay, ...status };
        } catch (e) {
            // ignore
        }
    };
}

function scheduleReconnect() {
    if (!reconnectTimer) {
        reconnectTimer = setTimeout(() => {
            reconnectTimer = null;
            connect();
        }, 3000);
    }
}

function setStatus(s) {
    if (onStatusChangeCallback) onStatusChangeCallback(s);
}

export function sendControl(cmd) {
    if (controlWs && controlWs.readyState === WebSocket.OPEN) {
        controlWs.send(JSON.stringify(cmd));
    }
}
