// Session tracking
const appStartTime = Date.now();
const telemetryLog = []; // Stores all received data points
const MAX_TELEMETRY_LOG = 10000; // Memory protection

// =============================================
// TOAST NOTIFICATION SYSTEM
// =============================================
const toastContainer = document.getElementById('toast-container');
function showToast(message, type = 'info', duration = 4000) {
    const icons = { success: 'check_circle', warning: 'warning', error: 'error', info: 'info' };
    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;
    toast.innerHTML = `
        <span class="icon toast-icon">${icons[type] || 'info'}</span>
        <span>${message}</span>
        <button class="toast-close" onclick="this.parentElement.classList.add('toast-exit'); setTimeout(() => this.parentElement.remove(), 300);">×</button>
    `;
    toastContainer.appendChild(toast);
    setTimeout(() => {
        if (toast.parentElement) {
            toast.classList.add('toast-exit');
            setTimeout(() => toast.remove(), 300);
        }
    }, duration);
}

// =============================================
// I18N SYSTEM
// =============================================
const i18n = {
    en: {
        telemetry: 'Telemetry', mission: 'Mission', altitude: 'Altitude', speed: 'Speed',
        power: 'Power', gforce: 'G-Force', orientation: 'Orientation', position: 'Position',
        realtimeGraph: 'Real-time Graph', camera: 'Camera', alerts: 'Alerts/Critical Info',
        alertsTitle: 'Alerts', criticalInfo: 'Critical Info', latency: 'Latency',
        currentCommand: 'Current Command', manualControl: 'Manual Control',
        statistics: 'Statistics', health: 'Health', packets: 'Packets',
        crcErrors: 'CRC Errors', dataPoints: 'Data Points',
        missionControl: 'Mission Control', activeOrder: 'Active Order',
        issueNewOrder: 'Issue New Order', advance: 'Advance', orbit: 'Orbit',
        targetAltitude: 'Target Altitude', targetLat: 'Target Lat', targetLon: 'Target Lon',
        orbitRadius: 'Orbit Radius', direction: 'Direction', send: 'Send', preview: 'Preview',
        missionPlan: 'Mission Plan', activeWaypoint: 'Active Waypoint',
        noWaypoints: 'No waypoints registered. Use the map to add them.',
        selectTelemetryPort: 'Select Telemetry Port', selectCamera: 'Select Camera',
        droneStatus: 'Drone Status', disconnected: 'Disconnected', connected: 'Connected',
        reconnecting: 'Reconnecting...', offline: 'OFFLINE',
        noSignal: 'No telemetry signal received from drone',
        lowAltitude: 'Low Altitude', lowBattery: 'Low Battery',
        exportSuccess: 'Data exported successfully!', exportError: 'Error exporting data',
        selectPath: 'Please select a save path first.',
        selectDestination: 'Please select a destination point on the map first.',
        selectMode: 'Please select a mode (Advance, Orbit, Manual).',
        sent: '✓ Sent', txError: '✗ TX Error', deletePoi: 'Delete this POI?',
        invalidCoords: 'Enter valid Lat/Lon coordinates for the POI.',
        missionExported: 'Mission exported successfully!',
        missionImported: 'Mission imported!', missionCleared: 'All waypoints cleared.',
        noWaypointsExport: 'No waypoints to export.',
        confirmClear: 'Clear all waypoints?',
        highGforce: '⚠ High G-Force detected!', negativeAlt: '⚠ Negative altitude!',
        highSpeed: '⚠ Excessive speed!', extremeAlt: '⚠ Extreme altitude!',
    },
    es: {
        telemetry: 'Telemetría', mission: 'Misión', altitude: 'Altitud', speed: 'Velocidad',
        power: 'Potencia', gforce: 'Fuerza G', orientation: 'Orientación', position: 'Posición',
        realtimeGraph: 'Gráfico en Tiempo Real', camera: 'Cámara', alerts: 'Alertas/Info Crítica',
        alertsTitle: 'Alertas', criticalInfo: 'Info Crítica', latency: 'Latencia',
        currentCommand: 'Comando Actual', manualControl: 'Control Manual',
        statistics: 'Estadísticas', health: 'Salud', packets: 'Paquetes',
        crcErrors: 'Errores CRC', dataPoints: 'Puntos de Datos',
        missionControl: 'Control de Misión', activeOrder: 'Orden Activa',
        issueNewOrder: 'Nueva Orden', advance: 'Avanzar', orbit: 'Orbitar',
        targetAltitude: 'Altitud Objetivo', targetLat: 'Lat Objetivo', targetLon: 'Lon Objetivo',
        orbitRadius: 'Radio de Órbita', direction: 'Dirección', send: 'Enviar', preview: 'Vista Previa',
        missionPlan: 'Plan de Misión', activeWaypoint: 'Waypoint Activo',
        noWaypoints: 'Sin waypoints registrados. Use el mapa para agregar.',
        selectTelemetryPort: 'Seleccionar Puerto', selectCamera: 'Seleccionar Cámara',
        droneStatus: 'Estado del Dron', disconnected: 'Desconectado', connected: 'Conectado',
        reconnecting: 'Reconectando...', offline: 'FUERA DE LÍNEA',
        noSignal: 'Sin señal de telemetría del dron',
        lowAltitude: 'Altitud Baja', lowBattery: 'Batería Baja',
        exportSuccess: '¡Datos exportados exitosamente!', exportError: 'Error al exportar datos',
        selectPath: 'Seleccione una ruta de guardado primero.',
        selectDestination: 'Seleccione un punto destino en el mapa primero.',
        selectMode: 'Seleccione un modo (Avanzar, Orbitar, Manual).',
        sent: '✓ Enviado', txError: '✗ Error TX', deletePoi: '¿Eliminar este POI?',
        invalidCoords: 'Ingrese coordenadas Lat/Lon válidas.',
        missionExported: '¡Misión exportada exitosamente!',
        missionImported: '¡Misión importada!', missionCleared: 'Todos los waypoints eliminados.',
        noWaypointsExport: 'No hay waypoints para exportar.',
        confirmClear: '¿Eliminar todos los waypoints?',
        highGforce: '⚠ ¡Alta Fuerza G detectada!', negativeAlt: '⚠ ¡Altitud negativa!',
        highSpeed: '⚠ ¡Velocidad excesiva!', extremeAlt: '⚠ ¡Altitud extrema!',
    }
};

let currentLang = localStorage.getItem('cadi_lang') || 'en';
function t(key) { return (i18n[currentLang] && i18n[currentLang][key]) || i18n.en[key] || key; }

function applyLanguage(lang) {
    currentLang = lang;
    localStorage.setItem('cadi_lang', lang);
    // Update static UI text
    document.querySelector('.tab-btn[data-target="telemetry-view"]').textContent = t('telemetry');
    document.querySelector('.tab-btn[data-target="commands-view"]').textContent = t('mission');
    document.querySelector('.tab-btn[data-target="debug-view"]').textContent = 'Debug Station';
    document.querySelector('#drone-state-indicator').textContent = t('droneStatus');
}

// =============================================
// STATISTICS TRACKER
// =============================================
const statsTracker = {
    alt: { min: Infinity, max: -Infinity, sum: 0, count: 0 },
    spd: { min: Infinity, max: -Infinity, sum: 0, count: 0 },
    g:   { min: Infinity, max: -Infinity, sum: 0, count: 0 },
    update(alt, spd, g) {
        this._track(this.alt, alt);
        this._track(this.spd, spd);
        this._track(this.g, g);
        if (this.alt.count % 5 === 0) this.render(); // render every 5th update
    },
    _track(obj, val) {
        if (val < obj.min) obj.min = val;
        if (val > obj.max) obj.max = val;
        obj.sum += val;
        obj.count++;
    },
    render() {
        const fmt = (v) => isFinite(v) ? v.toFixed(1) : '--';
        const avg = (o) => o.count > 0 ? o.sum / o.count : 0;
        document.getElementById('stat-alt-min').textContent = fmt(this.alt.min);
        document.getElementById('stat-alt-max').textContent = fmt(this.alt.max);
        document.getElementById('stat-alt-avg').textContent = fmt(avg(this.alt));
        document.getElementById('stat-spd-min').textContent = fmt(this.spd.min);
        document.getElementById('stat-spd-max').textContent = fmt(this.spd.max);
        document.getElementById('stat-spd-avg').textContent = fmt(avg(this.spd));
        document.getElementById('stat-g-min').textContent = fmt(this.g.min);
        document.getElementById('stat-g-max').textContent = fmt(this.g.max);
        document.getElementById('stat-g-avg').textContent = fmt(avg(this.g));
    },
    reset() {
        this.alt = { min: Infinity, max: -Infinity, sum: 0, count: 0 };
        this.spd = { min: Infinity, max: -Infinity, sum: 0, count: 0 };
        this.g =   { min: Infinity, max: -Infinity, sum: 0, count: 0 };
        this.render();
    }
};

document.getElementById('reset-stats')?.addEventListener('click', () => {
    statsTracker.reset();
    showToast('Statistics reset', 'info');
});

// Stats collapsible toggle
document.getElementById('toggle-stats')?.addEventListener('click', () => {
    const grid = document.getElementById('stats-grid');
    const icon = document.querySelector('#toggle-stats .icon');
    grid.classList.toggle('collapsed');
    icon.textContent = grid.classList.contains('collapsed') ? 'expand_more' : 'expand_less';
});

// =============================================
// MAP TILE LAYERS
// =============================================
const tileLayers = {
    dark: 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',
    satellite: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    terrain: 'https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',
    streets: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png'
};

function switchMapLayer(mapInstance, layerKey, buttonContainer) {
    // Remove current tile layer
    mapInstance.eachLayer(layer => { if (layer instanceof L.TileLayer) mapInstance.removeLayer(layer); });
    L.tileLayer(tileLayers[layerKey] || tileLayers.dark, { maxZoom: 19 }).addTo(mapInstance);
    // Update button states
    buttonContainer.querySelectorAll('.map-layer-btn').forEach(b => {
        b.classList.toggle('active', b.dataset.layer === layerKey);
    });
    localStorage.setItem('cadi_map_layer', layerKey);
}

// =============================================
// USER PREFERENCES
// =============================================
function loadPreferences() {
    const lang = localStorage.getItem('cadi_lang') || 'en';
    const langSelect = document.getElementById('lang-select');
    if (langSelect) langSelect.value = lang;
    applyLanguage(lang);
}

// Language selector
document.getElementById('lang-select')?.addEventListener('change', (e) => {
    applyLanguage(e.target.value);
    showToast(currentLang === 'es' ? 'Idioma cambiado a Español' : 'Language changed to English', 'success');
});

// =============================================
// SERIAL STATUS LISTENER
// =============================================
if (window.electronAPI.onSerialStatus) {
    window.electronAPI.onSerialStatus((status) => {
        const pill = document.getElementById('serial-status-pill');
        const text = document.getElementById('serial-status-text');
        pill.className = 'serial-status-indicator';
        if (status.status === 'connected') {
            pill.classList.add('connected');
            text.textContent = t('connected');
            setOnlineState(true); // Board detected
            showToast(`Serial ${t('connected')}: ${status.port || ''}`, 'success');
        } else if (status.status === 'disconnected' || status.status === 'failed' || status.status === 'error') {
            text.textContent = t('disconnected');
            setOnlineState(false); // Hardware not detected
            showToast(`Serial ${t('disconnected')}`, 'warning');
        } else if (status.status === 'reconnecting') {
            pill.classList.add('reconnecting');
            text.textContent = `${t('reconnecting')} (${status.attempt}/${status.maxRetries})`;
        } else if (status.status === 'error') {
            text.textContent = 'Error';
            showToast(`Serial Error: ${status.error}`, 'error');
        } else if (status.status === 'failed') {
            text.textContent = t('disconnected');
            showToast('Serial reconnection failed', 'error', 6000);
        }
    });
}

// Log message listener
if (window.electronAPI.onLogMessage) {
    window.electronAPI.onLogMessage((log) => {
        if (log.level === 'ERROR') showToast(log.message, 'error', 6000);
    });
}

function formatElapsed(ms) {
    const totalSeconds = Math.floor(ms / 1000);
    const minutes = Math.floor(totalSeconds / 60);
    const seconds = totalSeconds % 60;
    return `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
}

// =============================================
// MAP SETUP – Telemetry Map
// =============================================
const map = L.map('map', {
    zoomControl: false,
    attributionControl: false
}).setView([-33.456, -70.648], 15);

// Dark high-contrast tile layer
L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
    maxZoom: 19,
}).addTo(map);

// Custom drone icon for telemetry map
const droneIcon = L.divIcon({
    className: '',
    html: `<div style="
        width:18px;height:18px;
        border-radius:50%;
        background:radial-gradient(circle, var(--drone-glow) 0%, #4589f5 60%, rgba(69,137,245,0) 100%);
        box-shadow:0 0 12px var(--drone-glow), 0 0 24px rgba(0,229,255,0.5);
        border:2px solid #fff;
    "></div>`,
    iconAnchor: [9, 9]
});
const droneMarker = L.marker([-33.456, -70.648], { icon: droneIcon }).addTo(map);

// Trail polyline for telemetry map
const MAX_TRAIL_POINTS = 80;
const trailPoints = [];
const droneTrail = L.polyline([], {
    color: '#00e5ff',
    weight: 2,
    opacity: 0.6,
    dashArray: '4 4'
}).addTo(map);

function updateTrail(lat, lon) {
    trailPoints.push([lat, lon]);
    if (trailPoints.length > MAX_TRAIL_POINTS) trailPoints.shift();
    droneTrail.setLatLngs(trailPoints);
    // Sync mission map trail if initialized
    if (missionTrail) {
        missionTrail.setLatLngs(trailPoints);
        if (missionDroneMarker) missionDroneMarker.setLatLng([lat, lon]);
    }
}

// Chart.js Initialization
const ctx = document.getElementById('telemetryChart').getContext('2d');
const telemetryChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [], // Time labels
        datasets: [
            {
                label: 'Altitude',
                data: [],
                borderColor: '#4589f5',
                backgroundColor: 'rgba(69, 137, 245, 0.1)',
                fill: true,
                tension: 0.4,
                hidden: false
            },
            {
                label: 'Speed',
                data: [],
                borderColor: '#f5a623',
                backgroundColor: 'rgba(245, 166, 35, 0.1)',
                fill: true,
                tension: 0.4,
                hidden: true
            },
            {
                label: 'Power',
                data: [],
                borderColor: '#ff4d4d',
                backgroundColor: 'rgba(255, 77, 77, 0.1)',
                fill: true,
                tension: 0.4,
                hidden: true
            },
            {
                label: 'G-Force',
                data: [],
                borderColor: '#00e676',
                backgroundColor: 'rgba(0, 230, 118, 0.1)',
                fill: true,
                tension: 0.4,
                hidden: true
            }
        ]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        scales: {
            y: {
                beginAtZero: true,
                grid: { color: 'rgba(255, 255, 255, 0.1)' },
                ticks: { color: '#8892b0' }
            },
            x: {
                grid: { display: false },
                ticks: { color: '#8892b0' }
            }
        },
        plugins: {
            legend: { 
                display: true,
                labels: {
                    color: '#8892b0',
                    font: { family: 'Inter' }
                }
            }
        }
    }
});

// Graph Filters Logic
document.querySelectorAll('.filter-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        const datasetIndex = parseInt(btn.getAttribute('data-dataset'));
        const isVisible = telemetryChart.isDatasetVisible(datasetIndex);
        
        if (isVisible) {
            telemetryChart.hide(datasetIndex);
            btn.classList.remove('active');
        } else {
            telemetryChart.show(datasetIndex);
            btn.classList.add('active');
        }
    });
});

// Camera Access & Selection
const cameraSelect = document.getElementById('camera-select');
let currentStream = null;

async function listCameras() {
    try {
        const devices = await navigator.mediaDevices.enumerateDevices();
        const videoDevices = devices.filter(device => device.kind === 'videoinput');
        
        cameraSelect.innerHTML = '<option value="">Select Camera</option>';
        videoDevices.forEach(device => {
            const option = document.createElement('option');
            option.value = device.deviceId;
            option.text = device.label || `Camera ${cameraSelect.length}`;
            cameraSelect.appendChild(option);
        });
    } catch (error) {
        console.error('Error listing cameras:', error);
    }
}

async function startCamera(deviceId = null) {
    if (currentStream) {
        currentStream.getTracks().forEach(track => track.stop());
    }

    const constraints = {
        video: deviceId ? { deviceId: { exact: deviceId } } : { width: 1280, height: 720 }
    };

    try {
        currentStream = await navigator.mediaDevices.getUserMedia(constraints);
        const video = document.getElementById('webcam');
        if (video) {
            video.srcObject = currentStream;
            video.onloadedmetadata = () => {
                video.play().catch(e => console.error("Video play failed:", e));
            };
        }
    } catch (error) {
        console.error('Error accessing camera:', error);
    }
}

cameraSelect.addEventListener('change', (e) => {
    if (e.target.value) {
        startCamera(e.target.value);
    }
});

listCameras();
startCamera();

// Telemetry Port Selection
const portSelect = document.getElementById('telemetry-port-select');
const refreshPortsBtn = document.getElementById('refresh-ports');

async function listPorts() {
    try {
        const ports = await window.electronAPI.listPorts();
        const currentSelection = portSelect.value;
        portSelect.innerHTML = '<option value="">Select Telemetry Port</option>';
        ports.forEach(port => {
            const option = document.createElement('option');
            option.value = port.path;
            option.text = port.friendlyName || port.path;
            portSelect.appendChild(option);
        });
        // Restore selection if still available
        if (currentSelection) {
            const exists = Array.from(portSelect.options).some(opt => opt.value === currentSelection);
            if (exists) portSelect.value = currentSelection;
        }
    } catch (error) {
        console.error('Error listing ports:', error);
    }
}

refreshPortsBtn.addEventListener('click', () => {
    listPorts();
    // Visual feedback
    const icon = refreshPortsBtn.querySelector('.icon');
    icon.style.animation = 'spin 1s linear';
    setTimeout(() => { icon.style.animation = ''; }, 1000);
});

portSelect.addEventListener('change', async (e) => {
    if (e.target.value) {
        const baudRate = document.getElementById('baud-rate-select').value;
        const result = await window.electronAPI.connectSerial(e.target.value, baudRate);
        if (result.success) {
            showToast(`Connected to ${e.target.value} @ ${baudRate}`, 'success');
        } else {
            showToast('Failed to connect: ' + result.error, 'error');
        }
    }
});


listPorts();

// Map State
let isFollowTelemetry = false;
let isFollowMission = false;

function setupMapControls(mapId, followId, centerId) {
    const followBtn = document.getElementById(followId);
    const centerBtn = document.getElementById(centerId);
    
    followBtn.addEventListener('click', () => {
        if (followId === 'follow-telemetry') isFollowTelemetry = !isFollowTelemetry;
        else isFollowMission = !isFollowMission;
        
        followBtn.classList.toggle('active', (followId === 'follow-telemetry' ? isFollowTelemetry : isFollowMission));
        
        if (followId === 'follow-telemetry' ? isFollowTelemetry : isFollowMission) {
            if (window._lastDroneLat && window._lastDroneLon) {
                if (mapId === 'map') map.panTo([window._lastDroneLat, window._lastDroneLon]);
                else missionMap.panTo([window._lastDroneLat, window._lastDroneLon]);
            }
        }
    });

    centerBtn.addEventListener('click', () => {
        if (window._lastDroneLat && window._lastDroneLon) {
            if (mapId === 'map') map.panTo([window._lastDroneLat, window._lastDroneLon]);
            else missionMap.panTo([window._lastDroneLat, window._lastDroneLon]);
        }
    });
}

// Telemetry Handling
window.electronAPI.onTelemetryData((data) => {
    // Expected Data Format: Object
    if (!data || typeof data !== 'object' || Array.isArray(data)) return;

    const {
        latitude: lat,
        longitude: lon,
        altitude: alt,
        yaw,
        pitch,
        roll,
        gforce,
        velocity_mag,
        currentMode,
        cmd_yaw,
        cmd_pitch,
        cmd_roll,
        cmd_throttle,
        pos_local_x,
        pos_local_y,
        pos_local_z,
        lossRate = 0
    } = data;
    
    const elapsedMs = Date.now() - appStartTime;
    const formattedTime = formatElapsed(elapsedMs);

    const modes = ["Manual", "Waypoint", "Orbit"];
    const state = modes[currentMode] || "Unknown";
    const speed = velocity_mag * 3.6; // Convert m/s to Km/h
    const bearing = yaw;

    // Update Metrics
    document.querySelector('#altitude .metric-value').innerHTML = `${alt.toFixed(1)} <span class="unit">m</span>`;
    document.querySelector('#speed .metric-value').innerHTML = `${speed.toFixed(1)} <span class="unit">Km/h</span>`;
    const powerPct = Math.round((cmd_throttle / 180) * 100);
    document.querySelector('#power .metric-value').innerHTML = `${powerPct} <span class="unit">%</span>`;
    document.querySelector('#gforce .metric-value').innerHTML = `${gforce.toFixed(2)} <span class="unit">g</span>`;
    
    // Update Debug / System Status (New Tab Fields)
    const setVal = (id, val) => {
        const el = document.getElementById(id);
        if (el) el.textContent = val;
    };

    setVal('db-target-roll', `${cmd_roll.toFixed(1)}°`);
    setVal('db-target-pitch', `${cmd_pitch.toFixed(1)}°`);
    setVal('db-target-yaw', `${cmd_yaw.toFixed(1)}°`);
    setVal('db-target-throttle', cmd_throttle);

    setVal('db-pos-x', `${pos_local_x.toFixed(1)}m`);
    setVal('db-pos-y', `${pos_local_y.toFixed(1)}m`);
    setVal('db-pos-z', `${pos_local_z.toFixed(1)}m`);

    setVal('db-pkt-total', (window._totalPackets || 0) + 1);
    window._totalPackets = (window._totalPackets || 0) + 1;
    setVal('db-pkt-rate', `${lossRate}%`);
    
    // Update Signal Quality [F3]
    const signalVal = document.getElementById('signal-quality-value');
    const signalIcon = document.querySelector('#signal-strength-widget .icon');
    if (signalVal && signalIcon) {
        const quality = 100 - lossRate;
        signalVal.innerText = `${quality}%`;
        if (quality > 80) signalIcon.style.color = '#4caf50'; // Green
        else if (quality > 40) signalIcon.style.color = '#ffeb3b'; // Yellow
        else signalIcon.style.color = '#f44336'; // Red
    }

    // Update Orientation
    document.getElementById('pitch').innerText = `${pitch.toFixed(1)}°`;
    document.getElementById('roll').innerText = `${roll.toFixed(1)}°`;
    document.getElementById('yaw').innerText = `${yaw.toFixed(1)}°`;
    document.getElementById('bearing').innerText = `${bearing.toFixed(1)}°`;

    // Handle incoming device messages [F4]
    if (window.electronAPI.onDeviceMessage && !window._messageHandlerAdded) {
        window.electronAPI.onDeviceMessage((msg) => {
            const consoleEl = document.getElementById('debug-console');
            if (consoleEl) {
                const entry = document.createElement('div');
                entry.className = 'log-entry device';
                const time = new Date().toLocaleTimeString();
                entry.textContent = `[${time}] ${msg}`;
                consoleEl.appendChild(entry);
                consoleEl.scrollTop = consoleEl.scrollHeight;
            }
        });
        window._messageHandlerAdded = true;
    }

    // Update Map
    const coords = [lat, lon];
    droneMarker.setLatLng(coords);
    if (isFollowTelemetry) map.panTo(coords);
    document.getElementById('lat').innerText = `Lat: ${lat.toFixed(6)}`;
    document.getElementById('lon').innerText = `Lon: ${lon.toFixed(6)}`;
    document.querySelector('.drone-status').innerText = `State: ${state}`;

    // Update drone trail on both maps
    updateTrail(lat, lon);

    // Call HUD drawing
    drawHUD(pitch, roll, bearing, alt, speed);

    // Track last known position for mission map auto-centering
    if (window._telemetryUpdateHook) window._telemetryUpdateHook(lat, lon);
    window._lastDroneLat = lat;
    window._lastDroneLon = lon;

    // Hide OFFLINE banner on valid data
    setOnlineState(true);

    // Update Log for CSV Export
    telemetryLog.push({
        elapsed: formattedTime,
        ms: elapsedMs,
        lat, lon, alt, state, gforce, speed, pitch, roll, yaw, bearing
    });
    if (telemetryLog.length > MAX_TELEMETRY_LOG) {
        telemetryLog.splice(0, telemetryLog.length - MAX_TELEMETRY_LOG);
    }

    // Update Statistics
    statsTracker.update(alt, speed, gforce);

    // Anomaly Detection
    if (data._anomalies && data._anomalies.length > 0) {
        if (!window._lastAnomalyTime) window._lastAnomalyTime = {};
        const now = Date.now();
        data._anomalies.forEach(anomaly => {
            const key = { HIGH_GFORCE: 'highGforce', NEGATIVE_ALT: 'negativeAlt', HIGH_SPEED: 'highSpeed', EXTREME_ALT: 'extremeAlt' }[anomaly];
            if (key) {
                if (!window._lastAnomalyTime[anomaly] || (now - window._lastAnomalyTime[anomaly] > 5000)) {
                    window._lastAnomalyTime[anomaly] = now;
                    showToast(t(key), 'warning', 5000);
                }
            }
        });
    }

    // Update Health Dashboard
    document.getElementById('health-data-points').textContent = telemetryLog.length;
});

// =============================================
// CHART DATA SYNC (Independent of Serial)
// =============================================
function updateChartFromDashboard() {
    const elapsedMs = Date.now() - appStartTime;
    const formattedTime = formatElapsed(elapsedMs);
    
    // Parse values from dashboard HTML (enables simulation sync)
    const alt = parseFloat(document.querySelector('#altitude .metric-value').textContent) || 0;
    const speed = parseFloat(document.querySelector('#speed .metric-value').textContent) || 0;
    const power = parseFloat(document.querySelector('#power .metric-value').textContent) || 0;
    const gforce = parseFloat(document.querySelector('#gforce .metric-value').textContent) || 0;

    telemetryChart.data.labels.push(formattedTime);
    telemetryChart.data.datasets[0].data.push(alt);
    telemetryChart.data.datasets[1].data.push(speed);
    telemetryChart.data.datasets[2].data.push(power); 
    telemetryChart.data.datasets[3].data.push(gforce);

    if (telemetryChart.data.labels.length > 20) {
        telemetryChart.data.labels.shift();
        telemetryChart.data.datasets.forEach(ds => ds.data.shift());
    }
    telemetryChart.update('none');
}

// Run chart sync periodically
setInterval(updateChartFromDashboard, 1000);

// Removed duplicate fake data logic since it is in main.js simulator
// Export Modal Logic
const exportModal = document.getElementById('export-modal');
const openExportBtn = document.getElementById('open-export');
const closeExportBtn = document.getElementById('close-export');
const browsePathBtn = document.getElementById('browse-path');
const confirmExportBtn = document.getElementById('confirm-export');
const exportPathInput = document.getElementById('export-path');

openExportBtn.addEventListener('click', () => {
    exportModal.classList.add('active');
    const lastPoint = telemetryLog[telemetryLog.length - 1];
    if (lastPoint) {
        document.getElementById('export-end').value = lastPoint.elapsed;
    }
});

closeExportBtn.addEventListener('click', () => {
    exportModal.classList.remove('active');
});

browsePathBtn.addEventListener('click', async () => {
    const path = await window.electronAPI.selectSavePath();
    if (path) {
        exportPathInput.value = path;
    }
});

confirmExportBtn.addEventListener('click', async () => {
    const path = exportPathInput.value;
    if (!path) {
        showToast(t('selectPath'), 'warning');
        return;
    }
    const selectedStartTime = document.getElementById('export-start').value;
    const selectedEndTime = document.getElementById('export-end').value;
    const selectedVars = Array.from(document.querySelectorAll('input[name="export-var"]:checked'))
                              .map(cb => cb.value);
    const filteredData = telemetryLog.filter(point => {
        return point.elapsed >= selectedStartTime && (selectedEndTime === "Now" || point.elapsed <= selectedEndTime);
    });
    if (filteredData.length === 0) {
        showToast('No data found for the selected time range.', 'warning');
        return;
    }
    const headers = selectedVars.join(',');
    const rows = filteredData.map(point => {
        return selectedVars.map(v => {
            if (v === 'timestamp') return point.elapsed;
            return point[v];
        }).join(',');
    });
    const csvContent = headers + '\n' + rows.join('\n');
    const result = await window.electronAPI.saveCSVFile(path, csvContent);
    if (result.success) {
        showToast(t('exportSuccess'), 'success');
        exportModal.classList.remove('active');
    } else {
        showToast(t('exportError') + ': ' + result.error, 'error');
    }
});

// =============================================
// CONFIG MODAL LOGIC
// =============================================
const configModal = document.getElementById('config-modal');
const openConfigBtn = document.getElementById('open-config');
const closeConfigBtn = document.getElementById('close-config');
const configTabs = configModal.querySelectorAll('.modal-body .tab-btn');
const configSections = {
    'poi-config': document.getElementById('poi-config'),
    'appearance-config': document.getElementById('appearance-config'),
    'language-config': document.getElementById('language-config'),
    'calibration-config': document.getElementById('calibration-config'),
};

openConfigBtn.addEventListener('click', () => {
    configModal.classList.add('active');
});

closeConfigBtn.addEventListener('click', () => {
    configModal.classList.remove('active');
});

configTabs.forEach(btn => {
    btn.addEventListener('click', () => {
        const target = btn.getAttribute('data-config-target');
        configTabs.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        Object.entries(configSections).forEach(([id, el]) => {
            if (el) el.style.display = id === target ? '' : 'none';
        });
    });
});

// Customization Handlers
const applyLogoBtn = document.getElementById('apply-logo-btn');
applyLogoBtn.addEventListener('click', () => {
    const url = document.getElementById('logo-url-input').value.trim();
    if (url) {
        const h1 = document.querySelector('.title-section h1');
        if (h1) h1.innerHTML = `<img src="${url}" alt="Logo" style="height: 30px; vertical-align: middle; margin-right: 10px;"> C.A.D.I`;
    }
});

// Tab Switching Logic (Main Tabs)
const tabButtons = document.querySelectorAll('.tab-btn:not(.modal-body .tab-btn)');
const views = {
    'telemetry-view': document.getElementById('telemetry-view'),
    'commands-view': document.getElementById('commands-view'),
    'debug-view': document.getElementById('debug-view'),
};

let missionMapInitialized = false;
let missionMap = null;
let missionMapTargetMarker = null;

tabButtons.forEach(btn => {
    btn.addEventListener('click', () => {
        const target = btn.getAttribute('data-target');
        if (!target) return;

        tabButtons.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');

        Object.entries(views).forEach(([id, el]) => {
            if (el) el.style.display = id === target ? '' : 'none';
        });

        // Lazy-initialize the mission map only when opening the Commands view
        if (target === 'commands-view' && !missionMapInitialized) {
            initMissionMap();
            missionMapInitialized = true;
        }

    // Inform Leaflet and Chart.js of resize when switching so components render correctly
        if (target === 'commands-view' && missionMap) {
            setTimeout(() => {
                missionMap.invalidateSize();
            }, 100);
        } else if (target === 'telemetry-view') {
            setTimeout(() => {
                if (map) map.invalidateSize();
                if (telemetryChart) telemetryChart.resize();
            }, 100);
        }
    });
});

// =============================================
// MISSION MAP INITIALIZATION
// =============================================
function initMissionMap() {
    missionMap = L.map('mission-map', {
        zoomControl: true,
        attributionControl: false,
    }).setView(
        map.getCenter(),
        map.getZoom()
    );

    // Dark high-contrast tile layer (matches telemetry map)
    L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
        maxZoom: 19,
    }).addTo(missionMap);

    // Maps are independent for free view logic

    // Drone trail on the mission map
    missionTrail = L.polyline(trailPoints.slice(), {
        color: '#00e5ff',
        weight: 2,
        opacity: 0.6,
        dashArray: '4 4'
    }).addTo(missionMap);

    // Drone position marker on mission map
    missionDroneMarker = L.marker(
        map.getCenter(),
        { icon: droneIcon }
    ).addTo(missionMap);

    setupMapControls('mission-map', 'follow-mission', 'center-mission');

    // Click on the map to set target coordinates
    missionMap.on('click', (e) => {
        const { lat, lng } = e.latlng;
        document.getElementById('cmd-lat').value = lat.toFixed(7);
        document.getElementById('cmd-lon').value = lng.toFixed(7);
        document.getElementById('cmd-map-lat').innerText = `Lat ${lat.toFixed(6)}`;
        document.getElementById('cmd-map-lon').innerText = `Lon ${lng.toFixed(6)}`;

        if (missionMapTargetMarker) {
            missionMapTargetMarker.setLatLng([lat, lng]);
        } else {
            missionMapTargetMarker = L.marker([lat, lng], {
                icon: buildWpIcon(selectedMode)
            }).addTo(missionMap);
        }
    });

    // Add existing POIs to mission map
    pois.forEach((poi, index) => {
        const icon = buildPoiIcon(poi.icon, poi.customPath);
        const marker = L.marker([poi.lat, poi.lon], { icon }).addTo(missionMap);
        poiMissionMarkers[index] = marker;
    });
}


// Mission map trail and drone reference (populated when initMissionMap runs)
let missionTrail = null;
let missionDroneMarker = null;
const missionWpMarkers = []; // One marker per waypoint on the mission map

// Store last known drone coords so mission map can auto-center
window._lastDroneLat = null;
window._lastDroneLon = null;

// =============================================
// ORDER MODE SELECTION
// =============================================
let selectedMode = null; // null = none, 0=Manual, 1=Waypoint, 2=Orbit
const modeNames = { 0: 'Manual Control', 1: 'Advance', 2: 'Orbit' };
let selectedDirection = 0; // 0=CCW, 1=CW


// Show/hide orbit fields based on selected mode
function updateOrbitFieldsVisibility(mode) {
    const orbitFields = document.getElementById('orbit-fields');
    if (orbitFields) {
        orbitFields.style.display = (mode === 2) ? 'flex' : 'none';
    }
}

document.querySelectorAll('.order-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.order-btn').forEach(b => b.classList.remove('selected'));
        btn.classList.add('selected');
        selectedMode = parseInt(btn.getAttribute('data-mode'));
        document.getElementById('current-order-display').innerText = modeNames[selectedMode] || 'Ninguna';
        updateOrbitFieldsVisibility(selectedMode);
    });
});

// Direction toggle (CW/CCW) for orbit mode
document.querySelectorAll('.dir-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.dir-btn').forEach(b => b.classList.remove('selected'));
        btn.classList.add('selected');
        selectedDirection = parseInt(btn.getAttribute('data-dir'));
    });
});

// =============================================
// WAYPOINT MANAGEMENT
// =============================================
const waypoints = []; // Array of { lat, lon, alt, mode }
let activeWaypointIdx = 0;

function renderWaypoints() {
    const grid = document.getElementById('waypoints-grid');
    const idxSpan = document.getElementById('wp-active-idx');
    grid.innerHTML = '';

    if (waypoints.length === 0) {
        grid.innerHTML = `<div class="waypoint-item empty-state">No waypoints registered. Use the map to add them.</div>`;
        idxSpan.innerText = '–';
        return;
    }


    idxSpan.innerText = activeWaypointIdx + 1;

    waypoints.forEach((wp, i) => {
        const el = document.createElement('div');
        el.className = 'waypoint-item' + (i === activeWaypointIdx ? ' active-wp' : '');
        
        let extraInfo = '';
        if (wp.mode === 2) { // Orbit
            const dirLabel = wp.direction === 1 ? 'CW' : 'CCW';
            extraInfo = `<br>Rad: ${wp.radius}m | Dir: ${dirLabel}`;
        }

        el.innerHTML = `
            <span class="wp-number">WP ${i + 1} – ${modeNames[wp.mode] || '?'}</span>
            Lat: ${parseFloat(wp.lat).toFixed(5)}<br>
            Lon: ${parseFloat(wp.lon).toFixed(5)}<br>
            Alt: ${wp.alt} m${extraInfo}
        `;
        el.addEventListener('click', () => {
            activeWaypointIdx = i;
            renderWaypoints();
        });
        grid.appendChild(el);
    });
}

// Send Command button
document.getElementById('send-command-btn').addEventListener('click', async () => {
    const lat = parseFloat(document.getElementById('cmd-lat').value);
    const lon = parseFloat(document.getElementById('cmd-lon').value);
    const alt = parseFloat(document.getElementById('cmd-alt').value) || 100;
    const radius = parseFloat(document.getElementById('cmd-radius')?.value) || 50;

    if (isNaN(lat) || isNaN(lon)) {
        showToast(t('selectDestination'), 'warning');
        return;
    }

    if (selectedMode === null) {
        showToast(t('selectMode'), 'warning');
        return;
    }


    const wp = { lat, lon, alt, mode: selectedMode, direction: selectedDirection, radius };
    waypoints.push(wp);
    activeWaypointIdx = waypoints.length - 1;

    // Place a marker on the mission map if available
    if (missionMap) {
        const marker = L.marker([lat, lon], { icon: buildWpIcon(selectedMode) })
            .bindTooltip(`WP ${waypoints.length} – ${modeNames[selectedMode]}`, { permanent: false, direction: 'top' })
            .addTo(missionMap);
        missionWpMarkers.push(marker);
    }

    renderWaypoints();

    // Determine masterMode and order from selected mode
    // mode 0 = Manual -> masterMode=1, order=0
    // mode 1 = Avanzar (Waypoint) -> masterMode=2, order=1
    // mode 2 = Orbitar -> masterMode=2, order=2
    const masterMode = selectedMode === 0 ? 1 : 2;
    const order      = selectedMode === 0 ? 0 : selectedMode;

    const cfg = {
        masterMode,
        order,
        lat,
        lon,
        alt,
        direction: selectedDirection,
        radius
    };

    try {
        const result = await window.electronAPI.sendCommand(cfg);
        const btn = document.getElementById('send-command-btn');
        if (result.success) {
            btn.innerText = '✓ Sent';
            btn.style.background = '#28a745';
            setTimeout(() => { btn.innerText = 'Send'; btn.style.background = ''; }, 2000);
        } else {
            btn.innerText = '✗ TX Error';
            btn.style.background = '#dc3545';
            setTimeout(() => { btn.innerText = 'Send'; btn.style.background = ''; }, 2500);
            console.warn('TX error:', result.error);
        }

    } catch (e) {
        console.error('sendCommand failed:', e);
    }
});

// ── [F7] Apply Magnetic Declination ──
document.getElementById('apply-declination-btn')?.addEventListener('click', async () => {
    const dec = parseFloat(document.getElementById('declination-input').value);
    if (isNaN(dec)) return;
    
    // We send a ConfigPacket with the current mission state but updated declination
    const cfg = {
        masterMode: selectedMode === 0 ? 1 : 2,
        order: selectedMode === 0 ? 0 : selectedMode,
        lat: parseFloat(document.getElementById('cmd-lat').value) || 0,
        lon: parseFloat(document.getElementById('cmd-lon').value) || 0,
        alt: parseFloat(document.getElementById('cmd-alt').value) || 100,
        direction: selectedDirection,
        radius: parseFloat(document.getElementById('cmd-radius')?.value) || 50,
        declination: dec
    };
    
    const result = await window.electronAPI.sendCommand(cfg);
    if (result.success) showToast('Declination updated', 'success');
});

// ── [F2] Bulk Mission Upload ──
document.getElementById('upload-mission')?.addEventListener('click', async () => {
    if (waypoints.length === 0) {
        showToast(t('noWaypointsExport'), 'warning');
        return;
    }
    
    const result = await window.electronAPI.uploadMission(waypoints);
    if (result.success) {
        showToast('Mission uploaded successfully', 'success');
    } else {
        showToast('Upload failed: ' + result.error, 'error');
    }
});

// ── [F2] Clear Mission ──
document.getElementById('clear-mission')?.addEventListener('click', () => {
    if (confirm(t('confirmClear'))) {
        waypoints.length = 0;
        missionWpMarkers.forEach(m => missionMap.removeLayer(m));
        missionWpMarkers.length = 0;
        renderWaypoints();
        showToast(t('missionCleared'), 'info');
    }
});

// =============================================
// WAYPOINT ICON BUILDER
// Unique icon per mission mode
// =============================================
function buildWpIcon(mode) {
    const configs = {
        0: { color: '#f5a623', label: '✦', title: 'Manual' },   // orange diamond
        1: { color: '#4589f5', label: '▶', title: 'Avanzar' },  // blue arrow
        2: { color: '#c678dd', label: '↻', title: 'Orbitar' },  // purple cycle
    };
    const cfg = configs[mode] ?? { color: '#4589f5', label: '●', title: 'WP' };
    return L.divIcon({
        className: '',
        html: `<div style="
            min-width:24px; height:24px; border-radius:50%;
            background: ${cfg.color};
            border: 2px solid #fff;
            box-shadow: 0 0 8px ${cfg.color};
            display:flex; align-items:center; justify-content:center;
            font-size:12px; color:#fff; font-weight:bold;
            cursor:pointer;
        " title="${cfg.title}">${cfg.label}</div>`,
        iconAnchor: [12, 12]
    });
}

// =============================================
// POI MANAGEMENT
// =============================================
// =============================================
// POI MANAGEMENT WITH PERSISTENCE
// =============================================
let pois = [];
const poiMapMarkers = []; // Reference to markers on the main map
const poiMissionMarkers = []; // Reference to markers on the mission map

function savePoisToStorage() {
    localStorage.setItem('cadi_pois', JSON.stringify(pois));
}

function loadPoisFromStorage() {
    const saved = localStorage.getItem('cadi_pois');
    if (saved) {
        try {
            const parsed = JSON.parse(saved);
            pois = parsed;
            // Add markers to maps for each loaded POI
            pois.forEach(poi => addPoiToMaps(poi));
        } catch (e) {
            console.error('Error loading POIs from storage:', e);
            pois = [];
        }
    }
}

function addPoiToMaps(poi) {
    const icon = buildPoiIcon(poi.icon, poi.customPath);
    const m1 = L.marker([poi.lat, poi.lon], { icon }).addTo(map);
    poiMapMarkers.push(m1);
    
    if (missionMap) {
        const m2 = L.marker([poi.lat, poi.lon], { icon }).addTo(missionMap);
        poiMissionMarkers.push(m2);
    }
}

function buildPoiIcon(iconKey, customPath = null) {
    if (iconKey === 'custom' && customPath) {
        return L.divIcon({
            className: '',
            html: `<img src="${customPath}" style="width:32px; height:32px; filter: drop-shadow(0 0 5px rgba(255,255,255,0.5));" onerror="this.src='assets/icon.png'">`,
            iconAnchor: [16, 16]
        });
    }

    const icons = {
        'default': { color: '#ffffff', label: '📌' },
        'warning': { color: '#ff4d4d', label: '⚠️' },
        'target' : { color: '#00e5ff', label: '🎯' },
        'home'   : { color: '#00e676', label: '🏠' },
    };
    const cfg = icons[iconKey] || icons['default'];
    return L.divIcon({
        className: '',
        html: `<div style="
            min-width:28px; height:28px; border-radius:50%;
            background: rgba(0,0,0,0.6);
            border: 2px solid ${cfg.color};
            box-shadow: 0 0 8px ${cfg.color};
            display:flex; align-items:center; justify-content:center;
            font-size:16px; cursor:pointer;
        ">${cfg.label}</div>`,
        iconAnchor: [14, 14]
    });
}

function renderPois() {
    const grid = document.getElementById('poi-grid');
    grid.innerHTML = '';

    if (pois.length === 0) {
        grid.innerHTML = `<div class="waypoint-item empty-state">No POIs added.</div>`;
        return;
    }

    pois.forEach((poi, i) => {
        const el = document.createElement('div');
        el.className = 'waypoint-item';
        el.style.display = 'flex';
        el.style.justifyContent = 'space-between';
        el.style.alignItems = 'center';
        
        const info = document.createElement('div');
        info.style.flex = '1';
        info.style.cursor = 'pointer';
        info.innerHTML = `
            <span class="wp-number">POI ${i + 1}</span>
            Lat: ${poi.lat.toFixed(6)} | Lon: ${poi.lon.toFixed(6)}<br>
            Type: ${poi.icon === 'custom' ? 'Custom' : poi.icon}
        `;
        info.addEventListener('click', () => {
            map.setView([poi.lat, poi.lon], 17);
            if (missionMap) missionMap.setView([poi.lat, poi.lon], 17);
        });

        const actions = document.createElement('div');
        actions.style.display = 'flex';
        actions.style.gap = '5px';

        const editBtn = document.createElement('button');
        editBtn.className = 'icon-btn';
        editBtn.style.padding = '2px';
        editBtn.innerHTML = '<span class="icon" style="font-size:16px;">edit</span>';
        editBtn.title = 'Edit POI';
        editBtn.addEventListener('click', (e) => {
            e.stopPropagation();
            editPoi(i);
        });

        const delBtn = document.createElement('button');
        delBtn.className = 'icon-btn';
        delBtn.style.padding = '2px';
        delBtn.style.color = 'var(--critical-color)';
        delBtn.innerHTML = '<span class="icon" style="font-size:16px;">delete</span>';
        delBtn.title = 'Delete POI';
        delBtn.addEventListener('click', (e) => {
            e.stopPropagation();
            deletePoi(i);
        });

        actions.appendChild(editBtn);
        actions.appendChild(delBtn);
        
        el.appendChild(info);
        el.appendChild(actions);
        grid.appendChild(el);
    });
}

function deletePoi(index) {
    if (confirm(t('deletePoi'))) {
        // Remove from maps
        if (poiMapMarkers[index]) map.removeLayer(poiMapMarkers[index]);
        if (poiMissionMarkers[index]) missionMap.removeLayer(poiMissionMarkers[index]);
        
        // Remove references
        poiMapMarkers.splice(index, 1);
        poiMissionMarkers.splice(index, 1);
        pois.splice(index, 1);
        
        savePoisToStorage();
        renderPois();
    }
}

function editPoi(index) {
    const poi = pois[index];
    document.getElementById('poi-lat').value = poi.lat;
    document.getElementById('poi-lon').value = poi.lon;
    document.getElementById('poi-icon-select').value = poi.icon;
    
    if (poi.icon === 'custom') {
        document.getElementById('poi-custom-icon').style.display = 'block';
        document.getElementById('poi-custom-icon').value = poi.customPath || '';
    } else {
        document.getElementById('poi-custom-icon').style.display = 'none';
    }
    
    // Switch to ADD tab if not already there (modal is open anyway)
    // We'll change the ADD button to "UPDATE"
    const addBtn = document.getElementById('add-poi-btn');
    addBtn.innerText = 'UPDATE';
    addBtn.dataset.editIndex = index;
}

// Show/hide custom icon input
const poiIconSelect = document.getElementById('poi-icon-select');
const poiCustomInput = document.getElementById('poi-custom-icon');
poiIconSelect.addEventListener('change', () => {
    poiCustomInput.style.display = (poiIconSelect.value === 'custom') ? 'block' : 'none';
});

document.getElementById('add-poi-btn').addEventListener('click', () => {
    const btn = document.getElementById('add-poi-btn');
    const lat = parseFloat(document.getElementById('poi-lat').value);
    const lon = parseFloat(document.getElementById('poi-lon').value);
    const iconType = poiIconSelect.value;
    const customPath = poiCustomInput.value.trim();

    if (isNaN(lat) || isNaN(lon)) {
        showToast(t('invalidCoords'), 'warning');
        return;
    }

    if (btn.innerText === 'UPDATE') {
        const index = parseInt(btn.dataset.editIndex);
        // Remove old markers
        if (poiMapMarkers[index]) map.removeLayer(poiMapMarkers[index]);
        if (poiMissionMarkers[index]) missionMap.removeLayer(poiMissionMarkers[index]);
        
        // Update POI data
        pois[index] = { lat, lon, icon: iconType, customPath };
        
        // Re-add to maps
        const icon = buildPoiIcon(iconType, customPath);
        poiMapMarkers[index] = L.marker([lat, lon], { icon }).addTo(map);
        if (missionMap) {
            poiMissionMarkers[index] = L.marker([lat, lon], { icon }).addTo(missionMap);
        }
        
        btn.innerText = 'ADD';
        delete btn.dataset.editIndex;
    } else {
        const poi = { lat, lon, icon: iconType, customPath };
        pois.push(poi);
        addPoiToMaps(poi);
    }

    savePoisToStorage();
    renderPois();
    
    // Clear inputs
    document.getElementById('poi-lat').value = '';
    document.getElementById('poi-lon').value = '';
    document.getElementById('poi-custom-icon').value = '';
});

// Drone Icon Refresher
function updateDroneMarkerStyles() {
    const styleType = document.querySelector('input[name="drone-style"]:checked').value;
    const color = document.getElementById('drone-color-picker').value;
    const imagePath = document.getElementById('drone-icon-path').value.trim() || 'assets/icon.png';

    let newIcon;
    if (styleType === 'glow') {
        newIcon = L.divIcon({
            className: '',
            html: `<div style="
                width:18px;height:18px;
                border-radius:50%;
                background:radial-gradient(circle, ${color} 0%, #4589f5 60%, rgba(69,137,245,0) 100%);
                box-shadow:0 0 12px ${color}, 0 0 24px rgba(0,229,255,0.5);
                border:2px solid #fff;
            "></div>`,
            iconAnchor: [9, 9]
        });
    } else {
        newIcon = L.divIcon({
            className: '',
            html: `<img src="${imagePath}" style="width:30px; height:30px; filter: drop-shadow(0 0 8px ${color});" onerror="this.src='assets/icon.png'">`,
            iconAnchor: [15, 15]
        });
    }

    if (droneMarker) droneMarker.setIcon(newIcon);
    if (missionDroneMarker) missionDroneMarker.setIcon(newIcon);
}

document.querySelectorAll('input[name="drone-style"]').forEach(radio => {
    radio.addEventListener('change', updateDroneMarkerStyles);
});
document.getElementById('drone-color-picker').addEventListener('input', updateDroneMarkerStyles);
document.getElementById('drone-icon-path').addEventListener('input', updateDroneMarkerStyles);

// Initial renders
renderWaypoints();
loadPoisFromStorage();
renderPois();
listPorts();
setupMapControls('map', 'follow-telemetry', 'center-telemetry');

// =============================================
// OFFLINE WATCHDOG
// Show red OFFLINE banner if no telemetry for >3s
// =============================================
let _offlineTimer = null;
let _isOnline = false;

let _offlineOverlayTimeout = null;
function setOnlineState(online) {
    if (online === _isOnline) return;
    _isOnline = online;

    const overlay = document.getElementById('offline-overlay');
    const headerMsg = document.getElementById('offline-header-msg');
    const pill = document.getElementById('drone-state-indicator');

    if (online) {
        // Switch to Online
        if (overlay) overlay.classList.add('hidden');
        if (headerMsg) headerMsg.textContent = '';
        if (pill) pill.style.color = '';
        if (_offlineOverlayTimeout) {
            clearTimeout(_offlineOverlayTimeout);
            _offlineOverlayTimeout = null;
        }
    } else {
        // Switch to Offline
        if (overlay) {
            overlay.classList.remove('hidden');
            // Hide after 5 seconds but keep header message
            _offlineOverlayTimeout = setTimeout(() => {
                overlay.classList.add('hidden');
                if (headerMsg) {
                    headerMsg.textContent = 'SIGNAL LOST - OFFLINE';
                }
            }, 5000);
        }
        if (pill) pill.style.color = '#ff4d4d';
    }
}

window.electronAPI.onTelemetryData(() => {
    // Reset watchdog timer on every packet
    clearTimeout(_offlineTimer);
    _offlineTimer = setTimeout(() => setOnlineState(false), 3000);
});

// Start offline immediately (will clear once first packet arrives)
setTimeout(() => setOnlineState(false), 3000);

window._telemetryUpdateHook = function(lat, lon) {
    window._lastDroneLat = lat;
    window._lastDroneLon = lon;
    clearTimeout(_offlineTimer);
    _offlineTimer = setTimeout(() => setOnlineState(false), 3000);
    setOnlineState(true);
};

// =============================================
// HUD OVERLAY LAYER
// =============================================
const hudCanvas = document.getElementById('hud-canvas');
let hudCtx = null;
if (hudCanvas) {
    hudCtx = hudCanvas.getContext('2d');
}

function resizeHudCanvas() {
    if (hudCanvas) {
        // Must match visual size
        hudCanvas.width = hudCanvas.clientWidth;
        hudCanvas.height = hudCanvas.clientHeight;
        if (!window._lastTelemetryForHud) {
            drawHUD(0, 0, 0, 0, 0); // initial draw
        } else {
            const t = window._lastTelemetryForHud;
            drawHUD(t.pitch, t.roll, t.bearing, t.alt, t.speed);
        }
    }
}
window.addEventListener('resize', resizeHudCanvas);

// =============================================
// 3D GEOSPATIAL MATH & PROJECTION
// =============================================
function toRad(deg) { return deg * Math.PI / 180; }
function toDeg(rad) { return rad * 180 / Math.PI; }

function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = 6371e3; // meters
    const f1 = toRad(lat1);
    const f2 = toRad(lat2);
    const df = toRad(lat2 - lat1);
    const dl = toRad(lon2 - lon1);
    const a = Math.sin(df/2) * Math.sin(df/2) +
              Math.cos(f1) * Math.cos(f2) *
              Math.sin(dl/2) * Math.sin(dl/2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    return R * c;
}

function calculateBearing(lat1, lon1, lat2, lon2) {
    const f1 = toRad(lat1);
    const f2 = toRad(lat2);
    const dl = toRad(lon2 - lon1);
    const y = Math.sin(dl) * Math.cos(f2);
    const x = Math.cos(f1) * Math.sin(f2) -
              Math.sin(f1) * Math.cos(f2) * Math.cos(dl);
    const h = Math.atan2(y, x);
    return (toDeg(h) + 360) % 360;
}

function project3DTo2D(wpLat, wpLon, wpAlt, dLat, dLon, dAlt, pitch, roll, yaw, cx, cy) {
    // 1. Calculate relative physical position
    const dist = calculateDistance(dLat, dLon, wpLat, wpLon);
    if (dist < 1) return null; // Too close

    const targetBearing = calculateBearing(dLat, dLon, wpLat, wpLon);
    const dAltM = wpAlt - dAlt;

    // 2. Relative angle from camera center (yaw diff)
    let headingDiff = targetBearing - yaw;
    if (headingDiff > 180) headingDiff -= 360;
    if (headingDiff < -180) headingDiff += 360;

    // Relative vertical angle (pitch diff)
    const elevationAngle = toDeg(Math.atan2(dAltM, dist));
    const vertDiff = elevationAngle - pitch;

    // FOV specs for Caddx Baby Ratel 2
    const H_FOV = 115; 
    const V_FOV = H_FOV * (cy / cx); // Approximated aspect ratio mapping
    
    // Pixels per degree
    const pxPerDegX = (cx * 2) / H_FOV;
    const pxPerDegY = (cy * 2) / V_FOV;

    // Un-rolled X, Y on screen
    const xUnrolled = cx + (headingDiff * pxPerDegX);
    const yUnrolled = cy - (vertDiff * pxPerDegY); // screen Y decreases going up

    // Apply Camera Roll
    const rollRad = toRad(roll);
    const dx = xUnrolled - cx;
    const dy = yUnrolled - cy;
    const finalX = cx + (dx * Math.cos(rollRad) + dy * Math.sin(rollRad));
    const finalY = cy + (-dx * Math.sin(rollRad) + dy * Math.cos(rollRad));

    return {
        x: finalX,
        y: finalY,
        dist: dist,
        inView: (Math.abs(headingDiff) < H_FOV/1.8),
        headingDiff: headingDiff
    };
}

function drawHUD(pitch, roll, bearing, alt, speed) {
    if (!hudCtx || !hudCanvas) return;
    
    // Save last telemetry to re-draw on resize
    window._lastTelemetryForHud = { pitch, roll, bearing, alt, speed };

    const w = hudCanvas.width;
    const h = hudCanvas.height;
    const cx = w / 2;
    const cy = h / 2;

    // Clear previous frame
    hudCtx.clearRect(0, 0, w, h);

    // Styling constants
    hudCtx.strokeStyle = 'rgba(0, 229, 255, 0.8)';
    hudCtx.fillStyle = 'rgba(0, 229, 255, 0.8)';
    hudCtx.lineWidth = 2;
    hudCtx.font = '14px Orbitron, sans-serif';
    hudCtx.textAlign = 'center';
    hudCtx.textBaseline = 'middle';

    // 1. Static Crosshairs / Center Marks
    hudCtx.beginPath();
    // Center dot
    hudCtx.arc(cx, cy, 3, 0, Math.PI * 2);
    // Left wing
    hudCtx.moveTo(cx - 50, cy);
    hudCtx.lineTo(cx - 20, cy);
    hudCtx.lineTo(cx - 20, cy + 10);
    // Right wing
    hudCtx.moveTo(cx + 50, cy);
    hudCtx.lineTo(cx + 20, cy);
    hudCtx.lineTo(cx + 20, cy + 10);
    // Top tick
    hudCtx.moveTo(cx, cy - 20);
    hudCtx.lineTo(cx, cy - 10);
    hudCtx.stroke();

    // 2. Artificial Horizon (Pitch/Roll)
    hudCtx.save();
    hudCtx.translate(cx, cy);
    // Rotate canvas by roll (negative because browser Y is down)
    hudCtx.rotate(-roll * Math.PI / 180);
    
    // Draw Pitch ladder
    // 1 degree of pitch = 3 pixels of displacement (was 5)
    const pitchScale = 3.5;
    const pitchOffset = pitch * pitchScale;
    hudCtx.translate(0, pitchOffset);

    // Draw lines for pitch increments
    for (let p = -45; p <= 45; p += 15) {
        if (p === 0) continue; // Skip 0 line, we have the crosshair
        
        const y = -p * pitchScale;
        const lineW = p > 0 ? 50 : 30; // positive pitch uses wider lines
        
        hudCtx.beginPath();
        if (p < 0) {
            hudCtx.setLineDash([5, 5]); // dashed for down pitch
        }
        
        hudCtx.moveTo(-lineW, y);
        hudCtx.lineTo(-lineW/3, y);
        hudCtx.moveTo(lineW/3, y);
        hudCtx.lineTo(lineW, y);
        
        // Draw tick ends
        hudCtx.moveTo(-lineW, y);
        hudCtx.lineTo(-lineW, y + (p > 0 ? 5 : -5));
        hudCtx.moveTo(lineW, y);
        hudCtx.lineTo(lineW, y + (p > 0 ? 5 : -5));
        
        hudCtx.stroke();
        hudCtx.setLineDash([]);
        
        // Pitch text
        hudCtx.textAlign = 'right';
        hudCtx.fillText(Math.abs(p), -lineW - 5, y);
        hudCtx.textAlign = 'left';
        hudCtx.fillText(Math.abs(p), lineW + 5, y);
    }
    
    // Draw horizon line
    hudCtx.beginPath();
    hudCtx.moveTo(-120, 0);
    hudCtx.lineTo(-60, 0);
    hudCtx.moveTo(60, 0);
    hudCtx.lineTo(120, 0);
    hudCtx.stroke();
    
    hudCtx.restore();

    // 3. Roll Indicator (Top Arc)
    hudCtx.save();
    hudCtx.translate(cx, cy);
    const rollRadius = Math.min(cx, cy) - 50;
    
    if (rollRadius > 0) {
        // Draw static roll ticks
        hudCtx.beginPath();
        for (let a = -60; a <= 60; a += 15) {
            const rad = (a - 90) * Math.PI / 180;
            const x1 = Math.cos(rad) * rollRadius;
            const y1 = Math.sin(rad) * rollRadius;
            const len = a % 30 === 0 ? 10 : 5;
            const x2 = Math.cos(rad) * (rollRadius + len);
            const y2 = Math.sin(rad) * (rollRadius + len);
            hudCtx.moveTo(x1, y1);
            hudCtx.lineTo(x2, y2);
        }
        hudCtx.stroke();
        
        // Draw moving roll pointer
        hudCtx.rotate(-roll * Math.PI / 180);
        hudCtx.beginPath();
        hudCtx.moveTo(0, -rollRadius + 5);
        hudCtx.lineTo(8, -rollRadius + 20);
        hudCtx.lineTo(-8, -rollRadius + 20);
        hudCtx.closePath();
        hudCtx.fill();
        hudCtx.strokeStyle = 'rgba(0,0,0,0.5)';
        hudCtx.stroke(); // small border for contrast
    }
    hudCtx.restore();
    
    // 4. Heading Tape (Top Center)
    const tapeY = 35;
    const tapeWidth = Math.min(250, w * 0.5);
    hudCtx.save();
    hudCtx.beginPath();
    hudCtx.rect(cx - tapeWidth/2, tapeY - 20, tapeWidth, 40);
    hudCtx.clip();
    
    // Degree spacing: say 5 pixels per degree
    const tapeScale = 4;
    hudCtx.textAlign = 'center';
    
    // We draw from bearing - 30 to bearing + 30
    const startHdg = Math.floor(bearing - 30);
    const endHdg = Math.ceil(bearing + 30);
    
    for (let h_val = startHdg; h_val <= endHdg; h_val++) {
        if (h_val % 5 !== 0) continue; // Only draw every 5 degrees
        
        let displayHdg = h_val;
        if (displayHdg < 0) displayHdg += 360;
        if (displayHdg >= 360) displayHdg -= 360;
        
        const dx = (h_val - bearing) * tapeScale;
        const tx = cx + dx;
        
        hudCtx.beginPath();
        if (h_val % 10 === 0) {
            hudCtx.moveTo(tx, tapeY);
            hudCtx.lineTo(tx, tapeY - 8);
            
            // Format 0 as N, 90 as E, 180 as S, 270 as W
            let label = displayHdg.toString().padStart(3, '0');
            if (displayHdg === 0 || displayHdg === 360) label = 'N';
            if (displayHdg === 90) label = 'E';
            if (displayHdg === 180) label = 'S';
            if (displayHdg === 270) label = 'W';
            
            hudCtx.fillText(label, tx, tapeY + 12);
        } else {
            hudCtx.moveTo(tx, tapeY);
            hudCtx.lineTo(tx, tapeY - 4);
        }
        hudCtx.stroke();
    }
    
    // Heading center pip
    hudCtx.beginPath();
    hudCtx.moveTo(cx, tapeY + 22);
    hudCtx.lineTo(cx - 5, tapeY + 30);
    hudCtx.lineTo(cx + 5, tapeY + 30);
    hudCtx.closePath();
    hudCtx.fill();
    hudCtx.restore();

    // 5. Left/Right Info (Speed and Alt)
    const paddingX = 40;
    const tapeH = Math.min(200, h * 0.6);
    const tapeStartY = cy - tapeH / 2;

    hudCtx.textAlign = 'right';
    hudCtx.font = '14px Orbitron, sans-serif';
    hudCtx.fillText(`SPD`, paddingX + 35, tapeStartY - 10);
    hudCtx.fillText(`${speed.toFixed(1)} km/h`, paddingX + 50, cy + tapeH/2 + 20);
    
    hudCtx.textAlign = 'left';
    hudCtx.fillText(`ALT`, w - paddingX - 35, tapeStartY - 10);
    hudCtx.fillText(`${alt.toFixed(1)} m`, w - paddingX - 50, cy + tapeH/2 + 20);

    // Speed / Alt simple tapes (vertical boxes)
    // Speed (Left)
    hudCtx.strokeRect(paddingX - 10, tapeStartY, 35, tapeH);
    hudCtx.beginPath();
    hudCtx.moveTo(paddingX + 25, cy);
    hudCtx.lineTo(paddingX + 35, cy - 5);
    hudCtx.lineTo(paddingX + 35, cy + 5);
    hudCtx.closePath();
    hudCtx.fill();
    hudCtx.textAlign = 'center';
    hudCtx.fillText(speed.toFixed(0), paddingX + 7, cy);

    // Alt (Right)
    hudCtx.strokeRect(w - paddingX - 25, tapeStartY, 35, tapeH);
    hudCtx.beginPath();
    hudCtx.moveTo(w - paddingX - 25, cy);
    hudCtx.lineTo(w - paddingX - 35, cy - 5);
    hudCtx.lineTo(w - paddingX - 35, cy + 5);
    hudCtx.closePath();
    hudCtx.fill();
    hudCtx.textAlign = 'center';
    hudCtx.fillText(alt.toFixed(0), w - paddingX - 7, cy);

    // 6. 3D Augmented Reality Waypoints
    // We need current lat/lon from the persistent window vars
    if (window._lastDroneLat && window._lastDroneLon) {
        hudCtx.font = '12px Orbitron, sans-serif';
        hudCtx.lineWidth = 1.5;
        
        // Loop through MISSION waypoints
        for (let i = 0; i < waypoints.length; i++) {
            const wp = waypoints[i];
            const proj = project3DTo2D(
                wp.lat, wp.lon, wp.alt, 
                window._lastDroneLat, window._lastDroneLon, alt, 
                pitch, roll, bearing, cx, cy
            );
            if (!proj) continue;

            const isCurrentWP = (i === activeWaypointIdx);
            hudCtx.strokeStyle = isCurrentWP ? 'rgba(0, 255, 100, 0.9)' : 'rgba(255, 165, 0, 0.7)';
            hudCtx.fillStyle = hudCtx.strokeStyle;

            if (proj.inView) {
                // Determine size based on distance (clamp bounds)
                let size = Math.max(10, Math.min(30, 1000 / Math.max(proj.dist, 10)));
                
                // Draw Diamond
                hudCtx.beginPath();
                hudCtx.moveTo(proj.x, proj.y - size);
                hudCtx.lineTo(proj.x + size, proj.y);
                hudCtx.lineTo(proj.x, proj.y + size);
                hudCtx.lineTo(proj.x - size, proj.y);
                hudCtx.closePath();
                hudCtx.stroke();
                
                // Active WP gets an inner dot
                if (isCurrentWP) {
                    hudCtx.beginPath();
                    hudCtx.arc(proj.x, proj.y, 2, 0, Math.PI*2);
                    hudCtx.fill();
                }

                // Draw Text [WPx dist]
                hudCtx.textAlign = 'center';
                let dText = proj.dist > 1000 ? (proj.dist/1000).toFixed(1) + 'k' : proj.dist.toFixed(0) + 'm';
                hudCtx.fillText(`WP${i+1} ${dText}`, proj.x, proj.y - size - 8);
            } else {
                // Off-screen pointer logic
                if (isCurrentWP) {
                    // Draw chevron on the edge of the screen
                    const pointerOffset = 60;
                    hudCtx.beginPath();
                    if (proj.headingDiff > 0) {
                        // WP is to the right
                        hudCtx.moveTo(w - pointerOffset, cy - 15);
                        hudCtx.lineTo(w - pointerOffset + 15, cy);
                        hudCtx.lineTo(w - pointerOffset, cy + 15);
                    } else {
                        // WP is to the left
                        hudCtx.moveTo(pointerOffset, cy - 15);
                        hudCtx.lineTo(pointerOffset - 15, cy);
                        hudCtx.lineTo(pointerOffset, cy + 15);
                    }
                    hudCtx.stroke();
                    hudCtx.textAlign = proj.headingDiff > 0 ? 'right' : 'left';
                    let dText = proj.dist > 1000 ? (proj.dist/1000).toFixed(1) + 'k' : proj.dist.toFixed(0) + 'm';
                    hudCtx.fillText(`WP${i+1}`, proj.headingDiff > 0 ? w - pointerOffset - 10 : pointerOffset + 10, cy - 5);
                    hudCtx.fillText(dText, proj.headingDiff > 0 ? w - pointerOffset - 10 : pointerOffset + 10, cy + 10);
                }
            }
        }
    }
}

// Ensure the canvas is sized correctly later
setTimeout(resizeHudCanvas, 1000);

// =============================================
// MAP LAYER SWITCHING
// =============================================
document.getElementById('telemetry-layer-switcher')?.querySelectorAll('.map-layer-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        switchMapLayer(map, btn.dataset.layer, document.getElementById('telemetry-layer-switcher'));
    });
});

// Mission map layer switching (deferred until map is initialized)
function setupMissionLayerSwitcher() {
    document.getElementById('mission-layer-switcher')?.querySelectorAll('.map-layer-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            if (missionMap) switchMapLayer(missionMap, btn.dataset.layer, document.getElementById('mission-layer-switcher'));
        });
    });
}
// Called after mission map init in initMissionMap - hook it in
const _origInitMissionMap = typeof initMissionMap === 'function' ? initMissionMap : null;

// =============================================
// MISSION IMPORT / EXPORT / CLEAR
// =============================================
document.getElementById('export-mission-btn')?.addEventListener('click', async () => {
    if (waypoints.length === 0) {
        showToast(t('noWaypointsExport'), 'warning');
        return;
    }
    const result = await window.electronAPI.exportMission(waypoints);
    if (result.success) showToast(t('missionExported'), 'success');
});

document.getElementById('import-mission-btn')?.addEventListener('click', async () => {
    const result = await window.electronAPI.importMission();
    if (result.success && result.data) {
        // Clear existing
        missionWpMarkers.forEach(m => { if (missionMap) missionMap.removeLayer(m); });
        missionWpMarkers.length = 0;
        waypoints.length = 0;

        // Load imported
        result.data.forEach(wp => {
            waypoints.push(wp);
            if (missionMap) {
                const marker = L.marker([wp.lat, wp.lon], { icon: buildWpIcon(wp.mode) })
                    .bindTooltip(`WP ${waypoints.length} – ${modeNames[wp.mode]}`, { permanent: false, direction: 'top' })
                    .addTo(missionMap);
                missionWpMarkers.push(marker);
            }
        });
        activeWaypointIdx = 0;
        renderWaypoints();
        showToast(`${t('missionImported')} (${result.data.length} WPs)`, 'success');
    }
});

document.getElementById('clear-mission-btn')?.addEventListener('click', () => {
    if (waypoints.length === 0) return;
    if (!confirm(t('confirmClear'))) return;
    missionWpMarkers.forEach(m => { if (missionMap) missionMap.removeLayer(m); });
    missionWpMarkers.length = 0;
    waypoints.length = 0;
    activeWaypointIdx = 0;
    // Remove trajectory preview
    if (window._trajectoryLine) {
        if (missionMap) missionMap.removeLayer(window._trajectoryLine);
        window._trajectoryLine = null;
    }
    renderWaypoints();
    showToast(t('missionCleared'), 'info');
});

// =============================================
// TRAJECTORY PREVIEW
// =============================================
let _trajectoryLine = null;
document.getElementById('sim-trajectory-btn')?.addEventListener('click', () => {
    if (!missionMap) return;
    const btn = document.getElementById('sim-trajectory-btn');

    if (_trajectoryLine) {
        // Toggle off
        missionMap.removeLayer(_trajectoryLine);
        _trajectoryLine = null;
        btn.classList.remove('trajectory-active');
        return;
    }

    if (waypoints.length < 1) {
        showToast(t('noWaypointsExport'), 'warning');
        return;
    }

    // Build trajectory path: drone current -> wp1 -> wp2 -> ...
    const points = [];
    if (window._lastDroneLat && window._lastDroneLon) {
        points.push([window._lastDroneLat, window._lastDroneLon]);
    }
    waypoints.forEach(wp => points.push([wp.lat, wp.lon]));

    _trajectoryLine = L.polyline(points, {
        color: '#00e676',
        weight: 3,
        opacity: 0.8,
        dashArray: '10 6',
    }).addTo(missionMap);

    btn.classList.add('trajectory-active');

    // Add distance labels
    for (let i = 1; i < points.length; i++) {
        const dist = calculateDistance(points[i-1][0], points[i-1][1], points[i][0], points[i][1]);
        const midLat = (points[i-1][0] + points[i][0]) / 2;
        const midLon = (points[i-1][1] + points[i][1]) / 2;
        const dText = dist > 1000 ? (dist/1000).toFixed(1) + ' km' : dist.toFixed(0) + ' m';
        L.tooltip({ permanent: true, direction: 'center', className: '' })
            .setLatLng([midLat, midLon])
            .setContent(`<span style="color:#00e676; font-size:11px; text-shadow:0 0 3px #000;">${dText}</span>`)
            .addTo(missionMap);
    }

    showToast(`Trajectory preview: ${points.length} points`, 'info');
    window._trajectoryLine = _trajectoryLine;
});

// =============================================
// HEALTH DASHBOARD POLLING
// =============================================
setInterval(async () => {
    if (window.electronAPI.getPacketStats) {
        try {
            const stats = await window.electronAPI.getPacketStats();
            if (stats) {
                document.getElementById('health-packets').textContent = stats.received || 0;
                document.getElementById('health-crc-errors').textContent = stats.crcErrors || 0;
            }
        } catch (e) { /* ignore */ }
    }
}, 5000);

// =============================================
// INITIAL SETUP
// =============================================
loadPreferences();
setupMissionLayerSwitcher();

// Restore saved map layer
if (savedLayer && tileLayers[savedLayer]) {
    switchMapLayer(map, savedLayer, document.getElementById('telemetry-layer-switcher'));
}

// =============================================
// GLOBAL SHORTCUTS
// =============================================
window.addEventListener('keydown', (e) => {
    if (e.key === 'F11') {
        e.preventDefault();
        if (window.electronAPI && window.electronAPI.toggleFullscreen) {
            window.electronAPI.toggleFullscreen();
        }
    }
});
