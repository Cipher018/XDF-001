// Session tracking
const appStartTime = Date.now();
const telemetryLog = []; // Stores all received data points

function formatElapsed(ms) {
    const totalSeconds = Math.floor(ms / 1000);
    const minutes = Math.floor(totalSeconds / 60);
    const seconds = totalSeconds % 60;
    return `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
}

// Leaflet Map Initialization
const map = L.map('map', {
    zoomControl: false,
    attributionControl: false
}).setView([0, 0], 2);

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
}).addTo(map);

const droneMarker = L.marker([0, 0]).addTo(map);

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
                label: 'Temperature',
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
        const isHidden = telemetryChart.data.datasets[datasetIndex].hidden;
        
        // Toggle state
        telemetryChart.data.datasets[datasetIndex].hidden = !isHidden;
        btn.classList.toggle('active', !telemetryChart.data.datasets[datasetIndex].hidden);
        
        telemetryChart.update();
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
        const result = await window.electronAPI.connectSerial(e.target.value);
        if (result.success) {
            console.log('Connected to serial port:', e.target.value);
        } else {
            alert('Failed to connect: ' + result.error);
        }
    }
});

listPorts();

// Telemetry Handling
window.electronAPI.onTelemetryData((data) => {
    // Expected Data Format: [Latitude, Longitude, Altitude, State, Temperature, Speed, Pitch, Roll, Yaw, Bearing]
    
    if (!Array.isArray(data) || data.length < 10) return;

    const [lat, lon, alt, state, temp, speed, pitch, roll, yaw, bearing] = data;

    // Update Metrics
    document.querySelector('#altitude .metric-value').innerHTML = `${alt} <span class="unit">m</span>`;
    document.querySelector('#speed .metric-value').innerHTML = `${speed} <span class="unit">Km/h</span>`;
    document.querySelector('#temperature .metric-value').innerHTML = `${temp} <span class="unit">° C</span>`;

    // Update Orientation
    document.getElementById('pitch').innerText = `${pitch}°`;
    document.getElementById('roll').innerText = `${roll}°`;
    document.getElementById('yaw').innerText = `${yaw}°`;
    document.getElementById('bearing').innerText = `${bearing}°`;

    // Update Map
    const coords = [lat, lon];
    droneMarker.setLatLng(coords);
    map.panTo(coords);
    document.getElementById('lat').innerText = `Lat: ${lat.toFixed(4)}`;
    document.getElementById('lon').innerText = `Lon: ${lon.toFixed(4)}`;

    // Update Chart
    const elapsedMs = Date.now() - appStartTime;
    const formattedTime = formatElapsed(elapsedMs);
    telemetryChart.data.labels.push(formattedTime);
    
    // Dataset 0: Altitude
    telemetryChart.data.datasets[0].data.push(alt);
    // Dataset 1: Speed
    telemetryChart.data.datasets[1].data.push(speed);
    // Dataset 2: Power (Simulated or from data if available - for now using some variation or placeholder)
    telemetryChart.data.datasets[2].data.push(Math.random() * 50); // Simulated Power
    // Dataset 3: Temperature
    telemetryChart.data.datasets[3].data.push(temp);

    // Update Log for CSV Export
    telemetryLog.push({
        elapsed: formattedTime,
        ms: elapsedMs,
        lat, lon, alt, state, temp, speed, pitch, roll, yaw, bearing
    });

    if (telemetryChart.data.labels.length > 20) {
        telemetryChart.data.labels.shift();
        telemetryChart.data.datasets.forEach(ds => ds.data.shift());
    }
    telemetryChart.update('none');
});

// Fake data simulation for testing UI (Remove or comment out when real data is coming)
setInterval(() => {
    const fakeData = [
        -33.456 + (Math.random() * 0.01), // Lat
        -70.648 + (Math.random() * 0.01), // Lon
        100 + Math.floor(Math.random() * 10), // Alt
        "OK", // Act
        20 + Math.floor(Math.random() * 5), // Temp
        20 + Math.floor(Math.random() * 5), // Vel
        -10 + Math.floor(Math.random() * 2), // Pitch
        20 + Math.floor(Math.random() * 2), // Roll
        0 + Math.floor(Math.random() * 2), // Yaw
        220 + Math.floor(Math.random() * 2) // Bearing
    ];
    window.dispatchEvent(new CustomEvent('fake-telemetry', { detail: fakeData }));
}, 1000);

window.addEventListener('fake-telemetry', (e) => {
    // This is just to test without actual serial bridge if needed
    // In a real app index.js (preload) would bridge this
});
// Export Modal Logic
const exportModal = document.getElementById('export-modal');
const openExportBtn = document.getElementById('open-export');
const closeExportBtn = document.getElementById('close-export');
const browsePathBtn = document.getElementById('browse-path');
const confirmExportBtn = document.getElementById('confirm-export');
const exportPathInput = document.getElementById('export-path');

openExportBtn.addEventListener('click', () => {
    exportModal.classList.add('active');
    // Set default end time to current elapsed
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
        alert('Please select a save path first.');
        return;
    }

    const selectedStartTime = document.getElementById('export-start').value;
    const selectedEndTime = document.getElementById('export-end').value;
    const selectedVars = Array.from(document.querySelectorAll('input[name="export-var"]:checked'))
                              .map(cb => cb.value);

    // Convert MM:SS to ms for easier comparison if needed, or just use string comparison for simplicity if they match format
    const filteredData = telemetryLog.filter(point => {
        return point.elapsed >= selectedStartTime && (selectedEndTime === "Now" || point.elapsed <= selectedEndTime);
    });

    if (filteredData.length === 0) {
        alert('No data found for the selected time range.');
        return;
    }

    // Generate CSV
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
        alert('Data exported successfully!');
        exportModal.classList.remove('active');
    } else {
        alert('Error exporting data: ' + result.error);
    }
});
