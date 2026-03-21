const { app, BrowserWindow, ipcMain } = require("electron");
const path = require("path");
const { SerialPort } = require("serialport");
const { dialog } = require("electron");
const fs = require("fs");

// =============================================
// STRUCTURED LOGGING SYSTEM
// =============================================
const LOG_FILE = path.join(app.getPath("userData"), "xdf001_log.txt");
const MAX_LOG_LINES = 5000;
let _mainWindow = null;

function logMessage(level, message, details = null) {
  const ts = new Date().toISOString();
  const entry = `[${ts}] [${level}] ${message}${details ? ' | ' + JSON.stringify(details) : ''}`;
  console.log(entry);
  try {
    fs.appendFileSync(LOG_FILE, entry + '\n');
    // Periodically check size instead of reading every line count
    if (Math.random() < 0.05) { // 5% chance to check for cleanup to save I/O
      const stats = fs.statSync(LOG_FILE);
      if (stats.size > 1024 * 1024) { // > 1MB
        const data = fs.readFileSync(LOG_FILE, 'utf8').split('\n');
        if (data.length > MAX_LOG_LINES) {
          fs.writeFileSync(LOG_FILE, data.slice(-MAX_LOG_LINES).join('\n'));
        }
      }
    }
  } catch (e) { /* ignore log errors */ }
  
  // Forward to renderer for toast notifications with safety check
  if (_mainWindow && !_mainWindow.isDestroyed() && _mainWindow.webContents) {
    _mainWindow.webContents.send('log-message', { level, message, details, timestamp: ts });
  }
}

// =============================================
// TELEMETRY VALIDATION
// =============================================
function validateTelemetry(data) {
  const clamp = (v, min, max) => Math.max(min, Math.min(max, v));
  const isFiniteNum = (v) => typeof v === 'number' && isFinite(v);

  if (!data || typeof data !== 'object') return null;

  const validated = { ...data };

  // Validate and clamp ranges
  if (!isFiniteNum(validated.latitude) || validated.latitude < -90 || validated.latitude > 90) {
    logMessage('WARN', 'Invalid latitude', { value: validated.latitude });
    validated.latitude = clamp(validated.latitude || 0, -90, 90);
  }
  if (!isFiniteNum(validated.longitude) || validated.longitude < -180 || validated.longitude > 180) {
    logMessage('WARN', 'Invalid longitude', { value: validated.longitude });
    validated.longitude = clamp(validated.longitude || 0, -180, 180);
  }
  if (!isFiniteNum(validated.altitude)) {
    validated.altitude = 0;
  } else {
    validated.altitude = clamp(validated.altitude, -500, 50000);
  }
  if (!isFiniteNum(validated.yaw)) validated.yaw = 0;
  if (!isFiniteNum(validated.pitch)) validated.pitch = 0;
  if (!isFiniteNum(validated.gforce)) {
    validated.gforce = 1.0;
  } else {
    validated.gforce = clamp(validated.gforce, -20, 20);
  }
  if (!isFiniteNum(validated.velocity_mag)) {
    validated.velocity_mag = 0;
  } else {
    validated.velocity_mag = clamp(validated.velocity_mag, 0, 500);
  }
  if (!isFiniteNum(validated.cmd_throttle)) validated.cmd_throttle = 0;

  // Anomaly flags
  validated._anomalies = [];
  if (validated.gforce > 3.0) validated._anomalies.push('HIGH_GFORCE');
  if (validated.gforce < 0) validated._anomalies.push('NEGATIVE_GFORCE');
  if (validated.altitude < 0) validated._anomalies.push('NEGATIVE_ALT');
  if (validated.velocity_mag * 3.6 > 200) validated._anomalies.push('HIGH_SPEED');
  if (validated.altitude > 5000) validated._anomalies.push('EXTREME_ALT');

  return validated;
}

// =============================================
// WINDOW CREATION
// =============================================
function createWindow() {
  const win = new BrowserWindow({
    width: 1920,
    height: 1080,
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
      contextIsolation: true,
      nodeIntegration: false,
    },
    backgroundColor: "#000000",
    icon: path.join(__dirname, "assets/icon.ico"),
    fullscreen: true,
  });

  ipcMain.on("toggle-fullscreen", () => {
    const isFullScreen = win.isFullScreen();
    win.setFullScreen(!isFullScreen);
  });

  _mainWindow = win;
  win.setMenu(null); 
  win.loadFile("index.html");
  // win.webContents.openDevTools(); // Uncomment for debugging

  logMessage('INFO', 'Application window created');

  // For Demo: Start simulator automatically
  startSimulator(win.webContents);
}

app.whenReady().then(() => {
  createWindow();

  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createWindow();
    }
  });
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") {
    app.quit();
  }
});

function calculateCRC16(buffer, start, length) {
  let crc = 0xFFFF;
  for (let i = 0; i < length; i++) {
    crc ^= (buffer[start + i] << 8);
    for (let j = 0; j < 8; j++) {
      if ((crc & 0x8000) > 0) {
        crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
      } else {
        crc = (crc << 1) & 0xFFFF;
      }
    }
  }
  return crc;
}

// =============================================
// SERIAL PORT WITH AUTO-RETRY
// =============================================
let port;
let serialBuffer = Buffer.alloc(0);
let _serialConfig = { path: null, baudRate: 115200 };
let _retryCount = 0;
const MAX_RETRIES = 5;
let _retryTimer = null;
let _packetStats = { received: 0, valid: 0, crcErrors: 0, validationErrors: 0 };

function resetPacketStats() {
  _packetStats = { received: 0, valid: 0, crcErrors: 0, validationErrors: 0 };
}

function notifySerialStatus(status, details = {}) {
  if (_mainWindow && !_mainWindow.isDestroyed() && _mainWindow.webContents) {
    _mainWindow.webContents.send('serial-status', { status, ...details });
  }
}

function attemptReconnect() {
  if (_retryCount >= MAX_RETRIES) {
    logMessage('ERROR', `Serial reconnection failed after ${MAX_RETRIES} attempts`);
    notifySerialStatus('failed', { retries: _retryCount });
    _retryCount = 0;
    return;
  }

  _retryCount++;
  const delay = Math.min(1000 * Math.pow(2, _retryCount - 1), 10000); // Exponential backoff max 10s
  logMessage('INFO', `Attempting serial reconnect in ${delay}ms (attempt ${_retryCount}/${MAX_RETRIES})`);
  notifySerialStatus('reconnecting', { attempt: _retryCount, maxRetries: MAX_RETRIES, delayMs: delay });

  _retryTimer = setTimeout(async () => {
    try {
      await connectSerialInternal(_serialConfig.path, _serialConfig.baudRate);
      _retryCount = 0;
      logMessage('INFO', 'Serial reconnection successful');
      notifySerialStatus('connected', { port: _serialConfig.path });
    } catch (e) {
      logMessage('WARN', 'Reconnection attempt failed', { error: e.message });
      attemptReconnect();
    }
  }, delay);
}

async function connectSerialInternal(portPath, baudRate) {
  if (port && port.isOpen) {
    await new Promise((resolve) => port.close(resolve));
  }

  serialBuffer = Buffer.alloc(0);
  resetPacketStats();
  const rate = parseInt(baudRate) || 115200;
  port = new SerialPort({ path: portPath, baudRate: rate });
  _serialConfig = { path: portPath, baudRate: rate };
  logMessage('INFO', `Serial Port connected: ${portPath} @ ${rate}bps`);

  port.on("data", (chunk) => {
    serialBuffer = Buffer.concat([serialBuffer, chunk]);

    // Max buffer protection (2KB)
    if (serialBuffer.length > 2048) {
      serialBuffer = Buffer.alloc(0);
      logMessage('WARN', 'Serial buffer overflow, cleared');
    }

    // Process all complete packets in buffer
    while (serialBuffer.length >= 4) {
      const startIdx = serialBuffer.indexOf(0xAA);
      if (startIdx === -1) {
        serialBuffer = Buffer.alloc(0);
        break;
      } else if (startIdx > 0) {
        serialBuffer = serialBuffer.subarray(startIdx);
      }

      if (serialBuffer.length < 4) break;

      const type = serialBuffer[1];
      const len = serialBuffer[2];
      const expectedSize = 1 + 1 + 1 + len + 2 + 1;

      if (serialBuffer.length < expectedSize) break;

      if (serialBuffer[expectedSize - 1] !== 0x55) {
        serialBuffer = serialBuffer.subarray(1);
        continue;
      }

      _packetStats.received++;

      const crcHigh = serialBuffer[expectedSize - 3];
      const crcLow = serialBuffer[expectedSize - 2];
      const receivedCRC = (crcHigh << 8) | crcLow;
      const calculatedCRC = calculateCRC16(serialBuffer, 1, 1 + 1 + len);

      if (receivedCRC === calculatedCRC) {
        if (type === 0x02 && len === 50) {
          const offset = 3;
          const telemetryData = {
            latitude: serialBuffer.readFloatLE(offset),
            longitude: serialBuffer.readFloatLE(offset + 4),
            altitude: serialBuffer.readFloatLE(offset + 8),
            yaw: serialBuffer.readFloatLE(offset + 12),
            pitch: serialBuffer.readFloatLE(offset + 16),
            gforce: serialBuffer.readFloatLE(offset + 20),
            velocity_mag: serialBuffer.readFloatLE(offset + 24),
            pos_local_x: serialBuffer.readFloatLE(offset + 28),
            pos_local_y: serialBuffer.readFloatLE(offset + 32),
            pos_local_z: serialBuffer.readFloatLE(offset + 36),
            currentMode: serialBuffer.readUInt8(offset + 40),
            cmd_yaw: serialBuffer.readInt16LE(offset + 41),
            cmd_throttle: serialBuffer.readInt16LE(offset + 43),
            cmd_pitch: serialBuffer.readInt16LE(offset + 45),
            cmd_roll: serialBuffer.readInt16LE(offset + 47),
            lossRate: serialBuffer.readUInt8(offset + 49)
          };

          // Validate before sending to renderer
          const validated = validateTelemetry(telemetryData);
          if (validated) {
            _packetStats.valid++;
            if (_mainWindow && !_mainWindow.isDestroyed() && _mainWindow.webContents) {
              _mainWindow.webContents.send("telemetry-data", validated);
            }
          } else {
            _packetStats.validationErrors++;
          }
        }
        serialBuffer = serialBuffer.subarray(expectedSize);
      } else {
        _packetStats.crcErrors++;
        logMessage('WARN', `CRC Mismatch: Calc ${calculatedCRC.toString(16)}, Recv ${receivedCRC.toString(16)}`);
        serialBuffer = serialBuffer.subarray(expectedSize);
      }
    }
  });

  // Graceful disconnection handling
  port.on('close', () => {
    logMessage('WARN', 'Serial port closed');
    notifySerialStatus('disconnected');
    attemptReconnect();
  });

  port.on('error', (err) => {
    logMessage('ERROR', 'Serial port error', { error: err.message });
    notifySerialStatus('error', { error: err.message });
  });

  return { success: true };
}

ipcMain.handle("connect-serial", async (event, portPath, baudRate) => {
  try {
    if (_retryTimer) { clearTimeout(_retryTimer); _retryTimer = null; }
    _retryCount = 0;
    const result = await connectSerialInternal(portPath, baudRate);
    notifySerialStatus('connected', { port: portPath });
    return result;
  } catch (error) {
    logMessage('ERROR', 'Serial connection failed', { error: error.message });
    return { success: false, error: error.message };
  }
});

ipcMain.handle("list-ports", async () => {
  try {
    const ports = await SerialPort.list();
    return ports;
  } catch (error) {
    logMessage('ERROR', 'Failed to list ports', { error: error.message });
    return [];
  }
});

ipcMain.handle("select-save-path", async () => {
  const { filePath } = await dialog.showSaveDialog({
    title: "Export Telemetry Data",
    defaultPath: path.join(app.getPath("documents"), "telemetry_data.csv"),
    filters: [{ name: "CSV Files", extensions: ["csv"] }],
  });
  return filePath;
});

ipcMain.handle("save-csv-file", async (event, filePath, content) => {
  try {
    fs.writeFileSync(filePath, content, "utf8");
    logMessage('INFO', 'CSV file exported', { path: filePath });
    return { success: true };
  } catch (error) {
    logMessage('ERROR', 'CSV export failed', { error: error.message });
    return { success: false, error: error.message };
  }
});

// =============================================
// MISSION FILE IMPORT/EXPORT
// =============================================
ipcMain.handle("export-mission", async (event, missionData) => {
  try {
    const { filePath } = await dialog.showSaveDialog({
      title: "Export Mission Plan",
      defaultPath: path.join(app.getPath("documents"), "mission.json"),
      filters: [{ name: "Mission Files", extensions: ["json"] }],
    });
    if (!filePath) return { success: false, error: 'Cancelled' };
    fs.writeFileSync(filePath, JSON.stringify(missionData, null, 2), "utf8");
    logMessage('INFO', 'Mission exported', { path: filePath, waypoints: missionData.length });
    return { success: true, path: filePath };
  } catch (error) {
    logMessage('ERROR', 'Mission export failed', { error: error.message });
    return { success: false, error: error.message };
  }
});

ipcMain.handle("import-mission", async () => {
  try {
    const { filePaths } = await dialog.showOpenDialog({
      title: "Import Mission Plan",
      filters: [{ name: "Mission Files", extensions: ["json"] }],
      properties: ["openFile"],
    });
    if (!filePaths || filePaths.length === 0) return { success: false, error: 'Cancelled' };
    const content = fs.readFileSync(filePaths[0], "utf8");
    const data = JSON.parse(content);
    if (!Array.isArray(data)) throw new Error('Invalid mission file format');
    logMessage('INFO', 'Mission imported', { path: filePaths[0], waypoints: data.length });
    return { success: true, data };
  } catch (error) {
    logMessage('ERROR', 'Mission import failed', { error: error.message });
    return { success: false, error: error.message };
  }
});

// =============================================
// PACKET STATS
// =============================================
ipcMain.handle("get-packet-stats", async () => {
  return _packetStats;
});

// ═══════════════════════════════════════════════════════
// TX: send-command  (PC → CADI_G via serial)
// ═══════════════════════════════════════════════════════
const PKT_CONFIG  = 0x01;
const MAGIC_START = 0xAA;
const MAGIC_END   = 0x55;

ipcMain.handle("send-command", async (event, cfg) => {
  if (!port || !port.isOpen) {
    return { success: false, error: "Serial port not open" };
  }

  try {
    // Validate command inputs
    if (typeof cfg.lat !== 'number' || typeof cfg.lon !== 'number') {
      return { success: false, error: "Invalid coordinates" };
    }

    const payload = Buffer.alloc(23);
    payload.writeUInt8(cfg.masterMode,  0);
    payload.writeUInt8(cfg.order,       1);
    payload.writeFloatLE(cfg.lat,       2);
    payload.writeFloatLE(cfg.lon,       6);
    payload.writeFloatLE(cfg.alt,      10);
    payload.writeUInt8(cfg.direction,  14);
    payload.writeFloatLE(cfg.radius,   15);
    payload.writeFloatLE(cfg.declination || -6.0, 19);

    const len = payload.length;
    const packet = Buffer.alloc(1 + 1 + 1 + len + 2 + 1);
    let idx = 0;
    packet[idx++] = MAGIC_START;
    packet[idx++] = PKT_CONFIG;
    packet[idx++] = len;
    payload.copy(packet, idx); idx += len;

    const crc = calculateCRC16(packet, 1, 1 + 1 + len);
    packet[idx++] = (crc >> 8) & 0xFF;
    packet[idx++] =  crc       & 0xFF;
    packet[idx++] = MAGIC_END;

    await new Promise((resolve, reject) => {
      port.write(packet, (err) => { err ? reject(err) : resolve(); });
    });

    logMessage('INFO', 'Command sent', { mode: cfg.masterMode, order: cfg.order });
    return { success: true };
  } catch (error) {
    logMessage('ERROR', 'Send command failed', { error: error.message });
    return { success: false, error: error.message };
  }
});

// ═══════════════════════════════════════════════════════
// TX: upload-mission  (PC → CADI_G via serial) [F2]
// ═══════════════════════════════════════════════════════
const PKT_ROUTE = 0x06;

ipcMain.handle("upload-mission", async (event, waypoints) => {
  if (!port || !port.isOpen) {
    return { success: false, error: "Serial port not open" };
  }

  try {
    const count = waypoints.length;
    if (count === 0) return { success: false, error: "No waypoints to upload" };
    if (count > 16) return { success: false, error: "Max 16 waypoints allowed" };

    const wpSize = 18; // lat(4), lon(4), alt(4), mode(1), dir(1), radius(4)
    const payloadLen = 1 + (count * wpSize);
    const payload = Buffer.alloc(payloadLen);
    
    payload.writeUInt8(count, 0);
    
    for (let i = 0; i < count; i++) {
        const wp = waypoints[i];
        const base = 1 + (i * wpSize);
        payload.writeFloatLE(wp.lat,        base);
        payload.writeFloatLE(wp.lon,        base + 4);
        payload.writeFloatLE(wp.alt,        base + 8);
        payload.writeUInt8(wp.mode || 1,    base + 12);
        payload.writeUInt8(wp.direction || 0, base + 13);
        payload.writeFloatLE(wp.radius || 30.0, base + 14);
    }

    const packetLen = 1 + 1 + 1 + payloadLen + 2 + 1;
    const packet = Buffer.alloc(packetLen);
    let idx = 0;
    packet[idx++] = MAGIC_START;
    packet[idx++] = PKT_ROUTE;
    packet[idx++] = payloadLen;
    payload.copy(packet, idx); idx += payloadLen;

    const crc = calculateCRC16(packet, 1, 1 + 1 + payloadLen);
    packet[idx++] = (crc >> 8) & 0xFF;
    packet[idx++] =  crc       & 0xFF;
    packet[idx++] = MAGIC_END;

    await new Promise((resolve, reject) => {
      port.write(packet, (err) => { err ? reject(err) : resolve(); });
    });

    logMessage('INFO', 'Mission uploaded', { waypoints: count });
    return { success: true };
  } catch (error) {
    logMessage('ERROR', 'Upload mission failed', { error: error.message });
    return { success: false, error: error.message };
  }
});

// =============================================
// SIMULATOR
// =============================================
let _simLat = -33.456;
let _simLon = -70.648;
function startSimulator(webContents) {
  setInterval(() => {
    // Only simulate if real serial is not sending data
    if (port && port.isOpen) return;

    _simLat += (Math.random() - 0.5) * 0.0004;
    _simLon += (Math.random() - 0.5) * 0.0004;
    
    const rawSimData = {
        latitude:     _simLat,
        longitude:    _simLon,
        altitude:     120 + (Math.random() - 0.5) * 10,
        currentMode:  1,
        gforce:       1.0 + (Math.random() - 0.5) * 0.3,
        velocity_mag: 22 + (Math.random() - 0.5) * 5,
        pitch:        -8 + (Math.random() - 0.5) * 4,
        yaw:          45 + (Math.random() - 0.5) * 10,
        pos_local_x: 0, pos_local_y: 0, pos_local_z: 0,
        cmd_yaw: 0, cmd_throttle: 90, cmd_pitch: -8, cmd_roll: 0
    };

    const validated = validateTelemetry(rawSimData);
    if (validated && webContents && !webContents.isDestroyed()) {
      webContents.send("telemetry-data", validated);
    }
  }, 1000);
}
