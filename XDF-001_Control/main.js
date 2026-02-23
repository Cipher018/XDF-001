const { app, BrowserWindow, ipcMain } = require("electron");
const path = require("path");
const { SerialPort } = require("serialport");
const { dialog } = require("electron");
const fs = require("fs");

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
    icon: path.join(__dirname, "assets/icon.ico"), // Fallback if no icon
  });

  win.loadFile("index.html");
  // win.webContents.openDevTools(); // Uncomment for debugging

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

// Serial Port Logic
let port;
let serialBuffer = Buffer.alloc(0);

ipcMain.handle("connect-serial", async (event, portPath, baudRate) => {
  try {
    if (port && port.isOpen) {
      await new Promise((resolve) => port.close(resolve));
    }

    serialBuffer = Buffer.alloc(0); // Reset buffer on new connection
    const rate = parseInt(baudRate) || 115200;
    port = new SerialPort({ path: portPath, baudRate: rate });
    console.log(`Serial Port connected: ${portPath} @ ${rate}bps`);

    port.on("data", (chunk) => {
      serialBuffer = Buffer.concat([serialBuffer, chunk]);

      // Max buffer protection
      if (serialBuffer.length > 2048) {
          serialBuffer = Buffer.alloc(0);
      }

      // Process all complete packets in buffer
      while (serialBuffer.length >= 4) {
        // Find MAGIC_START
        const startIdx = serialBuffer.indexOf(0xAA);
        if (startIdx === -1) {
          serialBuffer = Buffer.alloc(0); // Garbage, clear all
          break;
        } else if (startIdx > 0) {
          serialBuffer = serialBuffer.subarray(startIdx); // Shift buffer
        }

        if (serialBuffer.length < 4) break; // Need at least header

        const type = serialBuffer[1];
        const len = serialBuffer[2];
        const expectedSize = 1 + 1 + 1 + len + 2 + 1; // START + TYPE + LEN + PAYLOAD + CRC + END

        if (serialBuffer.length < expectedSize) break; // Wait for more data

        // Check MAGIC_END
        if (serialBuffer[expectedSize - 1] !== 0x55) {
            serialBuffer = serialBuffer.subarray(1); // Invalid end, skip the start byte and search again
            continue; 
        }

        // Verify CRC over TYPE + LENGTH + PAYLOAD
        const crcHigh = serialBuffer[expectedSize - 3];
        const crcLow = serialBuffer[expectedSize - 2];
        const receivedCRC = (crcHigh << 8) | crcLow;
        const calculatedCRC = calculateCRC16(serialBuffer, 1, 1 + 1 + len);

        if (receivedCRC === calculatedCRC) {
            // Valid Packet
            if (type === 0x02 && len === 49) {
                // Telemetry Packet
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
                    cmd_roll: serialBuffer.readInt16LE(offset + 47)
                };
                event.sender.send("telemetry-data", telemetryData);
            }
            // Remove processed packet
            serialBuffer = serialBuffer.subarray(expectedSize);
        } else {
            // CRC Mismatch
            console.warn(`CRC Mismatch: Calculated ${calculatedCRC.toString(16)}, Received ${receivedCRC.toString(16)}`);
            serialBuffer = serialBuffer.subarray(expectedSize);
        }
      }
    });

    return { success: true };
  } catch (error) {
    return { success: false, error: error.message };
  }
});

ipcMain.handle("list-ports", async () => {
  try {
    const ports = await SerialPort.list();
    return ports;
  } catch (error) {
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
    return { success: true };
  } catch (error) {
    return { success: false, error: error.message };
  }
});

// ═══════════════════════════════════════════════════════
// TX: send-command  (PC → CADI_G via serial)
// ConfigPacket layout (19 bytes, __packed__):
//   [0]    masterMode  uint8   0=Discretion 1=Manual 2=Autonomous
//   [1]    order       uint8   0=None 1=Waypoint 2=Orbit
//   [2-5]  lat         float32 LE
//   [6-9]  lon         float32 LE
//  [10-13] alt         float32 LE
//  [14]    direction   uint8   0=CCW 1=CW
//  [15-18] radius      float32 LE  (orbit radius, metres)
// ═══════════════════════════════════════════════════════
const PKT_CONFIG  = 0x01;
const MAGIC_START = 0xAA;
const MAGIC_END   = 0x55;

ipcMain.handle("send-command", async (event, cfg) => {
  if (!port || !port.isOpen) {
    return { success: false, error: "Serial port not open" };
  }

  try {
    // Build 19-byte payload
    const payload = Buffer.alloc(19);
    payload.writeUInt8(cfg.masterMode,  0);  // 0=Discretion 1=Manual 2=Autonomous
    payload.writeUInt8(cfg.order,       1);  // 0=None 1=Waypoint 2=Orbit
    payload.writeFloatLE(cfg.lat,       2);
    payload.writeFloatLE(cfg.lon,       6);
    payload.writeFloatLE(cfg.alt,      10);
    payload.writeUInt8(cfg.direction,  14);  // 0=CCW 1=CW
    payload.writeFloatLE(cfg.radius,   15);  // orbit radius (metres)

    // Frame: [START][TYPE][LEN][PAYLOAD...][CRC_H][CRC_L][END]
    const len = payload.length;
    const packet = Buffer.alloc(1 + 1 + 1 + len + 2 + 1);
    let idx = 0;
    packet[idx++] = MAGIC_START;
    packet[idx++] = PKT_CONFIG;
    packet[idx++] = len;
    payload.copy(packet, idx); idx += len;

    // CRC16 over TYPE + LEN + PAYLOAD
    const crc = calculateCRC16(packet, 1, 1 + 1 + len);
    packet[idx++] = (crc >> 8) & 0xFF;
    packet[idx++] =  crc       & 0xFF;
    packet[idx++] = MAGIC_END;

    await new Promise((resolve, reject) => {
      port.write(packet, (err) => { err ? reject(err) : resolve(); });
    });

    return { success: true };
  } catch (error) {
    return { success: false, error: error.message };
  }
});

// Position-only Simulator (kept for map trail and waypoint testing)
// All values are fixed except lat/lon which drift slowly to simulate flight
let _simLat = -33.456;
let _simLon = -70.648;
function startSimulator(webContents) {
  setInterval(() => {
    _simLat += (Math.random() - 0.5) * 0.0004;
    _simLon += (Math.random() - 0.5) * 0.0004;
    const simData = {
        latitude:     _simLat,
        longitude:    _simLon,
        altitude:     120,
        currentMode:  1,
        gforce:       1.05,
        velocity_mag: 22,
        pitch:        -8,
        yaw:          45,
        pos_local_x: 0, pos_local_y: 0, pos_local_z: 0,
        cmd_yaw: 0, cmd_throttle: 90, cmd_pitch: -8, cmd_roll: 0
    };
    if (!webContents.isDestroyed()) {
      webContents.send("telemetry-data", simData);
    }
  }, 1000);
}
