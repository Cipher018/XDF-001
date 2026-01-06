const { app, BrowserWindow, ipcMain } = require("electron");
const path = require("path");
const { SerialPort } = require("serialport");
const { ReadlineParser } = require("@serialport/parser-readline");

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
    icon: path.join(__dirname, "assets/icon.png"), // Fallback if no icon
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

// Serial Port Logic
let port;
ipcMain.handle("connect-serial", async (event, portPath) => {
  try {
    if (port && port.isOpen) {
      await new Promise((resolve) => port.close(resolve));
    }

    port = new SerialPort({ path: portPath, baudRate: 9600 });
    const parser = port.pipe(new ReadlineParser({ delimiter: "\r\n" }));

    parser.on("data", (data) => {
      try {
        const telemetry = JSON.parse(data);
        event.sender.send("telemetry-data", telemetry);
      } catch (e) {
        console.error("Error parsing telemetry data:", e);
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

// Telemetry Simulator (for Demo)
function startSimulator(webContents) {
  setInterval(() => {
    const fakeData = [
      -33.456 + Math.random() * 0.001, // Latitude
      -70.648 + Math.random() * 0.001, // Longitude
      100 + Math.floor(Math.random() * 10), // Altitude
      "FLYING", // State
      20 + Math.floor(Math.random() * 5), // Temperature
      20 + Math.floor(Math.random() * 5), // Speed
      -10 + Math.floor(Math.random() * 2), // Pitch
      20 + Math.floor(Math.random() * 2), // Roll
      0 + Math.floor(Math.random() * 2), // Yaw
      220 + Math.floor(Math.random() * 2), // Bearing
    ];
    if (!webContents.isDestroyed()) {
      webContents.send("telemetry-data", fakeData);
    }
  }, 1000);
}
