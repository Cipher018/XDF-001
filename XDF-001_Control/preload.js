const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  // Telemetry
  onTelemetryData: (callback) => ipcRenderer.on('telemetry-data', (_event, value) => callback(value)),
  
  // Serial Port
  listPorts: () => ipcRenderer.invoke('list-ports'),
  connectSerial: (portPath, baudRate) => ipcRenderer.invoke('connect-serial', portPath, baudRate),
  onSerialStatus: (callback) => ipcRenderer.on('serial-status', (_event, value) => callback(value)),
  
  // Logging
  onLogMessage: (callback) => ipcRenderer.on('log-message', (_event, value) => callback(value)),
  
  // File Operations
  selectSavePath: () => ipcRenderer.invoke('select-save-path'),
  saveCSVFile: (path, content) => ipcRenderer.invoke('save-csv-file', path, content),
  
  // Commands
  sendCommand: (cfg) => ipcRenderer.invoke('send-command', cfg),
  
  // Mission
  exportMission: (data) => ipcRenderer.invoke('export-mission', data),
  importMission: () => ipcRenderer.invoke('import-mission'),
  
  // Stats
  getPacketStats: () => ipcRenderer.invoke('get-packet-stats'),
  
  // UI
  toggleFullscreen: () => ipcRenderer.send('toggle-fullscreen'),
});
