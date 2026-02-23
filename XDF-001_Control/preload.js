const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  onTelemetryData: (callback) => ipcRenderer.on('telemetry-data', (_event, value) => callback(value)),
  listPorts: () => ipcRenderer.invoke('list-ports'),
  connectSerial: (portPath, baudRate) => ipcRenderer.invoke('connect-serial', portPath, baudRate),
  selectSavePath: () => ipcRenderer.invoke('select-save-path'),
  saveCSVFile: (path, content) => ipcRenderer.invoke('save-csv-file', path, content),
  sendCommand: (cfg) => ipcRenderer.invoke('send-command', cfg),
});
