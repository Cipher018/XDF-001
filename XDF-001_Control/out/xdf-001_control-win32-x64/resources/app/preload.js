const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  onTelemetryData: (callback) => ipcRenderer.on('telemetry-data', (_event, value) => callback(value)),
  listPorts: () => ipcRenderer.invoke('list-ports'),
  connectSerial: (portPath) => ipcRenderer.invoke('connect-serial', portPath),
  selectSavePath: () => ipcRenderer.invoke('select-save-path'),
  saveCSVFile: (path, content) => ipcRenderer.invoke('save-csv-file', path, content)
});
