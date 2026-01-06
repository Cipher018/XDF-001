# C.A.D.I - Drone Telemetry Dashboard

C.A.D.I (Control and Automated Data Intelligence) is a professional-grade telemetry visualization application designed for custom-built drones. Built with Electron, it provides real-time monitoring of critical flight data, live video streaming, and dynamic mapping.

![Dashboard Preview](assets/preview.png) _(Note: Placeholder for actual screenshot)_

## Features

- **Real-time Telemetry**: High-visibility cards for Altitude, Speed, Wind, Temperature, and Humidity.
- **Dynamic Orientation**: Precise tracking of Pitch, Roll, Yaw, and Bearing.
- **Live Mapping**: Integrated Leaflet.js map tracking the drone's position in real-time.
- **Webcam Integration**: Live feed for USB-connected webcams.
- **Real-time Graphing**: Dynamic altitude tracking using Chart.js.
- **Professional Aesthetics**: Futuristically designed dark theme optimized for 1920x1080 resolution.

## Tech Stack

- **Framework**: Electron.js
- **Frontend**: Vanilla HTML5, CSS3, JavaScript (ES6+)
- **Mapping**: Leaflet.js
- **Graphics**: Chart.js
- **Connectivity**: Node SerialPort (for USB/COM communication)

## Installation

### Prerequisites

- [Node.js](https://nodejs.org/) (v16.0.0 or higher recommended)
- [npm](https://www.npmjs.com/) (usually bundled with Node.js)

### Setup

1. Clone the repository or download the source code.
2. Open a terminal in the project directory.
3. Install dependencies:
   ```bash
   npm install
   ```
4. Start the application:
   ```bash
   npm start
   ```

## Data Protocol

The application expects telemetry data via the Serial Port in a **JSON Array** format.

### Data Format Specification

The incoming data should be a stringified JSON array with the following order:

`[Latitude, Longitude, Altitude, State, Temperature, Speed, Pitch, Roll, Yaw, Bearing]`

| Index | Field       | Type          | Description                                |
| ----- | ----------- | ------------- | ------------------------------------------ |
| 0     | Latitude    | Float         | GPS Latitude (e.g., -33.456)               |
| 1     | Longitude   | Float         | GPS Longitude (e.g., -70.648)              |
| 2     | Altitude    | Integer/Float | Current height in meters (m)               |
| 3     | State       | String        | Operational state (e.g., "FLYING", "IDLE") |
| 4     | Temperature | Integer/Float | Ambient temperature in °C                  |
| 5     | Speed       | Integer/Float | Horizontal speed in Km/h                   |
| 6     | Pitch       | Integer/Float | Pitch angle in degrees                     |
| 7     | Roll        | Integer/Float | Roll angle in degrees                      |
| 8     | Yaw         | Integer/Float | Yaw angle in degrees                       |
| 9     | Bearing     | Integer/Float | Heading/Bearing in degrees                 |

### Simulation Mode

By default, the application includes a built-in simulator that generates dummy data to demonstrate the dashboard's capabilities without requiring hardware. This can be toggled or removed in `main.js`.

## License

This project is licensed under the ISC License.
