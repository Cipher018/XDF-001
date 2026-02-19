// Import libraries
#include <Bluepad32.h>
#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>

// ═══════════════════════════════════════════════════════
// VECTORS CLASS
// ═══════════════════════════════════════════════════════

class Vector3D {
public:
  float x, y, z;

  Vector3D(float _x = 0, float _y = 0, float _z = 0) : x(_x), y(_y), z(_z) {}

  Vector3D operator-(const Vector3D &v) const {
    return Vector3D(x - v.x, y - v.y, z - v.z);
  }

  Vector3D operator+(const Vector3D &v) const {
    return Vector3D(x + v.x, y + v.y, z + v.z);
  }

  Vector3D operator*(float s) const { return Vector3D(x * s, y * s, z * s); }

  float magnitude() const { return sqrt(x * x + y * y + z * z); }

  Vector3D normalize() const {
    float mag = magnitude();
    if (mag < 0.001)
      return Vector3D(0, 0, 0);
    return Vector3D(x / mag, y / mag, z / mag);
  }

  float dot(const Vector3D &v) const { return x * v.x + y * v.y + z * v.z; }

  Vector3D cross(const Vector3D &v) const {
    return Vector3D(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }
};

// ═══════════════════════════════════════════════════════
// TELEMETRY
// ═══════════════════════════════════════════════════════

// GPS Data
float latitude = 0;  // Degrees
float longitude = 0; // Degrees
float altitude = 0;  // Meters

// Orientation (Attitude)
float roll = 0;  // Degrees
float pitch = 0; // Degrees
float yaw = 0;   // Degrees

// System State
int batteryPercent = 0; // Percentage

// nRF24L01 Configuration
#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);
const byte pipeTX[6] = "CMD01"; // Command Out Pipe
const byte pipeRX[6] = "TEL01"; // Telemetry In Pipe

// Payload Structures
int16_t commands[4];
float telemetry[7]; // Lat, Lon, Alt, Roll, Pitch, Yaw, Battery

// HID Configuration
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// ═══════════════════════════════════════════════════════
// FILTER SYSTEM (KALMAN)
// ═══════════════════════════════════════════════════════

class KalmanFilter {
private:
  float _q; // Process noise covariance
  float _r; // Measurement noise covariance
  float _x; // Value
  float _p; // Estimation error covariance
  float _k; // Kalman gain

public:
  KalmanFilter(float q, float r, float p, float initial_value)
      : _q(q), _r(r), _p(p), _x(initial_value) {}

  float update(float measurement) {
    // Prediction update
    _p = _p + _q;

    // Measurement update
    _k = _p / (_p + _r);
    _x = _x + _k * (measurement - _x);
    _p = (1 - _k) * _p;

    return _x;
  }

  void setState(float x, float p) {
    _x = x;
    _p = p;
  }

  float getState() { return _x; }
};

// Global Kalman filters for velocity components
KalmanFilter kfVx(0.1, 1.5, 1.0, 0);
KalmanFilter kfVy(0.1, 1.5, 1.0, 0);
KalmanFilter kfVz(0.1, 1.5, 1.0, 0);

// ═══════════════════════════════════════════════════════
// NAVIGATION SYSTEM
// ═══════════════════════════════════════════════════════

// Reference System
Vector3D gpsOrigin(0, 0, 0);
bool originEstablished = false;

// Waypoints
Vector3D targetWaypoint(50.0, 100.0, 30.0); // 50m East, 100m North, 30m Alt
bool autonomousMode = false;                // false = manual, true = auto

// Aircraft state in local coordinates
struct AircraftState {
  Vector3D position;      // meters (x, y, z)
  Vector3D velocity;      // m/s (vx, vy, vz)
  float roll, pitch, yaw; // degrees
};

AircraftState aircraft;

// ═══════════════════════════════════════════════════════
// VELOCITY ESTIMATION (SENSOR FUSION)
// ═══════════════════════════════════════════════════════

Vector3D lastPosition(0, 0, 0);
unsigned long lastTimestamp = 0;
bool firstPositionFixed = true;

Vector3D calculateVelocity(Vector3D currentPos, float r, float p, float y) {
  unsigned long currentTimestamp = millis();

  if (firstPositionFixed) {
    firstPositionFixed = false;
    lastPosition = currentPos;
    lastTimestamp = currentTimestamp;
    return Vector3D(0, 0, 0);
  }

  // Time delta in seconds
  float dt = (currentTimestamp - lastTimestamp) / 1000.0f;
  if (dt < 0.01f)
    return aircraft.velocity;

  // 1. GPS-based velocity (Measurement)
  Vector3D rawVelocity = (currentPos - lastPosition) * (1.0f / dt);

  // 2. IMU-based velocity direction (Polar Projection)
  // Convert Euler to local unit vector
  float radP = p * PI / 180.0f;
  float radY = y * PI / 180.0f;

  // Standard aircraft projection: X=East, Y=North, Z=Up
  // Note: This applies attitude to the magnitude
  float currentMag = aircraft.velocity.magnitude();
  if (currentMag < 1.0f)
    currentMag = rawVelocity.magnitude(); // Fallback to raw if near zero

  Vector3D predictedVelocity(currentMag * cos(radP) * sin(radY),
                             currentMag * cos(radP) * cos(radY),
                             currentMag * sin(radP));

  // 3. Kalman Fusion
  // We update the Kalman filter for each axis
  Vector3D filteredVelocity(kfVx.update(rawVelocity.x),
                            kfVy.update(rawVelocity.y),
                            kfVz.update(rawVelocity.z));

  // Update for next cycle
  lastPosition = currentPos;
  lastTimestamp = currentTimestamp;

  return filteredVelocity;
}

// ═══════════════════════════════════════════════════════
// COORDINATE CONVERSION
// ═══════════════════════════════════════════════════════

void setOrigin(float lat, float lon, float alt) {
  if (!originEstablished) {
    gpsOrigin.x = lat;
    gpsOrigin.y = lon;
    gpsOrigin.z = alt;
    originEstablished = true;

    Serial.println("\n[SYSTEM] REFERENCE ORIGIN ESTABLISHED");
    Serial.print(" > Origin Lat: ");
    Serial.println(lat, 6);
    Serial.print(" > Origin Lon: ");
    Serial.println(lon, 6);
    Serial.print(" > Origin Alt: ");
    Serial.print(alt, 1);
    Serial.println(" m");
  }
}

Vector3D GPSToLocal(float lat, float lon, float alt) {
  if (!originEstablished)
    return Vector3D(0, 0, 0);

  // Flat Earth Approximation (<10km)
  float dLat = (lat - gpsOrigin.x) * 111320.0f;
  float dLon = (lon - gpsOrigin.y) * 111320.0f * cos(lat * PI / 180.0f);
  float dAlt = alt - gpsOrigin.z;

  return Vector3D(dLon, dLat, dAlt); // East (X), North (Y), Up (Z)
}

struct NavigationResult {
  float horizontalAngle; // Degrees
  float verticalAngle;   // Degrees (Elevation)
  float distance;        // Meters
  bool turnLeft;         // Direction
  bool targetReached;
};

// ═══════════════════════════════════════════════════════
// NAVIGATION ANALYSIS
// ═══════════════════════════════════════════════════════

NavigationResult analyzeNavigation(AircraftState state, Vector3D target) {
  NavigationResult result;

  // Horizontal velocity vector (direction of movement)
  Vector3D horizontalVel(state.velocity.x, state.velocity.y, 0);
  float speed = horizontalVel.magnitude();

  // Check minimum stall speed (for safety)
  const float STALL_SPEED = 5.0f; // m/s
  if (speed < STALL_SPEED) {
    Serial.print("[WARN] LOW SPEED DETECTED: ");
    Serial.print(speed, 1);
    Serial.println(" m/s");
    result.targetReached = false;
    // Even if slow, we calculate direction
  }

  Vector3D travelDirection = horizontalVel.normalize();

  // Full 3D error vector
  Vector3D error3D = target - state.position;
  result.distance = error3D.magnitude();

  // Horizontal error vector (XY plane)
  Vector3D horizontalError(error3D.x, error3D.y, 0);
  float horizontalDist = horizontalError.magnitude();

  // ---------- HORIZONTAL ANGLE ----------
  if (horizontalDist > 0.1) {
    Vector3D targetDirection = horizontalError.normalize();

    // Dot product for the angle between motion and target
    float dotVal = travelDirection.dot(targetDirection);
    dotVal = constrain(dotVal, -1.0f, 1.0f);
    result.horizontalAngle = acos(dotVal) * 180.0f / PI;

    // Cross product for turn direction (Z component)
    Vector3D crossProd = travelDirection.cross(targetDirection);
    result.turnLeft = (crossProd.z > 0);

  } else {
    result.horizontalAngle = 0;
    result.turnLeft = false;
  }

  // ---------- VERTICAL ANGLE ----------
  if (horizontalDist > 0.1) {
    result.verticalAngle = atan2(error3D.z, horizontalDist) * 180.0f / PI;
  } else {
    result.verticalAngle = 0;
  }

  // ---------- TARGET CHECK ----------
  const float DISTANCE_TOLERANCE = 8.0f; // meters
  result.targetReached = (result.distance < DISTANCE_TOLERANCE);

  return result;
}

// ═══════════════════════════════════════════════════════
// NAVIGATION COMMAND GENERATION
// ═══════════════════════════════════════════════════════

void generateNavigationCommands(NavigationResult analysis,
                                int16_t *outCommands) {
  if (analysis.targetReached) {
    Serial.println("[NAV] TARGET WAYPOINT REACHED!");
    // Maintain level flight / hover
    outCommands[0] = 0;  // Yaw
    outCommands[1] = 90; // Power (neutral)
    outCommands[2] = 0;  // Pitch
    outCommands[3] = 0;  // Roll
    return;
  }

  // Controller Gains (P-Control)
  const float KP_ROLL = 0.6f;
  const float KP_PITCH = 0.4f;

  // ---------- ROLL COMMAND ----------
  float desiredRoll = analysis.horizontalAngle * KP_ROLL;
  desiredRoll = constrain(desiredRoll, 0, 45.0f);

  // Apply sign based on turn direction
  int rollCmd = (int)(analysis.turnLeft ? desiredRoll : -desiredRoll);

  // ---------- PITCH COMMAND ----------
  float desiredPitch = analysis.verticalAngle * KP_PITCH;
  int pitchCmd = (int)constrain(desiredPitch, -20.0f, 20.0f);

  // ---------- THROTTLE COMMAND ----------
  int throttleCmd = 100; // Base power
  if (desiredPitch > 5)
    throttleCmd += 20; // Increase power when climbing
  throttleCmd = constrain(throttleCmd, 60, 180);

  // ---------- YAW COMMAND ----------
  int yawCmd = 0; // Usually managed by stabilization on-board

  // Assign to output array
  outCommands[0] = yawCmd;
  outCommands[1] = throttleCmd;
  outCommands[2] = pitchCmd;
  outCommands[3] = rollCmd;

  // Debug Dashboard
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║        AUTONOMOUS NAVIGATION          ║");
  Serial.println("╠═══════════════════════════════════════╣");
  Serial.print("║ Distance: ");
  Serial.print(analysis.distance, 1);
  Serial.println(" m");
  Serial.print("║ H-Angle:  ");
  Serial.print(analysis.horizontalAngle, 1);
  Serial.println("°");
  Serial.print("║ V-Angle:  ");
  Serial.print(analysis.verticalAngle, 1);
  Serial.println("°");
  Serial.print("║ Steering: ");
  Serial.println(analysis.turnLeft ? "LEFT ↺" : "RIGHT ↻");
  Serial.println("╟───────────────────────────────────────╢");
  Serial.print("║ Throttle: ");
  Serial.print(throttleCmd);
  Serial.println("%");
  Serial.print("║ Roll:     ");
  Serial.print(rollCmd);
  Serial.print("°  Pitch: ");
  Serial.print(pitchCmd);
  Serial.println("°");
  Serial.println("╚═══════════════════════════════════════╝");
}

// ═══════════════════════════════════════════════════════
// HID CALLBACKS
// ═══════════════════════════════════════════════════════

void onConnectedController(ControllerPtr ctl) {
  Serial.println("[HID] CONTROLLER ONLINE");
  myControllers[0] = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println("[HID] CONTROLLER OFFLINE");
  myControllers[0] = nullptr;
}

// ═══════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n╔═══════════════════════════════════════╗");
  Serial.println("║    GROUND STATION - ESP32 v2.0       ║");
  Serial.println("║    Mixed Sensor Fusion Nav System    ║");
  Serial.println("║    English Refactored Codebase       ║");
  Serial.println("╚═══════════════════════════════════════╝\n");

  // ---------- RADIO INITIALIZATION ----------
  if (!radio.begin()) {
    Serial.println("[ERROR] nRF24L01 NOT DETECTED");
    while (1)
      ;
  }

  radio.enableAckPayload();
  radio.setRetries(3, 5);
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  Serial.println("[OK] Radio Link: ONLINE");

  // ---------- HID INITIALIZATION ----------
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();

  Serial.println("[OK] Bluetooth HID: ONLINE\n");

  // ---------- INITIAL CONFIGURATION ----------
  Serial.println("[NAV] Target Waypoint:");
  Serial.print(" > East:  ");
  Serial.print(targetWaypoint.x, 1);
  Serial.println(" m");
  Serial.print(" > North: ");
  Serial.print(targetWaypoint.y, 1);
  Serial.println(" m");
  Serial.print(" > Alt:   ");
  Serial.print(targetWaypoint.z, 1);
  Serial.println(" m");
  Serial.println("\n[INFO] Press 'A' for AUTONOMOUS MODE");
  Serial.println("[INFO] Press 'B' for MANUAL MODE\n");
}

// ═══════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════

void loop() {
  bool dataUpdated = BP32.update();

  // ---------- MODE SWITCHING ----------
  if (dataUpdated && myControllers[0] && myControllers[0]->isConnected()) {
    // Button A = Autonomous Mode
    if (myControllers[0]->a()) {
      autonomousMode = true;
      Serial.println("[MODE] AUTONOMOUS ACTIVATED");
      delay(300); // Debounce
    }

    // Button B = Manual Mode
    if (myControllers[0]->b()) {
      autonomousMode = false;
      Serial.println("[MODE] MANUAL ACTIVATED");
      delay(300); // Debounce
    }
  }

  // ---------- COMMAND GENERATION ----------
  if (dataUpdated && myControllers[0] && myControllers[0]->isConnected()) {

    if (autonomousMode) {
      // ----- AUTONOMOUS MODE -----
      NavigationResult analysis = analyzeNavigation(aircraft, targetWaypoint);
      generateNavigationCommands(analysis, commands);

    } else {
      // ----- MANUAL MODE -----
      int axisX = myControllers[0]->axisX();
      int throttleIn = myControllers[0]->throttle();
      int axisRY = myControllers[0]->axisRY();
      int axisRX = myControllers[0]->axisRX();

      // Mapping HID axes to command limits
      commands[0] = map(axisX, -511, 512, -40, 40);   // Yaw
      commands[1] = map(throttleIn, 0, 1024, 0, 180); // Power
      commands[2] = map(axisRY, -511, 512, -30, 30);  // Pitch
      commands[3] = map(axisRX, -511, 512, -45, 45);  // Roll
    }

    // ---------- TRANSMISSION ----------
    radio.stopListening();
    bool ok = radio.write(&commands, sizeof(commands));

    if (ok) {
      radio.startListening();

      // ---------- TELEMETRY WAIT ----------
      unsigned long timeout = millis();
      while (!radio.available() && millis() - timeout < 200) {
        // Wait for ack payload
      }

      if (radio.available()) {
        radio.read(&telemetry, sizeof(telemetry));

        // Parse Received Floats
        latitude = telemetry[0];
        longitude = telemetry[1];
        altitude = telemetry[2];
        roll = telemetry[3];
        pitch = telemetry[4];
        yaw = telemetry[5];
        batteryPercent = (int)telemetry[6];

        // ---------- ORIGIN UPDATE ----------
        if (!originEstablished) {
          setOrigin(latitude, longitude, altitude);
        }

        // ---------- STATE FUSION ----------
        aircraft.position = GPSToLocal(latitude, longitude, altitude);
        aircraft.velocity =
            calculateVelocity(aircraft.position, roll, pitch, yaw);
        aircraft.roll = roll;
        aircraft.pitch = pitch;
        aircraft.yaw = yaw;

        // ---------- DISPLAY TELEMETRY ----------
        if (!autonomousMode) { // Reduced clutter in manual mode
          Serial.println("╔═══════════════════════════════════════╗");
          Serial.println("║         GROUND TELEMETRY (EN)         ║");
          Serial.println("╠═══════════════════════════════════════╣");
          Serial.print("║ GPS:   ");
          Serial.print(latitude, 6);
          Serial.print(", ");
          Serial.println(longitude, 6);
          Serial.print("║ Local: (");
          Serial.print(aircraft.position.x, 1);
          Serial.print(", ");
          Serial.print(aircraft.position.y, 1);
          Serial.print(", ");
          Serial.print(aircraft.position.z, 1);
          Serial.println(") m");
          Serial.print("║ Velocity: ");
          float spd = aircraft.velocity.magnitude();
          Serial.print(spd, 1);
          Serial.print(" m/s (");
          Serial.print(spd * 3.6, 1);
          Serial.println(" km/h)");
          Serial.print("║ Attitude: R=");
          Serial.print(roll, 0);
          Serial.print("° P=");
          Serial.print(pitch, 0);
          Serial.print("° Y=");
          Serial.print(yaw, 0);
          Serial.println("°");
          Serial.print("║ Battery:  ");
          Serial.print(batteryPercent);
          Serial.println("%");
          Serial.println("╚═══════════════════════════════════════╝");
        }

      } else {
        Serial.println("[WARN] TELEMETRY TIMEOUT");
      }

      radio.stopListening();

    } else {
      Serial.println("[ERROR] RF TRANSMISSION FAILED");
    }
  }

  delay(50);
}