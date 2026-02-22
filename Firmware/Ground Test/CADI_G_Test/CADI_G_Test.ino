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
float pitch = 0; // Degrees
float yaw = 0;   // Degrees

// System State
float gforce = 0; // G-Force

// nRF24L01 Configuration
#define CE_PIN 5
#define CSN_PIN 4

RF24 radio(CE_PIN, CSN_PIN);
const byte pipeTX[6] = "CMD01"; // Command Out Pipe
const byte pipeRX[6] = "TEL01"; // Telemetry In Pipe

// Payload Structures
int16_t commands[4];
float telemetry[6];

// HID Configuration
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// ═══════════════════════════════════════════════════════
// FILTER SYSTEM (KALMAN)
// ═══════════════════════════════════════════════════════

class KalmanFilter {
private:
  float _q, _r, _x, _p, _k;

public:
  KalmanFilter(float q, float r, float p, float initial_value)
      : _q(q), _r(r), _p(p), _x(initial_value) {}

  float update(float measurement) {
    _p = _p + _q;
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

KalmanFilter kfVx(0.1, 1.5, 1.0, 0);
KalmanFilter kfVy(0.1, 1.5, 1.0, 0);
KalmanFilter kfVz(0.1, 1.5, 1.0, 0);

// ═══════════════════════════════════════════════════════
// NAVIGATION SYSTEM
// ═══════════════════════════════════════════════════════

Vector3D gpsOrigin(0, 0, 0);
bool originEstablished = false;

enum NavigationMode { MODE_MANUAL = 0, MODE_WAYPOINT = 1, MODE_ORBIT = 2 };
NavigationMode currentMode = MODE_MANUAL;

Vector3D targetWaypoint(50.0, 100.0, 30.0);
Vector3D orbitCenter(100.0, 100.0, 50.0);
float orbitRadius = 50.0f;
float orbitAltitude = 50.0f;
bool orbitClockwise = false;

struct AircraftState {
  Vector3D position;
  Vector3D velocity;
  float pitch, yaw;
};

AircraftState aircraft;

struct ControlCommands {
  int yaw;
  int throttle;
  int pitch;
  int roll;
};

// ═══════════════════════════════════════════════════════
// VELOCITY ESTIMATION
// ═══════════════════════════════════════════════════════

Vector3D lastPosition(0, 0, 0);
unsigned long lastTimestamp = 0;
bool firstPositionFixed = true;

const float GPS_WEIGHT = 0.3f;
const float MAX_VELOCITY = 50.0f;
const int SMOOTHING_BUFFER_SIZE = 3;

Vector3D calculateVelocity(Vector3D currentPos, float r, float p, float y) {
  unsigned long currentTimestamp = millis();

  if (firstPositionFixed) {
    firstPositionFixed = false;
    lastPosition = currentPos;
    lastTimestamp = currentTimestamp;
    return Vector3D(0, 0, 0);
  }

  float dt = (currentTimestamp - lastTimestamp) / 1000.0f;
  if (dt < 0.01f)
    return aircraft.velocity;

  Vector3D gpsVelocity = (currentPos - lastPosition) * (1.0f / dt);
  float gpsSpeed = gpsVelocity.magnitude();

  if (gpsSpeed > MAX_VELOCITY) {
    lastPosition = currentPos;
    lastTimestamp = currentTimestamp;
    return aircraft.velocity;
  }

  float currentMagnitude = aircraft.velocity.magnitude();
  float radP = p * PI / 180.0f;
  float radY = y * PI / 180.0f;

  Vector3D imuVelocityDirection(cos(radP) * sin(radY), cos(radP) * cos(radY),
                                sin(radP));

  Vector3D imuVelocity = imuVelocityDirection * currentMagnitude;

  if (currentMagnitude < 2.0f) {
    imuVelocity = imuVelocityDirection * gpsSpeed;
  }

  Vector3D hybridVelocity(
      imuVelocity.x * (1.0f - GPS_WEIGHT) + gpsVelocity.x * GPS_WEIGHT,
      imuVelocity.y * (1.0f - GPS_WEIGHT) + gpsVelocity.y * GPS_WEIGHT,
      imuVelocity.z * (1.0f - GPS_WEIGHT) + gpsVelocity.z * GPS_WEIGHT);

  Vector3D filteredVelocity(kfVx.update(hybridVelocity.x),
                            kfVy.update(hybridVelocity.y),
                            kfVz.update(hybridVelocity.z));

  static Vector3D velocityBuffer[SMOOTHING_BUFFER_SIZE] = {
      {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  static int bufferIndex = 0;

  velocityBuffer[bufferIndex] = filteredVelocity;
  bufferIndex = (bufferIndex + 1) % SMOOTHING_BUFFER_SIZE;

  Vector3D smoothedVelocity(0, 0, 0);
  for (int i = 0; i < SMOOTHING_BUFFER_SIZE; i++) {
    smoothedVelocity = smoothedVelocity + velocityBuffer[i];
  }
  smoothedVelocity = smoothedVelocity * (1.0f / SMOOTHING_BUFFER_SIZE);

  lastPosition = currentPos;
  lastTimestamp = currentTimestamp;

  return smoothedVelocity;
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
    Serial.println(" m\n");
  }
}

Vector3D GPSToLocal(float lat, float lon, float alt) {
  if (!originEstablished)
    return Vector3D(0, 0, 0);

  float dLat = (lat - gpsOrigin.x) * 111320.0f;
  float dLon = (lon - gpsOrigin.y) * 111320.0f * cos(lat * PI / 180.0f);
  float dAlt = alt - gpsOrigin.z;

  return Vector3D(dLon, dLat, dAlt);
}

// ═══════════════════════════════════════════════════════
// WAYPOINT NAVIGATION SYSTEM
// ═══════════════════════════════════════════════════════

struct NavigationResult {
  float horizontalAngle;
  float verticalAngle;
  float distance;
  bool turnLeft;
  bool targetReached;
};

NavigationResult analyzeNavigation(AircraftState state, Vector3D target) {
  NavigationResult result;

  Vector3D horizontalVel(state.velocity.x, state.velocity.y, 0);
  float speed = horizontalVel.magnitude();

  const float STALL_SPEED = 5.0f;
  if (speed < STALL_SPEED) {
    result.targetReached = false;
  }

  Vector3D travelDirection = horizontalVel.normalize();
  Vector3D error3D = target - state.position;
  result.distance = error3D.magnitude();

  Vector3D horizontalError(error3D.x, error3D.y, 0);
  float horizontalDist = horizontalError.magnitude();

  if (horizontalDist > 0.1) {
    Vector3D targetDirection = horizontalError.normalize();
    float dotVal = travelDirection.dot(targetDirection);
    dotVal = constrain(dotVal, -1.0f, 1.0f);
    result.horizontalAngle = acos(dotVal) * 180.0f / PI;
    Vector3D crossProd = travelDirection.cross(targetDirection);
    result.turnLeft = (crossProd.z > 0);
  } else {
    result.horizontalAngle = 0;
    result.turnLeft = false;
  }

  if (horizontalDist > 0.1) {
    result.verticalAngle = atan2(error3D.z, horizontalDist) * 180.0f / PI;
  } else {
    result.verticalAngle = 0;
  }

  const float DISTANCE_TOLERANCE = 8.0f;
  result.targetReached = (result.distance < DISTANCE_TOLERANCE);

  return result;
}

ControlCommands generateWaypointCommands(NavigationResult analysis) {
  ControlCommands cmd;

  if (analysis.targetReached) {
    cmd.yaw = 0;
    cmd.throttle = 90;
    cmd.pitch = 0;
    cmd.roll = 0;
    return cmd;
  }

  const float KP_ROLL = 0.6f;
  const float KP_PITCH = 0.4f;

  float desiredRoll = analysis.horizontalAngle * KP_ROLL;
  desiredRoll = constrain(desiredRoll, 0, 45.0f);
  cmd.roll = (int)(analysis.turnLeft ? desiredRoll : -desiredRoll);

  float desiredPitch = analysis.verticalAngle * KP_PITCH;
  cmd.pitch = (int)constrain(desiredPitch, -20.0f, 20.0f);

  cmd.throttle = 100;
  if (desiredPitch > 5)
    cmd.throttle += 20;
  cmd.throttle = constrain(cmd.throttle, 60, 180);

  cmd.yaw = 0;

  return cmd;
}

// ═══════════════════════════════════════════════════════
// ORBIT NAVIGATION SYSTEM (abbreviated for space)
// ═══════════════════════════════════════════════════════

enum OrbitPhase {
  ORBIT_SEARCH_ENTRY,
  ORBIT_APPROACH,
  ORBIT_ALIGNMENT,
  ORBIT_CAPTURE,
  ORBIT_MAINTENANCE
};

struct OrbitState {
  OrbitPhase phase;
  int entryPointIndex;
  Vector3D entryPoints[4];
  float idealYaw;
  bool initialized;
};

OrbitState orbitState = {ORBIT_SEARCH_ENTRY, 0, {}, 0, false};

const float KP_ORBIT_RADIUS = 0.8f;
const float KP_ORBIT_ALTITUDE = 0.4f;
const float KP_ORBIT_YAW = 1.5f;
const float ORBIT_BASE_ROLL = 20.0f;

const float ORBIT_APPROACH_DIST = 25.0f;
const float ORBIT_ALIGNMENT_DIST = 15.0f;
const float ORBIT_CAPTURE_DIST = 8.0f;
const float ORBIT_YAW_TOLERANCE = 5.0f;

void initializeOrbit() {
  if (orbitState.initialized)
    return;
  orbitState.entryPoints[0] =
      Vector3D(orbitCenter.x, orbitCenter.y + orbitRadius, orbitAltitude);
  orbitState.entryPoints[1] =
      Vector3D(orbitCenter.x + orbitRadius, orbitCenter.y, orbitAltitude);
  orbitState.entryPoints[2] =
      Vector3D(orbitCenter.x, orbitCenter.y - orbitRadius, orbitAltitude);
  orbitState.entryPoints[3] =
      Vector3D(orbitCenter.x - orbitRadius, orbitCenter.y, orbitAltitude);
  orbitState.phase = ORBIT_SEARCH_ENTRY;
  orbitState.initialized = true;
}

float normalizeAngle360(float angle) {
  while (angle < 0)
    angle += 360.0f;
  while (angle >= 360.0f)
    angle -= 360.0f;
  return angle;
}

float calculateYawError(float current, float desired) {
  current = normalizeAngle360(current);
  desired = normalizeAngle360(desired);
  float error = desired - current;
  if (error > 180.0f)
    error -= 360.0f;
  if (error < -180.0f)
    error += 360.0f;
  return error;
}

float calculateIdealEntryYaw(int entryPoint) {
  if (orbitClockwise) {
    switch (entryPoint) {
    case 0:
      return 90.0f;
    case 1:
      return 180.0f;
    case 2:
      return 270.0f;
    case 3:
      return 0.0f;
    }
  } else {
    switch (entryPoint) {
    case 0:
      return 270.0f;
    case 1:
      return 0.0f;
    case 2:
      return 90.0f;
    case 3:
      return 180.0f;
    }
  }
  return 0.0f;
}

int findBestEntryPoint(AircraftState state) {
  float bestScore = -999999;
  int bestPoint = 0;
  for (int i = 0; i < 4; i++) {
    Vector3D wp = orbitState.entryPoints[i];
    Vector3D toWP = wp - state.position;
    float distance = toWP.magnitude();
    Vector3D dirWP = toWP.normalize();
    Vector3D dirVel =
        Vector3D(state.velocity.x, state.velocity.y, 0).normalize();
    float alignment = dirVel.dot(dirWP);
    float score = alignment * 100.0f - distance * 0.1f;
    if (score > bestScore) {
      bestScore = score;
      bestPoint = i;
    }
  }
  return bestPoint;
}

float calculateRadiusError(Vector3D position) {
  Vector3D toCenter(position.x - orbitCenter.x, position.y - orbitCenter.y, 0);
  return toCenter.magnitude() - orbitRadius;
}

ControlCommands generateOrbitCommands(AircraftState state) {
  ControlCommands cmd;
  if (!orbitState.initialized) {
    initializeOrbit();
  }

  switch (orbitState.phase) {
  case ORBIT_SEARCH_ENTRY: {
    orbitState.entryPointIndex = findBestEntryPoint(state);
    orbitState.idealYaw = calculateIdealEntryYaw(orbitState.entryPointIndex);
    orbitState.phase = ORBIT_APPROACH;
    break;
  }
  case ORBIT_APPROACH: {
    Vector3D target = orbitState.entryPoints[orbitState.entryPointIndex];
    NavigationResult nav = analyzeNavigation(state, target);
    cmd = generateWaypointCommands(nav);
    if (nav.distance < ORBIT_APPROACH_DIST) {
      orbitState.phase = ORBIT_ALIGNMENT;
    }
    return cmd;
  }
  case ORBIT_ALIGNMENT: {
    float yawError = calculateYawError(state.yaw, orbitState.idealYaw);
    cmd.yaw = (int)constrain(yawError * KP_ORBIT_YAW, -30.0f, 30.0f);
    float altError = orbitAltitude - state.position.z;
    cmd.pitch = (int)constrain(altError * KP_ORBIT_ALTITUDE, -20.0f, 20.0f);
    cmd.throttle = 100;
    if (cmd.pitch > 5)
      cmd.throttle += 15;
    cmd.throttle = constrain(cmd.throttle, 60, 180);
    cmd.roll = 0;
    Vector3D target = orbitState.entryPoints[orbitState.entryPointIndex];
    float distance = (target - state.position).magnitude();
    bool aligned = abs(yawError) < ORBIT_YAW_TOLERANCE;
    bool close = distance < ORBIT_ALIGNMENT_DIST;
    if (aligned && close) {
      orbitState.phase = ORBIT_CAPTURE;
    }
    return cmd;
  }
  case ORBIT_CAPTURE: {
    float radiusError = calculateRadiusError(state.position);
    float altError = state.position.z - orbitAltitude;
    Vector3D target = orbitState.entryPoints[orbitState.entryPointIndex];
    NavigationResult nav = analyzeNavigation(state, target);
    ControlCommands navCmd = generateWaypointCommands(nav);
    float rollCorrection = radiusError * KP_ORBIT_RADIUS;
    float rollTarget = orbitClockwise ? -ORBIT_BASE_ROLL : ORBIT_BASE_ROLL;
    int orbitRoll = (int)constrain(rollTarget + rollCorrection, -45.0f, 45.0f);
    int orbitPitch =
        (int)constrain(-altError * KP_ORBIT_ALTITUDE, -20.0f, 20.0f);
    int orbitThrottle = 55;
    if (orbitPitch > 5)
      orbitThrottle += 15;
    if (orbitPitch < -5)
      orbitThrottle -= 10;
    orbitThrottle = constrain(orbitThrottle, 30, 100);
    float factor =
        1.0f - constrain(abs(radiusError) / ORBIT_CAPTURE_DIST, 0.0f, 1.0f);
    cmd.roll = (int)(navCmd.roll * (1.0f - factor) + orbitRoll * factor);
    cmd.pitch = (int)(navCmd.pitch * (1.0f - factor) + orbitPitch * factor);
    cmd.throttle =
        (int)(navCmd.throttle * (1.0f - factor) + orbitThrottle * factor);
    cmd.yaw = 0;
    if (abs(radiusError) < ORBIT_CAPTURE_DIST) {
      orbitState.phase = ORBIT_MAINTENANCE;
    }
    return cmd;
  }
  case ORBIT_MAINTENANCE: {
    float radiusError = calculateRadiusError(state.position);
    float altError = state.position.z - orbitAltitude;
    float rollCorrection = radiusError * KP_ORBIT_RADIUS;
    float rollTarget = orbitClockwise ? -ORBIT_BASE_ROLL : ORBIT_BASE_ROLL;
    cmd.roll = (int)constrain(rollTarget + rollCorrection, -45.0f, 45.0f);
    cmd.pitch = (int)constrain(-altError * KP_ORBIT_ALTITUDE, -20.0f, 20.0f);
    cmd.throttle = 55;
    if (cmd.pitch > 5)
      cmd.throttle += 15;
    if (cmd.pitch < -5)
      cmd.throttle -= 10;
    cmd.throttle = constrain(cmd.throttle, 30, 100);
    cmd.yaw = 0;
    return cmd;
  }
  }
  return cmd;
}

void resetOrbitSystem() {
  orbitState.phase = ORBIT_SEARCH_ENTRY;
  orbitState.initialized = false;
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

  // Clear serial buffer
  while (Serial.available()) {
    Serial.read();
  }

  Serial.println("\n\n╔═══════════════════════════════════════╗");
  Serial.println("║    GROUND STATION - ESP32 v4.0       ║");
  Serial.println("║    FIELD TEST ENVIRONMENT (DEBUG)    ║");
  Serial.println("║    GPS+IMU Kalman Fusion System      ║");
  Serial.println("║    Waypoint + Orbit Navigation       ║");
  Serial.println("╚═══════════════════════════════════════╝\n");

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

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();

  Serial.println("[OK] Bluetooth HID: ONLINE");
  Serial.println("[INFO] Controls:");
  Serial.println(" > Button A: WAYPOINT MODE");
  Serial.println(" > Button B: MANUAL MODE");
  Serial.println(" > Button X: ORBIT MODE\n");
}

// ═══════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════

void loop() {
  bool dataUpdated = BP32.update();

  // MODE SWITCHING
  if (dataUpdated && myControllers[0] && myControllers[0]->isConnected()) {
    if (myControllers[0]->a()) {
      currentMode = MODE_WAYPOINT;
      Serial.println("\n[MODE] WAYPOINT NAVIGATION ACTIVATED");
      delay(300);
    }
    if (myControllers[0]->b()) {
      currentMode = MODE_MANUAL;
      resetOrbitSystem();
      Serial.println("\n[MODE] MANUAL CONTROL ACTIVATED");
      delay(300);
    }
    if (myControllers[0]->x()) {
      currentMode = MODE_ORBIT;
      resetOrbitSystem();
      Serial.println("\n[MODE] ORBIT NAVIGATION ACTIVATED");
      delay(300);
    }
  }

  // COMMAND GENERATION
  if (dataUpdated && myControllers[0] && myControllers[0]->isConnected()) {
    ControlCommands cmd;

    switch (currentMode) {
    case MODE_WAYPOINT: {
      NavigationResult nav = analyzeNavigation(aircraft, targetWaypoint);
      cmd = generateWaypointCommands(nav);
      break;
    }
    case MODE_ORBIT: {
      cmd = generateOrbitCommands(aircraft);
      break;
    }
    case MODE_MANUAL:
    default: {
      int axisX = myControllers[0]->axisX();
      int throttleIn = myControllers[0]->throttle();
      int axisRY = myControllers[0]->axisRY();
      int axisRX = myControllers[0]->axisRX();
      cmd.yaw = map(axisX, -511, 512, -40, 40);
      cmd.throttle = map(throttleIn, 0, 1024, 0, 180);
      cmd.pitch = map(axisRY, -511, 512, -30, 30);
      cmd.roll = map(axisRX, -511, 512, -45, 45);
      break;
    }
    }

    commands[0] = cmd.yaw;
    commands[1] = cmd.throttle;
    commands[2] = cmd.pitch;
    commands[3] = cmd.roll;

    // TRANSMISSION
    radio.stopListening();
    bool ok = radio.write(&commands, sizeof(commands));

    if (ok) {
      if (radio.isAckPayloadAvailable()) {
        radio.read(&telemetry, sizeof(telemetry));

        latitude = telemetry[0];
        longitude = telemetry[1];
        yaw = telemetry[2];
        altitude = telemetry[3];
        pitch = telemetry[4];
        gforce = telemetry[5];

        if (!originEstablished) {
          setOrigin(latitude, longitude, altitude);
        }

        aircraft.position = GPSToLocal(latitude, longitude, altitude);
        aircraft.velocity = calculateVelocity(aircraft.position, 0, pitch, yaw);
        aircraft.pitch = pitch;
        aircraft.yaw = yaw;

        // DISPLAY (only in manual mode)
        if (currentMode == MODE_MANUAL) {
          Serial.println("\n╔═══════════════════════════════════════╗");
          Serial.println("║         TELEMETRY DISPLAY             ║");
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
          Serial.print("║ Attitude: P=");
          Serial.print(pitch, 0);
          Serial.print("° Y=");
          Serial.print(yaw, 0);
          Serial.println("°");
          Serial.print("║ G-Force:  ");
          Serial.print(gforce, 2);
          Serial.println(" g");
          Serial.println("╚═══════════════════════════════════════╝");
        }
      }
    }
  }

  delay(50);
}