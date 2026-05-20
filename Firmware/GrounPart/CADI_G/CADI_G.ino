#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Bluepad32.h>
#include <Preferences.h>

// ═══════════════════════════════════════════════════════
// GLOBAL DEFINITIONS & MISSION STATE
// ═══════════════════════════════════════════════════════
const int MAX_WAYPOINTS = 16;

struct __attribute__((packed)) RouteWaypoint {
  float lat;
  float lon;
  float alt;
  uint8_t mode;      // 1=Waypoint, 2=Orbit
  uint8_t direction; // 0=CCW, 1=CW
  float radius;
};

RouteWaypoint waypointRoute[MAX_WAYPOINTS];
int waypointCount = 0;
int waypointIndex = 0;
bool routeLoop     = false;

enum NavigationMode { MODE_MANUAL = 0, MODE_WAYPOINT = 1, MODE_ORBIT = 2 };
NavigationMode currentMode = MODE_MANUAL;

// ── nRF24L01 Configuration ──
#define CE_PIN 5
#define CSN_PIN 4
RF24 radio(CE_PIN, CSN_PIN);
const byte pipeTX[6] = "CMD01";
const byte pipeRX[6] = "TEL01";

// ── FHSS Channels (Software hopping matching aircraft) ──
const uint8_t fhssChannels[8] = { 10, 25, 40, 55, 70, 85, 100, 115 };
uint8_t currentChannelIdx = 0;
unsigned long lastAckReceivedMs = 0;

// ── Gamepad & Origin State ──
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
bool originEstablished = false;
float gpsOriginLat = 0.0f;
float gpsOriginLon = 0.0f;
float gpsOriginAlt = 0.0f;

// ── Link Quality Stats ──
static uint8_t lastTelemSeq = 0;
static uint32_t packetLossCount = 0;
static uint32_t packetTotalCount = 0;
uint8_t currentLossRate = 0;

Preferences preferences;

// ═══════════════════════════════════════════════════════
// BINARY SERIAL PROTOCOL (GCS PC <-> GCS ESP32)
// ═══════════════════════════════════════════════════════
const uint8_t MAGIC_START = 0xAA;
const uint8_t MAGIC_END   = 0x55;

// Packet types
const uint8_t PKT_CONFIG    = 0x01; // PC -> GCS ESP32 (config)
const uint8_t PKT_TELEMETRY = 0x02; // GCS ESP32 -> PC (telemetry)
const uint8_t PKT_ACK       = 0x03; // Bidirectional
const uint8_t PKT_NACK      = 0x04; // Bidirectional
const uint8_t PKT_MESSAGE   = 0x05; // GCS ESP32 -> PC (string log)
const uint8_t PKT_ROUTE     = 0x06; // PC -> GCS ESP32 (upload mission route)

// Master mode values
const uint8_t MASTER_DISCRETION = 0;
const uint8_t MASTER_MANUAL     = 1;
const uint8_t MASTER_AUTONOMOUS = 2;

// Order values
const uint8_t ORDER_NONE     = 0;
const uint8_t ORDER_WAYPOINT = 1;
const uint8_t ORDER_ORBIT    = 2;

const uint8_t DIR_CCW = 0;
const uint8_t DIR_CW  = 1;

// PC Configuration structure
struct __attribute__((packed)) ConfigPacket {
  uint8_t masterMode; // 0=discretion, 1=manual, 2=autonomous
  uint8_t order;      // 0=none, 1=waypoint, 2=orbit
  float waypoint_lat;
  float waypoint_lon;
  float waypoint_alt;
  uint8_t direction;  // 0=CCW, 1=CW
  float orbit_radius;
  float declination;  // Degrees
};

// Aircraft raw telemetry from flight controller
struct __attribute__((packed)) AircraftTelemetry {
  float   latitude;
  float   longitude;
  int16_t altitude;     // m * 10
  int16_t heading;      // deg * 10
  int16_t pitch;        // deg * 10
  int16_t roll;         // deg * 10
  int16_t gforce;       // G * 100
  int16_t velocityX;    // m/s * 100
  int16_t velocityY;    // m/s * 100
  int16_t velocityZ;    // m/s * 100
  uint8_t seq;
  uint8_t _pad[3];
}; // Total: 28 bytes

// Full GCS Telemetry Packet sent to PC
struct __attribute__((packed)) TelemetryPacket {
  float   latitude;
  float   longitude;
  float   altitude;
  float   yaw;
  float   pitch;
  float   roll;
  float   gforce;
  float   velocity_mag;
  float   pos_local_x;
  float   pos_local_y;
  float   pos_local_z;
  uint8_t currentMode;
  int16_t cmd_yaw;
  int16_t cmd_throttle;
  int16_t cmd_pitch;
  int16_t cmd_roll;
  uint8_t lossRate;
};

// CRC16 calculations
uint16_t calculateCRC16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc = crc << 1;
    }
  }
  return crc;
}

uint16_t calculateRadioCRC16(const uint8_t *data, size_t length) {
  return calculateCRC16(data, length);
}

// Serial Buffering
const size_t SERIAL_BUFFER_SIZE = 512;
uint8_t serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialBufferIndex = 0;

ConfigPacket lastConfig = {MASTER_DISCRETION, ORDER_NONE, 0.0f, 0.0f, 0.0f, DIR_CCW, 30.0f, -6.0f};
bool configReceived = false;

// ═══════════════════════════════════════════════════════
// ROUTE UPLOAD STATE MACHINE
// ═══════════════════════════════════════════════════════
enum UploadState {
  UPLOAD_IDLE,
  UPLOAD_SENDING
};

UploadState uploadState = UPLOAD_IDLE;
int uploadIndex = 0;
unsigned long lastUploadAttemptMs = 0;
int uploadRetries = 0;

// ═══════════════════════════════════════════════════════
// SECURITY (XXTEA ENCRYPTION)
// ═══════════════════════════════════════════════════════
uint32_t sharedKey[4] = { 0x58444630, 0x30314B45, 0x595F3230, 0x32362121 }; // "XDF001KEY_2026!!"
#define XXTEA_DELTA 0x9e3779b9
#define XXTEA_MX (((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + (sharedKey[(p&3)^e] ^ z)))

void btea(uint32_t *v, int n) {
  uint32_t y, z, sum;
  unsigned p, rounds, e;
  if (n > 1) {
    rounds = 6 + 52/n;
    sum = 0;
    z = v[n-1];
    do {
      sum += XXTEA_DELTA;
      e = (sum >> 2) & 3;
      for (p=0; p<n-1; p++) {
        y = v[p+1];
        z = v[p] += XXTEA_MX;
      }
      y = v[0];
      z = v[n-1] += XXTEA_MX;
    } while (--rounds);
  } else if (n < -1) {
    n = -n;
    rounds = 6 + 52/n;
    sum = rounds*XXTEA_DELTA;
    y = v[0];
    do {
      e = (sum >> 2) & 3;
      for (p=n-1; p>0; p--) {
        z = v[p-1];
        y = v[p] -= XXTEA_MX;
      }
      z = v[n-1];
      y = v[0] -= XXTEA_MX;
      sum -= XXTEA_DELTA;
    } while (--rounds);
  }
}

// ── Secure Radio Headers ──
const uint8_t MAGIC_CMD   = 0xAA;
const uint8_t MAGIC_TELEM = 0xBB;
const uint8_t MAGIC_WP    = 0xAC;

struct __attribute__((packed)) SecureCommand {
  uint8_t  magic;
  uint8_t  seq;
  uint16_t crc;
  int16_t  targetRoll;
  int16_t  targetPitch;
  int16_t  targetYaw;
  int16_t  targetThrottle;
  float    homeLat;
  float    homeLon;
  int16_t  declinationX10;
  uint8_t  navMode;        // 0=MANUAL, 1=WAYPOINT, 2=ORBIT
  uint8_t  cmdFlags;       // Bit 0 = Reset route index
}; // Total: 24 bytes (6 words)

struct __attribute__((packed)) SecureWaypointPacket {
  uint8_t  magic;          // MAGIC_WP = 0xAC
  uint8_t  seq;
  uint16_t crc;
  uint8_t  wpIndex;        // 0..15
  uint8_t  totalWps;
  uint8_t  routeLoop;      // 0=no, 1=yes
  uint8_t  mode;           // NavigationMode (0=MANUAL, 1=WAYPOINT, 2=ORBIT)
  RouteWaypoint wp;        // 18 bytes
  uint8_t  _pad[2];        // Pad to 28 bytes (multiple of 4)
}; // Total: 28 bytes (7 words)

struct __attribute__((packed)) SecureTelemetry {
  uint8_t  magic;
  uint8_t  seq;
  uint16_t crc;
  AircraftTelemetry telem;
}; // 32 bytes

uint8_t cmdSeq = 0;
uint8_t uploadSeq = 0;
AircraftTelemetry aircraftTelem;

// ═══════════════════════════════════════════════════════
// SERIAL COMMUNICATION TO PC
// ═══════════════════════════════════════════════════════
void sendACK() {
  uint8_t packet[6];
  packet[0] = MAGIC_START;
  packet[1] = PKT_ACK;
  packet[2] = 0;
  uint16_t crc = calculateCRC16(&packet[1], 2);
  packet[3] = (crc >> 8) & 0xFF;
  packet[4] = crc & 0xFF;
  packet[5] = MAGIC_END;
  Serial.write(packet, 6);
  Serial.flush();
}

void sendNACK() {
  uint8_t packet[6];
  packet[0] = MAGIC_START;
  packet[1] = PKT_NACK;
  packet[2] = 0;
  uint16_t crc = calculateCRC16(&packet[1], 2);
  packet[3] = (crc >> 8) & 0xFF;
  packet[4] = crc & 0xFF;
  packet[5] = MAGIC_END;
  Serial.write(packet, 6);
  Serial.flush();
}

bool sendTelemetryPacket(const TelemetryPacket &telem) {
  const size_t payloadSize = sizeof(TelemetryPacket);
  const size_t totalSize = 1 + 1 + 1 + payloadSize + 2 + 1;

  uint8_t packet[totalSize];
  size_t idx = 0;

  packet[idx++] = MAGIC_START;
  packet[idx++] = PKT_TELEMETRY;
  packet[idx++] = (uint8_t)payloadSize;

  memcpy(&packet[idx], &telem, payloadSize);
  idx += payloadSize;

  uint16_t crc = calculateCRC16(&packet[1], 1 + 1 + payloadSize);
  packet[idx++] = (crc >> 8) & 0xFF;
  packet[idx++] = crc & 0xFF;
  packet[idx++] = MAGIC_END;

  Serial.write(packet, totalSize);
  return true;
}

bool sendTextMessage(const char* msg) {
  const size_t payloadSize = strlen(msg);
  const size_t totalSize = 1 + 1 + 1 + payloadSize + 2 + 1;

  uint8_t packet[totalSize];
  size_t idx = 0;

  packet[idx++] = MAGIC_START;
  packet[idx++] = PKT_MESSAGE;
  packet[idx++] = (uint8_t)payloadSize;

  memcpy(&packet[idx], msg, payloadSize);
  idx += payloadSize;

  uint16_t crc = calculateCRC16(&packet[1], 1 + 1 + payloadSize);
  packet[idx++] = (crc >> 8) & 0xFF;
  packet[idx++] = crc & 0xFF;
  packet[idx++] = MAGIC_END;

  Serial.write(packet, totalSize);
  return true;
}

// ═══════════════════════════════════════════════════════
// SERIAL PARSING
// ═══════════════════════════════════════════════════════
bool parseConfigPacket(const uint8_t *payload, size_t length) {
  if (length != sizeof(ConfigPacket)) return false;
  memcpy(&lastConfig, payload, sizeof(ConfigPacket));
  configReceived = true;
  return true;
}

bool parseRoutePacket(const uint8_t *payload, size_t length) {
  if (length < 1) return false;
  uint8_t count = payload[0];
  if (count > MAX_WAYPOINTS) count = MAX_WAYPOINTS;

  size_t expectedLen = 1 + count * sizeof(RouteWaypoint);
  if (length < expectedLen) return false;

  waypointCount = count;
  waypointIndex = 0;
  memcpy(waypointRoute, &payload[1], count * sizeof(RouteWaypoint));

  // Trigger sequential route upload to the flight controller
  uploadState = UPLOAD_SENDING;
  uploadIndex = 0;
  uploadRetries = 0;
  lastUploadAttemptMs = 0;

  sendTextMessage("[GCS] Route packet parsed. Initiating sequential upload...");
  return true;
}

void processSerialPacket() {
  if (serialBufferIndex < 4) return;

  int startIdx = -1;
  for (size_t i = 0; i < serialBufferIndex; i++) {
    if (serialBuffer[i] == MAGIC_START) {
      startIdx = i;
      break;
    }
  }

  if (startIdx == -1) {
    serialBufferIndex = 0;
    return;
  } else if (startIdx > 0) {
    memmove(serialBuffer, &serialBuffer[startIdx], serialBufferIndex - startIdx);
    serialBufferIndex -= startIdx;
  }

  if (serialBufferIndex < 4) return;

  uint8_t packetType = serialBuffer[1];
  uint8_t length = serialBuffer[2];
  size_t expectedSize = 1 + 1 + 1 + length + 2 + 1;

  if (expectedSize > SERIAL_BUFFER_SIZE) {
    serialBufferIndex = 0;
    return;
  }

  if (serialBufferIndex < expectedSize) return;

  if (serialBuffer[expectedSize - 1] != MAGIC_END) {
    sendNACK();
    serialBufferIndex = 0;
    return;
  }

  uint16_t receivedCRC = ((uint16_t)serialBuffer[3 + length] << 8) | serialBuffer[3 + length + 1];
  uint16_t calculatedCRC = calculateCRC16(&serialBuffer[1], 1 + 1 + length);

  if (receivedCRC != calculatedCRC) {
    sendNACK();
    memmove(serialBuffer, &serialBuffer[expectedSize], serialBufferIndex - expectedSize);
    serialBufferIndex -= expectedSize;
    return;
  }

  bool success = false;
  switch (packetType) {
    case PKT_CONFIG:
      success = parseConfigPacket(&serialBuffer[3], length);
      break;
    case PKT_ACK:
      success = true;
      break;
    case PKT_NACK:
      success = true;
      break;
    case PKT_ROUTE:
      success = parseRoutePacket(&serialBuffer[3], length);
      break;
  }

  if (packetType == PKT_CONFIG || packetType == PKT_ROUTE) {
    if (success) sendACK();
    else         sendNACK();
  }

  if (serialBufferIndex >= expectedSize) {
    memmove(serialBuffer, &serialBuffer[expectedSize], serialBufferIndex - expectedSize);
    serialBufferIndex -= expectedSize;
  } else {
    serialBufferIndex = 0;
  }
}

// ── Text Commands (SET_KEY) ──
static char setKeyLineBuf[32];
static uint8_t setKeyLineIdx = 0;

void handleSetKeyCommand(const char* line) {
  if (strncmp(line, "SET_KEY:", 8) != 0) return;
  const char* keyStr = line + 8;
  size_t keyLen = strlen(keyStr);
  uint8_t keyBytes[16] = {0};
  size_t copyLen = keyLen < 16 ? keyLen : 16;
  memcpy(keyBytes, keyStr, copyLen);

  Preferences prefs;
  prefs.begin("pairing", false);
  prefs.putBytes("shared_key", keyBytes, 16);
  prefs.end();

  sendTextMessage("[SEC] New key stored in GCS NVS. Restarting...");
  delay(200);
  ESP.restart();
}

void parseSetKeySerial() {
  while (Serial.available()) {
    char ch = (char)Serial.peek();
    if ((uint8_t)ch == MAGIC_START) break;
    Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (setKeyLineIdx > 0) {
        setKeyLineBuf[setKeyLineIdx] = '\0';
        handleSetKeyCommand(setKeyLineBuf);
        setKeyLineIdx = 0;
      }
    } else if (setKeyLineIdx < 30) {
      setKeyLineBuf[setKeyLineIdx++] = ch;
    }
  }
}

void updateSerialParser() {
  parseSetKeySerial();
  while (Serial.available() && serialBufferIndex < SERIAL_BUFFER_SIZE) {
    serialBuffer[serialBufferIndex++] = Serial.read();
  }
  if (serialBufferIndex > 0) {
    processSerialPacket();
  }
  if (serialBufferIndex >= SERIAL_BUFFER_SIZE) {
    serialBufferIndex = 0;
  }
}

// ═══════════════════════════════════════════════════════
// LOCAL TO GLOBAL COORDINATE CONVERSION
// ═══════════════════════════════════════════════════════
void setOrigin(float lat, float lon, float alt) {
  if (!originEstablished) {
    gpsOriginLat = lat;
    gpsOriginLon = lon;
    gpsOriginAlt = alt;
    originEstablished = true;
  }
}

// Local NED positions for PC telemetry view
struct LocalNED {
  float x; // North (m)
  float y; // East (m)
  float z; // Up (m)
};

LocalNED GPSToLocal(float lat, float lon, float alt) {
  LocalNED pos = {0,0,0};
  if (!originEstablished) return pos;
  pos.x = (lat - gpsOriginLat) * 111320.0f;
  pos.y = (lon - gpsOriginLon) * 111320.0f * cosf(lat * 3.14159265f / 180.0f);
  pos.z = alt - gpsOriginAlt;
  return pos;
}

// ═══════════════════════════════════════════════════════
// APPLICATION CONFIGURATION & TELEMETRY BRIDGE
// ═══════════════════════════════════════════════════════
void applySerialConfiguration() {
  if (!configReceived) return;

  static uint8_t lastMasterMode = MASTER_DISCRETION;
  static uint8_t lastOrder = ORDER_NONE;

  bool modeChanged = (lastConfig.masterMode != lastMasterMode);
  bool orderChanged = (lastConfig.order != lastOrder);

  if (!modeChanged && !orderChanged) {
    configReceived = false;
    return;
  }

  switch (lastConfig.masterMode) {
    case MASTER_DISCRETION:
      break;

    case MASTER_MANUAL:
      if (modeChanged) {
        currentMode = MODE_MANUAL;
        uploadState = UPLOAD_IDLE;
        sendTextMessage("[GCS] Manual Mode selected.");
      }
      break;

    case MASTER_AUTONOMOUS:
      if (modeChanged || orderChanged) {
        if (lastConfig.order == ORDER_WAYPOINT) {
          waypointRoute[0].lat = lastConfig.waypoint_lat;
          waypointRoute[0].lon = lastConfig.waypoint_lon;
          waypointRoute[0].alt = lastConfig.waypoint_alt;
          waypointRoute[0].mode = 1; // Waypoint
          waypointRoute[0].direction = 0;
          waypointRoute[0].radius = 0.0f;
          waypointCount = 1;
          waypointIndex = 0;
          currentMode = MODE_WAYPOINT;

          uploadState = UPLOAD_SENDING;
          uploadIndex = 0;
          uploadRetries = 0;
          lastUploadAttemptMs = 0;
          sendTextMessage("[GCS] Target waypoint command received. Uploading to FC...");
        } 
        else if (lastConfig.order == ORDER_ORBIT) {
          waypointRoute[0].lat = lastConfig.waypoint_lat;
          waypointRoute[0].lon = lastConfig.waypoint_lon;
          waypointRoute[0].alt = lastConfig.waypoint_alt;
          waypointRoute[0].mode = 2; // Orbit
          waypointRoute[0].direction = lastConfig.direction;
          waypointRoute[0].radius = max(lastConfig.orbit_radius, 15.0f);
          waypointCount = 1;
          waypointIndex = 0;
          currentMode = MODE_ORBIT;

          uploadState = UPLOAD_SENDING;
          uploadIndex = 0;
          uploadRetries = 0;
          lastUploadAttemptMs = 0;
          sendTextMessage("[GCS] Target orbit command received. Uploading to FC...");
        }
      }
      break;
  }

  lastMasterMode = lastConfig.masterMode;
  lastOrder = lastConfig.order;
  configReceived = false;
}

// ═══════════════════════════════════════════════════════
// FHSS SOFTWARE CHANNEL HOPPING
// ═══════════════════════════════════════════════════════
void handleFhssHopping() {
  if (millis() - lastAckReceivedMs > 300) {
    // Si no hemos recibido un ACK en 300ms, saltar al siguiente canal
    currentChannelIdx = (currentChannelIdx + 1) % 8;
    radio.setChannel(fhssChannels[currentChannelIdx]);
    lastAckReceivedMs = millis(); // Resetear temporizador para el canal nuevo
    
    char msg[64];
    snprintf(msg, sizeof(msg), "[FHSS] No ACK. Hopping to channel %d...", fhssChannels[currentChannelIdx]);
    sendTextMessage(msg);
  }
}

// ═══════════════════════════════════════════════════════
// ROUTE SEQUENTIAL UPLOADING OVER RADIO
// ═══════════════════════════════════════════════════════
void handleRouteUpload() {
  if (uploadIndex >= waypointCount) {
    uploadState = UPLOAD_IDLE;
    return;
  }

  if (millis() - lastUploadAttemptMs > 100) { // Enviar cada 100ms
    lastUploadAttemptMs = millis();

    SecureWaypointPacket secWp;
    secWp.magic = MAGIC_WP;
    secWp.seq   = uploadSeq++;
    secWp.wpIndex = uploadIndex;
    secWp.totalWps = waypointCount;
    secWp.routeLoop = routeLoop ? 1 : 0;
    secWp.mode = (uint8_t)currentMode;
    secWp.wp    = waypointRoute[uploadIndex];
    secWp.crc   = calculateRadioCRC16((uint8_t*)&secWp.wpIndex, 24);

    btea((uint32_t*)&secWp, 7); // Cifrar payload (28 bytes)

    radio.stopListening();
    bool ok = radio.write(&secWp, sizeof(SecureWaypointPacket));

    if (ok) {
      lastAckReceivedMs = millis(); // Validar enlace por FHSS
      
      char logMsg[64];
      snprintf(logMsg, sizeof(logMsg), "[GCS] Uploaded WP %d/%d successfully.", uploadIndex + 1, waypointCount);
      sendTextMessage(logMsg);

      uploadIndex++;
      uploadRetries = 0;

      if (uploadIndex >= waypointCount) {
        uploadState = UPLOAD_IDLE;
        sendTextMessage("[GCS] Route upload finished. Aircraft is autonomous.");
      }
    } else {
      uploadRetries++;
      if (uploadRetries > 10) {
        uploadState = UPLOAD_IDLE;
        sendTextMessage("[GCS] Error: Route upload failed. Connection timeout.");
      }
    }
  }
}

// ═══════════════════════════════════════════════════════
// MANUAL CONTROL & HEARTBEAT TRANSMISSION
// ═══════════════════════════════════════════════════════
void handleNormalControl() {
  int yawCmd = 0;
  int throttleCmd = 0;
  int pitchCmd = 0;
  int rollCmd = 0;

  bool gamepadConnected = (myControllers[0] && myControllers[0]->isConnected());

  if (gamepadConnected) {
    int axisX = myControllers[0]->axisX();
    int throttleIn = myControllers[0]->throttle();
    int axisRY = myControllers[0]->axisRY();
    int axisRX = myControllers[0]->axisRX();
    yawCmd = map(axisX, -511, 512, -40, 40);
    throttleCmd = map(throttleIn, 0, 1024, 0, 180);
    pitchCmd = map(axisRY, -511, 512, -30, 30);
    rollCmd = map(axisRX, -511, 512, -45, 45);
  }

  SecureCommand secCmd;
  secCmd.magic = MAGIC_CMD;
  secCmd.seq   = cmdSeq++;
  secCmd.targetYaw      = yawCmd;
  secCmd.targetThrottle = throttleCmd;
  secCmd.targetPitch    = pitchCmd;
  secCmd.targetRoll     = rollCmd;
  secCmd.declinationX10 = (int16_t)(lastConfig.declination * 10.0f);
  secCmd.navMode        = (uint8_t)currentMode;
  secCmd.cmdFlags       = 0;

  secCmd.homeLat = originEstablished ? gpsOriginLat : 0.0f;
  secCmd.homeLon = originEstablished ? gpsOriginLon : 0.0f;

  secCmd.crc = calculateRadioCRC16((uint8_t*)&secCmd.targetRoll, 20);
  btea((uint32_t*)&secCmd, 6); // Cifrar 24 bytes

  radio.stopListening();
  bool ok = radio.write(&secCmd, sizeof(SecureCommand));

  if (ok) {
    lastAckReceivedMs = millis(); // Actualizar link alive para FHSS

    if (radio.isAckPayloadAvailable()) {
      uint8_t len = radio.getDynamicPayloadSize();
      if (len == sizeof(SecureTelemetry)) {
        SecureTelemetry secTelem;
        radio.read(&secTelem, sizeof(SecureTelemetry));
        btea((uint32_t*)&secTelem, -8); // Descifrar (32 bytes)

        uint16_t crc = calculateRadioCRC16((uint8_t*)&secTelem.telem, sizeof(AircraftTelemetry));
        if (secTelem.magic == MAGIC_TELEM && secTelem.crc == crc) {
          aircraftTelem = secTelem.telem;

          float aircraftLat = aircraftTelem.latitude;
          float aircraftLon = aircraftTelem.longitude;
          float aircraftAlt = aircraftTelem.altitude / 10.0f;
          float aircraftYaw = aircraftTelem.heading / 10.0f;
          float aircraftPitch = aircraftTelem.pitch / 10.0f;
          float aircraftRoll = aircraftTelem.roll / 10.0f;
          float aircraftGForce = aircraftTelem.gforce / 100.0f;
          float vx = aircraftTelem.velocityX / 100.0f;
          float vy = aircraftTelem.velocityY / 100.0f;
          float vz = aircraftTelem.velocityZ / 100.0f;

          if (!originEstablished && aircraftLat != 0.0f) {
            setOrigin(aircraftLat, aircraftLon, aircraftAlt);
          }

          // Link Quality math
          packetTotalCount++;
          if (lastTelemSeq != 0) {
            uint8_t expected = lastTelemSeq + 1;
            if (aircraftTelem.seq != expected) {
              int lost = (int)aircraftTelem.seq - (int)expected;
              if (lost < 0) lost += 256;
              packetLossCount += lost;
              packetTotalCount += lost;
            }
          }
          lastTelemSeq = aircraftTelem.seq;
          currentLossRate = (uint8_t)((packetLossCount * 100) / packetTotalCount);
          if (packetTotalCount > 1000) {
            packetTotalCount = 0;
            packetLossCount = 0;
          }

          // PC telemetry reporting loop (~10Hz)
          static uint8_t telemCounter = 0;
          if (++telemCounter >= 5) { // Enviar cada 250ms a PC
            telemCounter = 0;

            LocalNED posNED = GPSToLocal(aircraftLat, aircraftLon, aircraftAlt);

            TelemetryPacket telemPkt;
            telemPkt.latitude     = aircraftLat;
            telemPkt.longitude    = aircraftLon;
            telemPkt.altitude     = aircraftAlt;
            telemPkt.yaw          = aircraftYaw;
            telemPkt.pitch        = aircraftPitch;
            telemPkt.roll         = aircraftRoll;
            telemPkt.gforce       = aircraftGForce;
            telemPkt.velocity_mag = sqrtf(vx*vx + vy*vy + vz*vz);
            telemPkt.pos_local_x  = posNED.x;
            telemPkt.pos_local_y  = posNED.y;
            telemPkt.pos_local_z  = posNED.z;
            telemPkt.currentMode  = (uint8_t)currentMode;
            telemPkt.cmd_yaw      = yawCmd;
            telemPkt.cmd_throttle = throttleCmd;
            telemPkt.cmd_pitch    = pitchCmd;
            telemPkt.cmd_roll     = rollCmd;
            telemPkt.lossRate     = currentLossRate;

            sendTelemetryPacket(telemPkt);
          }
        }
      } else {
        uint8_t dummy[32];
        radio.read(&dummy, len);
      }
    }
  }
}

// ═══════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);

  while (Serial.available()) {
    Serial.read();
  }

  if (!radio.begin()) {
    while (1);
  }

  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(3, 5);
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(fhssChannels[0]);
  radio.stopListening();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();

  // Load Secure pairing key from NVS
  preferences.begin("pairing", true);
  if (preferences.isKey("shared_key")) {
    uint8_t nvsKey[16];
    preferences.getBytes("shared_key", nvsKey, 16);
    memcpy(sharedKey, nvsKey, 16);
  }
  preferences.end();

  lastAckReceivedMs = millis();
}

// ═══════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════
void loop() {
  updateSerialParser();
  applySerialConfiguration();
  BP32.update();

  // FHSS channel management
  handleFhssHopping();

  // Control path / route upload path
  if (uploadState == UPLOAD_SENDING) {
    handleRouteUpload();
  } else {
    handleNormalControl();
  }

  delay(50); // Loop execution at ~20Hz
}
