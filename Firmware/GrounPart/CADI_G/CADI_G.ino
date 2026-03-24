#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Bluepad32.h>
#include <Preferences.h> // [F5]

// ── Global Mission State [F2] ──
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

// ── Forward Declarations ──
class Vector3D;
Vector3D GPSToLocal(float lat, float lon, float alt);
void resetOrbitSystem();

// ── nRF24L01 Configuration ──
#define CE_PIN 5
#define CSN_PIN 4
RF24 radio(CE_PIN, CSN_PIN);
const byte pipeTX[6] = "CMD01";
const byte pipeRX[6] = "TEL01";

// ── VECTORS CLASS ──
class Vector3D {
public:
  float x, y, z;
  Vector3D(float _x = 0, float _y = 0, float _z = 0) : x(_x), y(_y), z(_z) {}
  Vector3D operator-(const Vector3D &v) const { return Vector3D(x - v.x, y - v.y, z - v.z); }
  Vector3D operator+(const Vector3D &v) const { return Vector3D(x + v.x, y + v.y, z + v.z); }
  Vector3D operator*(float s) const { return Vector3D(x * s, y * s, z * s); }
  float magnitude() const { return sqrt(x * x + y * y + z * z); }
  Vector3D normalize() const {
    float mag = magnitude();
    if (mag < 0.001) return Vector3D(0, 0, 0);
    return Vector3D(x / mag, y / mag, z / mag);
  }
  float dot(const Vector3D &v) const { return x * v.x + y * v.y + z * v.z; }
  Vector3D cross(const Vector3D &v) const {
    return Vector3D(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }
};

// ── HID & Navigation Variables (Moved up to fix scope) ──
bool  originEstablished = false;
float orbitRadius       = 50.0f;
float orbitAltitude     = 50.0f;
bool  orbitClockwise     = false;
Vector3D gpsOrigin(0, 0, 0);
Vector3D targetWaypoint(50.0f, 100.0f, 30.0f);
Vector3D orbitCenter(100.0f, 100.0f, 50.0f);
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// ── Link Quality Stats [F3] ──
static uint8_t lastTelemSeq = 0;
static uint32_t packetLossCount = 0;
static uint32_t packetTotalCount = 0;
uint8_t currentLossRate = 0;

Preferences preferences;
// uint32_t sharedKey[4] is defined below in security section

// ═══════════════════════════════════════════════════════
// BINARY SERIAL PROTOCOL
// ═══════════════════════════════════════════════════════

// Protocol constants
const uint8_t MAGIC_START = 0xAA;
const uint8_t MAGIC_END = 0x55;

// Packet types
const uint8_t PKT_CONFIG    = 0x01; // PC -> ESP32 (configuration)
const uint8_t PKT_TELEMETRY = 0x02; // ESP32 -> PC (telemetry)
const uint8_t PKT_ACK       = 0x03; // Bidirectional (acknowledgment)
const uint8_t PKT_NACK      = 0x04; // Bidirectional (negative ack)
const uint8_t PKT_MESSAGE   = 0x05; // ESP32 -> PC (String messages)
const uint8_t PKT_ROUTE     = 0x06; // PC -> ESP32 (bulk waypoints) [F2]

// Master mode values
const uint8_t MASTER_DISCRETION = 0; // ESP32 decides (HID buttons)
const uint8_t MASTER_MANUAL = 1;     // Force manual mode
const uint8_t MASTER_AUTONOMOUS = 2; // Force autonomous mode

// Order values (only when MASTER_AUTONOMOUS)
const uint8_t ORDER_NONE = 0;
const uint8_t ORDER_WAYPOINT = 1;
const uint8_t ORDER_ORBIT = 2;

// Direction values (only for orbit)
const uint8_t DIR_CCW = 0; // Counter-clockwise
const uint8_t DIR_CW = 1;  // Clockwise

// Configuration packet structure (PC -> ESP32)
struct __attribute__((packed)) ConfigPacket {
  uint8_t masterMode; // 0=discretion, 1=manual, 2=autonomous
  uint8_t order;      // 0=none, 1=waypoint, 2=orbit
  float waypoint_lat; // Latitude (degrees)
  float waypoint_lon; // Longitude (degrees)
  float waypoint_alt; // Altitude (meters)
  uint8_t direction;  // 0=CCW, 1=CW
  float orbit_radius; // Radius (meters)
  float declination;  // [F7] Degrees
};

// ... (AircraftTelemetry remains same)

// Telemetry packet structure (ESP32 -> PC)
// ── Paquete recibido desde CADI_A (24 bytes, optimizado) ──
// Debe coincidir exactamente con TelemetryPacket en CADI_A.ino
struct __attribute__((packed)) AircraftTelemetry {
  float   latitude;     // 4B
  float   longitude;    // 4B
  int16_t altitude;     // 2B — metros × 10
  int16_t heading;      // 2B — grados × 10  (yaw, 0–3599)
  int16_t pitch;        // 2B — grados × 10
  int16_t roll;         // 2B — grados × 10
  int16_t gforce;       // 2B — G × 100
  int16_t velocityX;    // 2B — m/s × 100
  int16_t velocityY;    // 2B — m/s × 100
  int16_t velocityZ;    // 2B — m/s × 100
  uint8_t seq;          // 1B — Sequence [F3]
  uint8_t _pad[3];      // 3B — padding: 25→28 bytes (múltiplo de 4 para XXTEA)
};                      // Total: 28 bytes

// ── Paquete enviado al PC via serial (telemetría completa) ──
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
  uint8_t lossRate;     // 1B — 0-100% [F3]
};

// CRC16 calculation (CCITT)
uint16_t calculateCRC16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }

  return crc;
}

// Serial buffer for parsing
const size_t SERIAL_BUFFER_SIZE = 256;
uint8_t serialBuffer[SERIAL_BUFFER_SIZE];
size_t serialBufferIndex = 0;

// Last received configuration
ConfigPacket lastConfig = {MASTER_DISCRETION, ORDER_NONE, 0, 0, 0, DIR_CCW};
bool configReceived = false;

// ═══════════════════════════════════════════════════════
// SERIAL PROTOCOL FUNCTIONS
// ═══════════════════════════════════════════════════════

void sendACK() {
  const uint8_t type = PKT_ACK;
  const uint8_t len = 0;
  uint8_t packet[6];
  packet[0] = MAGIC_START;
  packet[1] = type;
  packet[2] = len;
  uint16_t crc = calculateCRC16(&packet[1], 2); // TYPE + LEN (no payload)
  packet[3] = (crc >> 8) & 0xFF;
  packet[4] = crc & 0xFF;
  packet[5] = MAGIC_END;
  Serial.write(packet, 6);
  Serial.flush();
}

void sendNACK() {
  const uint8_t type = PKT_NACK;
  const uint8_t len = 0;
  uint8_t packet[6];
  packet[0] = MAGIC_START;
  packet[1] = type;
  packet[2] = len;
  uint16_t crc = calculateCRC16(&packet[1], 2); // TYPE + LEN (no payload)
  packet[3] = (crc >> 8) & 0xFF;
  packet[4] = crc & 0xFF;
  packet[5] = MAGIC_END;
  Serial.write(packet, 6);
  Serial.flush();
}

bool sendTelemetryPacket(const TelemetryPacket &telem) {
  const size_t payloadSize = sizeof(TelemetryPacket);
  const size_t totalSize = 1 + 1 + 1 + payloadSize + 2 +
                           1; // START + TYPE + LEN + PAYLOAD + CRC + END

  uint8_t packet[totalSize];
  size_t idx = 0;

  // Header
  packet[idx++] = MAGIC_START;
  packet[idx++] = PKT_TELEMETRY;
  packet[idx++] = (uint8_t)payloadSize;

  // Payload — CRITICO: copiar antes de calcular CRC
  memcpy(&packet[idx], &telem, payloadSize);
  idx += payloadSize;

  // CRC16 (sobre TYPE + LENGTH + PAYLOAD)
  uint16_t crc = calculateCRC16(&packet[1], 1 + 1 + payloadSize);
  packet[idx++] = (crc >> 8) & 0xFF; // CRC high byte
  packet[idx++] = crc & 0xFF;        // CRC low byte

  // End marker
  packet[idx++] = MAGIC_END;

  // Send
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

bool parseConfigPacket(const uint8_t *payload, size_t length) {
  if (length != sizeof(ConfigPacket)) {
    return false;
  }
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
  
  // Forzar primer waypoint de la ruta si estamos en modo autónomo
  if (waypointCount > 0 && originEstablished) {
     targetWaypoint = GPSToLocal(waypointRoute[0].lat, waypointRoute[0].lon, waypointRoute[0].alt);
     // Si el primer modo es órbita, configurar también
     if (waypointRoute[0].mode == ORDER_ORBIT) {
        orbitCenter = targetWaypoint;
        orbitAltitude = waypointRoute[0].alt;
        orbitClockwise = (waypointRoute[0].direction == DIR_CW);
        orbitRadius = max(waypointRoute[0].radius, 10.0f);
        resetOrbitSystem();
     }
  }
  
  return true;
}

void processSerialPacket() {
  if (serialBufferIndex < 4)
    return; // Minimum packet size

  // Find magic start
  int startIdx = -1;
  for (size_t i = 0; i < serialBufferIndex; i++) {
    if (serialBuffer[i] == MAGIC_START) {
      startIdx = i;
      break;
    }
  }

  if (startIdx == -1) {
    serialBufferIndex = 0; // Garbage found, clear buffer
    return;
  } else if (startIdx > 0) {
    // Shift buffer
    memmove(serialBuffer, &serialBuffer[startIdx],
            serialBufferIndex - startIdx);
    serialBufferIndex -= startIdx;
  }

  if (serialBufferIndex < 4)
    return;

  uint8_t packetType = serialBuffer[1];
  uint8_t length = serialBuffer[2];
  size_t expectedSize =
      1 + 1 + 1 + length + 2 + 1; // START + TYPE + LEN + PAYLOAD + CRC + END

  if (expectedSize > SERIAL_BUFFER_SIZE) {
    serialBufferIndex = 0; // Prevent buffer overflow
    return;
  }

  if (serialBufferIndex < expectedSize)
    return; // Wait for more data

  // Verify end marker
  if (serialBuffer[expectedSize - 1] != MAGIC_END) {
    sendNACK();
    serialBufferIndex = 0;
    return;
  }

  // Verify CRC
  uint16_t receivedCRC =
      ((uint16_t)serialBuffer[3 + length] << 8) | serialBuffer[3 + length + 1];
  uint16_t calculatedCRC = calculateCRC16(&serialBuffer[1], 1 + 1 + length);

  if (receivedCRC != calculatedCRC) {
    sendNACK();
    memmove(serialBuffer, &serialBuffer[expectedSize],
            serialBufferIndex - expectedSize);
    serialBufferIndex -= expectedSize;
    return;
  }

  // Process packet based on type
  bool success = false;
  switch (packetType) {
  case PKT_CONFIG:
    success = parseConfigPacket(&serialBuffer[3], length);
    break;

  case PKT_ACK:
    // ACK received (handled in sendTelemetryPacket)
    success = true;
    break;

  case PKT_NACK:
    success = true;
    break;

  case PKT_ROUTE:
    success = parseRoutePacket(&serialBuffer[3], length);
    break;

  default:
    break;
  }

  // Send response
  if (packetType == PKT_CONFIG) {
    if (success) {
      sendACK();
    } else {
      sendNACK();
    }
  }

  // Remove processed packet from buffer
  if (serialBufferIndex >= expectedSize) {
    memmove(serialBuffer, &serialBuffer[expectedSize],
            serialBufferIndex - expectedSize);
    serialBufferIndex -= expectedSize;
  } else {
    serialBufferIndex = 0;
  }
}


// ═══════════════════════════════════════════════════════
// SET_KEY HANDLER [F5]
// Parsea comandos seriales de texto con formato:
//   SET_KEY:16characterkey
// Almacena la clave en NVS y reinicia el dispositivo.
// Protocolo: texto plano (no binario) para facilitar
// el pareado desde cualquier terminal serial.
// ═══════════════════════════════════════════════════════

static char setKeyLineBuf[32];
static uint8_t setKeyLineIdx = 0;

void handleSetKeyCommand(const char* line) {
  // Verificar prefijo SET_KEY:
  if (strncmp(line, "SET_KEY:", 8) != 0) return;

  const char* keyStr = line + 8;
  size_t keyLen = strlen(keyStr);

  // Rellenar con ceros o truncar a exactamente 16 bytes
  uint8_t keyBytes[16] = {0};
  size_t copyLen = keyLen < 16 ? keyLen : 16;
  memcpy(keyBytes, keyStr, copyLen);

  // Guardar en NVS
  Preferences prefs;
  prefs.begin("pairing", false); // read-write
  prefs.putBytes("shared_key", keyBytes, 16);
  prefs.end();

  // Confirmar y reiniciar
  sendTextMessage("[SEC] New key stored in NVS. Restarting...");
  delay(200);
  ESP.restart();
}

void parseSetKeySerial() {
  // Lee Serial byte a byte buscando lineas de texto (terminadas en '\n')
  // Solo actua si el byte NO es el magic binario (0xAA),
  // evitando conflicto con el parser binario.
  while (Serial.available()) {
    char ch = (char)Serial.peek();
    // Si el byte es el magic binario, dejar al parser binario manejarlo
    if ((uint8_t)ch == MAGIC_START) break;
    Serial.read(); // consumir
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
  // Parsear comandos de texto SET_KEY antes del parser binario
  parseSetKeySerial();

  // Read available serial data
  while (Serial.available() && serialBufferIndex < SERIAL_BUFFER_SIZE) {
    serialBuffer[serialBufferIndex++] = Serial.read();
  }

  // Process complete packets
  if (serialBufferIndex > 0) {
    processSerialPacket();
  }

  // Clear buffer if it's full (corrupted data)
  if (serialBufferIndex >= SERIAL_BUFFER_SIZE) {
    serialBufferIndex = 0;
  }
}

// [Moved applySerialConfiguration to bottom]

// ═══════════════════════════════════════════════════════
// VECTORS CLASS
// ═══════════════════════════════════════════════════════

// Vectors class moved to top

// ═══════════════════════════════════════════════════════
// TELEMETRY
// ═══════════════════════════════════════════════════════

// GPS Data
float latitude  = 0.0f;  // Degrees
float longitude = 0.0f;  // Degrees
float altitude  = 0.0f;  // Meters

// Orientation — recibidos desde Kalman en CADI_A
float pitch = 0.0f;  // Degrees
float roll  = 0.0f;  // Degrees
float yaw   = 0.0f;  // Degrees (0-360)

// System State
float gforce = 0.0f;  // G-Force

// nRF24L01 Configuration (Moved to top)

// ═══════════════════════════════════════════════════════
// SEGURIDAD DE COMUNICACIONES (XXTEA)
// ═══════════════════════════════════════════════════════
// ⚠ SEGURIDAD: Clave XXTEA hardcodeada — SOLO PARA PROTOTIPO.
// En producción, derivar de ESP.getEfuseMac() y almacenar en NVS cifrado.
// Cualquier persona con acceso al binario puede extraer esta clave.
uint32_t sharedKey[4] = { 0x58444630, 0x30314B45, 0x595F3230, 0x32362121 }; // "XDF001KEY_2026!!" en hex
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

// ── Headers Seguros ──
const uint8_t MAGIC_CMD   = 0xAA;
const uint8_t MAGIC_TELEM = 0xBB;

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
  int16_t  declinationX10; // [F7] Replaces padding
  uint8_t  _pad[2];         // padding to 24 bytes (multiple of 4 for XXTEA)
}; // Total: 24 bytes (6 words) // 24 bytes

struct __attribute__((packed)) SecureTelemetry {
  uint8_t  magic;
  uint8_t  seq;
  uint16_t crc;
  AircraftTelemetry telem;
}; // 28 bytes

uint8_t cmdSeq = 0;

uint16_t calculateRadioCRC16(const uint8_t *data, size_t length) {
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

// Payload Structures
int16_t        commands[4];
AircraftTelemetry aircraftTelem;

// HID Configuration
// (myControllers moved to top)

// ═══════════════════════════════════════════════════════
// NAVIGATION SYSTEM
// ═══════════════════════════════════════════════════════

// (Navigation variables moved to top)
// (orbitRadius, orbitAltitude, orbitClockwise moved to top)

struct AircraftState {
  Vector3D position;
  Vector3D velocity;  // m/s — recibido directamente desde CADI_A (Kalman en avión)
  float pitch;        // grados — desde Kalman de actitud en CADI_A
  float roll;         // grados — desde Kalman de actitud en CADI_A
  float yaw;          // grados — desde Kalman de actitud en CADI_A
};

AircraftState aircraft;

struct ControlCommands {
  int yaw;
  int throttle;
  int pitch;
  int roll;
};

// ═══════════════════════════════════════════════════════
// COORDINATE CONVERSION
// ═══════════════════════════════════════════════════════

void setOrigin(float lat, float lon, float alt) {
  if (!originEstablished) {
    gpsOrigin.x = lat;
    gpsOrigin.y = lon;
    gpsOrigin.z = alt;
    originEstablished = true;
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
// WAYPOINT NAVIGATION SYSTEM
// Algoritmo vectorial con L1, clamp adaptativo y captura en dos etapas
// ═══════════════════════════════════════════════════════

// ── Constantes de navegación ─────────────────────────
const float STALL_SPEED       = 5.0f;    // m/s — velocidad mínima de control
const float KP_ROLL           = 0.6f;    // ganancia proporcional roll
const float KI_ROLL           = 0.05f;   // ganancia integral mitigación viento
const float MAX_WIND_COMP     = 15.0f;   // grados máximos mitigación de viento cruzado
const float KP_PITCH          = 0.5f;    // ganancia proporcional pitch
const float KP_YAW            = 0.3f;    // ganancia proporcional yaw (suave)
const float K_L1              = 3.0f;    // multiplicador L1: L1_dist = K_L1 * |V|
const float THROTTLE_BASE     = 100.0f;  // throttle crucero
const float THROTTLE_STALL    = 180.0f;  // throttle máximo en recuperación
const float WP_CAPTURE_NEAR   = 8.0f;    // metros — confirmación de llegada
const float WP_CAPTURE_FAR    = 15.0f;   // metros — zona de desaceleración
const float ANTIPARALLEL_DEG  = 150.0f;  // umbral zona antiparalela (grados)
const float CROSS_DEADZONE    = 0.05f;   // zona muerta producto vectorial

// ── Estado persistente entre ciclos ──────────────────
static int  lastTurnSign      = 1;       // último signo de giro válido

// ─────────────────────────────────────────────────────
//  NavigationResult — resultado del análisis vectorial
// ─────────────────────────────────────────────────────
struct NavigationResult {
  float horizontalAngle;   // θ_h en grados — error lateral
  float verticalAngle;     // θ_v en grados — error de altitud
  float distance;          // |X| metros — distancia total al waypoint
  float horizontalDist;    // |X_h| metros — distancia horizontal
  int   turnSign;          // +1 derecha / -1 izquierda
  float l1Factor;          // factor L1 [0..1] — autoridad proporcional a distancia
  bool  nearCapture;       // dentro de WP_CAPTURE_NEAR
  bool  farCapture;        // dentro de WP_CAPTURE_FAR
  bool  stallDetected;     // velocidad por debajo de STALL_SPEED
};

// ─────────────────────────────────────────────────────
//  analyzeNavigation
//  Calcula todos los parámetros vectoriales del estado
//  de navegación dado el estado del avión y el target.
// ─────────────────────────────────────────────────────
NavigationResult analyzeNavigation(AircraftState state, Vector3D target) {
  NavigationResult result;

  // ── 1. Vectores base ─────────────────────────────
  Vector3D X   = target - state.position;          // error 3D
  Vector3D V_h(state.velocity.x, state.velocity.y, 0.0f);
  Vector3D X_h(X.x, X.y, 0.0f);

  result.distance       = X.magnitude();
  result.horizontalDist = X_h.magnitude();

  // ── 2. Guard: velocidad de pérdida ───────────────
  float speed = V_h.magnitude();
  result.stallDetected = (speed < STALL_SPEED);

  // ── 3. Guard: waypoint horizontalmente muy cercano
  //    (X_h ~ 0 → θ_h indefinido, solo hay error vertical)
  if (result.horizontalDist < 0.01f) {
    result.horizontalAngle = 0.0f;
    result.verticalAngle   = (X.z > 0.0f) ? 90.0f : -90.0f;
    result.turnSign        = lastTurnSign;
    result.l1Factor        = 0.0f;
    result.nearCapture     = (result.distance < WP_CAPTURE_NEAR);
    result.farCapture      = (result.distance < WP_CAPTURE_FAR);
    return result;
  }

  // ── 4. Error horizontal θ_h ──────────────────────
  //    Producto punto entre velocidades horizontales normalizadas
  float dotH = V_h.normalize().dot(X_h.normalize());
  dotH = constrain(dotH, -1.0f, 1.0f);
  result.horizontalAngle = acosf(dotH) * 180.0f / PI;  // grados

  // ── 5. Error vertical θ_v ────────────────────────
  //    atan2 preserva signo y maneja X_h ~ 0 sin división por cero
  result.verticalAngle = atan2f(X.z, result.horizontalDist) * 180.0f / PI;

  // ── 6. Sentido de giro — producto vectorial + zona muerta ──
  //    Se usa V completo (no solo V_h) para mayor estabilidad
  Vector3D C = state.velocity.cross(X);

  if (result.horizontalAngle > ANTIPARALLEL_DEG) {
    // Zona antiparalela: avión casi mirando al revés del waypoint
    // C.z puede ser muy pequeño e inestable → priorizar signo anterior
    if (fabsf(C.z) < CROSS_DEADZONE) {
      result.turnSign = 1;            // colapso total → forzar derecha
    } else {
      result.turnSign = (C.z > 0.0f) ? -1 : 1;
    }
  } else {
    // Zona normal
    if (fabsf(C.z) < CROSS_DEADZONE) {
      result.turnSign = lastTurnSign; // zona muerta → mantener signo anterior
    } else {
      result.turnSign = (C.z > 0.0f) ? -1 : 1;
      lastTurnSign    = result.turnSign;
    }
  }

  // ── 7. Factor L1 ─────────────────────────────────
  //    Reduce autoridad cuando el avión está cerca del waypoint.
  //    L1_dist = K_L1 * |V| — crece con la velocidad
  float L1_dist    = K_L1 * speed;
  L1_dist          = max(L1_dist, 5.0f);           // mínimo 5m para evitar /0
  result.l1Factor  = constrain(result.horizontalDist / L1_dist, 0.0f, 1.0f);

  // ── 8. Zonas de captura ──────────────────────────
  result.nearCapture = (result.distance < WP_CAPTURE_NEAR);
  result.farCapture  = (result.distance < WP_CAPTURE_FAR);

  return result;
}

// ─────────────────────────────────────────────────────
//  generateWaypointCommands
//  Convierte el NavigationResult en comandos de servo.
//  currentRoll: roll real del avión (del Kalman en CADI_A),
//               usado para pitch adaptativo y throttle coordinado.
// ─────────────────────────────────────────────────────
ControlCommands generateWaypointCommands(NavigationResult nav, float currentRoll) {
  ControlCommands cmd;

  // ── Guard: pérdida de velocidad ──────────────────
  if (nav.stallDetected) {
    cmd.roll     = 0;
    cmd.pitch    = 0;
    cmd.yaw      = 0;
    cmd.throttle = (int)THROTTLE_STALL;
    return cmd;
  }

  // ── Guard: waypoint alcanzado (etapa cercana) ────
  if (nav.nearCapture) {
    cmd.roll     = 0;
    cmd.pitch    = 0;
    cmd.yaw      = 0;
    cmd.throttle = (int)THROTTLE_BASE;
    return cmd;
  }

  // ── Zona de desaceleración (etapa lejana) ────────
  //    Factor de suavizado: 0.5 entre 8m y 15m, 1.0 fuera de 15m
  float captureFactor = nav.farCapture ? 0.5f : 1.0f;

  // ── Roll (Con Integrador de Viento Cruzado) ──────
  //    θ_h * KP_ROLL * factor_L1 * factor_captura * signo
  static float rollIntegral = 0.0f;
  static bool wasNearCapture = false;

  // Detectar transición: si el ciclo anterior fue nearCapture
  // y ahora ya no lo es, significa cambio de waypoint → reset integrador
  if (wasNearCapture && !nav.nearCapture) {
    rollIntegral = 0.0f;
  }
  wasNearCapture = nav.nearCapture;

  if (!nav.nearCapture) {
    // Calculamos el error direccional puro
    float errorCmd = nav.horizontalAngle * (float)nav.turnSign;
    // Acumulamos (dt base asume ~20Hz, 0.05s)
    rollIntegral += errorCmd * KI_ROLL * 0.05f;
    rollIntegral = constrain(rollIntegral, -MAX_WIND_COMP, MAX_WIND_COMP);
  }

  float proportionalRoll = nav.horizontalAngle * KP_ROLL * (float)nav.turnSign;
  float rollCmd = (proportionalRoll + rollIntegral) * nav.l1Factor * captureFactor;
  cmd.roll = (int)constrain(rollCmd, -45.0f, 45.0f);

  // ── Pitch adaptativo ─────────────────────────────
  //    Usa el roll REAL del avión (Kalman CADI_A), no el comandado.
  //    El avión tarda varios ciclos en ejecutar el roll comandado,
  //    por lo que el pitch debe basarse en el estado real actual.
  //    pitch_max = 20° a roll=0°, 10° a roll=45°, 0° a roll=90°
  float rollRad   = fabsf(currentRoll) * PI / 180.0f;
  float pitchMax  = 20.0f * (1.0f - 0.5f * constrain(rollRad / (PI / 2.0f), 0.0f, 1.0f));
  float pitchCmd  = nav.verticalAngle * KP_PITCH;
  cmd.pitch       = (int)constrain(pitchCmd, -pitchMax, pitchMax);

  // ── Yaw ──────────────────────────────────────────
  //    Corrección coordinada, misma dirección que el roll
  float yawCmd = nav.horizontalAngle * KP_YAW
               * nav.l1Factor
               * captureFactor
               * (float)nav.turnSign;
  cmd.yaw = (int)constrain(yawCmd, -40.0f, 40.0f);

  // ── Throttle con compensación de viraje ──────────
  //    Usa roll real para compensar pérdida de sustentación real.
  float cosRoll    = cosf(rollRad);
  float throttleF  = THROTTLE_BASE / max(cosRoll, 0.5f);
  if (cmd.pitch > 5)  throttleF += 15.0f;    // subida: más motor
  if (cmd.pitch < -5) throttleF -= 10.0f;    // bajada: menos motor
  cmd.throttle = (int)constrain(throttleF, 60.0f, 180.0f);

  return cmd;
}


// ═══════════════════════════════════════════════════════
// ORBIT NAVIGATION SYSTEM
// Mejoras vs original:
//   - PD sobre error de radio (+ término derivativo KD)
//   - Guard de stall en ALIGNMENT y MAINTENANCE
//   - Throttle coordinado base/cos(roll) en CAPTURE y MAINTENANCE
//   - Pitch adaptativo según roll en CAPTURE y MAINTENANCE
//   - ORBIT_YAW_TOLERANCE ampliada a 10° (magnetómetro con deriva)
//   - Timeout en ALIGNMENT: avanza a CAPTURE si supera 5s sin alinear
//   - orbitRadius desde ConfigPacket (fix bug hardcodeado)
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
  int        entryPointIndex;
  Vector3D   entryPoints[4];
  float      idealYaw;
  bool       initialized;
  unsigned long alignmentStartMs;  // timestamp inicio de ALIGNMENT (timeout)
};

OrbitState orbitState = {ORBIT_SEARCH_ENTRY, 0, {}, 0.0f, false, 0};

// ── Constantes de órbita ─────────────────────────────
const float KP_ORBIT_RADIUS   = 0.8f;   // ganancia proporcional — error de radio
const float KD_ORBIT_RADIUS   = 0.3f;   // ganancia derivativa   — amortigua oscilaciones
const float KP_ORBIT_ALTITUDE = 0.4f;   // corrección de altitud
const float KP_ORBIT_YAW      = 1.2f;   // corrección yaw en alignment
const float ORBIT_BASE_ROLL   = 20.0f;  // roll base de crucero (grados)

const float ORBIT_APPROACH_DIST  = 25.0f;  // metros — approach → alignment
const float ORBIT_ALIGNMENT_DIST = 15.0f;  // metros — alignment → capture
const float ORBIT_CAPTURE_DIST   =  8.0f;  // metros — capture  → maintenance
const float ORBIT_YAW_TOLERANCE  = 10.0f;  // grados — ampliado vs 5° original
                                            // compensa deriva del magnetómetro MPU-9250
const unsigned long ORBIT_ALIGNMENT_TIMEOUT_MS = 5000; // 5s timeout alignment

// Throttle base en órbita — igual que waypoint para transición suave
const float ORBIT_THROTTLE_BASE = 100.0f;

// Estado derivativo del radio — persiste entre ciclos
static float lastRadiusError = 0.0f;

// ─────────────────────────────────────────────────────
//  orbitThrottleCoordinated
//  Reemplaza el throttle=55 fijo del original.
//  Aplica la misma lógica de waypoint: base/cos(roll)
//  para compensar pérdida de sustentación en viraje.
// ─────────────────────────────────────────────────────
int orbitThrottleCoordinated(int rollCmd, int pitchCmd) {
  float rollRad   = fabsf((float)rollCmd) * PI / 180.0f;
  float throttleF = ORBIT_THROTTLE_BASE / max(cosf(rollRad), 0.5f);
  if (pitchCmd >  5) throttleF += 15.0f;
  if (pitchCmd < -5) throttleF -= 10.0f;
  return (int)constrain(throttleF, 60.0f, 180.0f);
}

// ─────────────────────────────────────────────────────
//  orbitPitchAdaptive
//  Aplica el mismo pitch adaptativo de waypoint:
//  pitch_max = 20° a roll=0°, 10° a roll=45°, ~0° a roll=90°
// ─────────────────────────────────────────────────────
int orbitPitchAdaptive(float altError, int rollCmd) {
  float rollRad  = fabsf((float)rollCmd) * PI / 180.0f;
  float pitchMax = 20.0f * (1.0f - 0.5f * constrain(rollRad / (PI / 2.0f), 0.0f, 1.0f));
  float pitchCmd = -altError * KP_ORBIT_ALTITUDE;
  return (int)constrain(pitchCmd, -pitchMax, pitchMax);
}

// ─────────────────────────────────────────────────────
//  radiusPD
//  Corrección de roll basada en error de radio con
//  término derivativo para amortiguar oscilaciones.
//  Positivo = avión afuera del radio → más roll hacia adentro
//  Negativo = avión adentro del radio → menos roll
// ─────────────────────────────────────────────────────
float radiusPD(float radiusError) {
  float derivative   = radiusError - lastRadiusError;
  lastRadiusError    = radiusError;
  return KP_ORBIT_RADIUS * radiusError + KD_ORBIT_RADIUS * derivative;
}

// ─────────────────────────────────────────────────────
//  initializeOrbit
// ─────────────────────────────────────────────────────
void initializeOrbit() {
  if (orbitState.initialized) return;

  orbitState.entryPoints[0] = Vector3D(orbitCenter.x,               orbitCenter.y + orbitRadius, orbitAltitude);
  orbitState.entryPoints[1] = Vector3D(orbitCenter.x + orbitRadius,  orbitCenter.y,               orbitAltitude);
  orbitState.entryPoints[2] = Vector3D(orbitCenter.x,               orbitCenter.y - orbitRadius, orbitAltitude);
  orbitState.entryPoints[3] = Vector3D(orbitCenter.x - orbitRadius,  orbitCenter.y,               orbitAltitude);

  orbitState.phase           = ORBIT_SEARCH_ENTRY;
  orbitState.initialized     = true;
  orbitState.alignmentStartMs= 0;
  lastRadiusError            = 0.0f;
}

// ─────────────────────────────────────────────────────
//  Utilidades angulares
// ─────────────────────────────────────────────────────
float normalizeAngle360(float angle) {
  while (angle <    0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

float calculateYawError(float current, float desired) {
  current = normalizeAngle360(current);
  desired = normalizeAngle360(desired);
  float error = desired - current;
  if (error >  180.0f) error -= 360.0f;
  if (error < -180.0f) error += 360.0f;
  return error;
}

float calculateIdealEntryYaw(int entryPoint) {
  if (orbitClockwise) {
    switch (entryPoint) {
      case 0: return  90.0f;
      case 1: return 180.0f;
      case 2: return 270.0f;
      case 3: return   0.0f;
    }
  } else {
    switch (entryPoint) {
      case 0: return 270.0f;
      case 1: return   0.0f;
      case 2: return  90.0f;
      case 3: return 180.0f;
    }
  }
  return 0.0f;
}

// ─────────────────────────────────────────────────────
//  findBestEntryPoint
//  Score = alineación velocidad-waypoint − penalización distancia.
// ─────────────────────────────────────────────────────
int findBestEntryPoint(AircraftState state) {
  float bestScore = -999999.0f;
  int   bestPoint = 0;
  Vector3D dirVel = Vector3D(state.velocity.x, state.velocity.y, 0.0f).normalize();

  for (int i = 0; i < 4; i++) {
    Vector3D toWP  = orbitState.entryPoints[i] - state.position;
    float    dist  = toWP.magnitude();
    float    align = dirVel.dot(toWP.normalize());
    float    score = align * 100.0f - dist * 0.1f;
    if (score > bestScore) {
      bestScore = score;
      bestPoint = i;
    }
  }
  return bestPoint;
}

// ─────────────────────────────────────────────────────
//  calculateRadiusError
//  + = afuera del radio objetivo
//  - = adentro del radio objetivo
// ─────────────────────────────────────────────────────
float calculateRadiusError(Vector3D position) {
  Vector3D toCenter(position.x - orbitCenter.x,
                    position.y - orbitCenter.y, 0.0f);
  return toCenter.magnitude() - orbitRadius;
}

// ─────────────────────────────────────────────────────
//  generateOrbitCommands
// ─────────────────────────────────────────────────────
ControlCommands generateOrbitCommands(AircraftState state) {
  ControlCommands cmd;

  if (!orbitState.initialized) initializeOrbit();

  // ── Guard stall — todas las fases ────────────────
  float speedH = Vector3D(state.velocity.x, state.velocity.y, 0.0f).magnitude();
  if (speedH < STALL_SPEED) {
    cmd.roll     = 0;
    cmd.pitch    = 0;
    cmd.yaw      = 0;
    cmd.throttle = (int)THROTTLE_STALL;
    return cmd;
  }

  switch (orbitState.phase) {

  // ── SEARCH_ENTRY ─────────────────────────────────
  case ORBIT_SEARCH_ENTRY: {
    orbitState.entryPointIndex  = findBestEntryPoint(state);
    orbitState.idealYaw         = calculateIdealEntryYaw(orbitState.entryPointIndex);
    orbitState.phase            = ORBIT_APPROACH;
    break;
  }

  // ── APPROACH ─────────────────────────────────────
  // Navega al punto de entrada usando el sistema waypoint completo.
  // Hereda: L1, stall guard, pitch adaptativo, throttle coordinado.
  case ORBIT_APPROACH: {
    Vector3D target = orbitState.entryPoints[orbitState.entryPointIndex];
    NavigationResult nav = analyzeNavigation(state, target);
    cmd = generateWaypointCommands(nav, state.roll);
    if (nav.distance < ORBIT_APPROACH_DIST) {
      orbitState.phase            = ORBIT_ALIGNMENT;
      orbitState.alignmentStartMs = millis();
    }
    return cmd;
  }

  // ── ALIGNMENT ────────────────────────────────────
  // Corrige yaw hasta quedar tangencial al círculo.
  // Mejoras: guard stall heredado, tolerancia ampliada,
  //          timeout de 5s para evitar bloqueo por deriva magnética.
  case ORBIT_ALIGNMENT: {
    float yawError = calculateYawError(state.yaw, orbitState.idealYaw);
    float altError = orbitAltitude - state.position.z;

    cmd.roll  = 0;
    cmd.yaw   = (int)constrain(yawError * KP_ORBIT_YAW, -30.0f, 30.0f);
    cmd.pitch = (int)constrain(altError * KP_ORBIT_ALTITUDE, -20.0f, 20.0f);
    cmd.throttle = orbitThrottleCoordinated(cmd.roll, cmd.pitch);

    Vector3D target  = orbitState.entryPoints[orbitState.entryPointIndex];
    float    dist    = (target - state.position).magnitude();
    bool     aligned = fabsf(yawError) < ORBIT_YAW_TOLERANCE;
    bool     close   = dist < ORBIT_ALIGNMENT_DIST;
    bool     timeout = (millis() - orbitState.alignmentStartMs) > ORBIT_ALIGNMENT_TIMEOUT_MS;

    if ((aligned && close) || timeout) {
      orbitState.phase = ORBIT_CAPTURE;
      lastRadiusError  = 0.0f;  // reset derivativo al entrar en captura
    }
    return cmd;
  }

  // ── CAPTURE ──────────────────────────────────────
  // Blending entre navegación waypoint (navCmd) y órbita pura (orbitCmd).
  // factor 0 = puro waypoint (lejos del radio), factor 1 = pura órbita.
  // Mejoras: throttle coordinado, pitch adaptativo, PD sobre radio.
  case ORBIT_CAPTURE: {
    float radiusError = calculateRadiusError(state.position);
    float altError    = state.position.z - orbitAltitude;

    // NavCmd: navegación waypoint al punto de entrada
    Vector3D target = orbitState.entryPoints[orbitState.entryPointIndex];
    NavigationResult nav = analyzeNavigation(state, target);
    ControlCommands  navCmd = generateWaypointCommands(nav, state.roll);

    // OrbitCmd: órbita pura con PD de radio y pitch adaptativo
    float rollCorrection = radiusPD(radiusError);
    float rollTarget     = orbitClockwise ? -ORBIT_BASE_ROLL : ORBIT_BASE_ROLL;
    int   orbitRoll      = (int)constrain(rollTarget + rollCorrection, -45.0f, 45.0f);
    int   orbitPitch     = orbitPitchAdaptive(altError, orbitRoll);
    int   orbitThrottle  = orbitThrottleCoordinated(orbitRoll, orbitPitch);

    // Blending: factor crece de 0→1 conforme el radio converge
    float factor = 1.0f - constrain(fabsf(radiusError) / ORBIT_CAPTURE_DIST, 0.0f, 1.0f);

    cmd.roll     = (int)(navCmd.roll     * (1.0f - factor) + orbitRoll     * factor);
    cmd.pitch    = (int)(navCmd.pitch    * (1.0f - factor) + orbitPitch    * factor);
    cmd.throttle = (int)(navCmd.throttle * (1.0f - factor) + orbitThrottle * factor);
    cmd.yaw      = 0;

    if (fabsf(radiusError) < ORBIT_CAPTURE_DIST) {
      orbitState.phase = ORBIT_MAINTENANCE;
    }
    return cmd;
  }

  // ── MAINTENANCE ──────────────────────────────────
  // Mantiene radio y altitud indefinidamente.
  // Mejoras: PD sobre radio, pitch adaptativo, throttle coordinado.
  case ORBIT_MAINTENANCE: {
    float radiusError = calculateRadiusError(state.position);
    float altError    = state.position.z - orbitAltitude;

    float rollCorrection = radiusPD(radiusError);
    float rollTarget     = orbitClockwise ? -ORBIT_BASE_ROLL : ORBIT_BASE_ROLL;
    cmd.roll  = (int)constrain(rollTarget + rollCorrection, -45.0f, 45.0f);
    cmd.pitch = orbitPitchAdaptive(altError, cmd.roll);
    cmd.throttle = orbitThrottleCoordinated(cmd.roll, cmd.pitch);
    cmd.yaw   = 0;
    return cmd;
  }

  } // switch
  return cmd;
}

void resetOrbitSystem() {
  orbitState.phase       = ORBIT_SEARCH_ENTRY;
  orbitState.initialized = false;
  lastRadiusError        = 0.0f;
}

// HID CALLBACKS
// ═══════════════════════════════════════════════════════

void onConnectedController(ControllerPtr ctl) { myControllers[0] = ctl; }

void onDisconnectedController(ControllerPtr ctl) { myControllers[0] = nullptr; }

// ═══════════════════════════════════════════════════════
// MANUAL PROTOTYPES TO FIX ARDUINO IDE BUILDER
// ═══════════════════════════════════════════════════════
Vector3D GPSToLocal(float lat, float lon, float alt);
NavigationResult analyzeNavigation(AircraftState state, Vector3D target);
ControlCommands generateWaypointCommands(NavigationResult nav, float currentRoll);
int findBestEntryPoint(AircraftState state);
float calculateRadiusError(Vector3D position);
ControlCommands generateOrbitCommands(AircraftState state);

// ═══════════════════════════════════════════════════════
// SERIAL CONFIGURATION
// ═══════════════════════════════════════════════════════

void applySerialConfiguration() {
  if (!configReceived)
    return;

  static uint8_t lastMasterMode = MASTER_DISCRETION;
  static uint8_t lastOrder = ORDER_NONE;

  // Check if configuration changed
  bool modeChanged = (lastConfig.masterMode != lastMasterMode);
  bool orderChanged = (lastConfig.order != lastOrder);

  if (!modeChanged && !orderChanged)
    return;

  // Apply master mode
  switch (lastConfig.masterMode) {
  case MASTER_DISCRETION:
    // Do nothing - HID buttons control mode
    break;

  case MASTER_MANUAL:
    if (modeChanged) {
      currentMode = MODE_MANUAL;
      resetOrbitSystem();
    }
    break;

  case MASTER_AUTONOMOUS:
    if (modeChanged || orderChanged) {
      // Convert GPS coordinates to local waypoint
      if (originEstablished) {
        Vector3D localWaypoint =
            GPSToLocal(lastConfig.waypoint_lat, lastConfig.waypoint_lon,
                       lastConfig.waypoint_alt);

        switch (lastConfig.order) {
        case ORDER_WAYPOINT:
          targetWaypoint = localWaypoint;
          currentMode = MODE_WAYPOINT;
          break;

        case ORDER_ORBIT:
          orbitCenter    = localWaypoint;
          orbitAltitude  = lastConfig.waypoint_alt;
          orbitClockwise = (lastConfig.direction == DIR_CW);
          orbitRadius    = max(lastConfig.orbit_radius, 10.0f); // mínimo 10m
          resetOrbitSystem();
          currentMode = MODE_ORBIT;
          break;

        default:
          break;
        }
      } else {
      }
    }
    break;
  }

  lastMasterMode = lastConfig.masterMode;
  lastOrder = lastConfig.order;
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

  if (!radio.begin()) {
    while (1)
      ;
  }

  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(3, 5);
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();

  // ── Cargado de Clave NVS (F5) ──────────────────
  preferences.begin("pairing", true); // Modo read-only
  if (preferences.isKey("shared_key")) {
     uint8_t nvsKey[16];
     preferences.getBytes("shared_key", nvsKey, 16);
     memcpy(sharedKey, nvsKey, 16);
  }
  preferences.end();
}

// ═══════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════

void loop() {
  // Update serial parser (non-blocking)
  updateSerialParser();

  // Apply serial configuration if received
  applySerialConfiguration();

  // ── GEOFENCE (F1) ────────────────────────────────
  if (originEstablished) {
    float hDist = sqrtf(aircraft.position.x * aircraft.position.x + aircraft.position.y * aircraft.position.y);
    if (hDist > 1800.0f) {
       if (currentMode != MODE_WAYPOINT || targetWaypoint.magnitude() > 1.0f) {
         currentMode = MODE_WAYPOINT;
         targetWaypoint = Vector3D(0, 0, orbitAltitude); // Regreso al origen
       }
    }
  }

  bool dataUpdated = BP32.update();

  // MODE SWITCHING (only if in DISCRETION mode)
  static unsigned long lastBtnTime = 0;
  if (dataUpdated && myControllers[0] && myControllers[0]->isConnected()) {
    if (millis() - lastBtnTime > 300) {
      if (lastConfig.masterMode == MASTER_DISCRETION) {
        if (myControllers[0]->a()) {
          currentMode = MODE_WAYPOINT;
          lastBtnTime = millis();
        } else if (myControllers[0]->b()) {
          currentMode = MODE_MANUAL;
          resetOrbitSystem();
          lastBtnTime = millis();
        } else if (myControllers[0]->x()) {
          currentMode = MODE_ORBIT;
          resetOrbitSystem();
          lastBtnTime = millis();
        }
      }
    }
  }

  // COMMAND GENERATION
  if (dataUpdated && myControllers[0] && myControllers[0]->isConnected()) {
    ControlCommands cmd;

    switch (currentMode) {
    case MODE_WAYPOINT: {
      NavigationResult nav = analyzeNavigation(aircraft, targetWaypoint);
      
      // AVANCE DE WAYPOINT (F2)
      if (nav.nearCapture && waypointCount > 0) {
          waypointIndex++;
          if (waypointIndex >= waypointCount) {
             waypointIndex = routeLoop ? 0 : waypointCount - 1;
          }
          
          RouteWaypoint nextWP = waypointRoute[waypointIndex];
          targetWaypoint = GPSToLocal(nextWP.lat, nextWP.lon, nextWP.alt);
          
          if (nextWP.mode == ORDER_ORBIT) {
             orbitCenter   = targetWaypoint;
             orbitAltitude = nextWP.alt;
             orbitClockwise= (nextWP.direction == DIR_CW);
             orbitRadius   = max(nextWP.radius, 10.0f);
             resetOrbitSystem();
             currentMode = MODE_ORBIT;
          }
          // Recalcular nav para el nuevo waypoint
          nav = analyzeNavigation(aircraft, targetWaypoint);
      }
      
      cmd = generateWaypointCommands(nav, aircraft.roll);
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

    SecureCommand secCmd;
    secCmd.magic = MAGIC_CMD;
    secCmd.seq   = cmdSeq++;
          secCmd.targetYaw      = commands[0];
          secCmd.targetThrottle = commands[1];
          secCmd.targetPitch    = commands[2];
          secCmd.targetRoll     = commands[3];
          secCmd.declinationX10 = (int16_t)(lastConfig.declination * 10.0f); // [F7]
    
    secCmd.homeLat = originEstablished ? gpsOrigin.x : 0.0f;
    secCmd.homeLon = originEstablished ? gpsOrigin.y : 0.0f;
    // secCmd.padding eliminado — reemplazado por declinationX10 [F7]

    // ══ ORDEN CRÍTICO: NO REORDENAR ══════════════════
    // Paso 1: CRC sobre datos en claro (desde targetRoll hasta el final)
    secCmd.crc = calculateRadioCRC16((uint8_t*)&secCmd.targetRoll, 20); 
    // Paso 2: Cifrar paquete completo (CRC incluido)
    btea((uint32_t*)&secCmd, 6); // Encrypt 6 words (24 bytes)

    // TRANSMISSION
    radio.stopListening();
    bool ok = radio.write(&secCmd, sizeof(SecureCommand));

    if (ok) {
      if (radio.isAckPayloadAvailable()) {
        uint8_t len = radio.getDynamicPayloadSize();
        if (len == sizeof(SecureTelemetry)) {
          SecureTelemetry secTelem;
          radio.read(&secTelem, sizeof(SecureTelemetry));
          btea((uint32_t*)&secTelem, -8); // Decrypt 32 bytes (SecureTelemetry)
          
          uint16_t crc = calculateRadioCRC16((uint8_t*)&secTelem.telem, sizeof(AircraftTelemetry));
          if (secTelem.magic == MAGIC_TELEM && secTelem.crc == crc) {
            aircraftTelem = secTelem.telem;

            // ── Decodificar int16 → float ───────────────
            latitude  = aircraftTelem.latitude;
            longitude = aircraftTelem.longitude;
            altitude  = aircraftTelem.altitude  / 10.0f;
            yaw       = aircraftTelem.heading   / 10.0f;
            pitch     = aircraftTelem.pitch     / 10.0f;
            roll      = aircraftTelem.roll      / 10.0f;
            gforce    = aircraftTelem.gforce    / 100.0f;

            // ── CÁLCULO DE CALIDAD DE ENLACE (F3) ────────
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
            // Reset periódico para estadísticas locales
            if (packetTotalCount > 1000) {
               packetTotalCount = 0;
               packetLossCount = 0;
            }

        // ── Velocidad directamente desde CADI_A ─────
        // Eliminado: calculateVelocity() — ya no se calcula aquí
        float vx = aircraftTelem.velocityX / 100.0f;
        float vy = aircraftTelem.velocityY / 100.0f;
        float vz = aircraftTelem.velocityZ / 100.0f;

        // ── Actualizar estado del avión ──────────────
        if (!originEstablished) {
          setOrigin(latitude, longitude, altitude);
        }

        aircraft.position = GPSToLocal(latitude, longitude, altitude);
        aircraft.velocity = Vector3D(vx, vy, vz);
        aircraft.pitch    = pitch;
        aircraft.roll     = roll;
        aircraft.yaw      = yaw;

        // ── Telemetría al PC via serial (cada 10 ciclos ~500ms) ──
        static uint8_t telemCounter = 0;
        if (++telemCounter >= 10) {
          telemCounter = 0;

          TelemetryPacket telemPkt;
          telemPkt.latitude     = latitude;
          telemPkt.longitude    = longitude;
          telemPkt.altitude     = altitude;
          telemPkt.yaw          = yaw;
          telemPkt.pitch        = pitch;
          telemPkt.roll         = roll;
          telemPkt.gforce       = gforce;
          telemPkt.velocity_mag = aircraft.velocity.magnitude();
          telemPkt.pos_local_x  = aircraft.position.x;
          telemPkt.pos_local_y  = aircraft.position.y;
          telemPkt.pos_local_z  = aircraft.position.z;
          telemPkt.currentMode  = (uint8_t)currentMode;
          telemPkt.cmd_yaw      = commands[0];
          telemPkt.cmd_throttle = commands[1];
          telemPkt.cmd_pitch    = commands[2];
          telemPkt.cmd_roll     = commands[3];
          telemPkt.lossRate     = currentLossRate; // [F3]

          sendTelemetryPacket(telemPkt);
        }
          }
        } else {
          // Flush invalid payload
          uint8_t dummy[32];
          radio.read(&dummy, len);
        }
      }
    }
  }

  delay(50); // Mantenemos el ciclo a 20Hz
}
