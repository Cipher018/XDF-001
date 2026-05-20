#include <mpu9250.h>
#include <Preferences.h> // [F5] NVS key storage
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>
#include <RF24.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <nRF24L01.h>

// ═══════════════════════════════════════════════════════
// TELEMETRY PACKET — OPTIMIZADO
// Tamaño: 4+4+2+2+2+2+2+2+2+2 = 24 bytes (límite nRF24: 32B)
// Conversiones para campos int16:
//   ángulos   × 10  → 0.1° resolución
//   gforce    × 100 → 0.01G resolución
//   velocidad × 100 → 0.01 m/s resolución
// ═══════════════════════════════════════════════════════
struct __attribute__((packed)) TelemetryPacket {
  float   latitude;     // 4B — float necesario (6 decimales GPS)
  float   longitude;    // 4B — float necesario (6 decimales GPS)
  int16_t altitude;     // 2B — metros × 10  (−3276m a +3276m, 0.1m res)
  int16_t heading;      // 2B — grados × 10  (0 a 3599 → 0° a 359.9°)
  int16_t pitch;        // 2B — grados × 10  (−900 a +900 → −90° a +90°)
  int16_t roll;         // 2B — grados × 10  (−1800 a +1800)
  int16_t gforce;       // 2B — G × 100      (0 a 800 → 0G a 8G)
  int16_t velocityX;    // 2B — m/s × 100    (−5000 a +5000 → ±50 m/s)
  int16_t velocityY;    // 2B — m/s × 100
  int16_t velocityZ;    // 2B — m/s × 100
  uint8_t seq;          // 1B — Contador incremental para detectar pérdida [F3]
  uint8_t _pad[3];      // 3B — padding: 25→28 bytes (múltiplo de 4 para XXTEA)
};                      // Total: 28 bytes ✓

TelemetryPacket telemPkt;

// ═══════════════════════════════════════════════════════
// DUBINS AND VECTOR CLASS
// ═══════════════════════════════════════════════════════
class Vector3D {
public:
  float x, y, z;
  Vector3D(float _x = 0, float _y = 0, float _z = 0) : x(_x), y(_y), z(_z) {}
  Vector3D operator-(const Vector3D &v) const { return Vector3D(x - v.x, y - v.y, z - v.z); }
  Vector3D operator+(const Vector3D &v) const { return Vector3D(x + v.x, y + v.y, z + v.z); }
  Vector3D operator*(float s) const { return Vector3D(x * s, y * s, z * s); }
  float magnitude() const { return sqrtf(x * x + y * y + z * z); }
  Vector3D normalize() const {
    float mag = magnitude();
    if (mag < 0.001f) return Vector3D(0, 0, 0);
    return Vector3D(x / mag, y / mag, z / mag);
  }
  float dot(const Vector3D &v) const { return x * v.x + y * v.y + z * v.z; }
  Vector3D cross(const Vector3D &v) const {
    return Vector3D(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }
};

// ── Global Mission State ──
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

// ═══════════════════════════════════════════════════════
// KALMAN FILTER — ACTITUD (pitch, roll, yaw)
// Modelo de 2 estados por eje: [ángulo, bias_gyro]
// ═══════════════════════════════════════════════════════
class KalmanAttitude {
public:
  KalmanAttitude(float q_angle = 0.001f,
                 float q_bias  = 0.003f,
                 float r_meas  = 0.03f)
    : Q_angle(q_angle), Q_bias(q_bias), R_meas(r_meas)
    , angle(0.0f), bias(0.0f)
  {
    P[0][0] = 0.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 0.0f;
  }

  float update(float gyroRate, float measurement, float dt) {
    float rate  = gyroRate - bias;
    angle      += rate * dt;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S  = P[0][0] + R_meas;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    float innovation = measurement - angle;
    angle += K0 * innovation;
    bias  += K1 * innovation;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;

    return angle;
  }

  void setAngle(float a) { angle = a; }
  float getAngle()  const { return angle; }
  float getBias()   const { return bias;  }

private:
  float Q_angle, Q_bias, R_meas;
  float angle, bias;
  float P[2][2];
};

KalmanAttitude kalmanPitch(0.001f, 0.003f, 0.03f);
KalmanAttitude kalmanRoll (0.001f, 0.003f, 0.03f);
KalmanAttitude kalmanYaw  (0.001f, 0.003f, 0.5f);  // mag: más ruido

// ═══════════════════════════════════════════════════════
// ESTIMACIÓN DE VELOCIDAD — Fusión GPS + IMU + Kalman
// ═══════════════════════════════════════════════════════
const float GPS_WEIGHT      = 0.3f;
const float MAX_VEL_GPS     = 60.0f;
const int   VEL_SMOOTH_SIZE = 3;

class KalmanVelocity {
public:
  KalmanVelocity(float q = 0.1f, float r = 1.5f)
    : _q(q), _r(r), _x(0.0f), _p(1.0f) {}

  float update(float measurement) {
    _p += _q;
    float k = _p / (_p + _r);
    _x = _x + k * (measurement - _x);
    _p = (1.0f - k) * _p;
    return _x;
  }

  void  setState(float x) { _x = x; _p = 1.0f; }
  float getState()  const { return _x; }

private:
  float _q, _r, _x, _p;
};

KalmanVelocity kfVx, kfVy, kfVz;

// ═══════════════════════════════════════════════════════
// HARDWARE DEFINITIONS
// ═══════════════════════════════════════════════════════
Servo esc;
#define ESC_PIN 14

int servo1Pos = 90, servo2Pos = 90, servo3Pos = 90, servo4Pos = 90;
int servo5Pos = 90, servo6Pos = 90, servo7Pos = 90, servo8Pos = 90;

TinyGPSPlus gps;
bfs::Mpu9250  mpu;

#define GPS_SERIAL Serial2
#define GPS_RX 17
#define GPS_TX 16

#define CE_PIN  5
#define CSN_PIN 4
RF24 radio(CE_PIN, CSN_PIN);
const byte pipeRX[6] = "CMD01";
const byte pipeTX[6] = "TEL01";

// ═══════════════════════════════════════════════════════
// FHSS AND CHANNELS (Salto de frecuencia por software)
// ═══════════════════════════════════════════════════════
const uint8_t fhssChannels[8] = { 10, 25, 40, 55, 70, 85, 100, 115 };
uint8_t currentChannelIdx = 0;
unsigned long lastPacketReceivedMs = 0;

// ═══════════════════════════════════════════════════════
// SECURITY (XXTEA)
// ═══════════════════════════════════════════════════════
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
  TelemetryPacket telem;
}; // 32 bytes

SecureCommand   secCmd;
SecureWaypointPacket secWpPkt;
SecureTelemetry secTelem;
uint8_t         telemSeq = 0;

// ── Target Angles and Flight State ──
float targetRoll     = 0.0f;
float targetPitch    = 0.0f;
float targetYaw      = 0.0f;
float targetThrottle = 0.0f;

float homeLat = 0.0f;
float homeLon = 0.0f;

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

unsigned long lastRFTime = 0;
bool imuActive = false;

// --- PCA9685 ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_CH_YAW    0
#define SERVO_CH_ROLL_L 1
#define SERVO_CH_ROLL_R 2
#define SERVO_CH_FLAP_L 3
#define SERVO_CH_FLAP_R 4
#define SERVO_CH_PITCH_L 5
#define SERVO_CH_PITCH_R 6
#define SERVO_PULSE_MIN 102
#define SERVO_PULSE_MAX 512
#define SERVO_FREQUENCY  50

// ═══════════════════════════════════════════════════════
// VARIABLES DE ESTADO (Compartidas / Volatile)
// ═══════════════════════════════════════════════════════
volatile float latitude   = 0.0f;
volatile float longitude  = 0.0f;
volatile float altitudeM  = 0.0f;
volatile float speedKmph  = 0.0f;

volatile float pitchDeg   = 0.0f;   // Kalman pitch (grados)
volatile float rollDeg    = 0.0f;   // Kalman roll  (grados)
volatile float yawDeg     = 0.0f;   // Kalman yaw   (grados, 0–360)
volatile float totalGForce= 1.0f;

volatile float velX = 0.0f;   // Este (m/s)
volatile float velY = 0.0f;   // Norte (m/s)
volatile float velZ = 0.0f;   // Arriba (m/s)

float lastLat  = 0.0f, lastLon  = 0.0f, lastAlt  = 0.0f;
unsigned long lastGPSVelTime = 0;
bool firstGPSFix = true;
unsigned long lastIMUTime = 0;
float magneticDeclinationDeg = -6.0f; 

// Wind vector variables
volatile float windX = 0.0f;  // East wind (m/s)
volatile float windY = 0.0f;  // North wind (m/s)
volatile float windZ = 0.0f;  // Vertical wind (m/s)

// FreeRTOS thread safety Mutex
portMUX_TYPE sharedStateMutex = portMUX_INITIALIZER_UNLOCKED;

// ═══════════════════════════════════════════════════════
// FLY-BY-WIRE & PID (LAZO INTERNO)
// ═══════════════════════════════════════════════════════
class SlewLimiter {
  float current;
  float maxRate; 
public:
  SlewLimiter(float rate) : current(0), maxRate(rate) {}
  float update(float target, float dt) {
    float delta = target - current;
    float maxDelta = maxRate * dt;
    if (delta > maxDelta) current += maxDelta;
    else if (delta < -maxDelta) current -= maxDelta;
    else current = target;
    return current;
  }
};

class PIDController {
public:
  float kp, ki, kd;
  float integralMax;
  float integral;
  float lastMeasured;
  bool firstRun;

  PIDController(float p, float i, float d, float iMax) : 
    kp(p), ki(i), kd(d), integralMax(iMax), integral(0), lastMeasured(0), firstRun(true) {}

  float compute(float setpoint, float measured, float dt) {
    if (firstRun) {
      lastMeasured = measured;
      firstRun = false;
    }
    
    float error = setpoint - measured;
    float pOut = kp * error;
    
    integral += error * dt;
    if (integral > integralMax) integral = integralMax;
    else if (integral < -integralMax) integral = -integralMax;
    float iOut = ki * integral;
    
    float dMeasured = (measured - lastMeasured) / dt;
    float dOut = -kd * dMeasured;
    lastMeasured = measured;
    
    return pOut + iOut + dOut;
  }
};

SlewLimiter slewRoll(90.0f); 
SlewLimiter slewPitch(90.0f);
PIDController pidRoll(1.2f, 0.1f, 0.2f, 20.0f);
PIDController pidPitch(1.5f, 0.1f, 0.2f, 20.0f);

// ═══════════════════════════════════════════════════════
// NAVIGATION CONSTANTS
// ═══════════════════════════════════════════════════════
const float WP_CAPTURE_NEAR   = 8.0f;    // metros — confirmación de llegada
const float WP_CAPTURE_FAR    = 15.0f;   // metros — zona de desaceleración
const float STALL_SPEED       = 6.0f;    // m/s — velocidad mínima de seguridad recto

// Forward declarations
void handleFhssHopping();
void runL1Navigation();
void runOrbitNavigation();
void applySafetyGuards();
void estimateWind();
void readGPS();
void readMPU();
void updateTelemetry();
void setServoAngle(uint8_t channel, int angle);
void pushTelemetryAck();

// ═══════════════════════════════════════════════════════
// FREERTOS TASKS
// ═══════════════════════════════════════════════════════
void taskStabilization(void *pvParameters);
void taskNavigationComms(void *pvParameters);

void setup() {
  Serial.begin(115200);
  GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  mpu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  int mpuRetries = 10;
  while (!mpu.Begin() && mpuRetries > 0) {
    delay(100);
    mpuRetries--;
  }
  imuActive = (mpuRetries > 0);

  if (imuActive) {
    mpu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_8G);
    mpu.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_500DPS);
    mpu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_20HZ);
    mpu.ConfigSrd(19);   // ~50Hz

    delay(1000);

    mpu.Read();
    float ax = mpu.accel_x_mps2();
    float ay = mpu.accel_y_mps2();
    float az = mpu.accel_z_mps2();
    float initPitch = atan2f(ay, sqrtf(ax*ax + az*az)) * 57.2958f;
    float initRoll  = atan2f(-ax, az) * 57.2958f;
    kalmanPitch.setAngle(initPitch);
    kalmanRoll.setAngle(initRoll);
    kalmanYaw.setAngle(0.0f);
    lastIMUTime = millis();
  }

  if (!radio.begin()) { while (1); }
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(3, 5);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.setChannel(fhssChannels[0]);
  radio.startListening();

  Preferences preferences;
  preferences.begin("pairing", false);
  if (preferences.isKey("shared_key")) {
    preferences.getBytes("shared_key", sharedKey, 16);
  }
  preferences.end();

  // Create thread-safe Tasks
  xTaskCreatePinnedToCore(
    taskStabilization, 
    "Stabilization", 
    4096, 
    NULL, 
    5, // Alta prioridad para lazo PID 50Hz
    NULL, 
    1  // Núcleo 1 (estabilización)
  );

  xTaskCreatePinnedToCore(
    taskNavigationComms, 
    "NavigationComms", 
    8192, 
    NULL, 
    2, // Prioridad normal para guiado/radio
    NULL, 
    0  // Núcleo 0 (cálculos vectoriales/comunicaciones)
  );

  lastRFTime = millis();
  lastPacketReceivedMs = millis();
}

void loop() {
  // Bucle principal vacío, FreeRTOS maneja las tareas en los dos núcleos
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ═══════════════════════════════════════════════════════
// CORE 1: ESTABILIZACIÓN RÁPIDA (50Hz)
// ═══════════════════════════════════════════════════════
void taskStabilization(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // Exactamente 20ms (50Hz)

  while (true) {
    float dt = 0.02f;

    if (imuActive) {
      readMPU();

      float rollLocal, pitchLocal;
      float targetRollLocal, targetPitchLocal, targetYawLocal, targetThrottleLocal;

      // Lectura y escritura atómica de variables compartidas
      portENTER_CRITICAL(&sharedStateMutex);
      targetRollLocal     = targetRoll;
      targetPitchLocal    = targetPitch;
      targetYawLocal      = targetYaw;
      targetThrottleLocal = targetThrottle;
      
      // Pasar actitud actual a navegación
      rollLocal           = rollDeg;
      pitchLocal          = pitchDeg;
      portEXIT_CRITICAL(&sharedStateMutex);

      // 1. Suavizar setpoints de usuario
      float smoothRoll = slewRoll.update(targetRollLocal, dt);
      float smoothPitch = slewPitch.update(targetPitchLocal, dt);

      // 2. Calcular PID
      float rollActuator = pidRoll.compute(smoothRoll, rollLocal, dt);
      float pitchActuator = pidPitch.compute(smoothPitch, pitchLocal, dt);
      float yawActuator = targetYawLocal; 

      // Mitigación de Torque Roll: rampa del acelerador para suavizar aceleraciones
      static float currentThrottle = 0.0f;
      float throttleDelta = targetThrottleLocal - currentThrottle;
      float maxThrottleDelta = 60.0f * dt; // max 60 unidades por segundo de rampa
      if (throttleDelta > maxThrottleDelta) currentThrottle += maxThrottleDelta;
      else if (throttleDelta < -maxThrottleDelta) currentThrottle -= maxThrottleDelta;
      else currentThrottle = targetThrottleLocal;

      // 3. Mezclador a Servos
      servo1Pos = (int)currentThrottle;
      servo2Pos = map(constrain(yawActuator, -40, 40),   -40, 40,  50, 130);
      servo3Pos = map(constrain(rollActuator, -45, 45),  -45, 45,  45, 135);
      servo4Pos = map(constrain(rollActuator, -45, 45),  -45, 45, 135,  45);  // invertido
      servo5Pos = 90;
      servo6Pos = 90;
      servo7Pos = map(constrain(pitchActuator, -30, 30), -30, 30,  60, 120);
      servo8Pos = map(constrain(pitchActuator, -30, 30), -30, 30,  60, 120);

      // 4. Aplicar al hardware
      esc.write(constrain(servo1Pos, 0, 180));
      setServoAngle(SERVO_CH_YAW,     servo2Pos);
      setServoAngle(SERVO_CH_ROLL_L,  servo3Pos);
      setServoAngle(SERVO_CH_ROLL_R,  servo4Pos);
      setServoAngle(SERVO_CH_FLAP_L,  servo5Pos);
      setServoAngle(SERVO_CH_FLAP_R,  servo6Pos);
      setServoAngle(SERVO_CH_PITCH_L, servo7Pos);
      setServoAngle(SERVO_CH_PITCH_R, servo8Pos);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ═══════════════════════════════════════════════════════
// CORE 0: NAVEGACIÓN Y COMUNICACIONES (10Hz)
// ═══════════════════════════════════════════════════════
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

  memcpy(sharedKey, keyBytes, 16);
  Serial.println("[SEC] New key stored in NVS. Restarting...");
  delay(200);
  ESP.restart();
}

void parseSetKeySerial() {
  while (Serial.available()) {
    char ch = (char)Serial.peek();
    if ((uint8_t)ch == MAGIC_CMD || (uint8_t)ch == MAGIC_TELEM || (uint8_t)ch == MAGIC_WP) break;
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

void taskNavigationComms(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz (100ms)

  while (true) {
    parseSetKeySerial();
    readGPS();

    // ── 1. Salto de frecuencia FHSS por software ──
    handleFhssHopping();

    // ── 2. Recepción de Radio ──
    if (radio.available()) {
      uint8_t len = radio.getDynamicPayloadSize();
      if (len == sizeof(SecureCommand)) {
        radio.read(&secCmd, sizeof(SecureCommand));
        btea((uint32_t*)&secCmd, -6); // Descifrar

        uint16_t crc = calculateRadioCRC16((uint8_t*)&secCmd.targetRoll, 20); 
        if (secCmd.magic == MAGIC_CMD && secCmd.crc == crc) {
          static uint8_t lastSeq = 255;
          uint8_t seqDelta = (uint8_t)(secCmd.seq - lastSeq);
          if (seqDelta > 0 && seqDelta <= 128) {
            lastSeq = secCmd.seq;
            lastRFTime = millis();
            lastPacketReceivedMs = millis();

            // Modo de navegación solicitado por GCS
            currentMode = (NavigationMode)secCmd.navMode;

            // Resetear ruta si se indica flag
            if (secCmd.cmdFlags & 0x01) {
              waypointIndex = 0;
            }

            if (currentMode == MODE_MANUAL) {
              // Modo directo (Joystick manual override)
              portENTER_CRITICAL(&sharedStateMutex);
              targetRoll     = secCmd.targetRoll;
              targetPitch    = secCmd.targetPitch;
              targetYaw      = secCmd.targetYaw;
              targetThrottle = secCmd.targetThrottle;
              portEXIT_CRITICAL(&sharedStateMutex);
            }

            magneticDeclinationDeg = secCmd.declinationX10 / 10.0f;
            if (secCmd.homeLat != 0.0f && secCmd.homeLon != 0.0f) {
              homeLat = secCmd.homeLat;
              homeLon = secCmd.homeLon;
            }

            pushTelemetryAck();
          }
        }
      } 
      else if (len == sizeof(SecureWaypointPacket)) {
        radio.read(&secWpPkt, sizeof(SecureWaypointPacket));
        btea((uint32_t*)&secWpPkt, -7); // Descifrar 7 palabras (28 bytes)

        uint16_t crc = calculateRadioCRC16((uint8_t*)&secWpPkt.wpIndex, 24);
        if (secWpPkt.magic == MAGIC_WP && secWpPkt.crc == crc) {
          lastPacketReceivedMs = millis();
          lastRFTime = millis();

          uint8_t idx = secWpPkt.wpIndex;
          if (idx < MAX_WAYPOINTS) {
            waypointRoute[idx] = secWpPkt.wp;
            waypointCount      = secWpPkt.totalWps;
            routeLoop          = (secWpPkt.routeLoop == 1);
            currentMode        = (NavigationMode)secWpPkt.mode;
            
            if (idx == 0) {
              waypointIndex = 0;
            }
          }
          pushTelemetryAck();
        }
      }
      else {
        uint8_t dummy[32];
        radio.read(&dummy, len);
      }
    }

    // ── 3. FAILSAFE / AUTOPILOT CONTROL ──
    bool linkLost = (millis() - lastRFTime > 1000);
    
    if (linkLost) {
      // Pérdida de señal: Invocar RTL autónomo
      currentMode = MODE_WAYPOINT;
      
      // Si home está establecido, crear waypoint temporal hacia home
      if (homeLat != 0.0f && homeLon != 0.0f) {
        waypointRoute[0].lat = homeLat;
        waypointRoute[0].lon = homeLon;
        waypointRoute[0].alt = 40.0f; // 40m altitud segura RTL
        waypointRoute[0].mode = 1; // Waypoint
        waypointCount = 1;
        waypointIndex = 0;
        routeLoop = false;
      } else {
        // Fallback: Senda de planeo nivelada si no hay GPS/Home
        portENTER_CRITICAL(&sharedStateMutex);
        targetRoll = 0;
        targetPitch = 3.0f;
        targetThrottle = 0;
        targetYaw = 0;
        portEXIT_CRITICAL(&sharedStateMutex);
        goto endLoop;
      }
    }

    // ── 4. CÁLCULO DE NAVEGACIÓN VECTORIAL COMPLETA ONBOARD ──
    if (currentMode == MODE_WAYPOINT || currentMode == MODE_ORBIT) {
      estimateWind();

      // ── Onboard Geofence Protection ──
      if (homeLat != 0.0f && homeLon != 0.0f) {
        Vector3D pos = GPSToLocal(latitude, longitude, altitudeM);
        float hDist = sqrtf(pos.x * pos.x + pos.y * pos.y);
        if (hDist > 1800.0f) {
          // Si nos salimos de los 1800m de geocerca, forzar RTL automático hacia home
          currentMode = MODE_WAYPOINT;
          waypointRoute[0].lat = homeLat;
          waypointRoute[0].lon = homeLon;
          waypointRoute[0].alt = 40.0f; // 40m altitud segura RTL
          waypointRoute[0].mode = 1; // Waypoint
          waypointCount = 1;
          waypointIndex = 0;
          routeLoop = false;
        }
      }

      RouteWaypoint currentWP = waypointRoute[waypointIndex];
      if (currentWP.mode == 2 || currentMode == MODE_ORBIT) {
        runOrbitNavigation();
      } else {
        runL1Navigation();
      }

      // Aplicar protecciones de Stall dinámico y limitador de Gs estructurales
      applySafetyGuards();
    }

    endLoop:
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ═══════════════════════════════════════════════════════
// ESTIMADOR HÍBRIDO DE VIENTO SIN TUBO DE PITOT
// ═══════════════════════════════════════════════════════
void estimateWind() {
  float currentThrottleLocal;
  float rollLocal, pitchLocal, yawLocal;

  portENTER_CRITICAL(&sharedStateMutex);
  currentThrottleLocal = targetThrottle;
  rollLocal  = rollDeg;
  pitchLocal = pitchDeg;
  yawLocal   = yawDeg;
  portEXIT_CRITICAL(&sharedStateMutex);

  // Estimación de Airspeed en base al motor (Mustang 1300mm planea ~12m/s a crucero)
  float throttleFactor = constrain(currentThrottleLocal / 100.0f, 0.3f, 1.7f);
  float airspeedEst = 12.0f * throttleFactor; 

  float pitchRad = pitchLocal * 0.0174533f;
  float yawRad   = yawLocal   * 0.0174533f;

  // Vector de velocidad inercial teórica (sin viento)
  float modelVx = airspeedEst * cosf(pitchRad) * sinf(yawRad);   // Este
  float modelVy = airspeedEst * cosf(pitchRad) * cosf(yawRad);   // Norte
  float modelVz = airspeedEst * sinf(pitchRad);                  // Arriba

  // Viento = Velocidad GPS real - Velocidad esperada
  float rawWindX = velX - modelVx;
  float rawWindY = velY - modelVy;
  float rawWindZ = velZ - modelVz;

  // Filtro paso bajo para suavizar viento (alpha = 0.02)
  windX = windX * 0.98f + rawWindX * 0.02f;
  windY = windY * 0.98f + rawWindY * 0.02f;
  windZ = windZ * 0.98f + rawWindZ * 0.02f;
}

// ═══════════════════════════════════════════════════════
// NAVEGACIÓN LATERAL L1 ADAPTATIVA Y TECS
// ═══════════════════════════════════════════════════════
Vector3D GPSToLocal(float lat, float lon, float alt) {
  if (homeLat == 0.0f || homeLon == 0.0f)
    return Vector3D(0, 0, 0);

  float dLat = (lat - homeLat) * 111320.0f;
  float dLon = (lon - homeLon) * 111320.0f * cosf(lat * 3.14159f / 180.0f);
  float dAlt = alt; // Altitud relativa

  return Vector3D(dLon, dLat, dAlt);
}

void runL1Navigation() {
  if (waypointCount == 0) {
    portENTER_CRITICAL(&sharedStateMutex);
    targetRoll = 0;
    targetPitch = 3.0f;
    targetThrottle = 90;
    portEXIT_CRITICAL(&sharedStateMutex);
    return;
  }

  Vector3D pos = GPSToLocal(latitude, longitude, altitudeM);
  Vector3D velocity(velX, velY, velZ);
  float speed = max(velocity.magnitude(), 2.0f);

  RouteWaypoint currentWP = waypointRoute[waypointIndex];
  Vector3D B = GPSToLocal(currentWP.lat, currentWP.lon, currentWP.alt);

  // Zonas de captura: fly-by dinámico
  float distToWP = (B - pos).magnitude();
  if (distToWP < WP_CAPTURE_NEAR) {
    waypointIndex++;
    if (waypointIndex >= waypointCount) {
      if (routeLoop) {
        waypointIndex = 0;
      } else {
        waypointIndex = waypointCount - 1;
        // Si no hay bucle, órbita de loiter en el último WP
        currentWP.mode = 2;
        currentWP.radius = 30.0f;
        currentWP.direction = 1;
      }
    }
    currentWP = waypointRoute[waypointIndex];
    B = GPSToLocal(currentWP.lat, currentWP.lon, currentWP.alt);
  }

  // Segmento AB L1
  Vector3D A;
  if (waypointIndex == 0) {
    A = Vector3D(0, 0, currentWP.alt);
  } else {
    RouteWaypoint prevWP = waypointRoute[waypointIndex - 1];
    A = GPSToLocal(prevWP.lat, prevWP.lon, prevWP.alt);
  }

  Vector3D AB = B - A;
  float segLen = AB.magnitude();
  if (segLen < 1.0f) {
    A = pos;
    AB = B - A;
    segLen = AB.magnitude();
  }

  Vector3D AB_norm = AB.normalize();
  Vector3D AP = pos - A;
  float crossTrackErr = AP.x * AB_norm.y - AP.y * AB_norm.x;

  // L1 distancia adaptativa
  float L1_dist = 3.0f * speed;
  L1_dist = constrain(L1_dist, 10.0f, 60.0f);

  float projDist = AP.dot(AB_norm);
  float targetDistOnSeg = projDist + sqrtf(max(0.0f, L1_dist*L1_dist - crossTrackErr*crossTrackErr));
  targetDistOnSeg = constrain(targetDistOnSeg, 0.0f, segLen);

  Vector3D targetL1 = A + AB_norm * targetDistOnSeg;
  Vector3D toTargetL1 = targetL1 - pos;

  float yawLocal;
  portENTER_CRITICAL(&sharedStateMutex);
  yawLocal = yawDeg;
  portEXIT_CRITICAL(&sharedStateMutex);

  float yawRad = yawLocal * 0.0174533f;
  Vector3D headingDir(sinf(yawRad), cosf(yawRad), 0);

  float dot = headingDir.dot(toTargetL1.normalize());
  dot = constrain(dot, -1.0f, 1.0f);
  float eta = acosf(dot);
  if (toTargetL1.x * headingDir.y - toTargetL1.y * headingDir.x < 0) {
    eta = -eta;
  }

  float a_lat = 2.0f * speed * speed / L1_dist * sinf(eta);
  float rollL1 = atanf(a_lat / 9.81f) * 57.2958f;

  float rollOut = constrain(rollL1, -35.0f, 35.0f);

  // Compensar viento cruzado feed-forward en roll
  float crosswind = -windX * cosf(yawRad) + windY * sinf(yawRad);
  rollOut += constrain(crosswind * 1.5f, -10.0f, 10.0f);
  rollOut = constrain(rollOut, -35.0f, 35.0f);

  // TECS (Total Energy Control System) para Pitch y Throttle
  float altErr = currentWP.alt - altitudeM;
  float speedErr = 12.0f - speed; // Velocidad objetivo 12 m/s

  float energyTotalErr = altErr * 0.1f + speedErr * 0.2f;
  float energyDistErr  = altErr * 0.1f - speedErr * 0.2f;

  float throttleTECS = 100.0f + energyTotalErr * 45.0f;
  float pitchTECS    = energyDistErr * 15.0f;

  portENTER_CRITICAL(&sharedStateMutex);
  targetRoll     = rollOut;
  targetPitch    = constrain(pitchTECS, -10.0f, 10.0f);
  targetThrottle = constrain((int)throttleTECS, 50, 160);
  targetYaw      = 0;
  portEXIT_CRITICAL(&sharedStateMutex);
}

// ═══════════════════════════════════════════════════════
// NAVEGACIÓN ORBITAL
// ═══════════════════════════════════════════════════════
void runOrbitNavigation() {
  Vector3D pos = GPSToLocal(latitude, longitude, altitudeM);
  Vector3D velocity(velX, velY, velZ);
  float speed = max(velocity.magnitude(), 2.0f);

  RouteWaypoint currentWP = waypointRoute[waypointIndex];
  Vector3D center = GPSToLocal(currentWP.lat, currentWP.lon, currentWP.alt);
  float radius = max(currentWP.radius, 15.0f);
  bool cw = (currentWP.direction == 1);

  Vector3D toCenter = center - pos;
  float distToCenter = toCenter.magnitude();

  float radiusErr = distToCenter - radius;
  static float lastRadiusErr = 0;
  float radiusDeriv = radiusErr - lastRadiusErr;
  lastRadiusErr = radiusErr;

  float baseRoll = cw ? -20.0f : 20.0f;
  float rollCorr = (radiusErr * 0.6f + radiusDeriv * 0.2f) * (cw ? 1.0f : -1.0f);
  float rollOrbit = baseRoll + rollCorr;

  float altErr = currentWP.alt - altitudeM;
  float pitchOrbit = constrain(altErr * 0.5f, -10.0f, 10.0f);

  float rollLocal;
  portENTER_CRITICAL(&sharedStateMutex);
  rollLocal = rollDeg;
  portEXIT_CRITICAL(&sharedStateMutex);

  float cosRoll = cosf(rollLocal * 0.0174533f);
  if (cosRoll < 0.2f) cosRoll = 0.2f;
  float throttleCoordinated = 100.0f / cosRoll;

  portENTER_CRITICAL(&sharedStateMutex);
  targetRoll     = constrain(rollOrbit, -35.0f, 35.0f);
  targetPitch    = pitchOrbit;
  targetThrottle = constrain((int)throttleCoordinated, 70, 150);
  targetYaw      = 0;
  portEXIT_CRITICAL(&sharedStateMutex);
}

// ═══════════════════════════════════════════════════════
// MEDIDAS DE SEGURIDAD (Stall Guard, G-Limiter)
// ═══════════════════════════════════════════════════════
void applySafetyGuards() {
  float rollLocal, gforceLocal;
  portENTER_CRITICAL(&sharedStateMutex);
  rollLocal   = rollDeg;
  gforceLocal = totalGForce;
  portEXIT_CRITICAL(&sharedStateMutex);

  // 1. Dynamic Stall Guard
  float cosRoll = cosf(rollLocal * 0.0174533f);
  if (cosRoll < 0.2f) cosRoll = 0.2f;
  float v_stall_din = STALL_SPEED / sqrtf(cosRoll);

  float speed = sqrtf(velX*velX + velY*velY + velZ*velZ);
  if (speed < v_stall_din) {
    portENTER_CRITICAL(&sharedStateMutex);
    targetRoll     = 0.0f;     // Nivelar alas al instante
    targetPitch    = -5.0f;    // Morro abajo para recuperar velocidad
    targetThrottle = 180;      // Motor al máximo
    targetYaw      = 0.0f;
    portEXIT_CRITICAL(&sharedStateMutex);
    return;
  }

  // 2. G-Limiter
  if (gforceLocal > 2.5f) {
    portENTER_CRITICAL(&sharedStateMutex);
    targetRoll     = targetRoll * 0.5f;   // Reducir viraje al instante
    targetPitch    = targetPitch * 0.3f;  // Reducir tirón del elevador
    portEXIT_CRITICAL(&sharedStateMutex);
  }
}

// ═══════════════════════════════════════════════════════
// FHSS SOFTWARE CHANNEL HOPPING
// ═══════════════════════════════════════════════════════
void handleFhssHopping() {
  if (millis() - lastPacketReceivedMs > 1000) {
    // Si perdimos el enlace por más de 1 segundo, saltar de canal cada 100ms buscando la base
    static unsigned long lastHopMs = 0;
    if (millis() - lastHopMs > 100) {
      currentChannelIdx = (currentChannelIdx + 1) % 8;
      radio.setChannel(fhssChannels[currentChannelIdx]);
      lastHopMs = millis();
    }
  }
}

// ═══════════════════════════════════════════════════════
// READ GPS
// ═══════════════════════════════════════════════════════
void readGPS() {
  while (GPS_SERIAL.available() > 0) {
    char c = GPS_SERIAL.read();
    if (!gps.encode(c)) continue;

    if (gps.location.isValid()) {
      float lat = gps.location.lat();
      float lon = gps.location.lng();
      float alt = gps.altitude.isValid() ? gps.altitude.meters() : altitudeM;

      if (firstGPSFix) {
        lastLat = lat; lastLon = lon; lastAlt = alt;
        lastGPSVelTime = millis();
        firstGPSFix = false;
      } else {
        unsigned long now = millis();
        float dt = (now - lastGPSVelTime) / 1000.0f;

        if (dt > 0.05f) {
          float dNorth = (lat - lastLat) * 111320.0f;
          float dEast  = (lon - lastLon) * 111320.0f * cosf(lat * 0.0174533f);
          float dUp    = alt - lastAlt;
          float gpsVx  = dEast  / dt;
          float gpsVy  = dNorth / dt;
          float gpsVz  = dUp    / dt;

          if (fabsf(gpsVx) < MAX_VEL_GPS &&
              fabsf(gpsVy) < MAX_VEL_GPS &&
              fabsf(gpsVz) < MAX_VEL_GPS) {
            
            float gpsMag = sqrtf(gpsVx*gpsVx + gpsVy*gpsVy + gpsVz*gpsVz);
            float pitchRad = pitchDeg * 0.0174533f;
            float yawRad   = yawDeg   * 0.0174533f;

            float imuDirX = cosf(pitchRad) * sinf(yawRad);
            float imuDirY = cosf(pitchRad) * cosf(yawRad);
            float imuDirZ = sinf(pitchRad);

            float curMag = sqrtf(velX*velX + velY*velY + velZ*velZ);
            float scaleMag = (curMag < 2.0f) ? gpsMag : curMag;

            float imuVx = imuDirX * scaleMag;
            float imuVy = imuDirY * scaleMag;
            float imuVz = imuDirZ * scaleMag;

            float hybX = imuVx * (1.0f - GPS_WEIGHT) + gpsVx * GPS_WEIGHT;
            float hybY = imuVy * (1.0f - GPS_WEIGHT) + gpsVy * GPS_WEIGHT;
            float hybZ = imuVz * (1.0f - GPS_WEIGHT) + gpsVz * GPS_WEIGHT;

            float filtX = kfVx.update(hybX);
            float filtY = kfVy.update(hybY);
            float filtZ = kfVz.update(hybZ);

            static float velSmBufX[VEL_SMOOTH_SIZE] = {0};
            static float velSmBufY[VEL_SMOOTH_SIZE] = {0};
            static float velSmBufZ[VEL_SMOOTH_SIZE] = {0};
            static int   velSmIdx = 0;

            velSmBufX[velSmIdx] = filtX;
            velSmBufY[velSmIdx] = filtY;
            velSmBufZ[velSmIdx] = filtZ;
            velSmIdx = (velSmIdx + 1) % VEL_SMOOTH_SIZE;

            float sumX = 0, sumY = 0, sumZ = 0;
            for (int i = 0; i < VEL_SMOOTH_SIZE; i++) {
              sumX += velSmBufX[i];
              sumY += velSmBufY[i];
              sumZ += velSmBufZ[i];
            }
            velX = sumX / VEL_SMOOTH_SIZE;
            velY = sumY / VEL_SMOOTH_SIZE;
            velZ = sumZ / VEL_SMOOTH_SIZE;
          }

          lastLat = lat; lastLon = lon; lastAlt = alt;
          lastGPSVelTime = now;
        }
      }

      latitude  = lat;
      longitude = lon;
      altitudeM = alt;
    }

    if (gps.speed.isValid()) {
      speedKmph = gps.speed.kmph();
    }
  }
}

// ═══════════════════════════════════════════════════════
// READ MPU & COMPUTE KALMAN (50Hz)
// ═══════════════════════════════════════════════════════
void readMPU() {
  if (!mpu.Read()) return;

  unsigned long now = millis();
  float dt = (now - lastIMUTime) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
  lastIMUTime = now;

  float ax = mpu.accel_x_mps2();
  float ay = mpu.accel_y_mps2();
  float az = mpu.accel_z_mps2();

  totalGForce = sqrtf(ax*ax + ay*ay + az*az) / 9.81f;

  float accelPitch = atan2f(ay, sqrtf(ax*ax + az*az)) * 57.2958f;
  float accelRoll  = atan2f(-ax, az) * 57.2958f;

  float gyroPitchRate = mpu.gyro_y_radps() * 57.2958f;
  float gyroRollRate  = mpu.gyro_x_radps() * 57.2958f;
  float gyroYawRate   = mpu.gyro_z_radps() * 57.2958f;

  bool highG = (fabsf(totalGForce - 1.0f) > 0.3f);

  if (highG) {
    pitchDeg = kalmanPitch.update(gyroPitchRate, kalmanPitch.getAngle(), dt);
    rollDeg  = kalmanRoll.update(gyroRollRate,   kalmanRoll.getAngle(),  dt);
  } else {
    pitchDeg = kalmanPitch.update(gyroPitchRate, accelPitch, dt);
    rollDeg  = kalmanRoll.update(gyroRollRate,   accelRoll,  dt);
  }

  float magX = mpu.mag_x_ut();
  float magY = mpu.mag_y_ut();
  float magZ = mpu.mag_z_ut();

  float pitchRad = pitchDeg * 0.0174533f;
  float rollRad  = rollDeg  * 0.0174533f;

  float magXComp =  magX * cosf(pitchRad)
                  + magY * sinf(rollRad) * sinf(pitchRad)
                  + magZ * cosf(rollRad) * sinf(pitchRad);

  float magYComp =  magY * cosf(rollRad)
                  - magZ * sinf(rollRad);

  float magYaw = atan2f(-magYComp, magXComp) * 57.2958f;
  if (magYaw < 0.0f) magYaw += 360.0f;

  magYaw += magneticDeclinationDeg;
  if (magYaw < 0.0f) magYaw += 360.0f;
  if (magYaw >= 360.0f) magYaw -= 360.0f;

  kalmanYaw.update(gyroYawRate, magYaw, dt);
  yawDeg = kalmanYaw.getAngle();

  // complementario Watchdog
  const float COMP_ALPHA = 0.98f;
  static float compPitch = 0.0f, compRoll = 0.0f;
  static bool compInit = false;

  if (!compInit) {
    compPitch = accelPitch;
    compRoll = accelRoll;
    compInit = true;
  }
  compPitch = COMP_ALPHA * (compPitch + gyroPitchRate * dt) + (1.0f - COMP_ALPHA) * accelPitch;
  compRoll = COMP_ALPHA * (compRoll + gyroRollRate * dt) + (1.0f - COMP_ALPHA) * accelRoll;

  const float KALMAN_DIVERGENCE_DEG = 15.0f;
  if (fabsf(pitchDeg - compPitch) > KALMAN_DIVERGENCE_DEG) {
    kalmanPitch.setAngle(compPitch);
    pitchDeg = compPitch;
  }
  if (fabsf(rollDeg - compRoll) > KALMAN_DIVERGENCE_DEG) {
    kalmanRoll.setAngle(compRoll);
    rollDeg = compRoll;
  }

  if (yawDeg < 0.0f)   yawDeg += 360.0f;
  if (yawDeg >= 360.0f) yawDeg -= 360.0f;
}

void updateTelemetry() {
  telemPkt.latitude  = latitude;
  telemPkt.longitude = longitude;
  telemPkt.altitude  = (int16_t)constrain(altitudeM   * 10.0f,  -32767, 32767);
  telemPkt.heading   = (int16_t)constrain(yawDeg      * 10.0f,       0, 35990);
  telemPkt.pitch     = (int16_t)constrain(pitchDeg    * 10.0f,   -9000,  9000);
  telemPkt.roll      = (int16_t)constrain(rollDeg     * 10.0f,  -18000, 18000);
  telemPkt.gforce    = (int16_t)constrain(totalGForce * 100.0f,      0,  3200);
  telemPkt.velocityX = (int16_t)constrain(velX        * 100.0f,  -5000,  5000);
  telemPkt.velocityY = (int16_t)constrain(velY        * 100.0f,  -5000,  5000);
  telemPkt.velocityZ = (int16_t)constrain(velZ        * 100.0f,  -5000,  5000);
}

void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  pwm.setPWM(channel, 0, pulse);
}

void pushTelemetryAck() {
  telemPkt.seq++;
  updateTelemetry();
  secTelem.magic = MAGIC_TELEM;
  secTelem.seq   = telemSeq++;
  secTelem.telem = telemPkt;
  secTelem.crc   = calculateRadioCRC16((uint8_t*)&secTelem.telem, sizeof(TelemetryPacket));
  btea((uint32_t*)&secTelem, 8); 
  radio.writeAckPayload(1, &secTelem, sizeof(SecureTelemetry));
}
