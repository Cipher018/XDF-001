#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>
#include <MPU9250.h>
#include <RF24.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <nRF24L01.h>

// ═══════════════════════════════════════════════════════
// TELEMETRY PACKET — OPTIMIZADO
// Tamaño: 4+4+2+2+2+2+2+2+2+2 = 24 bytes (límite nRF24: 32B)
//
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
};                      // Total: 24 bytes ✓

TelemetryPacket telemPkt;

// ═══════════════════════════════════════════════════════
// KALMAN FILTER — ACTITUD (pitch, roll, yaw)
// Modelo de 2 estados por eje: [ángulo, bias_gyro]
//
// Predicción  (con giróscopo):
//   angle += (gyroRate - bias) * dt
//   P += Q
//
// Corrección  (con acelerómetro / magnetómetro):
//   K = P / (P + R)
//   angle += K * (medición - angle)
//   bias  += K * (medición - angle)   // corrige drift del gyro
//   P = (1 - K) * P
//
// Q_angle: ruido del modelo (proceso)  — mayor → sigue al gyro más rápido
// Q_bias:  ruido del bias del gyro     — mayor → corrige bias más rápido
// R_meas:  ruido de la medición        — mayor → confía menos en accel/mag
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

  // Actualiza el filtro. Devuelve el ángulo estimado.
  // gyroRate: tasa del giróscopo en grados/s
  // measurement: ángulo del acelerómetro o magnetómetro en grados
  // dt: delta de tiempo en segundos
  float update(float gyroRate, float measurement, float dt) {
    // ── Predicción ───────────────────────────────
    float rate  = gyroRate - bias;
    angle      += rate * dt;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // ── Corrección ───────────────────────────────
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

// Una instancia por eje
// Yaw usa R_meas más alto porque el magnetómetro es más ruidoso
KalmanAttitude kalmanPitch(0.001f, 0.003f, 0.03f);
KalmanAttitude kalmanRoll (0.001f, 0.003f, 0.03f);
KalmanAttitude kalmanYaw  (0.001f, 0.003f, 0.5f);  // mag: más ruido

// ═══════════════════════════════════════════════════════
// ESTIMACIÓN DE VELOCIDAD — Fusión GPS + IMU + Kalman
//
// El Kalman de actitud da dirección precisa del movimiento.
// El GPS da velocidad inercial absoluta pero ruidosa.
// La fusión híbrida combina:
//   IMU direction × GPS_speed  → mejor dirección
//   GPS velocity               → referencia absoluta
//   Kalman                     → filtra el resultado
//   Buffer de suavizado        → elimina picos
//
// GPS_WEIGHT  : fracción GPS en la fusión  (0.3 = 30% GPS, 70% IMU)
// MAX_VEL_GPS : guard de velocidad imposible (descarta fix corrupto)
// VEL_SMOOTH  : tamaño del buffer de suavizado
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
// HARDWARE
// ═══════════════════════════════════════════════════════

// --- Throttle ESC ---
Servo esc;
#define ESC_PIN 14

// --- Servo positions ---
int servo1Pos = 90, servo2Pos = 90, servo3Pos = 90, servo4Pos = 90;
int servo5Pos = 90, servo6Pos = 90, servo7Pos = 90, servo8Pos = 90;

// --- Sensors ---
TinyGPSPlus gps;
MPU9250     mpu;

#define GPS_SERIAL Serial2
#define GPS_RX 17
#define GPS_TX 16

// --- Radio ---
#define CE_PIN  5
#define CSN_PIN 4
RF24 radio(CE_PIN, CSN_PIN);
const byte pipeRX[6] = "CMD01";
const byte pipeTX[6] = "TEL01";

// ═══════════════════════════════════════════════════════
// SEGURIDAD DE COMUNICACIONES (XXTEA)
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

struct __attribute__((packed)) SecureCommand {
  uint8_t  magic;
  uint8_t  seq;
  uint16_t crc;
  int16_t  commands[4];
}; // 12 bytes (3 words)

struct __attribute__((packed)) SecureTelemetry {
  uint8_t  magic;
  uint8_t  seq;
  uint16_t crc;
  TelemetryPacket telem;
}; // 28 bytes (7 words)

SecureCommand   secCmd;
SecureTelemetry secTelem;
uint8_t         telemSeq = 0;

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
// VARIABLES DE ESTADO
// ═══════════════════════════════════════════════════════
float latitude   = 0.0f;
float longitude  = 0.0f;
float altitudeM  = 0.0f;
float speedKmph  = 0.0f;

float pitchDeg   = 0.0f;   // Kalman pitch (grados)
float rollDeg    = 0.0f;   // Kalman roll  (grados)
float yawDeg     = 0.0f;   // Kalman yaw   (grados, 0–360)
float totalGForce= 0.0f;

// Velocidad estimada en m/s (marco NED local)
float velX = 0.0f;   // Este
float velY = 0.0f;   // Norte
float velZ = 0.0f;   // Arriba

// Estado para estimación de velocidad GPS
float lastLat  = 0.0f, lastLon  = 0.0f, lastAlt  = 0.0f;
unsigned long lastGPSVelTime = 0;
bool firstGPSFix = true;

// Timestamp IMU para dt preciso
unsigned long lastIMUTime = 0;

void pushTelemetryAck() {
  updateTelemetry();
  secTelem.magic = MAGIC_TELEM;
  secTelem.seq   = telemSeq++;
  secTelem.telem = telemPkt;
  secTelem.crc   = calculateRadioCRC16((uint8_t*)&secTelem.telem, sizeof(TelemetryPacket));
  
  btea((uint32_t*)&secTelem, 7); // Encrypt 7 words (28 bytes)
  radio.writeAckPayload(1, &secTelem, sizeof(SecureTelemetry));
}

// ═══════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  int mpuRetries = 10;
  while (mpu.begin() < 0 && mpuRetries > 0) {
    delay(100);
    mpuRetries--;
  }
  imuActive = (mpuRetries > 0);

  if (imuActive) {
    mpu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    mpu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    mpu.setSrd(19);   // ~50Hz

    delay(1000);
    mpu.calibrateGyro();

    // Inicializar Kalman con primera lectura del acelerómetro
    mpu.readSensor();
    float ax = mpu.getAccelX_mss();
    float ay = mpu.getAccelY_mss();
    float az = mpu.getAccelZ_mss();
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
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  pushTelemetryAck();
  lastRFTime = millis();
}

// ═══════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════
void loop() {
  readGPS();

  static unsigned long lastSensorUpdate = 0;
  if (millis() - lastSensorUpdate >= 20) {  // 50Hz
    lastSensorUpdate = millis();
    if (imuActive) readMPU();
  }

  if (radio.available()) {
    uint8_t len = radio.getDynamicPayloadSize();
    if (len == sizeof(SecureCommand)) {
      radio.read(&secCmd, sizeof(SecureCommand));
      btea((uint32_t*)&secCmd, -3); // Decrypt 3 words (12 bytes)
      
      uint16_t crc = calculateRadioCRC16((uint8_t*)&secCmd.commands, sizeof(secCmd.commands));
      if (secCmd.magic == MAGIC_CMD && secCmd.crc == crc) {
        lastRFTime = millis();

        // [0]=Yaw, [1]=Power, [2]=Pitch, [3]=Roll
        servo1Pos = secCmd.commands[1];
        servo2Pos = map(secCmd.commands[0], -40,  40,  50, 130);
        servo3Pos = map(secCmd.commands[3], -45,  45,  45, 135);
        servo4Pos = map(secCmd.commands[3], -45,  45, 135,  45);  // invertido
        servo5Pos = 90;
        servo6Pos = 90;
        servo7Pos = map(secCmd.commands[2], -30, 30, 60, 120);
        servo8Pos = map(secCmd.commands[2], -30, 30, 60, 120);

        int escAngle = map(servo1Pos, 0, 180, 0, 180);
        esc.write(escAngle);

        setServoAngle(SERVO_CH_YAW,     servo2Pos);
        setServoAngle(SERVO_CH_ROLL_L,  servo3Pos);
        setServoAngle(SERVO_CH_ROLL_R,  servo4Pos);
        setServoAngle(SERVO_CH_FLAP_L,  servo5Pos);
        setServoAngle(SERVO_CH_FLAP_R,  servo6Pos);
        setServoAngle(SERVO_CH_PITCH_L, servo7Pos);
        setServoAngle(SERVO_CH_PITCH_R, servo8Pos);

        pushTelemetryAck();
      }
    } else {
      // Flush invalid payload
      uint8_t dummy[32];
      radio.read(&dummy, len);
    }
  }

  // Failsafe
  if (millis() - lastRFTime > 1000) {
    esc.write(0);
    setServoAngle(SERVO_CH_YAW,     100); // 10 grados rudder para circulo
    setServoAngle(SERVO_CH_ROLL_L,  90);
    setServoAngle(SERVO_CH_ROLL_R,  90);
    setServoAngle(SERVO_CH_FLAP_L,  90);
    setServoAngle(SERVO_CH_FLAP_R,  90);
    setServoAngle(SERVO_CH_PITCH_L, 95); // Ligero pitch up
    setServoAngle(SERVO_CH_PITCH_R, 95);
  }
}

// ═══════════════════════════════════════════════════════
// updateVelocityFusion
// Fusión GPS + IMU para obtener velocidad precisa en 3 ejes.
//
// Flujo:
//   1. Velocidad GPS cruda: diferencia de posición / dt
//   2. Guard: descarta si supera MAX_VEL_GPS (fix corrupto)
//   3. Magnitud GPS: |V_gps|
//   4. Dirección IMU: vector unitario desde pitch/roll/yaw del Kalman
//      Vx_imu = cos(pitch) * sin(yaw)   → Este
//      Vy_imu = cos(pitch) * cos(yaw)   → Norte
//      Vz_imu = sin(pitch)              → Arriba
//   5. Si |V_actual| < 2 m/s (casi parado): usar magnitud GPS directa
//      para evitar que la dirección IMU amplifique ruido
//   6. V_hibrida = V_imu*(1-w) + V_gps*w   (w = GPS_WEIGHT)
//   7. Kalman 1D por eje filtra V_hibrida
//   8. Buffer de suavizado de VEL_SMOOTH_SIZE muestras
// ═══════════════════════════════════════════════════════

static float velSmBufX[VEL_SMOOTH_SIZE] = {0, 0, 0};
static float velSmBufY[VEL_SMOOTH_SIZE] = {0, 0, 0};
static float velSmBufZ[VEL_SMOOTH_SIZE] = {0, 0, 0};
static int   velSmIdx = 0;

void updateVelocityFusion(float gpsVx, float gpsVy, float gpsVz) {
  // ── 3. Magnitud GPS ──────────────────────────────
  float gpsMag = sqrtf(gpsVx*gpsVx + gpsVy*gpsVy + gpsVz*gpsVz);

  // ── 4. Dirección del movimiento según IMU (Kalman de actitud) ──
  float pitchRad = pitchDeg * 0.0174533f;
  float yawRad   = yawDeg   * 0.0174533f;

  // Vector unitario de dirección del avion en marco NED
  float imuDirX = cosf(pitchRad) * sinf(yawRad);   // Este
  float imuDirY = cosf(pitchRad) * cosf(yawRad);   // Norte
  float imuDirZ = sinf(pitchRad);                   // Arriba

  // Magnitud actual estimada
  float curMag = sqrtf(velX*velX + velY*velY + velZ*velZ);

  // ── 5. Si casi parado, usar magnitud GPS directamente ──
  float scaleMag = (curMag < 2.0f) ? gpsMag : curMag;

  float imuVx = imuDirX * scaleMag;
  float imuVy = imuDirY * scaleMag;
  float imuVz = imuDirZ * scaleMag;

  // ── 6. Fusión híbrida ─────────────────────────────
  float hybX = imuVx * (1.0f - GPS_WEIGHT) + gpsVx * GPS_WEIGHT;
  float hybY = imuVy * (1.0f - GPS_WEIGHT) + gpsVy * GPS_WEIGHT;
  float hybZ = imuVz * (1.0f - GPS_WEIGHT) + gpsVz * GPS_WEIGHT;

  // ── 7. Kalman por eje ────────────────────────────
  float filtX = kfVx.update(hybX);
  float filtY = kfVy.update(hybY);
  float filtZ = kfVz.update(hybZ);

  // ── 8. Buffer de suavizado ───────────────────────
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

// ═══════════════════════════════════════════════════════
// readGPS — parseo NMEA + disparo de fusión GPS+IMU
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
          // ── 1. Velocidad GPS cruda ────────────────
          float dNorth = (lat - lastLat) * 111320.0f;
          float dEast  = (lon - lastLon) * 111320.0f * cosf(lat * 0.0174533f);
          float dUp    = alt - lastAlt;
          float gpsVx  = dEast  / dt;
          float gpsVy  = dNorth / dt;
          float gpsVz  = dUp    / dt;

          // ── 2. Guard: fix corrupto ────────────────
          if (fabsf(gpsVx) < MAX_VEL_GPS &&
              fabsf(gpsVy) < MAX_VEL_GPS &&
              fabsf(gpsVz) < MAX_VEL_GPS) {
            // Fusión GPS + IMU (usa pitchDeg/yawDeg del Kalman de actitud)
            updateVelocityFusion(gpsVx, gpsVy, gpsVz);
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
// readMPU — Kalman actitud: pitch, roll, yaw
// ═══════════════════════════════════════════════════════
void readMPU() {
  mpu.readSensor();

  unsigned long now = millis();
  float dt = (now - lastIMUTime) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;  // guard: dt fuera de rango
  lastIMUTime = now;

  // ── Acelerómetro (m/s²) ──────────────────────────
  float ax = mpu.getAccelX_mss();
  float ay = mpu.getAccelY_mss();
  float az = mpu.getAccelZ_mss();

  // G-Force total
  totalGForce = sqrtf(ax*ax + ay*ay + az*az) / 9.81f;

  // Ángulos del acelerómetro (referencia absoluta, ruidosa)
  float accelPitch = atan2f(ay, sqrtf(ax*ax + az*az)) * 57.2958f;
  float accelRoll  = atan2f(-ax, az) * 57.2958f;

  // ── Giróscopo (grados/s) ─────────────────────────
  float gyroPitchRate = mpu.getGyroY_rads() * 57.2958f;
  float gyroRollRate  = mpu.getGyroX_rads() * 57.2958f;
  float gyroYawRate   = mpu.getGyroZ_rads() * 57.2958f;

  // ── Kalman pitch y roll ──────────────────────────
  //    Cuando la G total es muy distinta de 1G (maniobra/aceleración),
  //    el acelerómetro no es confiable. En ese caso solo usamos el gyro:
  //    aumentamos R_meas dinámicamente desactivando la corrección.
  bool highG = (fabsf(totalGForce - 1.0f) > 0.3f);

  if (highG) {
    // Solo predicción: integrar gyro sin corrección del accel
    // Hacemos update con measurement = ángulo actual (sin tirar de él)
    pitchDeg = kalmanPitch.update(gyroPitchRate, kalmanPitch.getAngle(), dt);
    rollDeg  = kalmanRoll.update(gyroRollRate,   kalmanRoll.getAngle(),  dt);
  } else {
    pitchDeg = kalmanPitch.update(gyroPitchRate, accelPitch, dt);
    rollDeg  = kalmanRoll.update(gyroRollRate,   accelRoll,  dt);
  }

  // ── Magnetómetro con compensación de inclinación ─
  float magX = mpu.getMagX_uT();
  float magY = mpu.getMagY_uT();
  float magZ = mpu.getMagZ_uT();

  float pitchRad = pitchDeg * 0.0174533f;
  float rollRad  = rollDeg  * 0.0174533f;

  // Compensación completa de tilt (pitch + roll)
  float magXComp =  magX * cosf(pitchRad)
                  + magY * sinf(rollRad) * sinf(pitchRad)
                  + magZ * cosf(rollRad) * sinf(pitchRad);

  float magYComp =  magY * cosf(rollRad)
                  - magZ * sinf(rollRad);

  float magYaw = atan2f(-magYComp, magXComp) * 57.2958f;
  if (magYaw < 0.0f) magYaw += 360.0f;

  // Kalman yaw: gyroYawRate en deg/s, magYaw como medición absoluta
  yawDeg = kalmanYaw.update(gyroYawRate, magYaw, dt);
  if (yawDeg < 0.0f)   yawDeg += 360.0f;
  if (yawDeg >= 360.0f) yawDeg -= 360.0f;
}

// ═══════════════════════════════════════════════════════
// updateTelemetry — empaqueta con conversión a int16
// ═══════════════════════════════════════════════════════
void updateTelemetry() {
  telemPkt.latitude  = latitude;
  telemPkt.longitude = longitude;

  // Conversión float → int16 con escala
  telemPkt.altitude  = (int16_t)constrain(altitudeM   * 10.0f,  -32767, 32767);
  telemPkt.heading   = (int16_t)constrain(yawDeg      * 10.0f,       0, 35990);
  telemPkt.pitch     = (int16_t)constrain(pitchDeg    * 10.0f,   -9000,  9000);
  telemPkt.roll      = (int16_t)constrain(rollDeg     * 10.0f,  -18000, 18000);
  telemPkt.gforce    = (int16_t)constrain(totalGForce * 100.0f,      0,  3200);
  telemPkt.velocityX = (int16_t)constrain(velX        * 100.0f,  -5000,  5000);
  telemPkt.velocityY = (int16_t)constrain(velY        * 100.0f,  -5000,  5000);
  telemPkt.velocityZ = (int16_t)constrain(velZ        * 100.0f,  -5000,  5000);
}

// ═══════════════════════════════════════════════════════
// setServoAngle — helper PCA9685
// ═══════════════════════════════════════════════════════
void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  pwm.setPWM(channel, 0, pulse);
}
