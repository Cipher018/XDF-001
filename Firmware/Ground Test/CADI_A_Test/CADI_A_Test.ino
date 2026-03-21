#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>
#include <RF24.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <mpu9250.h>
#include <nRF24L01.h>

// --- Throttle ESC ---
Servo esc;
#define ESC_PIN 14

// --- Servo Configuration ---
int servo1Pos = 90;
int servo2Pos = 90;
int servo3Pos = 90;
int servo4Pos = 90;
int servo5Pos = 90;
int servo6Pos = 90;
int servo7Pos = 90;
int servo8Pos = 90;

// --- Sensors & Devices ---
TinyGPSPlus gps;
MPU9250 mpu;

// GPS Serial configuration (ESP32 typically uses Serial2)
#define GPS_SERIAL Serial2
#define GPS_RX 17
#define GPS_TX 16

// --- Navigation Data Variables ---
float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;
float speedKmph = 0.0;
float heading = 0.0;
float pitch = 0.0;
float totalGForce = 0.0;

// --- Radio nRF24L01 Configuration ---
#define CE_PIN 5
#define CSN_PIN 4

RF24 radio(CE_PIN, CSN_PIN);
const byte pipeRX[6] = "CMD01";
const byte pipeTX[6] = "TEL01";

// ═══════════════════════════════════════════════════════
// SEGURIDAD DE COMUNICACIONES (XXTEA)
// ═══════════════════════════════════════════════════════
// ⚠ SEGURIDAD: Clave XXTEA hardcodeada — SOLO PARA PROTOTIPO.
// En producción, derivar de ESP.getEfuseMac() y almacenar en NVS cifrado.
// Cualquier persona con acceso al binario puede extraer esta clave.
uint32_t sharedKey[4] = {0x58444630, 0x30314B45, 0x595F3230,
                         0x32362121}; // "XDF001KEY_2026!!" en hex
#define XXTEA_DELTA 0x9e3779b9
#define XXTEA_MX                                                               \
  (((z >> 5 ^ y << 2) + (y >> 3 ^ z << 4)) ^                                   \
   ((sum ^ y) + (sharedKey[(p & 3) ^ e] ^ z)))

void btea(uint32_t *v, int n) {
  uint32_t y, z, sum;
  unsigned p, rounds, e;
  if (n > 1) {
    rounds = 6 + 52 / n;
    sum = 0;
    z = v[n - 1];
    do {
      sum += XXTEA_DELTA;
      e = (sum >> 2) & 3;
      for (p = 0; p < n - 1; p++) {
        y = v[p + 1];
        z = v[p] += XXTEA_MX;
      }
      y = v[0];
      z = v[n - 1] += XXTEA_MX;
    } while (--rounds);
  } else if (n < -1) {
    n = -n;
    rounds = 6 + 52 / n;
    sum = rounds * XXTEA_DELTA;
    y = v[0];
    do {
      e = (sum >> 2) & 3;
      for (p = n - 1; p > 0; p--) {
        z = v[p - 1];
        y = v[p] -= XXTEA_MX;
      }
      z = v[n - 1];
      y = v[0] -= XXTEA_MX;
      sum -= XXTEA_DELTA;
    } while (--rounds);
  }
}

// ── Headers Seguros ──
const uint8_t MAGIC_CMD = 0xAA;
const uint8_t MAGIC_TELEM = 0xBB;

struct __attribute__((packed)) SecureCommand {
  uint8_t magic;
  uint8_t seq;
  uint16_t crc;
  int16_t targetRoll;
  int16_t targetPitch;
  int16_t targetYaw;
  int16_t targetThrottle;
  float homeLat;
  float homeLon;
  uint16_t padding;
}; // 24 bytes

struct __attribute__((packed)) SecureTelemetry {
  uint8_t magic;
  uint8_t seq;
  uint16_t crc;
  float telem[6];
}; // 28 bytes

SecureCommand secCmd;
SecureTelemetry secTelem;
uint8_t telemSeq = 0;

// ── Target Angles from Ground Station ──
int16_t targetRoll = 0;
int16_t targetPitch = 0;
int16_t targetYaw = 0;
int16_t targetThrottle = 0;

// ── Home Coordinates for RTL ──
float homeLat = 0.0f;
float homeLon = 0.0f;

uint16_t calculateRadioCRC16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc = crc << 1;
    }
  }
  return crc;
}

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
    if (delta > maxDelta)
      current += maxDelta;
    else if (delta < -maxDelta)
      current -= maxDelta;
    else
      current = target;
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

  PIDController(float p, float i, float d, float iMax)
      : kp(p), ki(i), kd(d), integralMax(iMax), integral(0), lastMeasured(0),
        firstRun(true) {}

  float compute(float setpoint, float measured, float dt) {
    if (firstRun) {
      lastMeasured = measured;
      firstRun = false;
    }

    float error = setpoint - measured;
    float pOut = kp * error;

    integral += error * dt;
    if (integral > integralMax)
      integral = integralMax;
    else if (integral < -integralMax)
      integral = -integralMax;
    float iOut = ki * integral;

    float dMeasured = (measured - lastMeasured) / dt;
    float dOut = -kd * dMeasured;
    lastMeasured = measured;

    return pOut + iOut + dOut;
  }
};

// ── Controladores ──
SlewLimiter slewRoll(90.0f);
SlewLimiter slewPitch(90.0f);

PIDController pidRoll(1.2f, 0.1f, 0.2f, 20.0f);
PIDController pidPitch(1.5f, 0.1f, 0.2f, 20.0f);

// --- Payloads ---
float telemetryData[6];       // Telemetry packet to send
unsigned long lastRFTime = 0; // Failsafe timer
bool imuActive = false;

// --- Servo Driver PCA9685 Configuration ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_CH_YAW 0     // Yaw
#define SERVO_CH_ROLL_L 1  // Roll (left wing)
#define SERVO_CH_ROLL_R 2  // Roll (right wing)
#define SERVO_CH_FLAP_L 3  // Flaps (left wing)
#define SERVO_CH_FLAP_R 4  // Flaps (right wing)
#define SERVO_CH_PITCH_L 5 // Pitch (left elevator)
#define SERVO_CH_PITCH_R 6 // Pitch (right elevator)

#define SERVO_PULSE_MIN 102 // ~0 degrees
#define SERVO_PULSE_MAX 512 // ~180 degrees
#define SERVO_FREQUENCY 50  // 50Hz standard for servos

void setup() {
  Serial.begin(115200);
  Serial.println("Initialising System...");

  // Initialize GPS Serial
  GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS: Initialized");

  // Initialize MPU-9250 via I2C (Bus 0)
  Wire.begin(21, 22);    // SDA: 21, SCL: 22
  Wire.setClock(400000); // 400kHz I2C high speed

  int mpuRetries = 10;
  while (mpu.begin() < 0 && mpuRetries > 0) {
    delay(100);
    mpuRetries--;
  }
  imuActive = (mpuRetries > 0);

  if (imuActive) {
    // Configure MPU-9250 settings
    mpu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    mpu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    mpu.setSrd(19); // Internal sample rate divider to reach ~50Hz

    Serial.println("MPU-9250: Online");

    // Calibrate Gyroscope (keep sensor stationary during this phase)
    Serial.println(
        "CALIBRATING GYRO (3 seconds)... Please do not move the aircraft.");
    delay(1000);
    mpu.calibrateGyro();
    Serial.println("Gyroscope Calibration Complete.");
  } else {
    Serial.println("WARNING: MPU-9250 not detected. Running without IMU.");
  }

  // Initialize Radio nRF24L01
  if (!radio.begin()) {
    Serial.println("FATAL ERROR: nRF24L01 Radio not detected. System Halted.");
    while (1)
      ;
  }

  radio.enableAckPayload();
  radio.setRetries(3, 5);
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.setPALevel(RF24_PA_LOW); // PA_LOW for testing; use PA_MAX for flight
  radio.startListening();

  Serial.println("RF Radio: Online");

  // Initialize PCA9685 Servo Driver via I2C (Bus 1)
  Wire1.begin(32, 27); // SDA: 32, SCL: 27
  pwm = Adafruit_PWMServoDriver(0x40, Wire1);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQUENCY);
  delay(10);

  // Initialize dedicated ESC Throttle pin
  ESP32PWM::allocateTimer(0);
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, 1000, 2000); // Standard ESC pulse width
  esc.write(0);                    // Arming step (neutral)

  void pushTelemetryAck() {
    updateTelemetry();
    secTelem.magic = MAGIC_TELEM;
    secTelem.seq = telemSeq++;
    memcpy(secTelem.telem, telemetryData, sizeof(telemetryData));
    secTelem.crc =
        calculateRadioCRC16((uint8_t *)&secTelem.telem, sizeof(telemetryData));

    // ══ ORDEN CRÍTICO: NO REORDENAR ══════════════════
    // Paso 1 (arriba): CRC sobre datos en claro
    // Paso 2 (abajo): Cifrar paquete completo (CRC incluido)
    btea((uint32_t *)&secTelem, 7); // Encrypt 7 words (28 bytes)
    radio.writeAckPayload(1, &secTelem, sizeof(SecureTelemetry));
  }

  // Initialize nRF Radio after IMU... (Already inside setup function, I'll
  // place this just after servo zeroing) Set all servos to neutral position (90
  // degrees)
  for (int i = 0; i < 7; i++) {
    setServoAngle(i, 90);
  }

  Serial.println("Servo Driver PCA9685: Ready");

  radio.enableDynamicPayloads();

  // Prepare first telemetry packet for transmission
  pushTelemetryAck();
  lastRFTime = millis();

  Serial.println("\n--- AIRCRAFT CADI_A: READY FOR MISSION ---\n");
}

void loop() {
  // Continuous sensor acquisition
  readGPS();

  // ═══════════════════════════════════════════════════════
  // FAILSAFE: RETURN TO LAUNCH (RTL) AUTÓNOMO
  // ═══════════════════════════════════════════════════════
  if (millis() - lastRFTime > 1000) {
    if (homeLat != 0.0f && homeLon != 0.0f && latitude != 0.0f &&
        longitude != 0.0f) {
      float dLat = (homeLat - latitude) * 111320.0f;
      float dLon =
          (homeLon - longitude) * 111320.0f * cosf(latitude * PI / 180.0f);
      float distanceToHome = sqrtf(dLat * dLat + dLon * dLon);
      float bearingToHome = atan2f(dLon, dLat) * 180.0f / PI; // -180 a 180

      float currentYaw = heading;
      if (currentYaw > 180.0f)
        currentYaw -= 360.0f;

      float yawError = bearingToHome - currentYaw;
      if (yawError > 180.0f)
        yawError -= 360.0f;
      if (yawError < -180.0f)
        yawError += 360.0f;

      if (distanceToHome > 30.0f) {
        // RTL Mode: volar hacia la base
        targetRoll = constrain(yawError * 0.8f, -35.0f, 35.0f);
        targetPitch = 5.0f;
        targetYaw = 0.0f;
        targetThrottle = 120;
      } else {
        // LOITER Mode: órbita autónoma controlada por error de radio
        // Usa controlador PD simplificado con home como centro de órbita.
        // Radio objetivo = 30m (coincide con umbral RTL→Loiter).
        static float lastRtlRadiusErr = 0.0f;
        const float RTL_ORBIT_RADIUS = 30.0f;
        const float RTL_KP_RADIUS   = 0.6f;
        const float RTL_KD_RADIUS   = 0.2f;
        const float RTL_BASE_ROLL   = 20.0f;

        float radiusErr   = distanceToHome - RTL_ORBIT_RADIUS;
        float radiusDeriv = radiusErr - lastRtlRadiusErr;
        lastRtlRadiusErr  = radiusErr;

        float rollCorrection = RTL_KP_RADIUS * radiusErr + RTL_KD_RADIUS * radiusDeriv;
        targetRoll     = constrain(RTL_BASE_ROLL + rollCorrection, -35.0f, 35.0f);
        targetPitch    = 3.0f;
        targetYaw      = 0.0f;
        targetThrottle = 100;
      }
    } else {
      // Fallback: Senda de planeo (si no hay datos remotos o señal GPS)
      targetRoll = 0.0f;
      targetPitch = 5.0f;
      targetYaw = 0.0f;
      targetThrottle = 0;
    }

    static unsigned long lastFailsafePrint = 0;
    if (millis() - lastFailsafePrint > 1000) {
      Serial.println(
          "!!! FAILSAFE TRIGGERED !!! SIGNAL LOST - AUTONOMOUS RTL ACTIVE");
      lastFailsafePrint = millis();
    }
  }

  static unsigned long lastSensorUpdate = 0;
  unsigned long now = millis();
  if (now - lastSensorUpdate >= 20) { // 50Hz Inner Loop
    float dt = (now - lastSensorUpdate) / 1000.0f;
    if (dt <= 0)
      dt = 0.02f;
    lastSensorUpdate = now;

    if (imuActive) {
      readMPU();

      // 1. Suavizar setpoints de usuario
      float smoothRoll = slewRoll.update((float)targetRoll, dt);
      float smoothPitch = slewPitch.update((float)targetPitch, dt);

      // 2. Calcular PID
      float rollActuator = pidRoll.compute(smoothRoll, roll, dt);
      float pitchActuator = pidPitch.compute(smoothPitch, pitch, dt);

      float yawActuator = (float)targetYaw;

      // 3. Mezclador a Servos
      servo1Pos = targetThrottle; // Throttle
      servo2Pos =
          map(constrain(yawActuator, -40, 40), -40, 40, 50, 130); // Rudder/Yaw
      servo3Pos = map(constrain(rollActuator, -45, 45), -45, 45, 45,
                      135); // Left Aileron
      servo4Pos = map(constrain(rollActuator, -45, 45), -45, 45, 135,
                      45); // Right Aileron (Inverted)
      servo5Pos = 90;      // Flaps
      servo6Pos = 90;      // Flaps
      servo7Pos = map(constrain(pitchActuator, -30, 30), -30, 30, 60,
                      120); // Left Elevator
      servo8Pos = map(constrain(pitchActuator, -30, 30), -30, 30, 60,
                      120); // Right Elevator

      // 4. Aplicar al hardware (El Failsafe ahora dicta los targets si es
      // necesario)
      int escAngle = map(servo1Pos, 0, 180, 0, 180);
      esc.write(escAngle);

      setServoAngle(SERVO_CH_YAW, servo2Pos);
      setServoAngle(SERVO_CH_ROLL_L, servo3Pos);
      setServoAngle(SERVO_CH_ROLL_R, servo4Pos);
      setServoAngle(SERVO_CH_FLAP_L, servo5Pos);
      setServoAngle(SERVO_CH_FLAP_R, servo6Pos);
      setServoAngle(SERVO_CH_PITCH_L, servo7Pos);
      setServoAngle(SERVO_CH_PITCH_R, servo8Pos);
    }
  }

  // Process incoming radio commands if available
  if (radio.available()) {
    uint8_t len = radio.getDynamicPayloadSize();
    if (len == sizeof(SecureCommand)) {
      radio.read(&secCmd, sizeof(SecureCommand));
      // ══ ORDEN CRÍTICO: NO REORDENAR ══════════════════
      // Paso 1: CRC sobre datos en claro
      uint16_t crc = calculateRadioCRC16((uint8_t *)&secCmd.targetRoll, 16);
      // Paso 2: Verificar magic + CRC
      if (secCmd.magic == MAGIC_CMD && secCmd.crc == crc) {
        // ── Anti-replay: verificación de secuencia circular ──
        // seqDelta en [1..128] = paquete nuevo válido
        // seqDelta == 0 = duplicado (replay), >128 = del pasado
        static uint8_t lastSeq = 255;
        uint8_t seqDelta = (uint8_t)(secCmd.seq - lastSeq);
        if (seqDelta == 0 || seqDelta > 128) {
          goto skipCommand; // Paquete replay o fuera de orden
        }
        lastSeq = secCmd.seq;

        lastRFTime = millis();

        // [0]=Yaw, [1]=Power, [2]=Pitch, [3]=Roll
        targetRoll = secCmd.targetRoll;
        targetPitch = secCmd.targetPitch;
        targetYaw = secCmd.targetYaw;
        targetThrottle = secCmd.targetThrottle;

        // NOTA: esta condición impide que el home se resetee a (0,0) por
        // paquetes con campos vacíos, pero también bloquea un home legítimo
        // en lat=0, lon=0 (Gulf of Guinea). Aceptable para operación normal;
        // considerar flag explícito "homeValid" si se opera cerca del ecuador/meridiano.
        if (secCmd.homeLat != 0.0f && secCmd.homeLon != 0.0f) {
          homeLat = secCmd.homeLat;
          homeLon = secCmd.homeLon;
        }

        // Ya no aplicamos comandos al servo manual aquí.
        pushTelemetryAck();
        Serial.println("Command received. Telemetry updated.");
      }
      skipCommand:;
    } else {
      uint8_t dummy[32];
      radio.read(&dummy, len);
    }
  }
}

// Parse NMEA data from the GPS module
void readGPS() {
  while (GPS_SERIAL.available() > 0) {
    char c = GPS_SERIAL.read();

    if (gps.encode(c)) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      }

      if (gps.altitude.isValid()) {
        altitude = gps.altitude.meters();
      }

      if (gps.speed.isValid()) {
        speedKmph = gps.speed.kmph();
      }
    }
  }
}

// Acquire IMU data and calculate orientation
void readMPU() {
  mpu.readSensor();

  // Convert acceleration to G-units
  float accelX = mpu.getAccelX_mss() / 9.81;
  float accelY = mpu.getAccelY_mss() / 9.81;
  float accelZ = mpu.getAccelZ_mss() / 9.81;

  // Calculate resultant G-Force magnitude
  totalGForce = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  // Calculate Pitch (attitude) from accelerometer data
  pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 57.2958;

  // Retrieve Magnetometer readings
  float magX = mpu.getMagX_uT();
  float magY = mpu.getMagY_uT();
  float magZ = mpu.getMagZ_uT();

  // Apply tilt compensation to magnetometer data
  float pitchRad = pitch * 0.0174533;

  float magXComp = magX * cos(pitchRad) + magZ * sin(pitchRad);
  float magYComp = magX * sin(pitchRad) + magY - magZ * cos(pitchRad);

  // Calculate Heading (Yaw) from compensated magnetometer data
  heading = atan2(magYComp, magXComp) * 57.2958;

  // Normalize Heading to 0-360 range
  if (heading < 0)
    heading += 360;
}

// Package sensor data for transmission and provide local serial debugging
void updateTelemetry() {
  telemetryData[0] = latitude;
  telemetryData[1] = longitude;
  telemetryData[2] = heading;
  telemetryData[3] = altitude;
  telemetryData[4] = pitch;
  telemetryData[5] = totalGForce;

  // Local debug log (Frequency capped to 1Hz)
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000) {
    Serial.println("\n=== SYSTEM TELEMETRY ===");
    Serial.print("Location: ");
    Serial.print(latitude, 6);
    Serial.print(", ");
    Serial.println(longitude, 6);
    Serial.print("Heading:  ");
    Serial.print(heading, 1);
    Serial.println(" deg");
    Serial.print("Altitude: ");
    Serial.print(altitude, 1);
    Serial.println(" m");
    Serial.print("Pitch:    ");
    Serial.print(pitch, 1);
    Serial.println(" deg");
    Serial.print("G-Force:  ");
    Serial.print(totalGForce, 2);
    Serial.println(" g");
    Serial.println("========================\n");
    lastPrintTime = millis();
  }
}

// Helper to set servo angle via PCA9685 driver
void setServoAngle(uint8_t channel, int angle) {
  // Constrain angle to valid 0-180 range
  if (angle < 0)
    angle = 0;
  if (angle > 180)
    angle = 180;

  // Map angle to PWM duty cycle pulse
  int pulse = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);

  pwm.setPWM(channel, 0, pulse);
}