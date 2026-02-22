#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>
#include <MPU9250.h>
#include <RF24.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
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
// --- Radio nRF24L01 Configuration ---
#define CE_PIN 5
#define CSN_PIN 4

RF24 radio(CE_PIN, CSN_PIN);
const byte pipeRX[6] = "CMD01";
const byte pipeTX[6] = "TEL01";

// --- Payloads ---
int16_t commandData[4];       // Received commands [Yaw, Power, Pitch, Roll]
float telemetryData[6];       // Telemetry packet to send
unsigned long lastRFTime = 0; // Failsafe timer

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

  if (mpu.begin() < 0) {
    Serial.println("FATAL ERROR: MPU-9250 not detected. System Halted.");
    while (1)
      ;
  }

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

  // Set all servos to neutral position (90 degrees)
  for (int i = 0; i < 7; i++) {
    setServoAngle(i, 90);
  }

  Serial.println("Servo Driver PCA9685: Ready");

  // Prepare first telemetry packet for transmission
  updateTelemetry();
  radio.writeAckPayload(1, &telemetryData, sizeof(telemetryData));
  lastRFTime = millis();

  Serial.println("\n--- AIRCRAFT CADI_A: READY FOR MISSION ---\n");
}

void loop() {
  // Continuous sensor acquisition
  readGPS();

  static unsigned long lastSensorUpdate = 0;
  if (millis() - lastSensorUpdate >= 20) { // 50Hz timer
    lastSensorUpdate = millis();
    readMPU();
  }

  // Process incoming radio commands if available
  if (radio.available()) {
    lastRFTime = millis();
    // Retrieve command payload from ground station
    radio.read(&commandData, sizeof(commandData));

    // Command Mapping: [0]=Yaw, [1]=Power, [2]=Pitch, [3]=Roll
    // Map control range to appropriate servo angles
    servo1Pos = commandData[1]; // Throttle (Pass-through)
    servo2Pos = map(commandData[0], -40, 40, 50, 130); // Rudder/Yaw
    servo3Pos = map(commandData[3], -45, 45, 45, 135); // Left Aileron
    servo4Pos =
        map(commandData[3], -45, 45, 135, 45); // Right Aileron (Inverted)
    servo5Pos = 90;                            // Flaps - Future implementation
    servo6Pos = 90;                            // Flaps - Future implementation
    servo7Pos = map(commandData[2], -30, 30, 60, 120); // Left Elevator
    servo8Pos = map(commandData[2], -30, 30, 60, 120); // Right Elevator

    // Update dedicated ESC Throttle (Mapping 0-180 command to standard pulse
    // width optionally, or letting servo lib map it)
    int escAngle = map(servo1Pos, 0, 180, 0, 180);
    esc.write(escAngle);

    // Update servo physical positions through PCA9685
    setServoAngle(SERVO_CH_YAW, servo2Pos);
    setServoAngle(SERVO_CH_ROLL_L, servo3Pos);
    setServoAngle(SERVO_CH_ROLL_R, servo4Pos);
    setServoAngle(SERVO_CH_FLAP_L, servo5Pos);
    setServoAngle(SERVO_CH_FLAP_R, servo6Pos);
    setServoAngle(SERVO_CH_PITCH_L, servo7Pos);
    setServoAngle(SERVO_CH_PITCH_R, servo8Pos);

    // Prepare fresh telemetry for the next acknowledgment
    updateTelemetry();
    radio.writeAckPayload(1, &telemetryData, sizeof(telemetryData));

    Serial.println("Command received. Telemetry updated.");
  }

  // Failsafe check: Trigger if signal lost for > 1000ms
  if (millis() - lastRFTime > 1000) {
    esc.write(0); // Cut throttle
    setServoAngle(SERVO_CH_YAW, 90);
    setServoAngle(SERVO_CH_ROLL_L, 90);
    setServoAngle(SERVO_CH_ROLL_R, 90);
    setServoAngle(SERVO_CH_FLAP_L, 90);
    setServoAngle(SERVO_CH_FLAP_R, 90);
    setServoAngle(SERVO_CH_PITCH_L, 95); // Slight pitch up for glide
    setServoAngle(SERVO_CH_PITCH_R, 95);

    static unsigned long lastFailsafePrint = 0;
    if (millis() - lastFailsafePrint > 1000) {
      Serial.println("!!! FAILSAFE TRIGGERED !!! SIGNAL LOST");
      lastFailsafePrint = millis();
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