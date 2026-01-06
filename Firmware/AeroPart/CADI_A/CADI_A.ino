#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <TinyGPSPlus.h>
#include <MPU9250.h>

//Servos
int Servo1 = 90;
int Servo2 = 90;
int Servo3 = 90;
int Servo4 = 90;
int Servo5 = 90;
int Servo6 = 90;
int Servo7 = 90;
int Servo8 = 90;

//Sensors
TinyGPSPlus gps;
MPU9250 mpu;

// GPS Serial (ESP32 tiene Serial2)
#define GPS_SERIAL Serial2
#define GPS_RX 16
#define GPS_TX 17

// Variables de sensores
float latitud = 0.0;
float longitud = 0.0;
float altitud = 0.0;
float velocidad = 0.0;
float rumbo = 0.0;      
float actitud = 0.0;    
float aceleracion = 0.0; 

//=== RADIO nRF24L01 ===
#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);
const byte pipeRX[6] = "CMD01"; 
const byte pipeTX[6] = "TEL01"; 

//=== PAYLOADS ===
int16_t commands[4];    
float telemetry[6];     

//=== SERVO DRIVER PCA9685 ===
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_1  0 // Power/Throttle
#define SERVO_2  1 // Yaw
#define SERVO_3  2 // Roll (left wing)
#define SERVO_4  3 // Roll (right wing)
#define SERVO_5  4 // Flaps (left wing)
#define SERVO_6  5 // Flaps (right wing)
#define SERVO_7  6 // Pitch (Left stabilizer)
#define SERVO_8  7 // Pitch (right stabilizer)

#define SERVOMIN  102 // Minimum pulse
#define SERVOMAX  512 // Maximum pulse
#define SERVO_FREQ 50 // Frequency = 50Hz

void setup() {
  Serial.begin(115200);
  Serial.println("Checking Sensor Status");
  
  //GPS Block
  GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS: ONLINE");
  
  //MPU9250 Block
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
  
  if (mpu.begin() < 0) {
    Serial.println("ERROR: MPU-9250 no detected");
    while(1); // Detener si falla
  }
  
  // Configurar MPU-9250
  mpu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  mpu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  mpu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  mpu.setSrd(19); // 50Hz sample rate
  
  Serial.println("MPU-9250: ONLINE");
  
  // Calibrar giroscopio (mantén quieto el sensor)
  Serial.println("GIROSCOPE CALIBRATION (3 seg)...");
  delay(1000);
  mpu.calibrateGyro();
  Serial.println("CALIBRATION COMPLETE");
  
  //=== INICIALIZAR RADIO nRF24L01 ===
  if (!radio.begin()) {
    Serial.println("ERROR: RADIO NO DETECTED");
    while(1);
  }
  
  radio.enableAckPayload();
  radio.setRetries(3, 5);
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  
  Serial.println("RADIO: ONLINE");
  
  //=== INICIALIZAR SERVO DRIVER ===
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  
  // Posición neutral de todos los servos
  for(int i = 0; i < 8; i++) {
    setServoAngle(i, 90);
  }
  
  Serial.println("Servo Driver PCA9685: OK");
  
  // Preparar primera telemetría
  actualizarTelemetria();
  radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
  
  Serial.println("\n=== AIRPLANE: ONLINE ===\n");
}

void loop() {
  // Leer sensores constantemente
  leerGPS();
  leerMPU();
  
  // Procesar comandos de radio si están disponibles
  if (radio.available()) {
    // Leer comandos del control remoto
    radio.read(&commands, sizeof(commands));
    

    Servo1 = commands[1];                    // Power (0-180)
    Servo2 = map(commands[0], -40, 40, 50, 130); // Yaw
    Servo3 = map(commands[3], -45, 45, 45, 135); // Roll left
    Servo4 = map(commands[3], -45, 45, 135, 45); // Roll right 
    Servo5 = 90; // Flaps - WIP
    Servo6 = 90; // Flaps - WIP
    Servo7 = map(commands[2], -30, 30, 60, 120); // Pitch left
    Servo8 = map(commands[2], -30, 30, 60, 120); // Pitch right
    
    setServoAngle(SERVO_1, Servo1);
    setServoAngle(SERVO_2, Servo2);
    setServoAngle(SERVO_3, Servo3);
    setServoAngle(SERVO_4, Servo4);
    setServoAngle(SERVO_5, Servo5);
    setServoAngle(SERVO_6, Servo6);
    setServoAngle(SERVO_7, Servo7);
    setServoAngle(SERVO_8, Servo8);

    actualizarTelemetria();
    radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
    
    Serial.println("Telemetría actualizada y enviada");
  }
  
}


void leerGPS() {
  while (GPS_SERIAL.available() > 0) {
    char c = GPS_SERIAL.read();
    
    if (gps.encode(c)) {
      if (gps.location.isValid()) {
        latitud = gps.location.lat();
        longitud = gps.location.lng();
      }
      
      if (gps.altitude.isValid()) {
        altitud = gps.altitude.meters();
      }
      
      if (gps.speed.isValid()) {
        velocidad = gps.speed.kmph();
      }
    }
  }
}


void leerMPU() {
  mpu.readSensor();
  
  // Obtener datos de acelerómetro (en g)
  float accelX = mpu.getAccelX_mss() / 9.81;
  float accelY = mpu.getAccelY_mss() / 9.81;
  float accelZ = mpu.getAccelZ_mss() / 9.81;
  
  // Calcular magnitud de aceleración (fuerza G total)
  aceleracion = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
  
  // Calcular PITCH (actitud/cabeceo) desde acelerómetro
  actitud = atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ)) * 57.2958;
  
  // Calcular ROLL para compensación del magnetómetro
  float roll = atan2(-accelX, accelZ) * 57.2958;
  
  // Obtener magnetómetro
  float magX = mpu.getMagX_uT();
  float magY = mpu.getMagY_uT();
  float magZ = mpu.getMagZ_uT();
  
  // Compensar inclinación del magnetómetro
  float pitch_rad = actitud * 0.0174533;
  float roll_rad = roll * 0.0174533;
  
  float magX_comp = magX * cos(pitch_rad) + magZ * sin(pitch_rad);
  float magY_comp = magX * sin(roll_rad) * sin(pitch_rad) + 
                    magY * cos(roll_rad) - 
                    magZ * sin(roll_rad) * cos(pitch_rad);
  
  // Calcular YAW (rumbo)
  rumbo = atan2(magY_comp, magX_comp) * 57.2958;
  
  // Normalizar rumbo a 0-360°
  if (rumbo < 0) rumbo += 360;
}


void actualizarTelemetria() {
  telemetry[0] = latitud;      
  telemetry[1] = longitud;     
  telemetry[2] = rumbo;        
  telemetry[3] = altitud;      
  telemetry[4] = actitud;      
  telemetry[5] = aceleracion;  
  
  // Debu STATION
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    Serial.println("\n=== TELEMETRÍA ===");
    Serial.print("GPS: "); Serial.print(latitud, 6); 
    Serial.print(", "); Serial.println(longitud, 6);
    Serial.print("Rumbo: "); Serial.print(rumbo, 1); Serial.println("°");
    Serial.print("Altitud: "); Serial.print(altitud, 1); Serial.println(" m");
    Serial.print("Actitud: "); Serial.print(actitud, 1); Serial.println("°");
    Serial.print("Aceleración: "); Serial.print(aceleracion, 2); Serial.println(" g");
    Serial.println("==================\n");
    lastPrint = millis();
  }
}

void setServoAngle(uint8_t channel, int angle) {
  if(angle < 0) angle = 0;
  if(angle > 180) angle = 180;
  
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  pwm.setPWM(channel, 0, pulse);
}