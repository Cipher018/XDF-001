D#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//define servos angles
int Servo1 = 0;
int Servo2 = 0;
int Servo3 = 0;
int Servo4 = 0;
int Servo5 = 0;
int Servo6 = 0;
int Servo7 = 0;
int Servo8 = 0;

//Configuration of nRF24l01
#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);
const byte pipeRX[6] = "CMD01"; //RX Adress
const byte pipeTX[6] = "TEL01"; //TX Adress

//Setup payload
int16_t commands[4];     // 8 bytes
float telemetry[5];    // 20 bytes

//Configuration of Servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_1  0 //Power
#define SERVO_2  1 //Yaw
#define SERVO_3  2 //Roll (left wing)
#define SERVO_4  3 //Roll (right wing)
#define SERVO_5  4 //Flaps (left wing)
#define SERVO_6  5 //Flaps (right wing)
#define SERVO_7  6 //Pitch (Left stabilizer)
#define SERVO_8  7 //Pitch (right stabilizer)

#define SERVOMIN  102 //Minimun pulse
#define SERVOMAX  512 //Maximun pulse

#define SERVO_FREQ 50 //Frequency = 50hz

void setup() {
  Serial.begin(115200);
  // Radio block
  radio.begin();
  radio.enableAckPayload(); //Allows two direction comunication
  radio.setRetries(3, 5);
  radio.openWritingPipe(pipeTX);
  radio.openReadingPipe(1, pipeRX);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  Serial.println("Airplane: ONLINE");

  //Servo driver block
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  Serial.println("Servo Driver Status: OK")
}

void loop() {
  if (radio.available()) {
    //Read commands
    radio.read(&commands, sizeof(commands));
    
    Servo1 = commands[1];
    Servo2 = commands[0];
    Servo3 = commands[3];
    Servo4 = -1*commands[3];
    Servo5 = commands[0]; //WIP
    Servo6 = commands[0]; //WIP
    Servo7 = commands[2];
    Servo8 = commands[2];
    
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
  }
}

//Funtion to set servo angle
void setServoAngle(uint8_t channel, int angle) {
  // Limit angle
  if(angle < 0) angle = 0;
  if(angle > 180) angle = 180;
  
  // Convert angle to pulse
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  
  // Enviar al servo
  pwm.setPWM(channel, 0, pulse);
}

void actualizarTelemetria()  { //IP
  // Ejemplo: enviar posiciones actuales de servos
  telemetry[0] = Servo1;  
  telemetry[1] = Servo2;  
  telemetry[2] = Servo7;  
  telemetry[3] = Servo3;  
  telemetry[4] = 100.0;   
}
