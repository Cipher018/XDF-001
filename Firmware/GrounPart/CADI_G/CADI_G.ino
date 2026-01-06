//Import libraries
#include <Bluepad32.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>

//Define telemetry
float Position = 0;
float Speed = 0;
float Altitude = 0;
float Attitude = 0;
int Battery = 0;

//Configuration of nRF24l01
#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);
const byte pipeTX[6] = "CMD01"; // TX Address 
const byte pipeRX[6] = "TEL01"; // RX Address 

//Setup payload
int16_t commands[4];   
float telemetry[5];    

//Configuration of HID
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

//Review status of connection HID-Ground
void onConnectedController(ControllerPtr ctl) {
  Serial.println("HID ONLINE");
  myControllers[0] = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println("HID OFFLINE");
  myControllers[0] = nullptr;
}

void setup() {
  Serial.begin(115200); 
  
  // Radio block
  if (!radio.begin()) {
    Serial.println("ERROR: nRF24L01 no detectado");
    while(1); 
  }
  
  radio.enableAckPayload(); // Allows two direction communication
  radio.setRetries(3, 5);
  radio.openWritingPipe(pipeTX);    
  radio.openReadingPipe(1, pipeRX); 
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening(); 

  Serial.println("Ground Station: ONLINE");

  //HID block
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys(); // Opcional: limpiar emparejamientos previos
}

void loop() {
  bool dataUpdated = BP32.update();
  
  if (dataUpdated && myControllers[0] && myControllers[0]->isConnected()) {
    //READ HID
    int axisX = myControllers[0]->axisX();
    int throttle = myControllers[0]->throttle();
    int axisRY = myControllers[0]->axisRY();
    int axisRX = myControllers[0]->axisRX();

    //Parse commands
    int Yaw = map(axisX, -511, 512, -40, 40);
    int Power = map(throttle, 0, 1024, 0, 180);
    int Pitch = map(axisRY, -511, 512, -30, 30);
    int Roll = map(axisRX, -511, 512, -45, 45);

    // Llenar array de comandos
    commands[0] = Yaw;
    commands[1] = Power;
    commands[2] = Pitch;
    commands[3] = Roll;

    // Send commands
    radio.stopListening(); 
    bool ok = radio.write(&commands, sizeof(commands));
    
    if (ok) {
      Serial.println("✓ Comandos enviados correctamente");
      
      radio.startListening(); 
      
      unsigned long timeout = millis();
      while (!radio.available() && millis() - timeout < 200) {
      }
      
      if (radio.available()) {
        radio.read(&telemetry, sizeof(telemetry));
        
        Position = telemetry[0];
        Speed = telemetry[1];
        Altitude = telemetry[2];
        Attitude = telemetry[3];
        Battery = telemetry[4]; 


        Serial.println("--- Telemetría recibida ---");
        Serial.print("Position: "); Serial.println(Position);
        Serial.print("Speed: "); Serial.println(Speed); // ✅ Sin "+"
        Serial.print("Altitude: "); Serial.println(Altitude);
        Serial.print("Attitude: "); Serial.println(Attitude);
        Serial.print("Battery: "); Serial.print(Battery); Serial.println("%");
        Serial.println();
      } else {
        Serial.println("⚠ No se recibió telemetría");
      }
      
      radio.stopListening(); 
      
    } else {
      Serial.println("✗ RF FAILURE - No se pudo enviar");
    }
  }

  delay(50); 
}