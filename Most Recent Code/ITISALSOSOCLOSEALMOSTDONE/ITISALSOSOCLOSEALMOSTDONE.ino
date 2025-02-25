#include "Arduino_LED_Matrix.h"
#include "WiFiS3.h"

ArduinoLEDMatrix matrix;

char ssid[] = "Pixel 9";        // your network SSID (name)
char password[] = "nfhu5637"; 

// CHANGE THESE TO MATCH YOUR WIRING, THEN DELETE THE PREVIOUS "#error" LINE
inline void digitalToggle(byte pin){
  digitalWrite(pin, !digitalRead(pin));
}

const short Pin8 = 8;
const short Pin9 = 9;
const short Pin10 = 10;
const short Pin11 = 11;
const short Pin12 = 12;
const short Pin13 = 13;
const short RMove = 3;
const short Pin5 = 5;
const short Pin6 = 6;
const short TrigPin = 7;
const short EchoPin = 4;

String lastCommand = "stop";

// bool LeftEye = 0;
// bool RightEye = 0;
// bool prevLeft = 1;
// bool prevRight = 1;

unsigned const short initLPower = 100;
unsigned const short initRPower = 100;
unsigned const short maxPower = 100;
unsigned const short minPower = 55;

unsigned short driveMode = 1; //0 is auto, 1 is controlled by GUI. Not a bool because we want more options.

unsigned int LPower = initLPower;
unsigned int RPower = initRPower;
int prevLMPower = 20; 
int prevRMPower = 20;

WiFiServer server(3000);

void setup() {
  Serial.begin(9600);

  pinMode( Pin8, OUTPUT);
  pinMode( Pin9, OUTPUT);
  pinMode( Pin10, OUTPUT);
  pinMode( Pin11, OUTPUT);
  pinMode( Pin12, INPUT);
  pinMode( Pin13, INPUT);
  pinMode( Pin5, OUTPUT);
  pinMode( Pin6, OUTPUT);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);

  matrix.begin();

  LMPower(LPower);
  RMPower(RPower);

  while( !Serial) {
    ;
  } //makes sure the program doesnt begin until the serial monitor opens

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Start the server
  server.begin();
  Serial.println("Server started");
  Serial.println(WiFi.localIP());
}


void Motorleft(bool toGoFoward, bool toGoBack){
  if (toGoFoward){
      digitalWrite(Pin8, HIGH);
  }else{
      digitalWrite(Pin8, LOW);
  }
  if (toGoBack){
      digitalWrite(Pin9, HIGH);
  }else{
      digitalWrite(Pin9, LOW);
  }
}

void Motorright(bool toGoFoward, bool toGoBack){
  if (toGoFoward){
      digitalWrite(Pin10, HIGH);
  }else{
      digitalWrite(Pin10, LOW);
  }
  if (toGoBack){
      digitalWrite(Pin11, HIGH);
  }else{
      digitalWrite(Pin11, LOW);
  }
}

void Motormove(bool ControlLeft, bool ControlRight,bool toStop){ 
  if (toStop){
    Motorleft(1,1); 
    Motorright(1,1); //stop ie if Motormove (0,0,1) will stop it
  } else{
    if (ControlLeft && ControlRight){
      Motorleft(1,0);
      Motorright(1,0);
    } else if (!ControlRight && !ControlLeft) {
        Motorleft(0, 1);
        Motorright(0, 1);
    } else if(ControlLeft){
        Motorleft(1,0);
    } else if(!ControlLeft){
        Motorleft(0, 1);
    } else if(ControlRight){
        Motorright(1,0);
    } else if(!ControlRight){
        Motorright(0, 1);
    } 
  }
}

bool Sensorleft() {
  if (digitalRead(12) == HIGH) { //black
    return true;
  } else {
    return false;
  }

}

bool Sensorright() {
 if (digitalRead(13) == HIGH) { //black
    return true;
  } else {
    return false;
  }

}


void LMPower(int power){
  analogWrite(Pin6, power);
  //Serial.print("Left: ");
  //Serial.println(power); 
}

void RMPower(int power){
    analogWrite(Pin5, power);
    //Serial.print("Right: ");
    //Serial.println(power);
}

void turnLeft(){
    analogWrite(Pin5, maxPower + 20); //Right motor
    RPower = maxPower;
    analogWrite(Pin6, minPower); //left motor
    LPower = minPower;
}

void turnRight(){
    analogWrite(Pin5, minPower); //Right motor
    RPower = minPower;
    analogWrite(Pin6, maxPower + 20); //left motor
    LPower = maxPower;
}

void hardTurnLeft(){
  
}

const uint32_t happy[] = {
    0x1f820,
		0x40000900,
		0x90090000,
		66
};

void loop() {

  matrix.loadFrame(happy);

  WiFiClient client = server.available();

    if (client) {
      Serial.println("New client connected");
      while (client.connected()) {
        if (driveMode == 0) { 

          //String command = client.readStringUntil('\n'); // Read the incoming message
          //Serial.println("Received: " + command);       // Print the message to Serial Monitor
        
          // if ( command == "Remote") { 
          //   driveMode = 1;
          //   //Serial.println("Remote Mode On");
          // }
          
          //Serial.println("Entered auto-drive loop");
          digitalWrite(TrigPin, LOW);
          digitalWrite(TrigPin, HIGH);
          delay(10);  //zawg this delays us every single time TT, i.e. our code is running on 10 milliseconds delay every time because it wants to check this sensor
          digitalWrite(TrigPin, LOW);
          unsigned long duration = pulseIn(EchoPin, HIGH); //high starts the timing (microsecond)
          int distance = duration * 0.034 / 2; //converts the time to a speed

          //Serial.println(duration);
          //Serial.println(distance);
        
          if (distance < 10) {
            Serial.println("Sensor decteded obstacle");
            Motormove(1, 1, 1);
          } else {
            Serial.println(" Entered post distance-check loop");

            if (Sensorleft() && Sensorright()) { //both on black
            LMPower( maxPower); //set and print
            RMPower( maxPower); //set and print
            LPower = maxPower;
            RPower = maxPower;
            Serial.println("No sensor on line.");
            Motormove(1, 1, 0);
          } else if (!Sensorleft() && !Sensorright()){ //both on white
              //slow down and keep going
              LPower = minPower;
              RPower = minPower;
              Serial.println("Both sensor on line.");
              Motormove(1, 1, 0);
          }
            else if ( !Sensorleft() && Sensorright()){ //sensor left white sensor right is black
              Serial.println("Left sensor on line.");
              turnLeft();
              Serial.println(LPower);
              Serial.println(RPower);
              //delay(50);
              Serial.println("delay done"); 
          } else if ( Sensorleft() && !Sensorright()){
              Serial.println("Right sensor on line.");
              turnRight();
              Serial.println(LPower);
              Serial.println(RPower);
              //delay(50);
              Serial.println("delay done");
                }
          }
          } else if (driveMode == 1);
            if (client.available()) {
              String command = client.readStringUntil('\n'); // Read the incoming message
              Serial.println("Received: " + command);       // Print the message to Serial Monitor
          
              if ( command == "Remote") {
                driveMode = 1;
                Serial.println("Remote Mode On");
              } 
              if ( command == "Self") {
                driveMode = 0;
                Serial.println("Self-Driving Mode On");
              }
              if (command == "Forward") {
                RMPower(maxPower);
                LMPower(maxPower);
              
                Motormove(1, 1, 0);
              }
              if (command == "Left") {
                RMPower(maxPower);
                LMPower(maxPower);
                Motormove(0, 1, 0);
              }
              if (command == "Right") {
                RMPower(maxPower);
                LMPower(maxPower);
                Motormove(1, 0, 0);
              }
              if (command == "Back") {
                RMPower(maxPower);
                LMPower(maxPower);

                Motormove(0, 0, 0);
              }
              if (command == "Stop") {
                Motormove(1, 1, 1);
                }  
            } 
    } 

      client.stop(); // Close the connection
      Serial.println("Client disconnected");
  }
}
    // if ( digitalRead( LEYE ) == HIGH){
    //   LeftEye = 1;
    // } else {
    //   LeftEye = 0;
    // }
    // if ( digitalRead( REYE ) == HIGH){
    //   RightEye = 1;
    // } else {
    //   RightEye = 0;
    // }

    // if (LeftEye != prevLeft) {
    //     Serial.print("left ");
    //     digitalToggle ( Pin8);
    //     prevLeft = LeftEye;
    //   } 
    // if (RightEye != prevRight) {
    //     Serial.print("right ");
    //     digitalToggle ( Pin8);
    //     digitalToggle (RMove);
    //     prevRight = RightEye;
    // }
//   }
