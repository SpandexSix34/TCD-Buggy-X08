#include "Arduino_LED_Matrix.h"
#include "WiFiS3.h"

ArduinoLEDMatrix matrix;

char ssid[] = "Pixel 9";        
char password[] = "nfhu5637"; 
WiFiClient client; // Wifi client is called client from now on. any time we call client.x we call WiFiClient


// inline void digitalToggle(byte pin){
//   digitalWrite(pin, !digitalRead(pin)); // not used in code
// }

const byte Pin8 = 8;
const byte Pin9 = 9;
const byte Pin10 = 10;
const byte Pin11 = 11;
const byte Pin12 = 12;
const byte Pin13 = 13;
const byte RMove = 3;
const byte Pin5 = 5;
const byte Pin6 = 6;
const byte TrigPin = 7; // recives echo
const byte EchoPin = 4; // sends out an echo 
const byte LEncoder = 2;
const byte REncoder = 3; // Setting all the pins, bytes because small


unsigned const short initLPower = 115;
unsigned const short initRPower = 115;
unsigned const short maxPower = 115;
unsigned const short minPower = 70; // Setting min max and inital powers

unsigned short driveMode = 0; //0 is auto, 1 is controlled by GUI. Not a bool because we want more options.

unsigned long prevMillis = 0;

//Wheel specifications
const float wheelDiam = 0.06; // Diameter of wheel (not measured very accurately)
const int PPRev = 8; //Amount of interrupts per revolution of wheel
volatile int leftCount = 0; // Volatile is used whenever something can be changed by something that is not within the code itself (such as an interrupt from the motor) 
volatile int rightCount = 0; // interrupt stops current from flowing ^^
float totalDist = 0.0;  // Total distance travel

unsigned int LPower = initLPower;
unsigned int RPower = initRPower; // sets powers at the start to initial powers

int prevLMPower = 20; 
int prevRMPower = 20; // saves previous power for each motor

void  countLeft() { 
  leftCount++;  
}
void countRight() {   //Function to increment count of motor turns
  rightCount++;  
}

WiFiServer server(3000); // Port 3000 (chosen at random)

void setup() {
  Serial.begin(9600); // can change power 9600 times per second

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
  pinMode(LEncoder, INPUT_PULLUP);
  pinMode(REncoder, INPUT_PULLUP); // INPUT_PULLUP pins always read HIGH as their initial state. << connects resistor within the Arduino to the port (suplies voltage)

  //pinMode(RWheelEncoder)

  matrix.begin(); // starts the matrix, for fun

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

  WiFiClient client = server.available(); // returns true or false

  // Start the server
  server.begin();
  Serial.println("Server started");
  Serial.println(WiFi.localIP()); // begin server, tell user and print IP of device

  char prevCommand; // previous command (unsure if used)

  attachInterrupt(digitalPinToInterrupt(LEncoder), countLeft, RISING); // we set up an interrupt on the pins to be interrupted (2, 3). RISING triggers when going LOW to HIGH.
  attachInterrupt(digitalPinToInterrupt(REncoder), countRight, RISING); // whenever this signal gets interrupted, we increment the correct counters. 
}

char command = ' '; // unused

void Motorleft(bool toGoFoward, bool toGoBack){ // Motor left and right functions to control whether the left motor is going forwards or backwards.
  if (toGoFoward){                              // 0, 1 means we are going backwards. 1,0 because we are going ofrwards. 1,1 would make it not move
      digitalWrite(Pin8, HIGH);                 // but within our code we are "not allowed" to call Motorleft without using another function
  }else{                                        // and none of our functions do it except for the ones specifically meant to stop.
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

void Motormove(bool ControlLeft, bool ControlRight,bool toStop){  // Actually calls for motors to move. This is all of the options of waht can happen within the H bridge
  if (toStop){                                                    // Please look into how the H bridge actually works. Basically for any given side:
    Motorleft(1,1);                                               // if input = 1, 0 we get Vcc and ground sending the motor forwards. 0, 1 sends it backwards
    Motorright(1,1); //stop ie if Motormove (0,0,1) will stop it  // By default, 1,1 outputs nothing
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
  if (digitalRead(12) == HIGH) { //returns true if left sensor is sensing
    return true;
  } else {
    return false;
  }

}

bool Sensorright() {
 if (digitalRead(13) == HIGH) { //returns true if right sensor is sensing
    return true;
  } else {
    return false;
  }

}


void LMPower(int power){
  analogWrite(Pin6, power); // sets power (analog avlue) to left motor
  //Serial.print("Left: ");
  //Serial.println(power); 
}

void RMPower(int power){
    analogWrite(Pin5, power); // sets power (analog avlue) to right motor
    //Serial.print("Right: ");
    //Serial.println(power);
}

void turnLeft(){ // slowing down one motor to turn but not slow down to a crawl
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

float SSensor (int interval) { // Sound sensor. will explain in person to anyone LITERALLY JUST ECHOLOCATION 
                               // We send out sound beyond the human ear's perception and see how long it takes to come back
                               // we are batman
  digitalWrite(TrigPin, LOW);
  digitalWrite(TrigPin, HIGH); 

  delay(10); // delay to see how long it takes to reach us? explain pls
    
  digitalWrite(TrigPin, LOW);
  unsigned long duration = pulseIn(EchoPin, HIGH); //high starts the timing (microsecond)
  float distance = duration * 0.034 / 2; //converts the time to a speed
  return distance;
}

void selfDrive () { // auto-drive function
  //Serial.println("Entered F loop");

  //Serial.println(duration);
  float distance =  SSensor(10);

  client.println("D:" + String(distance));

  if (distance < 10) {
    //String sdistance;
    //sdistance = String(distance);
    
    //client.write("00000");
    // Serial.println("Sensor decteded obstacle");
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
} 


// void processCommand() {
//   if(!client || !client.connected()) {
//     client = server.available();

//   }
//   if(client.connected() && client ) {
//     //Serial.println("New client connected");
//     if (client.available()) {
//       char command = client.read();
//       Serial.println(command);
//     }
//     // while(client.available()){
//     // char command = client.read();
//     // if ( command != prevCommand) {
//     //     prevCommand = command;
//     Serial.println(command);
//     if (command != 'S' || command != 'F'){
//       char prevCommand;
//       command = prevCommand;
//     }
//     if (command == 'F') {
//                 selfDrive();
//                 prevCommand = 'F';
//             }
//             else if (command == 'S'){
//               Motormove (0, 0, 1);
//               prevCommand = 'S';
//             }
//     Serial.print("Now: ");
//     Serial.println(command); 
       
   // }     

      //Serial.println(prevCommand);
      // Serial.println("Before F loop");
      // if (prevCommand == 'F') {
      //           selfDrive();
      //       }
      //       else if (preCommand == 'S'){
      //         Motormove (0, 0, 1);
      //       } 
//     }
    
// }



const uint32_t happy[] = {
    0x1f820,
		0x40000900,
		0x90090000,
		66
};



void loop() {

  matrix.loadFrame(happy);
      
  // Print the message to Serial Monitor
  //WiFiClient client = server.available();
  //processCommand();

  //TODO Make the following code a function as oppose to running within loop. For neatness.

  float leftRotations = (float)leftCount / PPRev;
  float rightRotations = (float)rightCount / PPRev;

  float distLeft = leftRotations * (PI * wheelDiam);
  float distRight = rightRotations * (PI * wheelDiam);
  float avgDist = (distLeft + distRight) / 2;

  totalDist += avgDist;
  leftCount = 0;
  rightCount = 0;

  Serial.println(totalDist);

  if(!client || !client.connected()) {
    client = server.available();
    if(client){
      Serial.println("Client connected.");
    }

  }
  if(client.connected() && client ) {
    //Serial.println("New client connected");
    if (client.available()) {
      command = client.read();
      Serial.println(command);
    }
      if (command == 'F') {
        selfDrive();
          }
            else if (command == 'S'){
              Motormove (0, 0, 1);
              delay(500);
              client.println("S:" + String(totalDist));

    }
  }
}
