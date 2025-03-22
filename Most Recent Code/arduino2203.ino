#include "Arduino_LED_Matrix.h"
#include "WiFiS3.h"
//#include <PID_v1.h>

ArduinoLEDMatrix matrix;

char ssid[] = "Pixel 9";        // your network SSID (name)
char password[] = "nfhu5637"; 

// CHANGE THESE TO MATCH YOUR WIRING, THEN DELETE THE PREVIOUS "#error" LINE
inline void digitalToggle(byte pin){
  digitalWrite(pin, !digitalRead(pin));
}

#define Pin8 8
#define Pin9 9
#define Pin10 10
#define Pin11 11
#define Pin12 12
#define Pin13 13
#define RMove 3
#define Pin5 5
#define Pin6 6
#define TrigPin 7 // recives echo
#define EchoPin 4 // sends out an echo 
#define LEncoder 2
#define REncoder 3 // Setting all the pins, bytes because small

String lastCommand = " ";

unsigned const short initLPower = 130;
unsigned const short initRPower = 130;
unsigned short maxPower = 130;
unsigned short minPower = 0;
float distance;

unsigned short driveMode = 0; //0 is auto, 1 is controlled by GUI 2 is PID and 3 is just a testing mode because im pissed the fuck off. Not a bool because we want more options.

bool pidDrive = false;

unsigned long prevMillis = 0;

float Sdistance = 10;
unsigned long lastTime = 0, lastTime1;
const unsigned long intervalTime = 500;

float speed;

unsigned long prevMillis_speed = 0;

//Wheel specifications
const float wheelDiam = 0.06; // Diameter of wheel (not measured very accurately)
const int PPRev = 8; //Amount of interrupts per revolution of wheel
volatile int leftCount = 0; // Volatile is used whenever something can be changed by something that is not within the code itself (such as an interrupt from the motor) 
volatile int rightCount = 0; // interrupt stops current from flowing ^^
float totalDist = 0.0;  // Total distance travel

unsigned int LPower = initLPower;
unsigned int RPower = initRPower;
int prevLMPower = initLPower; 
int prevRMPower = initRPower;

int PID;
double error, sumError, rateError, lastError = 0;
double Kp = 1.5, Ki = 0.02, Kd = 0.5;

unsigned long prevTimeL = 0, prevTimeR = 0;

int targetSpeed = 0;
unsigned long elapsedTime;
double output;
//float setpoint = 15;

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
  pinMode(LEncoder, INPUT_PULLUP);
  pinMode(REncoder, INPUT_PULLUP); // INPUT_PULLUP pins always read HIGH as their initial state. << connects resistor within the Arduino to the port (suplies voltage)

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

  attachInterrupt(digitalPinToInterrupt(LEncoder), countLeft, RISING); // we set up an interrupt on the pins to be interrupted (2, 3). RISING triggers when going LOW to HIGH.
  attachInterrupt(digitalPinToInterrupt(REncoder), countRight, RISING); // whenever this signal gets interrupted, we increment the correct counters. 
}

float SSensor (int interval) { // Sound sensor. will explain in person to anyone LITERALLY JUST ECHOLOCATION 
                               // We send out sound beyond the human ear's perception and see how long it takes to come back
                               // we are batman
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2); 
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10); // delay to see how long it takes to reach us? explain pls
  digitalWrite(TrigPin, LOW);

  unsigned long duration = pulseIn(EchoPin, HIGH); //high starts the timing (microsecond)
  distance = duration * 0.034 / 2; //converts the time to a speed
  return distance;
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
    analogWrite(Pin5, maxPower); //Right motor
    RPower = maxPower;
    analogWrite(Pin6, minPower); //left motor
    LPower = minPower;
}

void turnRight(){
    analogWrite(Pin5, minPower); //Right motor
    RPower = minPower;
    analogWrite(Pin6, maxPower); //left motor
    LPower = maxPower;
}

void  countLeft() { 
  leftCount++;  
}
void countRight() {   //Function to increment count of motor turns
  rightCount++;  
}

String readCommand() {
  // Don't create a new client instance here
  WiFiClient client = server.available();
  
  if (client && client.connected() && client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();
    Serial.println("Command recieved: " + command);
    lastCommand = command;
    return command;
  }
  return ""; // Return empty string if no command available
}

void selfDrive() {
  WiFiClient client = server.available();
  if (client && client.available()) {
    client.println("Sensor:" + String(distance));
    Serial.println("dist from object" + String(distance));
  }

  String command = readCommand();

  if (lastCommand == "Stop"){
    Motormove(1, 1, 1);
    return;
  } else {
    int distance = SSensor(10);
      if (distance < 10) {
      //Serial.println("Sensor decteded obstacle");
      Motormove(1, 1, 1);
    } else {
      //Serial.println(" Entered post distance-check loop");

      if (Sensorleft() && Sensorright()) { //both on black
      RMPower(maxPower);
      LMPower(maxPower);
      Motormove(1, 1, 0);
    } else if (!Sensorleft() && !Sensorright()){ //both on white
      Motormove(1, 1, 0);
      RMPower(maxPower);
      LMPower(maxPower);
    } else if ( !Sensorleft() && Sensorright()){ //sensor left white sensor right is black
      Motormove(1, 1, 0);
      RMPower(maxPower);
      LMPower(minPower);
    } else if ( Sensorleft() && !Sensorright()){
      Motormove(1, 1, 0);
      RMPower(minPower);
      LMPower(maxPower);
      }
    }
  }
}


void remoteControl() {
WiFiClient client = server.available();

if (client && client.connected()) {

    float actualSpeed = getSpeed();
    Serial.println("Actual speed: " + String(actualSpeed));
    
    if (lastCommand == "Stop"){
      Motormove(1, 1, 1);
    }
    else if (lastCommand == "Forward"){
      Motormove(1, 1, 0);
      LMPower(maxPower);
      RMPower(maxPower);
    }
    else if (lastCommand == "Left"){
      Motormove(1, 1, 0);
      LMPower(0);
      RMPower(maxPower);
    }
    else if (lastCommand == "Right"){
      Motormove(1, 1, 0);
      LMPower(maxPower);
      RMPower(0);
    }
    else if (lastCommand == "Backward") {
      Motormove(0, 0, 0);
    }
  }
}

int goalDist = 15;

void followObj() {
  distance = SSensor(10);
  error = distance - goalDist;
  int pidOutput = Kp * error;
  int motorSpeed = constrain(targetSpeed + pidOutput, 0, 255);
  RMPower(motorSpeed);
  LMPower(motorSpeed);
}

float lastDist = 0;

float distTravelled() {
  float leftRotations = (float)leftCount / (float)PPRev;
  float rightRotations = (float)rightCount / (float)PPRev;

  float distLeft = leftRotations * (PI * wheelDiam);
  float distRight = rightRotations * (PI * wheelDiam);
  float avgDist = (distLeft + distRight) / 2;

  totalDist += avgDist;
  leftCount = 0;
  rightCount = 0;

  // Serial.println("Total Dist Calculated: " + String(totalDist));

  return totalDist;
}

unsigned long lastTime_speed = 0;

float getSpeed() {
  unsigned long currentTime_speed = millis();
  float timeDifference = (currentTime_speed - lastTime_speed) / 1000.0; // to seconds
  lastTime_speed = currentTime_speed;
 
  float dist = distTravelled();
  
  
  float speed =  (dist) / timeDifference; // Speed in cm/s
  lastDist = dist;
  dist = 0;  
  
  return speed;
}


int controlSpeed(int speed) {
  unsigned long currentTime = millis();
  double elapsedTime = currentTime - prevTimeL;

  error = targetSpeed - speed;
  sumError += error * elapsedTime;
  rateError = (error - lastError) / elapsedTime;
  
  PID = constrain(Kp * error + Ki * sumError + Kd * rateError, 50, 255);
  
  lastError = error;
  prevTimeL = currentTime;
  
  return PID;
}

const uint32_t happy[] = {
    0x1f820,
		0x40000900,
		0x90090000,
		66
};

String prevCommand = " ";

unsigned long prevMillis2 = 0;

void loop() {
  matrix.loadFrame(happy);

  WiFiClient client = server.available();

  
  //Serial.println("Distance to object is: " + String(distanceToObject));
  if (client && client.connected()) {
    float distanceToObject = SSensor(10);
    client.println("Sensor:" + String(distanceToObject));
  }
  
  // Process distance traveled

 String command = readCommand();  
  if (command != prevCommand){
    if (command.startsWith("SPEED:")) {
      pidDrive = false;
      int speedPos = command.indexOf("SPEED:");
      String speedVal = command.substring(speedPos + 6);
      int speedValue = speedVal.toInt();

      Serial.println("New Speed: " + speedValue);
      RMPower(speedValue);
      LMPower(speedValue);
      maxPower = speedValue;
    }
    else if (command == "PID") {
      driveMode = 2;
      pidDrive = true;
      Serial.println("drive mode 2");
    }
    else if (command == "Self") {
      driveMode = 0;
      pidDrive = false;
      Serial.println("drive mode 0");
    } 
    else if ( command == "Remote") {
      driveMode = 1;
      Serial.println("drive mode 1");
      pidDrive = false;
    }
    prevCommand = command;
  }
   unsigned long currentMillis = millis();
  if (currentMillis - prevMillis >= 250) {
    prevMillis = currentMillis;
        speed = getSpeed();
        Serial.println(speed);
 }

  // if (pidDrive) {
  //   RMPower(controlSpeed(10));
  //   LMPower(controlSpeed(10));
  // }
  
  // Send distance data to client periodically
  unsigned long millisNow = millis();
  if (millisNow - prevMillis2 >= 500) {
    prevMillis2 = millisNow;
    
    // Send distance data to any connected client
    if (client && client.connected()) {
      float distTrav = distTravelled();
      client.print("D:");
      client.println(String(distTrav));
      //Serial.println("Total Dist Calculated: " + String(distTrav));
      
      client.print("Speed:");
      client.println(String(speed));  

    }
 }

//   unsigned long currentMillis_speed = millis();
//   if (currentMillis_speed  - prevMillis_speed  >= 1000) {
//     prevMillis_speed  = currentMillis_speed ;
    
//     if (client && client.connected()) {
//       float speed = getSpeed();
//       client.print("Speed:");
//       client.println(String(speed));  
//     }
//  }

 


  
  // Check distance sensor
  float currentDistance = SSensor(10);
  if (currentDistance < 10) {
    if (client && client.connected()) {
      client.println("stop");
    }
  } else {
    //WiFiClient client = server.available();
    if (client && client.connected()) {
      client.println("nothing");
    }
  }

  // Execute the appropriate driving mode
  switch (driveMode) {
    case 0:
      selfDrive();
      break;
    case 1:
      remoteControl();
      break;
    case 2:
      selfDrive();
      break;
    case 3:
      // Testing mode - do nothing
      break;
  }
}






