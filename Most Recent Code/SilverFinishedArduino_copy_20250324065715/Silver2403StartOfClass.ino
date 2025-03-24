#include "Arduino_LED_Matrix.h"
#include "WiFiS3.h"
#include <PID_v1.h>

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

// 0 is auto, 1 is controlled by GUI, 2 is PID, 3 is testing mode, 4 is follow object mode
unsigned short driveMode = 0;

bool pidDrive = false; // default to PID control being on

unsigned long prevMillis = 0;

float Sdistance = 10;
unsigned long lastTime = 0, lastTime1;
const unsigned long intervalTime = 500;

int speedValue = 30; // Default speed value

float speed;

bool followObject = false;

// Target distance for follow mode (in cm)
const float TARGET_DISTANCE = 15.0;
// Tolerance for distance (in cm)
const float DISTANCE_TOLERANCE = 2.0;

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

double KpL = 1, KpR = 1; 
double KiL = 1, KiR = 1;
double KdL = 0.2, KdR = 0.2;

double SetpointLeft, InputLeft, OutputLeft;
double SetpointRight, InputRight, OutputRight;

unsigned long prevTimeL = 0, prevTimeR = 0;

int targetSpeed = 0;
unsigned long elapsedTime;
double output;
//float setpoint = 15;

PID LeftPID(&InputLeft, &OutputLeft, &SetpointLeft, KpL, KiL, KdL, DIRECT);
PID RightPID(&InputRight, &OutputRight, &SetpointRight, KpR, KiR, KdR, DIRECT);

WiFiServer server(3000);

double Setpoint, Input, Output;

void setup() {
  LeftPID.SetMode(AUTOMATIC);
  LeftPID.SetOutputLimits(50, 255);

  RightPID.SetMode(AUTOMATIC);
  RightPID.SetOutputLimits(50, 255);

  // Set default speed setpoints
  SetpointLeft = 30; // Default speed in RPM
  SetpointRight = 30; // Default speed in RPM

  Serial.begin(115200);

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

float totDistRight = 0;

float rightDistTravelled() {
  float rightRotations = (float)rightCount / (float)PPRev;
  float distRight = rightRotations * (PI * wheelDiam);
  rightCount = 0;

  totDistRight += distRight;

  return totDistRight;
  
}

float totDistLeft = 0;

float leftDistTravelled() {
  float leftRotations = (float)leftCount / (float)PPRev;
  float distLeft = leftRotations * (PI * wheelDiam);
  leftCount = 0;
  
  totDistLeft += distLeft;

  return totDistLeft;
}

float lastDistLeft = 0;  
float lastDistRight = 0;
float lastTimeLeft = 0;
float lastTimeRight = 0;

float LspeedRPM = 0;

int wait = 1000;

float getSpeedLeft() {
  unsigned long currentTime = millis();
  if (currentTime - lastTimeLeft >= wait) {
    LspeedRPM = (leftCount / PPRev) * (60000.0/wait);
    leftCount = 0;
    lastTimeLeft = currentTime;
  }
  
  return LspeedRPM;
}

float RspeedRPM = 0;

float getSpeedRight() {
  unsigned long currentTime = millis();
  if (currentTime - lastTimeRight >= wait) {
    RspeedRPM = (rightCount / PPRev) * (60000.0/wait);  
    rightCount = 0;                                                                     
    lastTimeRight = currentTime;

  }
  return RspeedRPM;
}

void remoteControl() {
  WiFiClient client = server.available();

  if (client && client.connected()) {
    //float actualSpeed = getSpeed();
    
    if (lastCommand == "Stop"){
      Motormove(1, 1, 1);
      // Reset PID setpoints to zero to prevent movement
      SetpointLeft = 0;
      SetpointRight = 0;
    }
    else if (lastCommand == "Forward"){
      Motormove(1, 1, 0); 
      // Use the current speedValue (in RPM) for direct PWM control
      int pwmValue = rpmToPWM(speedValue);
      LMPower(pwmValue);
      RMPower(pwmValue);
      Serial.println("Forward at " + String(speedValue) + " RPM (PWM: " + String(pwmValue) + ")");
    }
    else if (lastCommand == "Left"){
      Motormove(1, 1, 0);
      // For turning, calculate PWM based on speedValue
      LMPower(0);
      RMPower(rpmToPWM(speedValue));
      Serial.println("Left turn at " + String(speedValue) + " RPM");
    }
    else if (lastCommand == "Right"){
      Motormove(1, 1, 0);
      // For turning, calculate PWM based on speedValue
      LMPower(rpmToPWM(speedValue));
      RMPower(0);
      Serial.println("Right turn at " + String(speedValue) + " RPM");
    }
    else if (lastCommand == "Backward") {
      Motormove(0, 0, 0);
      // Use the current speedValue (in RPM) for direct PWM control
      int pwmValue = rpmToPWM(speedValue);
      LMPower(pwmValue);
      RMPower(pwmValue);
      Serial.println("Backward at " + String(speedValue) + " RPM (PWM: " + String(pwmValue) + ")");
    
    }
  }
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

int rpmToPWM(float rpm) {
  if (rpm <= 0) return 0;
  int minStartPWM = 50;
  int pwm = minStartPWM + (255 - minStartPWM) * (rpm / 60.0);
  return constrain(pwm, minStartPWM, 255);
}

unsigned long lastTime_speed = 0;

float getSpeed() {
  unsigned long currentTime_speed = millis();
  float timeDifference = (currentTime_speed - lastTime_speed) / 1000.0; // to seconds
  lastTime_speed = currentTime_speed;
 
  float dist = distTravelled();
  float distSinceLast = dist - lastDist;

  Serial.println("Dist since last check: " + String(distSinceLast));
  
  float speed = (distSinceLast) / timeDifference; // Speed in cm/s
  lastDist = dist; 
  
  return speed;
}

unsigned long prevMillis3 = 0;

void pidSpeedControl() {
  if (!pidDrive) return; // Skip PID control if it's disabled
  
  LeftPID.SetTunings(KpL, KiL, KdL);
  RightPID.SetTunings(KpR, KiR, KdR);

  // Process left motor independently
  InputLeft = getSpeedLeft();
  LeftPID.Compute();
  LMPower(OutputLeft);
  
  // Process right motor independently  
  InputRight = getSpeedRight();
  RightPID.Compute();
  RMPower(OutputRight);

  unsigned long millisNow = millis();
  if (millisNow - prevMillis3 >= 1000) {
    prevMillis3 = millisNow;
    Serial.println("Speed Left: " + String(InputLeft) + " RPM, Power: " + String(OutputLeft));
    Serial.println("Speed Right: " + String(InputRight) + " RPM, Power: " + String(OutputRight));
  }
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

  // Process distance measurement
  float distanceToObject = SSensor(10);
  
  // Handle WiFi client
  WiFiClient client = server.available();
  if (client && client.connected()) {
    client.println("Sensor:" + String(distanceToObject));
  }
  
  // Process commands
  String command = readCommand();  
  if (command != "" && command != prevCommand) {
    if (command.startsWith("SPEED:")) {
      int speedPos = command.indexOf("SPEED:");
      String speedVal = command.substring(speedPos + 6);
      speedValue = speedVal.toInt(); // This is now in RPM (0-60)
      
      // Check and constrain the RPM value to a valid range
      speedValue = constrain(speedValue, 0, 60);
      Serial.println("New Speed: " + String(speedValue) + " RPM");
      
      // Convert RPM to PWM since we're not using PID
      int newspeed = rpmToPWM(speedValue);
      LMPower(newspeed);
      RMPower(newspeed);
    }
    else if (command == "PID") {
      // We're ignoring PID commands as requested
      pidDrive = true;
      Serial.println("PID contol enabled");
    }
    else if (command == "NOPID") {
      pidDrive = false;
      Serial.println("PID control disabled");
    }
    else if (command == "Self") {
      driveMode = 0;
      Serial.println("Switched to Self driving mode");
    } 
    else if (command == "Remote") {
      driveMode = 1;
      Serial.println("Switched to Remote control mode");
      pidDrive = false; // No PID as requested
    }
    else if (command == "Follow") {
      driveMode = 4; // New mode for object following
      Serial.println("Switched to Follow object mode");
      pidDrive = false;
    }
    prevCommand = command;
  }

  pidSpeedControl();
  
  // Execute the appropriate mode
  switch (driveMode) {
    case 0: // Self driving (line following) mode
        selfDrive();
      break;
    case 1: // Remote control mode
      remoteControl();
      break;
    case 4: // Object following mode
      followObjectMode();
      break;
    default:
      // If no valid mode is set, stop motors
      Motormove(1, 1, 1);
      break;
  }
  
  // Send data updates to client periodically
  unsigned long currentMillis = millis();
  if (currentMillis - prevMillis2 >= 1000) {
    prevMillis2 = currentMillis;
    
    if (client && client.connected()) {
      // Send current status information
      client.println("Mode:" + String(driveMode));
      client.println("Speed:" + String(speedValue));
      client.println("LeftPower:" + String(LPower));
      client.println("RightPower:" + String(RPower));
      client.println("Distance:" + String(distanceToObject));
    }
  }
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
      Motormove(1, 1, 1);
    } else {
      if (Sensorleft() && Sensorright()) { //both on black
        Motormove(1, 1, 0);
        if (pidDrive) {
          // Let PID handle the speed control with equal setpoints
          SetpointLeft = speedValue;
          SetpointRight = speedValue;
          // pidSpeedControl() will be called in the main loop
        } else {
          // Use the RPM to PWM conversion for consistency
          int pwmValue = rpmToPWM(speedValue);
          RMPower(pwmValue);
          LMPower(pwmValue);
        }
      } else if (!Sensorleft() && !Sensorright()){ //both on white
        Motormove(1, 1, 0);
        if (pidDrive) {
          // Let PID handle the speed control with equal setpoints
          SetpointLeft = speedValue;
          SetpointRight = speedValue;
          // pidSpeedControl() will be called in the main loop
        } else {
          // Use the RPM to PWM conversion for consistency
          int pwmValue = rpmToPWM(speedValue);
          RMPower(pwmValue);
          LMPower(pwmValue);
        }
      }
        else if (!Sensorleft() && Sensorright()){ //sensor left white sensor right is black
        int pwmValue = rpmToPWM(speedValue);
        Motormove(1, 1, 0);
        LMPower(pwmValue);
        RMPower(0);
        }
        else if (Sensorleft() && !Sensorright()){
        int pwmValue = rpmToPWM(speedValue);
        Motormove(1, 1, 0);
        LMPower(0);
        RMPower(pwmValue);
        }
      }
    }
  }


void followObjectMode() {
  // Get current distance from sensor
  float currentDistance = SSensor(10);
  
  // Send distance info to client if connected
  WiFiClient client = server.available();
  if (client && client.connected()) {
    client.println("Sensor:" + String(currentDistance));
    client.println("Target:" + String(TARGET_DISTANCE));
  }
  
  // Check if we should stop (emergency stop)
  if (lastCommand == "Stop") {
    Motormove(1, 1, 1);
    return;
  }
  
  Serial.print("Current distance: ");
  Serial.print(currentDistance);
  Serial.print(", Target: ");
  Serial.println(TARGET_DISTANCE);
  
  // Logic for maintaining distance
  if (currentDistance >= 17) {
    selfDrive();
  } else {
    Motormove(1, 1, 1);
  }
}