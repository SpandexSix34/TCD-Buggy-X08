#include "robot_functions.h"

// Constants
const float wheelDiam = 0.06; // Diameter of wheel (in meters)
const int PPRev = 8; // Amount of interrupts per revolution of wheel
const unsigned short initLPower = 130;
const unsigned short initRPower = 130;
unsigned short maxPower = 130;
unsigned short minPower = 0;
const float TARGET_DISTANCE = 15.0;
const float DISTANCE_TOLERANCE = 2.0;
// unsigned long prevMillisLeft = 0;
// unsigned long prevMillisRight = 0;
// float currSpeedRight = 0;
// float currSpeedLeft = 0;
const unsigned short interruptDist = 0.025525;

// Variables
String lastCommand = " ";
unsigned short driveMode = 0;
bool pidDrive = true;
int speedValue = 30;
volatile int leftCount = 0;
volatile int rightCount = 0;
float totalDist = 0.0;
unsigned int LPower = initLPower;
unsigned int RPower = initRPower;
unsigned long prevMillisLeft = 0;
unsigned long prevMillisRight = 0;
float currSpeedRight = 0;
float currSpeedLeft = 0;

// PID variables 
double KpL = 3, KpR = 3;
double KiL = 1, KiR = 1;
double KdL = 0.2, KdR = 0.2;
double SetpointLeft, InputLeft, OutputLeft;
double SetpointRight, InputRight, OutputRight;

PID LeftPID(&InputLeft, &OutputLeft, &SetpointLeft, KpL, KiL, KdL, DIRECT);
PID RightPID(&InputRight, &OutputRight, &SetpointRight, KpR, KiR, KdR, DIRECT);

// Other helper variables
float totDistRight = 0;
float totDistLeft = 0;
float lastDistLeft = 0;  
float lastDistRight = 0;
float lastTimeLeft = 0;
float lastTimeRight = 0;
float lastDist = 0;
unsigned long lastTime_speed = 0;
float LspeedRPM = 0;
float RspeedRPM = 0;
int wait = 1000;

inline void digitalToggle(byte pin){
  digitalWrite(pin, !digitalRead(pin));
}

float SSensor(int interval) {
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2); 
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  unsigned long duration = pulseIn(EchoPin, HIGH);
  float distance = duration * 0.034 / 2;
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

void Motormove(bool ControlLeft, bool ControlRight, bool toStop){ 
  if (toStop){
    Motorleft(1,1); 
    Motorright(1,1); // stop if Motormove(0,0,1) will stop it
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
  if (digitalRead(Pin12) == HIGH) { // black
    return true;
  } else {
    return false;
  }
}

bool Sensorright() {
  if (digitalRead(Pin13) == HIGH) { // black
    return true;
  } else {
    return false;
  }
}

void LMPower(int power){
  analogWrite(Pin6, power);
}

void RMPower(int power){
  analogWrite(Pin5, power);
}

void turnLeft(){
  analogWrite(Pin5, maxPower); // Right motor
  RPower = maxPower;
  analogWrite(Pin6, minPower); // left motor
  LPower = minPower;
}

void turnRight(){
  analogWrite(Pin5, minPower); // Right motor
  RPower = minPower;
  analogWrite(Pin6, maxPower); // left motor
  LPower = maxPower;
}

void speedLeft() { 
  unsigned long currentMillis = millis();
  unsigned long deltaTLeft = (currentMillis - prevMillisLeft)/1000; // seconds

  currSpeedLeft = deltaTLeft/interruptDist; // cm/s
  Serial.println("Left: " + String(currSpeedLeft));
}

void speedRight() {
  unsigned long currentMillis = millis();
  unsigned long deltaTRight = (currentMillis - prevMillisRight)/1000; // seconds

  currSpeedRight = deltaTRight/interruptDist; // cm/s
  Serial.println("Right: " + String(currSpeedRight));
}

String readCommand() {
  WiFiClient client = server.available();
  
  if (client && client.connected() && client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();
    Serial.println("Command received: " + command);
    lastCommand = command;
    return command;
  }
  return ""; // Return empty string if no command available
}

float rightDistTravelled() {
  float rightRotations = (float)rightCount / (float)PPRev;
  float distRight = rightRotations * (PI * wheelDiam);
  rightCount = 0;

  totDistRight += distRight;

  return totDistRight;
}

float leftDistTravelled() {
  float leftRotations = (float)leftCount / (float)PPRev;
  float distLeft = leftRotations * (PI * wheelDiam);
  leftCount = 0;
  
  totDistLeft += distLeft;

  return totDistLeft;
}

float getSpeedLeft() {
  unsigned long currentTime = millis();
  if (currentTime - lastTimeLeft >= wait) {
    LspeedRPM = (leftCount / PPRev) * (60000.0/wait);
    leftCount = 0;
    lastTimeLeft = currentTime;
  }
  
  return LspeedRPM;
}

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

float distTravelled() {
  float leftRotations = (float)leftCount / (float)PPRev;
  float rightRotations = (float)rightCount / (float)PPRev;

  float distLeft = leftRotations * (PI * wheelDiam);
  float distRight = rightRotations * (PI * wheelDiam);
  float avgDist = (distLeft + distRight) / 2;

  totalDist += avgDist;
  leftCount = 0;
  rightCount = 0;

  return totalDist;
}

int rpmToPWM(float rpm) {
  if (rpm <= 0) return 0;
  int minStartPWM = 50;
  int pwm = minStartPWM + (255 - minStartPWM) * (rpm / 60.0);
  return constrain(pwm, minStartPWM, 255);
}

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

  static unsigned long prevMillis3 = 0;
  unsigned long millisNow = millis();
  if (millisNow - prevMillis3 >= 1000) {
    prevMillis3 = millisNow;
    Serial.println("Speed Left: " + String(InputLeft) + " RPM, Power: " + String(OutputLeft));
    Serial.println("Speed Right: " + String(InputRight) + " RPM, Power: " + String(OutputRight));
  }
}

void selfDrive() {
  WiFiClient client = server.available();
  float distance = SSensor(10);
  
  if (client && client.available()) {
    client.println("Sensor:" + String(distance));
    Serial.println("dist from object" + String(distance));
  }

  String command = readCommand();

  if (lastCommand == "Stop"){
    Motormove(1, 1, 1);
    return;
  } else {
    if (distance < 10) {
      Motormove(1, 1, 1);
    } else {
      if (Sensorleft() && Sensorright()) { // both on black
        if (pidDrive) {
          // Let PID handle the speed control with equal setpoints
          SetpointLeft = SetpointRight;
          // pidSpeedControl() will be called in the main loop
        } else {
          // Use the RPM to PWM conversion for consistency
          int pwmValue = rpmToPWM(speedValue);
          RMPower(pwmValue);
          LMPower(pwmValue);
        }
        Motormove(1, 1, 0);
      } else if (!Sensorleft() && !Sensorright()){ // both on white
        Motormove(1, 1, 0);
        if (pidDrive) {
          // Let PID handle the speed control with equal setpoints
          SetpointLeft = SetpointRight;
          // pidSpeedControl() will be called in the main loop
        } else {
          // Use the RPM to PWM conversion for consistency
          int pwmValue = rpmToPWM(speedValue);
          RMPower(pwmValue);
          LMPower(pwmValue);
        }
      }
      else if (!Sensorleft() && Sensorright()){ // sensor left white sensor right is black
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

void selfDriveNoPID() {
  int distance = SSensor(10);
  
  // Emergency stop if too close to an object
  if (distance < 10) {
    Motormove(1, 1, 1);
    return;
  }
  
  // Stop if commanded
  if (lastCommand == "Stop") {
    Motormove(1, 1, 1);
    return;
  }
  
  // Line following logic
  if (Sensorleft() && Sensorright()) { // Both on black
    Motormove(1, 1, 0); // Move forward
    int pwmValue = rpmToPWM(speedValue);
    RMPower(pwmValue);
    LMPower(pwmValue);
  } 
  else if (!Sensorleft() && !Sensorright()) { // Both on white
    Motormove(1, 1, 0); // Move forward
    int pwmValue = rpmToPWM(speedValue);
    RMPower(pwmValue);
    LMPower(pwmValue);
  }
  else if (!Sensorleft() && Sensorright()) { // Left on white, right on black
    Motormove(1, 1, 0);
    int pwmValue = rpmToPWM(speedValue);
    LMPower(pwmValue); // Left wheel moves, right stops to turn
    RMPower(0);
  }
  else if (Sensorleft() && !Sensorright()) { // Left on black, right on white
    Motormove(1, 1, 0);
    int pwmValue = rpmToPWM(speedValue);
    LMPower(0);
    RMPower(pwmValue); // Right wheel moves, left stops to turn
  }
}

void followObjectModeNoPID() {
  // Get current distance from sensor
  float currentDistance = SSensor(10);
  
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
  if (currentDistance < (TARGET_DISTANCE - DISTANCE_TOLERANCE)) {
    // Too close to object, move backward
    Motormove(0, 0, 0);
    int distanceDiff = TARGET_DISTANCE - currentDistance;
    float desiredRPM = min(30.0f, 10.0f + distanceDiff * 1.5f);
    int powerLevel = rpmToPWM(desiredRPM);
    
    RMPower(powerLevel);
    LMPower(powerLevel);
    
    Serial.println("Too close, moving backward");
  } 
  else if (currentDistance > (TARGET_DISTANCE + DISTANCE_TOLERANCE)) {
    // Too far from object, move forward
    Motormove(1, 1, 0);
    
    int distanceDiff = currentDistance - TARGET_DISTANCE;
    float desiredRPM = min(60.0f, 20.0f + distanceDiff * 2.0f);
    int powerLevel = rpmToPWM(desiredRPM);
    
    RMPower(powerLevel);
    LMPower(powerLevel);
    
    Serial.println("Too far, moving forward");
  } 
  else {
    // At the right distance, stop
    Motormove(1, 1, 1);
    Serial.println("At target distance, holding position");
  }
}

void combinedModeNoPID(float distanceToObject) {
  // Check for emergency stop conditions
  if (lastCommand == "Stop" || distanceToObject < 8) {
    Motormove(1, 1, 1);
    return;
  }
  
  // First priority: avoid collision (object following)
  if (distanceToObject < TARGET_DISTANCE) {
    Motormove(1, 1, 1);
  }
  // If safe distance, follow line (self-driving)
  else {
    // Line following logic
    int pwmValue = rpmToPWM(speedValue);
    
    if (Sensorleft() && Sensorright()) { // Both on black
      Motormove(1, 1, 0);
      RMPower(pwmValue);
      LMPower(pwmValue);
    } 
    else if (!Sensorleft() && !Sensorright()) { // Both on white
      Motormove(1, 1, 0);
      RMPower(pwmValue);
      LMPower(pwmValue);
    }
    else if (!Sensorleft() && Sensorright()) { // Left on white, right on black
      Motormove(1, 1, 0);
      LMPower(pwmValue);
      RMPower(0);
    }
    else if (Sensorleft() && !Sensorright()) { // Left on black, right on white
      Motormove(1, 1, 0);
      LMPower(0);
      RMPower(pwmValue);
    }
    
    // Slow down if getting close to target distance
    if (distanceToObject < TARGET_DISTANCE + 5) {
      pwmValue = rpmToPWM(speedValue * 0.7); // Reduce to 70% of normal speed
      RMPower(pwmValue);
      LMPower(pwmValue);
      Serial.println("Combined mode: Approaching object, slowing down");
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
  if (currentDistance < (TARGET_DISTANCE - DISTANCE_TOLERANCE)) {
    // We're too close to the object, stop
    Motormove(1, 1, 1);
    Serial.println("Too close, stopping");
    
    if (client && client.connected()) {
      client.println("Status:TooClose");
    }
  } 
  else if (currentDistance > (TARGET_DISTANCE + DISTANCE_TOLERANCE)) {
    // We're too far from the object, move forward
    Motormove(1, 1, 0);
    
    // Calculate power based on how far we are from target
    // The further away we are, the faster we go
    int distanceDiff = currentDistance - TARGET_DISTANCE;
    
    // Calculate desired RPM based on distance
    float desiredRPM = min(60.0f, 20.0f + distanceDiff * 2.0f); // Scale RPM with distance, max 60 RPM
    
    // Convert to PWM
    int powerLevel = rpmToPWM(desiredRPM);
    
    RMPower(powerLevel);
    LMPower(powerLevel);
    
    Serial.print("Too far, moving forward with RPM: ");
    Serial.print(desiredRPM);
    Serial.print(" (PWM: ");
    Serial.print(powerLevel);
    Serial.println(")");
    
    if (client && client.connected()) {
      client.println("Status:Moving");
    }
  } 
  else {
    // We're at the right distance, stop
    Motormove(1, 1, 1);
    Serial.println("At target distance, holding position");
    
    if (client && client.connected()) {
      client.println("Status:AtTarget");
    }
  }
}
