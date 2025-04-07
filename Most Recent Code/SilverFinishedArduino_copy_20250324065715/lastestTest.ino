#include "Arduino_LED_Matrix.h"
#include "WiFiS3.h"
#include "PID_v1.h"

ArduinoLEDMatrix matrix;

char ssid[] = "iPhone (35)";        // your network SSID (name)
char password[] = "aisling123"; 

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
String prevCommand = " ";
String oldCommand = " ";

unsigned const short initLPower = 100;
unsigned const short initRPower = 100;
unsigned short maxPower = 100;
unsigned short minPower = 0;
unsigned short minPowerPID = 50;
unsigned short maxPowerPID = 160;

int currentSliderValue = 0;


unsigned short driveMode = 0; //0 is auto, 1 is controlled by GUI 2 is PID and 3 is just a testing mode because im pissed the fuck off. Not a bool because we want more options.

unsigned long prevMillis = 0;
unsigned long prevMillis_speed = 0;

float distance = 10;
unsigned long lastTime = 0, lastTime1;
const unsigned long intervalTime = 500;

//Wheel specifications
const float wheelDiam = 0.06; // Diameter of wheel (not measured very accurately)
const int PPRev = 8; //Amount of interrupts per revolution of wheel
volatile int leftCount = 0; // Volatile is used whenever something can be changed by something that is not within the code itself (such as an interrupt from the motor) 
volatile int rightCount = 0; // interrupt stops current from flowing ^^
float totalDist = 0.0;  // Total distance travel

double PIDTolerence = (wheelDiam * 100 * PI) / (8 * 2);

unsigned int LPower = initLPower;
unsigned int RPower = initRPower;
int prevLMPower = initLPower; 
int prevRMPower = initRPower;

// unsigned long elapsedTime;
// double output;
// //float setpoint = 15;
// double error;
// double errSum, lastError, rateError;
// double kp = .2; //edit
// double ki = 0.01, kd = 0.05; //must be tuned for these constants


float lastDistLeft = 0;  
float lastDistRight = 0;
float lastTimeLeft = 0;
float lastTimeRight = 0;
float LspeedRPM = 0;
float RspeedRPM = 0;
int wait = 1000;
float totDistRight = 0;
float totDistLeft = 0;

// double KpM = 86, KpR = 1, KpD = 86; 
// double KiM = 59, KiR = 1, KiD = 59;
// double KdM = 2, KdR = 0.2, KdD = 2;
double KpM = 1.3, KpD = 1.1;
double KiM = 0.003, KiD = 0.003;
double KdM = 0.1, KdD = 0.1;

//double SetpointLeft, InputLeft, OutputLeft;
double SetpointMotor, InputMotor, OutputMotor;
double SetpointDist, InputDist, OutputDist;

unsigned long prevTimeL = 0, prevTimeR = 0;

int targetSpeed = 0;
unsigned long elapsedTime;
double output;

//PID LeftPID(&InputLeft, &OutputLeft, &SetpointLeft, KpL, KiL, KdL, DIRECT);
PID MotorPID(&InputMotor, &OutputMotor, &SetpointMotor, KpM, KiM, KdM, DIRECT);
PID DistancePID(&InputDist, &OutputDist, &SetpointDist, KpD, KiD, KdD, DIRECT);

WiFiServer server(3000);

void setup() {
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
  delay(1000);
  Serial.println("Server started");
  Serial.println(WiFi.localIP());

  // LeftPID.SetMode(AUTOMATIC);
  // LeftPID.SetOutputLimits(50, 255);

  MotorPID.SetMode(AUTOMATIC);
  MotorPID.SetOutputLimits(50, 255);

  DistancePID.SetMode(AUTOMATIC);
  DistancePID.SetOutputLimits(50, 255); // Min and max PWM values 

  // Set default speed setpoints
  //SetpointLeft = 30; // Default speed in RPM
  SetpointMotor = 30; // Default speed in RPM
  SetpointDist = 15;

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

void turnLeft(int turnSpeed){
    analogWrite(Pin5, turnSpeed + 20); //Right motor
    RPower = turnSpeed;
    analogWrite(Pin6, minPower); //left motor
    LPower = minPower;
}

void turnRight(int turnSpeed){
    analogWrite(Pin5, minPower); //Right motor
    RPower = minPower;
    analogWrite(Pin6, turnSpeed + 20); //left motor
    LPower = turnSpeed;
}

void  countLeft() { 
  leftCount++; 
}
void countRight() {   //Function to increment count of motor turns
  rightCount++; 
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
  
  //  unsigned long currentTime = millis();
  // if (currentTime - lastTimeLeft >= wait) {
    
     LspeedRPM = (float(leftCount) / float(PPRev)) * (60000.0/1000);
    //Serial.println("amount of turn" + String(leftCount));
      //leftCount = 0;
  //    lastTimeLeft = currentTime;
  //  }
  float Lspeed_m_s = (LspeedRPM * PI * wheelDiam) / (60);

  float Lspeed_cm_s = Lspeed_m_s * 100;

  return Lspeed_cm_s;
}

float getSpeedRight() {
  //  unsigned long currentTime = millis();
  //  if (currentTime - lastTimeRight >= wait) {
     RspeedRPM = (float(rightCount) / float(PPRev)) * (60000.0/1000);  
    // Serial.println("amount of turn" + String(rightCount));
    //rightCount = 0;                                                                     
  //    lastTimeRight = currentTime;
     

  //  }
  float Rspeed_m_s = (RspeedRPM * PI * wheelDiam) / (60000.0/wait);

  float Rspeed_cm_s = Rspeed_m_s * 100;
  
  return Rspeed_cm_s;
}

int rpmToPWM(float rpm) {
  if (rpm <= 0) return 0;
  int minStartPWM = 50;
  int pwm = minStartPWM + (255 - minStartPWM) * (rpm / 60.0);
  return constrain(pwm, minStartPWM, 255);
}
double cmsToRPM(double cms) {
  if(cms <= 0) return 0;
  double RPM = (cms / (wheelDiam * 100 * PI)) * 60;
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
  int distance = SSensor(10);

  WiFiClient client = server.available();
  if (client && client.available()) {
    client.println("Sensor:" + String(distance));
    Serial.println("dist from object" + String(distance));
    if (distance < 10) {
      client.println("stop");
    } else {
      client.println("nothing");
    }
 }

  String command = readCommand();

  // if (command.startsWith("SPEED:")) {
  //     // Find the position of "SPEED:" in the command
  //     int speedPos = command.indexOf("SPEED:");
  //     // Extract just the speed value part
  //     String speedVal = command.substring(speedPos + 6);
  //     maxPower = speedVal.toInt();
  //     Serial.println("value read");

  //     currentSliderValue = maxPower;
  //    }

  if (lastCommand == "Stop"){
    Motormove(0, 0, 1);
    return;
  } else {
    if (distance < 10) {
      //Serial.println("Sensor decteded obstacle");
      Motormove(1, 1, 1);
    } else {
      //Serial.println(" Entered post distance-check loop");

      if (Sensorleft() && Sensorright()) { //both on black
      LMPower( maxPower); //set and print
      RMPower( maxPower); //set and print
      LPower = maxPower;
      RPower = maxPower;
      //Serial.println("No sensor on line.");
      Motormove(1, 1, 0);
    } else if (!Sensorleft() && !Sensorright()){ //both on white
      //slow down and keep going
      LPower = minPower;
      RPower = minPower;
      //Serial.println("Both sensor on line.");
      Motormove(1, 1, 0);
    } else if ( !Sensorleft() && Sensorright()){ //sensor left white sensor right is black
      //Serial.println("Left sensor on line.");
      turnLeft(maxPower);
      // Serial.println(LPower);
      // Serial.println(RPower);
      //delay(50);
      //Serial.println("delay done"); 
    } else if ( Sensorleft() && !Sensorright()){
      //Serial.println("Right sensor on line.");
      turnRight(maxPower);
      //Serial.println(LPower);
      //Serial.println(RPower);
      //delay(50);
      //Serial.println("delay done");
      }
    }
  }
}


void remoteControl(float readSpeed) {
  //add stopping with ultrasonic sensor
  WiFiClient client = server.available();
  //Serial.println("in loop");

   int distance = SSensor(10);
  if (client && client.available()) {
    client.println("Sensor:" + String(distance));
   // Serial.println("dist from object" + String(distance));
    if (distance < 10) {
      client.println("stop");
    } else {
      client.println("nothing");
    }
  }

     String speed = readCommand();
     //Serial.println(speed);
     //if (speed.startsWith("SPEED:")) {
      if (speed.startsWith("S:")) {
      // Find the position of "SPEED:" in the command
      Serial.println("speed being read");
      int speedPos = speed.indexOf("SPEED:");
      // Extract just the speed value part
      String speedVal = speed.substring(speedPos + 6);
      int speedValue = speedVal.toInt();
      Serial.println("value read");

      currentSliderValue = speedValue;
     }
    //  else if (speed == "Stop" || lastCommand == "Stop") {
    //   Motormove(0, 0, 1);
    //  }
     
    double PIDTolerence = (wheelDiam * 100 * PI) / (8 * 2);
    unsigned long currentTime = millis();
    if (currentTime - lastTimeRight >= wait) {
       //InputMotor = (double)(getSpeedLeft() + getSpeedRight()) / 2;
      InputMotor = (double)(readSpeed);
      SetpointMotor = (double)currentSliderValue;

      if(abs(InputMotor - SetpointMotor) >= PIDTolerence){
        MotorPID.SetTunings(KpM, KiM, KdM);
        MotorPID.Compute();

        Serial.print(SetpointMotor);
        Serial.print(",");
        Serial.println(InputMotor);

        selfDrivePID(short(OutputMotor));
      }
      lastTimeRight = currentTime;
   }

    //double MotorPower = cmsToRPM((double)OutputMotor);
    //Serial.println("Forward at " + String(currentSliderValue) + " RPM (PWM: " + String(pwmValue) + ")");
       // }
}

void selfDrivePID(short power) {
  int distance = SSensor(10);
  if (power > maxPowerPID) {
    power = maxPowerPID;
  }

  WiFiClient client = server.available();
  if (client && client.available()) {
    client.println("Sensor:" + String(distance));
    //Serial.println("dist from object" + String(distance));
  }

  if (Sensorleft() && Sensorright()) { 
    LMPower(power);
    RMPower(power); 
    LPower = power;
    RPower = power;
    Motormove(1, 1, 0);
  } else if (!Sensorleft() && !Sensorright()){ 
    LPower = minPowerPID;
    RPower = minPowerPID;
    Motormove(1, 1, 0);
  } else if ( !Sensorleft() && Sensorright()){ 
    turnLeft(power);
  } else if ( Sensorleft() && !Sensorright()){
    turnRight(power);

   }
}

void followObjectMode() {
  // Get current distance from sensor
  float currentDistance = SSensor(10);
  String command = readCommand();
  // Serial.println("Loope entered");

  // Serial.print("Current distance: ");
  // Serial.print(currentDistance);
  // Serial.println(", Target: 15");

  // if(client) {
  //   client.println("in loop follow object");
  // 
 
  DistancePID.SetTunings(KpD, KiD, KdD);
  InputDist = currentDistance;
  DistancePID.Compute();

  // Serial.print(SetpointDist);
  // Serial.print(",");
  // Serial.println(InputDist);

 
  if(currentDistance >= SetpointDist) {
    selfDrivePID(maxPower + OutputDist);
  }
  else if (currentDistance < 15 && currentDistance >= 10) {
    selfDrivePID(maxPower - OutputDist);
  }
  else if (currentDistance < 10) {
    Motormove(0, 0, 1);
  }


} 


float distTravelled() {
  float leftRotations = (float)leftCount / (float)PPRev;
  float rightRotations = (float)rightCount / (float)PPRev;

  float distLeft = leftRotations * (PI * wheelDiam);
  float distRight = rightRotations * (PI * wheelDiam);
  float avgDist = (distLeft + distRight) / 2;

  totalDist += avgDist;
  // leftCount = 0;
  // rightCount = 0;

  // Serial.println("Total Dist Calculated: " + String(totalDist));

  return totalDist;
}


unsigned long lastTime_speed = 0;

float getSpeed() {
  //float speed = 0;
  static unsigned long lastTime_speed = 0;
  unsigned long currentTime_speed = millis();
  //if (currentTime_speed - lastTime_speed >= 1000) {
     float timeDifference = (currentTime_speed - lastTime_speed) / 1000.0; // to seconds
    lastTime_speed = currentTime_speed;

  
 float dist = distTravelled();

  
  float speed =  dist / timeDifference; // Speed in cm/s
  dist = 0;  
  return speed;
}

const uint32_t happy[] = {
    0x1f820,
		0x40000900,
		0x90090000,
		66
};

// float getSpeed1(int samplePeriod) {
//   //float speed = 0;
// float circumference = PI * wheelDiam;
//     int lastEncoderCount;
//     int pulsesDuringPeriod;
//     float speed;
    
//     // int leftCount = countLeft();
//     // int rightCount = countRight();
    
   
//     pulsesDuringPeriod = (leftCount + rightCount) - lastEncoderCount;
    
//     // // Handle overflow if needed
//     // if (pulsesDuringPeriod < 0) {
//     //     pulsesDuringPeriod += ENCODER_MAX_VALUE; // Depends on encoder bit depth
//     // }
    
//     // Calculate speed in cm/s
//     speed = (pulsesDuringPeriod * circumference) / (8 * samplePeriod);
    
//     // Update last count for next calculation
//     lastEncoderCount = leftCount + rightCount;
//     // leftCount = 0;
//     // rightCount = 0;
    
//     return speed;

// }



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
  if(lastCommand == "Stop") {
    Motormove(0, 0, 1);
  }
 //Serial.println("loop command " + command);
if (command != prevCommand){
  if (command == "FollowObject") {
    driveMode = 2;
    Serial.println("drive mode 2");
    client.println("drive mode 2");
  }
  else if (command == "Auto") {
    driveMode = 0;
    Serial.println("drive mode 0");
    client.println("drive mode 0");
  }
  else if (command == "Remote") {
    driveMode = 1;
    Serial.println("drive mode 1");
  }
  prevCommand = command;
}
  // Send distance data to client periodically
  unsigned long currentMillis = millis();
  if (currentMillis - prevMillis >= 1000) {
    prevMillis = currentMillis;
    
    // Send distance data to any connected client
    if (client && client.connected()) {
      float distTrav = distTravelled();
      client.print("D:");
      client.println(String(distTrav));
      //Serial.println("Total Dist Calculated: " + String(distTrav));

      float speed = (getSpeedLeft() + getSpeedRight()) / 2;
  //    float speed = getSpeed();
      // if(lastCommand == "Stop"){
      //   speed = 0;
      // }
      // else{
        client.print("Speed:");
        client.println(String(speed));
        //Serial.println("Speed: " + String(speed));
   //   }
      leftCount = 0;
      rightCount = 0;
      
     // Serial.println("Speed: " + String(speed));


  // Execute the appropriate driving mode
  switch (driveMode) {
    case 0:
      selfDrive();
      break;
    case 1:
      remoteControl(speed);
      break;
    case 2:
      followObjectMode();
      break;
    case 3:
      // Testing mode - do nothing
      break;
          }
 }
  }
}
