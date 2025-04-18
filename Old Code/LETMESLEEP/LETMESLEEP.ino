#include "Arduino_LED_Matrix.h"
#include "WiFiS3.h"

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
#define Pin12 1
#define Pin13 13
#define RMove 3
#define Pin5 5
#define Pin6 6
#define TrigPin 7 // recives echo
#define EchoPin 4 // sends out an echo 
#define LEncoder 2
#define REncoder 3 // Setting all the pins, bytes because small

String lastCommand = "stop";

// bool LeftEye = 0;
// bool RightEye = 0;
// bool prevLeft = 1;
// bool prevRight = 1;

unsigned const short initLPower = 160;
unsigned const short initRPower = 160;
unsigned const short maxPower = 160;
unsigned const short minPower = 0;

unsigned short driveMode = 1; //0 is auto, 1 is controlled by GUI 2 is PID and 3 is just a testing mode because im pissed the fuck off. Not a bool because we want more options.

unsigned long prevMillis = 0;

float distance = 10;
unsigned long lastTime = 0;
const unsigned long intervalTime = 500;

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

String readCommand() {
  // Don't create a new client instance here
  WiFiClient client = server.available();
  
  if (client && client.connected() && client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();
    return command;
  }
  return ""; // Return empty string if no command available
}

void selfDrive() {
  int distance = SSensor(10);

  if (distance < 10) {
    Serial.println("Sensor decteded obstacle");
    Motormove(1, 1, 1);
  } else {
    //Serial.println(" Entered post distance-check loop");

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
  } else if ( !Sensorleft() && Sensorright()){ //sensor left white sensor right is black
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

void remoteControl() {
  WiFiClient client = server.available();
  
  if (client && client.connected() && client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();
    
    if (command.length() == 0) {
      return;  // No command to process
    }
    
    Serial.println("Received: " + command);  // Print the message to Serial Monitor
  
    // Process speed commands with more flexible detection
    if (command.indexOf("SPEED:") != -1) {
      // Find the position of "SPEED:" in the command
      int speedPos = command.indexOf("SPEED:");
      // Extract just the speed value part
      String speedVal = command.substring(speedPos + 6);
      int speedValue = speedVal.toInt();
      Serial.println("New speed: " + speedVal);
      RMPower(speedValue);
      LMPower(speedValue);
    }
    // Process movement commands
    else if (command == "Forward") {
      Motormove(1, 1, 0);
    }
    else if (command == "Left") {
      Motormove(0, 1, 0);
    }
    else if (command == "Right") {
      Motormove(1, 0, 0);
    }
    else if (command == "Back") {
      Motormove(0, 0, 0);
    }
    else if (command == "Stop") {
      Motormove(1, 1, 1);
    }
    else if (command == "Remote") {
      driveMode = 1;
      Serial.println("Remote Mode On");
    } 
    else if (command == "Self") {
      driveMode = 0;
      Serial.println("Self-Driving Mode On");
    } 
    else if (command == "PID") {
      driveMode = 2;
      Serial.println("PID Driving mode");
    }
  }
}


void PIDdrive() {
  float distanceToObject = SSensor(10);
  Serial.println("Distance to object is: " + String(distanceToObject));
  if ( distanceToObject <= 14) {
    RMPower(130);
    LMPower(130);
    Motormove(0, 0, 0);
    Serial.println("We are too close, moving backwards.");
  } else if (distanceToObject >= 16) {
    RMPower(130);
    LMPower(130);
    Motormove(1, 1, 0);
    Serial.println("We are too far, moving forwards.");
  } else {
    Motormove(0, 0, 1);
    Serial.println("We are at around 15cm!");
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

const uint32_t happy[] = {
    0x1f820,
		0x40000900,
		0x90090000,
		66
};

void loop() {
  matrix.loadFrame(happy);
  
  // Process distance traveled
  float distTrav = distTravelled();
  
  // Send distance data to client periodically
  unsigned long currentMillis = millis();
  if (currentMillis - prevMillis >= 1000) {
    prevMillis = currentMillis;
    
    // Send distance data to any connected client
    WiFiClient client = server.available();
    if (client && client.connected()) {
      client.print("D:");
      client.println(distTrav);
    }
  }
  
  // Check distance sensor
  float currentDistance = SSensor(10);
  if (currentDistance < 10) {
    WiFiClient client = server.available();
    if (client && client.connected()) {
      client.println("stop");
    }
  } else {
    WiFiClient client = server.available();
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
      PIDdrive();
      break;
    case 3:
      // Testing mode - do nothing
      break;
  }
}



























// Hard to explain - The Strokes

// Was an honest man
// Asked me for the phone, tried to take control
// Oh, I don't see it that way
// I don't see it that way
// We shared some ideas
// All obsessed with fame, says we're all the same
// Oh, I don't see it that way
// I don't see it that way
// Raised in Carolina
// I'm not like that
// Trying to remind her
// When we go back
// I missed the last bus, I'll take the next train
// I'll try, but you see, it's hard to explain
// I say the right thing but act the wrong way
// I like it right here, but I cannot stay
// I'm watching TV, forget what I'm told
// Well, I am too young and they are too old
// The joke is on you, this place is a zoo
// You're right, it's true
// He says he can't decide
// I shake my head to say, "everything's just great"
// Oh, I just can't remember
// I just can't remember
// Raised in Carolina
// She says, "I'm not like that"
// Trying to remind her
// When we go back
// I say the right thing but act the wrong way
// I like it right here, but I cannot stay
// I'm watching TV, forget what I'm told
// Well, I am too young and they are too old
// Oh man, can't you see, I'm nervous, so please
// Pretend to be nice so I can be mean
// I missed the last bus, we'll take the next train
// I'll try, but you see, it's hard to explain

