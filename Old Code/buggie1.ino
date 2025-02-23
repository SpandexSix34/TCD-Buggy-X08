#include "Arduino_LED_Matrix.h"
#include "WiFiS3.h"

#define SECRET_SSID "iPhone (35)"
#define SECRET_PASS "aisling123"


ArduinoLEDMatrix matrix;

char ssid[] = "iPhone (35)";        // your network SSID (name)
char pass[] = "aisling123"; 

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

// bool LeftEye = 0;
// bool RightEye = 0;
// bool prevLeft = 1;
// bool prevRight = 1;

unsigned const short initLPower = 130;
unsigned const short initRPower = 130;
unsigned const short maxPower = 130;
unsigned const short minPower = 65;

unsigned int LPower = initLPower;
unsigned int RPower = initRPower;
int prevLMPower = LPower; 
int prevRMPower = RPower;

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

  WiFi.begin(ssid, pass);
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address:");
  Serial.println(ip);

  if ( WiFi.status() == WL_NO_MODULE) {
    Serial.println("Unable to Connect");

    while (true); //creates an infite loop that stops the code from executing without wifi
  }
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
  }
  else{
    if (ControlLeft){ 
      Motorleft(1,0); //foward
    }
    else{
      Motorleft(0,1); //back
    }
    if (ControlRight){
      Motorright(1,0); //foward
    }
    else{
    Motorright(0,1); //back
    }
  }
}  

bool Sensorleft() {
  if (digitalRead(12) == HIGH) { //black
    return true;
  } else {
    return false;
  }
  //delay(200);
}

bool Sensorright() {
 if (digitalRead(13) == HIGH) { //black
    return true;
  } else {
    return false;
  }
 // delay(200);
}

//Possibly unnessecary functions??
void LMPower(int power){
  if (power != prevLMPower) {
    analogWrite(Pin6, power);
    Serial.print("Left: ");
    Serial.println(power);
    prevLMPower = power; 
  }
}
void RMPower(int power){
 if (power != prevRMPower) {
    analogWrite(Pin5, power);
    Serial.print("Right: ");
    Serial.println(power);
    prevRMPower = power; 
  }
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


const uint32_t happy[] = {
    0x1f820,
		0x40000900,
		0x90090000,
		66
};

void loop() {

  matrix.loadFrame(happy);

  bool go = true;
  
if (go){ 
    digitalWrite(TrigPin, LOW);
    digitalWrite(TrigPin, HIGH);
    delay(10);  //delay the high sound for 10ms
    digitalWrite(TrigPin, LOW);
    //delay?

    unsigned long duration = pulseIn(EchoPin, HIGH); //high starts the timing (microsecond)
    int distance = duration * 0.034 / 2; //converts the time to a speed

    if (distance < 10){
      Serial.println("Sensor decteded obstacle");
      Motormove(0, 0, 1);
    }
    else {
      if ( Sensorleft() && Sensorright()) { //both on black
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
          delay(20);
          Serial.println("delay done"); 
      } else {
          Serial.println("Right sensor on line.");
          turnRight();
          Serial.println(LPower);
          Serial.println(RPower);
          delay(20);
          Serial.println("delay done");
    }
  }
}

// void loop() {
//     digitalWrite(TrigPin, LOW);
//     digitalWrite(TrigPin, HIGH);
//     delay(10);
//     digitalWrite(TrigPin, LOW);

//     long duration = pulseIn(echoPin, HIGH);
//     int distance = duration * 0.034 / 2;

//     if (distance < 10){
//       Serial.println("Sensor decteded obstacle");
//       Motormove(0, 0, 1);
//     }
//     else {
//       Motormove(1, 1, 0);
//     }
//     delay(500);
// }
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
}
