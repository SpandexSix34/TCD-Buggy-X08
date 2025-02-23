#include "Arduino_LED_Matrix.h"
#include "WiFiS3.h"

#define SECRET_SSID "Pixel 9"
#define SECRET_PASS "nfhu5637"

ArduinoLEDMatrix matrix;

char ssid[] = "Pixel 9";        // your network SSID (name)
char pass[] = "nfhu5637"; 

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

bool LeftEye = 0;
bool RightEye = 0;
bool prevLeft = 1;
bool prevRight = 1;

unsigned const short initLPower = 200;
unsigned const short initRPower = 200;
unsigned const short maxPower =200;
unsigned const short minPower = 100;

unsigned int LPower = initLPower;
unsigned int RPower = initRPower;

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

  matrix.begin();

  LMPower(LPower);
  RMPower(RPower);

  while( !Serial) {
    ;
  }

  WiFi.begin(ssid, pass);
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address:");
  Serial.println(ip);

  if ( WiFi.status() == WL_NO_MODULE) {
    Serial.println("Unable to Connect");

    while (true);
  }
}

void Motorleft(bool c,bool d){
  if (c){
      digitalWrite(Pin8, HIGH);
  }else{
      digitalWrite(Pin8, LOW);
  }
  if (d){
      digitalWrite(Pin9, HIGH);
  }else{
      digitalWrite(Pin9, LOW);
  }
}

void Motorright(bool c,bool d){
  if (c){
      digitalWrite(Pin10, HIGH);
  }else{
      digitalWrite(Pin10, LOW);
  }
  if (d){
      digitalWrite(Pin11, HIGH);
  }else{
      digitalWrite(Pin11, LOW);
  }
}

void Motormove(bool c, bool d,bool e){
  if (e){
    Motorleft(1,1);
    Motorright(1,1);
  }else{
    if (c){
    
    Motorleft(1,0);
  }else{
    Motorleft(0,1);
  }
  if (d){
    Motorright(1,0);
  }else{
    Motorright(0,1);
  }
  }
  
}  

bool Sensorleft() {
  if (digitalRead(12) == HIGH) {
    return true;
  } else {
    return false;
  }
}

bool Sensorright() {
 if (digitalRead(13) == HIGH) {
    return true;
  } else {
    return false;
  }
}

void LMPower(int power){
  analogWrite(Pin6, power);
  Serial.print("Left: ");
  Serial.println(power);
}

void RMPower(int power){
  analogWrite(Pin5, power);
  Serial.print("Right: ");
  Serial.println(power);
}

void LC(unsigned int c){
  for (int i=LPower;i>=LPower+c;i--){
    if (LPower >= minPower) {
    LPower -= 1;
    LMPower(i);
    RMPower(LPower);
    delay(100/c);
    }
  }
}

void RC(int c){
  for (int i=RPower;i>=RPower+c;i--){
      if (RPower >= minPower) {
      RPower -= 1;
      LMPower(RPower);
      RMPower(i);
      delay(100/c);
      }
  }
}

const uint32_t happy[] = {
    0x1f820,
		0x40000900,
		0x90090000,
		66
};

void loop() {

  matrix.loadFrame(happy);

  bool go = false;
  
if (go){ 
  if ( Sensorleft() && Sensorright()) {
    LMPower( maxPower);
    RMPower( maxPower);
    Motormove(1, 1, 0);
  } else if (!Sensorleft() && !Sensorright()){
    Motormove(0, 0, 1);
  } else if ( !Sensorleft() && Sensorright()){
    Serial.println(LPower);
    Serial.println(RPower);
    RC(-100);
    delay(100);
    Motormove(1, 1, 0);

  } else {
    Serial.println(LPower);
    Serial.println(RPower);
    LC(-100);
    delay(100);
    Motormove(1, 1, 0);
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
}