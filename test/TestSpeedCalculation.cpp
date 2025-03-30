#include "robot_functions.h"

void Setup() {
  Serial.begin(115200);

  pinMode(Pin8, OUTPUT);
  pinMode(Pin9, OUTPUT);
  pinMode(Pin10, OUTPUT);
  pinMode(Pin11, OUTPUT);
  pinMode(Pin12, INPUT);
  pinMode(Pin13, INPUT);
  pinMode(Pin5, OUTPUT);
  pinMode(Pin6, OUTPUT);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  pinMode(LEncoder, INPUT_PULLUP);
  pinMode(REncoder, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEncoder), speedLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(REncoder), speedRight, RISING);
}

void loop() {
  
}