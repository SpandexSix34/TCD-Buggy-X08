#ifndef ROBOT_FUNCTIONS_H
#define ROBOT_FUNCTIONS_H

#include <Arduino.h>
#include <PID_v1.h>
#include "Arduino_LED_Matrix.h"
#include "WiFiS3.h"

extern ArduinoLEDMatrix matrix;
extern WiFiServer server;

// Pin definitions
#define Pin8 8
#define Pin9 9
#define Pin10 10
#define Pin11 11
#define Pin12 12
#define Pin13 13
#define RMove 3
#define Pin5 5
#define Pin6 6
#define TrigPin 7 // receives echo
#define EchoPin 4 // sends out an echo 
#define LEncoder 2
#define REncoder 3

// Constants
extern const float wheelDiam;
extern const int PPRev;
extern const unsigned short initLPower;
extern const unsigned short initRPower;
extern unsigned short maxPower;
extern unsigned short minPower;
extern const float TARGET_DISTANCE;
extern const float DISTANCE_TOLERANCE;
extern const unsigned short interruptDist;

// Variables
extern String lastCommand;
extern unsigned short driveMode;
extern bool pidDrive;
extern int speedValue;
extern volatile int leftCount;
extern volatile int rightCount;
extern float totalDist;
extern unsigned int LPower;
extern unsigned int RPower;
extern unsigned long prevMillisLeft;
extern unsigned long prevMillisRight;
extern float currSpeedRight;
extern float currSpeedLeft;

// PID variables
extern double KpL, KpR;
extern double KiL, KiR;
extern double KdL, KdR;
extern double SetpointLeft, InputLeft, OutputLeft;
extern double SetpointRight, InputRight, OutputRight;
extern PID LeftPID;
extern PID RightPID;

// Function declarations
void digitalToggle(byte pin);
float SSensor(int interval);
void Motorleft(bool toGoFoward, bool toGoBack);
void Motorright(bool toGoFoward, bool toGoBack);
void Motormove(bool ControlLeft, bool ControlRight, bool toStop);
bool Sensorleft();
bool Sensorright();
void LMPower(int power);
void RMPower(int power);
void turnLeft();
void turnRight();
void speedLeft();
void speedRight();
String readCommand();
float rightDistTravelled();
float leftDistTravelled();
float getSpeedLeft();
float getSpeedRight();
void remoteControl();
float distTravelled();
int rpmToPWM(float rpm);
float getSpeed();
void pidSpeedControl();
void selfDrive();
void selfDriveNoPID();
void followObjectModeNoPID();
void combinedModeNoPID(float distanceToObject);
void followObjectMode();

#endif // ROBOT_FUNCTIONS_H
