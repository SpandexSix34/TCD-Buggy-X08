#include <Arduino.h>
#include <HCSR04.h>
#include <PID_v1.h>
#include "Arduino_LED_Matrix.h"
#include "WiFiS3.h"

// Wheel Encoders
#define LEncoder 2
#define REncoder 3

// US Rangefinder Pins
#define TrigPin 7
#define EchoPin 4

// PWM Pins
#define RPower 5
#define LPower 6

// Motor Control Pins
#define LMotor1 8
#define LMotor2 9
#define RMotor1 10
#define RMotor2 11

// IR Sensor Pins
#define LIR 12
#define RIR 13

// Direction Constants for Motors
#define FORWARD 1
#define BACKWARDS 0
#define STOPPED 2

// Drive mode constants
#define MODE_AUTO 0
#define MODE_REMOTE 1
#define MODE_FOLLOW_OBJECT 2
#define MODE_TEST 3

// volatile constants for interrupts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Wifi and Matrix
WiFiServer server(3000);
ArduinoLEDMatrix matrix;

char ssid[] = "Pixel 9";
char password[] = "nfhu5637"; 

// Global variables for commands and control
String lastCommand = "";
String prevCommand = "";
unsigned short driveMode = MODE_AUTO;
int currentSliderValue = 0;

// Motor power settings
unsigned const short initLPower = 100;
unsigned const short initRPower = 100;
unsigned short maxPower = 100;
unsigned short minPower = 0;
unsigned short minPowerPID = 50;
unsigned short maxPowerPID = 160;

// Time tracking
unsigned long prevMillis = 0;
unsigned long prevMillis_speed = 0;
unsigned long lastTimeLeft = 0;
unsigned long lastTimeRight = 0;
int wait = 1000;

// Distance and speed tracking
float distance = 10;
float totalDist = 0.0;
float totDistRight = 0;
float totDistLeft = 0;

// Wheel specifications
const float wheelDiam = 0.06; // Diameter of wheel in meters
const int PPRev = 8; // Amount of interrupts per revolution of wheel

// PID variables
double PIDTolerence = (wheelDiam * 100 * PI) / (8 * 2);

// PID tuning parameters
double KpM = 1.3, KpD = 1.1;
double KiM = 0.003, KiD = 0.003;
double KdM = 0.1, KdD = 0.1;

// PID setpoints and variables
double SetpointMotor, InputMotor, OutputMotor;
double SetpointDist, InputDist, OutputDist;

// PID controllers
PID MotorPID(&InputMotor, &OutputMotor, &SetpointMotor, KpM, KiM, KdM, DIRECT);
PID DistancePID(&InputDist, &OutputDist, &SetpointDist, KpD, KiD, KdD, DIRECT);

const uint32_t happy[] = {
    0x1f820,
    0x40000900,
    0x90090000,
    66
};

// Interrupt Functions
void leftEncoderInt() {
  leftEncoderCount++;
}

void rightEncoderInt() {
  rightEncoderCount++;
}

// Motor Class
class Motor {
  private: 
    uint8_t motorPin1;
    uint8_t motorPin2;
    uint8_t powerPin;
    uint8_t encoderPin;
    uint8_t currentDirection;
    uint8_t currentPower;
    float currentSpeed;
    volatile long* encoderCount;

  public:
    Motor(uint8_t motor1, uint8_t motor2, uint8_t pinPower, uint8_t encoder, volatile long* encCount) {
      motorPin1 = motor1;
      motorPin2 = motor2;
      powerPin = pinPower;
      encoderPin = encoder;
      encoderCount = encCount;
      currentDirection = STOPPED;
      currentPower = 0;

      pinMode(motorPin1, OUTPUT);
      pinMode(motorPin2, OUTPUT);
      pinMode(powerPin, OUTPUT);
      pinMode(encoderPin, INPUT_PULLUP);

      stop();
    }

    void scalePower(int8_t power) {
      if (power == 0) {
      }
      else if (power > 0 && power <= 255 && (currentPower + power) <= 255) {
        currentPower += power;
        setPower(currentPower);
      }
      else if (power < 0 && power >= -255 && (currentPower + power) >= 0) {
        currentPower += power;
        setPower(currentPower);
      }
    }

    void setPower(uint8_t power) {
      currentPower = power;
      analogWrite(powerPin, power);
    }

    void setDirection(uint8_t direction) {
      currentDirection = direction;
      switch(direction) {
        case FORWARD:
          digitalWrite(motorPin1, HIGH);
          digitalWrite(motorPin2, LOW);
          break;
        case BACKWARDS:
          digitalWrite(motorPin1, LOW);
          digitalWrite(motorPin2, HIGH);
          break;
        case STOPPED:
          digitalWrite(motorPin1, HIGH);
          digitalWrite(motorPin2, HIGH);
          break;
      }
    }

    void drive(uint8_t direction, uint8_t power) {
      setDirection(direction);
      setPower(power);
    }

    void stop() {
      setDirection(STOPPED);
      setPower(0);
    }

    uint8_t getDirection() {
      return currentDirection;
    }

    uint8_t getPower() {
      return currentPower;
    }

    long getEncoderCount() {
      return *encoderCount;
    }

    void resetEncoderCount() {
      noInterrupts();
      *encoderCount = 0;
      interrupts();
    }

    uint8_t getEncoderPin() {
      return encoderPin;
    }

    float getSpeedRPM() {
      return (float(*encoderCount) / float(PPRev)) * (60000.0/wait);
    }

    float getSpeedCmS() {
      float speedRPM = getSpeedRPM();
      float speed_m_s = (speedRPM * PI * wheelDiam) / 60.0;
      return speed_m_s * 100; // Convert to cm/s
    }

    float getDistanceTraveled() {
      float rotations = (float(*encoderCount) / float(PPRev));
      float dist = rotations * (PI * wheelDiam);
      return dist;
    }

    void debug() {
      Serial.println("Motor: ");
      Serial.println("Pin1="); Serial.print(motorPin1);
      Serial.println(", Pin2="); Serial.print(motorPin2);
      Serial.println(", Power="); Serial.print(powerPin);
      Serial.println(", Direction="); Serial.print(currentDirection);
      Serial.println(", PowerLevel="); Serial.print(currentPower);
      Serial.println(", EncoderCount="); Serial.println(*encoderCount);
    }
};

// Motor Controller Class
class MotorController {
  private:
    Motor& leftMotor;
    Motor& rightMotor;

  public:
    MotorController(Motor& left, Motor& right) : leftMotor(left), rightMotor(right) {}

    void setupInterrupts() {
      attachInterrupt(digitalPinToInterrupt(leftMotor.getEncoderPin()), leftEncoderInt, RISING);
      attachInterrupt(digitalPinToInterrupt(rightMotor.getEncoderPin()), rightEncoderInt, RISING);
    }

    void forward(uint8_t power) {
      leftMotor.drive(FORWARD, power);
      rightMotor.drive(FORWARD, power);
    }

    void backward(uint8_t power) {
      leftMotor.drive(BACKWARDS, power);
      rightMotor.drive(BACKWARDS, power);
    }

    void left(uint8_t power) {
      leftMotor.drive(STOPPED, 0);
      rightMotor.drive(FORWARD, power + 20);
    }

    void right(uint8_t power) {
      leftMotor.drive(FORWARD, power + 20);
      rightMotor.drive(STOPPED, 0);
    }

    void stop() {
      leftMotor.stop();
      rightMotor.stop();
    }

    void resetEncoders() {
      leftMotor.resetEncoderCount();
      rightMotor.resetEncoderCount();
    }

    long getLeftEncoderCount() {
      return leftMotor.getEncoderCount();
    }

    long getRightEncoderCount() {
      return rightMotor.getEncoderCount();
    }

    float getAverageSpeed() {
      return (leftMotor.getSpeedCmS() + rightMotor.getSpeedCmS()) / 2.0;
    }

    float getTotalDistance() {
      float leftDist = leftMotor.getDistanceTraveled();
      float rightDist = rightMotor.getDistanceTraveled();
      return (leftDist + rightDist) / 2.0;
    }

    void setPowers(uint8_t leftPower, uint8_t rightPower) {
      leftMotor.setPower(leftPower);
      rightMotor.setPower(rightPower);
    }
};

// IR Sensor Class
class IRSensor {
  private:
    uint8_t sensorPin;
    String sensorName;
    bool lastReading;
    unsigned long lastReadTime;
  
  public:
    IRSensor(uint8_t pin, String name) {
      sensorPin = pin;
      sensorName = name;
      lastReading = false;
      lastReadTime = 0;

      pinMode(sensorPin, INPUT);
    }

    bool read() {
      lastReading = digitalRead(sensorPin);
      lastReadTime = millis();
      return lastReading;
    }

    bool detecting() {
      return read();
    }

    bool getLastReading() { 
      return lastReading;
    }

    unsigned long getReadingAge() {
      return millis() - lastReadTime;
    }

    uint8_t getPin() {
      return sensorPin;
    }

    void debug() {
      Serial.print(sensorName);
      Serial.print(" (pin ");
      Serial.print(sensorPin);
      Serial.print("): ");
      Serial.println(detecting() ? "DETECTED" : "NOT DETECTED");
    }
};

// Ultrasonic Sensor class
class UltrasonicSensor {
  private:
    HCSR04* sensor;
    
  public:
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin) {
      sensor = new HCSR04(trigPin, echoPin);
    }
    
    ~UltrasonicSensor() {
      delete sensor;
    }
    
    float getDistance() {
      return sensor->dist();
    }
};

// WiFi Communication Handler Class
class WiFiCommunication {
  private:
    WiFiServer& server;
    WiFiClient client;
    
  public:
    WiFiCommunication(WiFiServer& serverRef) : server(serverRef) {}
    
    void setup(const char* ssid, const char* password) {
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
      }
      Serial.println("Connected to WiFi");
      
      server.begin();
      Serial.println("Server started");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    }
    
    String readCommand() {
      client = server.available();
      
      if (client && client.connected() && client.available()) {
        String command = client.readStringUntil('\n');
        command.trim();
        Serial.println("Command received: " + command);
        return command;
      }
      return "";
    }
    
    void sendData(String data) {
      if (client && client.connected()) {
        client.println(data);
      }
    }
    
    void sendSensorData(float distance) {
      if (client && client.connected()) {
        client.println("Sensor:" + String(distance));
      }
    }
    
    void sendDistanceData(float distance) {
      if (client && client.connected()) {
        client.print("D:");
        client.println(String(distance));
      }
    }
    
    void sendSpeedData(float speed) {
      if (client && client.connected()) {
        client.print("Speed:");
        client.println(String(speed));
      }
    }
    
    void sendStopCommand() {
      if (client && client.connected()) {
        client.println("stop");
      }
    }
    
    void sendNothing() {
      if (client && client.connected()) {
        client.println("nothing");
      }
    }
    
    void sendDriveMode(int mode) {
      if (client && client.connected()) {
        client.println("drive mode " + String(mode));
      }
    }
};

// Robot Controller Class
class RobotController {
  private:
    Motor& leftMotor;
    Motor& rightMotor;
    MotorController& motors;
    IRSensor& leftIR;
    IRSensor& rightIR;
    UltrasonicSensor& distanceSensor;
    WiFiCommunication& wifi;
    ArduinoLEDMatrix& ledMatrix;
    
    // PID controllers reference
    PID& motorPID;
    PID& distancePID;
    
  public:
    RobotController(Motor& left, Motor& right, MotorController& motorCtrl, 
                   IRSensor& lir, IRSensor& rir, UltrasonicSensor& ds, 
                   WiFiCommunication& wf, ArduinoLEDMatrix& matrix,
                   PID& mPID, PID& dPID) 
      : leftMotor(left), rightMotor(right), motors(motorCtrl), 
        leftIR(lir), rightIR(rir), distanceSensor(ds), 
        wifi(wf), ledMatrix(matrix),
        motorPID(mPID), distancePID(dPID) {}
    
    void setupPID() {
      motorPID.SetMode(AUTOMATIC);
      motorPID.SetOutputLimits(minPowerPID, maxPowerPID);
      
      distancePID.SetMode(AUTOMATIC);
      distancePID.SetOutputLimits(minPowerPID, maxPowerPID);
      
      // Set default setpoints
      SetpointDist = 15;  
    }
    
    void setupLEDMatrix() {
      ledMatrix.begin();
      ledMatrix.loadFrame(happy);
    }
    
    void selfDrive() {
      float distance = distanceSensor.getDistance();
      wifi.sendSensorData(distance);
      
      if (lastCommand == "Stop") {
        motors.stop();
        return;
      }
      
      if (distance < 10) {
        motors.stop();
      } else {
        if (leftIR.detecting() && rightIR.detecting()) { 
          leftMotor.setPower(maxPower);
          rightMotor.setPower(maxPower);
          motors.forward(maxPower);
        } else if (!leftIR.detecting() && !rightIR.detecting()) { 
          leftMotor.setPower(minPower);
          rightMotor.setPower(minPower);
          motors.forward(minPower);
        } else if (!leftIR.detecting() && rightIR.detecting()) { 
          motors.left(maxPower);
        } else if (leftIR.detecting() && !rightIR.detecting()) { 
          motors.right(maxPower);
        }
      }
    }
    
    void selfDrivePID(short power) {
      if (power > maxPowerPID) {
        power = maxPowerPID;
      }
      
      float distance = distanceSensor.getDistance();
      wifi.sendSensorData(distance);
      
      if (leftIR.detecting() && rightIR.detecting()) { 
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        motors.forward(power);
      } else if (!leftIR.detecting() && !rightIR.detecting()) { 
        leftMotor.setPower(minPowerPID);
        rightMotor.setPower(minPowerPID);
        motors.forward(minPowerPID);
      } else if (!leftIR.detecting() && rightIR.detecting()) { 
        motors.left(power);
      } else if (leftIR.detecting() && !rightIR.detecting()) {
        motors.right(power);
      }
    }
    
    void remoteControl(float currentSpeed) {
      float distance = distanceSensor.getDistance();
      wifi.sendSensorData(distance);
      
      if (distance < 10) {
        wifi.sendStopCommand();
      } else {
        wifi.sendNothing();
      }
      
      String command = wifi.readCommand();
      
      if (command.startsWith("S:") || command.startsWith("SPEED:")) {
        int speedPos = command.indexOf("SPEED:");
        if (speedPos != -1) {
          String speedVal = command.substring(speedPos + 6);
          currentSliderValue = speedVal.toInt();
          Serial.println("Speed value read: " + String(currentSliderValue));
        }
      }
      
      unsigned long currentTime = millis();
      if (currentTime - lastTimeRight >= wait) {
        InputMotor = currentSpeed;
        SetpointMotor = currentSliderValue;
        
        if (abs(InputMotor - SetpointMotor) >= PIDTolerence) {
          motorPID.SetTunings(KpM, KiM, KdM);
          motorPID.Compute();
          
          Serial.print(SetpointMotor);
          Serial.print(",");
          Serial.println(InputMotor);
          
          selfDrivePID(short(OutputMotor));
        }
        lastTimeRight = currentTime;
      }
    }
    
    void followObjectMode() {
      float currentDistance = distanceSensor.getDistance();
      
      distancePID.SetTunings(KpD, KiD, KdD);
      InputDist = currentDistance;
      distancePID.Compute();
      
      if (currentDistance >= SetpointDist) {
        selfDrivePID(maxPower + OutputDist);
      } else if (currentDistance < 15 && currentDistance >= 10) {
        selfDrivePID(maxPower - OutputDist);
      } else if (currentDistance < 10) {
        motors.stop();
      }
    }
    
    void processCommands() {
      String command = wifi.readCommand();
      
      if (!command.isEmpty()) {
        lastCommand = command;
      }
      
      if (lastCommand == "Stop") {
        motors.stop();
      }
      
      if (lastCommand != prevCommand) {
        if (lastCommand == "FollowObject") {
          driveMode = MODE_FOLLOW_OBJECT;
          Serial.println("Drive mode: Follow Object");
          wifi.sendDriveMode(MODE_FOLLOW_OBJECT);
        } else if (lastCommand == "Auto") {
          driveMode = MODE_AUTO;
          Serial.println("Drive mode: Auto");
          wifi.sendDriveMode(MODE_AUTO);
        } else if (lastCommand == "Remote") {
          driveMode = MODE_REMOTE;
          Serial.println("Drive mode: Remote");
          wifi.sendDriveMode(MODE_REMOTE);
        }
        prevCommand = lastCommand;
      }
    }
    
    void run() {
      processCommands();
      
      unsigned long currentMillis = millis();
      if (currentMillis - prevMillis >= 1000) {
        prevMillis = currentMillis;
        
        float totalDistance = motors.getTotalDistance();
        wifi.sendDistanceData(totalDistance);
        
        float speed = motors.getAverageSpeed();
        wifi.sendSpeedData(speed);
        
        motors.resetEncoders();
        
        switch (driveMode) {
          case MODE_AUTO:
            selfDrive();
            break;
          case MODE_REMOTE:
            remoteControl(speed);
            break;
          case MODE_FOLLOW_OBJECT:
            followObjectMode();
            break;
          case MODE_TEST:
            // Testing mode - do nothing
            break;
        }
      }
    }
};

// Declare global instances
Motor leftMotor(LMotor1, LMotor2, LPower, LEncoder, &leftEncoderCount);
Motor rightMotor(RMotor1, RMotor2, RPower, REncoder, &rightEncoderCount);
MotorController motorController(leftMotor, rightMotor);
IRSensor leftIR(LIR, "Left IR");
IRSensor rightIR(RIR, "Right IR");
UltrasonicSensor distanceSensor(TrigPin, EchoPin);
WiFiCommunication wifiComm(server);
RobotController robotController(
  leftMotor, rightMotor, motorController,
  leftIR, rightIR, distanceSensor,
  wifiComm, matrix, MotorPID, DistancePID
);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; 
  }
  
  // Set up initial state
  leftMotor.setPower(initLPower);
  rightMotor.setPower(initRPower);
  
  // Setup WiFi
  wifiComm.setup(ssid, password);
  
  // Setup interrupts for encoders
  motorController.setupInterrupts();
  
  // Setup PID controllers
  robotController.setupPID();
  
  // Setup LED Matrix
  robotController.setupLEDMatrix();
}

void loop() {
  // Run the robot controller
  robotController.run();
}