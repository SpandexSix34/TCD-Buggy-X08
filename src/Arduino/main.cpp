#include "robot_functions.h"

WiFiServer server(3000);
ArduinoLEDMatrix matrix;

const uint32_t happy[] = {
    0x1f820,
    0x40000900,
    0x90090000,
    66
};

char ssid[] = "Pixel 9";        // your network SSID (name)
char password[] = "nfhu5637";   // your network password

unsigned long prevMillis = 0;
unsigned long prevMillis2 = 0;
String prevCommand = " ";

void setup() {
  LeftPID.SetMode(AUTOMATIC);
  LeftPID.SetOutputLimits(50, 255);

  RightPID.SetMode(AUTOMATIC);
  RightPID.SetOutputLimits(50, 255);

  // Set default speed setpoints
  SetpointLeft = 30; // Default speed in RPM
  SetpointRight = 30; // Default speed in RPM

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

  matrix.begin();

  while(!Serial) {
    ; // Wait for serial monitor to connect
  }

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

  attachInterrupt(digitalPinToInterrupt(LEncoder), speedLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(REncoder), speedRight, RISING);
}

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
      driveMode = 0;
    }
    else if (command == "NOPID") {
      pidDrive = false;
      Serial.println("PID control disabled");
    }
    else if (command == "Self") {
      driveMode = 0;
      pidDrive = true;
      Serial.println("Switched to Self driving mode");
    } 
    else if (command == "Remote") {
      driveMode = 1;
      Serial.println("Switched to Remote control mode");
      pidDrive = false; // No PID as requested
    }
    else if (command == "Follow") {
      driveMode = 4; // New mode for object following
      pidDrive = false; // No PID as requested
      Serial.println("Switched to Follow object mode");
    }
    else if (command == "Combined") {
      driveMode = 5; // New mode for combined self-driving and object following
      pidDrive = false; // No PID as requested
      Serial.println("Switched to Combined mode (self-driving + object following)");
    }
    prevCommand = command;
  }
  
  // Execute the appropriate mode
  switch (driveMode) {
    case 0: // Self driving (line following) mode
      selfDriveNoPID();
      break;
    case 1: // Remote control mode
      remoteControl();
      break;
    case 4: // Object following mode
      followObjectModeNoPID();
      break;
    case 5: // Combined mode
      combinedModeNoPID(distanceToObject);
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
