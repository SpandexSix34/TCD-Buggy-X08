import processing.net.*;
import processing.serial.*;
import controlP5.*;
import meter.*;

Client client;
ControlP5 p5;
PFont Font;
PFont Font1;

boolean swState = false;
boolean pidActive = false; // Track PID state

float prevspeed = 0;

String lastCom;
float prevDistance = 0;
float prevDistanceAway = 0;

// Key and button states
boolean wPressed = false;
boolean aPressed = false;
boolean sPressed = false;
boolean dPressed = false;
boolean forwardButtonPressed = false;
boolean leftButtonPressed = false;
boolean rightButtonPressed = false;
boolean backButtonPressed = false;

String data;
float sliderValue = 0;  // Centralized slider value variable
Slider ChangeSpeed2;  // Declare slider at class level
String lastMessage = "";
long lastCommandTime = 0;
final int COMMAND_RATE = 100; // Send commands at most every 100ms

void setup() {
  client = new Client(this, "10.165.18.172", 3000);
   
  size(400, 800);
  p5 = new ControlP5(this);

  background(215, 218, 222);

  Font = createFont("Arial", 30);
  Font1 = createFont("Arial", 15);

  Button Stop = p5.addButton("Stop").setPosition(210, 465).setSize(170, 60).setLabel("Stop");
  Button Forward = p5.addButton("Start").setPosition(30, 465).setSize(170, 60).setLabel("Start");
  Button FollowObject = p5.addButton("Follow_Object").setPosition(30, 265).setSize(170, 60).setLabel("Follow object");
  Button RemoteControl = p5.addButton("RemoteControl").setPosition(210, 265).setSize(170, 60).setLabel("Remote Control");
  Button Combined = p5.addButton("Combined").setPosition(30, 330).setSize(170, 60).setLabel("Combined"); // New button for combined mode
  Button ChangeSpeed = p5.addButton("Change_Speed").setPosition(210, 330).setSize(170, 50).setLabel("Change speed");
  
  // Replace Auto button with a Toggle button
  Toggle pidToggle = p5.addToggle("togglePID")
    .setPosition(110, 190)
    .setSize(170, 60)
    .setMode(ControlP5.SWITCH)
    .setCaptionLabel("PID OFF")
    .setValue(false);
    
  Button Left = p5.addButton("Left").setPosition(30, 550).setSize(170, 60).setLabel("Left");
  Button Right = p5.addButton("Right").setPosition(210, 550).setSize(170, 60).setLabel("Right");
  Button Backward = p5.addButton("Backward").setPosition(120, 620).setSize(170, 60).setLabel("Backward");


  // Initialize slider and attach it to the sliderValue variable
  ChangeSpeed2 = p5.addSlider("updateSliderValue")
    .setPosition(30, 390)
    .setSize(350, 60)
    .setRange(0, 60)
    .setValue(0.00);

  Forward.setFont(Font);
  Stop.setFont(Font);
  FollowObject.setFont(Font1);
  RemoteControl.setFont(Font1);
  Combined.setFont(Font1); // Set font for the new button
  ChangeSpeed.setFont(Font1);
  Left.setFont(Font);
  Right.setFont(Font);
  Backward.setFont(Font);
  pidToggle.setFont(Font1);

  Forward.setColorBackground(color(130, 131, 155)) 
         .setColorForeground(color(181, 183, 216))   
         .setColorActive(color(181, 183, 216))     
         .setColorCaptionLabel(color(255));

  Stop.setColorBackground(color(130, 131, 155))    
       .setColorForeground(color(181, 183, 216))   
       .setColorActive(color(181, 183, 216))     
       .setColorCaptionLabel(color(255));
      
  FollowObject.setColorBackground(color(26, 36, 201))   
         .setColorForeground(color(110, 117, 242))  
         .setColorActive(color(110, 117, 242))      
         .setColorCaptionLabel(color(255));
         
  RemoteControl.setColorBackground(color(26, 36, 201))   
         .setColorForeground(color(110, 117, 242))  
         .setColorActive(color(110, 117, 242))      
         .setColorCaptionLabel(color(255));

  // Set colors for the new Both Modes button
  Combined.setColorBackground(color(26, 36, 201))   
         .setColorForeground(color(110, 117, 242))  
         .setColorActive(color(110, 117, 242))      
         .setColorCaptionLabel(color(255));
  
  // Set colors for the PID toggle      
  pidToggle.setColorBackground(color(26, 36, 201))   
         .setColorForeground(color(110, 117, 242))  
         .setColorActive(color(110, 117, 242))      
         .setColorCaptionLabel(color(255));
         
  ChangeSpeed.setColorBackground(color(26, 36, 201))   
         .setColorForeground(color(110, 117, 242))  
         .setColorActive(color(110, 117, 242))      
         .setColorCaptionLabel(color(255));

  ChangeSpeed2.setColorBackground(color(26, 36, 201))  
         .setColorForeground(color(110, 117, 242))        
         .setColorActive(color(110, 117, 242))       
         .setColorValue(color(255));
         
  Left.setColorBackground(color(130, 131, 155)) 
         .setColorForeground(color(181, 183, 216))   
         .setColorActive(color(181, 183, 216))     
         .setColorCaptionLabel(color(255));
         
  Right.setColorBackground(color(130, 131, 155)) 
         .setColorForeground(color(181, 183, 216))   
         .setColorActive(color(181, 183, 216))     
         .setColorCaptionLabel(color(255));
         
  Backward.setColorBackground(color(130, 131, 155)) 
         .setColorForeground(color(181, 183, 216))   
         .setColorActive(color(181, 183, 216))     
         .setColorCaptionLabel(color(255));
            
  fill(0);
  textSize(25);
  textAlign(LEFT);
}

void draw() {
  // Clear the background for proper text refreshing
  background(215, 218, 222);
  
  USSensor();
  
  float distanceTravelled = WheelEncoder();
  float distanceAway = readDistance();
  
  text("DISTANCE TRAVELLED:   " + distanceTravelled, 30, 130);
  
  text("DISTANCE FROM OBJECT:   " + distanceAway, 30, 180);
  
  float speed = readSpeed();
  text("SPEED: " + speed, 30, 80);
  
  // Show current PID state
  text("PID MODE: " + (pidActive ? "ON" : "OFF"), 30, 50);
}

// New toggle function for PID switch
void togglePID(boolean theFlag) {
  pidActive = theFlag;
  
  // Update toggle label
  Toggle t = (Toggle)p5.getController("togglePID");
  if (pidActive) {
    t.setCaptionLabel("PID ON");
    sendCommand("PID");
    lastCom = "PID";
  } else {
    t.setCaptionLabel("PID OFF");
    sendCommand("NOPID");
    lastCom = "NOPID";
  }
  
  println("PID state: " + (pidActive ? "ON" : "OFF"));
}

void mousePressed() {
  // Check if mouse is over buttons when pressed
  if (p5.getController("Start") != null && isMouseOver(p5.getController("Start"))) {
    forwardButtonPressed = true;
    sendCommand("Forward");
    lastCom = "Forward";
  } else if (p5.getController("Left") != null && isMouseOver(p5.getController("Left"))) {
    leftButtonPressed = true;
    sendCommand("Left");
    lastCom = "Left";
  } else if (p5.getController("Right") != null && isMouseOver(p5.getController("Right"))) {
    rightButtonPressed = true;
    sendCommand("Right");
    lastCom = "Right";
  } else if (p5.getController("Backward") != null && isMouseOver(p5.getController("Backward"))) {
    backButtonPressed = true;
    sendCommand("Backward");
    lastCom = "Backward";
  }
}

void mouseReleased() {
  // Check if any movement buttons were pressed
  if (forwardButtonPressed || leftButtonPressed || rightButtonPressed || backButtonPressed) {
    sendCommand("Stop");
    lastCom = "Stop";
    
    // Reset all button states
    forwardButtonPressed = false;
    leftButtonPressed = false;
    rightButtonPressed = false;
    backButtonPressed = false;
  }
}

// Helper function to check if mouse is over a controller
boolean isMouseOver(Controller c) {
  // For ControlP5 controllers
  float[] position = c.getPosition();
  return mouseX >= position[0] && 
         mouseX <= position[0] + c.getWidth() && 
         mouseY >= position[1] && 
         mouseY <= position[1] + c.getHeight();
}

// For button press handling
public void controlEvent(ControlEvent theEvent) {
  // We only need to check if the event is from a controller
  if (theEvent.isController()) {
    String controllerName = theEvent.getController().getName();
    
    // No need to handle button presses here since you already have:
    // 1. Individual button callbacks (Start(), Stop(), etc.)
    // 2. mousePressed() and mouseReleased() for tracking button states
    
    // You could log the event if desired
    println("Control event: " + controllerName);
  }
}

float readDistance() {
  if (client.active()) {
    textSize(25);
    String distanceAway = client.readStringUntil('\n'); // Read data
    if (distanceAway != null && distanceAway.length() > 0) {
      distanceAway = distanceAway.trim();  // Clean whitespace
      if (distanceAway.startsWith("Sensor:")) { // Check if correct data format
        String distanceAwayStr = distanceAway.substring(7); // Extract number (fixed the index)
        float distanceAwayFloat = float(distanceAwayStr);
        if (distanceAwayFloat != prevDistanceAway) {
           prevDistanceAway = distanceAwayFloat;
           println("distanceAway = " + distanceAwayStr);
            return distanceAwayFloat;
        }
      }
    } 
  } 
  return prevDistanceAway;
}
  
// Callback for slider value changes
void updateSliderValue(float value) {
  sliderValue = value;
}

void Start() {
  sendCommand("Forward"); 
  lastCom = "Forward";
}

void Stop() {
  sendCommand("Stop");
  lastCom = "Stop";
}

void Follow_Object() {
  sendCommand("Follow");
  lastCom = "Follow";
}

// Add the BothModes function to handle the new button
void Combined() {
  sendCommand("Combined");
  lastCom = "Combined";
}

void Left() {
  sendCommand("Left");
  lastCom = "Left";
}

void Right() {
  sendCommand("Right");
  lastCom = "Right";
}

void RemoteControl() {
  sendCommand("Remote");
  lastCom = "Remote";
}

void Backward() {
  sendCommand("Backward");
  lastCom = "Backward";
}

// Removed Auto() function since it's replaced by the toggle

void Change_Speed() {
  // Send the current slider value when the Change Speed button is pressed
  //sendCommand("SpeedChange");
  MotorSpeed(int(sliderValue));
  lastCom = "NewSpeed";
}

void sendCommand(String command) {
  if (client.active()) {
    println("Command Sent: " + command);
    client.write(command + "\n");
  } else {
    println("No Connection");
  }
}

void MotorSpeed(int changeSpeed) {
  if (client.active()) {
    println("Sent Speed: " + changeSpeed);
    // Send the speed value as a string to make it easier to parse on the Arduino side
    client.write("SPEED:" + changeSpeed + "\n");
  } else {
    println("No Connection");
  }
}

void USSensor() {
  if (client.active()) {
    textSize(25);
    String obstaclesAhead = client.readStringUntil('\n');

    if (obstaclesAhead != null && obstaclesAhead.trim().length() > 0) {
      lastMessage = obstaclesAhead.trim(); 
    }
    if (lastMessage.equals("stop")) {
      println("Obstacle ahead, stopping.");
      text("Obstacle ahead, stopping.", 30, 50);
    } else if (lastMessage.equals("nothing")) {
      fill(215, 218, 222);
      rect(270, 100, 80, 40);
      fill(0);
    }
  }
}

float readSpeed() {
  if (client.active()) {
    textSize(25);
    String speed = client.readStringUntil('\n'); // Read data
    if (speed != null && speed.length() > 0) {
      speed = speed.trim();  // Clean whitespace
      if (speed.startsWith("Speed:")) { // Check if correct data format
        String speedStr = speed.substring(6); // Extract number
        float speedFloat = float(speedStr);
        if (speedFloat != prevspeed) {
           prevspeed = speedFloat;
           println("actual speed = " + speedStr);
            return speedFloat;
        }
      }
    }
  }
  return prevspeed;
}

float WheelEncoder() {
  if (client.active()) {
    textSize(25);
    String distanceTravelled = client.readStringUntil('\n'); // Read data
    if (distanceTravelled != null && distanceTravelled.length() > 0) {
      distanceTravelled = distanceTravelled.trim();  // Clean whitespace
      if (distanceTravelled.startsWith("D:")) { // Check if correct data format
        String distanceValueStr = distanceTravelled.substring(2); // Extract number
        float distanceTravFloat = float(distanceValueStr);
        if (distanceTravFloat != prevDistance) {
           prevDistance = distanceTravFloat;
           return distanceTravFloat;
        }
      }
    }
  }
  return prevDistance;
}
