import processing.net.*;
import processing.serial.*;
import controlP5.*;
import meter.*;

Client client;
ControlP5 p5;
PFont Font;
PFont Font1;

boolean swState = false;

String lastCom;
float prevDistance = 0;
float prevDistanceAway = 0;
float prevSliderValue = 0;
float prevspeed = 0;

//float ChangeSpeed2 = 0;      // Current motor speed (0-100%)
//float maxSpeed = 225; 

String data;
float sliderValue = 0;  // Centralized slider value variable
Slider ChangeSpeed2;  // Declare slider at class level
String lastMessage = "";

void setup() {
  client = new Client(this, "172.20.10.4", 3000);
  //client.write("I am a new client\n"); 
   
  size(400, 700);
  p5 = new ControlP5(this);

  background(215, 218, 222);

  Font = createFont("Arial", 30);
  Font1 = createFont("Arial", 15);

  Button Stop = p5.addButton("Stop").setPosition(210, 410).setSize(170, 60).setLabel("Stop");
  Button Forward = p5.addButton("Start").setPosition(30, 410).setSize(170, 60).setLabel("Start");
  Button FollowObject = p5.addButton("Follow_Object").setPosition(30, 240).setSize(170, 60).setLabel("Follow object");
  Button ChangeSpeed = p5.addButton("Change_Speed").setPosition(210, 240).setSize(170, 60).setLabel("Change speed");
  Button Auto = p5.addButton("Auto").setPosition(30, 182).setSize(170, 50).setLabel("Auto");
  Button Remote = p5.addButton("Remote").setPosition(210, 182).setSize(170, 50).setLabel("Remote");
  Button Left = p5.addButton("Left").setPosition(30, 550).setSize(170, 60).setLabel("Left");
  Button Right = p5.addButton("Right").setPosition(210, 550).setSize(170, 60).setLabel("Right");
  Button Backward = p5.addButton("Backward").setPosition(120, 620).setSize(170, 60).setLabel("Backward");

  // Initialize slider and attach it to the sliderValue variable
  ChangeSpeed2 = p5.addSlider("updateSliderValue")
    .setPosition(30, 320)
    .setSize(350, 60)
    .setRange(0, 255)
    .setValue(0.00);

  Forward.setFont(Font);
  Stop.setFont(Font);
  FollowObject.setFont(Font1);
  ChangeSpeed.setFont(Font1);
  Auto.setFont(Font1);

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
         
  ChangeSpeed.setColorBackground(color(26, 36, 201))   
         .setColorForeground(color(110, 117, 242))  
         .setColorActive(color(110, 117, 242))      
         .setColorCaptionLabel(color(255));

 Auto.setColorBackground(color(26, 36, 201))  
         .setColorForeground(color(110, 117, 242))        
         .setColorActive(color(110, 117, 242))       
         .setColorValue(color(255));

 Remote.setColorBackground(color(26, 36, 201))  
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
 // drawSpeedometer(200, 600, 90);
  //WheelEncoder();
  //readDistance();
  //readMode();
  
  float distanceTravelled = WheelEncoder();
  float distanceAway = readDistance();
  float speed = readSpeed();
  
  // Display the current slider value
  if (sliderValue != prevSliderValue) {
    MotorSpeed(int(sliderValue));
    prevSliderValue = sliderValue;
  }
  text("SPEED: " + speed, 30, 80);
  
  text("DISTANCE TRAVELLED:   " + distanceTravelled, 30, 130);
  
  text("DISTANCE FROM OBJECT:   " + distanceAway, 30, 180);
}


float readDistance() {
  if (client.active()) {
    textSize(25);
    String distanceAway = client.readStringUntil('\n'); // Read data
    if (distanceAway != null && distanceAway.length() > 0) {
      distanceAway = distanceAway.trim();  // Clean whitespace
      if (distanceAway.startsWith("Sensor:")) { // Check if correct data format
        String distanceAwayStr = distanceAway.substring(7); // Extract number
        float distanceAwayFloat = float(distanceAwayStr);
        if (distanceAwayFloat != prevDistanceAway) {
           prevDistanceAway = distanceAwayFloat;
           //println("distanceAway = " + distanceAwayStr);
            return distanceAwayFloat;
        }
      }
    } 
  } 
  return prevDistanceAway;
}
  
  
 float readSpeed() {
  if (client.active()) {
    textSize(25);
    String speed = client.readStringUntil('\n'); // Read data
    if (speed != null && speed.length() > 0) {
      speed = speed.trim();  // Clean whitespace
      if (speed.startsWith("Speed:")) { // Check if correct data format
        String speedStr = speed.substring(7); // Extract number
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

void Left() {
  sendCommand("Left");
  lastCom = "Left";
}

void Right() {
  sendCommand("Right");
  lastCom = "Right";
}

void Backward() {
  sendCommand("Backward");
  lastCom = "Backward";
}

void Remote() {
  sendCommand("Remote");
  lastCom = "Remote";
}

void Follow_Object() {
  sendCommand("FollowObject");
  lastCom = "FollowObject";
}
void Auto() {
  sendCommand("Auto");
  lastCom = "Auto";
}

void Change_Speed() {
  // Send the current slider value when the Change Speed button is pressed
  sendCommand("ChangeSpeed");
  //MotorSpeed(int(sliderValue));
  
  lastCom = "SpeedChange";
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

//void readMode(){
//  if (client.active()) {
//    String mode = client.readStringUntil('\n');
//    //mode.trim();
//    if(mode == "drive mode 1") {
//      println("drive mode 1");
//     }
//     if(mode == "drive mode 2") {
//       println("drive mode 2");
//     }
//     if(mode == "drive mode 0") {
//       println("drive mode 0");
//     }
//  }
//}



void USSensor() {
  if (client.active()) {
    textSize(25);
    String obstaclesAhead = client.readStringUntil('\n');

    if (obstaclesAhead != null && obstaclesAhead.trim().length() > 0) {
      lastMessage = obstaclesAhead.trim(); 
    }
    if (lastMessage.equals("stop")) {
      //println("Obstacle ahead, stopping.");
      text("Obstacle ahead, stopping.", 30, 50);
    } else if (lastMessage.equals("nothing")) {
      fill(215, 218, 222);
      rect(270, 100, 80, 40);
      fill(0);
    }
  }
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


//void drawSpeedometer(float x, float y, float radius) {
//  // Draw speedometer background
//  noStroke();
//  fill(50);
//  ellipse(x, y, radius * 2.2, radius * 2.2);
//  fill(30);
//  ellipse(x, y, radius * 2, radius * 2);
  
//  // Draw speedometer ticks and labels
//  stroke(200);
//  strokeWeight(2);
//  fill(255);
//  textAlign(CENTER, CENTER);
  
//  // Draw major ticks and labels
//  for (int i = 0; i <= 10; i++) {
//    float angle = PI * 0.8 + i * (PI * 1.4 / 10);
//    float speed = i * maxSpeed / 10;
    
//    // Draw major tick
//    float outerX = x + cos(angle) * radius;
//    float outerY = y + sin(angle) * radius;
//    float innerX = x + cos(angle) * (radius - 15);
//    float innerY = y + sin(angle) * (radius - 15);
//    line(innerX, innerY, outerX, outerY);
    
//    // Draw label
//    float labelX = x + cos(angle) * (radius - 30);
//    float labelY = y + sin(angle) * (radius - 30);
//    textSize(12);
//    text(int(speed), labelX, labelY);
//  }
  
//  // Draw unit label
//  fill(200);
//  textSize(14);
//  text("RPM", x, y + 40);
  
//  // Draw current speed value in the center
//  fill(255);
//  textSize(24);
//  text(int(ChangeSpeed2), x, y);
  
//  // Draw needle
//  float needleAngle = PI * 0.8 + (motorSpeed / maxSpeed) * PI * 1.4;
//  float needleLength = radius - 10;
  
//  stroke(255, 0, 0);
//  strokeWeight(3);
//  line(x, y, x + cos(needleAngle) * needleLength, y + sin(needleAngle) * needleLength);
  
//  // Draw needle pivot
//  fill(200);
//  noStroke();
//  ellipse(x, y, 20, 20);
//  fill(100);
//  ellipse(x, y, 10, 10);
//}
