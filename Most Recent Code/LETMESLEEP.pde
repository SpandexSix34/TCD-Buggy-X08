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

String data;
float sliderValue = 0;  // Centralized slider value variable
Slider ChangeSpeed2;    // Declare slider at class level

void setup() {
  client = new Client(this, "10.163.240.172", 3000);
  //client.write("I am a new client\n"); 
   
  size(400, 500);
  p5 = new ControlP5(this);

  background(215, 218, 222);

  Font = createFont("Arial", 30);
  Font1 = createFont("Arial", 15);

  Button Stop = p5.addButton("Stop").setPosition(210, 400).setSize(170, 60).setLabel("Stop");
  Button Forward = p5.addButton("Start").setPosition(30, 400).setSize(170, 60).setLabel("Start");
  Button FollowObject = p5.addButton("Follow_Object").setPosition(30, 230).setSize(170, 60).setLabel("Follow object");
  Button ChangeSpeed = p5.addButton("Change_Speed").setPosition(210, 230).setSize(170, 60).setLabel("Change speed");

  // Initialize slider and attach it to the sliderValue variable
  ChangeSpeed2 = p5.addSlider("updateSliderValue")
    .setPosition(30, 310)
    .setSize(350, 60)
    .setRange(0, 255)
    .setValue(0.00);

  Forward.setFont(Font);
  Stop.setFont(Font);
  FollowObject.setFont(Font1);
  ChangeSpeed.setFont(Font1);

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

  ChangeSpeed2.setColorBackground(color(26, 36, 201))  
         .setColorForeground(color(110, 117, 242))        
         .setColorActive(color(110, 117, 242))       
         .setColorValue(color(255));
            
  fill(0);
  textSize(25);
  textAlign(LEFT);
}

void draw() {
  // Clear the background for proper text refreshing
  background(215, 218, 222);
  
  USSensor();
  WheelEncoder();
  
  // Display the current slider value
  text("SPEED: " + int(sliderValue), 30, 80);
  text("DISTANCE TRAVELLED: " + 10, 30, 130);
  text("DISTANCE FROM OBJECT: " + 100, 30, 180);
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
  sendCommand("PID");
  lastCom = "PID";
}

void Change_Speed() {
  // Send the current slider value when the Change Speed button is pressed
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
    if (obstaclesAhead != null && obstaclesAhead.trim().equals("stop")) {
      println("Obstacle ahead, stopping.");
      text("Obstacle ahead, stopping.", 30, 90);
    }
    if (obstaclesAhead != null && obstaclesAhead.trim().equals("nothing")) {
      fill(215, 218, 222);
      rect(270, 100, 80, 40);
      fill(0);
    }
  }
}
  
void WheelEncoder() {
  if (client.active()) {
    textSize(25);
    String distanceTravelled = client.readStringUntil('\n'); // Read data

    if (distanceTravelled != null && distanceTravelled.length() > 0) {
      distanceTravelled = distanceTravelled.trim();  // Clean whitespace
      if (distanceTravelled.startsWith("D:")) { // Check if correct data format
        String distanceValueStr = distanceTravelled.substring(2); // Extract number
        float distanceTravFloat = float(distanceValueStr);
        if (distanceTravFloat != prevDistance) {
          fill(215, 218, 222);
          rect(300, 150, 80, 40);
          fill(0);
          textAlign(CENTER, CENTER);
          println("Total distance: " + distanceTravFloat + " m");
          text(distanceTravFloat + "m", 300, 180);
          prevDistance = distanceTravFloat;
        }
      } 
    }
  }
}
