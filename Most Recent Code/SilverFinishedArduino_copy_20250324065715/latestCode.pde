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

Meter m;

void setup() {
  client = new Client(this, "172.20.10.4", 3000);
  //client.write("I am a new client\n"); 
   
  size(600, 800);
  p5 = new ControlP5(this);

  background(215, 218, 222);

  Font = createFont("Arial", 30);
  Font1 = createFont("Arial", 15);

  Button Stop = p5.addButton("Stop").setPosition(305, 210).setSize(265, 80).setLabel("Stop");
  Button Forward = p5.addButton("Start").setPosition(30, 210).setSize(265, 80).setLabel("Start");
  //Button FollowObject = p5.addButton("Follow_Object").setPosition(290, 300).setSize(240, 80).setLabel("Follow object");
  //Button Auto = p5.addButton("Auto").setPosition(30, 390).setSize(170, 60).setLabel("Auto");
  //Button Remote = p5.addButton("Remote").setPosition(30, 300).setSize(240, 80).setLabel("Remote");
  Button FollowObject = p5.addButton("Follow_Object").setPosition(390, 300).setSize(170, 80).setLabel("Follow object");
  Button Auto = p5.addButton("Auto").setPosition(30, 300).setSize(170, 80).setLabel("Auto");
  Button Remote = p5.addButton("Remote").setPosition(210, 300).setSize(170, 80).setLabel("Remote");


  // Initialize slider and attach it to the sliderValue variable
  ChangeSpeed2 = p5.addSlider("SliderValue")
    .setPosition(30, 390)
    .setSize(540, 80)
    .setRange(0, 30)
    .setValue(0.00);

  Forward.setFont(Font);
  Stop.setFont(Font);
  FollowObject.setFont(Font1);
  Remote.setFont(Font1);
  Auto.setFont(Font1);

//  Forward.setColorBackground(color(130, 131, 155)) 
//         .setColorForeground(color(181, 183, 216))   
//         .setColorActive(color(181, 183, 216))     
//         .setColorCaptionLabel(color(255));
  Forward.setColorBackground(color(51, 152, 66))      
  .setColorForeground(color(173, 255, 173))       
  .setColorActive(color(120, 255, 120))           
  .setColorCaptionLabel(color(0)); 
  //Stop.setColorBackground(color(130, 131, 155))    
  //     .setColorForeground(color(181, 183, 216))   
  //     .setColorActive(color(181, 183, 216))     
  //     .setColorCaptionLabel(color(255));
   Stop.setColorBackground(color(203, 56, 18))     
  .setColorForeground(color(255, 190, 190))      
  .setColorActive(color(255, 100, 100))          
  .setColorCaptionLabel(color(0));  
      
 FollowObject.setColorBackground(color(26, 36, 201))   
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

  m = new Meter(this, 50, 500);
  
  m.setTitleFontSize(20);
  m.setTitleFontName("Arial");
  m.setTitle("Speed, cm/s");
  
  String[] scaleLabels = {"0" ,"5", "10", "15", "20", "25", "30"};
  m.setScaleLabels(scaleLabels);
  m.setScaleFontSize(10);
  m.setScaleFontName("Arial");
  m.setScaleFontColor(color(200, 30, 70));
  m.setDisplayDigitalMeterValue(true);
  
  m.setMaxScaleValue(30);
  m.setMinInputSignal(0);
  m.setMaxInputSignal(30);
  
  m.setNeedleThickness(3);
  
            
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
  float speed = readSpeed();
  
  int speedInt = (int)speed;
  
  m.updateMeter(speedInt);
  
  // Display the current slider value
  if (sliderValue != prevSliderValue) {
    MotorSpeed(int(sliderValue));
    prevSliderValue = sliderValue;
  }
  text("SPEED: " + speed, 30, 50);
  text ("REFERENCE SPEED (cm/s):   " + nf(sliderValue, 0, 2), 30, 80);
  
  text("DISTANCE TRAVELLED (cm):   " + distanceTravelled, 30, 130);
  
  text("DISTANCE FROM OBJECT (cm):   " + distanceAway, 30, 180);
  
  //if(lastCommand = "Remote")
  
  //text("CURRENT MODE:   " + );
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
    //println("check1");
    if (speed != null) {
      //println("check2");
      if( speed.length() > 0) {
        //println("check3");
      speed = speed.trim();  // Clean whitespace
      if (speed.startsWith("Speed:")) { // Check if correct data format\
      //println("check3");
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
  } 
  return prevspeed;
}

// Callback for slider value changes
void SliderValue(float value) {
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
    client.write("S:" + changeSpeed + "\n");
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
      //println("Obstacle ahead, stopping.");
      text("Obstacle ahead, stopping.", 30, 20);
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
