import processing.net.*;
import controlP5.*;
import meter.*;

Client client;
ControlP5 p5;
PFont Font;

boolean swState = false;

char lastCom;

String data;
void setup() {
client = new Client(this,"10.151.52.172",3000);
//client.write("I am a new client\n"); 
 
size(400, 700);
p5 = new ControlP5(this);

background(255);

Font = createFont("Arial", 80);

Button Stop = p5.addButton("Stop").setPosition(0, 400).setSize(400, 300).setLabel("Stop");
Button Forward = p5.addButton("Start").setPosition(0, 100).setSize(400, 300).setLabel("Start");

Forward.setFont(Font);
Stop.setFont(Font);

Forward.setColorBackground(color(0, 0, 100)) 
          .setColorForeground(color(0, 0, 150)) 
          .setColorActive(color(0, 0, 255))     
          .setColorCaptionLabel(color(255));  
          
Stop.setColorBackground(color(80, 0, 0)) 
          .setColorForeground(color(120, 0, 0)) 
          .setColorActive(color(255, 0, 0))     
          .setColorCaptionLabel(color(255));  
          
fill(0);
textSize(50);



         
//Button Left = p5.addButton("Left").setPosition(175, 225).setSize(225, 125).setLabel("Left");
//Button Right = p5.addButton("Right").setPosition(625, 225).setSize(225, 125).setLabel("Right");
//Button Backwards = p5.addButton("Backwards").setPosition(400, 350).setSize(225, 125).setLabel("Backwards");
//Toggle Mode = p5.addToggle("Mode").setPosition(400, 550).setSize(225, 125).setValue(true).setCaptionLabel("").setLabel("Toggle Mode");
          
//Left.setColorBackground(color(0, 0, 100)) 
//          .setColorForeground(color(0, 0, 150)) 
//          .setColorActive(color(0, 0, 255))     
//          .setColorCaptionLabel(color(255));  
          
//Right.setColorBackground(color(0, 0, 100)) 
//          .setColorForeground(color(0, 0, 150))
//          .setColorActive(color(0, 0, 255))     
//          .setColorCaptionLabel(color(255));  
          
//Backwards.setColorBackground(color(0, 0, 100)) 
//          .setColorForeground(color(0, 0, 150)) 
//          .setColorActive(color(0, 0, 255))     
//          .setColorCaptionLabel(color(255));  
          


//Mode.getCaptionLabel()
//    .align(ControlP5.CENTER, ControlP5.CENTER) 
//    .setPadding(0, 0) 
//    .setColor(color(0)); 
//
}

void draw() {
  
  
  
  if(client.available() != 0) {
      
      String distance = client.readStringUntil('\n');
      if (distance != null) {
        distance = distance.trim();
        //if (distance.startsWith("D:")){
        //String distanceStr = distance.substring(2);
      if (float(distance) <= 10 && lastCom != 'S'){
        textSize(50);
        background(255);
        //print(distance);
        println("Obstacle ahead, stopping.");
        //float doneDist = float(distanceStr);
        text("Distance: " + distance + " cm", 15, 65);
      } 
     }
  } 
} 
    
      
 //if (swState) {
    //fill(0, 255, 0); // Green for ON
    //text("Remote-Control", 475, 730);
  //} else {
  //  fill(255, 0, 0); // Red for OFF
  //  text("Self-Control", 475, 730);
  //}
  
    
  
  //while (client.available() > 0) {
  //  char driveMode = (char) client.read();
  //  if (mode == '
  //  )
  //}
//}
 
 
void Start() {sendCommand('F'); lastCom = 'F';}
void Left() {sendCommand('L');}
void Right() {sendCommand('R');}
void Backwards() {sendCommand('Y');}
void Stop() {
  sendCommand('S');
  lastCom = 'S';
  background(255);
  delay(200);
  if (client.available() != 0){
  textSize(30);
  String distanceTrav = client.readStringUntil('\n');
  distanceTrav = distanceTrav.trim();
  if (distanceTrav.startsWith("S:")){
  String distTravStr = distanceTrav.substring(2);
  float distTravFloat = float(distTravStr);
  println(distTravFloat);
  //text("Distance Travelled: " + distTravFloat + " m", 15, 65);
  }
  }
}

//void Mode(boolean state) {
//    swState = state;
//   if (client.active()){
//    if (state) {
//      sendCommand('A');
//    } else {
//     sendCommand('B'); 
//    }
//  }
//}

void sendCommand (char command) {
  if (client.active()) {
    println("Command Sent: " + command);
    client.write(command);
  } else {
    println("No Connection");
  }
}

 
 
 
 
 
 
 

 
