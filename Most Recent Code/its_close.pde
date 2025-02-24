import processing.net.*;
import controlP5.*;
import meter.*;

Client client;
ControlP5 p5;

boolean swState = false;

String data;
void setup() {
 client = new Client(this,"192.168.254.172",5200);
 client.write("I am a new client"); 
 
 size(1000, 800);
 p5 = new ControlP5(this);
 
Button Forward = p5.addButton("Forward").setPosition(400, 100).setSize(225, 125).setLabel("Forward");
Button Left = p5.addButton("Left").setPosition(175, 225).setSize(225, 125).setLabel("Left");
Button Right = p5.addButton("Right").setPosition(625, 225).setSize(225, 125).setLabel("Right");
Button Backwards = p5.addButton("Backwards").setPosition(400, 350).setSize(225, 125).setLabel("Backwards");
Button Stop = p5.addButton("Stop").setPosition(400, 225).setSize(225, 125).setLabel("Stop");
Toggle Mode = p5.addToggle("Mode").setPosition(400, 550).setSize(225, 125).setValue(false).setCaptionLabel("").setLabel("Toggle Mode");

Forward.setColorBackground(color(0, 0, 100)) 
          .setColorForeground(color(0, 0, 150)) 
          .setColorActive(color(0, 0, 255))     
          .setColorCaptionLabel(color(255));  
          
Left.setColorBackground(color(0, 0, 100)) 
          .setColorForeground(color(0, 0, 150)) 
          .setColorActive(color(0, 0, 255))     
          .setColorCaptionLabel(color(255));  
          
Right.setColorBackground(color(0, 0, 100)) 
          .setColorForeground(color(0, 0, 150))
          .setColorActive(color(0, 0, 255))     
          .setColorCaptionLabel(color(255));  
          
Backwards.setColorBackground(color(0, 0, 100)) 
          .setColorForeground(color(0, 0, 150)) 
          .setColorActive(color(0, 0, 255))     
          .setColorCaptionLabel(color(255));  
          
Stop.setColorBackground(color(80, 0, 0)) 
          .setColorForeground(color(120, 0, 0)) 
          .setColorActive(color(255, 0, 0))     
          .setColorCaptionLabel(color(255));  

Mode.getCaptionLabel()
    .align(ControlP5.CENTER, ControlP5.CENTER) 
    .setPadding(0, 0) 
    .setColor(color(0)); 
}

void draw() {
 background(255);
 
 if (swState) {
    fill(0, 255, 0); // Green for ON
    text("Remote-Control", 475, 730);
  } else {
    fill(255, 0, 0); // Red for OFF
    text("Self-Control", 475, 730);
  }
}
 
void Forward() {
  if (client.active()) {
    client.write("forward\n");
    println("Sent: Forward");
  }
}

void Left() {
  if (client.active()) {
    client.write("left\n");
    println("Sent: Forward");
  }
}

void Right() {
  if (client.active()) {
    client.write("right\n");
    println("Sent: Forward");
  }
}

void Backwards() {
  if (client.active()) {
    client.write("backwards\n");
    println("Sent: Forward");
  }
}

void Stop() {
  if (client.active()) {
    client.write("stop\n");
    println("Sent: Forward");
  }
}

void Mode(boolean state) {
    swState = state;
   if (client.active()){
    if (state) {
      client.write("remote control\n");
    } else {
     client.write("self control\n"); 
    }
  }
}


 
 
 
 
 
 
 
 
