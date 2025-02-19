import processing.net.*;
import controlP5.*;
import meter.*;

Client myClient;
ControlP5 p5;

int myColour = color(255, 255, 255);

Button Forward;

String data;
void setup() {
 myClient = new Client(this,"192.168.147.172",5200);
 myClient.write("I am a new client"); 
 
 size(1000, 800);
 p5 = new ControlP5(this);
 
p5.addButton("Forward").setPosition(100, 100).setSize(200,20);
 
}

void draw() {
 background(myColour);
}

public void controlEvent(ControlEvent ev) { 
   println(ev.getController().getName());
 }
 
 public void Forward(){
   myColour = color(255, random(255), random(255)); 
 }
