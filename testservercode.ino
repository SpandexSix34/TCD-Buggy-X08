#include <WiFiS3.h>
char ssid[] = "Pixel 9";
char pass[] = "nfhu5637";

WiFiServer server(5200);

void setup() {
 Serial.begin(9600);

 WiFi.begin(ssid, pass);

 IPAddress ip = WiFi.localIP();

 Serial.print("IP Address:");
 Serial.println(ip);
 server.begin();
}
void loop() {
  WiFiClient client = server.available();

  if (client.connected()) {
    client.write("Client Connected");
  } else {
    Serial.println("Client not connected");
    delay(1000);
  }
}
