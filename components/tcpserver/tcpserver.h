#include "esphome.h"
#include "esphome/core/log.h"

#ifdef ESP32              // depending on the microcontroller-type 
#include <WiFi.h>         // include the right library for ESP32
#elif defined(ESP8266)
#include <ESP8266WiFi.h>  // or ESP8266
#endif

// this tcp_server demo-code creates its own WiFi-network 
// where the tcp_client demo-code connects to
// the ssid and the portNumber must be the same to make it work

const char* ssid     = "ESP32-AP";
const uint16_t portNumber = 50000; // System Ports 0-1023, User Ports 1024-49151, dynamic and/or Private Ports 49152-65535

WiFiServer server(portNumber);
WiFiClient client;
bool connected = false;
class MyCustomComponent  : public Component  {
public:
void setup() override {
  WiFi.softAP(ssid);

  IPAddress IP = WiFi.softAPIP();
  server.begin();
}

void loop() override {

  char TCP_Char;
  char serialChar;
  //ESP_LOGD("custom","temp= %f",temperature);
  if (!connected) {
    // listen for incoming clients
    client = server.available();
    if (client) {
      ESP_LOGD("custom","Got a client connected to my WiFi !");
      if (client.connected()) {
        ESP_LOGD("custom","an now this client has connected over TCP!");
        connected = true;
      } else {
        ESP_LOGD("custom","but it's not connected over TCP!");      
        client.stop();  // close the connection:
      }
    }
  } 
  else {
    if (client.connected()) {
      
      // if characters sended from client is in the buffer
      while ( client.available() ) { 
        TCP_Char = client.read(); // take one character out of the TCP-receive-buffer
      }  

      // if characters have been typed into the serial monitor  
      
    } 
    else {
      ESP_LOGD("custom","Client has disconnected the TCP-connection");
      client.stop();  // close the connection:
      connected = false;
    }
  }
}
};