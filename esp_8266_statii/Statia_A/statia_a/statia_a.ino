//ESP8266 WiFi access point to test WiFi signals, no other use ;-)
//Just aprovide power and check (laptop, phone) if you see and SSID "MyWiFi"

//................................................//
//.................Acces Point....................//
//................................................//

#include <ESP8266WiFi.h>
//#include <WiFi.h>

// Acces Point credentiale // 
const char* ssid = "Statia_A";
const char* password = "12345678";

void setup() {
  Serial.begin(115200);
  // Configurează ESP32 ca Access Point
  WiFi.softAP(ssid, password);
}

void loop() {
  // Statiile doar furnizeaza o retea wifi
}
