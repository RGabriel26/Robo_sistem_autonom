//ESP8266 WiFi access point to test WiFi signals, no other use ;-)
//Just aprovide power and check (laptop, phone) if you see and SSID "MyWiFi"

//................................................//
//.................Acces Point....................//
//................................................//

#include <ESP8266WiFi.h>
//#include <WiFi.h>

// Acces Point credentiale // 
const char* ssid = "Zona_B";
const char* password = "12345678";
// const char* ssid = "Zona_C";
// const char* password = "12345678";

// Pini pentru LED de stare
// pentru Zona_B
#define PIN_HIGH D1
#define PIN_LOW  D2
// pentru Zona_C
// #define PIN_HIGH D5
// #define PIN_LOW  D6

void setup() {
  Serial.begin(115200);
  // Configurează ESP32 ca Access Point
  WiFi.softAP(ssid, password);

  // Configurează pini pentru LED de stare
  pinMode(PIN_HIGH, OUTPUT);
  pinMode(PIN_LOW, OUTPUT);
  digitalWrite(PIN_HIGH, HIGH); // Setează PIN_HIGH la HIGH
  digitalWrite(PIN_LOW, LOW);   // Setează PIN_LOW la LOW
}

void loop() {
}
