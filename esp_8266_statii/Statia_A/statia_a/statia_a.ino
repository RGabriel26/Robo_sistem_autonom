//ESP8266 WiFi access point to test WiFi signals, no other use ;-)
//Just aprovide power and check (laptop, phone) if you see and SSID "MyWiFi"

//................................................//
//.................Acces Point....................//
//................................................//

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Zona_A";
const char* password = "12345678";

WiFiUDP udp;
const IPAddress receiverIP(192, 168, 4, 2); // IP ESP32
const unsigned int receiverPort = 4210;

#define PIN_TRANSMIT D6 // IR LED
#define PIN_RECEIVE  D5 // IR Receiver

// Pini pentru LED de stare
#define PIN_HIGH D1
#define PIN_LOW  D2

bool irActive = false;
unsigned long lastToggle = 0;

unsigned long ultimaHigh = 0;
int prezentaObiect = 1;
unsigned long lastTransmit = 0;

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);

  pinMode(PIN_RECEIVE, INPUT);
  pinMode(PIN_TRANSMIT, OUTPUT);

  // pini pentru ledul de stare
  pinMode(PIN_HIGH, OUTPUT);
  pinMode(PIN_LOW, OUTPUT);
  digitalWrite(PIN_HIGH, HIGH); // Setează PIN_HIGH la HIGH
  digitalWrite(PIN_LOW, LOW);   // Setează PIN_LOW la LOW

  analogWriteFreq(38000);
  analogWrite(PIN_TRANSMIT, 128); // pornit la start
}

void loop() {
  unsigned long now = millis();

  // 1. PWM on/off la 50 Hz (10ms ON/OFF)
  if (now - lastToggle >= 10) {
    lastToggle = now;
    irActive = !irActive;
    analogWrite(PIN_TRANSMIT, irActive ? 128 : 0);
  }

  // 2. Logică pentru detecția prezenței
  int starePin = digitalRead(PIN_RECEIVE);

  if (starePin == LOW) {
    ultimaHigh = now;
    prezentaObiect = 0;
  } else {
    if (now - ultimaHigh >= 200) {
      prezentaObiect = 1;
    }
  }

  // 3. Trimitere UDP la fiecare 500 ms
  if (now - lastTransmit >= 600) {
    lastTransmit = now;

    udp.beginPacket(receiverIP, receiverPort);
    udp.print(prezentaObiect);
    udp.endPacket();

    Serial.print("prezentaObiect = ");
    Serial.println(prezentaObiect);
  }
}
