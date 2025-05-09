#include "motoare.h" // functii destinate deplasarii 
#include "antena_control.h" // funtii destinate orientarii antenei
#include "brat_control.h" // functii destinate controlului bratului apucator
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Arduino.h>
#include "driver/ledc.h"

// test git

/*
================================================================================
 FUNCȚII DISPONIBILE ÎN FIȘIERELE HEADER IMPORTATE
================================================================================

motoare.h:
  - FORWARD()           → deplasare înainte
  - BACK()              → deplasare înapoi
  - LEFT()              → rotire în stânga
  - RIGHT()             → rotire în dreapta
  - VERTICAL_LEFT()     → deplasare laterală stânga (roți magnum)
  - VERTICAL_RIGHT()    → deplasare laterală dreapta (roți magnum)
  - STOP()              → oprire completă a motoarelor

antena_control.h:
  - connect_statie(const char* ssid, const char* password)          → conectare la o rețea Wi-Fi specificată
  - det_unghi_orientare()                                           → returnează unghiul cu cel mai bun semnal RSSI detectat

brat_control.h:
  - init_brat()         → poziționare inițială a servomotoarelor brațului apucător
  - brat_prindere()     → secvență de prindere: capul apucă, brațul se ridică
  - brat_eliberare()    → secvență de eliberare: brațul coboară, capul eliberează

================================================================================
*/

// ANTENA
// log data pentru conectarea la statiile remote
const char* ssid_station_A = "Statia_A";
const char* password_station_A = "12345678";
const char* ssid_station_B = "Statia_B";
const char* password_station_B = "12345678";
const char* ssid_station_C = "Statia_C";
const char* password_station_C = "12345678";
// variabile pentru determinarea unghioului de orientare al antenei
int interval[2] = {LIM_INF , LIM_SUP}; // interval cu unghiurile pe care le va explora antena
Servo servoAntena; // instanta servo de control al servomotorului antenei

// BRAT APUCATOR
Servo servoBase;
Servo servoHead;

void setup() {
  // setare UART 0 in cazul debugging ului
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 35, -1);
  // initializare pini control motoare / drivere
  pinMode(DRIVER_A_IN1, OUTPUT);
  pinMode(DRIVER_A_IN2, OUTPUT);
  pinMode(DRIVER_A_IN3, OUTPUT);
  pinMode(DRIVER_A_IN4, OUTPUT);
  pinMode(DRIVER_B_IN1, OUTPUT);
  pinMode(DRIVER_B_IN2, OUTPUT);
  pinMode(DRIVER_B_IN3, OUTPUT);
  pinMode(DRIVER_B_IN4, OUTPUT);
  pinMode(pinPWM, OUTPUT);

  // initializarea servo antenei 
  pinMode(pinServoAntena, OUTPUT);
  servoAntena.attach(pinServoAntena); 

  // initializare serbo brat
  pinMode(pinServoBase, OUTPUT);
  pinMode(pinServoHead, OUTPUT);
  servoBase.attach(pinServoBase);
  servoHead.attach(pinServoHead);

  delay(100);
  servoAntena.write(90);
  init_brat();

  // initializare tasks
  // TEST
  xTaskCreatePinnedToCore(
    taskAntena,       // funcția
    "TaskAntena",     // nume task
    4096,             // stack size
    NULL,             // parametru
    1,                // prioritate
    NULL,             // handle (optional)
    0                 // core 0
  );

  xTaskCreatePinnedToCore(
    taskControl,
    "TaskControl",
    4096,
    NULL,
    1,
    NULL,
    1               // rulează pe core 1
  );

  // stare finala
  digitalWrite(pinPWM, HIGH);
}

// Task 1: Verificare conexiune si orientare antena
void taskAntena(void *parameter) {
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      servoAntena.write(90);
      connect_statie(ssid_station_C, password_station_C);
    } else {
      Serial.print("valoare unghi: ");
      Serial.println(det_unghi_orientare());
    }
    vTaskDelay(0); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}

// Task 2: Control motoare si brat robotic
void taskControl(void *parameter) {
  int valoare = 0;
  while (true) {
    if (Serial.available() > 0) {
      valoare = Serial.parseInt();
      if (valoare != '\n' && valoare != '\r' && valoare != '\0') {
        switch (valoare) {
          case 1: FORWARD(); break;
          case 2: BACK(); break;
          case 3: LEFT(); break;
          case 4: RIGHT(); break;
          case 5: VERTICAL_LEFT(); break;
          case 6: VERTICAL_RIGHT(); break;
          case 7: brat_prindere(); break;
          case 8: brat_eliberare(); break;
          case 9: STOP(); break;
          default: STOP(); break;
        }
      }
    } else LEFT();
    int dataUART = 0;
    if (Serial1.available() > 0) {
      dataUART = Serial1.parseInt();
      switch (dataUART) {
        case 1: brat_prindere(); break;
        case 2: brat_eliberare(); break;
        default: break;
      }
    }
    vTaskDelay(0); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}


void loop() {
}
