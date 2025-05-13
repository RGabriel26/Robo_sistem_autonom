#include "motoare.h" // functii destinate deplasarii 
#include "antena_control.h" // funtii destinate orientarii antenei
#include "brat_control.h" // functii destinate controlului bratului apucator
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Arduino.h>
#include "driver/ledc.h"

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
const char* ssid_statie[] = {"", "Statia_A", "Statia_B", "Statia_C"}; // indexul 0 este folosit atunci cand se doreste blocarea task ului taskAntena
const char* password_statie[] = {"", "12345678", "12345678", "12345678"};

// variabile pentru determinarea unghioului de orientare al antenei
int interval[2] = {LIM_INF , LIM_SUP}; // interval cu unghiurile pe care le va explora antena
Servo servoAntena; // instanta servo de control al servomotorului antenei

// BRAT APUCATOR
Servo servoBase;
Servo servoHead;

// Variabile globale
volatile int statieCurenta = 0;
volatile int prezentaObiect = 0;
volatile int valoareRSSI = 0;

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
  // Initializare statia A
  statieCurenta = 1; // conectare la statia A (pickup obiect)
  delay(1000);       // delay 1 sec pentru asteptarea verificarii prezentei obiectului

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
    taskControl_TEST,
    "TaskControl_TEST",
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
    if (WiFi.status() != WL_CONNECTED) { // daca nu este conectat, se incearca conectarea la statia curenta din pasul comportamental
      servoAntena.write(90);             // pozitionare antena in pozitie default
      connect_statie(ssid_statie[statieCurenta], password_statie[statieCurenta]); // conectare la statia curenta
    } else {
      valoareRSSI = det_unghi_orientare();
      Serial.println(det_unghi_orientare());
    }
    vTaskDelay(0); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}

// Task 2: Control motoare si brat robotic
void taskControl_TEST(void *parameter) {
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
  // FUNCTIA LOOP NU TREBUIE LASATA SA RULEZE LA INFINIT, AI NEVOIE DE FUNCTII BLOCANTE PENTRU A PASTRA O LOGICA DESFASURATA IN TIMP

  // statie initializata in functia setup - staita A
  // verificare daca este facuta conectiunea cu staita A
  if (prezentaObiect){
    // mentinerea conectiunii cu statia A
    // logica de comanda pentru deplasarea spre aceas zona
    // verificare proximitate

    // verificare detectie obiect pentru switch al controlului folosind datele din detectie pentru centrare
        // obiect in pozitia corecta
            // prindere
            // rotarie 180
            // conectare zona B
            // break if actual
  }else{
    statieCurenta = 3 // conectare la statia C (repaus)
    // logica de comanda pentru deplasare spre zona respectiva
    // verificare proximitate
    // conectare staie A
    // stop - verificare pentru prezentaObiect
  }

  // logica pentru pasul de deplasare spre zona B
  // verificare proximitate fata de zona B
      // decuplare obiect
      // comada in spate + rotatie 180 de grade


  // DEFALCHEAZA LOGICA DE MAI SUS INTR UN SWITCH DE STARI - ALEATORIU MASINA DE STARE
}
