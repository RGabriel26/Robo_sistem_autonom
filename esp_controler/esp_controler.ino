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

// date de conectare pentru statii
const char* ssid_statie[] = {"", "Statia_A", "Statia_B", "Statia_C"}; // indexul 0 este folosit atunci cand se doreste blocarea task ului taskAntena
const char* password_statie[] = {"", "12345678", "12345678", "12345678"};

// instante servo
Servo servoAntena; // pentru antena
Servo servoBase;   // pentru brat - servo de la baza    - miscare verticala
Servo servoHead;   // pentru brat - servo de la efector - miscare orizontala

// variabile globale
// enum folosit pentru determinarea executiei comenzilor algoritmului comportamental
enum StareComportament{
  STARE_VERIFICARE_PREZENTA_OBIECT,
  STARE_ZONA_A,
  STARE_ZONA_B, 
  STARE_ZONA_C, 
  STARE_REPAUS
};
volatile int conectareStatie = 0;                   // sttari posibilie {0,1,2,3} = {STOP, ZONA_A, ZONA_B, ZONA_C}
volatile int prezentaObiect = 0;                    // pentru stocarea prezentei obiectului din ZONA_A
volatile int valoareRSSI = 0;                       // valoare obtinuta din det_unghi_orientare si folosita pentru determinarea proximitatii fata de o ZONA
volatile int interval[2] = {LIM_INF , LIM_SUP};     // interval cu unghiurile pe care le va explora antena

StareComportament stareComport_Robot = STARE_VERIFICARE_PREZENTA_OBIECT; 

TaskHandle_t handleTaskDeplasare_Urmarire = NULL;
TaskHandle_t handleTaskDeplasare_CautareStationara = NULL;
TaskHandle_t handleTaskDeplasare_PozitionareObiect = NULL; 

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
  conectareStatie = 1; // conectare la statia A (pickup obiect)
  delay(1000);         // delay 1 sec pentru asteptarea verificarii prezentei obiectului

  // initializare tasks
  // TEST
  xTaskCreatePinnedToCore(
    taskControl_servoAntena,       // funcția
    "taskControl_servoAntena",     // nume task
    4096,             // stack size
    NULL,             // parametru
    1,                // prioritate
    NULL,             // handle (optional)
    0                 // core 0
  );

  xTaskCreatePinnedToCore(
    taskControl_Deplasare_Urmarire,
    "taskControl_Deplasare_Urmarire",
    4096,
    NULL,
    1,
    &handleTaskDeplasare_Urmarire,
    1               // rulează pe core 1
  );

  xTaskCreatePinnedToCore(
    taskControl_Deplasare_CautareStationare,
    "taskControl_Deplasare_CautareStationare",
    4096,
    NULL,
    2,
    &handleTaskDeplasare_CautareStationara,
    1               // rulează pe core 1
  );

  xTaskCreatePinnedToCore(
    taskControl_Deplasare_PozitionareObiect,
    "taskControl_Deplasare_PozitionareObiect",
    4096,
    NULL,
    2,
    &handleTaskDeplasare_PozitionareObiect,
    1               // rulează pe core 1
  );

  // stare finala
  digitalWrite(pinPWM, HIGH);
}

// Task 1: Verificare conexiune si orientarea antenei spre sursa de semnal wifi
void taskControl_servoAntena(void *parameter) {
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {                                          // daca nu este conectat, se incearca conectarea la statia curenta din pasul comportamental
      servoAntena.write(90);                                                      // pozitionare antena in pozitie default
      connect_statie(ssid_statie[conectareStatie], password_statie[conectareStatie]); // conectare la statia curenta
    } else {
      // verificare prezentaObiect

      // determinarea unghiului de orientare spre sursa de semnal
      valoareRSSI = det_unghi_orientare();
      Serial.println(det_unghi_orientare()); // FOLOSIT PENTRU TEST
    }
    vTaskDelay(0); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}

// Task 2: Control motoarelor pentru a duce unghiul de orientare spre zona tampon de mers inainte
void taskControl_Deplasare_Urmarire(void *parameter) {
  while (true) {

    vTaskDelay(0); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}
void taskControl_Deplasare_CautareStationare(void *parameter) {
  while (true) {

    vTaskDelay(0); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}
void taskControl_Deplasare_PozitionareObiect(void *parameter) {
  while (true) {

    vTaskDelay(0); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}

void loop() {
  // ROBOTUL IN TIMPUL INITIALIZARII, INCEARCA CONECTAREA LA ZONA A
  // LA PRIMA EXECUTIEI A FUNCTIEI LOOP AR TREBUI CA VARIABILA prezentaObiect SA FIE DEJA POPULATA SAU NU
  // VERIFICARE_OBIECT
      // DA
        // CAUTARE_ZONA_A
      // NU
        // CAUTARE_ZONA_C
  // CAUTARE_ZONA_A
      // - conectare STATIE_A
      // - verificare proximitate
      // - detectie
      // - prindere obiect
      // - rotire
      // - CAUTARE_ZONA_B
  // CAUTARE_ZONA_B
      // - conectare STATIE_B
      // - verificare proximitate
      // - eliberare obiect
      // - rotire
      // - VERIFICARE_OBIECT
  // CAUTARE_ZONA_C
      // - conectare STATIE_C
      // - verificare proximitate
      // - veridicare prezenta obiect din zona A
        // REPAUS
          // - conectare la STATIE_A
          // - asteptare prezentaObiect = true
          // CAUTARE_ZONA_A

  switch (stareComport_Robot) {
    // -----------------------------------------------------------------------------
    case STARE_VERIFICARE_PREZENTA_OBIECT: 
      if(prezentaObiect)
        stareComport_Robot = STARE_ZONA_A;
      else
        stareComport_Robot = STARE_ZONA_C;
      break;
    // -----------------------------------------------------------------------------
    case STARE_ZONA_A:
        conectareStatie = 1; // conectare STATIE_A
        // verificare proximitate sau detectie obiect
        if (valoareRSSI < 35 ){   // ajuns in zona de proximitate a statiei A
            // daca se ajunge aici inseamna ca robotul nu a reusit sa detecteze obiectul
            // se executa miscari stanga dreapta in speranta detectiei obiectului
              // oprire taskControl_DeplasareUrmarire pentru a se activa taskControl_DeplasareCautareStationare
            vTaskSuspend(handleTaskDeplasare_Urmarire);
            vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
            vTaskResume(handleTaskDeplasare_CautareStationara);
        }else{
          // mentinere taskControl_DeplasareUrmarire
            vTaskSuspend(handleTaskDeplasare_CautareStationara);
            vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
            vTaskResume(handleTaskDeplasare_Urmarire);
        }

        


        conectareStatie = 2; // conectare STATIE_B
      break;
    // -----------------------------------------------------------------------------
    case STARE_ZONA_B: 

      break;
    // -----------------------------------------------------------------------------
    case STARE_ZONA_C: 

      break;
    // -----------------------------------------------------------------------------
    case STARE_REPAUS:

      break;
  }
    
}
