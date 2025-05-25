
#include "motoare.h" // functii destinate deplasarii 
#include "antena_control.h" // funtii destinate orientarii antenei
#include "brat_control.h" // functii destinate controlului bratului apucator
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Arduino.h>
#include <WiFiUdp.h>

/*
================================================================================
 FUNCTII DISPONIBILE IN FISIERELE HEADER IMPORTATE
================================================================================

motoare.h:
  - motoare_deplasareFata()           - deplasare inainte
  - motoare_deplasareSpate()          - deplasare inapoi
  - motoare_rotireStanga()            - rotire in stanga
  - motoare_rotireDreapta()           - rotire in dreapta
  - motoare_verticalStanga()          - deplasare laterala stanga (roti magnum)
  - motoare_verticalDreapta()         - deplasare laterala dreapta (roti magnum)
  - motoare_stop()                    - oprire completa a motoarelor
  - motoare_executieRetragere()       - pentru executia manevrei de retragere din statie dupa prindere/eliberare obiect

antena_control.h:
  - connect_statie(const char* ssid, const char* password)          - conectare la o retea Wi-Fi specificata
  - det_unghi_orientare()                                           - returneaza unghiul cu cel mai bun semnal RSSI detectat

brat_control.h:
  - init_brat()         - pozitionare initiala a servomotoarelor bratului apucator
  - brat_prindere()     - secventa de prindere: capul apuca, bratul se ridica
  - brat_eliberare()    - secventa de eliberare: bratul coboara, capul elibereaza

================================================================================
*/

// date de conectare pentru statii
const char* ssid_statie[] = {"", "Statia_A", "Statia_B", "Statia_C"};     // indexul 0 este folosit atunci cand se doreste blocarea task ului taskAntena
const char* password_statie[] = {"", "12345678", "12345678", "12345678"}; 

// instante servo
Servo servoAntena; // pentru antena
Servo servoBase;   // pentru brat - servo de la baza    - miscare verticala
Servo servoHead;   // pentru brat - servo de la efector - miscare orizontala

// instanda UDP
WiFiUDP udpReceiver;
const unsigned int localPort = 4210; // port pentru comunicarea prin protocolul UDP cu statia A
 
// parametrii
#define PROX_RSSI_MAX -45            // parametru de prag pentru determinarea zonei de proximitate fata de o statie

// variabile globale
enum StareComportament{             // enum folosit pentru determinarea executiei comenzilor algoritmului comportamental
  STARE_VERIFICARE_PREZENTA_OBIECT,
  STARE_ZONA_A,
  STARE_ZONA_B, 
  STARE_ZONA_C, 
  STARE_REPAUS
};
volatile int interval[2] = {LIM_INF , LIM_SUP};     // interval cu unghiurile pe care le va explora antena
volatile int conectareStatie = 0;                   // sttari posibilie {0,1,2,3} = {STOP, ZONA_A, ZONA_B, ZONA_C}
volatile int obiect_prezent_zonaA = 0;              // pentru stocarea prezentei obiectului din ZONA_A
volatile int unghiOrientare = 0;                    // unghiul de comanda al servo unde s-a depistat valoare maxima RSSI din intervalul de unghiuri in care s-au facut cautarile
volatile int valoareRSSI = 0;                       // valoarea rssi masurata corespunzatoare unghiului de orientare
volatile int obiect_detectat = 0;                   // inregistreaza prezenta obiectului din zona A
volatile int obiect_x = 0;                          // coordonate x al obiectului detectat
volatile int obiect_y = 0;                          // coordonate y al obiectului detectat
volatile int obiect_w = 0;                          // coordonate w al obiectului detectat
volatile int obiect_h = 0;                          // coordonate h al obiectului detectat
portMUX_TYPE muxUART = portMUX_INITIALIZER_UNLOCKED;    // mutex acces variabile prin uart
portMUX_TYPE muxRSSI = portMUX_INITIALIZER_UNLOCKED;    // mutex acces variabila RSSI
portMUX_TYPE muxVarG = portMUX_INITIALIZER_UNLOCKED;    // mutex acces variabile globale
portMUX_TYPE muxUNGHI = portMUX_INITIALIZER_UNLOCKED;   // mutex acces variabila unghi de orientare al antenei
unsigned long ultimaAfisare = 0;                    // variabila global de timp - afisare mesaje de debug - in conditii de test

// instanta a structurii de date enum StareComportament
StareComportament stareComport_Robot = STARE_VERIFICARE_PREZENTA_OBIECT; 

// handelere pentru controlul activarii taskurilor
TaskHandle_t handleTaskDeplasare_Urmarire = NULL;
TaskHandle_t handleTaskDeplasare_CautareStationara = NULL;
TaskHandle_t handleTaskDeplasare_PozitionareObiect = NULL; 
TaskHandle_t handleTaskControl_servoAntena = NULL;
TaskHandle_t handleTaskComportamentRobot = NULL;
TaskHandle_t handleTaskCitireUART = NULL;

void setup() {
  // setare UART 0 in cazul debugging ului
  Serial.begin(115200);
  // setare UART 1 pentru primirea pachetelor de date de la ESP CAM
  Serial1.begin(9600, SERIAL_8N1, 35, -1);
  // initializare pini control motoare / drivere
  setup_pini();
  // initializare brat
  init_brat();
  // initializarea conectiune cu o statie care nu exista
  conectareStatie = 0; 
  // initializare tasks
  // stare finala
  initTaskuri();
  delay(1000);
  digitalWrite(pinPWM, LOW); // setat in HIGH pentru tensiune continua de 3.3V, echivalent PWM 100 %
}
// Task 1: Verificare conexiune si orientarea antenei spre sursa de semnal wifi
void taskControl_servoAntena(void *parameter) {
  while (true) {
    portENTER_CRITICAL(&muxVarG);
    int varLocal = conectareStatie;
    portEXIT_CRITICAL(&muxVarG);

    if(varLocal == 0) {
      vTaskDelay(50);
      continue;
    }
    if (WiFi.status() != WL_CONNECTED || WiFi.SSID() != ssid_statie[varLocal]) { // daca nu este conectat, se incearca conectarea la statia curenta din pasul comportamental
      servoAntena.write(90);                                                     // pozitionare antena in pozitie default
      connect_statie(ssid_statie[varLocal], password_statie[varLocal]);          // conectare la statia curenta
      Serial.println("DEBUG - taskControl_servoAntena - conectat la retea + activare handleTaskComportamentRobot");
      vTaskResume(handleTaskComportamentRobot);
    } else {
      // posibila necesitatea unei verificari suplimentare statiei actual conectate
      if(WiFi.SSID() == ssid_statie[1]){ // verificare conectiune statia A
        int packetSize = udpReceiver.parsePacket();
        if (packetSize) {
          char incomingPacket[255];
          int len = udpReceiver.read(incomingPacket, 255);
          if (len > 0) {
            incomingPacket[len] = '\0';         // terminator de string
            int valoare = atoi(incomingPacket); // conversie la int

            portENTER_CRITICAL(&muxUART);
            obiect_prezent_zonaA = valoare;
            portEXIT_CRITICAL(&muxUART);

            Serial.print("DEBUG - taskControl_servoAntena - Mesaj UDP primit (zona A): ");
            Serial.println(valoare);
          }
        }
      }
      // activare roti daca robotul este conectat la o statie
      digitalWrite(pinPWM, HIGH);
    
      // determinarea unghiului de orientare spre sursa de semnal si obtinerea valorii RSSI
      int val_RSSI_temp = 0;
      int val_unghi_temp = det_unghi_orientare(val_RSSI_temp);

      // salvare valoare rssi determinata in variabila globala
      portENTER_CRITICAL(&muxRSSI);
      valoareRSSI = val_RSSI_temp;
      portEXIT_CRITICAL(&muxRSSI); 
      //salvare valoare unghi determinata in variabila globala
      portENTER_CRITICAL(&muxUNGHI);
      unghiOrientare = val_unghi_temp;
      portEXIT_CRITICAL(&muxUNGHI);

      Serial.print("DEBUG - taskControl_servoAntena - unghi: ");
      Serial.println(val_unghi_temp); // FOLOSIT PENTRU TEST
    }
    vTaskDelay(1); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}
// Task 2: Control motoarelor pentru a duce unghiul de orientare spre zona tampon de mers inainte
void taskControl_Deplasare_Urmarire(void *parameter) {
  // control dupa unghiul de comanda al servomotorului anatenei
  while (true) {
    Serial.println("\n=== DEBUG - taskControl_Deplasare_Urmarire ===");
    //digitalWrite(pinPWM, HIGH);
    // Serial.println("DEGUB - taskControl_Deplasare_Urmarire - intrat in executie");
    // Serial.print("DEGUB - taskControl_Deplasare_Urmarire - pin pwm: ");
    // Serial.println(digitalRead(pinPWM));

    // protectie la citire
    portENTER_CRITICAL(&muxUNGHI);
    int unghiProvenienta_local = unghiOrientare;
    portEXIT_CRITICAL(&muxUNGHI);

    if (unghiProvenienta_local >= 80 && unghiProvenienta_local <= 100) {
      motoare_deplasareFata();
    } else if (unghiProvenienta_local < 80) {
      motoare_rotireStanga();
    } else if (unghiProvenienta_local > 100) {
      motoare_rotireDreapta();
    }
    vTaskDelay(1); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}
// Task 3: Control motoarelor pentru cautare stationara
void taskControl_Deplasare_CautareStationare(void *parameter) {
  // comenzi repetate de stanga - dreapta
  while (true) {
    Serial.println("\n=== DEBUG - taskControl_Deplasare_CautareStationare ===");
    //digitalWrite(pinPWM, HIGH);
    // Serial.println("DEGUB - taskControl_Deplasare_CautareStationare - intrat in executie");
    // Serial.print("DEGUB - taskControl_Deplasare_CautareStationare - pin pwm: ");
    // Serial.println(digitalRead(pinPWM));
    motoare_rotireDreapta();
    vTaskDelay(500);
    motoare_rotireStanga();
    vTaskDelay(1000);
    motoare_rotireDreapta();
    vTaskDelay(500);
    
    vTaskDelay(1); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}
// Task 4: Control motoarelor pentru pozitionarea in dreptul obiectului
void taskControl_Deplasare_PozitionareObiect(void *parameter) {
  // - determinarea pozitiei de prindere
  // - activare brat_prindere()
  // - activare motoare_executieRetragere()
  // - schimbare stare - STARE_STATIA_B
  while (true) {
      Serial.println("\n=== DEBUG - taskControl_Deplasare_PozitionareObiect ===");
    // digitalWrite(pinPWM, HIGH);
    // Serial.println("DEGUB - taskControl_Deplasare_PozitionareObiect - intrat in executie");
    // Serial.print("DEGUB - taskControl_Deplasare_PozitionareObiect - pin pwm: ");
    // Serial.println(digitalRead(pinPWM));

    vTaskDelay(1); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}
// Task 5: Citire date de la ESP CAM prin UART
void taskCitireUART(void *parameter){
  // - decodificare mesaj uart + salvare
  while (true) {
    String buffer = "";
    while (Serial1.available()) {
      char c = Serial1.read();
      buffer += c;

      if (buffer.endsWith("<END>")) {
        int x, y, w, h;
        int start = buffer.indexOf("<START>") + 7;
        int end = buffer.indexOf("<END>");
        String continut = buffer.substring(start, end);

        if (continut == "NO_OBJ") {
          portENTER_CRITICAL(&muxUART);
          obiect_detectat = 0;
          portEXIT_CRITICAL(&muxUART);
        } else {
          int index1 = continut.indexOf(",");
          int index2 = continut.indexOf(",", index1 + 1);
          int index3 = continut.indexOf(",", index2 + 1);
          x = continut.substring(0, index1).toInt();
          y = continut.substring(index1 + 1, index2).toInt();
          w = continut.substring(index2 + 1, index3).toInt();
          h = continut.substring(index3 + 1).toInt();

          portENTER_CRITICAL(&muxUART);
          obiect_detectat = 1;
          obiect_x = x;
          obiect_y = y;
          obiect_w = w;
          obiect_h = h;
          portEXIT_CRITICAL(&muxUART);
        }

        buffer = "";
      }
    }
    vTaskDelay(1);
  }
}
// Task 6: Comportamentul robotului - logica de decizie
void taskComportamentRobot(void *parameter) {
  while (true) {
    vTaskResume(handleTaskControl_servoAntena);
    vTaskResume(handleTaskCitireUART);

    // actualizare variabile locale sincronizate
    portENTER_CRITICAL(&muxVarG);
    int statieCurenta = conectareStatie;
    portEXIT_CRITICAL(&muxVarG);

    portENTER_CRITICAL(&muxRSSI);
    int rssi_local = valoareRSSI;
    portEXIT_CRITICAL(&muxRSSI);

    portENTER_CRITICAL(&muxUART);
    int obiect_prezent_local  = obiect_prezent_zonaA;
    int obiect_detectat_local = obiect_detectat;
    portEXIT_CRITICAL(&muxUART);

    // debug scurt
    Serial.println("\n=== DEBUG COMPORTAMENT ===");
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    Serial.print("STARE: "); Serial.println(stareComport_Robot);
    Serial.print("RSSI: "); Serial.println(rssi_local);
    Serial.print("PREZENTA: "); Serial.println(obiect_prezent_local);
    Serial.print("OBIECT DETECTAT: "); Serial.println(obiect_detectat_local);

    // executie comportament pe baza starii
    switch (stareComport_Robot) {
      case STARE_VERIFICARE_PREZENTA_OBIECT:
        handleStare_Verificare(obiect_prezent_local);
        break;

      case STARE_ZONA_A:
        handleStareZona_A(rssi_local, obiect_detectat_local);
        break;

      case STARE_ZONA_B:
        handleStareZona_B(rssi_local);
        break;

      case STARE_ZONA_C:
        handleStareZona_C(rssi_local, obiect_prezent_local);
        break;

      case STARE_REPAUS:
        break;
    }

    vTaskDelay(500);
  }
}

void handleStare_Verificare(int obiect_prezent) {
  Serial.println("DEBUG - STARE_VERIFICARE");

  conectareStatie = 1;

  if (isConnectedToStation(1)) {
    vTaskDelay(3000);
    if (obiect_prezent) {
      Serial.println("DEBUG - STARE_VERIFICARE - Obiect prezent. Trecere la STARE_ZONA_A.");
      stareComport_Robot = STARE_ZONA_A;
    } else {
      Serial.println("DEBUG - STARE_VERIFICARE - Obiect absent. Trecere la STARE_ZONA_C.");
      stareComport_Robot = STARE_ZONA_C;
    }
  } else {
    asteptareReconectare(1);
    }
}
void handleStareZona_A(int RSSI, int obiect_detectat) {
  Serial.println("DEBUG - STARE_ZONA_A");

  conectareStatie = 1;
  
  if (isConnectedToStation(1)) {
    if (obiect_detectat == 1) {
      Serial.println("DEBUG - STARE_ZONA_A - Obiect detectat vizual. - Activare POZITIONARE.");
      activeazaTaskuri_Deplasare(false, false, true);
    } else if (RSSI > PROX_RSSI_MAX) {
      Serial.println("DEBUG - STARE_ZONA_A - RSSI puternic.          - Activare STATIONARA.");
      activeazaTaskuri_Deplasare(false, true, false);
    } else {
      Serial.println("DEBUG - STARE_ZONA_A - RSSI slab.              - Activare URMARIRE.");
      activeazaTaskuri_Deplasare(true, false, false);
    }
  } else {
    activeazaTaskuri_Deplasare(false, false, false);
    asteptareReconectare(1);
  }
}
void handleStareZona_B(int RSSI) {
  Serial.println("DEBUG - STARE_ZONA_B");

  conectareStatie = 2;

  if (isConnectedToStation(2)) {
    if (RSSI > PROX_RSSI_MAX) {
      Serial.println("DEBUG - STARE_ZONA_B - Executare eliberare.");
      brat_eliberare();
      motoare_executieRetragere();
      digitalWrite(pinPWM, LOW);
      stareComport_Robot = STARE_VERIFICARE_PREZENTA_OBIECT;
    } else {
      Serial.println("DEBUG - STARE_ZONA_B - RSSI slab.              - Activare URMARIRE.");
      activeazaTaskuri_Deplasare(true, false, false);
    }
  } else {
    activeazaTaskuri_Deplasare(false, false, false);
    asteptareReconectare(2);
  }
}
void handleStareZona_C(int RSSI, int obiect_prezent) {
  Serial.println("DEBUG - STARE_ZONA_C");

  conectareStatie = 3;

  if (isConnectedToStation(3)) {
    if (RSSI > PROX_RSSI_MAX) {
      Serial.println("DEBUG - STARE_ZONA_C - PROXIMITATE - Oprire si asteptare prezenta obiect.");
      // verificare prezenta obiect zona A
      // stabilire conectare statia A
      conectareStatie = 1;
      activeazaTaskuri_Deplasare(false, false, false);
      asteptareReconectare(1);
      if (isConnectedToStation(1)) {
        int prezenta = obiect_prezent;
        while (!prezenta) {
          Serial.println("DEBUG - STARE_ZONA_C - PROXIMITATE -  Asteptare prezenta obiect in zona A...");
          portENTER_CRITICAL(&muxUART);
          prezenta = obiect_prezent_zonaA;
          portEXIT_CRITICAL(&muxUART);
          vTaskDelay(1000);
        }
      }else{
        Serial.println("DEBUG - STARE_ZONA_C - CONECTARE ZONA_A - ESUATA");
      }
      Serial.println("DEBUG - STARE_ZONA_C - Obiect prezent in zona A. Trecere la STARE_ZONA_A.");
      stareComport_Robot = STARE_ZONA_A;
      activeazaTaskuri_Deplasare(false, false, false);
    } else {
      Serial.println("DEBUG - STARE_ZONA_C - RSSI slab.              - Activare URMARIRE.");
      activeazaTaskuri_Deplasare(true, false, false);
    }
  } else {
    activeazaTaskuri_Deplasare(false, false, false);
    asteptareReconectare(3);
  }
}

void activeazaTaskuri_Deplasare(bool urmarire, bool cautare, bool pozitionare) {
  if (urmarire) vTaskResume(handleTaskDeplasare_Urmarire);
  else vTaskSuspend(handleTaskDeplasare_Urmarire);

  if (cautare) vTaskResume(handleTaskDeplasare_CautareStationara);
  else vTaskSuspend(handleTaskDeplasare_CautareStationara);

  if (pozitionare) vTaskResume(handleTaskDeplasare_PozitionareObiect);
  else vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
}
bool isConnectedToStation(int stationIndex) {
  return WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid_statie[stationIndex];
}
void asteptareReconectare(int conectareStare_local){
  Serial.println("DEBUG - asteptareReconectare");
  digitalWrite(pinPWM, LOW);
  
  portENTER_CRITICAL(&muxVarG);
  conectareStatie = conectareStare_local;
  portEXIT_CRITICAL(&muxVarG); 

  servoAntena.write(90);

  vTaskSuspend(handleTaskComportamentRobot);
  vTaskDelay(1);
}
// FUNCTII SETUP
void setup_pini() {
  // setare pini pentru motoare
  pinMode(DRIVER_A_IN1, OUTPUT);
  pinMode(DRIVER_A_IN2, OUTPUT);
  pinMode(DRIVER_A_IN3, OUTPUT);
  pinMode(DRIVER_A_IN4, OUTPUT);
  pinMode(DRIVER_B_IN1, OUTPUT);
  pinMode(DRIVER_B_IN2, OUTPUT);
  pinMode(DRIVER_B_IN3, OUTPUT);
  pinMode(DRIVER_B_IN4, OUTPUT);
  pinMode(pinPWM, OUTPUT);

  // setare pini pentru servo
  pinMode(pinServoAntena, OUTPUT);
  servoAntena.attach(pinServoAntena); 

  // setare pini pentru brat
  pinMode(pinServoBase, OUTPUT);
  pinMode(pinServoHead, OUTPUT);
  servoBase.attach(pinServoBase);
  servoHead.attach(pinServoHead);

  delay(100);
  servoAntena.write(90);
}
// initializare taskuri
void initTaskuri(){
    xTaskCreatePinnedToCore(
    taskControl_servoAntena,       // functia
    "taskControl_servoAntena",     // nume task
    4096,             // stack size
    NULL,             // parametru
    1,                // prioritate
    &handleTaskControl_servoAntena,             // handle (optional)
    0                 // core 0
  );
  vTaskSuspend(handleTaskControl_servoAntena); // suspendare initiala a taskului de control al antenei

  xTaskCreatePinnedToCore(
    taskControl_Deplasare_Urmarire,
    "taskControl_Deplasare_Urmarire",
    4096,
    NULL,
    1,
    &handleTaskDeplasare_Urmarire,
    1               // core 1
  );
  vTaskSuspend(handleTaskDeplasare_Urmarire); // suspendare initiala a taskului de deplasare urmarire

  xTaskCreatePinnedToCore(
    taskControl_Deplasare_CautareStationare,
    "taskControl_Deplasare_CautareStationare",
    4096,
    NULL,
    1,
    &handleTaskDeplasare_CautareStationara,
    1               // core 1
  );
  vTaskSuspend(handleTaskDeplasare_CautareStationara); // suspendare initiala a taskului de deplasare cautare stationara

  xTaskCreatePinnedToCore(
    taskControl_Deplasare_PozitionareObiect,
    "taskControl_Deplasare_PozitionareObiect",
    4096,
    NULL,
    1,
    &handleTaskDeplasare_PozitionareObiect,
    1               // core 1
  );
  vTaskSuspend(handleTaskDeplasare_PozitionareObiect); // suspendare initiala a taskului de deplasare pozitionare obiect

  xTaskCreatePinnedToCore(
    taskCitireUART,
    "taskCitireUART",
    4096,
    NULL,
    1,
    &handleTaskCitireUART,
    1               // core 1
  );
  vTaskSuspend(handleTaskCitireUART); // suspendare initiala a taskului de citire UART

  xTaskCreatePinnedToCore(
    taskComportamentRobot,
    "taskComportamentRobot",
    4096,
    NULL,
    1,
    &handleTaskComportamentRobot,
    1               // core 1
  );
}

void loop(){
  // loop continuu
}
