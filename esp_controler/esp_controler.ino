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
  - FORWARD()           - deplasare inainte
  - BACK()              - deplasare inapoi
  - LEFT()              - rotire in stanga
  - RIGHT()             - rotire in dreapta
  - VERTICAL_LEFT()     - deplasare laterala stanga (roti magnum)
  - VERTICAL_RIGHT()    - deplasare laterala dreapta (roti magnum)
  - STOP()              - oprire completa a motoarelor

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
const char* ssid_statie[] = {"", "Statia_A", "Statia_B", "Statia_C"}; // indexul 0 este folosit atunci cand se doreste blocarea task ului taskAntena
const char* password_statie[] = {"", "12345678", "12345678", "12345678"};

// instante servo
Servo servoAntena; // pentru antena
Servo servoBase;   // pentru brat - servo de la baza    - miscare verticala
Servo servoHead;   // pentru brat - servo de la efector - miscare orizontala

// instanda UDP
WiFiUDP udpReceiver;
const unsigned int localPort = 4210; // trebuie sa fie acelasi ca pe ESP8266

// parametrii
#define PROX_RSSI_MAX -45

// variabile globale
// enum folosit pentru determinarea executiei comenzilor algoritmului comportamental
enum StareComportament{
  STARE_VERIFICARE_PREZENTA_OBIECT,
  STARE_ZONA_A,
  STARE_ZONA_B, 
  STARE_ZONA_C, 
  STARE_REPAUS
};
volatile int interval[2] = {LIM_INF , LIM_SUP};     // interval cu unghiurile pe care le va explora antena
volatile int conectareStatie = 0;                   // sttari posibilie {0,1,2,3} = {STOP, ZONA_A, ZONA_B, ZONA_C}
volatile int prezentaObiect_zonaA = 0;              // pentru stocarea prezentei obiectului din ZONA_A
volatile int unghiOrientare = 0;                  // unghiul de comanda al servo unde s-a depistat valoare maxima RSSI din intervalul de unghiuri in care s-au facut cautarile
volatile int valoareRSSI = 0;
volatile int obiect_detectat = 0;
volatile int obiect_x = 0;
volatile int obiect_y = 0;
volatile int obiect_w = 0; 
volatile int obiect_h = 0;
portMUX_TYPE muxUART = portMUX_INITIALIZER_UNLOCKED;    // mutex pentru blocarea accesului la o structura de cod - folosit pentru scriere/citire variabile globale
portMUX_TYPE muxRSSI = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxVarG = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxUNGHI = portMUX_INITIALIZER_UNLOCKED;
unsigned long ultimaAfisare = 0;  // variabila global de timp - afisare mesaje de debug

// variabile de activare pentru taskurile de deplasare
int task_urmarire = 0;
int task_pozitionare = 0;
int task_cautareStationara = 0;

StareComportament stareComport_Robot = STARE_VERIFICARE_PREZENTA_OBIECT; // instanta a structurii de date enum StareComportament

// handelere pentru controlul activarii taskurilor
TaskHandle_t handleTaskDeplasare_Urmarire = NULL;
TaskHandle_t handleTaskDeplasare_CautareStationara = NULL;
TaskHandle_t handleTaskDeplasare_PozitionareObiect = NULL; 
TaskHandle_t handleTaskControl_servoAntena = NULL;
TaskHandle_t handleTaskComportamentRobot = NULL;

void setup() {
  // setare UART 0 in cazul debugging ului
  Serial.begin(115200);
  // setare UART 1 pentru primirea pachetelor de date de la ESP CAM
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
  conectareStatie = 0; 
  stareComport_Robot = STARE_VERIFICARE_PREZENTA_OBIECT;

  // initializare tasks
  initTaskuri();

  // oprire task uri inutile
  delay(1000);
  vTaskSuspend(handleTaskDeplasare_Urmarire);
  vTaskSuspend(handleTaskDeplasare_CautareStationara);
  vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
  //vTaskSuspend(handleTaskControl_servoAntena);

  // stare finala
  digitalWrite(pinPWM, LOW); // setat in HIGH pentru tensiune continua de 3.3V, echivalent PWM 100 %

  // delay pentru asigurarea initializarilor si verificarilor
  delay(1000);
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
      Serial.println("taskControl_servoAntena - conectat la retea - activare handletaskComportamentRobot");
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
            prezentaObiect_zonaA = valoare;
            portEXIT_CRITICAL(&muxUART);

            Serial.print("DEBUG - taskControl_servoAntena - Mesaj UDP primit (zona A): ");
            Serial.println(prezentaObiect_zonaA);
          }
        }
      }
      // activare roti daca robotul este conectat la o statie
      digitalWrite(pinPWM, HIGH);

      // determinarea unghiului de orientare spre sursa de semnal
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
    //taskYIELD();
    vTaskDelay(1); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}

// Task 2: Control motoarelor pentru a duce unghiul de orientare spre zona tampon de mers inainte
void taskControl_Deplasare_Urmarire(void *parameter) {
  // control dupa unghiul de comanda al servomotorului anatenei
  while (true) {
    if (task_urmarire){
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
    }else
      digitalWrite(pinPWM, LOW)
    //taskYIELD();
    vTaskDelay(1); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}
void taskControl_Deplasare_CautareStationare(void *parameter) {
  // comenzi repetate de stanga - dreapta
  while (true) {
    if (task_cautareStationara){
    //digitalWrite(pinPWM, HIGH);
    // Serial.println("DEGUB - taskControl_Deplasare_CautareStationare - intrat in executie");
    // Serial.print("DEGUB - taskControl_Deplasare_CautareStationare - pin pwm: ");
    // Serial.println(digitalRead(pinPWM));
    motoare_rotireDreapta();
    delay(500);
    motoare_rotireStanga();
    delay(1000);
    motoare_rotireDreapta();
    delay(500);
    
    }else
      digitalWrite(pinPWM, LOW)
    //taskYIELD();
    vTaskDelay(1); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}
void taskControl_Deplasare_PozitionareObiect(void *parameter) {
  // - determinarea pozitiei de prindere
  // - activare brat_prindere()
  // - activare motoare_executieRetragere()
  // - schimbare stare - STARE_STATIA_B
  while (true) {
    if(task_pozitionare){
    //digitalWrite(pinPWM, HIGH);
    // Serial.println("DEGUB - taskControl_Deplasare_PozitionareObiect - intrat in executie");
    // Serial.print("DEGUB - taskControl_Deplasare_PozitionareObiect - pin pwm: ");
    // Serial.println(digitalRead(pinPWM));

    //taskYIELD();
    }else
      digitalWrite(pinPWM, LOW)
    vTaskDelay(1); // pentru a permite task ului sa cedeze prioritatea altui task
  }
}

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
    //taskYIELD();
    vTaskDelay(1);
  }
}

void taskComportamentRobot(void *parameter) {
  while (1) {

  //   if (eTaskGetState(handleTaskControl_servoAntena) == eSuspended)
  //     vTaskResume(handleTaskControl_servoAntena);

  portENTER_CRITICAL(&muxVarG);
  int conectareStare_local = conectareStatie;
  portEXIT_CRITICAL(&muxVarG);
  
  portENTER_CRITICAL(&muxRSSI);
  int valoareRSSI_local = valoareRSSI;
  portEXIT_CRITICAL(&muxRSSI);
  
  // static unsigned long ultimaAfisare = 0;
  // unsigned long timpCurent = millis();
  // if (timpCurent - ultimaAfisare >= 500) {
  //   ultimaAfisare = timpCurent;
  //   Serial.println("");
  //   Serial.println("TEST:");
  //   Serial.print("TEST - pin de pwm: ");
  //   Serial.println(digitalRead(pinPWM));
  //   Serial.print("TEST - statie conectata: ");
  //   Serial.println(WiFi.SSID());
  //   Serial.print("TEST - stare: ");
  //   Serial.println(stareComport_Robot);
  //   Serial.print("TEST - prezenta obiect: ");
  //   Serial.println(prezentaObiect_zonaA);
  //   Serial.print("TEST - valoare RSSI: ");
  //   Serial.println(valoareRSSI_local);
  // }

  Serial.println("");
  Serial.println("TEST:");
  Serial.print("TEST - pin de pwm: ");
  Serial.println(digitalRead(pinPWM));
  Serial.print("TEST - statie conectata: ");
  Serial.println(WiFi.SSID());
  Serial.print("TEST - stare: ");
  Serial.println(stareComport_Robot);
  Serial.print("TEST - prezenta obiect: ");
  Serial.println(prezentaObiect_zonaA);
  Serial.print("TEST - valoare RSSI: ");
  Serial.println(valoareRSSI_local);

  // activare task uri de deplasare
  task_cautareStationara = 1; 
  task_pozitionare = 1; 
  task_urmarire = 1; 

  switch (stareComport_Robot) {
    case STARE_VERIFICARE_PREZENTA_OBIECT:
      Serial.println("TEST - switch 1 - STARE VERIFICARE");
      conectareStare_local = 1;
      vTaskSuspend(handleTaskDeplasare_Urmarire);
      if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid_statie[conectareStare_local]) {
        vTaskDelay(3000);
        if (prezentaObiect_zonaA) {
          Serial.println("DEBUG - switch - prezenta obiect - salt stare A");
          stareComport_Robot = STARE_ZONA_A;
        } else {
          Serial.println("DEBUG - switch - absenta obiect  - salt stare C ");
          stareComport_Robot = STARE_ZONA_C;
        }
      } else {
        asteptareReconectare(conectareStare_local);
      }
      break;

    case STARE_ZONA_A:
      Serial.println("DEBUG - switch 2 - STARE A");
      conectareStare_local = 1;
      vTaskSuspend(handleTaskDeplasare_Urmarire);
      if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid_statie[conectareStare_local]) {
        if (obiect_detectat == 1) {
          Serial.println("DEBUG - switch 2 - OBIECT DETECTAT VIDEO - TASK POZITIONARE OBC");
          vTaskResume(handleTaskDeplasare_PozitionareObiect);
          vTaskSuspend(handleTaskDeplasare_Urmarire);
          vTaskSuspend(handleTaskDeplasare_CautareStationara);
        }
        if (valoareRSSI_local > PROX_RSSI_MAX) {
          Serial.println("DEBUG - switch 2 - IN ZONA PROXIMITATE - TASK CAUTARE STATIONARA");
          vTaskResume(handleTaskDeplasare_CautareStationara);
          vTaskSuspend(handleTaskDeplasare_Urmarire);
          vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
        } else {
          Serial.println("DEBUG - switch 2 - OUT ZONA PROXIMITATE - TASK URMARIRE");
          vTaskResume(handleTaskDeplasare_Urmarire);
          vTaskSuspend(handleTaskDeplasare_CautareStationara);
          vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
        }
      } else {
        vTaskSuspend(handleTaskDeplasare_Urmarire);
        vTaskSuspend(handleTaskDeplasare_CautareStationara);
        vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
        asteptareReconectare(conectareStare_local);
      }
      break;

    case STARE_ZONA_B:
      Serial.println("DEBUG - switch 3 - STARE B");
      conectareStare_local = 2;
      vTaskSuspend(handleTaskDeplasare_Urmarire);
      if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid_statie[conectareStare_local]) {
        if (valoareRSSI_local > PROX_RSSI_MAX) {
          Serial.println("DEBUG - switch 3 - IN ZONA PROXIMITATE - EXECUTARE ELIBERARE");
          brat_eliberare();
          motoare_executieRetragere();
          stareComport_Robot = STARE_VERIFICARE_PREZENTA_OBIECT;
        } else {
          Serial.println("DEBUG - switch 3 - OUT ZONA PROXIMITATE - TASK URMARIRE");
          vTaskResume(handleTaskDeplasare_Urmarire);
          vTaskSuspend(handleTaskDeplasare_CautareStationara);
          vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
        }
      } else {
        vTaskSuspend(handleTaskDeplasare_Urmarire);
        vTaskSuspend(handleTaskDeplasare_CautareStationara);
        vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
        asteptareReconectare(conectareStare_local);
      }
      break;

    case STARE_ZONA_C:
      Serial.println("DEBUG - switch 4 - STARE C");
      conectareStare_local = 3;
      vTaskSuspend(handleTaskDeplasare_Urmarire);
      if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid_statie[conectareStare_local]) {
        if (valoareRSSI_local > PROX_RSSI_MAX) {
          Serial.println("DEBUG - switch 4 - IN ZONA PROXIMITATE - EXECUTARE OPRIRE - VERIFICARE PREZENTA OBIECT");
          // vTaskSuspend(handleTaskDeplasare_Urmarire);
          // conectareStare_local = 1;
          // // schimbare retea conectata 
          // portENTER_CRITICAL(&muxVarG);
          // conectareStatie = conectareStare_local;
          // portEXIT_CRITICAL(&muxVarG); 
          // vTaskSuspend(handleTaskComportamentRobot);
          // delay(1000); // asteptare dupa verificare prezentei obiecutului din zona A
          // if (prezentaObiect_zonaA)
          //   stareComport_Robot = STARE_ZONA_A;
          // else 
          //   stareComport_Robot = STARE_ZONA_C;
          
          // e executa oprirea si se asteapta conectarea la o statia A
          digitalWrite(pinPWM, LOW);
          conectareStare_local = 1;
          portENTER_CRITICAL(&muxVarG);
          conectareStatie = conectareStare_local;
          portEXIT_CRITICAL(&muxVarG); 
          vTaskSuspend(handleTaskDeplasare_Urmarire);
          vTaskSuspend(handleTaskDeplasare_CautareStationara);
          vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
          vTaskSuspend(handleTaskComportamentRobot);
          // conectare realizata cand se reactiveaza task ul comportamental
          while(!prezentaObiect_zonaA){
            Serial.println("DEBUG - switch 4 - IN ZONA PROXIMITATE - EXECUTARE OPRIRE - ASTEPTARE DUPA prezentaObiect_zonaA.....");
            vTaskSuspend(handleTaskDeplasare_Urmarire);
            vTaskSuspend(handleTaskDeplasare_CautareStationara);
            vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
            vTaskDelay(100);
          }
          stareComport_Robot = STARE_ZONA_A;          
        } else {
          Serial.println("DEBUG - switch 4 - OUT ZONA PROXIMITATE - TASK URMARIRE");
          vTaskResume(handleTaskDeplasare_Urmarire);
          vTaskSuspend(handleTaskDeplasare_CautareStationara);
          vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
        }
      } else {
        vTaskSuspend(handleTaskDeplasare_Urmarire);
        vTaskSuspend(handleTaskDeplasare_CautareStationara);
        vTaskSuspend(handleTaskDeplasare_PozitionareObiect);
        asteptareReconectare(conectareStare_local);
      }
      break;

    case STARE_REPAUS:
      // inca nu am nevoie
      break;
    }
    //taskYIELD();
    vTaskDelay(1); // delay scurt pentru cooperativitate
  }
}

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

  xTaskCreatePinnedToCore(
    taskControl_Deplasare_Urmarire,
    "taskControl_Deplasare_Urmarire",
    4096,
    NULL,
    1,
    &handleTaskDeplasare_Urmarire,
    1               // core 1
  );

  xTaskCreatePinnedToCore(
    taskControl_Deplasare_CautareStationare,
    "taskControl_Deplasare_CautareStationare",
    4096,
    NULL,
    1,
    &handleTaskDeplasare_CautareStationara,
    1               // core 1
  );

  xTaskCreatePinnedToCore(
    taskControl_Deplasare_PozitionareObiect,
    "taskControl_Deplasare_PozitionareObiect",
    4096,
    NULL,
    1,
    &handleTaskDeplasare_PozitionareObiect,
    1               // core 1
  );

  xTaskCreatePinnedToCore(
    taskCitireUART,
    "taskCitireUART",
    4096,
    NULL,
    1,
    &handleTaskDeplasare_PozitionareObiect,
    1               // core 1
  );

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

void asteptareReconectare(int conectareStare_local){
  Serial.println("DEBUG - switch - retea incorecta - dezactivare handleTaskComportamentRobot");
  digitalWrite(pinPWM, LOW);
  
  portENTER_CRITICAL(&muxVarG);
  conectareStatie = conectareStare_local;
  portEXIT_CRITICAL(&muxVarG); 

  servoAntena.write(90);

  vTaskSuspend(handleTaskComportamentRobot);
  // taskYIELD();
  vTaskDelay(1);
}

void loop(){
  // ar trebui sa ramana gol
}
